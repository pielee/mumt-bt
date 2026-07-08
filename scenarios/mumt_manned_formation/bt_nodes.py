"""
MUM-T 유인기 편대 — 계층형 유도(FormationGuidance) → UE PID 내루프.

formation_guidance.py(JSBSim F-16 3기로 검증, 선회 중 슬롯오차 ~5m) 포팅.
리더 상태(5006) → (heading, alt, speed, roll_ff) 명령을 산출해 AircraftSetpoint로
전송하면, UE의 PID 내루프가 60Hz로 추종한다. GetStick(조준) 방식을 대체.

핵심 설계:
  · 슬롯 = 리더 트랙 프레임에서 (forward, right, up) 오프셋
  · 슬롯 속도 피드포워드 v_slot = v_leader + ω×r (선회 시 바깥 윙맨 가속/안쪽 감속)
  · along-track → speed_cmd = |v_slot| + PD(e_along)  (앞뒤 간격)
  · cross-track → heading_cmd (pure pursuit lookahead, 적응형 L)
  · vertical → alt_cmd = 슬롯고도 + 상승률 선행보상
  · roll_ff = atan(ω·V/g)·scale — 리더 뱅크를 팔로워가 '동시에' 잡는 피드포워드

좌표: 5006은 UE 월드 cm(x=East, y=South, z=Up). 유도는 NED(m)로 변환해 계산:
  north = -y/100, east = x/100, alt = z/100. 리더 track = 리더 yaw(나침반 0=북).
  heading_cmd = atan2(Δeast, Δnorth) 나침반 → UE 내루프 psi(0=북)와 정합.
"""

import math

from std_msgs.msg import String
from custom_msgs.msg import AircraftSetpoint

from modules.base_bt_nodes import (
    BTNodeList, Status, Node, Sequence, ReactiveSequence, Fallback, ReactiveFallback,
)
from modules.base_bt_nodes_ros import ConditionWithROSTopics, ActionWithROSTopic

# 기존 노드/헬퍼 재사용 (import 부수효과로 GatherState 등록)
from scenarios.mumt.bt_nodes import (
    GatherState, clamp, heading_to_unit_xy, _alt_m, _own_routing_name,
    CM_TO_M, _STATE_TOPIC, _SETPOINT_TOPIC,
)

BTNodeList.ACTION_NODES.extend(["FormationGuidance"])


def _delta_heading(target, current):
    """최단 회전각 [-180,180]."""
    return ((target - current + 180.0) % 360.0) - 180.0


class FormationGuidance(ActionWithROSTopic):
    """자체 이륙 → 유인기 슬롯 편대 유도 (heading/alt/speed/roll_ff 명령).

    TAKEOFF : 스폰 전방+상방 목표로 헤딩/고도/속도 명령 → 내루프가 이륙.
              스폰 대비 takeoff_climb_m 상승 시 FORMATION 전환.
    FORMATION: formation_guidance.py 유도. 항상 RUNNING. 결손 시 마지막 명령 래칭.

    슬롯 오프셋(리더 트랙 프레임): front_m(+앞/−뒤), right_m(+우/−좌), up_m(+위).
    기본 (−80, +100, 0) = 리더 뒤 80m·우측 100m (검증된 윙맨 위치)."""

    KP_ALONG   = 0.30
    KD_ALONG   = 0.55
    V_CORR_MAX = 45.0
    L_AHEAD_MIN = 250.0
    L_AHEAD_T   = 2.0

    def __init__(self, name, agent, own_name="", leader_name="M_F16",
                 front_m=-80.0, right_m=100.0, up_m=0.0,
                 min_speed_mps=70.0, max_speed_mps=335.0,
                 runway_heading_deg=90.0, takeoff_forward_m=3000.0, takeoff_up_m=800.0,
                 takeoff_speed_mps=220.0, takeoff_climb_m=150.0, min_agl_m=150.0,
                 bt_rate_hz=10.0):
        super().__init__(name, agent, (AircraftSetpoint, _SETPOINT_TOPIC))
        self._own_name = own_name or (getattr(agent, "agent_id", "") or "").strip("/")
        self._leader   = leader_name
        self._front, self._right, self._up = float(front_m), float(right_m), float(up_m)
        self._spd_min, self._spd_max = float(min_speed_mps), float(max_speed_mps)
        self._rwy_hdg  = float(runway_heading_deg)
        self._to_fwd   = float(takeoff_forward_m)
        self._to_up    = float(takeoff_up_m)
        self._to_spd   = float(takeoff_speed_mps)
        self._to_climb = float(takeoff_climb_m)
        self._min_agl  = float(min_agl_m)
        self._dt       = 1.0 / float(bt_rate_hz)
        self._spawn    = None            # (n, e, alt) NED m — own 최초관측 래칭
        self._airborne = False
        self._prev_track = None          # 리더 선회율 미분용
        self._turn_rate_f = 0.0
        self._prev_lalt = None           # 리더 상승률 미분용
        self._last_msg = None

    @staticmethod
    def _ned(ac):
        """UE cm(x=E,y=S,z=U) → NED m: (north=-y/100, east=x/100, alt=z/100)."""
        return (-ac.get("y", 0.0) * CM_TO_M, ac.get("x", 0.0) * CM_TO_M, ac.get("z", 0.0) * CM_TO_M)

    def _emit(self, agent, own, heading, alt, speed, roll_ff):
        alt = max(alt, self._spawn[2] + self._min_agl)      # 고도 하한 가드
        msg = AircraftSetpoint()
        msg.aircraft_name    = _own_routing_name(own, self._own_name)
        msg.heading_deg      = float(heading % 360.0)
        msg.altitude_m       = float(alt)
        msg.target_speed_mps = float(clamp(speed, self._spd_min, self._spd_max))
        msg.roll_ff_deg      = float(roll_ff)
        self._last_msg = msg
        return msg

    def _build_message(self, agent, blackboard):
        own    = blackboard.get("own_state")
        leader = blackboard.get("leader_state")
        if not own or not leader:
            return self._last_msg        # 결손 → 래칭 (GatherState가 선행 차단)

        on, oe, oalt = self._ned(own)
        if self._spawn is None:
            self._spawn = (on, oe, oalt)

        # ── 페이즈 전환 ──
        climb_m = oalt - self._spawn[2]
        if not self._airborne and climb_m >= self._to_climb:
            self._airborne = True
            agent.ros_bridge.node.get_logger().info(
                f"[FormationGuidance] 이륙 완료(+{climb_m:.0f}m) → 편대 유도 전환")

        if not self._airborne:
            # ── TAKEOFF: 스폰 전방+상방으로 heading/alt/speed 명령 (내루프가 이륙) ──
            heading = self._rwy_hdg
            alt     = self._spawn[2] + self._to_up
            agent.ros_bridge.node.get_logger().info(
                f"[FormationGuidance] 이륙 상승 +{climb_m:.0f}/{self._to_climb:.0f}m")
            return self._emit(agent, own, heading, alt, self._to_spd, 0.0)

        # ── FORMATION: formation_guidance.py 유도 ──
        ln, le, lalt = self._ned(leader)
        track = leader.get("yaw", 0.0)               # 리더 track(=nose, 나침반 0=북)
        gs    = leader.get("speed_mps", 0.0)

        # 리더 선회율·상승률 미분(+저역필터)
        if self._prev_track is None:
            tr_raw = 0.0
        else:
            tr_raw = clamp(_delta_heading(track, self._prev_track) / self._dt, -10.0, 10.0)
        self._turn_rate_f = 0.95 * self._turn_rate_f + 0.05 * tr_raw
        self._prev_track = track
        turn_rate = self._turn_rate_f
        climb = 0.0 if self._prev_lalt is None else (lalt - self._prev_lalt) / self._dt
        self._prev_lalt = lalt

        tr = math.radians(track)
        c, s = math.cos(tr), math.sin(tr)
        # 슬롯 NED: forward=(cosψ,sinψ), right=(-sinψ,cosψ)
        dn = self._front * c + self._right * (-s)
        de = self._front * s + self._right * c
        slot_n, slot_e, slot_alt = ln + dn, le + de, lalt + self._up

        # 슬롯 속도 v_slot = v_leader + ω×r
        vn, ve = gs * c, gs * s
        rn = self._front * c - self._right * s
        re = self._front * s + self._right * c
        w = math.radians(turn_rate)
        vn += -w * re
        ve += w * rn
        v_slot_mag = math.hypot(vn, ve)

        # own 속도 근사 (5006엔 vn/ve 없음 → yaw+speed로)
        oy = math.radians(own.get("yaw", 0.0))
        ospd = own.get("speed_mps", 0.0)
        ovn, ove = ospd * math.cos(oy), ospd * math.sin(oy)

        # 오차 분해 (리더 트랙 프레임)
        en, ee = slot_n - on, slot_e - oe
        e_along = en * c + ee * s                     # +: 슬롯이 내 앞 → 가속
        e_cross = -en * s + ee * c                    # +: 슬롯이 리더 우측

        # along → speed (피드포워드 + PD)
        de_along = (vn - ovn) * c + (ve - ove) * s
        v_corr = clamp(self.KP_ALONG * e_along + self.KD_ALONG * de_along,
                       -self.V_CORR_MAX, self.V_CORR_MAX)
        speed_cmd = v_slot_mag + v_corr

        # cross → heading (pure pursuit, 적응형 lookahead)
        L = max(self.L_AHEAD_MIN, self.L_AHEAD_T * v_slot_mag, 0.8 * abs(e_cross))
        if v_slot_mag > 1e-3:
            aim_dir = math.atan2(ve, vn) + w * (L / v_slot_mag) * 0.5
        else:
            aim_dir = tr
        aim_n = slot_n + math.cos(aim_dir) * L
        aim_e = slot_e + math.sin(aim_dir) * L
        heading_cmd = math.degrees(math.atan2(aim_e - oe, aim_n - on)) % 360.0

        # vertical
        alt_cmd = slot_alt + climb * 4.0

        # roll_ff = atan(ω·V/g)·scale
        phi_ff = math.degrees(math.atan2(w * max(speed_cmd, 50.0), 9.81))
        ff_scale = max(0.0, 1.0 - abs(e_cross) / 400.0)
        roll_ff = phi_ff * ff_scale

        agent.ros_bridge.node.get_logger().info(
            f"[FormationGuidance] along={e_along:+.0f} cross={e_cross:+.0f} vert={slot_alt-oalt:+.0f} "
            f"Hdg={heading_cmd:.0f} V={speed_cmd:.0f} Rff={roll_ff:+.0f} | 리더(trk={track:.0f} ω={turn_rate:+.1f})")

        return self._emit(agent, own, heading_cmd, alt_cmd, speed_cmd, roll_ff)

    def _interpret_publish(self, msg, agent, blackboard) -> Status:
        return Status.RUNNING
