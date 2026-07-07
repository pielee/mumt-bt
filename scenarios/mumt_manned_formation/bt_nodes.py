"""
MUM-T 유인기 편대비행 — StickController(조준 제어기) 기반, 우측 전방 슬롯.

유인기(M_F16)는 사람이 조이스틱으로 조종하고, UAV(F16_UAV1)는:
  ① 자체 이륙: 스폰 전방(활주로 헤딩)+상방 목표점을 조준해 지상에서 이륙·상승
  ② 편대 합류: 스폰 대비 takeoff_climb_m 이상 오르면 유인기 슬롯 추종으로 전환

슬롯 = 유인기 **앞 front_m · 우측 right_m · 고도차 up_m** (사용자 지정: 앞100/우40/동일고도).

★ 조준 제어기(StickController)의 슬롯 추종 설계:
  슬롯 자체를 조준하면 슬롯에 도달한 순간 목표거리→0으로 기하가 퇴화(명령↓)한다.
  대신 **캐럿(carrot) 조준**: 슬롯에서 리더 헤딩 방향으로 carrot_m 앞의 점을 조준
  → 정위치에선 기수가 리더 헤딩과 정렬되고, 횡편차는 자연히 슬롯 라인으로 수렴.
  전후 위치는 속도로 잡는다: 목표속도 = 리더속도 + kp×(슬롯까지 along-track 거리),
  [closure_min, closure_max] 클램프, [min, max] 속도 한계 (MaintainFormation과 같은 법칙).

★ 전방 슬롯 주의: UAV가 리더를 '앞질러' 유지해야 하므로, 같은 기체 특성상
  유인기가 최고속도로 날면 유지 불가 — 중간 스로틀로 비행해야 슬롯이 잡힌다.

좌표: 슬롯/캐럿은 UE 월드 cm로 직접 계산해 use_waypoint setpoint로 전송
  (UE x=East, y=South, z=Up; 나침반 헤딩 h → 단위벡터 (sin h, -cos h)).
"""

import math

from std_msgs.msg import String
from custom_msgs.msg import AircraftSetpoint

from modules.base_bt_nodes import (
    BTNodeList, Status, Node, Sequence, ReactiveSequence, Fallback, ReactiveFallback,
)
from modules.base_bt_nodes_ros import ConditionWithROSTopics, ActionWithROSTopic

# 기존 노드/헬퍼 재사용 (import 부수효과로 GatherState 등록 포함)
from scenarios.mumt.bt_nodes import (
    GatherState, clamp, heading_to_unit_xy, _alt_m, _own_routing_name,
    CM_TO_M, _STATE_TOPIC, _SETPOINT_TOPIC,
)

BTNodeList.ACTION_NODES.extend(["FormationFlight"])


class FormationFlight(ActionWithROSTopic):
    """자체 이륙 → 유인기 우측 전방 슬롯 편대 (단일 노드, 내부 페이즈 전환).

    TAKEOFF : 스폰 xy + 활주로 헤딩 방향 takeoff_forward_m, 상방 takeoff_up_m 지점 조준,
              takeoff_speed_mps. 스폰 대비 takeoff_climb_m 상승 시 FORMATION 전환.
    FORMATION: 캐럿 조준 + along-track 속도법칙 (모듈 docstring 참조).
    항상 RUNNING. own/leader 순간 결손 시 마지막 setpoint 재발행(래칭)."""

    def __init__(self, name, agent, own_name="", leader_name="M_F16",
                 front_m=100.0, right_m=40.0, up_m=0.0, carrot_m=800.0,
                 kp_speed=0.05, closure_min_mps=-60.0, closure_max_mps=60.0,
                 min_speed_mps=70.0, max_speed_mps=335.0,   # 상한=JSBSim f16 실측 수평 지속최대(~335m/s@0.3~1.2km, M0.98)
                 runway_heading_deg=90.0,
                 takeoff_forward_m=3000.0, takeoff_up_m=800.0,
                 takeoff_speed_mps=220.0, takeoff_climb_m=150.0):
        super().__init__(name, agent, (AircraftSetpoint, _SETPOINT_TOPIC))
        self._own_name  = own_name or (getattr(agent, "agent_id", "") or "").strip("/")
        self._front     = float(front_m)
        self._right     = float(right_m)
        self._up        = float(up_m)
        self._carrot    = float(carrot_m)
        self._kp        = float(kp_speed)
        self._clo_min   = float(closure_min_mps)
        self._clo_max   = float(closure_max_mps)
        self._spd_min   = float(min_speed_mps)
        self._spd_max   = float(max_speed_mps)
        self._rwy_hdg   = float(runway_heading_deg)
        self._to_fwd    = float(takeoff_forward_m)
        self._to_up     = float(takeoff_up_m)
        self._to_spd    = float(takeoff_speed_mps)
        self._to_climb  = float(takeoff_climb_m)
        self._spawn     = None          # (x,y,z) UE cm — own 최초 관측 래칭
        self._airborne  = False         # False=TAKEOFF, True=FORMATION
        self._last_msg  = None

    def _build_message(self, agent, blackboard):
        own    = blackboard.get("own_state")
        leader = blackboard.get("leader_state")
        if not own or not leader:
            return self._last_msg       # 결손 → 래칭 (GatherState가 선행 차단)

        ox, oy, oz = own.get("x", 0.0), own.get("y", 0.0), own.get("z", 0.0)
        if self._spawn is None:
            self._spawn = (ox, oy, oz)

        # 페이즈 전환: 스폰 대비 상승량
        climb_m = (oz - self._spawn[2]) * CM_TO_M
        if not self._airborne and climb_m >= self._to_climb:
            self._airborne = True
            agent.ros_bridge.node.get_logger().info(
                f"[FormationFlight] 이륙 완료(+{climb_m:.0f}m) → 편대 합류 전환")

        if not self._airborne:
            # ── TAKEOFF: 스폰 전방+상방 목표점 조준 ──
            fx, fy = heading_to_unit_xy(self._rwy_hdg)
            vx = self._spawn[0] + fx * self._to_fwd * 100.0
            vy = self._spawn[1] + fy * self._to_fwd * 100.0
            vz = self._spawn[2] + self._to_up * 100.0
            speed = self._to_spd
            agent.ros_bridge.node.get_logger().info(
                f"[FormationFlight] 이륙 상승 +{climb_m:.0f}/{self._to_climb:.0f}m Vtgt={speed:.0f}")
        else:
            # ── FORMATION: 슬롯(앞 front · 우 right · 고도차 up) + 캐럿 조준 ──
            lx, ly, lz = leader.get("x", 0.0), leader.get("y", 0.0), leader.get("z", 0.0)
            lyaw = leader.get("yaw", 0.0)
            lspd = leader.get("speed_mps", 0.0)
            fx, fy = heading_to_unit_xy(lyaw)                    # 리더 전진 (UE 단위벡터)
            rx, ry = heading_to_unit_xy((lyaw + 90.0) % 360.0)   # 리더 우측

            sx = lx + (self._front * fx + self._right * rx) * 100.0
            sy = ly + (self._front * fy + self._right * ry) * 100.0
            sz = lz + self._up * 100.0

            vx = sx + fx * self._carrot * 100.0                  # 캐럿: 슬롯의 리더헤딩 전방
            vy = sy + fy * self._carrot * 100.0
            vz = sz

            # 전후 위치는 속도로: 슬롯까지 along-track 거리(m) → 리더속도 ± closure
            along_m = ((sx - ox) * fx + (sy - oy) * fy) * CM_TO_M
            speed = clamp(lspd + clamp(self._kp * along_m, self._clo_min, self._clo_max),
                          self._spd_min, self._spd_max)

            slot_dist = math.hypot(sx - ox, sy - oy) * CM_TO_M
            agent.ros_bridge.node.get_logger().info(
                f"[FormationFlight] 슬롯거리={slot_dist:.0f}m along={along_m:+.0f}m "
                f"Vtgt={speed:.0f} | 리더(hdg={lyaw:.0f} spd={lspd:.0f})")

        msg = AircraftSetpoint()
        msg.aircraft_name    = _own_routing_name(own, self._own_name)
        msg.use_waypoint     = True
        msg.target_x         = float(vx)
        msg.target_y         = float(vy)
        msg.target_z         = float(vz)
        msg.target_speed_mps = float(speed)
        self._last_msg = msg
        return msg

    def _interpret_publish(self, msg, agent, blackboard) -> Status:
        return Status.RUNNING           # 편대는 끝나지 않음
