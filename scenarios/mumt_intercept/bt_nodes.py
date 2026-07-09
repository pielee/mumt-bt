"""
MUM-T 요격 기능 체크 — 적기(원 선회) vs 아군 무인기(이륙→후방 추격→격추).

기체 배역 (RL_2 레벨, 팀은 에디터 HealthComponent.Team):
  F16_UAV1 = 적기(Enemy)        : 이륙 → 지정 원 선회 (OrbitPoint)
  F16_UAV2 = 아군(FriendlyUAV)  : 지상이면 이륙 → 적 생존 확인 → tail-chase 추격
                                   → 후방 사정거리·조준 조건 충족 시 격추 (InterceptTarget)
  M_F16    = 유인기(사람)        : 자유 비행 — P2 수정으로 아군의 표적에서 제외됨

★ 설계 근거:
  - 명령 방식 (Phase 4): InterceptTarget 교전은 guidance_mode="attack" + 표적지정만 —
    추격 heading/alt/speed는 UE(FPursuitGuidance)가 표적 pawn 직독으로 60Hz 계산.
    BT는 표적선정 + WEZ 방아쇠(10Hz로 충분)만. 이륙·OrbitPoint는 direct 명령 유지.
  - tail-chase 속도법칙: V = 적기속도 + kp×(거리−standoff), [min, chase_max] — 후방 위치 유지.
  - 적기 원 선회 = 8점 웨이포인트 순회 (반경 2km).
  - 추격 속도 상한 265: 선회반경 기하 고려 (고속 = 큰 선회원 = 수렴 불가).
  - 폐루프(UE) 재검증 TODO: 내루프 heading 추종 지연·정상상태 오차가 방아쇠(5°/15°)
    개폐에 미치는 영향 확인 (구 GetStick 검증치 무효).
"""

import math
import time

from std_msgs.msg import String
from custom_msgs.msg import AircraftSetpoint

from modules.base_bt_nodes import (
    BTNodeList, Status, Node, Sequence, ReactiveSequence, Fallback, ReactiveFallback,
)
from modules.base_bt_nodes_ros import ConditionWithROSTopics, ActionWithROSTopic

# 재사용 (import 부수효과로 등록 포함): GatherCombatState는 P2 적대관계 수정판
from scenarios.mumt.bt_nodes import (
    clamp, heading_to_unit_xy, unit_xy_to_heading, _alt_m, _own_routing_name,
    CM_TO_M, _STATE_TOPIC, _SETPOINT_TOPIC,
)
from scenarios.mumt_dogfight_1v1.bt_nodes import GatherCombatState, ConditionEnemyAlive
from scenarios.mumt_waypoint.bt_nodes import GatherOwnState

BTNodeList.ACTION_NODES.extend(["OrbitPoint", "InterceptTarget"])


class _TakeoffMixin:
    """스폰 래칭 + 이륙 페이즈 공용 로직 (OrbitPoint/InterceptTarget 공유)."""

    def _takeoff_init(self, runway_heading_deg, takeoff_forward_m, takeoff_up_m,
                      takeoff_speed_mps, takeoff_climb_m, min_agl_m):
        self._rwy_hdg  = float(runway_heading_deg)
        self._to_fwd   = float(takeoff_forward_m)
        self._to_up    = float(takeoff_up_m)
        self._to_spd   = float(takeoff_speed_mps)
        self._to_climb = float(takeoff_climb_m)
        self._min_agl  = float(min_agl_m)
        self._spawn    = None
        self._airborne = False

    def _takeoff_update(self, agent, own):
        """스폰 래칭·이륙 완료 판정. 이륙 중이면 (vx,vy,vz,speed) 반환, 완료면 None."""
        ox, oy, oz = own.get("x", 0.0), own.get("y", 0.0), own.get("z", 0.0)
        if self._spawn is None:
            self._spawn = (ox, oy, oz)
        climb_m = (oz - self._spawn[2]) * CM_TO_M
        if not self._airborne and climb_m >= self._to_climb:
            self._airborne = True
            agent.ros_bridge.node.get_logger().info(
                f"[{self.name}] 이륙 완료(+{climb_m:.0f}m)")
        if self._airborne:
            return None
        fx, fy = heading_to_unit_xy(self._rwy_hdg)
        return (self._spawn[0] + fx * self._to_fwd * 100.0,
                self._spawn[1] + fy * self._to_fwd * 100.0,
                self._spawn[2] + self._to_up * 100.0,
                self._to_spd)

    def _agl_floor(self, vz):
        return max(vz, self._spawn[2] + self._min_agl * 100.0)


class OrbitPoint(_TakeoffMixin, ActionWithROSTopic):
    """이륙 후 지정 원을 계속 선회 (적기/대기 공용). 항상 RUNNING.

    원 = 스폰 기준 (center_north/east_m, orbit_alt_m 상대) 중심, radius_m 반경의
    num_points개 웨이포인트 순회 (accept_radius_m 도달 시 다음 점)."""

    def __init__(self, name, agent, own_name="",
                 center_north_m=0.0, center_east_m=3000.0, radius_m=2000.0,
                 orbit_alt_m=800.0, speed_mps=180.0,
                 num_points=8, accept_radius_m=400.0,
                 runway_heading_deg=90.0, takeoff_forward_m=3000.0, takeoff_up_m=800.0,
                 takeoff_speed_mps=220.0, takeoff_climb_m=150.0, min_agl_m=150.0):
        super().__init__(name, agent, (AircraftSetpoint, _SETPOINT_TOPIC))
        self._own_name = own_name or (getattr(agent, "agent_id", "") or "").strip("/")
        self._cn, self._ce = float(center_north_m), float(center_east_m)
        self._radius   = float(radius_m)
        self._orbit_up = float(orbit_alt_m)
        self._speed    = float(speed_mps)
        self._npts     = int(num_points)
        self._accept   = float(accept_radius_m)
        self._takeoff_init(runway_heading_deg, takeoff_forward_m, takeoff_up_m,
                           takeoff_speed_mps, takeoff_climb_m, min_agl_m)
        self._wps  = None
        self._idx  = 0
        self._last_msg = None

    def _build_message(self, agent, blackboard):
        own = blackboard.get("own_state")
        if not own:
            return self._last_msg

        to = self._takeoff_update(agent, own)
        if to is not None:
            vx, vy, vz, speed = to
        else:
            if self._wps is None:   # 원주 웨이포인트 생성 (UE cm 절대좌표)
                ux = self._spawn[0] + self._ce * 100.0            # E → +x
                uy = self._spawn[1] - self._cn * 100.0            # N → -y
                uz = self._spawn[2] + self._orbit_up * 100.0
                self._wps = []
                for k in range(self._npts):
                    a = 2.0 * math.pi * k / self._npts
                    n_rel, e_rel = self._radius * math.cos(a), self._radius * math.sin(a)
                    self._wps.append((ux + e_rel * 100.0, uy - n_rel * 100.0, uz))
            ox, oy = own.get("x", 0.0), own.get("y", 0.0)
            wx, wy, wz = self._wps[self._idx]
            if math.hypot(wx - ox, wy - oy) * CM_TO_M < self._accept:
                self._idx = (self._idx + 1) % self._npts
                wx, wy, wz = self._wps[self._idx]
            vx, vy, vz, speed = wx, wy, wz, self._speed
            agent.ros_bridge.node.get_logger().info(
                f"[OrbitPoint] wp {self._idx+1}/{self._npts} Vtgt={speed:.0f}")

        vz = self._agl_floor(vz)
        ox, oy = own.get("x", 0.0), own.get("y", 0.0)         # UE cm
        heading = unit_xy_to_heading(vx - ox, vy - oy)        # 목표점 방위 → 나침반 heading
        msg = AircraftSetpoint()
        msg.aircraft_name    = _own_routing_name(own, self._own_name)
        msg.heading_deg      = float(heading)                 # 내루프(heading PID) 추종
        msg.altitude_m       = float(vz * CM_TO_M)            # 목표점 고도(UE-Z m)
        msg.target_speed_mps = float(speed)
        self._last_msg = msg
        return msg

    def _interpret_publish(self, msg, agent, blackboard) -> Status:
        return Status.RUNNING


class InterceptTarget(_TakeoffMixin, ActionWithROSTopic):
    """지상이면 이륙 → 최근접 적기를 tail-chase(적기 자체 조준)로 추격 →
    후방 standoff_m에 수렴하면 자연히 사격 기하 형성 → WEZ 방아쇠.
      속도 = 적기속도 + kp×(거리−standoff), [min, chase_max] — 후방 위치 유지
      미사일: 거리<missile_range ∧ |방위오차|<missile_bearing ∧ 쿨다운 ∧ 잔탄
      기총  : 거리<gun_range ∧ |방위오차|<gun_bearing 인 동안 연사
    적 전멸 → 기총 끄고 SUCCESS. own/적 순간 결손 → 래칭."""

    def __init__(self, name, agent, own_name="",
                 standoff_m=200.0, kp_speed=0.15,
                 chase_speed_max=265.0, min_speed_mps=70.0,
                 gun_range_m=1500.0, gun_bearing_deg=5.0,
                 missile_range_m=8000.0, missile_bearing_deg=15.0,
                 missile_cooldown_sec=10.0,
                 runway_heading_deg=90.0, takeoff_forward_m=3000.0, takeoff_up_m=800.0,
                 takeoff_speed_mps=220.0, takeoff_climb_m=150.0, min_agl_m=150.0):
        super().__init__(name, agent, (AircraftSetpoint, _SETPOINT_TOPIC))
        self._own_name  = own_name or (getattr(agent, "agent_id", "") or "").strip("/")
        self._standoff  = float(standoff_m)
        self._kp        = float(kp_speed)
        self._spd_max   = float(chase_speed_max)
        self._spd_min   = float(min_speed_mps)
        self._gun_range = float(gun_range_m)
        self._gun_brg   = float(gun_bearing_deg)
        self._msl_range = float(missile_range_m)
        self._msl_brg   = float(missile_bearing_deg)
        self._msl_cd    = float(missile_cooldown_sec)
        self._takeoff_init(runway_heading_deg, takeoff_forward_m, takeoff_up_m,
                           takeoff_speed_mps, takeoff_climb_m, min_agl_m)
        self._fire_id   = 0
        self._last_fire = None
        self._last_msg  = None
        self._done      = False

    def _build_message(self, agent, blackboard):
        own     = blackboard.get("own_state")
        enemies = blackboard.get("enemies") or []
        if not own:
            return self._last_msg

        msg = AircraftSetpoint()
        msg.aircraft_name = _own_routing_name(own, self._own_name)

        to = self._takeoff_update(agent, own)
        if to is not None:                       # 지상 → 먼저 이륙 (direct 명령)
            vx, vy, vz, speed = to
            vz = self._agl_floor(vz)
            ox, oy = own.get("x", 0.0), own.get("y", 0.0)     # UE cm
            msg.heading_deg      = float(unit_xy_to_heading(vx - ox, vy - oy))
            msg.altitude_m       = float(vz * CM_TO_M)
            msg.target_speed_mps = float(speed)
            msg.missile_fire_id  = self._fire_id
        elif not enemies:                        # 적 전멸 → 기총 끄고 SUCCESS
            self._done = True
            if self._last_msg is not None:
                self._last_msg.gun_firing = False
                return self._last_msg
            msg.heading_deg      = float(own.get("yaw", 0.0))
            msg.altitude_m       = float(_alt_m(own))
            msg.target_speed_mps = 200.0
            msg.missile_fire_id  = self._fire_id
        else:
            # ── 교전: 추격 유도는 UE attack 모드(60Hz, 표적 직독) — BT는 표적지정+방아쇠만 ──
            self._done = False
            ox, oy = own.get("x", 0.0) * CM_TO_M, own.get("y", 0.0) * CM_TO_M
            oyaw   = own.get("yaw", 0.0)

            def dist_m(a):
                return math.hypot(a.get("x", 0.0) * CM_TO_M - ox,
                                  a.get("y", 0.0) * CM_TO_M - oy)
            target = min(enemies, key=dist_m)
            dist   = dist_m(target)
            bearing = unit_xy_to_heading(target.get("x", 0.0) * CM_TO_M - ox,
                                         target.get("y", 0.0) * CM_TO_M - oy)
            brg_err = abs(((bearing - oyaw + 180.0) % 360.0) - 180.0)

            # WEZ 방아쇠 (10Hz 상태 기반 — 지연 비민감, BT 유지)
            now = time.monotonic()
            fired = False
            if (dist < self._msl_range and brg_err < self._msl_brg
                    and own.get("missile_count", 1) > 0
                    and (self._last_fire is None or now - self._last_fire >= self._msl_cd)):
                self._fire_id += 1
                self._last_fire = now
                fired = True
            gun = dist < self._gun_range and brg_err < self._gun_brg

            msg.guidance_mode = "attack"
            msg.target_name   = str(target.get("aircraft_name", ""))
            msg.min_speed_mps = float(self._spd_min)
            msg.max_speed_mps = float(self._spd_max)          # 추격 상한(선회반경 기하)
            msg.min_alt_m     = float(self._spawn[2] * CM_TO_M + self._min_agl)
            msg.gun_firing    = bool(gun)
            msg.missile_fire_id = self._fire_id

            agent.ros_bridge.node.get_logger().info(
                f"[InterceptTarget] tgt={target.get('aircraft_name')} 거리={dist:.0f}m "
                f"방위오차={brg_err:.1f}° hp={target.get('hp', '?')} | attack모드(UE 60Hz) "
                f"gun={'ON' if gun else 'off'} msl_id={self._fire_id}{' ★발사' if fired else ''}")

        self._last_msg = msg
        return msg

    def _interpret_publish(self, msg, agent, blackboard) -> Status:
        return Status.SUCCESS if self._done else Status.RUNNING
