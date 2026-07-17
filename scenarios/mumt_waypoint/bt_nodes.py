"""
MUM-T 웨이포인트 추종 — BT-level direct 유도(heading/altitude), 인엔진 3계층
제어스택(FixedWingGuidance → F16CommandController) 구동.

Phase 4 개편으로 AircraftSetpoint.{use_waypoint, target_x/y/z}가 폐지됐다. 대신
매 틱 own→현재 웨이포인트 벡터에서 나침반 헤딩을 계산해 guidance_mode="direct"로
heading_deg/altitude_m/target_speed_mps를 발행한다(계산 자체는 BT가 10Hz로 수행 —
이 시나리오는 UE 인엔진 유도 모드를 쓰지 않는 direct-only 경로). 속도는 기존과
동일하게 오토스로틀(target_speed_mps)이 유지.

★ Takeoff 노드는 재사용하지 않는다: 첫 웨이포인트를 전방(활주로 동쪽)+상방에 두어
   지상에서부터 그 방위·고도를 조준하는 것만으로 이륙·상승이 되게 한다.

좌표: 웨이포인트는 사람이 읽기 쉬운 **스폰 상대 (dNorth, dEast, dUp) 미터**로 저작.
  BT가 스폰(최초관측) UE 위치에 더해 UE 월드 cm 절대좌표로 변환(도달판정·헤딩계산용):
    target_x = x0 + dEast*100      (UE +X = East)
    target_y = y0 - dNorth*100     (UE +Y = South → North은 -Y)
    target_z = z0 + dUp*100        (UE +Z = Up)
  헤딩은 scenarios.mumt의 unit_xy_to_heading(dx,dy) = atan2(ΔEast,ΔNorth) 재사용
  (UE x=East, y=South → 나침반 0=북,90=동과 동일 규약).
"""

import json
import math

from std_msgs.msg import String
from custom_msgs.msg import AircraftSetpoint

from modules.base_bt_nodes import (
    BTNodeList, Status, Node, Sequence, ReactiveSequence, Fallback, ReactiveFallback,
)
from modules.base_bt_nodes_ros import ConditionWithROSTopics, ActionWithROSTopic

from scenarios.mumt.bt_nodes import (
    _name_matches, _alt_m, _own_routing_name, unit_xy_to_heading,
    CM_TO_M, _STATE_TOPIC, _SETPOINT_TOPIC,
)

BTNodeList.CONDITION_NODES.extend(["GatherOwnState"])
BTNodeList.ACTION_NODES.extend(["FollowWaypoint"])

# 기본 경로 (스폰 상대 dNorth, dEast, dUp 미터). 활주로 헤딩 90°(동)이라 첫 점은 동쪽+상승.
DEFAULT_WAYPOINTS_NEU = [
    (0.0,    3000.0, 1000.0),   # 3km 동쪽 + 1000m 상승 — 이륙/상승 구간
    (3000.0, 3000.0, 1200.0),   # 북쪽으로 선회
    (3000.0, 0.0,    1200.0),   # 서쪽
    (0.0,    0.0,    1200.0),   # 남쪽 → 스폰 위로 복귀 (loop)
]


# ══════════════════════════════════════════════════════════════════════════════
# GatherOwnState — own 상태만 blackboard에 기록 (leader/enemy 불필요)
# ══════════════════════════════════════════════════════════════════════════════
class GatherOwnState(ConditionWithROSTopics):
    """상태 배치에서 own을 찾아 blackboard["own_state"]에 기록. 찾으면 SUCCESS.
       캐시가 최신 메시지를 유지하므로 순간 끊김에도 마지막 상태로 True 유지."""

    def __init__(self, name, agent, own_name=""):
        super().__init__(name, agent, [(String, _STATE_TOPIC, "states")])
        self._own_name = own_name or (getattr(agent, "agent_id", "") or "").strip("/")

    def _predicate(self, agent, blackboard) -> bool:
        raw = self._cache.get("states")
        if not raw:
            return False
        try:
            payload = json.loads(raw.data)
        except (json.JSONDecodeError, AttributeError):
            return False
        if isinstance(payload, dict) and "aircraft" in payload:
            aircraft = payload["aircraft"]
        elif isinstance(payload, list):
            aircraft = payload
        else:
            return False

        own = next((a for a in aircraft
                    if _name_matches(a.get("aircraft_name", ""), self._own_name)), None)
        blackboard["own_state"] = own
        return own is not None


# ══════════════════════════════════════════════════════════════════════════════
# FollowWaypoint — 스폰 상대 경로를 direct 헤딩/고도 조준으로 추종 (이륙 포함)
# ══════════════════════════════════════════════════════════════════════════════
class FollowWaypoint(ActionWithROSTopic):
    """스폰 상대 웨이포인트(NEU 미터)를 UE 월드 cm 절대좌표로 변환해, own→목표점
       방위를 나침반 헤딩으로 계산 후 guidance_mode="direct" setpoint로 발행.
       도달반경(3D) 안에 들면 다음 점으로, loop면 순환. 항상 RUNNING(끝은 SUCCESS 옵션).
       own 순간 결손 시 마지막 setpoint 재발행(래칭)."""

    def __init__(self, name, agent, own_name="",
                 cruise_speed_mps=220.0, accept_radius_m=400.0, loop=True,
                 min_agl_m=150.0):    # 고도 하한 가드: target_z ≥ 스폰+이 값
        super().__init__(name, agent, (AircraftSetpoint, _SETPOINT_TOPIC))
        self._own_name = own_name or (getattr(agent, "agent_id", "") or "").strip("/")
        self._speed    = float(cruise_speed_mps)
        self._radius   = float(accept_radius_m)
        self._loop     = bool(loop)
        self._min_agl  = float(min_agl_m)
        self._wps      = list(DEFAULT_WAYPOINTS_NEU)
        self._idx      = 0
        self._spawn    = None       # (x0,y0,z0) UE cm, 최초관측 래칭
        self._last_msg = None
        self._done     = False

    def _target_ue_cm(self, dN, dE, dU):
        """스폰 상대 (dNorth,dEast,dUp) 미터 → UE 월드 cm 절대좌표."""
        x0, y0, z0 = self._spawn
        return (x0 + dE * 100.0, y0 - dN * 100.0, z0 + dU * 100.0)

    def _build_message(self, agent, blackboard):
        own = blackboard.get("own_state")
        if not own:
            return self._last_msg      # 결손 → 래칭 (None이면 FAILURE, GatherOwnState가 먼저 막음)

        if self._spawn is None:
            self._spawn = (own.get("x", 0.0), own.get("y", 0.0), own.get("z", 0.0))

        dN, dE, dU = self._wps[self._idx]
        tx, ty, tz = self._target_ue_cm(dN, dE, dU)
        tz = max(tz, self._spawn[2] + self._min_agl * 100.0)   # 고도 하한 가드

        ox, oy, oz = own.get("x", 0.0), own.get("y", 0.0), own.get("z", 0.0)
        heading = unit_xy_to_heading(tx - ox, ty - oy)

        msg = AircraftSetpoint()
        msg.aircraft_name    = _own_routing_name(own, self._own_name)
        msg.guidance_mode    = "direct"
        msg.heading_deg      = float(heading)
        msg.altitude_m       = float(tz * CM_TO_M)
        msg.target_speed_mps = float(self._speed)

        # 도달 판정용 3D 거리(m)
        dist_m = math.hypot(math.hypot(tx - ox, ty - oy), tz - oz) * CM_TO_M

        agent.ros_bridge.node.get_logger().info(
            f"[FollowWaypoint] wp {self._idx+1}/{len(self._wps)} "
            f"(N{dN:.0f} E{dE:.0f} U{dU:.0f}) hdg={heading:.0f}° 거리={dist_m:.0f}m Vtgt={self._speed:.0f}")

        # 도달 시 다음 웨이포인트로
        if dist_m < self._radius:
            if self._idx + 1 < len(self._wps):
                self._idx += 1
            elif self._loop:
                self._idx = 0
            else:
                self._done = True

        self._last_msg = msg
        return msg

    def _interpret_publish(self, msg, agent, blackboard) -> Status:
        return Status.SUCCESS if self._done else Status.RUNNING
