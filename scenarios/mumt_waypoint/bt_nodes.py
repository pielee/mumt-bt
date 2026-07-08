"""
MUM-T 웨이포인트 추종 — StickController(Controller_CY) 조준 제어기 구동.

BVRGym 제거 후 새 제어 방식: BT는 heading/altitude가 아니라 **목표점(UE 월드 cm)**을
AircraftSetpoint.{use_waypoint, target_x/y/z}로 보낸다. UE가 그 점을 NEU 미터로 변환해
StickController::GetStick에 먹여 기수를 조준(pitch로 상하까지)한다. 속도는 기존
오토스로틀(target_speed_mps)이 유지 — StickController는 throttle 출력이 없다.

★ Takeoff 노드는 재사용하지 않는다: 그건 heading/altitude를 보내는데 BVRGym이
   사라져 UE가 무시한다. 대신 첫 웨이포인트를 전방(활주로 동쪽)+상방에 두어
   StickController가 지상에서부터 기수를 들어 이륙·상승하게 한다.

좌표: 웨이포인트는 사람이 읽기 쉬운 **스폰 상대 (dNorth, dEast, dUp) 미터**로 저작.
  BT가 스폰(최초관측) UE 위치에 더해 UE 월드 cm로 패킹해 전송:
    target_x = x0 + dEast*100      (UE +X = East)
    target_y = y0 - dNorth*100     (UE +Y = South → North은 -Y)
    target_z = z0 + dUp*100        (UE +Z = Up)
  UE는 역변환(North=-y/100, East=x/100, Up=z/100)으로 되돌린다 (왕복 항등).
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
    _name_matches, _alt_m, _own_routing_name, CM_TO_M, _STATE_TOPIC, _SETPOINT_TOPIC,
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
# FollowWaypoint — 스폰 상대 경로를 조준 제어기로 추종 (이륙 포함)
# ══════════════════════════════════════════════════════════════════════════════
class FollowWaypoint(ActionWithROSTopic):
    """스폰 상대 웨이포인트(NEU 미터)를 UE 월드 cm로 패킹해 use_waypoint setpoint로 발행.
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

        msg = AircraftSetpoint()
        msg.aircraft_name    = _own_routing_name(own, self._own_name)
        msg.use_waypoint     = True
        msg.target_x         = float(tx)
        msg.target_y         = float(ty)
        msg.target_z         = float(tz)
        msg.target_speed_mps = float(self._speed)

        # 도달 판정용 3D 거리(m)
        ox, oy, oz = own.get("x", 0.0), own.get("y", 0.0), own.get("z", 0.0)
        dist_m = math.hypot(math.hypot(tx - ox, ty - oy), tz - oz) * CM_TO_M

        agent.ros_bridge.node.get_logger().info(
            f"[FollowWaypoint] wp {self._idx+1}/{len(self._wps)} "
            f"(N{dN:.0f} E{dE:.0f} U{dU:.0f}) 거리={dist_m:.0f}m Vtgt={self._speed:.0f}")

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
