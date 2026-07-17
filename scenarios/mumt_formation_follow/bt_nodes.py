"""
MUMT 편대 추종(formation-follow) BT — 인엔진 3계층 유도
(FormationGuidance → FixedWingGuidance → F16CommandController) 위에서 BT는
모드/파라미터만 지정한다. 틱마다 슬롯·closure 같은 유도 숫자를 계산하지 않음
(그건 UE가 리더 pawn을 직독해 60Hz로 계산).

흐름: GatherState(상태 인지, scenarios.mumt 재사용) → WaitForLeaderTakeoff/Takeoff
(재사용) → SetLeader/SetFormationSlot(blackboard 기록) → EnableFormationFollow
(formation setpoint 발행 + UE 래치 확인) → CheckFormationCaptured(슬롯 캡처 대기)
→ CheckFormationMaintained(편대 유지 모니터).

leader_id/follower_id, 슬롯 오프셋·허용오차는 SetLeader/SetFormationSlot이
blackboard에 기록하고 EnableFormationFollow가 읽어 AircraftSetpoint
(guidance_mode="formation")로 패킹해 발행한다. LeaveFormation은 편대 이탈이자
곧 '편대추종 해제' 명령(direct 모드로 전환) — 별도 Disable 노드는 두지 않는다.

own_state의 "guidance" 필드는 UE가 5006 상태 배치에 실어 보내는 유도 상태
(mode/e_along_m/e_cross_m/closing_mps/captured/maintained/...)이며, 편대 모드가
아니거나 UE가 아직 갱신하지 않았으면 없을 수 있으므로 항상 방어적으로 읽는다.
"""

import time

from custom_msgs.msg import AircraftSetpoint

from modules.base_bt_nodes import BTNodeList, Status, Node, Sequence, ReactiveSequence
from modules.base_bt_nodes_ros import ActionWithROSTopic

# GatherState/WaitForLeaderTakeoff/Takeoff 재사용 (import 부수효과로 등록도 포함)
from scenarios.mumt.bt_nodes import (
    GatherState, WaitForLeaderTakeoff, Takeoff,
    _alt_m, _own_routing_name, _SETPOINT_TOPIC,
)

# ── 노드 등록 ──────────────────────────────────────────────────────────────────
BTNodeList.ACTION_NODES.extend(
    ["SetLeader", "SetFormationSlot", "EnableFormationFollow", "LeaveFormation"])
BTNodeList.CONDITION_NODES.extend(["CheckFormationCaptured", "CheckFormationMaintained"])

# ── 상수 ────────────────────────────────────────────────────────────────────────
DEFAULT_LEADER_ID   = "M_F16"
DEFAULT_FOLLOWER_ID = "F16_UAV1"
SAFE_LEAVE_SPEED_MPS = 220.0   # halt() 안전 setpoint 목표속도


def _guidance(own) -> dict:
    """own_state에서 UE 인엔진 유도 상태(guidance)를 방어적으로 읽는다 (없으면 빈 dict)."""
    return (own or {}).get("guidance") or {}


# ══════════════════════════════════════════════════════════════════════════════
# SetLeader / SetFormationSlot — blackboard 파라미터 기록 (ROS 없음, 동기)
# ══════════════════════════════════════════════════════════════════════════════
class SetLeader(Node):
    """리더/팔로워 식별자를 blackboard에 기록. ROS 통신 없이 즉시 SUCCESS."""

    def __init__(self, name, agent, leader_id=DEFAULT_LEADER_ID, follower_id=DEFAULT_FOLLOWER_ID):
        super().__init__(name)
        self.type = "Action"
        self._leader_id   = str(leader_id)
        self._follower_id = str(follower_id)

    async def run(self, agent, blackboard):
        blackboard["leader_id"]   = self._leader_id
        blackboard["follower_id"] = self._follower_id
        self.status = Status.SUCCESS
        return self.status


class SetFormationSlot(Node):
    """편대 슬롯 오프셋·허용오차를 blackboard dict 'formation_slot'에 기록.
       ROS 통신 없이 즉시 SUCCESS."""

    def __init__(self, name, agent,
                 offset_front_m=-100.0, offset_right_m=50.0, offset_up_m=0.0,
                 capture_tolerance_m=30.0, maintain_tolerance_m=50.0,
                 minimum_separation_m=30.0, maximum_closing_speed_mps=0.0):
        super().__init__(name)
        self.type = "Action"
        self._slot = {
            "offset_front_m":            float(offset_front_m),
            "offset_right_m":            float(offset_right_m),
            "offset_up_m":                float(offset_up_m),
            "capture_tolerance_m":        float(capture_tolerance_m),
            "maintain_tolerance_m":       float(maintain_tolerance_m),
            "minimum_separation_m":       float(minimum_separation_m),
            "maximum_closing_speed_mps":  float(maximum_closing_speed_mps),
        }

    async def run(self, agent, blackboard):
        blackboard["formation_slot"] = dict(self._slot)
        self.status = Status.SUCCESS
        return self.status


# ══════════════════════════════════════════════════════════════════════════════
# EnableFormationFollow — formation setpoint 발행 (UE 래치 → confirm_ticks회 확인)
# ══════════════════════════════════════════════════════════════════════════════
class EnableFormationFollow(ActionWithROSTopic):
    """SetLeader/SetFormationSlot이 채운 blackboard(leader_id/follower_id/
       formation_slot)로 guidance_mode="formation" setpoint를 구성해 발행.
       UE가 setpoint를 래치하므로 confirm_ticks회 발행하면 SUCCESS(그 사이 RUNNING).

       halt() 시(하위 조건 FAILURE로 인한 Sequence 리셋 등) direct 모드 안전
       setpoint를 1회 발행해 편대 명령이 UE에 남아있지 않게 한다 — 목표는 own의
       현재 헤딩·고도 유지(target_speed_mps=220)."""

    def __init__(self, name, agent, min_speed_mps=120.0, max_speed_mps=335.0,
                 min_agl_m=150.0, confirm_ticks=5):
        super().__init__(name, agent, (AircraftSetpoint, _SETPOINT_TOPIC))
        self._spd_min  = float(min_speed_mps)
        self._spd_max  = float(max_speed_mps)
        self._min_agl  = float(min_agl_m)
        self._confirm  = int(confirm_ticks)
        self._ticks    = 0
        self._last_own = None
        self._own_name = ""

    def _build_message(self, agent, blackboard):
        own = blackboard.get("own_state")
        if own:
            self._last_own = own

        leader_id   = blackboard.get("leader_id", DEFAULT_LEADER_ID)
        follower_id = blackboard.get("follower_id", DEFAULT_FOLLOWER_ID)
        self._own_name = _own_routing_name(own, follower_id)
        slot = blackboard.get("formation_slot") or {}

        base = (blackboard.get("init_alt") or {}).get(
            (own or {}).get("aircraft_name", ""), _alt_m(own) if own else 0.0)

        msg = AircraftSetpoint()
        msg.aircraft_name    = self._own_name
        msg.guidance_mode    = "formation"
        msg.leader_name       = str(leader_id)
        msg.slot_front_m      = float(slot.get("offset_front_m", 0.0))
        msg.slot_right_m      = float(slot.get("offset_right_m", 0.0))
        msg.slot_up_m          = float(slot.get("offset_up_m", 0.0))
        msg.capture_tolerance_m       = float(slot.get("capture_tolerance_m", 0.0))
        msg.maintain_tolerance_m      = float(slot.get("maintain_tolerance_m", 0.0))
        msg.minimum_separation_m      = float(slot.get("minimum_separation_m", 0.0))
        msg.maximum_closing_speed_mps = float(slot.get("maximum_closing_speed_mps", 0.0))
        msg.min_speed_mps    = self._spd_min
        msg.max_speed_mps    = self._spd_max
        msg.min_alt_m        = float(base + self._min_agl)

        agent.ros_bridge.node.get_logger().info(
            f"[EnableFormationFollow] leader={leader_id} slot=(F{msg.slot_front_m:.0f} "
            f"R{msg.slot_right_m:.0f} U{msg.slot_up_m:.0f}) minAlt={msg.min_alt_m:.0f} "
            f"tick={self._ticks + 1}/{self._confirm}")
        return msg

    def _interpret_publish(self, msg, agent, blackboard) -> Status:
        self._ticks += 1
        return Status.SUCCESS if self._ticks >= self._confirm else Status.RUNNING

    def halt(self):
        self._ticks = 0
        pub = getattr(self, "_pub", None)
        own = self._last_own
        if pub is None or own is None:
            return
        msg = AircraftSetpoint()
        msg.aircraft_name    = self._own_name or str(own.get("aircraft_name", ""))
        msg.guidance_mode    = "direct"
        msg.heading_deg      = float(own.get("yaw", 0.0))
        msg.altitude_m       = float(_alt_m(own))
        msg.target_speed_mps = SAFE_LEAVE_SPEED_MPS
        pub.publish(msg)
        self.ros.node.get_logger().info(
            "[EnableFormationFollow] halt → direct 안전 setpoint 발행 (편대명령 래치 방지)")


# ══════════════════════════════════════════════════════════════════════════════
# CheckFormationCaptured — 슬롯 캡처 대기 (own_state.guidance.captured)
# ══════════════════════════════════════════════════════════════════════════════
class CheckFormationCaptured(Node):
    """own_state.guidance.captured가 True가 될 때까지 RUNNING.
       timeout_s(최초 틱 기준 경과 시간) 초과 시 FAILURE. own_state는 GatherState가
       채우므로 별도 토픽 구독 없이 blackboard만 읽는다."""

    def __init__(self, name, agent, timeout_s=180.0):
        super().__init__(name)
        self.type = "Condition"
        self.is_expanded = False
        self._timeout = float(timeout_s)
        self._t0 = None

    async def run(self, agent, blackboard):
        if self._t0 is None:
            self._t0 = time.monotonic()

        own = blackboard.get("own_state")
        captured = bool(_guidance(own).get("captured", False))

        if captured:
            self.status = Status.SUCCESS
        elif time.monotonic() - self._t0 >= self._timeout:
            agent.ros_bridge.node.get_logger().warn(
                f"[CheckFormationCaptured] timeout {self._timeout:.0f}s 초과 → FAILURE")
            self.status = Status.FAILURE
        else:
            self.status = Status.RUNNING

        blackboard[self.name] = {"status": self.status, "is_expanded": self.is_expanded}
        return self.status

    def halt(self):
        self._t0 = None


# ══════════════════════════════════════════════════════════════════════════════
# CheckFormationMaintained — 편대 유지 모니터
# ══════════════════════════════════════════════════════════════════════════════
class CheckFormationMaintained(Node):
    """own_state.guidance.captured/maintained를 감시하는 모니터 노드.
         - captured가 break_grace_s 이상 연속으로 False → FAILURE (편대 붕괴)
         - maintained의 짧은 dip(< break_grace_s)은 무시하고 연속유지 스트릭을 깨지 않음
         - hold_s>0 이면 maintained가 hold_s 연속 유지될 때 SUCCESS (임무 종료 모드)
         - hold_s=0(기본)이면 계속 RUNNING (편대비행을 계속 유지하는 시나리오)"""

    def __init__(self, name, agent, hold_s=0.0, break_grace_s=3.0):
        super().__init__(name)
        self.type = "Condition"
        self.is_expanded = False
        self._hold  = float(hold_s)
        self._grace = float(break_grace_s)
        self._hold_since         = None   # 연속 maintained 스트릭 시작 시각
        self._maint_lost_since   = None   # maintained가 끊긴 시작 시각(dip 판정용)
        self._capture_lost_since = None   # captured가 끊긴 시작 시각(붕괴 판정용)

    async def run(self, agent, blackboard):
        now = time.monotonic()
        own = blackboard.get("own_state")
        g = _guidance(own)
        captured   = bool(g.get("captured", False))
        maintained = bool(g.get("maintained", False))

        # 1) captured 이탈 감시 — grace 초과 지속 시 편대 붕괴 FAILURE
        if captured:
            self._capture_lost_since = None
        else:
            if self._capture_lost_since is None:
                self._capture_lost_since = now
            elif now - self._capture_lost_since >= self._grace:
                agent.ros_bridge.node.get_logger().warn(
                    f"[CheckFormationMaintained] captured 이탈 {self._grace:.0f}s 초과 → FAILURE")
                self.status = Status.FAILURE
                blackboard[self.name] = {"status": self.status, "is_expanded": self.is_expanded}
                return self.status

        # 2) maintained 연속시간 추적 — grace 이내 dip은 스트릭을 리셋하지 않음
        if maintained:
            self._maint_lost_since = None
            if self._hold_since is None:
                self._hold_since = now
        else:
            if self._maint_lost_since is None:
                self._maint_lost_since = now
            elif now - self._maint_lost_since >= self._grace:
                self._hold_since = None   # 긴 dip → 연속유지 스트릭 리셋(FAILURE는 아님)

        if self._hold > 0.0 and self._hold_since is not None and now - self._hold_since >= self._hold:
            self.status = Status.SUCCESS
        else:
            self.status = Status.RUNNING

        blackboard[self.name] = {"status": self.status, "is_expanded": self.is_expanded}
        return self.status

    def halt(self):
        self._hold_since = None
        self._maint_lost_since = None
        self._capture_lost_since = None


# ══════════════════════════════════════════════════════════════════════════════
# LeaveFormation — 편대 이탈 (= 편대추종 해제, 별도 Disable 노드 없음)
# ══════════════════════════════════════════════════════════════════════════════
class LeaveFormation(ActionWithROSTopic):
    """direct 모드로 전환해 현재 헤딩+오프셋으로 이탈 기동. publish_ticks회 발행 후
       SUCCESS — EnableFormationFollow가 발행한 formation 명령을 UE에서 확실히
       덮어써서 편대추종을 해제하는 역할도 겸한다."""

    def __init__(self, name, agent, heading_offset_deg=45.0, alt_offset_m=0.0,
                 speed_mps=220.0, publish_ticks=5):
        super().__init__(name, agent, (AircraftSetpoint, _SETPOINT_TOPIC))
        self._hdg_off       = float(heading_offset_deg)
        self._alt_off       = float(alt_offset_m)
        self._speed         = float(speed_mps)
        self._ticks_needed  = int(publish_ticks)
        self._ticks         = 0

    def _build_message(self, agent, blackboard):
        own = blackboard.get("own_state")
        if not own:
            return None    # GatherState가 선행에서 own 결손을 막아줌

        follower_id = blackboard.get("follower_id", DEFAULT_FOLLOWER_ID)
        heading  = (own.get("yaw", 0.0) + self._hdg_off) % 360.0
        altitude = _alt_m(own) + self._alt_off

        msg = AircraftSetpoint()
        msg.aircraft_name    = _own_routing_name(own, follower_id)
        msg.guidance_mode    = "direct"
        msg.heading_deg      = float(heading)
        msg.altitude_m       = float(altitude)
        msg.target_speed_mps = self._speed

        agent.ros_bridge.node.get_logger().info(
            f"[LeaveFormation] 이탈 hdg={heading:.0f} alt={altitude:.0f} Vtgt={self._speed:.0f} "
            f"tick={self._ticks + 1}/{self._ticks_needed}")
        return msg

    def _interpret_publish(self, msg, agent, blackboard) -> Status:
        self._ticks += 1
        return Status.SUCCESS if self._ticks >= self._ticks_needed else Status.RUNNING
