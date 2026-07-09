"""
MUM-T 유인기 편대 — BT는 ModeManager, 유도 계산은 UE 인엔진 60Hz.

Phase 4 개편: 슬롯·ω×r·벡터필드·roll_ff 계산이 전부 UE(FormationGuidance.h)로
이관됐다. UE가 리더 pawn을 같은 월드에서 직독(지연 0)하므로, 구 BT 유도의
왕복지연(~100-200ms)·선회율 필터 지연(τ=2s)이 사라진다.

BT의 역할 (이 노드):
  TAKEOFF  : 스폰 전방+상방 direct 명령(heading/alt/speed) → 내루프가 이륙.
             스폰 대비 takeoff_climb_m 상승 시 FORMATION 전환.
  FORMATION: guidance_mode="formation" + 리더/슬롯 지정만 발행 — 숫자 계산 없음.
             항상 RUNNING. own 결손 시 마지막 명령 래칭.

검증: JSBSim F-16 2기 폐루프(V1, 2026-07-09) — 직선 3.3m / 3°/s@220 4.5m /
5°/s@170+400m상승 정착 12.5m (mean 슬롯오차). 전환 과도 ≤136m, 20~30s 회복.
"""

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


class FormationGuidance(ActionWithROSTopic):
    """자체 이륙(direct) → 편대 모드 지정(formation). 유도 숫자는 UE가 60Hz 계산.

    슬롯 오프셋(리더 트랙 프레임): front_m(+앞/−뒤), right_m(+우/−좌), up_m(+위).
    기본 (−80, +100, 0) = 리더 뒤 80m·우측 100m."""

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
        self._to_fwd   = float(takeoff_forward_m)   # (미사용 잔존 파라미터 — XML 호환)
        self._to_up    = float(takeoff_up_m)
        self._to_spd   = float(takeoff_speed_mps)
        self._to_climb = float(takeoff_climb_m)
        self._min_agl  = float(min_agl_m)
        self._spawn_alt = None           # own 최초관측 고도(UE-Z m) — 상대 판정·고도가드 기준
        self._airborne  = False
        self._last_msg  = None

    def _build_message(self, agent, blackboard):
        own = blackboard.get("own_state")
        if not own:
            return self._last_msg        # 결손 → 래칭 (GatherState가 선행 차단)

        oalt = _alt_m(own)
        if self._spawn_alt is None:
            self._spawn_alt = oalt

        climb_m = oalt - self._spawn_alt
        if not self._airborne and climb_m >= self._to_climb:
            self._airborne = True
            agent.ros_bridge.node.get_logger().info(
                f"[FormationGuidance] 이륙 완료(+{climb_m:.0f}m) → 편대 모드 전환 (유도=UE 60Hz)")

        msg = AircraftSetpoint()
        msg.aircraft_name = _own_routing_name(own, self._own_name)

        if not self._airborne:
            # ── TAKEOFF: direct 명령 (내루프가 이륙) ──
            msg.heading_deg      = float(self._rwy_hdg % 360.0)
            msg.altitude_m       = float(self._spawn_alt + self._to_up)
            msg.target_speed_mps = float(self._to_spd)
            agent.ros_bridge.node.get_logger().info(
                f"[FormationGuidance] 이륙 상승 +{climb_m:.0f}/{self._to_climb:.0f}m")
        else:
            # ── FORMATION: 모드 지정만 — 슬롯 계산은 UE FormationGuidance.h ──
            msg.guidance_mode = "formation"
            msg.leader_name   = self._leader
            msg.slot_front_m  = self._front
            msg.slot_right_m  = self._right
            msg.slot_up_m     = self._up
            msg.min_speed_mps = self._spd_min
            msg.max_speed_mps = self._spd_max
            msg.min_alt_m     = float(self._spawn_alt + self._min_agl)   # 고도 하한 가드 이관

        self._last_msg = msg
        return msg

    def _interpret_publish(self, msg, agent, blackboard) -> Status:
        return Status.RUNNING
