"""
MUM-T BT (py_bt_ros) — Ground → wait for leader → takeoff → follow manned in formation.

UAV(F16_UAV)가 유인기(M_F16, 사람이 조이스틱으로 조종)를 편대 추종한다. 4 milestone:
  M0 HoldSetpoint  : 고정 setpoint 발행(체인 검증/나이브 이륙)
  M1 +GatherState  : own+leader 상태 인지
  M2 Wait→Takeoff  : 리더 이륙까지 대기 후 UAV 이륙
  M3 MaintainFormation : 오프셋 슬롯 + closure 로 편대 유지

★ 이 repo의 실제 인터페이스에 맞춰 스펙을 적응함 (repo = source of truth):
  - AircraftSetpoint = {aircraft_name, heading_deg, altitude_m, throttle_norm,
    target_speed_mps, launch_missile}. UE는 aircraft_name으로 pawn 라우팅 →
    **반드시 채움**. 속도는 throttle 개루프 대신 **target_speed_mps(UE 오토스로틀)**로
    (상승 중 실속 방지). BT는 '원하는 속도'만 결정하고, throttle 계산은 UE 제어기가 함.
    라우팅은 상태에서 받은 정확한 이름 우선.
  - 헤딩 = 나침반(0=북, 90=동). UE 월드 x=East, y=South → atan2(ΔEast,ΔNorth)=atan2(Δx,-Δy).
  - 고도(altitude_m)는 **UE Location.Z/100 (UE-Z, m)** 기준 — 상태 z(cm)/100과 동일.
    (스펙 §8.2의 ASL 가정과 반대. 오토파일럿이 UE-Z를 쓰도록 바뀜.)
  - 지면이 음수/맵마다 달라 **이륙·고도 판정은 '스폰(최초관측) 대비 상대'**로.
  - 상태 토픽 = {"message_type":..., "aircraft":[{...}]} 배치 JSON.
"""

import json
import math
import random

from std_msgs.msg import String
from custom_msgs.msg import AircraftSetpoint

from modules.base_bt_nodes import (
    BTNodeList, Status, Sequence, ReactiveSequence, Fallback, ReactiveFallback,
)
from modules.base_bt_nodes_ros import ConditionWithROSTopics, ActionWithROSTopic

# ── 노드 등록 ──────────────────────────────────────────────────────────────────
BTNodeList.CONDITION_NODES.extend(["GatherState"])
BTNodeList.ACTION_NODES.extend(
    ["HoldSetpoint", "WaitForLeaderTakeoff", "Takeoff", "MaintainFormation",
     "OrbitLeader", "RandomLeader"])

# ── 토픽/상수 ────────────────────────────────────────────────────────────────────
_STATE_TOPIC    = "/mumt/aircraft_states"
_SETPOINT_TOPIC = "/aircraft/setpoint"
CM_TO_M = 0.01

OWN_NAME    = "F16_UAV"   # 실제 UE pawn 이름(접미사 _C_N은 토큰매칭 허용)
LEADER_NAME = "M_F16"

# 이륙/상승 (전부 '스폰 대비 상대' + 오토스로틀 목표속도)
RUNWAY_HEADING_DEG       = 90.0
TAKEOFF_SPEED_MPS        = 260.0    # 이륙 중 리더에 덜 처지도록(리더 순항속도에 근접)
LEADER_AIRBORNE_CLIMB_M  = 80.0     # 리더가 스폰 대비 +이만큼 오르면 '이륙' 간주
UAV_AIRBORNE_CLIMB_M     = 120.0    # UAV가 스폰 대비 +이만큼 오르면 '이륙 완료'(편대 빨리 시작 → 리더 이탈 전)
TAKEOFF_CLIMB_TARGET_M   = 1000.0   # 이륙 단계 목표 상승고도(스폰 대비)

# 편대
# AFT_OFFSET_M: +면 리더 뒤, -면 리더 앞(전방 편대). 전방은 UAV가 리더를 '추월'해야 도달 →
# 리더가 최고속도면 추월 불가(동일 f16), 중간속도면 추월·유지 가능. 유지 자체는 후방과 동일.
AFT_OFFSET_M      = -100.0  # 리더 앞 100m (전방 편대)
LATERAL_OFFSET_M  = -40.0   # +면 우측, -면 좌측 윙맨 (부호로 좌/우 선택)
VERTICAL_OFFSET_M = 0.0
BLEND_RADIUS_M    = 400.0   # 이 거리 안에서 리더 헤딩으로 수렴(velocity match) — 약간 키워 weaving 완화
RENDEZVOUS_M      = 2500.0  # 이 거리 밖에선 '리더+여유'로 추격, 안에선 거리비례 보정
CATCHUP_MARGIN    = 50.0    # 먼거리 추격 시 리더보다 이만큼만 빠르게(절대속도 아님 → 느린 리더도 안 지나침)
KP_SPEED          = 0.05    # along-track 위치오차(m) → 목표속도 가감(m/s)
ALONG_SPD_MIN     = -60.0   # 앞질렀을 때 더 강하게 감속해 리더 뒤로 복귀
ALONG_SPD_MAX     = 50.0    # 뒤처지면 따라잡기(단 기체 최고속도에서 포화)
MIN_FORM_SPEED    = 70.0    # 실속 하한(기체 최저속도)만. 그 외엔 리더 속도를 그대로 추종 →
                            # 리더 속도가 변해도(빠르든 느리든) UAV가 매칭해 위치 유지
MIN_AGL_M         = 60.0    # 슬롯 고도 하한 = 스폰 + 이 값 (지면 유도 방지)


# ── 헬퍼 ────────────────────────────────────────────────────────────────────────
def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def unit_xy_to_heading(dx, dy) -> float:
    """UE x/y(동/남) 벡터 → 나침반 헤딩[0,360) (0=북,90=동). bearing=atan2(ΔEast,ΔNorth)."""
    return math.degrees(math.atan2(dx, -dy)) % 360.0


def heading_to_unit_xy(deg):
    """나침반 헤딩 → UE x/y 단위벡터 (unit_xy_to_heading의 역함수). (sinH, -cosH)."""
    h = math.radians(deg)
    return math.sin(h), -math.cos(h)


def _leader_axes(lyaw):
    """리더 전진/우측 단위벡터 (UE x=동, y=남). 전진=(sinH,-cosH), 우측=(cosH,sinH)."""
    fwd = heading_to_unit_xy(lyaw)
    h = math.radians(lyaw)
    return fwd, (math.cos(h), math.sin(h))


def shortest_heading_blend(h_from, h_to, w) -> float:
    """두 헤딩을 '짧은 쪽'으로 보간. w=0 → h_from, w=1 → h_to."""
    delta = ((h_to - h_from + 180.0) % 360.0) - 180.0
    return (h_from + w * delta) % 360.0


def _name_matches(full: str, key: str) -> bool:
    """토큰 경계 매칭: full==key 또는 full이 key로 시작+다음 글자 비영숫자.
       'F16_UAV'→'F16_UAV_C_2' O, 'F16_UAV2' X."""
    if not key:
        return False
    if full == key:
        return True
    if full.startswith(key):
        return not full[len(key):len(key) + 1].isalnum()
    return False


def _alt_m(state):       # 상태 z(cm) → UE-Z m
    return state.get("z", 0.0) * CM_TO_M


def _climb(blackboard, state):
    """현재 고도 − 스폰(최초관측) 고도."""
    base = (blackboard.get("init_alt") or {}).get(state.get("aircraft_name", ""), _alt_m(state))
    return _alt_m(state) - base


def _setpoint(aircraft_name, heading, altitude, target_speed=0.0, throttle=0.0):
    msg = AircraftSetpoint()
    msg.aircraft_name    = str(aircraft_name)
    msg.heading_deg      = float(heading)
    msg.altitude_m       = float(altitude)
    msg.throttle_norm    = clamp(float(throttle), 0.0, 1.0)
    msg.target_speed_mps = float(target_speed)
    msg.launch_missile   = False
    return msg


def _own_routing_name(own, fallback):
    return str((own.get("aircraft_name") if own else "") or fallback)


# ══════════════════════════════════════════════════════════════════════════════
# M0 — HoldSetpoint : 고정 setpoint (체인 검증 + 나이브 이륙)
# ══════════════════════════════════════════════════════════════════════════════
class HoldSetpoint(ActionWithROSTopic):
    """상태 구독 없이 고정 setpoint를 계속 발행하고 항상 RUNNING.
       aircraft_name=own_name(UE가 substring 매칭). 지상 시작이라 이륙+상승 테스트도 됨."""

    def __init__(self, name, agent, own_name=OWN_NAME,
                 heading_deg=RUNWAY_HEADING_DEG, altitude_m=3000.0,
                 target_speed_mps=TAKEOFF_SPEED_MPS):
        super().__init__(name, agent, (AircraftSetpoint, _SETPOINT_TOPIC))
        self._own = own_name
        self._hdg = float(heading_deg)
        self._alt = float(altitude_m)
        self._spd = float(target_speed_mps)

    def _build_message(self, agent, blackboard):
        agent.ros_bridge.node.get_logger().info(
            f"[HoldSetpoint] name={self._own} hdg={self._hdg:.0f} alt={self._alt:.0f} Vtgt={self._spd:.0f}")
        return _setpoint(self._own, self._hdg, self._alt, target_speed=self._spd)

    def _interpret_publish(self, msg, agent, blackboard) -> Status:
        return Status.RUNNING


# ══════════════════════════════════════════════════════════════════════════════
# OrbitLeader — 리더 UAV: 이륙 후 고도·속도 유지하며 계속 선회(오빗)
# ══════════════════════════════════════════════════════════════════════════════
class OrbitLeader(ActionWithROSTopic):
    """상태구독 없이 고정 고도·속도를 유지하되 heading을 매 틱 turn_rate만큼 증가시켜
       원을 그리며 비행. 처음 straight_time_s 동안은 직진(이륙/상승 안정화) 후 선회 시작.
       항상 RUNNING. (윙맨은 리더 yaw로 슬롯이 회전하므로 이 선회를 그대로 추종)"""

    def __init__(self, name, agent, own_name=OWN_NAME, altitude_m=1000.0,
                 target_speed_mps=200.0, start_heading_deg=RUNWAY_HEADING_DEG,
                 turn_rate_dps=5.0, straight_time_s=20.0, tick_rate_hz=10.0):
        super().__init__(name, agent, (AircraftSetpoint, _SETPOINT_TOPIC))
        self._own      = own_name
        self._alt      = float(altitude_m)
        self._spd      = float(target_speed_mps)
        self._hdg      = float(start_heading_deg)
        self._rate     = float(turn_rate_dps)      # 선회율 (deg/s, +면 우선회)
        self._straight = float(straight_time_s)    # 이 시간 동안 직진(이륙) 후 선회
        self._dt       = 1.0 / float(tick_rate_hz)
        self._t        = 0.0

    def _build_message(self, agent, blackboard):
        self._t += self._dt
        turning = self._t >= self._straight
        if turning:
            self._hdg = (self._hdg + self._rate * self._dt) % 360.0
        agent.ros_bridge.node.get_logger().info(
            f"[OrbitLeader] t={self._t:.0f}s {'선회' if turning else '직진(이륙)'} "
            f"hdg={self._hdg:.0f} alt={self._alt:.0f} Vtgt={self._spd:.0f} rate={self._rate:.0f}dps")
        return _setpoint(self._own, self._hdg, self._alt, target_speed=self._spd)

    def _interpret_publish(self, msg, agent, blackboard) -> Status:
        return Status.RUNNING


# ══════════════════════════════════════════════════════════════════════════════
# RandomLeader — 리더 UAV: 무작위 기동 비행
# ══════════════════════════════════════════════════════════════════════════════
class RandomLeader(ActionWithROSTopic):
    """상태구독 없이 무작위로 기동. straight_time_s 동안 직진 이륙/상승 후,
       change_interval_s 마다 새 목표를 랜덤으로 뽑음: heading=현재±max_turn_deg,
       altitude=[alt_min,alt_max], speed=[spd_min,spd_max]. heading은 max_rate_dps로
       부드럽게 슬루(급기동 방지). 항상 RUNNING. seed>0이면 재현 가능.
       (윙맨은 리더 상태를 읽어 이 무작위 기동을 편대로 추종)"""

    def __init__(self, name, agent, own_name=OWN_NAME,
                 start_heading_deg=RUNWAY_HEADING_DEG,
                 altitude_m=1000.0, target_speed_mps=200.0,
                 max_rate_dps=6.0, change_interval_s=8.0, max_turn_deg=120.0,
                 alt_min=800.0, alt_max=1300.0, spd_min=180.0, spd_max=250.0,
                 straight_time_s=20.0, seed=0, tick_rate_hz=10.0):
        super().__init__(name, agent, (AircraftSetpoint, _SETPOINT_TOPIC))
        self._own      = own_name
        self._hdg      = float(start_heading_deg)   # 명령 heading(슬루됨)
        self._tgt_hdg  = float(start_heading_deg)   # 무작위 목표 heading
        self._alt      = float(altitude_m)
        self._spd      = float(target_speed_mps)
        self._rate     = float(max_rate_dps)
        self._interval = float(change_interval_s)
        self._maxturn  = float(max_turn_deg)
        self._alt_rng  = (float(alt_min), float(alt_max))
        self._spd_rng  = (float(spd_min), float(spd_max))
        self._straight = float(straight_time_s)
        self._dt       = 1.0 / float(tick_rate_hz)
        self._t        = 0.0
        self._t_change = 0.0
        self._rng      = random.Random(int(seed)) if int(seed) else random.Random()

    def _build_message(self, agent, blackboard):
        self._t += self._dt
        if self._t >= self._straight:
            if self._t - self._t_change >= self._interval:      # 주기마다 새 무작위 목표
                self._t_change = self._t
                self._tgt_hdg = (self._hdg + self._rng.uniform(-self._maxturn, self._maxturn)) % 360.0
                self._alt     = self._rng.uniform(*self._alt_rng)
                self._spd     = self._rng.uniform(*self._spd_rng)
            d    = ((self._tgt_hdg - self._hdg + 180.0) % 360.0) - 180.0   # 최단각
            step = clamp(d, -self._rate * self._dt, self._rate * self._dt) # 슬루 제한
            self._hdg = (self._hdg + step) % 360.0
        agent.ros_bridge.node.get_logger().info(
            f"[RandomLeader] t={self._t:.0f}s hdg={self._hdg:.0f}→{self._tgt_hdg:.0f} "
            f"alt={self._alt:.0f} Vtgt={self._spd:.0f}")
        return _setpoint(self._own, self._hdg, self._alt, target_speed=self._spd)

    def _interpret_publish(self, msg, agent, blackboard) -> Status:
        return Status.RUNNING


# ══════════════════════════════════════════════════════════════════════════════
# M1 — GatherState : own + leader 상태 인지
# ══════════════════════════════════════════════════════════════════════════════
class GatherState(ConditionWithROSTopics):
    """상태 배치에서 own/leader를 찾아 blackboard에 기록. 둘 다 찾으면 SUCCESS.
       스폰 고도도 1회 기록(상대 이륙 판정용). 캐시가 최신 메시지를 유지하므로
       순간 끊김에도 마지막 상태로 True 유지(래칭 안전)."""

    def __init__(self, name, agent, own_name=OWN_NAME, leader_name=LEADER_NAME):
        super().__init__(name, agent, [(String, _STATE_TOPIC, "states")])
        self._own_name    = own_name or (getattr(agent, "agent_id", "") or "")
        self._leader_name = leader_name

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
        elif isinstance(payload, dict):
            aircraft = [payload]
        else:
            return False

        blackboard["all_states"] = {a.get("aircraft_name", ""): a for a in aircraft}
        init = blackboard.setdefault("init_alt", {})
        for a in aircraft:
            init.setdefault(a.get("aircraft_name", ""), _alt_m(a))

        own    = next((a for a in aircraft if _name_matches(a.get("aircraft_name", ""), self._own_name)), None)
        leader = next((a for a in aircraft if _name_matches(a.get("aircraft_name", ""), self._leader_name)), None)
        blackboard["own_state"]    = own
        blackboard["leader_state"] = leader
        if own is not None and leader is not None:
            agent.ros_bridge.node.get_logger().info(
                f"[GatherState] own={own.get('aircraft_name')} alt={_alt_m(own):.0f}(+{_climb(blackboard,own):.0f}) "
                f"yaw={own.get('yaw',0):.0f} spd={own.get('speed_mps',0):.0f} | "
                f"leader={leader.get('aircraft_name')} alt={_alt_m(leader):.0f}(+{_climb(blackboard,leader):.0f}) "
                f"yaw={leader.get('yaw',0):.0f} spd={leader.get('speed_mps',0):.0f}")
            return True
        return False


# ══════════════════════════════════════════════════════════════════════════════
# M2 — WaitForLeaderTakeoff / Takeoff
# ══════════════════════════════════════════════════════════════════════════════
class WaitForLeaderTakeoff(ActionWithROSTopic):
    """리더가 스폰 대비 leader_airborne_climb_m 이상 오를 때까지 지상 대기(idle setpoint).
       오르면 SUCCESS. 대기 중엔 throttle 0 으로 정지 유지."""

    def __init__(self, name, agent, leader_airborne_climb_m=LEADER_AIRBORNE_CLIMB_M,
                 own_name=OWN_NAME):
        super().__init__(name, agent, (AircraftSetpoint, _SETPOINT_TOPIC))
        self._thresh = float(leader_airborne_climb_m)
        self._own_name = own_name

    def _build_message(self, agent, blackboard):
        own = blackboard.get("own_state")
        hdg = own.get("yaw", RUNWAY_HEADING_DEG) if own else RUNWAY_HEADING_DEG
        alt = _alt_m(own) if own else 0.0
        # idle: throttle 0, target_speed 0 → 추력 0 으로 지상 정지
        return _setpoint(_own_routing_name(own, self._own_name), hdg, alt,
                         target_speed=0.0, throttle=0.0)

    def _interpret_publish(self, msg, agent, blackboard) -> Status:
        leader = blackboard.get("leader_state")
        if not leader:
            return Status.RUNNING
        climbed = _climb(blackboard, leader)
        agent.ros_bridge.node.get_logger().info(
            f"[WaitForLeaderTakeoff] 리더 상승량={climbed:.0f}m / 필요={self._thresh:.0f}m")
        return Status.SUCCESS if climbed >= self._thresh else Status.RUNNING


class Takeoff(ActionWithROSTopic):
    """UAV 이륙: 활주로 헤딩 유지 + 목표속도(오토스로틀) + 스폰 대비 상승.
       own이 스폰 대비 uav_airborne_climb_m 이상 오르면 SUCCESS."""

    def __init__(self, name, agent,
                 runway_heading_deg=RUNWAY_HEADING_DEG,
                 climb_target_m=TAKEOFF_CLIMB_TARGET_M,
                 uav_airborne_climb_m=UAV_AIRBORNE_CLIMB_M,
                 own_name=OWN_NAME):
        super().__init__(name, agent, (AircraftSetpoint, _SETPOINT_TOPIC))
        self._hdg = float(runway_heading_deg)
        self._climb_target = float(climb_target_m)
        self._airborne = float(uav_airborne_climb_m)
        self._own_name = own_name

    def _build_message(self, agent, blackboard):
        own = blackboard.get("own_state")
        if not own:
            return None
        base = (blackboard.get("init_alt") or {}).get(own.get("aircraft_name", ""), _alt_m(own))
        target_alt = base + self._climb_target
        agent.ros_bridge.node.get_logger().info(
            f"[Takeoff] alt={_alt_m(own):.0f}(+{_climb(blackboard,own):.0f}/{self._airborne:.0f}) "
            f"spd={own.get('speed_mps',0):.0f} → climb to {target_alt:.0f}")
        return _setpoint(_own_routing_name(own, self._own_name),
                         self._hdg, target_alt, target_speed=TAKEOFF_SPEED_MPS)

    def _interpret_publish(self, msg, agent, blackboard) -> Status:
        own = blackboard.get("own_state")
        if not own:
            return Status.RUNNING
        return Status.SUCCESS if _climb(blackboard, own) >= self._airborne else Status.RUNNING


# ══════════════════════════════════════════════════════════════════════════════
# M3 — MaintainFormation : 오프셋 슬롯 + closure (우리 설계)
# ══════════════════════════════════════════════════════════════════════════════
class MaintainFormation(ActionWithROSTopic):
    """리더 앞/뒤·옆 고정 슬롯을 추종(aft_offset 부호로 앞/뒤). 멀면 슬롯방위 추적, 가까우면 리더 헤딩으로 수렴(weaving 방지).
       목표속도 = 리더속도 + along-track closure (오토스로틀이 유지). 항상 RUNNING.
       래칭 안전: own/leader 순간 결손 시 마지막 setpoint 재발행 + RUNNING (FAILURE 금지 →
       부모 Sequence가 Wait/Takeoff로 되돌아가지 않음)."""

    def __init__(self, name, agent,
                 aft_offset_m=AFT_OFFSET_M, lateral_offset_m=LATERAL_OFFSET_M,
                 vertical_offset_m=VERTICAL_OFFSET_M, blend_radius_m=BLEND_RADIUS_M,
                 kp_speed=KP_SPEED, own_name=OWN_NAME):
        super().__init__(name, agent, (AircraftSetpoint, _SETPOINT_TOPIC))
        self._aft   = float(aft_offset_m)
        self._lat   = float(lateral_offset_m)
        self._voff  = float(vertical_offset_m)
        self._blend = float(blend_radius_m)
        self._kp    = float(kp_speed)
        self._own_name = own_name
        self._last_msg = None

    def _build_message(self, agent, blackboard):
        own    = blackboard.get("own_state")
        leader = blackboard.get("leader_state")
        if not own or not leader:
            agent.ros_bridge.node.get_logger().warn(
                "[MaintainFormation] own/leader 결손 → 마지막 setpoint 유지")
            return self._last_msg          # None이면 FAILURE지만 GatherState가 먼저 막아줌

        ox, oy = own.get("x", 0.0) * CM_TO_M, own.get("y", 0.0) * CM_TO_M
        lx, ly = leader.get("x", 0.0) * CM_TO_M, leader.get("y", 0.0) * CM_TO_M
        lyaw   = leader.get("yaw", 0.0)
        lspd   = leader.get("speed_mps", 0.0)
        (fx, fy), (rx, ry) = _leader_axes(lyaw)

        # 슬롯(월드, m)
        sx = lx - self._aft * fx + self._lat * rx
        sy = ly - self._aft * fy + self._lat * ry
        slot_alt = _alt_m(leader) + self._voff

        dx, dy = sx - ox, sy - oy
        dist   = math.hypot(dx, dy)

        # 헤딩: 멀면 슬롯방위, 가까우면 리더 헤딩으로 블렌드
        bearing = unit_xy_to_heading(dx, dy)
        w = clamp(dist / self._blend, 0.0, 1.0)
        heading = shortest_heading_blend(lyaw, bearing, w)

        # 고도: 슬롯고도, 단 스폰+MIN_AGL 하한
        base = (blackboard.get("init_alt") or {}).get(own.get("aircraft_name", ""), _alt_m(own))
        altitude = max(slot_alt, base + MIN_AGL_M)

        # 속도: 멀면 따라잡기(리더+50), 가까우면 리더+along closure (하한 MIN_FORM_SPEED)
        dist_to_leader = math.hypot(lx - ox, ly - oy)
        if dist_to_leader > RENDEZVOUS_M:
            speed = lspd + CATCHUP_MARGIN        # 멀면 '리더+여유'로 추격 (리더 속도에 상대)
        else:
            along = dx * fx + dy * fy            # +면 슬롯 앞(뒤처짐)→가속, -면 앞질러→감속
            speed = lspd + clamp(self._kp * along, ALONG_SPD_MIN, ALONG_SPD_MAX)
        speed = max(MIN_FORM_SPEED, speed)        # 실속 하한만 — 나머진 리더 속도를 그대로 추종

        agent.ros_bridge.node.get_logger().info(
            f"[MaintainFormation] 슬롯거리={dist:.0f}m 리더거리={dist_to_leader:.0f}m "
            f"hdg={heading:.0f}° alt={altitude:.0f} Vtgt={speed:.0f} | 리더(spd={lspd:.0f} alt={_alt_m(leader):.0f})")
        self._last_msg = _setpoint(_own_routing_name(own, self._own_name),
                                   heading, altitude, target_speed=speed)
        return self._last_msg

    def _interpret_publish(self, msg, agent, blackboard) -> Status:
        return Status.RUNNING       # 편대는 끝나지 않음 (항상 RUNNING, FAILURE 금지)
