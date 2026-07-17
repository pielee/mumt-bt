"""
MUMT formation-flight BT nodes — manned leader + N UAV wingmen.

친구 로직(한 프로세스에서 UAV 여러 대 + UAV별 상태 분리)을 현재 MUMT_Sim
인터페이스에 맞게 재작성:
  - /aircraft/setpoint = custom_msgs/AircraftSetpoint (aircraft_name 라우팅 + autothrottle target_speed)
  - 헤딩/슬롯은 나침반 프레임(UE x=East, y=South) — 오토파일럿 yaw와 동일 규약
  - 이륙 판정은 '스폰(최초 관측) 고도 대비 상대 상승량' (맵 무관, UE-Z 지면이 음수여도 OK)
  - throttle 대신 '목표속도(target_speed)' 발행 → UE 오토스로틀이 속도 유지(상승 중 실속 방지)
  - UAV별 상태는 MUMTWorldState로 분리 → 한 트리에서 여러 UAV가 서로 안 섞임

UAV 이름: XML uav_name 속성 우선(한 프로세스 다중기), 없으면 --ns, 없으면 UAV_NAME.
이름은 실제 UE pawn 이름과 일치해야 함 (예: F16_UAV, F16_UAV2 / 유인기 M_F16).
"""

import json
import math

from std_msgs.msg import String
from custom_msgs.msg import AircraftSetpoint

from modules.base_bt_nodes import (
    BTNodeList, Status,
    Sequence, ReactiveSequence, Fallback, ReactiveFallback, Parallel,  # 제어노드(XML에서 참조)
)
from modules.base_bt_nodes_ros import ConditionWithROSTopics, ActionWithROSTopic
from scenarios.mumt.controlv2_seq import get_seq

# ── 노드 등록 ──────────────────────────────────────────────────────────────────

BTNodeList.CONDITION_NODES.extend(["MannedAircraftTookOff", "UAVReachedAltitude"])
BTNodeList.ACTION_NODES.extend(["UAVTakeOff", "FormationFlight"])

# ── 상수 ────────────────────────────────────────────────────────────────────────

_STATE_TOPIC    = "/mumt/aircraft_states"
_SETPOINT_TOPIC = "/aircraft/setpoint"

UAV_NAME               = "F16_UAV"    # 기본/폴백 (XML uav_name 또는 --ns로 덮어씀)
MANNED_NAME            = "M_F16"      # 유인기 pawn 이름 (다른 UAV와 구분)
MANNED_TAKEOFF_CLIMB_M = 100.0        # 유인기가 스폰 대비 +이만큼 오르면 '이륙'
UAV_TAKEOFF_CLIMB_M    = 200.0        # UAV가 스폰 대비 +이만큼까지 상승(이륙 목표)
TAKEOFF_SPEED_MPS      = 420.0        # 이륙/상승 목표 대기속도 (UE 오토스로틀이 유지)
RENDEZVOUS_SPEED_MPS   = 260.0        # 랑데부 목표속도 하한
RENDEZVOUS_MARGIN_MPS  = 50.0         # 랑데부는 '리더속도+이만큼'으로 → 항상 리더보다 빨라 따라잡음
TAKEOFF_HEADING_DEG    = 90.0
ALT_TOLERANCE_M        = 50.0
SPEED_FLOOR_MPS        = 120.0        # 이 속도 미만이면 상승 멈추고 수평유지(가속)로 실속 회복(백업)
TRAIL_M                = 100.0
LATERAL_M              = 150.0
ALTITUDE_OFFSET_M      = 30.0
RENDEZVOUS_DIST_M      = 2000.0
# 편대유지: 목표속도 = 리더속도 + along-track 위치오차 보정
K_ALONG_SPD            = 0.05         # 위치오차(m) → 목표속도 가감(m/s)
ALONG_SPD_MIN          = -15.0
ALONG_SPD_MAX          = 20.0

# ── 헬퍼 함수 ────────────────────────────────────────────────────────────────────

def _parse_state(raw: dict) -> dict:
    return {
        "aircraft_name": raw.get("aircraft_name", ""),
        "x_m":           float(raw.get("x", 0.0)) / 100.0,
        "y_m":           float(raw.get("y", 0.0)) / 100.0,
        "altitude_m":    float(raw.get("z", 0.0)) / 100.0,   # UE world Z (cm→m)
        "heading_deg":   float(raw.get("yaw", 0.0)),         # 나침반(0=북)
        "speed_mps":     float(raw.get("speed_mps", 0.0)),
    }


def _ns_name(agent) -> str:
    ns = (getattr(agent, "agent_id", "") or "").strip("/")
    return "" if ns in ("", "no_id_agent") else ns


def _resolve_key(agent, explicit) -> str:
    """UAV 식별 키: XML uav_name 명시값 > --ns namespace > UAV_NAME 기본값."""
    return explicit or _ns_name(agent) or UAV_NAME


def _name_matches(full: str, key: str) -> bool:
    """토큰 경계 매칭: full == key 거나, full이 key로 시작하고 그 다음 글자가
    영숫자가 아닐 때(예: 'F16_UAV' → 'F16_UAV_C_2' O, 'F16_UAV2' X).
    'F16_UAV'가 'F16_UAV2'의 접두사여도 둘을 정확히 구분한다."""
    if not key:
        return False
    if full == key:
        return True
    if full.startswith(key):
        nxt = full[len(key):len(key) + 1]
        return not nxt.isalnum()
    return False


def _heading_to(fx, fy, tx, ty) -> float:
    """UE x/y(= 동/남)에서 나침반 헤딩(0=북, 90=동). +X=East, +Y=South(=-North)
       → bearing = atan2(ΔEast, ΔNorth). 오토파일럿이 추종하는 yaw와 동일 규약."""
    d_east  = tx - fx
    d_north = -(ty - fy)
    return math.degrees(math.atan2(d_east, d_north)) % 360.0


def _dist(x1, y1, x2, y2) -> float:
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def _leader_axes(lyaw):
    """리더 나침반 헤딩 → UE x/y(동/남) 전진/우측 단위벡터.
       전진=(sinH,-cosH), 우측(H+90)=(cosH,sinH)."""
    h = math.radians(lyaw)
    return (math.sin(h), -math.cos(h)), (math.cos(h), math.sin(h))


def _formation_target(lx, ly, la, lyaw, sign):
    """ARROW 슬롯(리더 뒤 TRAIL, 옆 LATERAL). sign=-1 좌측, +1 우측."""
    (fx, fy), (rx, ry) = _leader_axes(lyaw)
    tx = lx - TRAIL_M * fx + sign * LATERAL_M * rx
    ty = ly - TRAIL_M * fy + sign * LATERAL_M * ry
    ta = max(la - ALTITUDE_OFFSET_M, 0.0)
    return tx, ty, ta


def _station_speed(leader_speed, along_gap):
    """편대유지 목표속도: 리더 속도 + along-track 위치오차 보정.
       along_gap>0 = 슬롯이 앞(=뒤처짐) → 빠르게, <0 = 앞섬 → 느리게."""
    spd_bias = max(ALONG_SPD_MIN, min(ALONG_SPD_MAX, K_ALONG_SPD * along_gap))
    return max(0.0, leader_speed + spd_bias)


def _setpoint_msg(aircraft_name, heading, altitude,
                  throttle=0.0, target_speed=0.0) -> AircraftSetpoint:
    """target_speed>0 이면 UE 오토스로틀이 그 속도 유지(throttle 무시).
       <=0 이면 throttle(개루프) 사용 (하위호환)."""
    msg = AircraftSetpoint()
    msg.aircraft_name    = str(aircraft_name)
    msg.heading_deg      = float(heading)
    msg.altitude_m       = float(altitude)
    msg.throttle_norm    = max(0.0, min(1.0, float(throttle)))
    msg.target_speed_mps = float(target_speed)
    msg.launch_missile   = False
    return msg


# ── 월드 상태 (UAV별 인스턴스) ───────────────────────────────────────────────────

class MUMTWorldState:
    """/mumt/aircraft_states를 구독해 '내 UAV'와 유인기 상태를 유지(UAV별로 분리).
      내 UAV : 키 정확매칭 우선, 없으면 유일 substring(접미사 _C_N 허용)
      유인기 : MANNED_NAME 매칭 (다른 UAV는 무시)
      스폰(최초 관측) 고도도 1회 기록 → 상대 이륙 판정용."""

    def __init__(self, agent, uav_name: str):
        self._uav_name    = uav_name
        self._manned      = {}
        self._uav         = {}
        self._manned_init = None
        self._uav_init    = None
        agent.ros_bridge.node.create_subscription(String, _STATE_TOPIC, self._cb, 1)

    def _cb(self, msg: String):
        try:
            aircraft = json.loads(msg.data).get("aircraft", [])
        except (json.JSONDecodeError, AttributeError):
            return

        own = next((a for a in aircraft
                    if _name_matches(a.get("aircraft_name", ""), self._uav_name)), None)
        if own is not None:
            self._uav = _parse_state(own)
            if self._uav_init is None:
                self._uav_init = self._uav["altitude_m"]

        manned = next((a for a in aircraft
                       if _name_matches(a.get("aircraft_name", ""), MANNED_NAME)), None)
        if manned is not None:
            self._manned = _parse_state(manned)
            if self._manned_init is None:
                self._manned_init = self._manned["altitude_m"]

    def manned(self):      return dict(self._manned)
    def uav(self):         return dict(self._uav)
    def manned_init(self): return self._manned_init
    def uav_init(self):    return self._uav_init


def _get_world_state(agent, blackboard, uav_name: str) -> MUMTWorldState:
    key = f"_mumt_ws_{uav_name}"
    ws = blackboard.get(key)
    if ws is None:
        ws = MUMTWorldState(agent, uav_name)
        blackboard[key] = ws
    return ws


def _own_name(ws: MUMTWorldState, fallback: str) -> str:
    """setpoint aircraft_name: 상태에서 받은 정확한 pawn 이름 우선, 없으면 키."""
    uav = ws.uav()
    return str((uav.get("aircraft_name") if uav else "") or fallback)


def _uav_target_alt(ws: MUMTWorldState) -> float:
    """UAV 이륙 목표 절대고도(UE-Z) = 스폰 고도 + UAV_TAKEOFF_CLIMB_M."""
    base = ws.uav_init()
    if base is None:
        u = ws.uav()
        base = u.get("altitude_m", 0.0) if u else 0.0
    return base + UAV_TAKEOFF_CLIMB_M


# ──────────────────────────────────────────────────────────────────────────────
# 조건 노드
# ──────────────────────────────────────────────────────────────────────────────

class MannedAircraftTookOff(ConditionWithROSTopics):
    """유인기가 스폰 대비 MANNED_TAKEOFF_CLIMB_M 이상 상승하면 SUCCESS (이륙 감지)."""

    def __init__(self, name, agent, uav_name=None):
        # (super가 구독을 만들어 run()의 cache 가드를 통과시킴 — 실제 판정은 MUMTWorldState 사용)
        super().__init__(name, agent, [(String, _STATE_TOPIC, "aircraft_states")])
        self._uav_name = _resolve_key(agent, uav_name)

    def _predicate(self, agent, blackboard) -> bool:
        ws = _get_world_state(agent, blackboard, self._uav_name)
        manned = ws.manned()
        if not manned or ws.manned_init() is None:
            return False
        climbed = manned["altitude_m"] - ws.manned_init()
        agent.ros_bridge.node.get_logger().info(
            f"[MannedAircraftTookOff:{self._uav_name}] 유인기 상승량={climbed:.0f}m / 필요={MANNED_TAKEOFF_CLIMB_M:.0f}m")
        return climbed >= MANNED_TAKEOFF_CLIMB_M


class UAVReachedAltitude(ConditionWithROSTopics):
    """무인기가 이륙 목표고도(스폰+UAV_TAKEOFF_CLIMB_M)−허용오차 이상이면 SUCCESS."""

    def __init__(self, name, agent, uav_name=None):
        super().__init__(name, agent, [(String, _STATE_TOPIC, "aircraft_states")])
        self._uav_name = _resolve_key(agent, uav_name)

    def _predicate(self, agent, blackboard) -> bool:
        ws  = _get_world_state(agent, blackboard, self._uav_name)
        uav = ws.uav()
        if not uav:
            return False
        target = _uav_target_alt(ws)
        agent.ros_bridge.node.get_logger().info(
            f"[UAVReachedAltitude:{self._uav_name}] 현재 alt={uav['altitude_m']:.0f}m "
            f"spd={uav['speed_mps']:.0f}m/s / 목표 alt={target:.0f}m")
        return uav["altitude_m"] >= target - ALT_TOLERANCE_M


# ──────────────────────────────────────────────────────────────────────────────
# 액션 노드
# ──────────────────────────────────────────────────────────────────────────────

class UAVTakeOff(ActionWithROSTopic):
    """스폰+UAV_TAKEOFF_CLIMB_M 까지 상승. 도달 시 SUCCESS, 미달 시 RUNNING."""

    def __init__(self, name, agent, target_heading_deg=TAKEOFF_HEADING_DEG, uav_name=None):
        super().__init__(name, agent, (AircraftSetpoint, _SETPOINT_TOPIC))
        self._target_heading = target_heading_deg
        self._uav_name       = _resolve_key(agent, uav_name)

    def _build_message(self, agent, blackboard):
        ws     = _get_world_state(agent, blackboard, self._uav_name)
        uav    = ws.uav()
        spd    = uav.get("speed_mps", 0.0) if uav else 0.0
        cur    = uav.get("altitude_m", 0.0) if uav else 0.0
        target = _uav_target_alt(ws)
        # 속도 보호: 너무 느리면 상승 멈추고 현재 고도 유지(오토스로틀 풀파워로 가속)
        cmd_alt = cur if spd < SPEED_FLOOR_MPS else target
        mode    = "수평유지(가속)" if spd < SPEED_FLOOR_MPS else f"상승 {target:.0f}m"
        agent.ros_bridge.node.get_logger().info(
            f"[UAVTakeOff:{self._uav_name}] 현재 alt={cur:.0f}m spd={spd:.0f}m/s → {mode} (Vtgt={TAKEOFF_SPEED_MPS:.0f})")
        return _setpoint_msg(_own_name(ws, self._uav_name),
                             self._target_heading, cmd_alt, target_speed=TAKEOFF_SPEED_MPS)

    def _interpret_publish(self, msg, agent, blackboard) -> Status:
        ws  = _get_world_state(agent, blackboard, self._uav_name)
        uav = ws.uav()
        if not uav:                       # 상태 못 받으면(이름 매칭 실패 등) 완료로 오판 금지
            return Status.RUNNING
        target = _uav_target_alt(ws)
        if uav["altitude_m"] >= target - ALT_TOLERANCE_M:
            agent.ros_bridge.node.get_logger().info(
                f"[UAVTakeOff:{self._uav_name}] 이륙 완료 {uav['altitude_m']:.0f}m (목표 {target:.0f}m 이상)")
            return Status.SUCCESS
        return Status.RUNNING


class FormationFlight(ActionWithROSTopic):
    """
    ARROW 편대 슬롯 지정 — Phase 4: 유도 계산(랑데부/슬롯 추종)은 더 이상 BT가 하지
    않는다. UE FormationGuidance가 유인기 pawn을 직독해 60Hz로 랑데부·슬롯·closure를
    계산하므로(구 Phase1 랑데부/Phase2 전환 로직을 REJOIN이 대체), BT는
    guidance_mode="formation" + 리더/슬롯 지정만 발행한다. 항상 RUNNING.
    ports는 그대로 유지: lateral_sign(-1=좌/+1=우 윙맨), uav_name.
    """

    def __init__(self, name, agent, lateral_sign=-1.0, uav_name=None):
        super().__init__(name, agent, (AircraftSetpoint, _SETPOINT_TOPIC))
        self._sign     = lateral_sign
        self._uav_name = _resolve_key(agent, uav_name)
        self._last_msg = None

    def _build_message(self, agent, blackboard):
        ws     = _get_world_state(agent, blackboard, self._uav_name)
        manned = ws.manned()
        if not manned:
            # 유인기 상태가 잠깐 비어도 마지막 setpoint 를 재발행 = 같은 sequence heartbeat 유지
            # (bridge 가 command_timestamp 를 매 packet 새로 스탬프하므로 watchdog 은 살아있음).
            agent.ros_bridge.node.get_logger().warn(
                f"[FormationFlight:{self._uav_name}] 유인기 상태 없음 → 마지막 setpoint 유지")
            return self._last_msg

        msg = AircraftSetpoint()
        msg.aircraft_name = _own_name(ws, self._uav_name)
        msg.guidance_mode = "formation"
        msg.leader_name   = str(manned.get("aircraft_name") or MANNED_NAME)
        # ARROW 슬롯(리더 트랙 프레임): 뒤(TRAIL_M)·좌/우(LATERAL_M×sign)·아래(ALTITUDE_OFFSET_M)
        msg.slot_front_m  = -TRAIL_M
        msg.slot_right_m  = LATERAL_M * self._sign
        msg.slot_up_m     = -ALTITUDE_OFFSET_M

        # ── ControlV2 운용 편대 (Phase I-A): 매 tick control_mode="formation" heartbeat 발행 ──
        # guidance_mode="formation" 은 그대로 둔다 — ControlV2 가 명령을 거부/미소유하는 프레임엔
        # RouteControlV2 가 구형 편대 writer 로 되돌리므로(안전 baseline), 여기서 direct 로 두면
        # 그 fallback 이 heading0/alt0 급강하가 된다. ControlV2 가 소유하는 프레임엔 구형 경로가
        # skip 되어 둘이 동시에 돌지 않는다. slot/leader 가 그대로면 sequence 는 유지(heartbeat).
        msg.control_mode      = "formation"
        msg.command_sequence  = get_seq(msg.aircraft_name).sequence_for(
            "formation", msg.leader_name, (msg.slot_front_m, msg.slot_right_m, msg.slot_up_m))
        msg.command_timestamp = 0.0   # bridge 가 CLOCK_MONOTONIC 으로 스탬프

        agent.ros_bridge.node.get_logger().info(
            f"[FormationFlight:{self._uav_name}] leader={msg.leader_name} slot=(F{msg.slot_front_m:.0f} "
            f"R{msg.slot_right_m:.0f} U{msg.slot_up_m:.0f}) | ControlV2 formation seq={msg.command_sequence}")
        self._last_msg = msg
        return msg

    def _interpret_publish(self, msg, agent, blackboard) -> Status:
        return Status.RUNNING
