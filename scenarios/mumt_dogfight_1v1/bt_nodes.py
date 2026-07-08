"""
MUM-T 1v1 dogfight BT — 이륙 후 적기 교전 (pure pursuit + WEZ 방아쇠).

UAV1(Enemy) vs UAV2(FriendlyUAV). 팀은 **에디터에서** UHealthComponent.Team으로
지정하고, BT는 5006 상태의 Phase 3 확장 필드(team/destroyed/hp/missile_count)로
적을 식별한다. 명중 판정(WEZ 원추)은 UE UWeaponComponent가 하므로, BT는
"대략 겨눠졌을 때" 방아쇠만 당긴다 (기총 방아쇠 5° > WEZ 원추 2.5°).

발사 프로토콜 (Phase 3, AircraftSetpoint):
  gun_firing      : 상태(레벨) — true인 동안 UE가 연사
  missile_fire_id : 이벤트(에지) — 값이 바뀔 때마다 1발. 1부터 시작 (0=미발사 초기값)

기존 노드 재사용 (scenarios.mumt — 파일 수정 없이 import만):
  Takeoff     : 스폰 대비 상대 상승 판정 이륙
  OrbitLeader : 적 전멸 후 선회 대기 (loiter)
"""

import json
import math
import time

from std_msgs.msg import String
from custom_msgs.msg import AircraftSetpoint

from modules.base_bt_nodes import (
    BTNodeList, Status, Node, Sequence, ReactiveSequence, Fallback, ReactiveFallback,
)
from modules.base_bt_nodes_ros import ConditionWithROSTopics, ActionWithROSTopic

# scenarios.mumt 노드/헬퍼 재사용 (import 부수효과로 Takeoff/OrbitLeader 등도 등록됨)
from scenarios.mumt.bt_nodes import (
    Takeoff, OrbitLeader, HoldSetpoint,
    unit_xy_to_heading, _name_matches, _alt_m, _setpoint, _own_routing_name,
    CM_TO_M, _STATE_TOPIC, _SETPOINT_TOPIC,
)

# ── 노드 등록 ──────────────────────────────────────────────────────────────────
BTNodeList.CONDITION_NODES.extend(["GatherCombatState", "ConditionEnemyAlive"])
BTNodeList.ACTION_NODES.extend(["EngageTarget"])

# 팀 적대 관계: 아군 진영(manned + friendly_uav) vs 적 진영(enemy).
# "내 팀과 다른 팀 전부"로 판정하면 아군 무인기가 유인기를 적으로 오인한다.
_HOSTILE = {
    "manned":       ("enemy",),
    "friendly_uav": ("enemy",),
    "enemy":        ("manned", "friendly_uav"),
}


# ══════════════════════════════════════════════════════════════════════════════
# GatherCombatState — own + 적 팀 생존 기체 목록 인지
# ══════════════════════════════════════════════════════════════════════════════
class GatherCombatState(ConditionWithROSTopics):
    """상태 배치에서 own과 '적 팀 생존 기체 목록'을 blackboard에 기록. own을 찾으면 SUCCESS.
       (scenarios.mumt.GatherState의 전투 확장 — leader 대신 enemies를 채운다)

       blackboard 출력:
         own_state : own 기체 dict (없으면 None)
         all_states: {aircraft_name: dict}
         init_alt  : 기체별 스폰(최초관측) 고도 래칭 — Takeoff의 상대 판정용
         own_team  : own의 team 문자열 ("enemy"/"friendly_uav"/"manned", 컴포넌트 없으면 None)
         enemies   : team이 존재하고 own_team과 다르며 destroyed가 아닌 기체 목록"""

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

        blackboard["all_states"] = {a.get("aircraft_name", ""): a for a in aircraft}
        init = blackboard.setdefault("init_alt", {})
        for a in aircraft:
            init.setdefault(a.get("aircraft_name", ""), _alt_m(a))

        own = next((a for a in aircraft
                    if _name_matches(a.get("aircraft_name", ""), self._own_name)), None)
        own_team = (own or {}).get("team")
        blackboard["own_state"] = own
        blackboard["own_team"]  = own_team
        blackboard["enemies"]   = [
            a for a in aircraft
            if a.get("team") in _HOSTILE.get(own_team, ())
            and not a.get("destroyed", False) and a is not own
        ]

        if own is None:
            return False
        agent.ros_bridge.node.get_logger().info(
            f"[GatherCombatState] own={own.get('aircraft_name')} team={own_team} "
            f"hp={own.get('hp', '?')} msl={own.get('missile_count', '?')} "
            f"| 생존 적={len(blackboard['enemies'])}")
        return True


# ══════════════════════════════════════════════════════════════════════════════
# ConditionEnemyAlive — 적 팀 생존 기체 존재 여부
# ══════════════════════════════════════════════════════════════════════════════
class ConditionEnemyAlive(Node):
    """GatherCombatState가 채운 enemies가 비어있지 않으면 SUCCESS."""

    def __init__(self, name, agent):
        super().__init__(name)
        self.type = "Condition"
        self.is_expanded = False

    async def run(self, agent, blackboard):
        self.status = Status.SUCCESS if blackboard.get("enemies") else Status.FAILURE
        blackboard[self.name] = {"status": self.status, "is_expanded": self.is_expanded}
        return self.status


# ══════════════════════════════════════════════════════════════════════════════
# EngageTarget — 최근접 적 pure pursuit + WEZ 방아쇠
# ══════════════════════════════════════════════════════════════════════════════
class EngageTarget(ActionWithROSTopic):
    """적 팀 생존 기체 중 최근접 1대를 추적: 매 틱 표적 방위→heading, 표적 고도→altitude.
       발사 판정(5006의 자기 위치·헤딩 기준, wrap-around 처리):
         미사일: 거리<missile_range_m ∧ |방위오차|<missile_bearing_deg
                 → fire_id += 1 (쿨다운 missile_cooldown_sec, 잔탄>0일 때만)
         기총  : 거리<gun_range_m ∧ |방위오차|<gun_bearing_deg 인 동안 gun_firing=True
       생존 적이 없어지면(표적 destroyed) 기총을 끄고 SUCCESS.
       own 순간 결손 시 마지막 setpoint 재발행(래칭)."""

    def __init__(self, name, agent, own_name="",
                 engage_speed_mps=250.0,
                 missile_range_m=8000.0, missile_bearing_deg=15.0,
                 missile_cooldown_sec=10.0,
                 gun_range_m=1500.0, gun_bearing_deg=5.0):
        super().__init__(name, agent, (AircraftSetpoint, _SETPOINT_TOPIC))
        self._own_name  = own_name or (getattr(agent, "agent_id", "") or "").strip("/")
        self._speed     = float(engage_speed_mps)
        self._msl_range = float(missile_range_m)
        self._msl_brg   = float(missile_bearing_deg)
        self._msl_cd    = float(missile_cooldown_sec)
        self._gun_range = float(gun_range_m)
        self._gun_brg   = float(gun_bearing_deg)
        self._fire_id   = 0        # 0=미발사 초기값, 첫 발사에 1
        self._last_fire = None     # time.monotonic() of last missile
        self._last_msg  = None
        self._done      = False

    def _build_message(self, agent, blackboard):
        own     = blackboard.get("own_state")
        enemies = blackboard.get("enemies") or []

        if not enemies:
            # 표적 destroyed(적 전멸) → 기총 끄고 SUCCESS
            self._done = True
            if self._last_msg is not None:
                self._last_msg.gun_firing = False
                return self._last_msg
            hdg = (own or {}).get("yaw", 0.0)
            alt = _alt_m(own) if own else 1000.0
            msg = _setpoint(_own_routing_name(own, self._own_name), hdg, alt,
                            target_speed=self._speed)
            msg.missile_fire_id = self._fire_id
            return msg

        self._done = False
        if not own:
            return self._last_msg   # 순간 결손 → 래칭 (None이면 FAILURE)

        ox, oy = own.get("x", 0.0) * CM_TO_M, own.get("y", 0.0) * CM_TO_M
        oyaw   = own.get("yaw", 0.0)

        def dist_m(a):
            return math.hypot(a.get("x", 0.0) * CM_TO_M - ox,
                              a.get("y", 0.0) * CM_TO_M - oy)

        target  = min(enemies, key=dist_m)          # 최근접 1대
        dist    = dist_m(target)
        bearing = unit_xy_to_heading(target.get("x", 0.0) * CM_TO_M - ox,
                                     target.get("y", 0.0) * CM_TO_M - oy)
        brg_err = abs(((bearing - oyaw + 180.0) % 360.0) - 180.0)   # wrap-around
        alt     = _alt_m(target)

        # 미사일: WEZ + 쿨다운 + 잔탄 (missile_count 필드 없으면 발사 허용)
        now = time.monotonic()
        fired = False
        if (dist < self._msl_range and brg_err < self._msl_brg
                and own.get("missile_count", 1) > 0
                and (self._last_fire is None or now - self._last_fire >= self._msl_cd)):
            self._fire_id += 1
            self._last_fire = now
            fired = True

        gun = dist < self._gun_range and brg_err < self._gun_brg

        agent.ros_bridge.node.get_logger().info(
            f"[EngageTarget] tgt={target.get('aircraft_name')} 거리={dist:.0f}m "
            f"방위오차={brg_err:.1f}° hp={target.get('hp', '?')} | "
            f"gun={'ON' if gun else 'off'} msl_id={self._fire_id}"
            f"{' ★발사' if fired else ''}")

        msg = _setpoint(_own_routing_name(own, self._own_name), bearing, alt,
                        target_speed=self._speed)
        msg.gun_firing      = bool(gun)
        msg.missile_fire_id = self._fire_id
        self._last_msg = msg
        return msg

    def _interpret_publish(self, msg, agent, blackboard) -> Status:
        return Status.SUCCESS if self._done else Status.RUNNING
