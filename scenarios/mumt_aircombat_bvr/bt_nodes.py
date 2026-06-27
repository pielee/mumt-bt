"""
BVR air-combat BT nodes for the mumt_aircombat_bvr scenario.

Blackboard contract
-------------------
Inputs (populated by world-state subscriber before each tick):
  "own_state"     : dict  — {heading_deg, altitude_m, x, y, speed_mps, ...}
  "target_state"  : dict  — {heading_deg, altitude_m, ...}  (may be None)
  "maw_warning"   : bool  — Missile Approach Warning active
  "own_missile"   : dict  — active own missile state (may be None)

Output topic:
  /aircraft/setpoint  (custom_msgs/msg/AircraftSetpoint)
"""

import json
import math
import time

from std_msgs.msg import String

from modules.base_bt_nodes import BTNodeList, Sequence, Status
from modules.base_bt_nodes_ros import ActionWithROSTopic

try:
    from custom_msgs.msg import AircraftSetpoint
except ImportError:
    AircraftSetpoint = None  # type: ignore

from scenarios.mumt_aircombat_bvr.actions.bvr_actions import (
    Pursue,
    Launch,
    MAW_evade,
    MAW_guide_evade,
    Guide_own,
    resolve_aircraft_name,
)

# ── Node registration ──────────────────────────────────────────────────────────

CUSTOM_ACTION_NODES = [
    "CruiseFlight",
    "TakeOff",
    "TurnRight",
    "Pursue",
    "Launch",
    "MAW_evade",
    "MAW_guide_evade",
    "Guide_own",
]

BTNodeList.ACTION_NODES.extend(CUSTOM_ACTION_NODES)

# ── World-state subscriber ─────────────────────────────────────────────────────

_STATE_TOPIC = "/mumt/aircraft_states"


class BVRWorldState:
    """
    Subscribes to /mumt/aircraft_states (JSON batch from UE5 bridge) and
    keeps own_state dict up-to-date.

    JSON schema from UE5 BuildPawnState:
      { "aircraft": [ { "aircraft_name", "x", "y", "z",
                        "yaw", "pitch", "roll",
                        "speed_mps", "throttle", ... } ] }

    Field mapping → own_state:
      yaw      → heading_deg   (JSBSim Psi in degrees)
      z / 100  → altitude_m    (UE5 cm → m)
      x, y     → x_m, y_m     (UE5 cm → m, for relative geometry)
      speed_mps, pitch, roll, throttle passed through unchanged
    """

    def __init__(self, agent):
        self._latest: dict = {}
        # This UAV's id (== launch --ns). Used to pick our own entry out of the
        # multi-aircraft state batch.
        self._name = (getattr(agent, "agent_id", "") or "").strip("/")
        agent.ros_bridge.node.create_subscription(
            String, _STATE_TOPIC, self._cb, 1
        )

    def _select_own(self, aircraft_list):
        """Pick THIS UAV's entry out of the multi-aircraft state batch.

        No namespace set → first entry (single-UAV / manned). Otherwise match by
        name: exact first (collision-free for prefixes like F16_UAV vs F16_UAV2),
        then substring ONLY if exactly one entry contains the name. An ambiguous
        or missing name returns None (wait) rather than grabbing a wrong aircraft.
        """
        if not aircraft_list:
            return None
        if not self._name:
            return aircraft_list[0]
        for a in aircraft_list:                                  # 1) exact
            if a.get("aircraft_name", "") == self._name:
                return a
        subs = [a for a in aircraft_list                          # 2) unambiguous substring
                if self._name in a.get("aircraft_name", "")]
        if len(subs) == 1:
            return subs[0]
        return None

    def _cb(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        aircraft_list = payload.get("aircraft", [])
        raw = self._select_own(aircraft_list)
        if raw is None:
            return
        self._latest = {
            "heading_deg": float(raw.get("yaw", 0.0)),
            "altitude_m":  float(raw.get("z", 0.0)) / 100.0,
            "x_m":         float(raw.get("x", 0.0)) / 100.0,
            "y_m":         float(raw.get("y", 0.0)) / 100.0,
            "speed_mps":   float(raw.get("speed_mps", 0.0)),
            "pitch":       float(raw.get("pitch", 0.0)),
            "roll":        float(raw.get("roll", 0.0)),
            "throttle":    float(raw.get("throttle", 0.0)),
            "aircraft_name": raw.get("aircraft_name", ""),
        }

    def own_state(self) -> dict:
        return dict(self._latest)


def _get_world_state(agent, blackboard) -> BVRWorldState:
    ws = blackboard.get("_bvr_world_state")
    if ws is None:
        ws = BVRWorldState(agent)
        blackboard["_bvr_world_state"] = ws
    return ws


# ── Helpers ────────────────────────────────────────────────────────────────────

_SETPOINT_TOPIC = "/aircraft/setpoint"


def _delta_heading(target: float, current: float) -> float:
    return ((target - current + 180.0) % 360.0) - 180.0


def _refresh_own_state(agent, blackboard) -> dict:
    ws = _get_world_state(agent, blackboard)
    own_state = ws.own_state()
    if own_state:
        blackboard["own_state"] = own_state
    return blackboard.get("own_state") or {}


# ── CruiseFlight ──────────────────────────────────────────────────────────────

class CruiseFlight(ActionWithROSTopic):
    """
    목표 헤딩/고도로 순항. 헤딩 오차 < ARRIVE_THRESH_DEG 이면 SUCCESS.

    XML attributes:
      target_heading_deg, target_altitude_m, throttle
    """

    DEFAULT_THROTTLE  = 0.8
    ARRIVE_THRESH_DEG = 5.0

    def __init__(self, name, agent,
                 target_heading_deg=None,
                 target_altitude_m=None,
                 throttle=None):
        super().__init__(name, agent, (AircraftSetpoint, _SETPOINT_TOPIC))
        self._fixed_heading  = float(target_heading_deg) if target_heading_deg is not None else None
        self._fixed_alt      = float(target_altitude_m)  if target_altitude_m  is not None else None
        self._fixed_throttle = float(throttle)           if throttle            is not None else None

    def _build_message(self, agent, blackboard):
        if AircraftSetpoint is None:
            return None

        own    = _refresh_own_state(agent, blackboard)
        target = blackboard.get("target_state") or {}

        target_heading = self._fixed_heading if self._fixed_heading is not None \
                         else float(target.get("heading_deg", own.get("heading_deg", 0.0)))
        target_alt     = self._fixed_alt     if self._fixed_alt     is not None \
                         else float(target.get("altitude_m",  own.get("altitude_m",  3000.0)))
        throttle       = self._fixed_throttle if self._fixed_throttle is not None \
                         else float(blackboard.get("throttle_override", self.DEFAULT_THROTTLE))

        msg = AircraftSetpoint()
        msg.heading_deg    = target_heading
        msg.altitude_m     = target_alt
        msg.throttle_norm  = max(0.0, min(1.0, throttle))
        msg.launch_missile = False
        msg.aircraft_name  = resolve_aircraft_name(agent, blackboard)

        agent.ros_bridge.node.get_logger().info(
            f"[CruiseFlight] hdg={own.get('heading_deg', '?'):.1f}° alt={own.get('altitude_m', '?'):.0f}m"
            f" → {target_heading:.1f}° {target_alt:.0f}m"
        ) if own else None

        return msg

    def _interpret_publish(self, msg, agent, blackboard):
        own = blackboard.get("own_state") or {}
        diff = abs(_delta_heading(msg.heading_deg, float(own.get("heading_deg", msg.heading_deg))))
        return Status.SUCCESS if diff < self.ARRIVE_THRESH_DEG else Status.RUNNING


# ── TakeOff ───────────────────────────────────────────────────────────────────

class TakeOff(ActionWithROSTopic):
    """
    목표 고도까지 상승. 목표고도 ±1% 이내를 10초 이상 유지하면 SUCCESS.

    XML attributes:
      target_altitude_m  — 목표 고도 (기본 1500.0 m)
      target_heading_deg — 이륙 헤딩 유지 (기본 90.0 °)
      throttle           — 스로틀 (기본 0.9)
    """

    STABLE_DURATION_S = 10.0
    ALT_TOLERANCE_PCT = 0.01  # 1%

    def __init__(self, name, agent,
                 target_altitude_m=1500.0,
                 target_heading_deg=90.0,
                 throttle=0.9):
        super().__init__(name, agent, (AircraftSetpoint, _SETPOINT_TOPIC))
        self._target_alt     = float(target_altitude_m)
        self._target_heading = float(target_heading_deg)
        self._throttle       = float(throttle)
        self._stable_since: float | None = None

    def halt(self):
        self._stable_since = None

    def _build_message(self, agent, blackboard):
        if AircraftSetpoint is None:
            return None

        own = _refresh_own_state(agent, blackboard)

        msg = AircraftSetpoint()
        msg.heading_deg    = self._target_heading
        msg.altitude_m     = self._target_alt
        msg.throttle_norm  = self._throttle
        msg.launch_missile = False
        msg.aircraft_name  = resolve_aircraft_name(agent, blackboard)

        current_alt = float(own.get("altitude_m", 0.0))
        stable_secs = (time.monotonic() - self._stable_since) if self._stable_since else 0.0

        agent.ros_bridge.node.get_logger().info(
            f"[TakeOff] alt={current_alt:.0f}m / {self._target_alt:.0f}m"
            f"  err={current_alt - self._target_alt:+.0f}m"
            f"  stable={stable_secs:.1f}s / {self.STABLE_DURATION_S:.0f}s"
        ) if own else None

        return msg

    def _interpret_publish(self, msg, agent, blackboard):
        own         = blackboard.get("own_state") or {}
        current_alt = float(own.get("altitude_m", 0.0))
        tolerance   = self._target_alt * self.ALT_TOLERANCE_PCT
        now         = time.monotonic()

        if abs(current_alt - self._target_alt) <= tolerance:
            if self._stable_since is None:
                self._stable_since = now
            elif now - self._stable_since >= self.STABLE_DURATION_S:
                return Status.SUCCESS
        else:
            self._stable_since = None

        return Status.RUNNING


# ── TurnRight ─────────────────────────────────────────────────────────────────

class TurnRight(ActionWithROSTopic):
    """
    목표 헤딩으로 우회전. 헤딩 오차 < ARRIVE_THRESH_DEG 이면 SUCCESS.

    XML attributes:
      target_heading_deg — 목표 헤딩 (°)
      target_altitude_m  — 유지 고도 (기본 own_state 값)
      throttle           — 스로틀 (기본 0.8)
    """

    ARRIVE_THRESH_DEG = 5.0

    def __init__(self, name, agent,
                 target_heading_deg=180.0,
                 target_altitude_m=None,
                 throttle=0.8):
        super().__init__(name, agent, (AircraftSetpoint, _SETPOINT_TOPIC))
        self._target_heading = float(target_heading_deg)
        self._fixed_alt      = float(target_altitude_m) if target_altitude_m is not None else None
        self._throttle       = float(throttle)

    def _build_message(self, agent, blackboard):
        if AircraftSetpoint is None:
            return None

        own        = _refresh_own_state(agent, blackboard)
        target_alt = self._fixed_alt if self._fixed_alt is not None \
                     else float(own.get("altitude_m", 1500.0))

        msg = AircraftSetpoint()
        msg.heading_deg    = self._target_heading
        msg.altitude_m     = target_alt
        msg.throttle_norm  = self._throttle
        msg.launch_missile = False
        msg.aircraft_name  = resolve_aircraft_name(agent, blackboard)

        current_hdg = float(own.get("heading_deg", 0.0))
        diff = _delta_heading(self._target_heading, current_hdg)

        agent.ros_bridge.node.get_logger().info(
            f"[TurnRight] hdg={current_hdg:.1f}° → {self._target_heading:.1f}°  diff={diff:+.1f}°"
        ) if own else None

        return msg

    def _interpret_publish(self, _msg, _agent, blackboard):
        own  = blackboard.get("own_state") or {}
        diff = abs(_delta_heading(self._target_heading, float(own.get("heading_deg", self._target_heading))))
        return Status.SUCCESS if diff < self.ARRIVE_THRESH_DEG else Status.RUNNING
