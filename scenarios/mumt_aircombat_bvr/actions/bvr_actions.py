"""
BVR action nodes ported from BVRGym jsb_gym/bts/BVR/actions.py.

Each node publishes an AircraftSetpoint on /aircraft/setpoint based on the
current blackboard world state.  World-state fields are read from the
blackboard keys documented below — a real data source (e.g. the
mumt_ros_bridge telemetry topic) must populate these before each tick.

Blackboard schema
-----------------
own_state   : dict  {heading_deg, altitude_m, lat, lon, speed_mps}
target_state: dict  {heading_deg, altitude_m, lat, lon, speed_mps}  | None
own_missile : dict  {active, heading_deg, altitude_m, lat, lon}     | None
maw_warning : bool

All setpoints are published to /aircraft/setpoint
(custom_msgs/msg/AircraftSetpoint).
"""

import math

from modules.base_bt_nodes import Status
from modules.base_bt_nodes_ros import ActionWithROSTopic

try:
    from custom_msgs.msg import AircraftSetpoint
except ImportError:
    AircraftSetpoint = None  # type: ignore

_TOPIC = "/aircraft/setpoint"

# ── Geometry helpers ──────────────────────────────────────────────────────────

def _delta_heading(target: float, current: float) -> float:
    return ((target - current + 180.0) % 360.0) - 180.0


def _bearing_deg(from_lat, from_lon, to_lat, to_lon) -> float:
    """Simple equirectangular bearing (degrees, 0=N, 90=E)."""
    dlat = to_lat - from_lat
    dlon = to_lon - from_lon
    return math.degrees(math.atan2(dlon, dlat)) % 360.0


def _distance_m(from_lat, from_lon, to_lat, to_lon) -> float:
    """Flat-earth approximation in metres (good enough for BVR distances)."""
    R = 6_371_000.0
    dlat = math.radians(to_lat - from_lat)
    dlon = math.radians(to_lon - from_lon)
    lat_m = math.radians((from_lat + to_lat) / 2.0)
    return math.sqrt((dlat * R) ** 2 + (dlon * R * math.cos(lat_m)) ** 2)


def resolve_aircraft_name(agent, blackboard) -> str:
    """The UAV this BT controls.

    Prefer the exact name echoed back in own_state (UE puts Pawn->GetName() in
    every state entry); fall back to the launch namespace (--ns) before the
    first state batch arrives.  UE matches the setpoint's aircraft_name against
    the pawn name (exact first, then substring), so either form routes correctly.
    """
    own = (blackboard or {}).get("own_state") or {}
    name = own.get("aircraft_name")
    if name:
        return str(name)
    return (getattr(agent, "agent_id", "") or "").strip("/")


# ── Base for all BVR actions ──────────────────────────────────────────────────

class _BVRActionBase(ActionWithROSTopic):
    """Shared plumbing: constructs publisher, guards against missing msg type."""

    DEFAULT_THROTTLE = 0.9

    def __init__(self, name, agent):
        super().__init__(name, agent, (AircraftSetpoint, _TOPIC))

    def _make_setpoint(self, heading_deg: float, altitude_m: float,
                       throttle: float = DEFAULT_THROTTLE,
                       launch: bool = False) -> "AircraftSetpoint | None":
        if AircraftSetpoint is None:
            return None
        msg = AircraftSetpoint()
        msg.heading_deg    = float(heading_deg) % 360.0
        msg.altitude_m     = float(altitude_m)
        msg.throttle_norm  = max(0.0, min(1.0, float(throttle)))
        msg.launch_missile = bool(launch)
        return msg

    # Subclasses override this
    def _build_message(self, agent, blackboard):
        raise NotImplementedError

    async def run(self, agent, blackboard):
        # Same as ActionWithROSTopic.run, but stamp this UAV's name so UE routes
        # the setpoint to the right aircraft (one BT per UAV, shared topic).
        msg = self._build_message(agent, blackboard)
        if msg is None:
            self.status = Status.FAILURE
            return self.status
        if not getattr(msg, "aircraft_name", ""):
            msg.aircraft_name = resolve_aircraft_name(agent, blackboard)
        self._pub.publish(msg)
        self.status = self._interpret_publish(msg, agent, blackboard)
        return self.status


# ── 1. Pursue ────────────────────────────────────────────────────────────────

class Pursue(_BVRActionBase):
    """
    Close to gun/missile range.  Turns toward the target and climbs/dives to
    match its altitude.  Returns RUNNING until within ENGAGE_RANGE_M.

    TODO: populate own_state and target_state from telemetry topic.
    """

    ENGAGE_RANGE_M  = 18_000.0   # nm ≈ 18 km inside-envelope check
    PURSUE_THROTTLE = 1.0        # full power while closing

    def _build_message(self, agent, blackboard):
        own    = blackboard.get("own_state")    or {}
        target = blackboard.get("target_state") or {}

        if not own or not target:
            return None

        bearing = _bearing_deg(
            own.get("lat", 0), own.get("lon", 0),
            target.get("lat", 0), target.get("lon", 0))
        alt = float(target.get("altitude_m", own.get("altitude_m", 3000.0)))

        return self._make_setpoint(bearing, alt, self.PURSUE_THROTTLE)

    def _interpret_publish(self, msg, agent, blackboard):
        own    = blackboard.get("own_state")    or {}
        target = blackboard.get("target_state") or {}
        if not own or not target:
            return Status.RUNNING

        dist = _distance_m(
            own.get("lat", 0), own.get("lon", 0),
            target.get("lat", 0), target.get("lon", 0))
        return Status.SUCCESS if dist < self.ENGAGE_RANGE_M else Status.RUNNING


# ── 2. Launch ────────────────────────────────────────────────────────────────

class Launch(_BVRActionBase):
    """
    Commands a missile launch and holds the current heading/altitude.
    Returns SUCCESS immediately (single-shot).

    TODO: populate own_state from telemetry topic.
    """

    def _build_message(self, agent, blackboard):
        own = blackboard.get("own_state") or {}
        return self._make_setpoint(
            heading_deg = own.get("heading_deg", 0),
            altitude_m  = own.get("altitude_m", 3000.0),
            throttle    = self.DEFAULT_THROTTLE,
            launch      = True)

    def _interpret_publish(self, msg, agent, blackboard):
        return Status.SUCCESS


# ── 3. MAW_evade ─────────────────────────────────────────────────────────────

class MAW_evade(_BVRActionBase):
    """
    Missile Approach Warning evasion.  Turns 90° perpendicular to the threat
    and descends to terrain-masking altitude while increasing throttle.
    Returns RUNNING as long as maw_warning is True.

    TODO: use maw_warning from telemetry.
    """

    EVADE_ALT_M     = 300.0   # low-level escape altitude
    EVADE_THROTTLE  = 1.0

    def _build_message(self, agent, blackboard):
        own    = blackboard.get("own_state") or {}
        target = blackboard.get("target_state") or {}

        current_head = float(own.get("heading_deg", 0))

        if target:
            threat_bearing = _bearing_deg(
                own.get("lat", 0), own.get("lon", 0),
                target.get("lat", 0), target.get("lon", 0))
            # Beam the threat: turn 90° to the left of the threat bearing
            evade_head = (threat_bearing - 90.0) % 360.0
        else:
            evade_head = (current_head + 90.0) % 360.0

        return self._make_setpoint(evade_head, self.EVADE_ALT_M,
                                   self.EVADE_THROTTLE)

    def _interpret_publish(self, msg, agent, blackboard):
        still_active = bool(blackboard.get("maw_warning", False))
        return Status.RUNNING if still_active else Status.SUCCESS


# ── 4. MAW_guide_evade ───────────────────────────────────────────────────────

class MAW_guide_evade(_BVRActionBase):
    """
    Guides own missile toward target while simultaneously executing an
    evasion manoeuvre for an incoming missile.  Blends the two headings
    (own-missile guidance dominates at 70%, evasion at 30%).
    Returns RUNNING while both maw_warning and own_missile are active.

    TODO: populate own_missile from telemetry.
    """

    BLEND_GUIDE = 0.7
    BLEND_EVADE = 0.3

    def _build_message(self, agent, blackboard):
        own     = blackboard.get("own_state")    or {}
        target  = blackboard.get("target_state") or {}
        missile = blackboard.get("own_missile")  or {}

        current_head = float(own.get("heading_deg", 0))

        # Guidance heading: steer toward target from missile position
        if missile and target:
            guide_head = _bearing_deg(
                missile.get("lat", own.get("lat", 0)),
                missile.get("lon", own.get("lon", 0)),
                target.get("lat", 0), target.get("lon", 0))
        else:
            guide_head = current_head

        # Evasion heading: beam the threat
        if target:
            threat_bearing = _bearing_deg(
                own.get("lat", 0), own.get("lon", 0),
                target.get("lat", 0), target.get("lon", 0))
            evade_head = (threat_bearing - 90.0) % 360.0
        else:
            evade_head = (current_head + 90.0) % 360.0

        # Blend: weighted circular average via unit-vector method
        gr, er = math.radians(guide_head), math.radians(evade_head)
        bx = self.BLEND_GUIDE * math.cos(gr) + self.BLEND_EVADE * math.cos(er)
        by = self.BLEND_GUIDE * math.sin(gr) + self.BLEND_EVADE * math.sin(er)
        blended_head = math.degrees(math.atan2(by, bx)) % 360.0

        return self._make_setpoint(blended_head,
                                   float(own.get("altitude_m", 3000.0)),
                                   1.0)

    def _interpret_publish(self, msg, agent, blackboard):
        maw    = bool(blackboard.get("maw_warning", False))
        active = bool((blackboard.get("own_missile") or {}).get("active", False))
        return Status.RUNNING if (maw and active) else Status.SUCCESS


# ── 5. Guide_own ─────────────────────────────────────────────────────────────

class Guide_own(_BVRActionBase):
    """
    Steers the aircraft to keep the own missile on a collision course with
    the target (maintain line-of-sight heading).
    Returns RUNNING while own_missile.active is True.

    TODO: populate own_missile from telemetry.
    """

    def _build_message(self, agent, blackboard):
        own     = blackboard.get("own_state")    or {}
        target  = blackboard.get("target_state") or {}
        missile = blackboard.get("own_missile")  or {}

        if target:
            # Point aircraft nose at target to maximise datalink range
            guide_head = _bearing_deg(
                own.get("lat", 0), own.get("lon", 0),
                target.get("lat", 0), target.get("lon", 0))
            guide_alt  = float(target.get("altitude_m",
                                          own.get("altitude_m", 3000.0)))
        else:
            guide_head = float(own.get("heading_deg", 0))
            guide_alt  = float(own.get("altitude_m", 3000.0))

        return self._make_setpoint(guide_head, guide_alt,
                                   self.DEFAULT_THROTTLE)

    def _interpret_publish(self, msg, agent, blackboard):
        active = bool((blackboard.get("own_missile") or {}).get("active", False))
        return Status.RUNNING if active else Status.SUCCESS
