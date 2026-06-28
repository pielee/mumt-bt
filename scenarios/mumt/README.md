# 🛩️ MUM-T Formation (MUMT_Sim / UE5 + JSBSim)

**Manned-leader + UAV-wingman formation flight, driven by a Behaviour Tree over the ROS↔UDP bridge**

## Overview

This scenario flies a single **UAV (`F16_UAV`)** as the wingman of a **manned leader (`M_F16`)**. The
leader is flown by a human via a joystick; the UAV is autonomous and runs **this BT as one ROS
namespace / process**. The BT never talks to the UE5 simulator directly — it exchanges ROS 2 topics with
the `mumt_ros_bridge`, which translates them to/from UDP for the UE5 + JSBSim flight model.

- **Out:** `/aircraft/setpoint` (`custom_msgs/AircraftSetpoint`) → bridge → UDP `5010` → per-UAV autopilot in UE.
- **In:** `/mumt/aircraft_states` (`std_msgs/String`, JSON state batch) ← bridge ← UDP `5006`.

The BT publishes setpoints at the configured tick rate (`bt_tick_rate: 10.0` → ~10 Hz). Because addressing is
**name-based** (the UE pawn responds only to the `aircraft_name` it matches), the joystick→manned and BT→UAV
data paths share these topics simultaneously without interfering.

> ⚠️ **One setpoint publisher per UAV.** Running two BT processes for the same `F16_UAV` produces conflicting
> setpoints and autopilot oscillation. To drive a second UAV, launch a second process with a different `--ns`.

---

## Behaviour Tree (`default_bt.xml`)

```
ReactiveSequence
├── GatherState               (own_name="F16_UAV", leader_name="M_F16")
└── Sequence
    ├── WaitForLeaderTakeoff   (leader_airborne_climb_m="80")
    ├── Takeoff                (runway_heading_deg="90", climb_target_m="1000", uav_airborne_climb_m="200")
    └── MaintainFormation      (aft_offset_m="-80", lateral_offset_m="-40", vertical_offset_m="0",
                                blend_radius_m="300", kp_speed="0.05")
```

The root is a **`ReactiveSequence`**: `GatherState` is re-evaluated every tick, so if perception of the leader
or own aircraft is lost the inner `Sequence` is interrupted; once both are seen again it resumes.

> 💡 This framework takes each node's **name from its XML tag** — do **not** add a `name=` attribute (it
> collides with the framework's own `name`). XML comments must not contain `--`.

### Nodes

| Node | XML attributes | Type | Success / Failure semantics (from `bt_nodes.py`) |
|------|----------------|------|--------------------------------------------------|
| **GatherState** | `own_name`, `leader_name` | Condition (`ConditionWithROSTopics`) | Parses the latest `/mumt/aircraft_states` batch into `all_states`, records each aircraft's spawn altitude once into `init_alt` (`setdefault`), and stores `own_state` / `leader_state`. **SUCCESS** iff both own and leader are found (token-boundary name match); otherwise **FAILURE**. The topic cache keeps the last message, so a momentary gap still resolves against the last state (latching). |
| **WaitForLeaderTakeoff** | `leader_airborne_climb_m` | Action (`ActionWithROSTopic`) | Holds on the ground, publishing an **idle** setpoint (`throttle=0`, `target_speed=0`) at the own aircraft's current heading/altitude. **SUCCESS** once the leader has climbed `≥ leader_airborne_climb_m` above its spawn altitude; otherwise **RUNNING** (RUNNING also while the leader state is missing). |
| **Takeoff** | `runway_heading_deg`, `climb_target_m`, `uav_airborne_climb_m` | Action (`ActionWithROSTopic`) | Publishes a setpoint holding the runway heading and commanding `target_speed=TAKEOFF_SPEED_MPS` (autothrottle) toward `spawn_alt + climb_target_m`. **SUCCESS** once the UAV has climbed `≥ uav_airborne_climb_m` above its spawn; otherwise **RUNNING**. |
| **MaintainFormation** | `aft_offset_m`, `lateral_offset_m`, `vertical_offset_m`, `blend_radius_m`, `kp_speed` | Action (`ActionWithROSTopic`) | Computes the offset slot relative to the leader and publishes heading/altitude/`target_speed`. **Always RUNNING** (formation never "finishes", and never FAILURE so the parent `Sequence` won't fall back to `Takeoff`). On a momentary own/leader gap it re-publishes the **last** setpoint and stays RUNNING. |

### Milestone structure & test variants

The scenario was built bottom-up; the XML keeps the earlier milestones as commented-out blocks so each stage
can be exercised in isolation (uncomment one and comment out the final tree):

- **M0 — `HoldSetpoint`** — publishes a fixed setpoint forever (`heading_deg`, `altitude_m`, `target_speed_mps`),
  always RUNNING. Validates the full ROS→bridge→UDP→autopilot chain and gives a naive ground-start takeoff.
- **M1 — `GatherState` + M0** — adds own/leader perception on top of the fixed setpoint.
- **M2 — `WaitForLeaderTakeoff` + `Takeoff`** — wait-for-leader then UAV takeoff (no formation).
- **M3 — `MaintainFormation`** — the offset-slot + closure follower; the active tree above.

---

## MaintainFormation geometry

Working in the UE world frame (x=East, y=South, metres), with the leader's forward unit vector
`fwd = (sin H, −cos H)` and right unit vector `right = (cos H, sin H)` from leader yaw `H`:

```
slot = leader_pos − aft_offset_m · fwd + lateral_offset_m · right
```

- **`aft_offset_m` sign selects front/behind:** **NEGATIVE = slot in FRONT of the leader**, positive = behind.
  The current config uses **`aft_offset_m = -80`, so the UAV flies ~80 m in FRONT of the manned jet.**
- **`lateral_offset_m`:** positive = right wing, negative = left wing (current config `-40` = left).
- **Altitude:** `max(leader_alt + vertical_offset_m, own_spawn + MIN_AGL_M)` — the spawn-relative floor
  (`MIN_AGL_M = 60`) keeps the slot from being driven into the ground.
- **Heading blend:** `shortest_heading_blend(leader_yaw, bearing_to_slot, w)` with `w = clamp(dist_to_slot /
  blend_radius_m, 0, 1)`. Far from the slot → track the slot bearing; near the slot → converge to the leader's
  heading (anti-weave).
- **Speed (via autothrottle `target_speed_mps`):**
  - **Rendezvous** — if `dist_to_leader > RENDEZVOUS_M (1500)` → `max(RENDEZVOUS_SPEED 260, leader_speed + 50)`.
  - **Station-keeping** — else `max(MIN_FORM_SPEED 180, leader_speed + clamp(kp_speed · along, −20, +30))`,
    where `along = (slot − own) · fwd` (positive = lagging behind the slot → speed up).
- **Latching / RUNNING:** the node never returns FAILURE; on a transient state gap it re-publishes the last
  setpoint, so the parent `Sequence` is never popped back to `Takeoff`.

---

## Key constants (`bt_nodes.py`)

| Constant | Value | Meaning |
|----------|-------|---------|
| `OWN_NAME` | `"F16_UAV"` | UAV pawn name (token-boundary matched, e.g. `F16_UAV_C_2`) |
| `LEADER_NAME` | `"M_F16"` | Manned leader pawn name |
| `RUNWAY_HEADING_DEG` | `90.0` | Takeoff/runway heading (East) |
| `TAKEOFF_SPEED_MPS` | `220.0` | Autothrottle target during takeoff |
| `LEADER_AIRBORNE_CLIMB_M` | `80.0` | Leader climb-above-spawn to count as airborne |
| `UAV_AIRBORNE_CLIMB_M` | `200.0` | UAV climb-above-spawn to finish takeoff |
| `TAKEOFF_CLIMB_TARGET_M` | `1000.0` | Takeoff climb target above spawn |
| `AFT_OFFSET_M` | `-80.0` | Negative = slot in FRONT of leader |
| `LATERAL_OFFSET_M` | `-40.0` | Negative = left wing |
| `VERTICAL_OFFSET_M` | `0.0` | Vertical slot offset |
| `BLEND_RADIUS_M` | `300.0` | Heading-blend radius |
| `RENDEZVOUS_M` | `1500.0` | Beyond this distance → rendezvous speed |
| `RENDEZVOUS_SPEED` | `260.0` | Catch-up target speed |
| `KP_SPEED` | `0.05` | Along-track error (m) → speed trim (m/s) |
| `ALONG_SPD_MIN` / `ALONG_SPD_MAX` | `-20.0` / `30.0` | Closure speed-trim clamp |
| `MIN_FORM_SPEED` | `180.0` | Formation speed floor (stall guard) |
| `MIN_AGL_M` | `60.0` | Slot altitude floor above spawn |
| `CM_TO_M` | `0.01` | State positions are cm → m |

---

## Interface contract

- **`AircraftSetpoint` fields** (all populated by `_setpoint(...)`): `aircraft_name`, `heading_deg`,
  `altitude_m`, `throttle_norm` (clamped 0–1), `target_speed_mps`, `launch_missile` (always `False` here).
  Speed is commanded through **`target_speed_mps` (autothrottle)** rather than open-loop throttle, to avoid
  stalling during the climb.
- **`aircraft_name` is mandatory** — UE routes each setpoint to the pawn whose instance name **token-matches**
  the field (`_name_matches`: `"F16_UAV"` matches `F16_UAV_C_2` but not `F16_UAV2`). The node prefers the
  exact name observed in the state batch, falling back to `OWN_NAME` / the agent namespace.
- **Compass heading:** `0 = North, 90 = East`. With UE world x=East, y=South:
  `heading = degrees(atan2(Δx, −Δy)) % 360` (`unit_xy_to_heading`); inverse `(sin H, −cos H)`
  (`heading_to_unit_xy`). The math-frame `atan2(Δy, Δx)` would be 90° wrong.
- **Altitude is UE-Z**, not ASL: `altitude_m` and state `z/100` share the same frame
  (`AltM = Location.Z / 100` in the autopilot). Because ground level can be negative / map-dependent, all
  takeoff and altitude-floor decisions use **spawn-relative** climb thresholds (`init_alt` baseline).
- **One setpoint publisher per UAV** (see warning above).

---

## How to run

The UE5 sim and the `mumt_ros_bridge` must already be up (the bridge owns UDP `5006`/`5010`). Then:

```bash
cd ~/dev/py_bt_ros
source ~/dev/mumt_ros_ws/install/setup.bash      # custom_msgs/AircraftSetpoint, etc.
python3 main.py --config scenarios/mumt/configs/mumt.yaml
```

Config (`configs/mumt.yaml`): `environment: scenarios.mumt`, `namespaces: "/F16_UAV"` (the agent id; also the
default `own_name` if the XML omits it), `behavior_tree_xml: default_bt.xml`, `bt_tick_rate: 10.0`.

To drive a differently-named UAV without editing the config, override the namespace:

```bash
python3 main.py --config scenarios/mumt/configs/mumt.yaml --ns /F16_UAV_2
```

`--ns` replaces `config['agent']['namespaces']`, becoming the agent id used as the fallback routing name.
