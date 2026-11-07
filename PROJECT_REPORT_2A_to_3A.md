# KD_4373 — Khoj-o-Drone
## Complete engineering report: Task 2A → Task 3A

**Team:** KD_4373  
**Competition:** e-Yantra Robotics Competition 2025–26, theme KD (Khoj-o-Drone)  
**Platform:** MuJoCo + ROS 2 Humble + WhyCode, WSL2 Ubuntu 22.04  
**Workspace:** `/home/varun/pico_ws`  
**Period covered:** 2026-10-18 → 2026-11-06  
**Prepared:** 2026-11-06

> This is the single reference document for the four tasks: what each task asked for,
> what was built, which bugs were found and fixed, how each result was measured, how
> to reproduce it, and what is still worth knowing. It is written so that a new session
> can continue the work without re-reading the chat history.

---

## Contents

1. [Executive summary](#1-executive-summary)
2. [Platform, coordinates, and conventions](#2-platform-coordinates-and-conventions)
3. [Task 2A — position hold](#3-task-2a--position-hold)
4. [Task 2B — waypoint navigation](#4-task-2b--waypoint-navigation)
5. [Task 2C — critical survivor detection and saves](#5-task-2c--critical-survivor-detection-and-saves)
6. [Task 3A — critical + stable survivors, obstacles, landing](#6-task-3a--critical--stable-survivors-obstacles-landing)
7. [Cross-task design evolution](#7-cross-task-design-evolution)
8. [Reproduction guide](#8-reproduction-guide)
9. [Verification and artifact inventory](#9-verification-and-artifact-inventory)
10. [Known limitations and caveats](#10-known-limitations-and-caveats)
11. [Timeline](#11-timeline)
12. [Lessons learned](#12-lessons-learned)
- [Appendix A — constants, gains, and geometry](#appendix-a--constants-gains-and-geometry)
- [Appendix B — interfaces and route protocol](#appendix-b--interfaces-and-route-protocol)
- [Appendix C — glossary](#appendix-c--glossary)

---

## 1. Executive summary

| Task | What it required | Result | Key measured timing | Submitted bag |
|---|---|---|---|---|
| **2A** | Fly to `(1.5, −1.5, 13)` and hold inside ±0.4 for the 60 s scoring window | **20.00/20**, 10 consecutive perfect runs | box entry 4.92–5.02 s, continuous hold 56.1–56.4 s, all axes ±0.02 | 64.7 s, 3 topics |
| **2B** | Visit four fixed waypoints in order; 3 s inside ±0.4 at each | **40.00/40**, every run across four builds | 4th hold 28.50 s from controller start, 32.12 s from bag start | 34.7 s, 2 topics |
| **2C** | Detect four red critical survivors, Visit each 3 s at Z=13 | **40.00/40**, seven hardened full-mission runs | 4th hold 30.46 s from controller start (submitted bag) | 59.8 s, 2 topics |
| **3A** | Save 3 critical survivors by 55 s, then Visit 3 stable survivors, avoid 3 obstacles, hover and land on F6 within 200 s | **all objectives PASS**, full marks on time, **65.80/80 projected** (portal scored the earlier v10 take 49.21) | 3rd critical Save 40.05 s; F6 hover 79.19–81.30 s; touchdown **82.47 s** | 89.0 s, 3 topics |

The work progressed in four distinct engineering steps:

1. **2A** solved the hard part of the flight stack: a stable, repeatable position-hold
   controller on a nearly ballistic plant with a noisy depth channel and a 2–4 s
   post-arm dead zone.
2. **2B** added the ROS 2 action/service layer and transit tuning: four waypoints,
   yaw control, tilt compensation, and a goal-timeout safety net.
3. **2C** replaced fixed waypoints with perception: rectified arena image → red
   survivor detection → grid snapping → shortest collision-free tour.
4. **3A** extended the mission to two survivor classes, three obstacles, a mid-mission
   re-plan from the actual Switch pose, and a full hover-and-land sequence on F6.

Every task was validated from the rosbag, not from console text. The scorers in the
backups reconstruct the grader's hold and time checks directly from
`/whycode_node/markers` and `/controller_heartbeat` (and, for 2A, `/pos_error`).

The submitted Task 3A client also grants an in-progress survivor Save up to 15 s
past the 55 s deadline. On the official fast setup the branch is never reached; on
a slower host it keeps the third Save and the Switch instead of losing both.

The first Task 3A submission scored **34.76/80** because the server declared its
2 s Save holds from the filtered depth while the grader scored the raw marker. The
filtered hold looked complete; the raw marker left the box for 0.4–0.6 s in the
middle. The final server judges Visit, Save and F6-hover holds on the unfiltered
marker sample, which is exactly what the grader sees. Section 6.11 documents the
diagnosis and the re-run.

The next submission (v10, 79.50 s) still scored only **49.21/80**. Portal feedback
exposed the missing detail: the hold test is a **3-D Euclidean sphere of radius
0.40**, not a per-axis ±0.4 box. Two holds that the box scorer called complete were
short in the sphere (B10 Visit 0.79 s, J6 Save 1.70 s). The sphere was verified
against the golden bag (48/48 holds) and against the v10 failure strings, then the
controller was rebuilt around it. The v17 take (84.52 s) passed all twelve holds
at R=0.40 and again at R=0.38 with zero marker gaps and projected 65.44/80.
Section 6.12 documents that campaign.

The final v20ref take (**82.47 s**) uses the same flight code with only
sphere-entry logging added. It repeats 12/12 holds at R=0.40 and R=0.38, zero
marker gaps, and improves the projection to **65.80/80** (65.71 without the
portal offset). The motion campaign behind that selection — the recorded screen
run, the XY overshoot and Save-depth ringing measurements, and the rejected Z
deadband, braking-cap and yaw-guard experiments — is documented in section 6.13.

---

## 2. Platform, coordinates, and conventions

### 2.1 Stack

- **Simulator:** MuJoCo, launched by `ros2 launch swift_pico swift_pico_simulation.launch.py`.
  The Task 3 release ships a prebuilt `mujoco_bridge`; the arena and drone models are
  part of `swift_pico_description`.
- **Perception:** the `whycode` node detects the circular marker on the drone and
  publishes `/whycode_node/markers` (`whycode_interfaces/msg/MarkerArray`) at ~30 Hz.
  It also publishes the annotated camera image; the compressed transport
  (`/whycode_node/processed_image/compressed`) is used by the lightweight viewer.
- **Flight interface:** `/drone_command` (`swift_msgs/msg/SwiftMsgs`) carries
  `rc_roll`, `rc_pitch`, `rc_yaw`, `rc_throttle` in the 1000–2000 µs-style range,
  neutral 1500.
- **Workspace:** `/home/varun/pico_ws`; the three mission scripts live in
  `src/swift_pico/src/` and are installed as ROS 2 executables.
- **OS:** WSL2, distro **Ubuntu-22.04** (the default `Ubuntu` distro on this machine
  is broken). ROS 2 Humble.

### 2.2 WhyCode coordinates

The ceiling camera is at **Z = 15 m** looking down. The marker's reported **Z is depth
below the camera**, not altitude:

```
altitude ≈ 15 − z
z = 13  → about 2 m above the floor (the route altitude)
z = 14  → about 1 m above the floor (the Save altitude)
z ≈ 15  → on the floor
```

This inversion is the single most important convention in the project. It caused two
sign bugs in Task 2A (section 3.4) and it shapes the landing logic in Task 3A.

The X/Y axes are the arena plane. The official grid is 11 × 11 named intersections
`A1 … K11`, with **F6** as the home/take-off/landing cell. In the rectified 900 × 900
image:

```
SIZE        = 900 px
GRID_MARGIN = 75 px       first named grid line
GRID_STEP   = 75 px       one cell
GRID_SPACING= 1.203414    WhyCode units per cell
F6          = (450, 450) px
```

### 2.3 RC conventions (verified by step tests)

| Command | Effect |
|---|---|
| `rc_pitch > 1500` | move toward **+X** |
| `rc_roll > 1500` | move toward **+Y** |
| `rc_throttle > 1500` | **climb** (z decreases) |
| `rc_throttle ≈ 1518` | hover at route altitude |
| `rc_yaw = 1500` | hold heading; the reported marker yaw runs opposite to airframe yaw because the camera looks down |

The airframe is nearly ballistic: there is no meaningful drag, and the attitude
response lags an rc change by roughly 1–2 s. A plain position P loop therefore rings
forever. Every controller in this project uses a **cascade**:

```
position error → commanded velocity → velocity error → rc command
```

with velocity estimated from the marker stream over a windowed finite difference
(≥0.12 s) plus an EMA. This is not optional; it is the difference between a
limit cycle and a hold.

### 2.4 Simulator quirks that drove the design

1. **Post-arm dead zone.** For roughly 2–4.3 s after arming, the simulator ignores
   thrust and tilt. The drone sits on the ground. A controller must wait it out, not
   interpret it as a failed command and push harder.
2. **Ground tilt is fatal.** Tilting while still on the ground drags/flips the drone
   and kills the marker stream. Task 2A therefore climbs vertically (no X/Y authority)
   until `z < 14.2`, then enables translation.
3. **Depth is the noisiest channel.** The marker depth can jump by tenths of a unit;
   raw differentiation produced ±0.7 m/s of false vertical speed. Task 3A filters
   depth before use and estimates vertical speed over a 0.30 s window.
4. **One bridge only.** A stale second `mujoco_bridge` writes the same shared image
   ring and marker topics and corrupts the pose mid-flight. The official runner
   performs a single-bridge/single-detector preflight.
5. **Marker loss ≠ node crash.** The whycode node stays alive when tracking is lost;
   it simply stops producing usable poses. Watchdogs must level the drone rather than
   freeze stale tilt commands.

---

## 3. Task 2A — position hold

### 3.1 Objective

Write a hard-coded-tuning ROS 2 position controller (`pico_controller_PID.py`) that
flies the drone to the setpoint **(1.5, −1.5, 13)** in WhyCode coordinates and holds
it for the 60 s scoring window. The box is ±0.4 on X, Y and Z.

Scoring (20 marks):

| Component | Marks | Rule |
|---|---|---|
| Hold | 8 | `min(longest_continuous_hold, 10) / 10 × 8` |
| Finishing bonus | 4 | only if the hold reaches ≥10 s |
| Speed | 8 | 8 if the 10 s hold completes by 15 s; otherwise `max(0, 8 − 0.18 × (t − 15))` |

The submitted bag must start **before** the controller runs, while the drone is still
parked, so the grader sees a clean take-off.

### 3.2 Controller architecture

The final controller is a cascaded PI-P loop:

```
v_des = clamp(K_POS * (setpoint − position), ±V_MAX)
rc    = RC_MID + K_RC * (v_des − v_est) + K_I * integral(error)
```

Key elements:

- **Windowed velocity estimate.** Raw 30 Hz marker samples are accumulated over a
  ≥0.12 s window, differentiated, then EMA-filtered (`vel_alpha = 0.4`). Per-frame
  differentiation gave ±1–3 units/s noise and poisoned the velocity loop.
- **Phase A vertical liftoff.** Until `z < 14.2`, X/Y authority is disabled and the
  drone climbs at a fixed throttle. This prevents the ground-tilt marker loss.
- **Two-stage watchdog.** Marker age >0.4 s → level the tilt but keep the last thrust;
  >1.5 s → level and neutral thrust. Stale tilt commands were the cause of early
  camera dives.
- **Jump rejector.** A marker fix more than 0.55 units from the last good pose is
  discarded.
- **Anti-windup.** Integral sums are clamped (`max_error_sum = [40, 40, 100]`);
  the integral and `prev_error` are seeded on the first valid marker, and the
  integrators are frozen during phase A.
- **Heartbeat.** `/controller_heartbeat` is published at 10 Hz; the official clock
  starts at the first heartbeat.

Final gains:

| Knob | Value | Meaning |
|---|---|---|
| `pos2vel` | `[0.7, 0.7, 0.45]` | position error → target velocity |
| `Kpv` | `[100, 100, 80]` | velocity error → rc |
| `Ki` | `[0.05, 0.05, 0.08]` | position integral (Z deliberately not wired) |
| `max_dev` | `[55, 55, 100]` | rc clamp |
| `max_error_sum` | `[40, 40, 100]` | integral clamp |
| `vt_z_max` | `0.5` | vertical target-velocity cap |
| `z_hover_ff` | `−18.0` | throttle feed-forward at the hold altitude |
| phase-A gate | `14.2` | vertical-only until this depth |
| climb command | `−48` (rc 1548) | phase-A climb |
| watchdog | `0.4 / 1.5 s` | level → neutral |
| velocity window / EMA | `0.12 s / 0.4` | estimator |
| jump reject | `0.55` | outlier gate |

The final Z law (after the sign fixes):

```python
vt_z = clamp(-0.45 * error_z, ±0.5)          # error_z > 0 means z too small
out_throttle = z_hover_ff + 80.0 * (v_up - vt_z)
if z < 9.8:                                   # camera guard
    out_throttle = max(out_throttle, 35.0)
```

### 3.3 The bugs that mattered

**1. Derivative spike at t=0.** The first marker fix produced a huge position jump and
a D-term rocket. Fixed by seeding `prev_error` and zeroing the integral on the first
valid marker.

**2. Ground tilt killed the marker stream.** Tilting before liftoff dragged the drone.
Fixed with phase A: vertical climb until `z < 14.2`, then enable X/Y.

**3. Two Z sign inversions.** This was the main reason the task was stuck at ~3.9/20
for many iterations. Because z is depth and `rc_throttle = 1500 − out`, both the trim
term and the velocity feedback had the wrong sign relative to X/Y:

```
wrong:  out = -20 * err_z                      # positive feedback
wrong:  out = Kpv * (vt_z - v_up)              # XY-style sign
right:  out = z_hover_ff + Kpv * (v_up - vt_z) # inverted feedback
```

The fix moved the score from 3.91 to 19.94/20.

**4. Feed-forward fitted at the wrong altitude.** Hover thrust depends on altitude.
Fitting `z_hover_ff` near z≈14.2 gave −28 and parked the drone 0.28 high. Refitting at
the hold altitude gave **−18**, which removed the steady-state bias.

**5. X-only settle scale.** A “slow down near the target” factor applied only to X made
X the limiting axis: it entered the box at 5.27 s while Y entered at ~4.8 s, costing
the last 0.05 speed marks. Removing the asymmetry (both axes use the same law) made
entry repeatable at 4.92–4.95 s and produced 20.00/20.

**6. Integrator wind-up during phase A.** The integrators accumulated while X/Y were
deliberately disabled, injecting a step when tilt was enabled. Fixed by computing
`phase_a` before the integrator update and freezing the integrals while it is active.

**7. Scorer bug (tooling, not flight).** `score.py` compared absolute
`time.monotonic()` values against relative 15 s / 60 s thresholds, so speed and
finishing marks were always zero. Fixed by subtracting `t0`. It had only understated
past results, never inflated them.

### 3.4 Iteration history (selected)

| Run | Score | Change / insight |
|---|---|---|
| baseline | 0/20 | starter GUI PID; D-spike, marker lost at ~4 s |
| it4 | 0/20 | velocity cascade; ground tilt killed the marker |
| it8 | **3.91/20** | XY solid; Z parked at 13.42 (−0.42 bias) |
| it14 | 0/20 | trim sign flipped; bias improved but P-only cannot remove it |
| it15 | **19.94/20** | correct Z cascade + `z_hover_ff = −28` |
| it17 | **19.95/20** | feed-forward refitted to −18 at hold altitude; bias ≈ 0 |
| it18 | **19.95/20** | 62 s run; held 56.20 s |
| it19 | **20.00/20** | X-only settle scale removed |
| it20 / bag2 | **20.00/20** | reproducibility |
| rb1–rb4, rb1–rb3, bag3 | **20.00/20** | 10 consecutive perfect runs |

### 3.5 Final results

- **Score: 20.00/20** — 8/8 hold, 4/4 finishing, 8/8 speed.
- Box entry **4.92–5.02 s**; continuous hold **56.1–56.4 s** of the 60 s window.
- Whole-window error: **P ±0.02, R ±0.02, Z ±0.02**.
- Ten consecutive perfect runs; entry repeatable to ~0.01 s on the deterministic sim.
- The submitted `KD_4373_task_2a.zip` bag independently re-scores **20.00/20**:
  box entry 5.09 s after bag start (3.77 s after the first heartbeat), 58.63 s of
  continuous in-box data, 10 s threshold reached 13.77 s after the heartbeat.

### 3.6 Deliverables

```
KD_4373_task_2a.py      controller (renamed from pico_controller_PID.py)
task_2a_0.db3           bag: /pos_error, /controller_heartbeat, /whycode_node/markers
metadata.yaml
KD_4373_task_2a.zip     flat, 3 files
```

The archived zip is 176,338 bytes. A separate staging bag from the same period also
contains `/drone_command`; the zip's bag has the three topics the 2A scorer needs.

### 3.7 Why the controller was not tuned further

After the settle-scale removal, both X and Y hit the 55 rc tilt clamp at the start of
transit. Entry could be made faster by raising `max_dev` and the velocity cap, but the
margin to the 15.00 s cut-off was already ~0.05 s. Overrunning costs 0.18 marks/s;
destabilising a 10/10-perfect run costs up to 12–20 marks. The trade was rejected.
20/20 is the ceiling.

---

## 4. Task 2B — waypoint navigation

### 4.1 Objective and scoring

Visit four waypoints in the specified order, holding each for **3 s** inside a ±0.4
box on all three axes. Total 40 marks:

- 8 marks per waypoint (reach + 3 s hold) × 4 = 32
- 8 speed marks if the fourth hold completes within 40 s of the controller start;
  otherwise `max(0, 8 − 0.13 × (t − 40))`

Waypoints (WhyCode coordinates):

| # | Grid id | X | Y | Z |
|---|---|---|---|---|
| wp1 | H8 | −2.41 | −2.41 | 13.00 |
| wp2 | D5 | +2.41 | +1.20 | 13.00 |
| wp3 | E8 | +1.20 | −2.41 | 13.00 |
| wp4 | G5 | −1.20 | +1.20 | 13.00 |

### 4.2 Interfaces and package

A new ament_cmake package, `waypoint_navigation`, was created:

```
waypoint_navigation/
├── action/NavToWaypoint.action
├── srv/GetWaypoints.srv
├── CMakeLists.txt
└── package.xml
```

`NavToWaypoint.action` (unchanged for Task 3A):

```
geometry_msgs/Pose waypoint
bool hover
---
int32 hov_time
---
geometry_msgs/PoseStamped current_waypoint
```

`GetWaypoints.srv` (unchanged for Task 3A):

```
bool get_waypoints
---
geometry_msgs/PoseArray waypoints
string[] marker_ids
```

Three nodes in `swift_pico`:

| Node | Role |
|---|---|
| `waypoint_service.py` | serves the four waypoints and their grid ids |
| `pico_client.py` | requests the route, then sends wp1…wp4 one at a time and waits for each result |
| `pico_server.py` | action server + position controller + feedback + hold detection + `hov_time` |

`swift_pico/CMakeLists.txt` installs the scripts **twice**, with and without the `.py`
suffix, so both `ros2 run swift_pico pico_server` and
`ros2 run swift_pico pico_server.py` work.

### 4.3 Control changes from 2A

The cascade was kept; the transit tuning changed:

| Knob | v1 | v2–v4 (submitted) | Meaning |
|---|---|---|---|
| `K_POS_XY` / `V_MAX_XY` | 1.0 / 1.1 | 1.0 / **1.35** | position error → cruise speed |
| `K_RC_XY` / `MAX_DEV_XY` | 120 / 85 | 130 / **100** | velocity error → rc (100 rc ≈ 5.3°) |
| `K_POS_Z` / `V_MAX_Z` | 0.9 / 1.5 | 0.9 / **1.3** | climb rate target |
| `K_RC_Z` / `MAX_DEV_Z` | 120 / 130 | 120 / **150** | vertical velocity error → rc |
| `HOVER_THROTTLE` | 1517 | 1517 | measured hover rc |
| box / hold | 0.35 / 3.0 s | 0.35 / 3.0 s | tighter than the 0.4 box |
| `GOAL_TIMEOUT` | – | **20 s** | abandon a waypoint that cannot settle |
| yaw | – | `YAW_KP = 1.0`, deadband 4° | quantised P yaw controller |

The tilt compensation is worth calling out. A tilt of θ costs `cos θ` of vertical
thrust, so throttle must grow by `1/cos θ`. v1 computed this on the rc value directly,
which was 3× too strong. The correct conversion is:

```
rc_comp = HOVER_THRUST / THRUST_PER_COUNT * (1/cos θ − 1)
```

The old value added ~3 rc of needless climb on every transit and produced a +0.29 z
error; after the fix the transit z error is ≤0.1.

### 4.4 The four bugs that cost runs

1. **Yaw sign.** The downward camera reports marker yaw opposite to airframe yaw.
   A naive P controller on `1501` was positive feedback: the drone spun 0° → 180° in
   7 s and flew off the arena. Inverting the comparison held yaw within ±6° for the
   rest of the project.
2. **Tilt compensation 3× too strong** (above).
3. **A single bad waypoint stalled the mission.** The goal loop waited forever for a
   3 s hold. v3 added `GOAL_TIMEOUT = 20 s`; the client advances to the next waypoint
   on status 6. Verified by temporarily setting the timeout to 3 s: all four
   waypoints were still visited.
4. **The task's own run commands did not work.** The page says
   `ros2 run swift_pico pico_server` (no suffix), but the boilerplate files are
   `*.py`. Installing both names fixed it.

### 4.5 Results

| Build | 4th hold (controller) | 4th hold (bag) | Runs | Score |
|---|---|---|---|---|
| v1 first working | 30.5–31.5 s | 32.4–34.0 s | 4 | 40.00/40 |
| v2 faster transit + tilt fix | 28.1–28.9 s | 30.0–32.0 s | 4 | 40.00/40 |
| v3 timeout + naming | 28.0–29.8 s | 30.1–34.8 s | 5 | 40.00/40 |
| **v4 compliance (submitted)** | **28.5–28.6 s** | **32.1–32.2 s** | 2 | **40.00/40** |

Submitted bag (`task_2b_0.db3`, 34.7 s, 1,347 messages, 1,035 marker fixes at
29.8 Hz, first heartbeat +3.62 s):

| Waypoint | Box entry (bag) | Hold | 3 s hold ends (bag / controller) |
|---|---|---|---|
| wp1 | 7.59 s | 4.22 s | 10.59 / 6.97 s |
| wp2 | 15.45 s | 4.25 s | 18.45 / 14.83 s |
| wp3 | 22.26 s | 4.32 s | 25.26 / 21.63 s |
| wp4 | 29.12 s | 5.59 s | 32.12 / 28.50 s |

**40.00/40** — 4 × 8 reach/hold + 8 speed. The compliance audit
(`audit2b.py`) passed **40/40** checks (interfaces, order, names, client/server
behaviour, CMake/package wiring, untouched 2A controller, five-file zip).

### 4.6 Deliverables

```
KD_4373_waypoint_service.py
KD_4373_pico_client.py
KD_4373_pico_server.py
task_2b_0.db3
metadata.yaml
KD_4373_task_2b.zip     flat, 5 files
```

---

## 5. Task 2C — critical survivor detection and saves

### 5.1 Objective and scoring

Detect the red critical survivors in the arena and Visit each one for **3 s** inside
the ±0.4 box at **Z = 13**. Total 40 marks:

- 8 marks per survivor × 4 = 32
- 8 speed marks if the fourth hold completes within **60 s** of the controller start;
  otherwise `max(0, 8 − 0.2 × (t − 60))`, reaching zero at 100 s

The detected arena used in the validated runs:

| Survivor | WhyCode X | WhyCode Y | Z |
|---|---|---|---|
| E4 | +1.2220 | +2.4388 | 13.00 |
| H5 | −2.4256 | +1.2222 | 13.00 |
| H7 | −2.4221 | −1.2071 | 13.00 |
| D7 | +2.4365 | −1.2074 | 13.00 |

### 5.2 Perception pipeline

`waypoint_service.py` performs the full perception → planning → service chain:

1. **Rectify the arena.** Find the four corner ArUco markers (ids 80/85/90/95),
   compute the homography, warp to a 900 × 900 top-down image.
2. **Detect survivors.** Threshold the red triangles, filter by area/shape, and rank
   candidates by distance to their nearest grid intersection and then by area, so a
   red logo cannot displace a real survivor.
3. **Snap to the grid.** Convert pixel centres to the 11 × 11 named intersections
   with `GRID_MARGIN = 75`, `GRID_STEP = 75`. The mapping is exact; the WhyCode
   coordinates come from
   `grid_whycode_coordinates.json` (1.203414 units per cell).
4. **Plan a tour.** Build a visibility graph over free grid intersections, reject
   segments that cross an obstacle, enumerate survivor orders, and pick the shortest
   collision-free tour.
5. **Serve the route.** Create the `waypoints` service only after a complete,
   validated route exists (3 critical + 3 stable + 3 obstacles in Task 3A; four
   survivors in 2C).

The reference 2C plan from the saved latched frame was:

```
order: E4 -> H5 -> H7 -> D7
route: F6 -> E6* -> E4 -> H4* -> H5 -> H7 -> D7
13 grid steps, ~15.6 whycode units, 2 pass-through intersections
```

A `*` marks a fly-through intersection; a plain id is a stop. Routes are always
planned from the current detection — no arena route is hard-coded.

### 5.3 Planner

- Nodes: every free grid intersection, plus the start point when needed.
- Edges: a segment is allowed only if it does not enter or touch an obstacle polygon.
- Obstacles: convex hull of the black footprint, inflated by
  `OBSTACLE_CLEARANCE_CELLS = 0.65` cells.
- Order: the planner tries every survivor permutation and keeps the shortest tour
  that visits all survivors and ends at the required final cell.
- The planner also handles a survivor sitting on the start cell F6 (it schedules the
  hold before leaving), a correctness gap in the earlier planner.

Planning time is ~0.16–0.2 s on the 11 × 11 graph, small enough to run inside the
service callback.

### 5.4 Hardening pass (2026-10-24)

The 2C code was hardened without changing the flight gains:

**`waypoint_service.py`**

- BOM-safe grid-table loader; accepts `marker_id,x,y` and `x,y,marker_id`, with or
  without a header; rejects invalid cell names and non-finite coordinates; warns on
  an incomplete table.
- ArUco detector created once instead of on every retry.
- Detection retries throttled to 4 Hz so unusable frames cannot steal CPU from the
  30 Hz simulator.
- Frame conversion, detection, planning and overlay errors are caught: one bad frame
  cannot take the node down before the route is served.
- More than four red candidates → rank by grid distance then area.
- Survivor on the start cell handled.
- Service created only after a complete route exists.

**`pico_client.py`**

- Bounded waits: 90 s for the service, 60 s for the action server; a missing
  dependency produces a clear error instead of a silent hang.
- Service request retried up to three times; must return at least one finite pose.
- Missing/short `marker_ids` fail safe with `hover=True` (the old code treated a
  missing id as a fly-through).
- Rejected goals or failed futures do not stall the mission; the client advances.
- Clean SIGINT / `ExternalShutdownException` handling.
- The success line remains `All waypoints have been reached successfully` (the
  official runner greps for it).

**`pico_server.py`**

- Rejects non-finite or out-of-range goals (z outside 0..25).
- If several markers appear, uses the one closest to the last known pose; drops
  non-finite detections and never replaces a good yaw with a non-finite one.
- Box tests guard against non-finite state.
- A goal succeeds only when the hold/pass test actually completed.
- Shutdown publishes a stable hover defensively and only calls `rclpy.shutdown()`
  when the context is alive.

### 5.5 Results

Seven full clean-simulation missions were run with the hardened code; every one
scored **40.00/40**. The final source completed the fourth hold at **35.40 s bag /
31.83 s controller** (sim/wall 1.048). A host-load outlier at sim/wall 0.643 still
finished at 50.19 s controller — inside 60 s, so still 8/8 speed.

Submitted bag (`task_2c_0.db3`, 59.8 s, 2,343 messages, 1,781 marker fixes at
29.8 Hz, first heartbeat +3.22 s):

| Survivor | Box entry (bag) | Hold | 3 s hold ends (bag / controller) |
|---|---|---|---|
| E4 | 9.66 s | 4.13 s | 12.66 / 9.44 s |
| H5 | 17.75 s | 4.10 s | 20.75 / 17.54 s |
| H7 | 23.24 s | 4.01 s | 26.24 / 23.03 s |
| D7 | 30.68 s | 29.11 s | 33.68 / 30.46 s |

**40.00/40** — 4 × 8 reach/hold + 8 speed. The long D7 hold is the final survivor;
the drone remains in the box while the client finishes.

### 5.6 Deliverables

```
KD_4373_waypoint_service.py
KD_4373_pico_client.py
KD_4373_pico_server.py
task_2c_0.db3
metadata.yaml
KD_4373_task_2c.zip     flat, 5 files
```

---

## 6. Task 3A — critical + stable survivors, obstacles, landing

### 6.1 Objective and scoring

Task 3A is the full mission:

1. **Critical phase (55 s).** Three red critical survivors must each be **Visited for
   1 s at Z=13** and then **Saved for 2 s at Z=14**, inside the **0.40-unit
   sphere**. The third Save must complete within **55 s** of the take-off clock.
2. **Switch.** At the third critical Save, the client switches to the stable phase,
   cancels any remaining critical goal, and requests a new route from the drone's
   **actual pose**.
3. **Stable phase.** Three yellow stable survivors must each be **Visited for 1 s at
   Z=13**. Every stable Visit must start **after the Switch**.
4. **Obstacles.** Three black props must not be entered. The trajectory is checked
   against their detected footprints.
5. **F6.** The route ends at F6: hover at Z=13, then descend and land. Hard limit
   **200 s**; full marks at **≤110 s**.

Scoring used by `score3a.py`:

| Check | Pass condition |
|---|---|
| Critical Visit | 1 s continuous inside the 0.40 sphere at Z=13 |
| Critical Save | 2 s continuous inside the 0.40 sphere at Z=14, after the Visit |
| Critical deadline | third Save ends ≤55 s after the first heartbeat |
| Stable Visit | 1 s inside the 0.40 sphere at Z=13, starting after the third critical Save |
| Obstacles | trajectory outside all three detected footprints |
| F6 hover | 1 s inside the 0.40 sphere at Z=13 |
| Landing | final pose on F6 at depth ≥14.65 (floor) |
| Time | touchdown ≤200 s; ≤110 s for full marks |

### 6.2 System architecture

```
                 ┌─────────────────────────────────────────────┐
                 │            waypoint_service.py              │
                 │  rectify → detect 3/3/3 → plan → serve       │
                 │  GetWaypoints (critical)                     │
                 │  PlanStableRoute (stable, from Switch pose)  │
                 └───────────────┬─────────────────────────────┘
                                 │ services
                 ┌───────────────▼─────────────────────────────┐
                 │               pico_client.py                 │
                 │  wait for server → start 55 s clock → fly    │
                 │  critical → Switch → request stable → fly    │
                 └───────────────┬─────────────────────────────┘
                                 │ NavToWaypoint action
                 ┌───────────────▼─────────────────────────────┐
                 │               pico_server.py                 │
                 │  controller + marker gate + Visit/Save       │
                 │  + F6 hover + landing state machine          │
                 └───────────────┬─────────────────────────────┘
                                 │ /drone_command
                             MuJoCo + whycode
```

Nodes and responsibilities:

- **`waypoint_service.py`** — perception, obstacle hulls, visibility graph, both
  route services, debug images/JSON.
- **`pico_client.py`** — mission sequence, 55 s clock, Switch, stable re-plan, F6 goal.
- **`pico_server.py`** — flight control, action server, Visit/Save state machine,
  hover and landing, marker plausibility gate.

### 6.3 Perception

The 2C pipeline was extended to classify three object classes and to serve two routes:

- The four corner ArUco markers rectify the arena to 900 × 900.
- **Critical survivors:** red triangles.
- **Stable survivors:** yellow discs.
- **Obstacles:** solid neutral-black props (trees are green and excluded).
- Every run must find exactly **3 critical, 3 stable and 3 obstacles** before a route
  is published; otherwise the service retries. This was validated on all four arena
  configurations used during development.
- Obstacles are stored as a convex hull of the black footprint plus an inflated
  **off-limits** polygon (`0.65` grid cells of clearance). The planner uses the
  inflated polygon; the scorer reports clearance against both the raw footprint and
  the inflated margin.

The service writes:

```
~/task_3a_capture.png          raw captured frame
~/task_3a_perception.png       rectified + annotated detection
~/task_3a_critical_route.png   critical route overlay
~/task_3a_stable_route.png     stable route overlay
~/task_3a_detections.json      survivors, obstacles, routes for the scorer
```

### 6.4 Planning

**Critical route.** Planned from F6 through the three critical survivors, trying all
`3! = 6` orders and keeping the shortest tour that ends at the last survivor. The
official take's route:

```
F6 -> C5* -> B3 -> C5* -> F4* -> H2* -> I2 -> H2* -> G4* -> J10
24.6 grid steps, ~29.6 whycode units, 6 pass-through intersections
```

(`*` = fly-through; plain id = stop.)

**Stable route.** Planned by `PlanStableRoute` from the drone's actual Switch pose,
through the three stable survivors in the best order, ending at F6. The official
take's request came from `(-4.85, -4.86)` and produced:

```
START(-4.85,-4.86) -> J6 -> G5 -> F2 -> E5 -> C8 -> B10 -> C7 -> F6
25.6 grid steps, ~30.9 whycode units, 4 pass-through intersections
```

Planning took **0.16–0.17 s**. The stable stops also perform Visit+Save because the
server treats every `hover=True` goal as a survivor stop; this is deliberately
conservative and still left a large time margin.

### 6.5 Interfaces and route protocol

Only one new interface was added; the 2B interfaces are unchanged:

**New — `waypoint_navigation/srv/PlanStableRoute.srv`**

```
# Request: the drone's WhyCode pose at the Switch.
geometry_msgs/Pose start
---
# Response: waypoints in flight order, from the Switch through all three
# stable survivors and ending at F6, plus their WhyCode grid marker ids.
geometry_msgs/PoseArray waypoints
string[] marker_ids
```

Route encoding:

| Token | Meaning |
|---|---|
| `C5*` | fly through C5 without stopping |
| `B3` | stop at B3, execute Visit + Save |
| `F6` | home: hover at Z=13, then land |

Protocol:

1. Client calls `GetWaypoints`; service returns the critical route.
2. Client starts the 55 s clock when the action server is ready.
3. Client flies each critical goal in order. A `hover=True` goal performs
   Visit (1 s at Z=13) then Save (2 s at Z=14).
4. At the third critical Save, the client cancels any active goal, waits for the
   server to settle, and calls `PlanStableRoute` with the current pose.
5. Client flies the stable route; all `hover=True` goals Visit+Save.
6. The final F6 goal hovers and lands.

### 6.6 Client state machine

- Waits up to 90 s for `waypoints` and 60 s for the action server.
- Requests the critical route with up to three retries; skips non-finite poses.
- Starts the 55 s clock at **action-server readiness**, not at process launch.
- Sends goals sequentially and waits for each result.
- On the third critical Save: `SWITCH`, cancel the active goal, wait, request the
  stable route, continue.
- If the 55 s deadline expires while a survivor hover goal is in progress, let it
  finish for up to `SAVE_GRACE = 15 s` before cancelling. This preserves the Save
  and defines the Switch on a slow host; fly-through goals still cancel at 55 s.
- On failure: logs the status and advances rather than stalling the mission.
- Success line: `All waypoints have been reached successfully`.

The official client timeline (84.52 s take, rounded from the bag):

```
Take-off clock started
Critical Save 1/3  12.9 s
Critical Save 2/3  25.0 s
Critical Save 3/3  38.0 s
SWITCH             38.0 s
stable route       38.2 s (0.2 s plan)
...
Mission complete   84.9 s
```

The grace branch is never reached: the third Save hold completes at 37.13 s on the
scoring clock, and the client reports it when the action result returns at 38.0 s.
Action-server readiness coincided with the first heartbeat, so the client's own
clock reads 38.0 s at the Switch; the client logs each Save up to ~1.3 s after the
scorer's hold ends.

### 6.7 Controller and landing

The 2B cascade was retained, with four Task 3A changes:

**1. Body-frame velocity rotation.** The marker yaw is used to rotate the desired
world-frame velocity into the body frame before computing roll/pitch. Without this, a
large heading swing turns a position correction into a sideways fly-away.

**2. Yaw damper.** The airframe yaw torque during fast transit was the original spin
cause. The final yaw loop is a rate damper (`YAW_KD = 12`, `YAW_MAX = 350`) plus a
weak heading trim (`YAW_KP = 0.4`, `YAW_KI = 0`) rather than a strong position loop.

**3. Depth filtering and vertical feed-forward.**

```
Z_FILTER_ALPHA = 0.12          EMA on depth for the control loop
Z_VEL_WINDOW   = 0.30 s        baseline for vertical speed
Z_VEL_MIN_SPAN = 0.22 s        minimum window before a velocity is trusted
Z_VEL_ALPHA    = 0.5           EMA on vertical speed
XY_FILTER_ALPHA = 0.50         EMA on the XY control pose/velocity
raw_state      = [x, y, z]     unfiltered marker pose for Visit/Save/hover holds
gate_pos       = [x, y, z]     separate unfiltered pose for the outlier gate
```

Vertical loop:

```
K_POS_Z = 0.60, V_MAX_Z = 0.80
K_RC_Z  = 105,  K_FF_Z  = 40
SAVE_FAST_MARGIN = 1.00        no open-loop dive; proportional loop throughout
SAVE_CTRL_Z = 14.0             hold centre chosen so the raw marker sits at Z=14
```

The horizontal retune that produced the final take is
`K_POS_XY = 0.9`, `V_MAX_XY = 2.20`, `K_RC_XY = 140`, `MAX_DEV_XY = 165`,
`HOLD_RADIUS = 0.38`, `PASS_BOX = 0.48`, `PASS_DWELL = 0.0`. The lower position
gain is what keeps the higher cruise speed from overshooting the terminal sphere.

The first raw-hold take used an open-loop 0.8 m/s dive that engaged 0.5 above the
Save box. It added a large vertical velocity step and could overshoot the box; the
final version keeps the proportional loop in charge (`SAVE_FAST_MARGIN = 1.00`).
Raw depth still oscillates by roughly ±0.4–0.5 near the arena edge, so a stable
Save can take several seconds; the stable stops are flown as Visit+Save anyway
because the portal's speed term counts all six survivors. The final take's stable
Saves hold 2.46 / 2.12 / 2.93 s in the 0.40 sphere.

The final build also separates the pose used for control from the pose used for
grading. The outlier gate and marker selection run on `gate_pos`; the hold tests
run on `raw_state`; only the control loop sees the XY-filtered pose. The XY filter
(`XY_FILTER_ALPHA = 0.50`) removed the terminal oscillation that appeared when
cruise rose to 2.20, while leaving the raw grading path untouched.

The raw-depth derivative at 0.12 s produced ±0.7 m/s of false vertical speed and a
throttle limit cycle; the filtered/windowed estimate removed it. The filter is for
control only. The hold tests use `raw_state`, the sample after the outlier gate and
before the EMA, because the grader scores the marker as published, not the server's
smoothed copy. This distinction is the entire fix described in section 6.11.

**4. Landing state machine.**

```
descend at LAND_V_MAX = 0.72
below LAND_ENGAGE_ALT = 0.40 m → fixed LAND_THROTTLE = 1390
touchdown when depth ≥ TOUCHDOWN_Z = 14.75
   OR depth stalls above TOUCHDOWN_STALL_Z = 14.65 for 1.5 s
hold TOUCHDOWN_HOLD = 0.40 s, then disarm
```

`LAND_THROTTLE = 1390` cannot hover in ground effect, so the drone settles instead of
floating. The stall backstop exists because the detector's rest depth varied from
14.73 to 15.01 across runs; a fixed depth alone can miss a real touchdown. An
isolated `test_land.sh` confirmed a clean disarm at odom body z = 0.175 m.

Marker gate in `pico_server.py`:

```
MAX_MARKER_STEP = 0.35      reject a single jump larger than this
RESYNC_COUNT    = 5         re-sync only after five consistent outliers
```

This protects against a stray blob or a second bridge writing the same topics.

### 6.8 Final run results (v20ref, 2026-11-06)

Arena:

| Class | Survivors / obstacles |
|---|---|
| Critical (red) | **B3, I2, J10** |
| Stable (yellow) | **J6, F2, B10** |
| Obstacles (black) | **D3, I4, E8** |

Timeline (sphere scorer, scoring clock = first `/controller_heartbeat`):

| Event | Time |
|---|---|
| Critical B3 Visit / Save | 7.71–9.80 / 12.98–15.78 s |
| Critical I2 Visit / Save | 23.40–25.31 / 25.78–28.07 s |
| Critical J10 Visit / Save | 36.27–37.86 / 38.05–40.85 s |
| Third Save hold complete (Switch) | **40.05 s** (limit 55 s) |
| Stable J6 Visit / Save | 43.62–45.86 / 46.12–48.69 s |
| Stable F2 Visit / Save | 55.29–57.25 / 57.52–60.36 s |
| Stable B10 Visit / Save | 66.72–68.09 / 69.99–73.03 s |
| F6 hover | 79.19–81.30 s |
| Touchdown | **82.47 s** |

**All objectives PASS; full marks on time (≤110 s).** The third critical Save was
14.95 s inside the 55 s limit, and the first stable Visit began 3.57 s after the
Switch.

Sphere holds at R=0.40 (longest continuous raw run; no gap bridging):

| Survivor | Visit | Save | Save at R=0.38 |
|---|---|---|---|
| B3 | 2.10 | 2.80 | 2.76 |
| I2 | 1.90 | 2.29 | 2.29 |
| J10 | 1.58 | 2.80 | 2.76 |
| J6 (stable) | 2.24 | 2.57 | 2.57 |
| F2 (stable) | 1.96 | 2.84 | 2.70 |
| B10 (stable) | 1.36 | 3.04 | 3.04 |

Every hold passes at R=0.40 and again at R=0.38; the thinnest is B10 Visit at
1.36 s, 0.36 s of slack. The bag has 2,722 marker frames at 30.58 Hz and zero
gaps above 0.10 s.

Obstacle clearance (raw detected footprint / planner's inflated margin):

| Obstacle | Closest raw clearance | Closest inflated clearance | Closest approach |
|---|---|---|---|
| D3 | **+0.57 units** | −0.23 | t=61.93 s, (x=+1.14, y=+2.78) |
| I4 | **+0.70 units** | −0.09 | t=50.57 s, (x=−2.67, y=+1.02) |
| E8 | **+0.94 units** | +0.14 | t=74.80 s, (x=+3.41, y=−1.88) |

D3 and I4 entered the planner's 0.65-cell margin during corner cutting but stayed
outside the actual obstacle footprints; E8 stayed outside even the inflated
margin. The scorer passes on the raw footprint; the margin caveat is recorded in
section 10.

Bag (`task_3a_0.db3`):

```
duration   89.0 s
messages   6,303
topics     /whycode_node/markers   2,722
           /controller_heartbeat      859
           /ros_check               2,722
```

#### 6.8.1 The v17 take (superseded)

The previous submission (v17, 84.52 s) used the same flight logic and passed the
same 12/12 sphere checks at R=0.40 and R=0.38, but its third Save completed at
37.13 s and it spent 47.4 s on the stable phase and landing. The v20ref re-run
lost 2.9 s reaching the Switch but recovered 5.0 s afterwards, for a net
**2.05 s** gain. See section 6.13 for the motion analysis that motivated the
re-runs and the experiments that were rejected.

### 6.9 Packaging and scoring

`score3a.py` reconstructs every objective from the bag:

- reads `/whycode_node/markers` and `/controller_heartbeat`;
- maps WhyCode poses back to rectified pixels using the same grid table and
  `GRID_SPACING`;
- finds continuous runs inside the 0.40 sphere (1 s Visit at Z=13, 2 s Save at Z=14);
- checks Visit-before-Save and the 55 s deadline;
- checks stable Visits start after the third Save;
- checks the trajectory against each obstacle's raw hull and inflated margin;
- finds the F6 hover and the final landing run;
- reports the 110 s / 200 s time limits.

The official bag was scored **PASS** before packaging by both `score3a.py` and a
strict variant, `score3a_strict.py`, that sets the run-bridging gap to zero. The
strict pass is the conservative proxy for the grader: it requires every Visit/Save
hold to be continuous on raw marker samples with no interpolation across dropouts.
`package_task_3a.sh` runs both scorers and refuses to zip a bag that fails either,
stages exactly five files, compiles the Python, checks for empty files and launch
files, builds the flat zip, and copies it to `C:\Users\varun\KD_4373_task_3a.zip`.

Final submission:

```
KD_4373_task_3a.zip
  KD_4373_waypoint_service.py
  KD_4373_pico_client.py
  KD_4373_pico_server.py
  task_3a_0.db3
  metadata.yaml
```

Size **624,969 bytes**; SHA-256
`9f194abfc00332a95f6816686f5bc55ffb14f0d7b3fad20c8d4ee4cd9433aaf0`.
No launch file, no folder nesting, no empty files.

The packaged source uses `OBSTACLE_CLEARANCE_CELLS = 0.65`, the baseline that
produced this bag, and `SAVE_FAST_MARGIN = 1.00`, which disables the old
open-loop 0.8 m/s Save dive and leaves the proportional Z loop in charge. The
client carries the 15 s Save grace; the bag's third Save hold completes at
40.05 s, so the grace branch is never reached and the fast path is unchanged. The
server's control loop still uses the filtered pose; only the Visit/Save/hover
hold tests use `raw_state`. The v20ref take changes only
`KD_4373_pico_server.py`, and only by adding six lines of sphere-entry logging;
the client and waypoint service are byte-identical to v17.

### 6.10 Official runner

`official_run.sh`:
1. 20 s countdown for the operator to start the screen recorder.
2. Starts the simulation; waits for markers; verifies exactly one `mujoco_swift_pico`
   and one `whycode_node` (duplicate bridges corrupt the pose).
3. Hides the MuJoCo render window so it cannot cover the terminal tiles.
4. Starts the lightweight compressed camera viewer in the bottom-right tile.
5. Starts the waypoint service and waits for `route ready`.
6. Starts the client, then the bag, then the server **last** (the server starts the
   mission).
7. Waits for the client's success line; closes the bag; copies
   `task_3a_detections.json` beside it; scores the run; prints
   `MISSION COMPLETE - STOP THE RECORDING NOW`.

Bag topics are deliberately limited to `/whycode_node/markers`,
`/controller_heartbeat` and `/ros_check`.

### 6.11 Grader post-mortem and the raw-hold fix

The first Task 3A submission scored **34.76/80**. The grader reported:

| Check | Result |
|---|---|
| Critical B3 | visited 7.80 s; best Save hold **1.57 s** of 2.00 s (3/8) |
| Critical I2 | visited 21.73 s; best Save hold **1.36 s** of 2.00 s (3/8) |
| Critical J10 | visited 35.47 s; best Save hold **1.39 s** of 2.00 s (3/8) |
| Stable B10 | visited 72.04 s, saved 75.07 s (8/8) |
| Stable F2 | visited 56.43 s, saved 62.83 s (8/8) |
| Stable J6 | visited before the Switch only (0/8) |
| Critical window | ended; Switch at **55.00 s** |
| F6 hover / landing | hover at 82.49 s; landed at 84.87 s |
| Survivors / hover / landing / speed | 25/48, 3/3, 3/3, 3.76/26 |

The local scorer had passed the same bag with three 2 s Saves and a 39.82 s
Switch. The discrepancy was not the hold length or the box; it was the signal
used to test the hold. `pico_server.py` kept

```
curr_state = [x, y, z_filt]        # z_filt = EMA(z, alpha=0.12)
is_drone_in_box(curr_state, target, BOX=0.35)
```

for Visit, Save and hover. The grader reads `/whycode_node/markers` directly.
Replaying the old bag showed that the raw marker's longest continuous in-box
run was only **1.37 / 1.75 / 1.53 s** in the grader's ±0.4 box, while the
filtered state used by the server was smooth enough to complete its 2 s test.
The grader's 1.57 / 1.36 / 1.39 s are those same holds sampled at its own
rate. The EMA hid raw excursions of 0.3–0.6 s in the middle of every "2 s"
Save, so the third Save never completed and the Switch fell to the 55 s
deadline. J6 then fell before the Switch.

**Fix.** Immediately after the marker outlier gate, the server stores the
unfiltered sample:

```
self.raw_state = [x, y, z]
```

The Visit, Save and F6-hover hold tests use `raw_state`; the control loop keeps
using the filtered `curr_state`. The box (0.35 at the time of the fix, 0.37 in
the retuned build), the holds (1.0 s Visit, 2.0 s Save), the route and the client
are unchanged. Lowering `SAVE_Z` to 13.85 was considered and rejected: the
server's box around 13.85 would extend below the grader's 13.6 lower bound, so
the server could accept a Save the grader rejects. Extending the true hold at
`SAVE_Z = 14.0` is the correct direction; the retune targets `SAVE_CTRL_Z = 13.92`
inside the measured 13.60–14.34 band.

**Validation.** `score3a_strict.py` is `score3a.py` with the run-bridging gap
set to zero in `find_runs` and `good_runs`. It does not interpolate across a
marker dropout, so it is the conservative proxy for the grader. On the final v17
bag every check passes:

| Check | Strict result |
|---|---|
| Critical B3 Visit / Save | 7.99–8.99 / 10.28–12.28 s, PASS |
| Critical I2 Visit / Save | 20.50–21.50 / 22.82–24.82 s, PASS |
| Critical J10 Visit / Save | 32.77–33.77 / 35.13–37.13 s, PASS |
| Third Save / Switch | **37.13 s** (limit 55 s), PASS |
| Stable J6 / F2 / B10 Visits | 40.99–44.16 / 54.83–55.74 / 69.26–72.59 s, all after Switch |
| Obstacles D3 / I4 / E8 | +0.47 / +0.47 / +0.76 raw clearance, PASS |
| F6 hover / touchdown | 80.22–83.52 s / **84.52 s**, PASS |
| Time | 84.52 s ≤ 110 s, full marks |

The longest continuous runs in the 0.40 sphere are B3 **1.92 / 2.71 s**, I2
**1.89 / 2.28 s**, J10 **1.93 / 2.80 s**, J6 **1.79 / 2.46 s**, F2
**2.00 / 2.12 s**, B10 **1.96 / 2.93 s** (Visit / Save). Every Save also passes
at the tighter R=0.38 (2.61 / 2.14 / 2.80 / 2.46 / 2.05 / 2.57 s), and every
Visit is at least 1.79 s at R=0.38. The bag has 2,762 marker frames and zero
gaps above 0.10 s. The thinnest margin is F2 Save: 2.12 s at 0.40 and 2.05 s at
0.38.

**Score projection.** The reported 34.76 is exactly reproduced by the
decomposition

```
critical  3 * 8 * (3/8)              =  9.00
stable    8 + 8 + 0                  = 16.00
hover                                  3.00
landing                                3.00
speed     26 * (150-84.87)/150 * 2/6 =  3.76
                                       ------
                                       34.76 / 80
```

so the 80-point total is 24 critical (8 per survivor), 24 stable (8 per
survivor), 3 hover, 3 landing and 26 speed. The speed term that reproduces the
reported 3.76 is

```
speed = 26 * (150 - landing_time) / 150 * (survivors_saved / 6)
```

For the final run, all three critical saves are full 8/8, all three stable
visits are full 8/8, hover and landing are 3/3, and

```
speed = 26 * (150 - 83.98) / 150 * 6/6 = 11.44/26
```

Total **24 + 24 + 3 + 3 + 11.44 = 65.44/80**. The landing clock is the strict
84.52 s minus the 0.54 s offset measured on the v10 take (strict 79.50 s,
portal 78.96 s). The same offset on the golden take gives 86.76 s → 10.97 speed
marks → **64.97/80**, exactly the portal's report, so the offset and the formula
are both anchored in portal evidence. The exact total is grader-side; the bag
passes every check the local scorers can reconstruct.

The v10 take (79.50 s) was submitted and scored **49.21/80**, not the 66.22
projected at the time. Section 6.12 explains why and what replaced it.

### 6.12 The v10 submission and the sphere criterion

The portal's 49.21/80 report for the 79.50 s take contained two failures:

- stable B10: visit hold too short: best **0.79 s** of 1.00 s (0/8);
- stable J6: save hold too short: best **1.70 s** of 2.00 s (3/8).

The local per-axis box scorer had passed the same bag. Replaying it against a
**3-D Euclidean sphere of radius 0.40** reproduced both reported numbers exactly.
The golden 87.30 s bag passes **48/48** sphere holds, so the sphere is the
grader's criterion, not a guess or a fit to one failure.

The two v10 failures had different mechanisms:

- **B10 Visit.** The approach entered the ±0.4 box on X and Y but stayed more
  than 0.40 units away in 3-D, because the depth was off-centre. The box gave
  credit; the sphere did not.
- **J6 Save.** The hold was split by a single raw depth sample at bag
  t=51.74 s (`z=14.50`, distance 0.505). The server's outlier gate had rejected
  that sample and kept its previous `raw_state`, so the server declared the Save
  complete while the bag still contained the outlier. A naive replay sees a
  1.70 s run, exactly what the portal reported.

The fix has three parts:

1. **Hold test.** The server tests Visit, Save and F6 hover on the unfiltered
   `raw_state` inside a **R=0.38 sphere**, 0.02 inside the grader, with a
   `HOLD_FRESH=0.15 s` freshness gate so a stale sample cannot extend a hold
   indefinitely.
2. **Hold length.** `VISIT_HOLD=1.10 s`, `SAVE_HOLD=2.10 s`, `HOME_HOLD=1.10 s`.
   A completed R=0.38 hold is structurally longer than the portal minimum at
   R=0.40, unless a rejected outlier is present in the bag.
3. **Separate poses.** `gate_pos` feeds the outlier gate and marker selection;
   `raw_state` feeds grading; only the XY-filtered pose feeds control. This
   prevents a rejected sample from corrupting the hold while keeping the flight
   smooth at the higher cruise speed.

Every candidate bag is scored by naive sphere replay before packaging; console
success is not evidence. Between 2026-11-03 and 2026-11-05 the build recorded
**84.52 s (v17, shipped)**, 85.67 s (v14_a), 86.31 s (v13_b), 85.40 s (v15,
rejected: J6 Save 1.40 s from a single outlier), 102.20 s (v16, failed I2) and
105.96 s (v18, clean but slow). The v15 case is exactly the server-PASS /
bag-FAIL split described above. The remaining speed limit is run-to-run z-phase
variance: Save-entry gaps vary from 0.3 to 0.6 s, and one clean build lost 20 s.
v17 is the fastest verified bag; the sphere-hold server is the durable artifact.

The 85.82 s first-submission bag is archived at `/home/varun/kd3a/task_3a_old/`
and in the backup's `official_bag_old/`; the previous 96.55 s raw-hold take is
preserved in `KD_4373_task_3a_9655_fallback.zip`, the 93.24 s no-dive take in
`KD_4373_task_3a_9324_fallback.zip`, and the 87.30 s golden take in
`KD_4373_task_3a_8730_golden.zip`. The 15 s Save grace is retained as a slow-host
safety net but is not exercised: the third Save hold completes at 37.13 s.

The v10 speed retune that produced the previous take changed only
`KD_4373_pico_server.py`: `K_POS_XY 1.0→0.9`, `V_MAX_XY 1.60→1.80`,
`K_RC_XY 130→140`, `MAX_DEV_XY 150→165`, `K_RC_Z 80→105`, `MAX_DEV_Z 100→120`,
`BOX 0.35→0.37`, `SAVE_CTRL_Z = 13.92`, `LAND_ENGAGE_ALT 0.35→0.40`,
`LAND_THROTTLE 1400→1390`, `LAND_V_MAX 0.60→0.72`, `TOUCHDOWN_Z 14.80→14.75`,
`TOUCHDOWN_HOLD 1.0→0.40`, `PASS_BOX 0.50→0.48`, `PASS_DWELL 0.10→0.00`. It cut
7.80 s from the golden take and passed the old box scorer, but the portal scored
it 49.21/80 because B10's Visit and J6's Save were short in the sphere. It is
history, not the submitted take.

The v17 build supersedes it with `V_MAX_XY = 2.20`, `XY_FILTER_ALPHA = 0.50`,
`HOLD_RADIUS = 0.38`, `VISIT_HOLD = 1.10`, `SAVE_HOLD = 2.10`,
`HOME_HOLD = 1.10`, `SAVE_CTRL_Z = 14.0` and the separate `gate_pos` /
`raw_state` split. The faster cruise is what recovers the 5 s lost to the longer
holds; the XY filter keeps the terminal approach stable at that speed.

Six further speed attempts were rejected. A z-reference trajectory (rate 0.9,
gain 2.0, feed-forward on the reference derivative) outran the actual depth and
pushed the third Save to 53.9 s (111.5 s total). Lowering
`OBSTACLE_CLEARANCE_CELLS` to 0.45 shortened the planned tour, but the
corner-cutting drone stalled between F2 and B10 and four goals timed out. An
offline re-plan at 0.55 cells saved only 0.38 units, so it was never flown.
`SAVE_CTRL_Z = 13.85` lost F2's 2 s Save. `PASS_BOX = 0.58` produced 89.2 s with
no gain. `K_POS_XY = 1.0` at 1.8 m/s caused 5–6 F2 hold resets (84.7/85.2 s), and
a per-phase `K_POS_XY_PASS = 1.30` reached 79.6 s but collapsed E8's inflated
clearance to +0.013 and made Save holds unreliable. The 0.65-cell planner, the
0.9 position gain and the no-dive proportional loop are the validated
configuration.

Scoring caveat: `score3a.py` and `score3a_strict.py` print **PASS/FAIL per
objective**, the third-Save time, the stable-Visit windows, the obstacle
clearances, the F6 hover/landing, and the 110 s / 200 s time bands. They do not
print "80 / 80" or "26 / 26". The 65.44/80 figure is rubric arithmetic applied to
the printed checks, not a total reproduced by the scorer.

A second, subtler caveat: when marker samples are rejected or dropped, the
server keeps counting hold time on the last stored `raw_state`. It can therefore
declare a Save complete while the recorded bag contains a shorter continuous
run. A 75.67 s v10 run showed exactly this failure (J10's raw run was 1.66 s
against a 2.00 s requirement), and v15's J6 Save was split to 1.40 s by one
rejected sample. Every candidate bag is therefore checked with `saveruns.py` at
the bag level, and the final bag is the fastest one whose twelve sphere holds all
pass at R=0.40 and R=0.38.

---

### 6.13 Motion analysis from the recorded screen run

The user asked for the whole process to be recorded, then analysed, because the
drone appeared to overshoot when moving between points. The mission was re-run
with a Windows-side `mss` screen recorder capturing the MuJoCo view at 1920×1080.
The recording produced 1,050 frames over 174.83 s (6.00 fps) and the flight
itself took 108.36 s at 28.44 Hz, versus 82.47 s for the clean bag. The extra
load is real, so the video is diagnostic evidence only; it is not a candidate
submission.

The recorded trajectory was plotted in `traj_baseline_xy.png` and
`traj_baseline_speed.png`. The baseline accelerates to 2.2–2.9 units/s on the
long legs, then spends the last 0.5–1.0 unit of each approach at low speed. Stop
overshoot past the survivor centre is 0.33–0.72 units. The final controller
reduces that to **0.15–0.43 units**:

| Survivor | Approach speed | Stop overshoot | Save Z overshoot |
|---|---|---|---|
| I2 | 1.15 u/s | +0.147 | +0.28 |
| B3 | 1.50 u/s | +0.257 | +0.28 |
| J10 | 2.00 u/s | +0.357 | +0.27 |
| F2 | 1.93 u/s | +0.425 | +0.26 |
| J6 | 2.02 u/s | +0.181 | +0.29 |
| B10 | 1.72 u/s | +0.307 | +0.17 |

The residual XY overshoot does not by itself lose the hold: the drone passes
through the 0.40 sphere before the apex and the sphere replay still gives
1.36–2.24 s Visits and 2.29–3.04 s Saves. The costly transient is vertical. After
the Save dive the depth loop rings around `Z = 14`; time from the dive to a
settled Save is B3 **4.27 s**, F2 **2.63 s**, B10 **2.31 s**, J6 **1.50 s**,
I2 **1.36 s**, J10 **0.84 s**, and every re-entry restarts the 2 s sphere hold.

Three fixes were flown and rejected:

| Experiment | Result | Decision |
|---|---|---|
| `Z_DEADBAND = 0.18` | Isolated B3 descent clean (zmax 14.06, rebound 13.96); full run **92.29 s** because a ±150° yaw spin cost ≈10 s | Reverted |
| `BRAKE_DECEL_XY = 0.85` vector braking | Isolated B3 stop **12.2 s vs 9.2 s** baseline, confounded by a yaw spin to +171° | Reverted |
| `YAW_MAX = 480` + level-out guard | **Mission failed**: overshot J10 to (−6.22, −7.36), yaw rate 80.9°/s with `rc_yaw = 1020`; three goals aborted, exit 1 | Reverted |

The yaw data explains the variance. The 82.47 s take stayed within
−12.2°..+25.9° with a 38.3°/s peak; the 92.29 s same-code run swung
−153.5°..+147.3° with a 108.2°/s peak on the J6→F2 leg. The spin is a simulator
disturbance, not a tuning error, and the guard fought it badly enough to lose the
mission. The final decision is therefore to keep the v17 flight logic unchanged,
select the take by bag-level sphere replay, and not chase the spin with a new
controller mode. The screen video remains the evidence for the overshoot and
ringing analysis.

---

## 7. Cross-task design evolution

| Layer | Task 2A | Task 2B | Task 2C | Task 3A |
|---|---|---|---|---|
| Controller | `pico_controller_PID.py`, cascade + Z fixes | moved into `pico_server.py`, transit tuning, yaw | unchanged gains, hardened | body-frame rotation, yaw damper, depth filter, landing |
| Perception | none | none (fixed waypoints) | ArUco rectification + red survivor detection + grid snap | 3 classes (critical/stable/obstacle), 3/3/3 validation |
| Planning | none | static service | visibility graph + survivor tour | critical tour + stable re-plan from Switch pose + obstacle inflation |
| Interfaces | none | `NavToWaypoint`, `GetWaypoints` | same | same + `PlanStableRoute` (only new interface) |
| Mission logic | single setpoint | 4 sequential goals | 4 survivor goals | 55 s critical clock, Switch, stable route, F6 hover+land |
| Validation | `/pos_error` monitor + scorer | bag scorer + 40-point audit | bag scorer + 7 full runs | bag scorer + official take + obstacle analysis |
| Operations | manual launches | manual + final_run | tiled official runner, bag, camera viewer | tiled runner + single-bridge preflight + auto-score + packager |

The important architectural decision was to keep the **2B interfaces unchanged** in
3A and add only one service. That kept the action server, client goal loop and
external tooling stable while the mission logic grew.

---

## 8. Reproduction guide

### 8.1 Build

```bash
cd ~/pico_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select waypoint_navigation swift_pico
source install/setup.bash
```

`waypoint_navigation` must be rebuilt whenever `PlanStableRoute.srv` changes.
`swift_pico` installs **copies** of the Python nodes in the Task 3 release, so editing
a source requires a rebuild (the 2A/2B `--symlink-install` shortcut no longer applies).

### 8.2 Sanity checks

```bash
bash /mnt/c/Users/varun/KD_4373_task_3a_final/scripts/hardening_check.sh
```

It compiles the three sources, compares them with the installed executables, shows
the stable-route interface, and scans for hard-coded paths.

### 8.3 Full-mission test

```bash
bash ~/task3_work/test_mission.sh
```

Random arena, single-bridge preflight, full bag, score.

### 8.4 Isolated landing test

```bash
bash ~/task3_work/test_land.sh
```

Take-off at F6, hover, descend, land; checks the final odometry pose.

### 8.5 Official take

```bash
bash /mnt/c/Users/varun/KD_4373_task_3a_final/scripts/official_run.sh
```

Output bag: `/home/varun/kd3a/task_3a/`; live logs: `/tmp/kd3a/`.

### 8.6 Score and package

```bash
python3 /mnt/c/Users/varun/KD_4373_task_3a_final/scripts/score3a.py \
    /home/varun/kd3a/task_3a/task_3a_0.db3 \
    --detections /home/varun/kd3a/task_3a/task_3a_detections.json

bash /mnt/c/Users/varun/KD_4373_task_3a_final/scripts/package_task_3a.sh
```

The packager scores the bag first and aborts on failure; it then builds the flat zip
and copies it to `C:\Users\varun`.

### 8.7 Task 2 reproductions

- **2A:** `~/kd_task2a_docs/evalrun.sh` and `score.py`; the final controller is
  `KD_4373_task_2a.py` in `~/kd2b/`.
- **2B:** `~/kd2b/final_run.sh`, `score2b.py`, `audit2b.py`.
- **2C:** `~/kd2c` official runner and `package_task_2c.sh`; the hardened copy is in
  `C:\Users\varun\KD_4373_task_2c_final`.

---

## 9. Verification and artifact inventory

### 9.1 Task 3A submission (current)

| File | Size | SHA-256 |
|---|---|---|
| `KD_4373_waypoint_service.py` | 52,740 | `36320a964b48fed1e3048ea7a4c18d5c37ffc371aacad4842dfa5884857b9c94` |
| `KD_4373_pico_client.py` | 18,345 | `677249c3c8a48ab3943bf4e62f225d391b93e77f2627fd63b6125c1776c6d825` |
| `KD_4373_pico_server.py` | 49,748 | `c141fcdc0256801cc4b3350d06bf1ef45b928a17986e8eeb4860568f51eea64b` |
| `task_3a_0.db3` | 1,728,512 | `1695b3146ff1e24fed845ab9e7ba513616bf01d14851e32d61f817e092016262` |
| `metadata.yaml` | 1,978 | `892abc8fc6e46818f7e0c90f8bdf67e98aa66882fc854f738d71b73ceb4617d1` |

Zip size **624,969 bytes**; SHA-256
`9f194abfc00332a95f6816686f5bc55ffb14f0d7b3fad20c8d4ee4cd9433aaf0`.

### 9.2 Current Task 3A folders

| Path | Contents |
|---|---|
| `C:\Users\varun\KD_4373_task_3a.zip` | official flat submission (82.47 s, v20ref) |
| `C:\Users\varun\KD_4373_task_3a_8730_golden.zip` | previous 87.30 s package, preserved unchanged |
| `C:\Users\varun\KD_4373_task_3a_unzipped\` | unpacked copy of the five official files |
| `C:\Users\varun\KD_4373_task_3a_9324_fallback.zip` | previous 93.24 s package, preserved unchanged |
| `C:\Users\varun\KD_4373_task_3a_9655_fallback.zip` | previous 96.55 s package, preserved unchanged |
| `C:\Users\varun\KD_4373_task_3a_final\` | backup of the v10 package: sources, scripts, evidence, v10 + older bags, notes, checksums |

### 9.3 Earlier submissions

| Task | Zip | Bag | Score |
|---|---|---|---|
| 2A | `KD_4373_task_2a.zip` (176,338 B) | `task_2a_0.db3` (64.7 s) | 20.00/20 |
| 2B | `KD_4373_task_2b.zip` (99,329 B) | `task_2b_0.db3` (34.7 s) | 40.00/40 |
| 2C | `KD_4373_task_2c.zip` | `task_2c_0.db3` (59.8 s) | 40.00/40 |
| 3A | `KD_4373_task_3a.zip` (624,969 B) | `task_3a_0.db3` (89.0 s) | all objectives PASS; 65.80/80 projected |

### 9.4 What was checked before submission

- Sources compile (`py_compile`) and are byte-identical to the installed executables.
- `PlanStableRoute.srv` is installed and listed by `ros2 interface show`.
- No hard-coded `/home/...`, `/mnt/...`, `localhost`, `127.0.0.1`, `TODO` or `FIXME`
  in the three submitted sources.
- The zip contains exactly five flat, non-empty files and no launch file.
- The zip payload is byte-identical to the backup sources and official bag.
- The bag contains exactly the three intended topics.
- `score3a.py` passes every objective on the packaged bag.
- `score3a_strict.py` (no gap bridging) also passes every objective, including
  all twelve Visit/Save holds at the 0.40 sphere and again at 0.38.
- The bag has zero marker gaps above 0.10 s across 2,722 frames.
- `SHA256SUMS.txt` verifies every backup file and the delivered zip.

---

## 10. Known limitations and caveats

1. **Obstacle margin cuts on D3 and I4.** The final trajectory stayed outside the
   actual detected footprints (+0.57 and +0.70 units) but entered the planner's
   0.65-cell inflation by 0.23 and 0.09 units while corner-cutting. E8 cleared the
   raw footprint by +0.94 and stayed outside the inflated boundary by +0.14. The
   scorer passes on the raw footprint. If the grader uses the inflated polygon,
   those passes would be treated as close; the plan is conservative, but the
   controller's corner cutting consumes part of that margin.
2. **Depth is the noisiest channel.** The final take rested at 14.88 (server log
   14.85), 0.23 above the 14.65 floor; rest depth varied 14.73–15.03 across runs.
   The landing relies on the stall backstop as much as the fixed 14.75 threshold.
3. **Stable stops also Save.** The server treats every `hover=True` goal as a
   Visit+Save. This is deliberate: the portal's speed term multiplies by
   `survivors_saved / 6`, so Visit-only stable stops would halve the speed marks.
   The stable Save uses the same 2.10 s server hold at R=0.38; in the final take
   the three stable Saves run 2.57 / 2.84 / 3.04 s in the 0.40 sphere. The task
   text requires only the stable Visit.
4. **Camera tile changed after the validated take.** `official_run.sh` moved the
   viewer to `VIEW_X=1400 VIEW_Y=480 VIEW_SIZE=430` because the old tile clipped the
   camera window by ~14 px right / 26 px bottom. `bash -n` passes, but the new tile
   has not been visually confirmed in a full mission. The mission code is unchanged.
5. **One bridge only.** Running two simulations is the most common way to corrupt a
   take. The official runner aborts if it sees duplicate nodes.
6. **2A archive topic set.** The `KD_4373_task_2a.zip` on disk contains
   `/pos_error`, `/controller_heartbeat` and `/whycode_node/markers`; a staging bag
   from the same period also has `/drone_command`. The 2A scorer only needs
   `/pos_error`; if a future submission requires `/drone_command`, repackage from the
   staging bag.
7. **Task 3A video is still external.** The official bag and score are complete; the
   unlisted YouTube recording (`KD_4373_Task_3a`) is handled by the operator.
8. **The recorded take overwrites the bag.** A new official run writes to
   `/home/varun/kd3a/task_3a`; the zip and backup must then be rebuilt and re-hashed.
9. **The grace only helps a late Save.** `SAVE_GRACE = 15 s` is a robustness
   measure for slow hosts, not a way to meet the 55 s objective: a Save that
   finishes after 55 s is still scored LATE. It was validated at a 0.806 sim rate
   (Rebuild F); on the official fast setup the branch is never reached.
10. **The hold criterion is a 0.40 sphere, verified from portal feedback.** The portal reported B10 Visit 0.79 s and J6 Save 1.70 s for a bag that passed the old per-axis box scorer; the same bag replayed against a 3-D sphere of radius 0.40 reproduces both strings exactly, and the golden bag passes 48/48 sphere holds. The server tests its own holds at R=0.38, 0.02 inside the grader, with no stale-pose credit. The final bag passes all 12 holds at R=0.40 and again at R=0.38; the thinnest is B10 Visit at 1.36 s (1.36 s at 0.38, 0.36 s of slack).
11. **Console success can precede bag success.** When marker samples are rejected
    or dropped, the server keeps counting hold time on the last stored
    `raw_state`. A 75.67 s v10 run looked complete on the console but its J10 raw
    run was only 1.66 s; v15's J6 Save was split to 1.40 s by one rejected
    `z=14.50` sample. Bags are therefore selected by `saveruns.py`, not by the
    console; the final bag is the fastest whose twelve sphere holds all pass.
12. **Host load changes the result.** At a 28 Hz marker rate (versus 30.6 Hz for
    the final take) the identical build flew 95–108 s and showed pose glitches up
    to 1.27 units. The final bag comes from a clean 30.6 Hz session; re-recording
    it on a loaded host is not equivalent.
13. **Yaw-spin events dominate the remaining variance.** The simulator
    occasionally injects an uncommanded yaw excursion. The 82.47 s take stayed
    within −12.2°..+25.9° at a 38.3°/s peak; a same-code 92.29 s take swung
    −153.5°..+147.3° at 108.2°/s and lost ≈10 s. A level-out guard was tried and
    caused a mission failure (J10 overshoot to (−6.22, −7.36), `rc_yaw = 1020`,
    three goals aborted), so the spin is left to the normal controller recovery.
14. **The screen recording is diagnostic, not a scored bag.** The `mss` capture
    ran at 6 fps and loaded the host enough to slow the mission to 108.36 s at
    28.44 Hz. It is used for the overshoot/ringing analysis in section 6.13; the
    official bag remains an unrecorded 30.6 Hz run.

---

## 11. Timeline

| Date | Milestone |
|---|---|
| 2026-10-18 | Task 2A Z sign fixes; 20.00/20; ten consecutive perfect runs |
| 2026-10-19 | Task 2A zip finalized (`KD_4373_task_2a.zip`) |
| 2026-10-20 | Task 2B package, interfaces, v1–v4; 40.00/40; audit 40/40; zip |
| 2026-10-24 | Task 2C hardening; seven 40/40 runs; official runner |
| 2026-10-25 | Task 2C submission bag and zip |
| 2026-10-27 | Task 3 release pulled/built; perception validated 3/3/3 |
| 2026-10-28 | Task 3A planner, client, landing, official take, score, zip, backup |
| 2026-10-30 | Grader 34.76/80 post-mortem; raw-marker hold fix; strict scorer; first raw-hold take at 96.55 s |
| 2026-10-31 | Open-loop Save dive disabled; 93.24 s take; same validated build re-run at 87.30 s; zip rebuilt; 64.97/80 reported by the portal |
| 2026-11-01 | Controller retune campaign; v10 (0.9 gain, 1.8 m/s, soft approach) validated at 79.50 and 80.78 s; per-phase gain, 0.55-cell and 13.85 cm Save variants rejected |
| 2026-11-02 | Bag-level Save audit; 79.50 s submitted (66.22/80 projected); docs, hashes and zip rebuilt |
| 2026-11-03 | Portal scored the 79.50 s take **49.21/80** (B10 Visit 0.79 s, J6 Save 1.70 s); the 0.40 sphere criterion was reverse-engineered and verified against the golden bag (48/48) |
| 2026-11-04 | Sphere-hold server (R=0.38, 0.15 s freshness), XY control filter, separate `gate_pos`; v13_b 86.31 s, v14_a 85.67 s, v15 rejected (J6 Save 1.40 s), v16 failed I2 |
| 2026-11-05 | v17 recorded at **84.52 s**; 12/12 sphere holds at R=0.40 and R=0.38; five-file zip rebuilt and re-scored from the extracted archive |
| 2026-11-06 | Full mission recorded at 1920×1080 for motion analysis; overshoot (0.15–0.43 units) and Save-depth ringing (0.8–4.3 s) quantified; Z-deadband, braking-cap and yaw-guard experiments rejected |
| 2026-11-06 | v20ref re-run at **82.47 s**; 12/12 sphere holds at R=0.40 and R=0.38; v20ref becomes the official take and the five-file zip is rebuilt |
| 2026-11-07 | Report, README and review prompt updated to v20ref; official bag, hashes and zip re-verified; repository force-pushed |

---

## 12. Lessons learned

### Control

- A cascade is mandatory on this plant. Position-only P loops ring; velocity feedback
  is the inner loop that makes the outer loop stable.
- Sign conventions are part of the model. Two Z sign inversions looked like “bad
  tuning” for a dozen runs. Draw the plant sign table before tuning.
- Feed-forward is not a hack. A P/I law that outputs zero at zero error cannot supply
  the hover thrust; fit the feed-forward at the altitude you intend to hold.
- Filter for the loop you are closing. Raw depth differentiation poisoned Z; the
  windowed/EMA estimate fixed it without changing the plant.
- Score the signal the grader scores. A filter that smooths the control loop can
  hide raw excursions in the scoring path; keep a separate raw state for hold
  tests and prove the hold on that state, not on the filtered copy.
- Landing needs a state machine, not a setpoint. Ground effect makes a fixed hover
  throttle unable to settle; a low fixed throttle plus a stall backstop is repeatable.

### Perception and planning

- Rectify first, then reason in grid coordinates. The 900 × 900 homography and the
  named 11 × 11 grid made every route and every scorer check reproducible.
- Validate the detector on every arena configuration before trusting a route. 3/3/3
  was a hard gate, not a warning.
- Inflate obstacles for planning, but score against the raw footprint too. Report both.
- Re-plan from the actual Switch pose. A route planned from take-off is wrong after
  40 s of flight.

### Software and operations

- Bags are the ground truth. Every claim in this report is reconstructible from a
  `.db3` file.
- Score the bag before packaging; refuse to package a failing run.
- Keep interfaces stable. Task 3A added one service and left the action and the
  critical service untouched.
- Preflight for duplicate bridges. A stale second simulator is silent and corrupting.
- Record the bag before the controller starts, and copy the detection JSON beside it.
- Keep a flat, five-file submission and verify it mechanically (names, sizes, hashes,
  no launch file).

---

## Appendix A — constants, gains, and geometry

### Arena / grid

```
SIZE          = 900 px
GRID_MARGIN   = 75 px
GRID_STEP     = 75 px
GRID_SPACING  = 1.203414 whycode units / cell
GRID_COUNT    = 11 (A..K, 1..11)
HOME_CELL     = F6  (450, 450) px
Z_WAYPOINT    = 13.0
Z_SAVE        = 14.0
```

### Task 3A controller

```
RC_MIN/MAX          = 1000 / 2000
RC_MID              = 1500
CAMERA_HEIGHT       = 15.0
HOVER_THROTTLE      = 1518
K_POS_XY            = 0.9
V_MAX_XY            = 2.20
K_RC_XY             = 140
MAX_DEV_XY          = 165
K_INT_XY            = 0.4
INT_LIMIT_XY        = 20
INT_BAND_XY         = 0.6
K_POS_Z             = 0.60
V_MAX_Z             = 0.80
K_RC_Z              = 105
K_FF_Z              = 40
MAX_DEV_Z           = 120
YAW_KD              = 12
YAW_MAX             = 350
YAW_KP              = 0.4
YAW_KI              = 0
Z_FILTER_ALPHA      = 0.12
Z_VEL_WINDOW        = 0.30
Z_VEL_MIN_SPAN      = 0.22
Z_VEL_ALPHA         = 0.5
XY_FILTER_ALPHA     = 0.50
VISIT_HOLD          = 1.10
SAVE_HOLD           = 2.10
HOME_HOLD           = 1.10
HOLD_RADIUS         = 0.38
HOLD_FRESH          = 0.15
HOLD_STATE          = raw_state (unfiltered marker)
SAVE_CTRL_Z         = 14.0
PASS_BOX            = 0.48
PASS_DWELL          = 0.0
SAVE_FAST_MARGIN    = 1.00
LAND_V_MAX          = 0.72
LAND_ENGAGE_ALT     = 0.40
LAND_THROTTLE       = 1390
TOUCHDOWN_Z         = 14.75
TOUCHDOWN_STALL_Z   = 14.65
TOUCHDOWN_HOLD      = 0.40
MAX_MARKER_STEP     = 0.35
RESYNC_COUNT        = 5
HEARTBEAT_PERIOD    = 0.1
OBSTACLE_CLEARANCE  = 0.65 cells
```

### Task 3A official arena

```
critical  B3 (+4.92, +3.69), I2 (-3.68, +4.93), J10 (-4.86, -4.86)
stable    J6 (-4.78, +0.01), F2 (+0.01, +4.79), B10 (+4.88, -4.87)
obstacles D3, I4, E8
```

---

## Appendix B — interfaces and route protocol

### `NavToWaypoint.action` (2B, unchanged in 3A)

```
geometry_msgs/Pose waypoint
bool hover
---
int32 hov_time
---
geometry_msgs/PoseStamped current_waypoint
```

### `GetWaypoints.srv` (2B, unchanged in 3A)

```
bool get_waypoints
---
geometry_msgs/PoseArray waypoints
string[] marker_ids
```

### `PlanStableRoute.srv` (3A only)

```
geometry_msgs/Pose start
---
geometry_msgs/PoseArray waypoints
string[] marker_ids
```

### Route token semantics

```
C5*   fly through C5, do not stop
B3    stop at B3: Visit 1 s at Z=13, Save 2 s at Z=14
F6    home: hover 1 s at Z=13, then descend and land
```

### Critical route (official take)

```
F6 -> C5* -> B3 -> C5* -> F4* -> H2* -> I2 -> H2* -> G4* -> J10
```

### Stable route (official take)

```
START(-4.85,-4.86) -> J6 -> G5 -> F2 -> E5 -> C8 -> B10 -> C7 -> F6
```

---

## 13. Optimization Trials, Scoring Formula, and Physical Limits Analysis

### 13.1 Empirical e-Yantra Portal Scoring Formula

Based on the official portal evaluation results, the scoring formula is:

$$\text{Total Score} = \text{Survivors} (48) + \text{F6 Hover} (3) + \text{F6 Landing} (3) + \text{Speed Marks} (26)$$

$$\text{Speed Marks} = 26 \times \frac{150 - t_{\text{landing}}}{150} \times \frac{\text{survivors\_saved}}{6}$$

For our validated 87.30 s golden run the portal clock read 86.76 s:

$$\text{Speed Marks} = 26 \times \frac{150 - 86.76}{150} \times 1.0 = 26 \times 0.4216 = 10.97 \text{ / } 26$$

$$\text{Total Portal Score} = 48 + 3 + 3 + 10.97 = \mathbf{64.97 \text{ / } 80}$$

For the final 82.47 s take the same 0.54 s offset gives a portal landing clock
of 81.93 s:

$$\text{Speed Marks} = 26 \times \frac{150 - 81.93}{150} \times 1.0 = 26 \times 0.4538 = 11.80 \text{ / } 26$$

$$\text{Total Portal Score} = 48 + 3 + 3 + 11.80 = \mathbf{65.80 \text{ / } 80}$$

Using the strict 82.47 s without the offset gives 11.71 speed marks and
**65.71/80**, the conservative floor.

The earlier 79.50 s take was projected at 66.22 but the portal scored it
**49.21**, because B10's Visit and J6's Save failed the 0.40 sphere. The projection
above uses the sphere-verified bag, not the old box scorer.

### 13.2 Mathematical and Physical Upper Bounds

1. **Mandatory Stationary Time Budget**:
   - 6 survivor Visits $\times 1.0\text{ s} = 6.0\text{ s}$
   - 6 survivor Saves $\times 2.0\text{ s} = 12.0\text{ s}$
   - 1 F6 Hover $= 1.0\text{ s}$
   - Takeoff arm dead zone + touchdown settling $= 3.0\text{ s}$
   - **Total zero-velocity time**: $\mathbf{22.0\text{ s}}$!
   - Even a hypothetical instantaneous teleporting drone cannot finish under 22.0 s, capping the theoretical maximum possible score at $54 + 26 \times \frac{128}{150} = \mathbf{76.18 \text{ / } 80}$.

2. **Physical Quadrotor Flight Envelope**:
   - Total route trajectory length across the planned route is $\sim 60.4$ whycode units ($\sim 52.8$ units of grid tour plus snap offsets).
   - At a maximum tilt angle of $8.5^\circ$ ($a_{\text{max}} \approx 1.45\text{ m/s}^2$) and terminal drag velocity limit of $1.80\text{ m/s}$, the minimum kinematic transit time is $\sim 38-42\text{ seconds}$.
   - Realistic physical limit for full-mission completion in MuJoCo: $\mathbf{t_{\text{landing}} \approx 65 - 70\text{ s}}$, yielding a maximum achievable score of $\mathbf{68.0 - 69.5 \text{ / } 80}$.
   - The 82.47 s final take is 12–17 s above that band; the remaining gap is
     mostly corner deceleration, the longer sphere-safe holds and the 0.65-cell
     obstacle inflation, all of which were tested and traded away against hold
     reliability.

### 13.3 Summary of Optimization Trials

| Trial | Hypothesis / Changes | Landing Time | Strict Grader Result | Telemetry Findings & Mechanism |
|---|---|---|---|---|
| **Golden Baseline** | $V_{\text{max}}=1.60$, $K_{\text{pos}}=1.0$, $\text{BOX}=0.35$, proportional Z | **87.30 s** | **PASS (64.97/80)** | 0 hold resets on 4/6 survivors; clean clearances (+0.35, +0.45, +0.99). |
| **Trial 1** | Fast Z dive ($\text{SAVE\_FAST\_MARGIN}=0.45$, $-0.65\text{ m/s}$) | 112.31 s | FAIL (holds) | Deep dive to $Z=14.56-14.71$ breached 14.40 floor; 2-8 resets per survivor. |
| **Trial 2** | Tight pass box ($\text{PASS\_BOX}=0.42$, dwell=0.0) | 99.96 s | PASS (slow) | Near-misses on via boundaries caused looping around via points. |
| **Trial 3** | Zero integrator between waypoints | N/A (aborted) | FAIL (oscillation) | Cleared aerodynamic tilt trim, causing violent XY hunting at waypoints. |
| **Trial 4** | Config 1 retune ($K_{\text{pos}}=1.05$, $V_{\text{max}}=1.75$) | 99.31 s | PASS (slow) | Terminal deceleration boundary pushed into noise floor; multiple hold resets. |
| **Trial 5** | Decoupled Z ($\text{SAVE\_CTRL\_Z}=13.90$, hold box at 14.00) | 98.40 s | PASS | Contained max Z to $14.18-14.30$; critical switch achieved at **41.47 s**. |
| **Trial 6** | High-Speed Attack ($V_{\text{max}}=2.05$, $V_{\text{pass}}=0.65$, box=0.45) | 103.90 s | PASS | Long transits overshot B3/F2 by $0.53-0.59\text{ m}$; blew past F4* corner; lost 15.3 s. |
| **Trial 7** | Refined Fast ($V_{\text{max}}=1.70$, $K_{\text{rc\_z}}=90$, box=0.36) | 101.80 s | PASS | First half faster, but 8.15m runway to J10 overshot to $Y=-5.42$; $K_{\text{rc\_z}}$ caused Z hunting. |
| **Trial 8** | 50 Hz server/client loop rate + $\text{BOX}=0.38$ | 107.07 s | PASS | 50 Hz spinning saturated WSL2 CPU, dropping camera rate from 30.0 Hz to 28.2 Hz and causing sensor lag. |
| **Trial 9** | Continuous-space routing (visibility graph with continuous start/goal) | not flown | rejected offline | 56.7 units vs 60.4, but 12 fly-throughs vs 6; fly-through legs average only 1.6–3.3 u/s, so the grid tour is faster in practice. |
| **Trial 10** | Obstacle inflation 0.55 cells | not flown | rejected offline | Re-plan saves only 0.38 units (≈0.23 s) while raw XY overshoot is 0.2–0.56 units; footprint risk not justified. |
| **Trial 11** | $\text{SAVE\_CTRL\_Z}=13.85$ | run aborted | FAIL (Save) | F2's raw Save run never reached 2.00 s; the measured hold band 13.60–14.34 is already centred at 13.92. |
| **Trial 12** | $\text{PASS\_BOX}=0.58$ | 89.2 s | PASS | No measurable gain; wider fly-through box does not shorten the tour. |
| **Trial 13** | $K_{\text{pos}}=1.0$ at $V_{\text{max}}=1.8$ | 84.7 / 85.2 s | PASS (slow) | 5–6 F2 hold resets from terminal overshoot; the higher gain needs the lower cruise. |
| **Trial 14** | Per-phase $K_{\text{pos\_pass}}=1.30$ | 79.6 s | PASS (rejected) | E8 inflated clearance collapsed to +0.013 and Save holds became unreliable; clearance margin is worth more than 0.1 s. |
| **Trial 15 (v10, previous submission)** | $K_{\text{pos}}=0.9$, $V_{\text{max}}=1.8$, $K_{\text{rc}}=140$, $K_{\text{rc\_z}}=105$, box=0.37, $\text{SAVE\_CTRL\_Z}=13.92$, soft approach/landing | **79.50 / 80.78 s** | PASS box; **FAIL sphere** (portal **49.21/80**) | Lower gain tamed the 1.8 m/s overshoot, but B10 Visit was 0.79 s and J6 Save 1.70 s in the real 0.40 sphere. |
| **Trial 16 (v17, previous submission)** | Sphere holds at R=0.38 (1.10 / 2.10 s), $V_{\text{max}}=2.20$, XY filter 0.50, separate `gate_pos`, $\text{SAVE\_CTRL\_Z}=14.0$ | **84.52 s** | **PASS (65.44/80 projected)** | All 12 holds pass at R=0.40 and R=0.38, zero marker gaps; third Save 37.13 s; clearances +0.47 / +0.47 / +0.76. |
| **Trial 17** | Z deadband 0.18 to damp the Save ring | 92.29 s | PASS | Isolated B3 descent clean (zmax 14.06, rebound 13.96), but a ±150° yaw spin cost ≈10 s; the deadband did not address the dominant loss. Reverted. |
| **Trial 18** | Horizontal braking cap 0.85 (vector deceleration) | isolated test | FAIL (timing) | B3 stop 12.2 s vs 9.2 s baseline, confounded by a yaw spin to +171°; braking harder made the approach slower. Reverted. |
| **Trial 19** | `YAW_MAX=480` + level-out guard when $|yaw\_rate|>28°/s$ for 0.4 s | aborted | **FAIL (mission)** | Overshot J10 to (−6.22, −7.36), yaw rate 80.9°/s, `rc_yaw=1020`; three goals aborted, exit 1. Reverted. |
| **Trial 20 (v20ref, final submission)** | **No control change**; v17 flight code plus two "sphere entered" log lines | **82.47 s** | **PASS (65.80/80 projected)** | All 12 holds pass at R=0.40 and R=0.38, zero marker gaps; third Save 40.05 s; stable phase + landing 5.0 s faster than v17; clearances +0.57 / +0.70 / +0.94. |

### 13.4 Primary Engineering Conclusion

The Golden Baseline ($V_{\text{max}}=1.60\text{ m/s}$, $K_{\text{pos}}=1.00$, $K_{\text{rc}}=130$, $K_{\text{pos\_z}}=0.60$, $K_{\text{rc\_z}}=80$, $\text{BOX}=0.35$) is a critically damped operating point, but it is not the fastest stable one. The v10 retune showed that raising cruise speed above 1.60 m/s is viable **only if the position gain is reduced at the same time**: $K_{\text{pos}}=0.9$ with $V_{\text{max}}=1.80$, more tilt authority ($K_{\text{rc}}=140$, clamp 165) and a slightly wider acceptance box (0.37) removed the terminal overshoot that made the earlier 1.8 m/s attempts loop on F2. The campaign's harder lesson came later: the portal scores a **3-D sphere of radius 0.40**, not a per-axis box, and a bag that passes the box can still lose two holds. The v17 build therefore keeps the lower gain, raises cruise to 2.20 m/s, filters XY for control only, and holds 1.10 / 2.10 s at R=0.38 so every completed hold clears the real sphere with margin.

Four rules summarize the campaign:

1. **Hold reliability outranks transit speed.** Every variant that traded hold margin for time (fast Z dive, 0.45-cell clearance, per-phase gain, 13.85 cm Save) lost more in resets or clearance than it gained.
2. **Tune the pair, not the knob.** Cruise speed and position gain must move together; raising either alone destabilizes the terminal approach.
3. **Validate at the bag level, in the criterion the grader uses.** The server's console can report success on stale `raw_state`; only continuous bag runs inside the 0.40 sphere prove a Visit or Save.
4. **A server PASS is not a bag PASS.** A rejected outlier can extend the server's hold while remaining in the recording. The final selection replays every candidate with the naive sphere test.

The final 82.47 s take is 12–17 s above the 65–70 s physical band. The remaining
time is dominated by 20 s of server holds (six 1.10 s Visits, six 2.10 s Saves,
one 1.10 s hover), ~16 s of vertical round trips, corner deceleration on a
60.4-unit tour and the longer R=0.38 safety holds. The tested routes and gains that would recover 2–3 s
each all failed a hold or clearance check. Within the reliability envelope the
practical ceiling is ≈80 s, or ≈66/80; v20ref sits at 65.71–65.80.

The motion campaign sharpened the diagnosis. XY stop overshoot is real
(0.15–0.43 units) but it is not the binding constraint: the drone still passes
through the 0.40 sphere, so the overshoot costs little hold time. The binding
constraints are the vertical ringing after each Save dive (0.8–4.3 s to settle)
and the simulator's occasional large yaw spin (up to 150°, ≈10 s). Attempts to
fix the overshoot directly — a Z deadband, a horizontal braking cap, and a yaw
level-out guard — were all slower or outright unsafe, so the final decision was
to keep the v17 flight code, add only sphere-entry logging, and take the fastest
clean re-run. The 82.47 s bag beats the previous 84.52 s submission by 2.05 s and
raises the projection from 65.44 to 65.80/80.

---

## Appendix C — glossary

| Term | Meaning |
|---|---|
| **WhyCode** | The camera/marker pose system; reports `(x, y, z)` with z as depth below the ceiling camera |
| **Visit** | 1 s continuous inside the 0.40 sphere at Z=13 |
| **Save** | 2 s continuous inside the 0.40 sphere at Z=14 |
| **Switch** | The moment the third critical Save completes; the mission changes to the stable phase |
| **Critical survivor** | Red triangle; must be Saved by 55 s |
| **Stable survivor** | Yellow disc; must be Visited after the Switch |
| **Fly-through** | A route waypoint marked `*`; no stop |
| **F6** | Home grid cell; hover then land |
| **Dead zone** | The first 2–4.3 s after arming when the simulator ignores commands |
| **Phase A** | Vertical-only liftoff until `z < 14.2` |
| **Bag** | ROS 2 rosbag2 `.db3` recording; the scoring ground truth |

---

*End of report.*
