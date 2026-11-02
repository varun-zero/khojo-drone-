# Google AI Studio review prompt

## How to use

1. Open [Google AI Studio](https://aistudio.google.com/) and choose a model with a large context window (Gemini 2.5 Pro / 3 Pro or newer).
2. Attach these files:
   - `KD_4373_waypoint_service.py`
   - `KD_4373_pico_client.py`
   - `KD_4373_pico_server.py`
   - `PlanStableRoute.srv`
   - `PROJECT_REPORT_2A_to_3A.md`
   - `official_bag/task_3a_detections.json`
3. Paste the prompt below.
4. For a second pass, use the follow-up prompt at the end.

The prompt asks for a code review, fine-tuning suggestions, and optimization suggestions. It is deliberately strict about evidence: every finding must cite a file and line, and every tuning change must include a validation plan.

---

## Prompt

```text
You are a senior robotics engineer with deep experience in ROS 2 Humble, nonlinear control, computer vision, motion planning, and competition robotics. Review the attached Task 3A submission as if you were the technical lead deciding whether it is ready for a national final — and then as the engineer who has to make it faster and more robust.

## Context

The project is the e-Yantra Robotics Competition 2025–26, theme KD (Khoj-o-Drone). A simulated quadrotor uses MuJoCo, ROS 2 Humble, and WhyCode fiducial markers. The arena has four ArUco corner markers, three red critical survivors, three yellow stable survivors, and three black obstacles.

Task 3A requires:

1. Critical phase (55 s): visit each red survivor for 1.0 s inside a 3-D sphere of radius 0.40 at Z=13, then save it for 2.0 s inside the same sphere at Z=14. All three saves must complete within 55 s of the take-off clock.
2. Switch: after the third critical save, cancel the remaining critical goal and re-plan the stable route from the drone's actual pose.
3. Stable phase: visit each yellow survivor for 1.0 s inside the 0.40 sphere at Z=13. Every stable visit must start after the Switch.
4. Obstacles: the flown trajectory must stay outside the three detected footprints.
5. F6: hover for 1.0 s at Z=13, then land on F6. Hard limit 200 s; the local scorer's full-marks time band is ≤110 s.

Scoring used by the local scorer:

- 24 points for the three critical survivors (8 each).
- 24 points for the three stable survivors (8 each).
- 3 points for the F6 hover.
- 3 points for landing.
- 26 points for speed.
- The speed projection that reproduces the grader's reported 34.76 on the first submission is:
  speed = 26 * (150 - landing_time) / 150 * (survivors_saved / 6).
- The exact grader weighting is not published. Treat the local scorer as a proxy, not ground truth.

Current official result (v17, 84.52 s strict):

- Landing time: 84.52 s; portal landing clock is ≈83.98 s (0.54 s offset measured on the previous take).
- Third critical save hold: 37.13 s.
- Stable visits: J6 40.99–44.16 s, F2 54.83–55.74 s, B10 69.26–72.59 s (sphere runs; all after the 37.13 s Switch).
- F6 hover: 80.22–83.52 s; touchdown 84.52 s; final z=15.03.
- Critical raw holds at R=0.40 (Visit/Save): B3 1.92/2.71 s, I2 1.89/2.28 s, J10 1.93/2.80 s.
- Stable raw holds at R=0.40 (Visit/Save): J6 1.79/2.46 s, F2 2.00/2.12 s, B10 1.96/2.93 s.
- Every save also passes at R=0.38 (2.61/2.14/2.80/2.46/2.05/2.57 s); the bag has zero marker gaps above 0.10 s.
- Obstacle raw clearances: D3 +0.47, I4 +0.47, E8 +0.76 units.
- Projected score: 65.44/80 (48 survivors + 3 hover + 3 landing + 11.44 speed).
- The previous v10 take (79.50 s) was submitted and the portal scored it 49.21/80, because B10's Visit was 0.79 s and J6's Save 1.70 s in the real 0.40 sphere. That is the central lesson: the grader's criterion is a sphere, not a per-axis box.
- The v17 build recorded 84.52, 85.67, 86.31 and 85.40 s takes, plus 102.20 s (failed I2) and 105.96 s (clean but slow). Run-to-run variance is real, and the server console can overstate a Save.

Files and responsibilities:

- `KD_4373_waypoint_service.py`: rectifies the arena, detects 3 critical / 3 stable / 3 obstacles, builds obstacle hulls and inflated margins, plans the critical route and the stable route, serves GetWaypoints and PlanStableRoute, writes debug images and JSON.
- `KD_4373_pico_client.py`: mission sequence, 55 s clock, Switch, stable re-plan, F6 goal, Save grace.
- `KD_4373_pico_server.py`: flight controller, NavToWaypoint action server, marker plausibility gate, raw-marker Visit/Save/hover tests, landing state machine.
- `PlanStableRoute.srv`: the only new interface. GetWaypoints.srv and NavToWaypoint.action must remain unchanged.
- `PROJECT_REPORT_2A_to_3A.md`: the full engineering report, including the first-submission post-mortem and all measured results.
- `official_bag/task_3a_detections.json`: the exact detected geometry used to score the official bag.

## Hard constraints

- Do not propose changing GetWaypoints.srv or NavToWaypoint.action. They are fixed by the task.
- PlanStableRoute.srv is the only allowed new interface.
- The grader scores the raw `/whycode_node/markers` signal against a 3-D sphere of radius 0.40, not the server's filtered state. Any hold-test suggestion must preserve or improve raw-marker continuity inside that sphere. Do not suggest making the hold test more permissive in a way that only passes the local scorer.
- Critical saves must be continuous 2.00 s raw holds; stable visits must start after the Switch.
- The final submission must remain a flat five-file package: three Python nodes, task_3a_0.db3, metadata.yaml.
- Do not suggest exploiting the scorer, hard-coding the arena, or special-casing the official bag. Suggestions must generalize to a new arena.
- Preserve the existing service/action interfaces and the mission contract.

## What to review

Work through the code and the report, not just the README. Cite file names and line numbers for every finding. Quantify where possible. Separate “definitely a bug” from “tuning opportunity” from “style”.

1. ROS 2 architecture and correctness
   - Node lifecycle, executors, callbacks, threading, and shutdown.
   - Service/action timeouts, cancellation, goal handling, and error paths.
   - QoS, message sizes, and whether the bag topic set is sufficient.
   - Any race between the marker callback, the control loop, the action server, and the client.
   - Any place where a stale pose, stale route, or stale goal can be used.
   - Whether the client's 55 s clock starts at the right event and whether the Switch is defined robustly.

2. Controller and tuning
   - The cascade: position, velocity, attitude/thrust mapping, yaw damper, feed-forward.
   - The depth filter (`Z_FILTER_ALPHA`, `Z_VEL_WINDOW`, `Z_VEL_MIN_SPAN`, `Z_VEL_ALPHA`), the XY control filter (`XY_FILTER_ALPHA`), and the separate `raw_state` / `gate_pos` grading path.
   - The Save approach (`SAVE_FAST_MARGIN = 1.00`) and whether a different profile could shorten the critical phase without losing a 2 s hold in the 0.40 sphere.
   - The landing state machine (`LAND_V_MAX`, `LAND_ENGAGE_ALT`, `LAND_THROTTLE`, `TOUCHDOWN_Z`, `TOUCHDOWN_STALL_Z`) and the risk of a false or missed touchdown.
   - Stability margins, saturation, integrator windup, and behavior under marker dropouts.
   - For every proposed gain or threshold change, give the current value, the proposed value, the expected effect, the risk, and the exact metric to compare.

3. Perception and planning
   - Rectification, detection thresholds, grid snapping, and the 3/3/3 validation gate.
   - Obstacle hull construction and the 0.65-cell inflation. Is the inflation policy right for the observed D3/I4 margin cuts?
   - The visibility graph, pass-through intersections, and the 3! order search.
   - The stable re-plan from the actual Switch pose: is the start pose handled correctly, and is the route still collision-free after the Switch?
   - Robustness to a missed detection, a false positive, a partially visible prop, or a different arena configuration.
   - Whether the planner can return a route that is valid on the grid but unsafe for the controller's corner-cutting behavior.

4. Timing and optimization
   - Where the 84.52 s is spent. Use the report's timeline and the code to build a time budget.
   - Which waits are structural (hold times, plan time, action settling) and which are controller inefficiency.
   - The highest-leverage changes to reach a lower landing time without risking the 55 s deadline or the raw holds.
   - Whether the stable phase can be shortened by better ordering, fewer pass-through stops, or a different Save policy.
   - Whether the critical phase can be shortened by better route order, transit speed, or a smarter descent profile.
   - Do not propose changes that only help the official arena.

5. Reliability and edge cases
   - Marker dropouts, outliers, duplicate bridges, and pose jumps.
   - Action server not ready, service not ready, route planning failure, goal timeout, cancellation.
   - Slow host / simulation rate below real time; the 15 s Save grace.
   - Landing on a noisy depth channel; stall detection.
   - Bag recording, scoring, and packaging failure modes.
   - What should be logged or asserted so a failed run is diagnosable from the bag alone.

6. Code quality and testability
   - Magic numbers that should be named constants or parameters.
   - Duplicated logic between the critical and stable phases.
   - Whether the controller, planner, and mission logic can be unit-tested or replayed offline.
   - Whether the scorer and the server share the same sphere/hold semantics, and where they could drift.
   - Documentation gaps that would block another engineer from reproducing the result.

## Output format

Return a structured review with these sections:

### A. Executive summary
Five bullets: the most important findings, in priority order. State whether the submission is safe to submit as-is, and what the single highest-leverage change is.

### B. Findings table
A table with columns:
ID | Severity (blocker / high / medium / low) | Area | File:line | Finding | Evidence | Proposed change | Expected impact | Risk | Validation

Every row must cite a file and line. If you cannot cite a line, say “not verifiable from the attached files” instead of guessing.

### C. Tuning table
A table with columns:
Parameter | Current | Proposed | Why | Expected effect on 55 s deadline | Expected effect on raw holds | Expected effect on landing time | Risk | How to validate

Include only changes you would actually test. Rank them by expected value / risk.

### D. Optimization roadmap
Three groups:
1. Quick wins (low risk, same interfaces, test in one session).
2. Medium changes (need a code change and a multi-run validation).
3. Risky changes (could break the 55 s deadline, raw holds, or landing; only with a fallback).

For each item, give the estimated time saving and the failure mode.

### E. Test plan
A concrete plan to validate the top changes:
- Which bag or scenario to run.
- Which metrics to compare (third-save time, raw hold lengths at R=0.40 and R=0.38, stable visit windows, obstacle clearances, hover, touchdown, final Z).
- How many runs are needed to separate a real improvement from run-to-run variance.
- Acceptance criteria and rollback criteria.

### F. Patch sketches
For the top 3–5 changes, give minimal unified-diff-style snippets or exact before/after code. Do not rewrite whole files. Keep the existing interfaces.

### G. Open questions
List the assumptions you had to make and the information that would change your recommendation.

## Style

- Be specific and quantitative. “Tune the controller” is not acceptable; “change K_POS_Z from 0.60 to 0.70 and expect a 0.2 s faster settle, validated by the 2 s raw hold at B3” is acceptable.
- Distinguish correlation from causation. The 87.30 s and 93.24 s takes were re-runs of the same build; the 79.50 s v10 take and the 84.52 s v17 take are real code changes, but the sphere criterion changed the result more than the speed. Run-to-run variance is still large; ask for repeated runs before attributing a gain to any single constant.
- Prefer small, reversible changes with a clear metric.
- Do not invent APIs, files, or results. If the attached files do not contain something, say so.
- Do not suggest changing the scoring rules or the task contract.
```

---

## Follow-up prompt

Use this after the first review, once you have picked the changes you want to explore:

```text
Now focus only on the top three changes you recommended. For each one:

1. Write the exact code change as a minimal patch against the attached files.
2. List every constant or parameter that must change with it.
3. Predict the effect on the critical-phase timeline, the raw 2 s holds, the stable phase, and the landing.
4. Give a run matrix that separates the change from run-to-run variance (at least 3 runs per configuration, same arena, same start pose).
5. Define the rollback condition.

Do not propose any change that alters GetWaypoints.srv, NavToWaypoint.action, or the raw-marker hold semantics.
```
