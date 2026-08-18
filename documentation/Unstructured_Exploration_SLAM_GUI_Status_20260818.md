# Unstructured exploration, start matching and SLAM status — 2026-08-18

## Result

The compact hardware `UnstructuredGates` pipeline is now perception-driven for
both `HardwareLab` and Manual/Custom maps. It no longer uses the configured
goal or a SLAM frontier as a path-following route.

The runtime sequence is:

1. optional initial LiDAR start matching with the motors inhibited;
2. continuously evaluate the live 360-degree LiDAR and update the map;
3. advance straight when the forward swept footprint is free;
4. before reaching a boundary/obstacle, select a bounded forward LiDAR arc;
5. immediately preempt exploration when a temporally confirmed physical gate
   is detected;
6. generate the gate clothoid and follow it with MPC;
7. verify crossing with longitudinal and lateral gate-plane tests;
8. mark only the physical gate as passed and resume exploration.

If no safe forward arc exists in a closed corner, a short LiDAR-validated
reverse arc may reposition the car. It is an exceptional geometric escape,
never a gate-pursuit command. Both wheel groups always retain the same sign.

Frontier targets are tagged separately from physical gates. Reaching a
frontier therefore cannot increment `passed_gates`. Compact unstructured runs
have no artificial finite gate count: they continue looking for as many
physical gates as possible until the operator stops the run.

## GUI controls

The Hardware Planner setup panel has two default-on checkboxes:

- `Initial LiDAR start matching`: when enabled, a missing, unstable, corrupt,
  wrong-serial or rejected reference fails closed and keeps both PWM commands
  at zero. Disabling it is the explicit debug bypass.
- `SLAM Toolbox`: enables/disables direct runner-to-sidecar scan submission.
  Changing either checkbox is sent to the connected Raspberry runner and
  performs a safe planner reset.

The live diagnostics show the exploration state, control source, map source,
frontier, start-match status/confidence, SLAM session/reset reason, map age,
graph nodes, loop edges and reference invalidation reason. These values are
also included in streamed telemetry, JSON and CSV reports.

## Mapping ownership and persistence

For hardware, the Raspberry runner is the sole SLAM client. The GUI no longer
submits the same hardware scan to a second client. Simulation can still use the
GUI-local bridge.

The apparent map deletion had two independent causes:

- the old local display map was the short-lived collision grid and removed
  cells using `occupancy_decay_steps`;
- the UDP sidecar interpreted any duplicate/out-of-order sequence as a new
  run and reset SLAM Toolbox.

The hardware stream now carries three distinct layers:

- `local_safety_occupied_points`: short-lived collision evidence;
- `global_free_points` / `global_occupied_points`: persistent exploration map;
- optimized SLAM Toolbox occupancy cells when the backend is connected.

Every runner process creates a unique SLAM session. Only a session change
resets the graph; stale UDP packets are discarded. If the backend is disabled,
stale or has no complete free/occupied map yet, exploration uses the
non-decaying local grid accumulated for the current run.

Start the workstation backend with:

```sh
./tools/slam_toolbox_bridge/run.sh
```

The runner enables SLAM by default. When `--stream-host <pc-ip>` is supplied,
the same host becomes the default bridge host. The GUI launch hint includes
the explicit `--slam-bridge-host <pc-ip> --slam-bridge-port 9760` form.

## Start-reference prerequisite

Start matching is intentionally fail-closed. Before a physical run with the
checkbox enabled, capture and copy the matching reference described in
`Initial_LiDAR_Start_Matcher_20260730.md`, normally:

```text
datasets/localization/car_unstructured_hardware_lab_start.csv
```

Use the GUI checkbox or `--no-start-matching` only for an intentional debug
run. The runtime also checks the stored LiDAR serial against the connected
sensor before accepting scans.

## Verification performed

- `thesis_world_stream_roundtrip`: runtime checkboxes, exploration frame,
  persistent map vectors and extended telemetry survive serialization.
- `thesis_unstructured_exploration_pipeline`: immediate live-LiDAR motion,
  straight open-space exploration, bounded forward search near an arena edge,
  no SLAM-frontier control, persistent fallback, SLAM/fallback switch,
  fail-closed start matching, debug bypass and gate priority.
- `thesis_slam_session_sequence_policy`: new session resets; duplicate and
  reordered UDP packets are stale and never reset the graph.
- Python bytecode compilation for the SLAM bridge scripts.
- Full C++ builds of `thesis_robot_runner` and `thesis_planner_sim`.
- Synthetic HardwareLab runner check (600 steps / 60 s): two detected physical
  apertures crossed, continued open-ended exploration, no collision, zero pure
  pivots and zero opposite-PWM cycles.

## Hardware report correction — 21:02 / 21:04

The reports `thesis_hardware_mixed_mixed_hardware_road_gate_gui_manual_20260818_210234_584`
and `thesis_hardware_unstructured_manual_gate_editor_gui_manual_20260818_210448_649`
exposed two additional control defects:

- Manual Editor repeatedly selected and invalidated a lateral frontier. The
  unsafe reference vectors survived after their diagnostic flag was cleared,
  while the strict lock interlock forced speed/PWM back to zero. This produced
  27.27 s in `INITIAL_SCAN` with zero encoder distance.
- Mixed mode allowed point-acquisition recovery to override a valid gate
  clothoid. At large heading error it emitted direct yaw with opposite wheel
  PWM (`-45`, `+63` in the final sample), creating the observed arc/pivot.

For a car-like hardware profile, exploration frontiers are now accepted only
along the current straight LiDAR-supported corridor. No lateral frontier is
converted into a search arc: absence of straight known-free space produces
`HOLD`. Invalid references are physically cleared. A valid physical-gate
clothoid remains authoritative for the MPC, while direct-yaw/in-place recovery
and opposite-wheel PWM are forbidden for car-like mixed/unstructured runs.

This conservative `HOLD` policy was an intermediate correction and is
superseded by the 21:43 live-LiDAR exploration pipeline below. Mixed mode keeps
the no-direct-yaw rule; Manual unstructured uses only swept, same-sign arcs.

The regression test also covers a sparse 90-beam scan, forward motion after the
initial scan, near-zero exploration yaw, and absence of opposite-wheel commands
in both unstructured and mixed modes.

## No-clothoid forward exploration correction — 21:25

The report
`thesis_hardware_unstructured_manual_gate_editor_gui_manual_20260818_212510_695`
confirmed that gate recognition and crossing were working (`passed_gates=1`,
about 0.64 m of encoder travel), but also exposed one remaining incorrect
interlock: 219 of 277 samples were `HOLD` and 255 had zero PWM. At the final
sample the LiDAR was current and usable (176 valid returns and 1.20 m front
clearance), there was no safety stop, and the only invalidation reason was
`no_straight_known_free_frontier`.

A missing SLAM frontier or planner reference is therefore no longer a reason
to remain stationary. After the initial scan, the compact car-like
unstructured controller now enters:

- `ADVANCING_STRAIGHT` / `STRAIGHT_EXPLORATION` when no physical gate clothoid
  is active, the current LiDAR scan has enough valid samples, the forward
  corridor is clear, and a forward footprint probe remains inside the arena;
- `GATE_LOCKED` and then `TRACKING_GATE_CLOTHOID` / `GATE_MPC` as soon as the
  temporally confirmed physical gate yields a valid reference;
- `HOLD` only when current LiDAR data is insufficient, the front corridor is
  blocked, the forward probe would leave the arena, or another safety/interlock
  condition is active.

The direct exploration command is deliberately limited to 0.045--0.060 m/s
and always requests zero curvature and zero yaw rate. It does not synthesize a
search arc and cannot override a confirmed physical-gate clothoid. The
integration regression explicitly forces the frontier minimum beyond the
entire test arena and verifies positive forward PWM with an empty reference
trajectory, followed by the existing gate-over-exploration priority test.

The zero-curvature fallback described in this section remains the open-space
case, but it is no longer the only exploration command. The 21:43 correction
adds a forward LiDAR turn before straight motion would enter a dead end.

## Gate steering and real exploration correction — 21:43

The report
`thesis_hardware_unstructured_manual_gate_editor_gui_manual_20260818_214343_936`
showed that the robot passed the first aperture, but did not reliably steer
towards the next detected gate. During the failing segment the MPC requested a
negative yaw rate while the estimated heading error grew from about 24 to 47
degrees. The logical PWM was differential but forward (`82/70` in several
samples); the physical left encoder repeatedly stayed at zero while the right
side advanced. Therefore the report contains both a software-authority problem
and evidence of residual left-wheel break-away/asymmetry. It does not justify
another motor-channel swap; calibration profile 1.4 remains unchanged until an
isolated left/right wheel test says otherwise.

The historical comparison also excludes MVC itself as the cause. Commit
`bdb01fb` mainly split the old monolithic runner; forward-arc search and tangent
reacquisition were added later in `56af018` and `1e0c76a`. The old reports do
not contain a conclusive pre-MVC physical validation: several apparent
successes used the old virtual-goal completion or ended outside the compact
arena. The repeatable validation was synthetic.

The corrected Manual pipeline is now:

1. a fresh valid LiDAR scan immediately enables exploration; there is no
   stationary Manual startup phase;
2. SLAM Toolbox keeps building/displaying the global map, but its frontier is
   never converted into a motion reference;
3. with no gate, one sector selector returns either
   `ADVANCING_STRAIGHT/STRAIGHT_EXPLORATION` or
   `SEARCHING_FREE_SPACE/FORWARD_SEARCH`;
4. every candidate arc is checked at three poses over 0.18--0.26 m using the
   oriented robot footprint, arena bounds and current LiDAR endpoints;
5. a lateral/rear gate remains available as a search-direction hint but is
   excluded from the clothoid planner until it enters the forward 85-degree
   reachable sector;
6. a reachable gate is locked, the planner generates its clothoid and the MPC
   becomes authoritative;
7. after crossing, the robot continues straight for one body length so its
   rear clears both posts before free-space search resumes.

For gate tracking, the actual runtime geometry now exposes `6.0 1/m` maximum
curvature instead of the previous `1.8 1/m`: the permitted model radius changes
from about 0.56 m to about 0.17 m, matching the tighter forward differential
authority of the two motor groups. Ordinary errors remain under MPC control.
If the local clothoid tangent error exceeds 0.70 rad, a hysteretic forward-only
tangent acquisition engages and releases below 0.35 rad. Gate turns receive a
14--38 PWM minimum left/right difference; both commands remain positive. The
target is the clothoid tangent, not the gate-centre chord, so this does not
reintroduce point pursuit.

Verification after this correction:

- complete Ninja build;
- all 9 CTest tests passed;
- the expanded integration test passed for open-space straight motion, the
  exact 21:25 no-reference pose, pre-emptive boundary turning, an offset gate
  with non-zero yaw and unequal positive PWM, SLAM persistence/toggle behavior,
  start-matching bypass and mixed-mode no-pivot behavior;
- the 600-step HardwareLab synthetic run timed out normally in open-ended
  exploration after `passed_gates=2`, with `status=timeout`, no collision,
  `pivot_command_cycles=0` and `opposite_pwm_cycles=0`.
