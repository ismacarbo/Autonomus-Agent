# Unstructured exploration, start matching and SLAM status — 2026-08-18

## Result

The compact hardware `UnstructuredGates` pipeline is now perception-driven for
both `HardwareLab` and Manual/Custom maps. It no longer uses the configured
goal or the legacy forward search arc as an exploration route.

The runtime sequence is:

1. optional initial LiDAR start matching with the motors inhibited;
2. stationary initial scan;
3. select a reachable information frontier from known-free/unknown cells;
4. generate and validate a local clothoid toward that frontier;
5. immediately preempt the frontier when a temporally confirmed physical gate
   is detected;
6. generate the gate clothoid and follow it with MPC;
7. verify crossing with longitudinal and lateral gate-plane tests;
8. mark only the physical gate as passed and resume exploration.

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
- `thesis_unstructured_exploration_pipeline`: stationary start, no legacy
  search arc, frontier selection, persistent fallback, SLAM/fallback switch,
  fail-closed start matching, debug bypass and gate-over-frontier priority.
- `thesis_slam_session_sequence_policy`: new session resets; duplicate and
  reordered UDP packets are stale and never reset the graph.
- Python bytecode compilation for the SLAM bridge scripts.
- Full C++ builds of `thesis_robot_runner` and `thesis_planner_sim`.
- Synthetic HardwareLab runner check: the robot crossed the detected physical
  aperture (`passed_gates=1`) without collision, then correctly remained in
  open-ended exploration rather than declaring a scripted goal complete.

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

The regression test also covers a sparse 90-beam scan, forward motion after the
initial scan, near-zero exploration yaw, and absence of opposite-wheel commands
in both unstructured and mixed modes.
