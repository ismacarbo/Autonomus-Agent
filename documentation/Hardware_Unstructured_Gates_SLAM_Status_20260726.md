# Hardware unstructured gates and SLAM status — 2026-07-26

## Runs analysed

- `thesis_hardware_unstructured_manual_gate_editor_gui_bundle_20260726_205248_667`
- `thesis_hardware_unstructured_manual_gate_editor_gui_bundle_20260726_205601_258`

Both runs use the definitive car geometry (`0.25 x 0.15 m`, wheel radius
`0.0327 m`, 38 encoder ticks/revolution) in a `1.20 x 1.20 m` map.

The first run remained live for `114.61 s`: only 29 of 821 samples contained a
selected gate and 752 samples requested zero target speed. The second reached
the two-gate completion condition in `106.77 s`, but contained about 67 seconds
without a planner reference between the first and second selected gates. The
dominant fault was therefore candidate acquisition, not just motor power.

## Changes applied

### GUI footprint

Compact unstructured hardware maps now use the same `0.42` visual scale as
Structured Validation Road. Previously the unstructured GUI used `1.45`, so
the drawing was over three times larger even though the planner geometry was
already correct. This is display-only and does not alter collision or LiDAR
geometry.

### PWM floor

Compact unstructured hardware uses:

- minimum effective PWM: `60`;
- break-away PWM: `120`;
- stall detection target threshold: `0.018 m/s`;
- stall boost after at most two control cycles.

The effective PWM band is applied again after slew limiting. Previously a
valid minimum command could be reduced to 42 PWM by the ramp and fail to move
the real car. These overrides are local to compact unstructured hardware; the
calibration profile still states that a dedicated multi-PWM sweep is pending.

The measured calibrated simulation also exposed a final-stop race: the robot
entered the goal region above the completion speed threshold, lost the final
LiDAR target for one frame and accelerated again until it crossed the map
boundary. Unstructured simulation now latches a controlled stop upon entering
the final goal region and waits for the vehicle to settle. Intermediate gates
cannot activate this latch. The car dynamics model now also treats
`target_speed = 0` as an explicit stop, consistently with the tracked model;
previously zero was mistaken for an unavailable target and replaced by a
rolling acceleration-derived target.

Dynamic unstructured simulation now also caps gate-mode speed from target
distance, heading error and LiDAR clearance. This prevents the measured
Calibrated car plant from carrying its 0.2 m/s cruise speed through a tight
aperture turn; the cap remains above 0.055 m/s so it does not recreate the
sub-threshold hardware command problem.

The Calibrated profile loader now keeps both sides of the actuation model
consistent: applying `speed_estimate_per_pwm` also sets
`wheel_speed_to_pwm_gain = 1 / speed_estimate_per_pwm` and removes the legacy
PWM bias. The hardware runner already did this; simulation previously combined
the new measured forward slope with the old inverse gain and bias, turning a
0.055 m/s gate command into roughly 0.24 m/s plant speed.

For the car, the nonzero gate-motion floor is now enforced after MPC rather
than only on its desired-speed input: a valid gate trajectory can no longer
alternate zero and sub-break-away commands because of the finite-horizon
solution. The explicit final-goal stop latch is evaluated afterwards and
therefore always remains authoritative.

### Dynamic gate pipeline

The active data flow is now:

1. filter and motion-compensate LiDAR beams;
2. build a `0.018–0.030 m` local occupancy grid in the 1.20 m arena;
3. find range-jump and free-space sectors;
4. measure aperture width between the two obstacle boundary points;
5. calculate the true Euclidean midpoint of those boundary points;
6. score candidates using width, depth, clearance, forward progress, goal
   alignment and continuity;
7. associate candidates with persistent temporal tracks;
8. pass the confirmed, score-ordered candidate list to the planner workflow;
9. re-score the list for reachability and mission progress;
10. lock the selected persistent gate until physical crossing.

The compact hardware minimum aperture is `max(configured width, body width +
0.04 m)`, currently 0.21 m. The old generic formula required about 0.32 m and
the old grid used 4–7 cm cells, which rejected most candidates in the reports.
There was also a second footprint error: `path_clearance_radius_m` already
contained half the robot width and `scan_supports_target()` added another half
width. A real 0.30 m aperture was consequently rejected as if the car needed
roughly 0.32 m. The field now contains only the additional 0.01 m corridor
margin; both the current-scan and accumulated-perception checks construct the
total corridor as half body width plus that margin.

Gate names now retain their persistent identity (`gap_track_N`) while locked.
Position, aperture midpoint and measured width are filtered together by the
persistent track. The tracking target is placed 8 cm beyond the aperture
midpoint. A gate is counted only after the robot centre reaches the aperture
plane, with a bounded tolerance; proximity of the front edge alone is no
longer sufficient. The final mission goal uses an entry plane at the edge of
its acceptance region, so it cannot be counted as an arbitrary intermediate
gate and does not require the robot to drive beyond the map goal.

In `pose-fusion auto`, the old local scan-to-accumulated-points correction is
no longer applied. That matcher used the same drifting frame for observations
and reference points and could produce confident jumps in symmetric maps. It
remains available only as the explicit diagnostic mode `--pose-fusion lidar`;
bounded optimized corrections come from SLAM Toolbox only when
`--slam-pose-feedback` is explicitly enabled.

The same contract now applies to Calibrated simulation: its published mode was
already `Encoders + IMU (LiDAR perception only)`, but the old local matcher was
still executed in unstructured maps. It is now disabled there as declared;
Ideal/non-calibrated diagnostics retain the lightweight matcher, while global
optimized corrections belong to the SLAM Toolbox backend.

### True SLAM Toolbox backend

The original ROS 2 `slam_toolbox` sidecar under `tools/slam_toolbox_bridge/`
has been completed:

- it publishes a uniform 360-bin `sensor_msgs/LaserScan` instead of treating
  irregular returns as uniformly spaced;
- it publishes the measured `base_link -> laser` translation;
- it obtains corrected pose from the optimized `map -> odom` transform;
- it resets the pose graph at a new run/session;
- it returns both free and occupied occupancy cells;
- it reports pose-graph nodes and non-sequential loop edges;
- oversized maps thin free display cells before touching occupied cells.

The container uses ROS 2 Jazzy and the current SLAM Toolbox interfaces. The
older Humble 2.6.x package does not expose the pose-graph event and reset APIs
used by the bridge.

When connected, the GUI and `*_slam_reference.png` render the optimized
occupancy grid. The raw `*_lidar_reconstruction.*` artifacts remain available
and explicitly labelled as passive estimated-pose reconstruction. The PNG
bounds are expanded to include the optimized free/occupied cells and trail,
so loop-closure corrections outside the editor rectangle are not clipped.

## Verification

- C++ runner, planner GUI, SLAM bridge and world-stream targets compile.
- All five CTest unit/integration tests pass.
- Python bridge passes bytecode compilation.
- `Hardware Lab`, car, simulated hardware pipeline: `goal_reached`, 2 gates,
  205 steps, 20.5 simulated seconds, no safety stop.
- All six deterministic Unstructured car presets (`validation`, `tight`,
  `slalom`, `lower`, `hardware_lab`, `ideal`) reach the goal in both Ideal and
  Calibrated modes: 12/12 runs, zero collisions/timeouts, 4000-step limit.
- `Hardware Lab`, car: Ideal `goal_reached` in 232 steps; Calibrated
  `goal_reached` in 1008 steps.
- Structured Validation Road and Circle Loop, car: 4/4 Ideal/Calibrated runs
  reach the goal within 1167 steps.
- Nonzero motion commands in this run respect the 60 PWM minimum.

The `Closed Obstacle Road` hardware-plant regression is improved but not yet
closed: it now keeps encoder/IMU odometry stable and completes the first
bypass instead of remaining in the first safety stop, but the rejoin target at
the upper boundary can still enter a prolonged pivot. It must not be counted
as a passing regression until that rejoin is corrected and rerun.

The actual ROS 2 container could not be started on this workstation because
Docker/ROS 2 are not installed locally. The next hardware validation should
therefore run the sidecar on the GUI workstation and confirm that the panel
shows `SLAM Toolbox`, nonzero map updates and an occupancy-grid
`*_slam_reference.png`.

The legacy large-coordinate unstructured presets were not redesigned in this
change. Their physical miniaturization is a separate map-design task; the
current hardware focus remains Manual Gate Editor and Hardware Lab.
