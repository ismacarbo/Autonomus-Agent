# Hardware unstructured gates and SLAM status — 2026-07-26

## Runs analysed

- `thesis_hardware_unstructured_manual_gate_editor_gui_bundle_20260726_205248_667`
- `thesis_hardware_unstructured_manual_gate_editor_gui_bundle_20260726_205601_258`
- `thesis_hardware_unstructured_manual_gate_editor_gui_bundle_20260727_005015_803`
- `thesis_hardware_unstructured_manual_gate_editor_gui_bundle_20260727_005640_579`
- `thesis_hardware_unstructured_manual_gate_editor_gui_manual_20260727_005758_670`
- `thesis_hardware_unstructured_manual_gate_editor_gui_manual_20260727_012728_643`
- `thesis_hardware_unstructured_manual_gate_editor_gui_manual_20260727_012859_110`
- `thesis_hardware_unstructured_manual_gate_editor_gui_manual_20260727_012940_125`

All runs use the definitive car geometry (`0.25 x 0.15 m`, wheel radius
`0.0327 m`, 38 encoder ticks/revolution) in a `1.20 x 1.20 m` map.

The first run remained live for `114.61 s`: only 29 of 821 samples contained a
selected gate and 752 samples requested zero target speed. The second reached
the two-gate completion condition in `106.77 s`, but contained about 67 seconds
without a planner reference between the first and second selected gates. The
dominant fault was therefore candidate acquisition, not just motor power.

The 27 July follow-up isolated a second, independent actuator fault. The first
run reached the goal, but the two failed runs spent 557/635 and 261/302 samples
in pure rotation. They emitted opposing left/right PWM in 633/635 and 300/302
samples. In those same runs the right encoder reported zero delta despite an
active command in 310 and 176 samples. Encoder distance consequently exceeded
the estimated chassis path by 2.05 m and 0.74 m. The passive LiDAR images show
the corresponding tight odometry loops. In the successful run, by contrast,
43 samples drove both wheels forward and only three of those had a zero right
encoder delta. This makes the opposing-wheel search manoeuvre, not gate
geometry, the differentiating failure mode.

## Changes applied

### GUI footprint

Compact unstructured hardware maps now use the same `0.42` visual scale as
Structured Validation Road. Previously the unstructured GUI used `1.45`, so
the drawing was over three times larger even though the planner geometry was
already correct. This is display-only and does not alter collision or LiDAR
geometry.

### PWM floor

Compact unstructured hardware uses:

- minimum effective PWM: `70` for the car;
- per-wheel break-away PWM: `90`, matching the measured calibration run;
- stall detection target threshold: `0.018 m/s`;
- stall boost after at most two control cycles.

Stall detection is now evaluated independently for the left and right wheel.
One moving wheel can no longer hide the other stationary wheel by keeping the
old maximum-wheel-speed test above its threshold. The 90-PWM break-away is
applied only to the stalled channel and its host integral is reset once, while
normal commands retain the lower 70-PWM floor. The effective PWM band is
re-applied after slew limiting. These overrides are local to the car in compact
unstructured hardware; the calibration profile still states that a dedicated
multi-PWM sweep is pending.

### Forward-arc gate search

The compact unstructured car no longer pivots to acquire or recover a gate.
Its 360-degree LiDAR already observes the full azimuth, so a chassis rotation
is not required merely to continue scanning. With sufficient clearance the
runner scores forward sectors, chooses the clearest competitive heading and
advances at 0.04--0.06 m/s. If no arc is safe it holds position and continues
processing scans.

When a gate is selected, large heading errors are followed with a bounded
forward arc. The yaw-rate limit is tied to current forward speed so both target
wheel speeds remain positive. The same-sign constraint is enforced again
after host feedback and after PWM slew limiting. This removes the asymmetric
pivot that made one slipping or stalled wheel appear as false linear encoder
motion. Structured car behaviour, mixed-mode tuning and all tank commands are
unchanged.

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

The container uses ROS 2 Jazzy. The binary 2.8.x package exposes graph
diagnostics through `/slam_toolbox/graph_visualization`; the bridge converts
those markers into node and loop-edge counters and uses the native reset API.

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
- All five CTest unit/integration tests pass after the forward-arc change.
- The hardware-runner car smoke checks for `validation`, `tight`, `slalom`,
  `lower` and `hardware_lab` produce zero pure-pivot cycles and zero opposing
  PWM cycles. Several legacy synthetic presets already collide at their
  initial footprint and therefore are not accepted as goal regressions for the
  Manual Gate Editor world.
- Nonzero compact-unstructured car commands respect the 70 PWM minimum.

The `Closed Obstacle Road` hardware-plant regression is improved but not yet
closed: it now keeps encoder/IMU odometry stable and completes the first
bypass instead of remaining in the first safety stop, but the rejoin target at
the upper boundary can still enter a prolonged pivot. It must not be counted
as a passing regression until that rejoin is corrected and rerun.

Docker Engine and the ROS 2 Jazzy sidecar are now installed on the workstation.
The container reaches the active lifecycle state, listens on UDP 9760 and an
end-to-end synthetic scan produced a valid 64 x 64 occupancy grid. Hardware
runs shown in the GUI use the GUI's automatic local forwarding. A headless
runner must instead pass `--slam-bridge-port 9760`; the earlier reports were
captured while no working sidecar response was available and therefore
correctly record the passive reconstruction backend.

The legacy large-coordinate unstructured presets were not redesigned in this
change. Their physical miniaturization is a separate map-design task; the
current hardware focus remains Manual Gate Editor and Hardware Lab.

## Manual Gate Editor follow-up — 2026-07-27 01:27

The three additional runs exposed a mission-semantics defect rather than a
LiDAR field-of-view limitation:

- run `012728` stayed live for `37.58 s`, travelled an estimated `3.03 m`,
  observed candidates in only 23/274 samples, selected one in 17 samples and
  passed no gate;
- runs `012859` and `012940` both stopped as `goal_reached` after exactly two
  gates, in `3.52 s` and `5.70 s` respectively;
- after losing a candidate, the first run continued its forward search beyond
  the editor arena instead of sweeping toward the remaining openings.

The hardware detector saw the complete 360-degree scan, but its candidate
score and second-stage planner score were both biased toward the editor's
virtual global goal. After the first physical aperture the detector also
inserted that virtual goal into the candidate list. Together with the generic
two-gate completion condition, this explained both symptoms: lateral/rear
openings lost to forward candidates, and a straight virtual-goal trajectory
could terminate a scene that contained more physical gates.

Manual Gate Editor is now an explicit open-ended exploration contract:

- no virtual global goal is inserted or used to score dynamic gates;
- confirmed candidates remain eligible to about 145 degrees from the current
  heading and persistent tracks remain associated through almost the full
  LiDAR azimuth;
- forward and goal-alignment weights are reduced, while lateral coverage is
  rewarded after the initial forward acquisition period;
- the recovery sweep alternates direction, searches a 280-degree sector and
  rejects every probe whose robot footprint would leave the 1.20 m arena;
- when no safe arc exists the car holds still and keeps scanning;
- there is no automatic completion after two apertures: the operator saves or
  stops the manual run when the intended physical set has been traversed.

This open-ended rule is deliberately limited to the `Custom` Manual Gate
Editor world. Fixed presets such as Hardware Lab keep their configured finite
gate count and automatic `goal_reached` behaviour.

The editor world is also normalized to the selected hardware scale as soon as
the GUI state is initialized. The displayed start, goal and 1.20 m bounds can
therefore no longer retain the raw 2.0 m preset geometry until the first
manual edit.

## Slam Toolbox crash follow-up — 2026-07-27

The reported mapper abort was reproduced and traced to the Karto probability
grid. With correlation dimension `0.35 m` and resolution `0.01 m`, Karto
created 36 cells. Its coarse matcher samples at twice that resolution; the
last sample therefore landed one cell beyond the even-sized probability grid
and threw `Mapper FATAL ERROR - unable to get pointer in probability search`.

The sidecar now uses:

- `correlation_search_space_dimension: 0.36`, yielding the symmetric odd
  count `0.36 / 0.01 + 1 = 37`;
- `max_laser_range: 1.2`, matching the incoming hardware scan instead of the
  previous 2.0 m setting;
- validation on both the C++ and Python sides that drops non-finite metadata,
  invalid beams and scans with fewer than eight finite returns;
- lifecycle supervision that makes Docker exit nonzero if the real
  `/slam_toolbox` node disappears, rather than leaving a misleadingly active
  UDP-only container.

The rebuilt Jazzy container survived an invalid-packet injection followed by
201 synthetic full-circle scans. It remained active and returned 43 map
updates, a 48 x 48 occupancy grid, valid corrected pose and 100 graph nodes.
The synthetic exact-odometry path did not require a loop edge; loop closure
must be assessed with the next real repeated-area hardware run.

For GUI-driven hardware tests, the GUI forwards the Raspberry Pi stream to
the sidecar on local UDP 9760. `--slam-bridge-host/port` belong only to a
headless runner that sends directly to the workstation. Do not enable both
forwarding paths for the same run, because alternating session identifiers
would reset the pose graph.

## Dynamic clothoid tracking follow-up — 2026-07-27 02:00

Runs analysed:

- `thesis_hardware_unstructured_manual_gate_editor_gui_bundle_20260727_020004_126`;
- `thesis_hardware_unstructured_manual_gate_editor_gui_bundle_20260727_020210_466`.

The first run passed one gate, but a planner reference existed for only 36 of
321 samples. The second run passed none and had a reference for 41 of 203
samples. In those 41 samples, 38 commands still used a large direct yaw rate.
The GUI clothoid was therefore valid and visible, but it was not the command
contract executed by the wheels.

Four causes were found in the trajectory-to-control hand-off:

1. Manual Gate Editor inherited the generic MPC preview of `1.4 m` and minimum
   lookahead of `0.6 m`; both are larger than the local 0.2--0.5 m clothoids.
2. The MPC anchor index survived when a reference was regenerated from the
   robot's new projection, so it could start near the previous endpoint.
3. Any gate-heading error above about eight degrees replaced the valid MPC
   result with direct pursuit of the gate centre.
4. Close-range loss of the two LiDAR boundary returns could unlock the target
   before the robot centre crossed the aperture plane.

The Manual Gate Editor car now uses an 0.10 m preview and 0.055 m minimum
lookahead, resets the anchor for every regenerated local reference, and keeps
the selected static aperture committed for up to 30 seconds or until its
physical crossing. The terminal clothoid heading comes from the measured
aperture-midpoint-to-beyond-gate segment instead of the diagonal
robot-to-target chord.

While a valid clothoid exists, normal heading corrections come exclusively
from its MPC follower. Direct acquisition is reserved for a path-tangent error
above 40 degrees, with a 20-degree release threshold; even that recovery aligns
to the clothoid tangent rather than to the gate centre. Structured, Mixed,
tracked vehicles and fixed unstructured completion rules are unchanged.

After the change all six deterministic unstructured car presets reach the goal
in both Ideal and Calibrated modes (12/12), and all five CTest tests pass. The
Manual Gate Editor itself requires a real LiDAR rerun because its geometry is
operator-defined and intentionally has no deterministic preset.

## Car clothoid actuation follow-up — 2026-07-27 02:20

Runs analysed:

- `thesis_hardware_unstructured_manual_gate_editor_gui_bundle_20260727_021855_700`;
- `thesis_hardware_unstructured_manual_gate_editor_gui_bundle_20260727_022027_774`.

The earlier trajectory fix is active: the two runs retain an MPC reference for
60 and 180 samples respectively, instead of the 36--41 samples seen before.
The remaining straight motion is downstream of the clothoid:

- in run `021855`, all 60 reference samples request non-zero yaw, but 56/60
  planned commands are flattened to equal PWM; the mean requested yaw is
  `0.122 rad/s` while the mean wheel PWM difference is only `0.67`;
- in run `022027`, all 180 reference samples request non-zero yaw and the mean
  wheel PWM difference is `14.94`, but every one of the 135 samples with both
  a meaningful PWM difference and encoder motion responds on the opposite
  physical encoder side (`0/135` matching, `135/135` opposite).

Equal-PWM straight-line calibration cannot reveal this second issue. The data
show that the controller's electrical motor channels are reversed relative to
the physical left/right encoder labels. The calibrated car profile is therefore
version `1.2.0` and records `controller_motor_channels_swapped: true`.

The planner, MPC, wheel feedback and localization continue to use physical
left/right semantics. Only the final hardware transport swaps motor targets;
encoder observations are deliberately not swapped. The same mapping is applied
to direct PWM and to the future MCU wheel-velocity mode. Synthetic controllers
retain canonical channel ordering, so the wiring compatibility flag is disabled
under `--simulate`.

The compact forward-arc controller now also preserves a yaw-dependent PWM
difference after slew limiting and after the final minimum-PWM floor. A non-zero
clothoid turn can no longer collapse to `70/70`; the enforced difference is
bounded to 8--18 PWM according to requested yaw rate.

Verification after the complete change:

- `thesis_robot_runner`, `thesis_planner_sim` and
  `thesis_world_stream_smoke` build successfully;
- all five CTest unit/integration tests pass;
- all six deterministic unstructured car presets reach the goal in both Ideal
  and Calibrated dynamics: 12/12 runs;
- the structured hardware-runner simulation reaches `goal_reached` in 490
  steps, confirming that real wiring compensation is not applied to the
  synthetic controller.

The next Manual Gate Editor hardware run must confirm the physical result. At
startup it must print `startup_calibration_profile_version=1.2.0` and
`startup_controller_motor_channels_swapped=1`. In the next report the planned
logical PWM faster side should agree with the faster physical encoder side,
while controller target telemetry is expected to show the intentionally
swapped electrical channel order.
