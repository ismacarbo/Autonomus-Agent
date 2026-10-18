Hardware Mixed observation fixture, not a simulator scenario.

`mixed_20261017_160347_observed_scans.txt` contains the last five measured scans
from `thesis_hardware_mixed_mixed_hardware_road_gate_gui_snapshot_20261017_160347_774_lidar_reconstruction.csv`.
The timestamps are relative to the final recorded scan; only the last 0.60 s
are retained, matching the hardware observation memory. Each scan starts with
`timestamp origin_x origin_y point_count`, followed by measured `hit_x hit_y`.
Origins use the recorded pose and calibrated LiDAR translation (0.075, 0.040) m.
Final recorded robot pose: (0, 0, 0.000480 rad). The GUI export has already
filtered returns and rounded coordinates; this is not the raw wire stream.

The test must retain the unobserved sector around -120 degrees and reject
motion whose padded footprint cannot be verified. No missing returns are
filled in and no coordinates in this fixture enter a hardware mission.

`mixed_20261017_163834_observed_scans.txt` adds the last five scans of
`thesis_hardware_mixed_mixed_hardware_road_gate_gui_snapshot_20261017_163834_681_lidar_reconstruction.csv`.
They cover times 22.306581–22.868832 s, with pose (0, 0, 0 rad), and use the
same format/extrinsics. The runtime log independently reports the same
`current_unknown_footprint` witness near (-0.0586364, -0.165) m. As with the
older export, missing far returns/quality information cannot be recovered
from this filtered CSV. Only the separate raw-capture checker can apply the
real navigation filters to raw returns. Both fixtures must remain rejected;
an offline observed-clear synthetic capture is tested separately.

`mixed_20261017_160242_raw_scans.csv` contains the last five actual raw capture
scans (IDs 5–9, 1,293 positive-range returns) from
`car_mixed_local_start_20261017_160242_473086656_raw_scans.csv`, supplied by the
operator on 2026-10-18. Angles, ranges, quality and matcher decisions are
unchanged. Start/end timestamps are shifted by one common epoch; unused
columns are omitted. This time the fixture includes the five returns above
2.5 m: the navigation converter keeps them as capped non-occupied rays, unlike
the matcher. Even with them, the current footprint remains unobserved. The
test checks that no recorded return is dropped by the navigation filters.
This file is only an offline regression input, never a mission obstacle map.
The driver's raw export omits zero-range nodes, so it still cannot separate
sensor no-returns from acquisition loss at the wire level.

`mixed_20261018_094252_rejoin_scan.txt` contains the final filtered scan (238
returns) of `thesis_hardware_mixed_mixed_hardware_road_gate_gui_snapshot_20261018_094252_934`.
Header: `pose_x pose_y yaw road_progress point_count`; rows: world angle and
measured range. Recorded pose: (0.866818, 0.362334, 0.534017 rad). The operator
explicitly pushed the robot/moved obstacles after the first snapshot, so this
is a regression of planning from that pose, not an autonomous mission success.
Both forward rejoin candidates exceed the car's unchanged curvature limit.
The test checks stable failure during the retry interval, continued retries,
actual sensor-loss priority, and recovery after injecting feasible geometry.

With gate-return checking enabled, the older 16:38 fixture in obstacles-only
mode passes initial footprint coverage but rejects all nine gate connections:
none has a verified return to its 1.30 m road. The previous expectation of
GATE_FOLLOWING was therefore replaced with an explicit geometric-dead-end
rejection. Separate larger mathematical fixtures still require a feasible
gate/return, stable MPC tracking and two successive bypass/rejoin cycles.

## Mixed 2026-10-18 10:18:39 — relaxed planning comparison

`mixed_20261018_101839_observed_scans.txt` contains the last five scans (1,286
reconstructed endpoints, t=54.204593–54.799871 s) from
`reports/thesis_hardware_mixed_mixed_hardware_road_gate_gui_snapshot_20261018_101839_285_lidar_reconstruction.csv`.
Source SHA-256: `122ea0d7855a8bdef3119d4fe571741ce2edbb478b624682ca7abb91d7f3ab93`.
Fixture SHA-256: `bfcf5e1054c92b55ab4c43f8dc709a7613371dceb74af02f3b5c47d3ac0e0294`.

Each scan header is `timestamp pose_x pose_y pose_yaw origin_x origin_y count`,
followed by `count` rows of `world_angle_rad range_m`. Regenerate with
`python3 tools/extract_mixed_relaxed_fixture.py`. No missing rays are fabricated.
Raw quality and scan durations are unavailable in this export. The comparison
uses the same reconstructed observations and gate geometries for both policies;
it does not replay every search in the original terminal log. Collision tests
use zero motion inflation for this stationary snapshot; hardware retains live
motion uncertainty. See `documentation/Hardware_Mixed_Experimental_Relaxed.md`.
