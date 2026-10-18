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
