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
