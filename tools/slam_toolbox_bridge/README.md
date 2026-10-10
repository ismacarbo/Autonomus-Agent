# SLAM Toolbox bridge

This sidecar converts the thesis planner's odometry and LiDAR stream into the
ROS 2 messages consumed by the original `slam_toolbox` implementation. It is
the graph-SLAM backend for scan matching, pose-graph optimization and loop
closure; the C++ endpoint accumulation remains a clearly labelled fallback.
The container uses ROS 2 Jazzy so the pose graph and per-session reset API are
available without adding ROS dependencies to the C++ project. The binary Jazzy
2.8.x package exposes graph diagnostics through
`/slam_toolbox/graph_visualization`; the bridge converts those markers into
node and loop-edge counters for the existing UDP protocol.

## Workstation usage

Run the sidecar on the same workstation as the GUI:

```sh
./tools/slam_toolbox_bridge/run.sh
```

The launcher is idempotent. If the named sidecar is already running, it
attaches to its logs instead of attempting to create a conflicting container;
`Ctrl+C` then stops only the log view.

Then start the GUI and the Raspberry runner normally. The Raspberry runner is
the only hardware SLAM client and sends odometry plus LiDAR directly to UDP
`9760` on the workstation. The GUI only renders the resulting occupancy grid;
it never forwards hardware scans a second time. This avoids interleaved
sessions and accidental pose-graph resets.

Each new process/run has a unique session ID and resets the active pose graph
once. Duplicated or reordered UDP packets are discarded and do not reset the
map. The bridge
publishes:

- `odom -> base_link` and `base_link -> laser` transforms;
- a uniform 360-bin `/scan` message;
- the measured LiDAR translation relative to `base_link`;
- free and occupied `/map` cells back to the GUI;
- the corrected map pose derived from `map -> odom`;
- pose-graph node and loop-edge counters when a graph update is published.

When saving a thesis bundle:

- `*_slam_reference.png` is the optimized occupancy grid if the sidecar is
  connected;
- `*_lidar_reconstruction.png` is always the passive raw reconstruction;
- the JSON `slam_backend` object records which backend actually produced the
  reference image.

Docker and host networking are required by `run.sh`. SLAM is enabled by
default in the runner. With `--stream-host <pc-ip>`, that host is also used as
the default SLAM bridge host; the GUI launch hint nevertheless emits the
endpoint explicitly. The GUI `SLAM Toolbox` checkbox can disable or re-enable
submission at runtime. If disabled or disconnected, navigation falls back to
a non-decaying local free/occupied grid for the current run. Do not enable
`--slam-pose-feedback` until the map pose has been checked against independent
external ground truth.

## Comparing SLAM before enabling feedback

Use `--slam-observe-only` on the runner for a first comparison. This records
the scan-matched pose and its innovation against the odometry of the same
scan, while keeping both EKF feedback and the navigation map unchanged.
It is **not** a stationary robot mode: normal navigation still runs.
The flag is incompatible with `--slam-pose-feedback` and survives GUI profile
and map changes. Keep the GUI's SLAM Toolbox checkbox enabled and remove
`--no-slam-toolbox` from the runner command.

```sh
./build-ninja/simulator/thesis_robot_runner \
  --controller-port /dev/ttyACM0 --lidar-port /dev/ttyUSB0 \
  --scenario unstructured --unstructured-map hardware_lab --vehicle-model car \
  --pose-fusion auto --no-start-matching \
  --slam-observe-only --slam-bridge-host 100.66.27.57 --slam-bridge-port 9760 \
  --stream-host 100.66.27.57 --stream-port 9559 \
  --stop-on-stream-loss --max-steps 200
```

Check `slam_toolbox_connected=1`, `slam_pose_valid=1` and increasing graph/map
counters. In JSON history, inspect `slam_pose_x/y/yaw`,
`slam_position_innovation_m`, `slam_yaw_innovation_deg`; in this mode
`slam_correction_accepted` must stay zero. The reference PNG remains the local
navigation grid, identified as such; the comparison poses are in the history.
Scan matching still uses the same LiDAR and an odometry prior, so it is not
independent ground truth. The October 10 stationary front/right captures
identified the incorrect car LiDAR frame in profile 1.7.0. Profile 1.8.0 uses
zero yaw offset and mirrored raw angles: raw 0 degrees is forward and raw
+90 degrees is physical right. Update the calibration file on both machines
and restart the runner and GUI; recapture start references made with the old
orientation. Validate the resulting live SLAM/odometry comparison before using
those poses as feedback or claiming gate accuracy.

Replies are paired with a bounded history of submitted scan poses. Unknown
sessions, out-of-order sequences, nonfinite poses and responses older than two
seconds are discarded. The SLAM frame is aligned once at session start; drift
is retained in the comparison. When normal map use is enabled, occupied/free
cells are additionally transformed into the runner frame before collision
checks. SLAM poses are logged even when EKF feedback is off, and disconnects
invalidate them.

After updating the adapter, restart the sidecar so it loads the rebuilt image.
`run.sh` attaches to a container already running; it does not replace its code.

## Input and Karto grid constraints

The bridge rejects malformed metadata, non-finite beams and scans with fewer
than eight finite returns before publishing `/scan`. The C++ sender performs
the same validation, so corrupt or nearly empty UDP datagrams cannot enter the
scan matcher.

`max_laser_range` is `1.2 m`, matching the range published by the hardware
pipeline. Keep the Karto correlation grid symmetric when tuning its search
space. The search dimension divided by the search resolution, plus one, must
produce an odd cell count. The current `0.36 / 0.01 + 1 = 37` configuration
satisfies that constraint. The previous `0.35 / 0.01 + 1 = 36` configuration
let the final coarse-search sample fall outside the probability grid and could
terminate Slam Toolbox with `unable to get pointer in probability search`.

The container entrypoint supervises both the UDP adapter and the actual
`/slam_toolbox` lifecycle node. If the mapper aborts, the container now exits
with an error instead of remaining deceptively `Up` with only the UDP process
alive. Re-running `run.sh` replaces a stopped container and attaches to an
already running one.
