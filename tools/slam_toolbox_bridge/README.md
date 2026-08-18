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
