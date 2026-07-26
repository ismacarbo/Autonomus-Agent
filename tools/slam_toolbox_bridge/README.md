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

Then start the GUI normally. It automatically sends the live hardware scans to
UDP `127.0.0.1:9760`. The SLAM panel changes from `LiDAR reconstruction` to
`SLAM Toolbox` after the first map response.

Each new simulation or hardware run resets the active pose graph. The bridge
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

Docker and host networking are required by `run.sh`. For a headless runner
connected directly to a sidecar, add `--slam-bridge-port 9760` and optionally
`--slam-bridge-host HOST`. Do not enable `--slam-pose-feedback` until the map
pose has been checked against the independent external ground truth.
