#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/humble/setup.bash
ros2 run slam_toolbox async_slam_toolbox_node \
  --ros-args --params-file /opt/thesis_slam_bridge/mapper_params.yaml &
SLAM_PID=$!
trap 'kill ${SLAM_PID} 2>/dev/null || true' EXIT INT TERM
python3 /opt/thesis_slam_bridge/bridge_node.py
