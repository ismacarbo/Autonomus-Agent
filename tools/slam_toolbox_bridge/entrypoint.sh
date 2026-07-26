#!/usr/bin/env bash
set -eo pipefail

# ROS setup scripts probe optional variables, so nounset must only be enabled
# after the environment has been loaded.
source /opt/ros/jazzy/setup.bash
set -u

# The Jazzy executable is a lifecycle node. The upstream launch file performs
# the configure/activate transitions before scans are accepted.
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:=/opt/thesis_slam_bridge/mapper_params.yaml \
  use_sim_time:=false &
SLAM_PID=$!
trap 'kill ${SLAM_PID} 2>/dev/null || true' EXIT INT TERM
python3 /opt/thesis_slam_bridge/bridge_node.py
