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
python3 /opt/thesis_slam_bridge/bridge_node.py &
BRIDGE_PID=$!

cleanup() {
  kill "${BRIDGE_PID}" "${SLAM_PID}" 2>/dev/null || true
  wait "${BRIDGE_PID}" "${SLAM_PID}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# ros2 launch can remain alive after a component process aborts. Supervise the
# actual lifecycle node as well, otherwise Docker reports a healthy container
# while only the UDP adapter is left running.
STARTUP_DEADLINE=$((SECONDS + 20))
SLAM_WAS_HEALTHY=0
while kill -0 "${SLAM_PID}" 2>/dev/null && kill -0 "${BRIDGE_PID}" 2>/dev/null; do
  if ros2 node list 2>/dev/null | grep -qx '/slam_toolbox'; then
    SLAM_WAS_HEALTHY=1
  elif (( SLAM_WAS_HEALTHY == 1 )); then
    echo "slam_toolbox lifecycle node disappeared; stopping sidecar" >&2
    exit 1
  elif (( SECONDS >= STARTUP_DEADLINE )); then
    echo "slam_toolbox lifecycle node did not become ready" >&2
    exit 1
  fi
  sleep 1
done

echo "slam_toolbox bridge process exited unexpectedly" >&2
exit 1
