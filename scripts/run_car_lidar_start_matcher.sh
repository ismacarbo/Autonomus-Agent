#!/bin/sh
set -eu

if [ "$#" -lt 2 ]; then
  echo "Usage:" >&2
  echo "  $0 /dev/ttyUSB0 --capture-reference datasets/localization/car_start_reference.csv" >&2
  echo "  $0 /dev/ttyUSB0 --reference datasets/localization/car_start_reference.csv" >&2
  exit 2
fi

script_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
repository_root=$(CDPATH= cd -- "$script_dir/.." && pwd)
lidar_port=$1
shift

executable="$repository_root/build-ninja/simulator/thesis_lidar_start_matcher"
if [ ! -x "$executable" ] && [ -x "$repository_root/build/simulator/thesis_lidar_start_matcher" ]; then
  executable="$repository_root/build/simulator/thesis_lidar_start_matcher"
fi
if [ ! -x "$executable" ]; then
  echo "Missing $executable" >&2
  echo "Build it with: cmake --build build-ninja --target thesis_lidar_start_matcher" >&2
  exit 2
fi

cd "$repository_root"
exec "$executable" --lidar-port "$lidar_port" --vehicle-model car "$@"
