#!/bin/sh
set -eu

script_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
repository_root=$(CDPATH= cd -- "$script_dir/.." && pwd)

executable="$repository_root/build-ninja/simulator/thesis_sim_lidar_start_matcher"
if [ ! -x "$executable" ] && [ -x "$repository_root/build/simulator/thesis_sim_lidar_start_matcher" ]; then
  executable="$repository_root/build/simulator/thesis_sim_lidar_start_matcher"
fi
if [ ! -x "$executable" ]; then
  echo "Missing $executable" >&2
  echo "Build it with: cmake --build build-ninja --target thesis_sim_lidar_start_matcher" >&2
  exit 2
fi

cd "$repository_root"
exec "$executable" \
  --scenario mixed \
  --mixed-map obstacle \
  --vehicle-model car \
  --calibrated \
  --runs 20 \
  --random-offsets \
  --confirmations 3 \
  --scan-count 6 \
  "$@"
