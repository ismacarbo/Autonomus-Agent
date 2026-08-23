#!/bin/sh
set -eu

if [ "$#" -lt 1 ]; then
  echo "Usage: $0 /dev/ttyACM0 [additional diagnostic options]" >&2
  exit 2
fi

script_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
repository_root=$(CDPATH= cd -- "$script_dir/.." && pwd)
controller_port=$1
shift

executable="$repository_root/build-ninja/simulator/thesis_drivetrain_diagnostics"
if [ ! -x "$executable" ] && [ -x "$repository_root/build/simulator/thesis_drivetrain_diagnostics" ]; then
  executable="$repository_root/build/simulator/thesis_drivetrain_diagnostics"
fi
if [ ! -x "$executable" ]; then
  echo "Missing $executable" >&2
  echo "Build it with: cmake --build build-ninja --target thesis_drivetrain_diagnostics" >&2
  exit 2
fi

cd "$repository_root"
exec "$executable" --controller-port "$controller_port" "$@"
