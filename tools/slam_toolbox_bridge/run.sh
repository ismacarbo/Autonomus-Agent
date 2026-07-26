#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IMAGE="thesis-slam-toolbox-bridge:jazzy"

if command -v docker >/dev/null 2>&1; then
  DOCKER=(docker)
elif command -v flatpak-spawn >/dev/null 2>&1; then
  DOCKER=(flatpak-spawn --host docker)
else
  echo "Docker is required to run the ROS 2 slam_toolbox sidecar." >&2
  exit 1
fi

"${DOCKER[@]}" build -t "${IMAGE}" "${ROOT_DIR}"
"${DOCKER[@]}" run --rm --network host --name thesis-slam-toolbox-bridge "${IMAGE}"
