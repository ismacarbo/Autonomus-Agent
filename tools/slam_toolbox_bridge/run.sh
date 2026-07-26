#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IMAGE="thesis-slam-toolbox-bridge:jazzy"
CONTAINER="thesis-slam-toolbox-bridge"

if command -v docker >/dev/null 2>&1; then
  DOCKER=(docker)
elif command -v flatpak-spawn >/dev/null 2>&1; then
  DOCKER=(flatpak-spawn --host docker)
else
  echo "Docker is required to run the ROS 2 slam_toolbox sidecar." >&2
  exit 1
fi

if "${DOCKER[@]}" container inspect "${CONTAINER}" >/dev/null 2>&1; then
  if [[ "$("${DOCKER[@]}" inspect --format '{{.State.Running}}' "${CONTAINER}")" == "true" ]]; then
    echo "${CONTAINER} is already running; following its logs (Ctrl+C only stops log output)."
    exec "${DOCKER[@]}" logs --follow "${CONTAINER}"
  fi

  # A stopped container cannot be reused by `docker run --name`. It contains
  # no persistent state: the SLAM session is deliberately recreated per run.
  "${DOCKER[@]}" container rm "${CONTAINER}" >/dev/null
fi

"${DOCKER[@]}" build -t "${IMAGE}" "${ROOT_DIR}"
"${DOCKER[@]}" run --rm --network host --name "${CONTAINER}" "${IMAGE}"
