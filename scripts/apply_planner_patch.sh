#!/usr/bin/env bash
set -euo pipefail

repository_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
planner_dir="${repository_root}/third_party/progettotesi"
patch_file="${repository_root}/patches/progettotesi-small-robot-gate-distance.patch"

if [[ ! -f "${planner_dir}/src/lib/behaviours.cpp" ]]; then
  echo "Planner submodule missing. Run: git submodule update --init --recursive" >&2
  exit 1
fi

if git -C "${planner_dir}" apply --reverse --check "${patch_file}" >/dev/null 2>&1; then
  echo "Planner small-robot gate patch already applied."
  exit 0
fi

# Check before writing. Incompatible local changes are left untouched.
git -C "${planner_dir}" apply --check "${patch_file}"
git -C "${planner_dir}" apply "${patch_file}"
echo "Applied planner small-robot gate patch."
