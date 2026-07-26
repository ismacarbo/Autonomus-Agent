#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUNNER="${ROOT_DIR}/build/simulator/thesis_planner_sim"
OUTPUT_DIR="${ROOT_DIR}/results/primary_mvc_regression_matrix"
SUMMARY="${OUTPUT_DIR}/summary.tsv"
MAX_STEPS="${MAX_STEPS:-4000}"
TIMEOUT_SECONDS="${TIMEOUT_SECONDS:-180}"

mkdir -p "${OUTPUT_DIR}"
if [[ ! -x "${RUNNER}" ]]; then
  echo "Missing ${RUNNER}; build thesis_planner_sim first." >&2
  exit 2
fi

failures=0
printf 'mode\tpreset\trobot\tlevel\tstatus\ttime_s\tsteps\tpassed_gates\tdistance_to_goal_m\tartifacts\treport_json\n' >"${SUMMARY}"

run_case() {
  local mode="$1"
  local preset="$2"
  local robot="$3"
  local level="$4"
  local map_flag="$5"
  local output="${OUTPUT_DIR}/${mode}_${preset}_${robot}_${level}.txt"
  local args=(
    --headless
    --scenario "${mode}"
    --vehicle-model "${robot}"
    --sim-level "${level}"
    --max-steps "${MAX_STEPS}"
  )
  if [[ -n "${map_flag}" ]]; then
    args+=("${map_flag%%=*}" "${map_flag#*=}")
  fi
  if [[ "${mode}" != "structured" ]]; then
    args+=(--dynamic-lidar-gates)
  fi

  set +e
  timeout "${TIMEOUT_SECONDS}s" "${RUNNER}" "${args[@]}" >"${output}" 2>&1
  local rc=$?
  set -e

  local status time_s steps passed_gates distance_to_goal report_json slam_png artifacts
  status="$(sed -n 's/^status=//p' "${output}" | tail -n 1)"
  [[ -n "${status}" ]] || status="runner_error"
  time_s="$(sed -n 's/^time=//p' "${output}" | tail -n 1)"
  steps="$(sed -n 's/^steps=//p' "${output}" | tail -n 1)"
  passed_gates="$(sed -n 's/^passed_gates=//p' "${output}" | tail -n 1)"
  distance_to_goal="$(sed -n 's/^distance_to_goal=//p' "${output}" | tail -n 1)"
  report_json="$(sed -n 's/^report_json=//p' "${output}" | tail -n 1)"
  slam_png="${report_json%.json}_slam_reference.png"
  artifacts="missing"
  if [[ -n "${report_json}" && -s "${report_json}" && -s "${slam_png}" ]]; then
    artifacts="json+slam_png"
  fi

  printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n' \
    "${mode}" "${preset}" "${robot}" "${level}" "${status}" \
    "${time_s:-n/a}" "${steps:-n/a}" "${passed_gates:-n/a}" \
    "${distance_to_goal:-n/a}" "${artifacts}" "${report_json:-n/a}" >>"${SUMMARY}"
  printf '%-13s %-18s %-7s %-11s %s\n' \
    "${mode}" "${preset}" "${robot}" "${level}" "${status}"

  if [[ ${rc} -ne 0 || "${status}" != "goal_reached" || "${artifacts}" != "json+slam_png" ]]; then
    failures=$((failures + 1))
  fi
}

# Structured presets actually used in the validation workflow.
for preset in validation circle; do
  for robot in car tank; do
    for level in ideal calibrated; do
      run_case structured "${preset}" "${robot}" "${level}" \
        "--structured-map=${preset}"
    done
  done
done

# All deterministic unstructured presets. The manual editor is intentionally
# excluded because it has no single reproducible geometry.
for preset in validation tight slalom lower hardware_lab ideal; do
  for robot in car tank; do
    for level in ideal calibrated; do
      run_case unstructured "${preset}" "${robot}" "${level}" \
        "--unstructured-map=${preset}"
    done
  done
done

# Mixed presets used in the laboratory workflow.
for preset in obstacle hardware; do
  for robot in car tank; do
    for level in ideal calibrated; do
      run_case mixed "${preset}" "${robot}" "${level}" \
        "--mixed-map=${preset}"
    done
  done
done

if [[ ${failures} -ne 0 ]]; then
  echo "primary_mvc_regression_failures=${failures}" >&2
  exit 1
fi
echo "primary_mvc_regression=all_goal_reached_with_json_and_slam_png"
