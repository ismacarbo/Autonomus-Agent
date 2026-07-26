#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUNNER="${ROOT_DIR}/build/simulator/thesis_planner_sim"
OUTPUT_DIR="${ROOT_DIR}/results/canonical_simulation_matrix"
mkdir -p "${OUTPUT_DIR}"
SUMMARY="${OUTPUT_DIR}/summary.tsv"

if [[ ! -x "${RUNNER}" ]]; then
  echo "Missing ${RUNNER}; build thesis_planner_sim first." >&2
  exit 2
fi

failures=0
printf 'mode\trobot\tlevel\tstatus\ttime_s\tsteps\tpassed_gates\tdistance_to_goal_m\n' >"${SUMMARY}"
printf '%-14s %-8s %-15s %-14s\n' "mode" "robot" "level" "status"
for mode in structured unstructured mixed; do
  for robot in car tank; do
    for level in ideal calibrated; do
      output="${OUTPUT_DIR}/${mode}_${robot}_${level}.txt"
      args=(--headless --scenario "${mode}" --vehicle-model "${robot}" --max-steps 1800)
      if [[ "${level}" == "ideal" ]]; then
        args+=(--ideal-sim)
      else
        args+=(--calibrated-sim)
      fi
      if [[ "${mode}" != "structured" ]]; then
        args+=(--dynamic-lidar-gates)
      fi
      set +e
      timeout 180s "${RUNNER}" "${args[@]}" >"${output}" 2>&1
      rc=$?
      set -e
      status="$(sed -n 's/^status=//p' "${output}" | tail -n 1)"
      [[ -n "${status}" ]] || status="runner_error"
      time_s="$(sed -n 's/^time=//p' "${output}" | tail -n 1)"
      steps="$(sed -n 's/^steps=//p' "${output}" | tail -n 1)"
      passed_gates="$(sed -n 's/^passed_gates=//p' "${output}" | tail -n 1)"
      distance_to_goal="$(sed -n 's/^distance_to_goal=//p' "${output}" | tail -n 1)"
      printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n' \
        "${mode}" "${robot}" "${level}" "${status}" \
        "${time_s:-n/a}" "${steps:-n/a}" "${passed_gates:-n/a}" "${distance_to_goal:-n/a}" \
        >>"${SUMMARY}"
      printf '%-14s %-8s %-15s %-14s\n' "${mode}" "${robot}" "${level}" "${status}"
      if [[ ${rc} -ne 0 || "${status}" != "goal_reached" ]]; then
        failures=$((failures + 1))
      fi
    done
  done
done

if [[ ${failures} -ne 0 ]]; then
  echo "canonical_matrix_failures=${failures}" >&2
  exit 1
fi
echo "canonical_matrix=all_goal_reached"
