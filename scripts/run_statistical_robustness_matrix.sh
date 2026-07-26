#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUNNER="${ROOT_DIR}/build/simulator/thesis_planner_sim"
OUTPUT_DIR="${OUTPUT_DIR:-${ROOT_DIR}/results/statistical_robustness_matrix}"
SEEDS="${SEEDS:-20}"
FIRST_SEED="${FIRST_SEED:-1}"
MAX_STEPS="${MAX_STEPS:-4000}"
TIMEOUT_SECONDS="${TIMEOUT_SECONDS:-180}"
MIN_SUCCESS_RATE="${MIN_SUCCESS_RATE:-0.95}"

if [[ ! -x "${RUNNER}" ]]; then
  echo "Missing ${RUNNER}; build thesis_planner_sim first." >&2
  exit 2
fi
mkdir -p "${OUTPUT_DIR}"
RUNS_TSV="${OUTPUT_DIR}/runs.tsv"
AGGREGATE_TSV="${OUTPUT_DIR}/aggregate.tsv"
printf 'mode\tpreset\trobot\tlevel\tseed\tstatus\trc\treport_json\n' >"${RUNS_TSV}"

run_case() {
  local mode="$1" preset="$2" robot="$3" level="$4" seed="$5" map_flag="$6"
  local stem="${mode}_${preset}_${robot}_${level}_seed_${seed}"
  local output="${OUTPUT_DIR}/${stem}.txt"
  local args=(
    --headless --scenario "${mode}" --vehicle-model "${robot}"
    --sim-level "${level}" --simulation-seed "${seed}"
    --max-steps "${MAX_STEPS}"
  )
  args+=("${map_flag%%=*}" "${map_flag#*=}")
  if [[ "${mode}" != structured ]]; then
    args+=(--dynamic-lidar-gates)
  fi
  set +e
  timeout "${TIMEOUT_SECONDS}s" "${RUNNER}" "${args[@]}" >"${output}" 2>&1
  local rc=$?
  set -e
  local status report_json
  status="$(sed -n 's/^status=//p' "${output}" | tail -n 1)"
  report_json="$(sed -n 's/^report_json=//p' "${output}" | tail -n 1)"
  [[ -n "${status}" ]] || status=runner_error
  printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n' \
    "${mode}" "${preset}" "${robot}" "${level}" "${seed}" \
    "${status}" "${rc}" "${report_json:-n/a}" >>"${RUNS_TSV}"
}

run_family() {
  local mode="$1" preset="$2" map_flag="$3"
  local robot level index seed
  for robot in car tank; do
    for level in ideal calibrated; do
      for ((index=0; index<SEEDS; ++index)); do
        seed=$((FIRST_SEED + index))
        run_case "${mode}" "${preset}" "${robot}" "${level}" "${seed}" "${map_flag}"
      done
    done
  done
}

# Primary workflow only: the two structured maps in use, every reproducible
# unstructured map, and the two mixed maps used in the laboratory.
run_family structured validation --structured-map=validation
run_family structured circle --structured-map=circle
for preset in validation tight slalom lower hardware_lab ideal; do
  run_family unstructured "${preset}" "--unstructured-map=${preset}"
done
run_family mixed obstacle --mixed-map=obstacle
run_family mixed hardware --mixed-map=hardware

awk -F '\t' -v threshold="${MIN_SUCCESS_RATE}" '
  NR == 1 { next }
  {
    key=$1 FS $2 FS $3 FS $4
    total[key]++
    if ($6 == "goal_reached" && $7 == 0) success[key]++
  }
  END {
    print "mode\tpreset\trobot\tlevel\truns\tsuccesses\tsuccess_rate\tstable"
    failed=0
    for (key in total) {
      rate=success[key]/total[key]
      stable=(rate >= threshold ? "yes" : "no")
      printf "%s\t%d\t%d\t%.6f\t%s\n", key, total[key], success[key], rate, stable
      if (stable == "no") failed=1
    }
    exit failed
  }
' "${RUNS_TSV}" >"${AGGREGATE_TSV}"

echo "runs=${RUNS_TSV}"
echo "aggregate=${AGGREGATE_TSV}"
