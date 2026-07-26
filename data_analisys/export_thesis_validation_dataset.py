"""Export the curated thesis validation report subset.

The full local report archive contains many development, tuning, and failed
runs.  This helper copies only the runs used as thesis/release evidence into a
small dataset folder and generates a manifest plus a compact metrics table.
"""

from __future__ import annotations

import csv
import hashlib
import json
import math
import os
import shutil
from dataclasses import dataclass
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = REPO_ROOT / "datasets" / "thesis_validation"

SOURCE_REPORT_DIRS = [
    ("current_reports", REPO_ROOT / "reports"),
    (
        "local_backup_20260625",
        Path(
            os.environ.get(
                "AUTONOMOUS_AGENT_REPORT_BACKUP",
                str(REPO_ROOT.parent / "Autonomus-Agent_local_artifacts_backup_20260625" / "reports"),
            )
        ),
    ),
]


@dataclass(frozen=True)
class DatasetRun:
    run_id: str
    subset: str
    platform: str
    mode: str
    environment: str
    validation_role: str
    notes: str


RUNS = [
    DatasetRun(
        "thesis_planner_structured_ideal_validation_road_ideal_headless_20260516_112936_522",
        "car_structured",
        "car_like",
        "structured",
        "simulation_ideal",
        "multilevel_validation_l0",
        "Ideal structured simulation used as clean reference in the thesis validation.",
    ),
    DatasetRun(
        "thesis_planner_structured_validation_road_headless_20260515_084019_344",
        "car_structured",
        "car_like",
        "structured",
        "simulation_baseline",
        "multilevel_validation_l1",
        "Baseline structured simulation with the same validation road.",
    ),
    DatasetRun(
        "thesis_hardware_structured_validation_road_gui_manual_20260424_005850_634",
        "car_structured",
        "car_like",
        "structured",
        "hardware",
        "multilevel_validation_l2",
        "Validated hardware structured-road run.",
    ),
    DatasetRun(
        "thesis_planner_unstructured_ideal_hardware_lab_lidar_dynamic_ideal_headless_20260516_112926_925",
        "car_unstructured",
        "car_like",
        "unstructured",
        "simulation_ideal",
        "multilevel_validation_l0",
        "Ideal unstructured dynamic-gate simulation.",
    ),
    DatasetRun(
        "thesis_planner_unstructured_hardware_lab_lidar_dynamic_headless_20260515_083542_221",
        "car_unstructured",
        "car_like",
        "unstructured",
        "simulation_baseline",
        "multilevel_validation_l1",
        "Baseline unstructured dynamic-gate simulation.",
    ),
    DatasetRun(
        "thesis_hardware_unstructured_manual_gate_editor_gui_manual_20260513_045435_200",
        "car_unstructured",
        "car_like",
        "unstructured",
        "hardware",
        "multilevel_validation_l2",
        "Validated hardware unstructured-gate run.",
    ),
    DatasetRun(
        "thesis_planner_mixed_mixed_ideal_hardware_aligned_lidar_dynamic_ideal_headless_20260516_112925_029",
        "car_mixed",
        "car_like",
        "mixed",
        "simulation_ideal",
        "multilevel_validation_l0",
        "Ideal mixed road/gate simulation on the hardware-aligned map.",
    ),
    DatasetRun(
        "thesis_planner_mixed_mixed_hardware_aligned_lidar_dynamic_headless_20260515_083538_038",
        "car_mixed",
        "car_like",
        "mixed",
        "simulation_baseline",
        "multilevel_validation_l1",
        "Baseline mixed road/gate simulation used for sim-to-real comparison.",
    ),
    DatasetRun(
        "thesis_hardware_mixed_mixed_hardware_road_gate_gui_manual_20260512_052657_596",
        "car_mixed",
        "car_like",
        "mixed",
        "hardware",
        "multilevel_validation_l2",
        "First validated compact mixed hardware run.",
    ),
    DatasetRun(
        "thesis_hardware_mixed_mixed_hardware_road_gate_gui_manual_20260512_053236_711",
        "car_mixed",
        "car_like",
        "mixed",
        "hardware",
        "multilevel_validation_l2",
        "Second validated compact mixed hardware run.",
    ),
    DatasetRun(
        "thesis_hardware_mixed_mixed_hardware_road_gate_gui_manual_20260621_204747_088",
        "aruco_ground_truth",
        "car_like",
        "mixed",
        "hardware",
        "external_pose_validation",
        "Final full-video run associated with the ArUco ground-truth comparison.",
    ),
    DatasetRun(
        "thesis_hardware_mixed_mixed_hardware_road_gate_gui_manual_20260622_202820_630",
        "tank_mixed",
        "tracked",
        "mixed",
        "hardware",
        "portability_validation",
        "Tracked-vehicle mixed run used as portability evidence, trial 1.",
    ),
    DatasetRun(
        "thesis_hardware_mixed_mixed_hardware_road_gate_gui_manual_20260622_203250_485",
        "tank_mixed",
        "tracked",
        "mixed",
        "hardware",
        "portability_validation",
        "Tracked-vehicle mixed run used as portability evidence, trial 2.",
    ),
    DatasetRun(
        "thesis_hardware_mixed_mixed_hardware_road_gate_gui_manual_20260622_203417_716",
        "tank_mixed",
        "tracked",
        "mixed",
        "hardware",
        "portability_validation",
        "Tracked-vehicle mixed run used as portability evidence, trial 3.",
    ),
    DatasetRun(
        "thesis_planner_mixed_mixed_closed_obstacle_road_tracked_vehicle_lidar_dynamic_gui_auto_20260627_114503_438",
        "simulation_support",
        "tracked",
        "mixed",
        "simulation_baseline",
        "qualitative_sim_real_support",
        "Recent corrected mixed closed-road simulation used only as qualitative support.",
    ),
]


README_TEXT = """# Thesis Validation Dataset

This folder contains a curated subset of the experimental reports used for the
thesis and public project release.  It is intentionally small: the complete
local archive contains development runs, tuning attempts, collisions, old schema
versions, and reports produced before later planner fixes.

Use this folder as a reproducibility artifact, not as a general benchmark.

## Contents

- `manifest.csv`: one row per included report, with provenance and validation
  role.
- `metrics_summary.csv`: compact metrics extracted from each report.
- `archive_summary.json`: high-level count of the larger local report folders at
  export time.
- `raw_reports/`: selected JSON reports grouped by validation subset.

## Subsets

- `car_structured`: structured-road validation on the car-like robot.
- `car_unstructured`: LiDAR gate validation on the car-like robot.
- `car_mixed`: mixed road/gate validation on the car-like robot.
- `aruco_ground_truth`: hardware run associated with external ArUco pose
  validation.
- `tank_mixed`: mixed-mode validation runs on the tracked platform.
- `simulation_support`: recent simulation reports used as qualitative support
  for figures and sim-to-real discussion.

## Excluded Data

The larger report archive is not copied here because it mixes several project
phases:

- early development runs with different report fields;
- failed tuning attempts, collisions, and timeouts;
- runs produced before the mixed-gate fixes;
- runs generated while map and vehicle presets were still being adjusted.

Those files are useful as engineering history, but they would weaken a clean
quantitative dataset if published without curation.

## Regeneration

From the repository root, regenerate this folder with:

```bash
python3 -m data_analisys.export_thesis_validation_dataset
```

If you want to regenerate the dataset from an external backup archive, set:

```bash
export AUTONOMOUS_AGENT_REPORT_BACKUP=/path/to/report/archive
```

If the variable is not set, the exporter falls back to the local backup path
used during thesis cleanup. If that archive is not present, only reports
available in the current repository `reports/` folder can be exported.
"""


def load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8", errors="ignore"))


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def find_report(run_id: str) -> tuple[str, Path] | None:
    name = f"{run_id}.json"
    for label, directory in SOURCE_REPORT_DIRS:
        candidate = directory / name
        if candidate.exists():
            return label, candidate
    return None


def date_from_run_id(run_id: str) -> str:
    for part in run_id.split("_"):
        if len(part) == 8 and part.isdigit():
            return part
    return ""


def safe_float(value: Any) -> float | None:
    if value is None:
        return None
    try:
        out = float(value)
    except (TypeError, ValueError):
        return None
    if math.isfinite(out):
        return out
    return None


def path_length_from_points(points: list[dict[str, Any]], x_key: str, y_key: str) -> float | None:
    total = 0.0
    previous: tuple[float, float] | None = None
    used = 0
    for row in points:
        x = safe_float(row.get(x_key))
        y = safe_float(row.get(y_key))
        if x is None or y is None:
            continue
        if previous is not None:
            total += math.hypot(x - previous[0], y - previous[1])
        previous = (x, y)
        used += 1
    return total if used >= 2 else None


def percentile(values: list[float], p: float) -> float | None:
    if not values:
        return None
    values = sorted(values)
    idx = (len(values) - 1) * p
    lo = math.floor(idx)
    hi = math.ceil(idx)
    if lo == hi:
        return values[int(idx)]
    return values[lo] * (hi - idx) + values[hi] * (idx - lo)


def telemetry_values(rows: list[dict[str, Any]], key: str, absolute: bool = False) -> list[float]:
    values: list[float] = []
    for row in rows:
        value = safe_float(row.get(key))
        if value is None:
            continue
        values.append(abs(value) if absolute else value)
    return values


def extract_metrics(report: dict[str, Any]) -> dict[str, Any]:
    metrics: dict[str, Any] = {
        "status": report.get("status") or report.get("run_state") or report.get("final_state") or "",
        "duration_s": None,
        "steps": None,
        "passed_gates": None,
        "switch_events": None,
        "structured_to_gate_switches": None,
        "gate_to_structured_switches": None,
        "gate_time_s": None,
        "estimated_path_length_m": None,
        "encoder_distance_m": None,
        "min_clearance_m": None,
        "max_cross_track_m": None,
        "heading_error_p95_deg": None,
        "max_heading_error_deg": None,
        "final_goal_distance_m": None,
        "planner_compute_p95_ms": None,
        "sample_count": None,
    }

    thesis_metrics = report.get("thesis_metrics")
    if isinstance(thesis_metrics, dict):
        metrics.update(
            {
                "duration_s": thesis_metrics.get("runtime_s") or thesis_metrics.get("time_to_goal_s"),
                "passed_gates": thesis_metrics.get("passed_gates"),
                "switch_events": thesis_metrics.get("switch_events"),
                "structured_to_gate_switches": thesis_metrics.get("structured_to_gate_switches"),
                "gate_to_structured_switches": thesis_metrics.get("gate_to_structured_switches"),
                "gate_time_s": thesis_metrics.get("gate_time_s"),
                "estimated_path_length_m": thesis_metrics.get("estimated_path_length_m"),
                "encoder_distance_m": thesis_metrics.get("encoder_distance_m"),
                "min_clearance_m": thesis_metrics.get("min_clearance_m"),
                "max_cross_track_m": thesis_metrics.get("max_cross_track_m"),
                "max_heading_error_deg": thesis_metrics.get("max_heading_error_deg"),
                "final_goal_distance_m": thesis_metrics.get("final_goal_distance_m"),
            }
        )
    else:
        summary = report.get("summary")
        if isinstance(summary, dict):
            metrics.update(
                {
                    "duration_s": summary.get("sim_time"),
                    "steps": summary.get("steps"),
                    "passed_gates": summary.get("passed_gates"),
                    "final_goal_distance_m": summary.get("distance_to_goal"),
                }
            )
        performance = report.get("performance")
        if isinstance(performance, dict):
            planning_ms = performance.get("planning_ms")
            if isinstance(planning_ms, dict):
                metrics["planner_compute_p95_ms"] = (
                    planning_ms.get("p95")
                    or planning_ms.get("p95_ms")
                    or planning_ms.get("max")
                    or planning_ms.get("max_ms")
                )

    history = report.get("history")
    if isinstance(history, list):
        metrics["sample_count"] = len(history)
        metrics["steps"] = metrics["steps"] or len(history)
        times = telemetry_values(history, "time")
        if times:
            metrics["duration_s"] = metrics["duration_s"] or max(times)
        distance_to_goal = telemetry_values(history, "distance_to_goal")
        if distance_to_goal:
            metrics["final_goal_distance_m"] = metrics["final_goal_distance_m"] or distance_to_goal[-1]
        min_lidar = telemetry_values(history, "min_lidar")
        front_lidar = telemetry_values(history, "front_lidar")
        clearance_candidates = [value for value in min_lidar + front_lidar if value > 0.0]
        if clearance_candidates:
            metrics["min_clearance_m"] = metrics["min_clearance_m"] or min(clearance_candidates)
        metrics["estimated_path_length_m"] = metrics["estimated_path_length_m"] or path_length_from_points(
            history, "position_x", "position_y"
        )
        passed_gates = telemetry_values(history, "passed_gates")
        if passed_gates:
            metrics["passed_gates"] = metrics["passed_gates"] or int(max(passed_gates))
        planning = telemetry_values(history, "planning_ms")
        if planning:
            metrics["planner_compute_p95_ms"] = metrics["planner_compute_p95_ms"] or percentile(planning, 0.95)
        cross_track = telemetry_values(history, "tracker_cross_track", absolute=True)
        if cross_track:
            metrics["max_cross_track_m"] = metrics["max_cross_track_m"] or max(cross_track)
        heading = telemetry_values(history, "tracker_heading_error_deg", absolute=True)
        metrics["heading_error_p95_deg"] = percentile(heading, 0.95)
    telemetry = report.get("telemetry")
    if isinstance(telemetry, list):
        metrics["sample_count"] = len(telemetry)
        metrics["estimated_path_length_m"] = metrics["estimated_path_length_m"] or path_length_from_points(
            telemetry, "x", "y"
        )
        cross_track = telemetry_values(telemetry, "tracker_cross_track", absolute=True)
        heading = telemetry_values(telemetry, "tracker_heading_error_deg", absolute=True)
        if cross_track:
            metrics["max_cross_track_m"] = max(cross_track)
        metrics["heading_error_p95_deg"] = metrics["heading_error_p95_deg"] or percentile(heading, 0.95)

    return metrics


def summarize_archive() -> dict[str, Any]:
    summary: dict[str, Any] = {}
    for label, directory in SOURCE_REPORT_DIRS:
        entry: dict[str, Any] = {"exists": directory.exists()}
        if directory.exists():
            json_files = sorted(directory.glob("*.json"))
            entry["json_reports"] = len(json_files)
            states: dict[str, int] = {}
            dates: dict[str, int] = {}
            for path in json_files:
                date = date_from_run_id(path.stem)
                if date:
                    dates[date] = dates.get(date, 0) + 1
                try:
                    data = load_json(path)
                except Exception:
                    states["unreadable"] = states.get("unreadable", 0) + 1
                    continue
                state = data.get("status") or data.get("run_state") or data.get("final_state") or "unknown"
                states[str(state)] = states.get(str(state), 0) + 1
            entry["status_counts"] = dict(sorted(states.items()))
            entry["date_counts"] = dict(sorted(dates.items()))
        summary[label] = entry
    return summary


def write_csv(path: Path, rows: list[dict[str, Any]], fieldnames: list[str]) -> None:
    def cell(value: Any) -> Any:
        if value is None:
            return ""
        if isinstance(value, float):
            return f"{value:.6f}".rstrip("0").rstrip(".")
        return value

    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({name: cell(row.get(name)) for name in fieldnames})


def export_dataset(output_dir: Path = DEFAULT_OUTPUT) -> None:
    if output_dir.exists():
        shutil.rmtree(output_dir)
    (output_dir / "raw_reports").mkdir(parents=True, exist_ok=True)

    manifest_rows: list[dict[str, Any]] = []
    metrics_rows: list[dict[str, Any]] = []
    missing: list[str] = []

    for run in RUNS:
        found = find_report(run.run_id)
        if found is None:
            missing.append(run.run_id)
            continue
        source_label, source = found

        destination_dir = output_dir / "raw_reports" / run.subset
        destination_dir.mkdir(parents=True, exist_ok=True)
        destination = destination_dir / source.name
        shutil.copy2(source, destination)

        report = load_json(source)
        metrics = extract_metrics(report)
        relative_destination = destination.relative_to(output_dir).as_posix()

        manifest_rows.append(
            {
                "run_id": run.run_id,
                "date": date_from_run_id(run.run_id),
                "subset": run.subset,
                "platform": run.platform,
                "mode": run.mode,
                "environment": run.environment,
                "validation_role": run.validation_role,
                "status": metrics.get("status"),
                "raw_report": relative_destination,
                "source_archive": source_label,
                "source_filename": source.name,
                "sha256": sha256_file(destination),
                "notes": run.notes,
            }
        )

        metrics_rows.append({"run_id": run.run_id, "subset": run.subset, **metrics})

    write_csv(
        output_dir / "manifest.csv",
        manifest_rows,
        [
            "run_id",
            "date",
            "subset",
            "platform",
            "mode",
            "environment",
            "validation_role",
            "status",
            "raw_report",
            "source_archive",
            "source_filename",
            "sha256",
            "notes",
        ],
    )
    write_csv(
        output_dir / "metrics_summary.csv",
        metrics_rows,
        [
            "run_id",
            "subset",
            "status",
            "duration_s",
            "steps",
            "sample_count",
            "passed_gates",
            "switch_events",
            "structured_to_gate_switches",
            "gate_to_structured_switches",
            "gate_time_s",
            "estimated_path_length_m",
            "encoder_distance_m",
            "min_clearance_m",
            "max_cross_track_m",
            "heading_error_p95_deg",
            "max_heading_error_deg",
            "final_goal_distance_m",
            "planner_compute_p95_ms",
        ],
    )

    archive_summary = summarize_archive()
    archive_summary["included_reports"] = len(manifest_rows)
    archive_summary["missing_requested_reports"] = missing
    (output_dir / "archive_summary.json").write_text(
        json.dumps(archive_summary, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    (output_dir / "README.md").write_text(README_TEXT, encoding="utf-8")

    if missing:
        print("Missing reports:")
        for run_id in missing:
            print(f"  - {run_id}")
    print(f"Exported {len(manifest_rows)} reports to {output_dir}")


def main() -> None:
    export_dataset()


if __name__ == "__main__":
    main()
