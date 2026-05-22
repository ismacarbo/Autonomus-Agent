from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from data_analisys.svg_charts import write_grouped_bar_chart, write_time_series_chart, write_timeline_chart  # noqa: E402


DEFAULT_OUTPUT_DIR = Path("data_analisys/outputs/multilevel_validation_20260516")
DEFAULT_REPORTS: dict[str, dict[str, list[Path]]] = {
    "structured": {
        "ideal": [
            Path("reports/thesis_planner_structured_ideal_validation_road_ideal_headless_20260516_112936_522.json"),
        ],
        "baseline": [
            Path("reports/thesis_planner_structured_validation_road_headless_20260515_084019_344.json"),
        ],
        "hardware": [
            Path("reports/thesis_hardware_structured_validation_road_gui_manual_20260424_005850_634.json"),
        ],
    },
    "unstructured": {
        "ideal": [
            Path("reports/thesis_planner_unstructured_ideal_hardware_lab_lidar_dynamic_ideal_headless_20260516_112926_925.json"),
        ],
        "baseline": [
            Path("reports/thesis_planner_unstructured_hardware_lab_lidar_dynamic_headless_20260515_083542_221.json"),
        ],
        "hardware": [
            Path("reports/thesis_hardware_unstructured_manual_gate_editor_gui_manual_20260513_045435_200.json"),
        ],
    },
    "mixed": {
        "ideal": [
            Path("reports/thesis_planner_mixed_mixed_ideal_hardware_aligned_lidar_dynamic_ideal_headless_20260516_112925_029.json"),
        ],
        "baseline": [
            Path("reports/thesis_planner_mixed_mixed_hardware_aligned_lidar_dynamic_headless_20260515_083538_038.json"),
        ],
        "hardware": [
            Path("reports/thesis_hardware_mixed_mixed_hardware_road_gate_gui_manual_20260522_034044_274.json"),
            Path("reports/thesis_hardware_mixed_mixed_hardware_road_gate_gui_manual_20260522_034454_542.json"),
        ],
    },
}


@dataclass(frozen=True)
class RunMetrics:
    mode: str
    level: str
    run_id: str
    source: str
    status: str
    samples: int
    duration_s: float | None
    body_width_m: float | None
    road_length_m: float | None
    path_length_m: float | None
    path_over_reference: float | None
    final_goal_distance_m: float | None
    progress_norm: float | None
    tracking_p95_m: float | None
    tracking_p95_body: float | None
    heading_p95_deg: float | None
    yaw_rate_p95_rad_s: float | None
    speed_mean_mps: float | None
    path_speed_mps: float | None
    time_per_meter_s_m: float | None
    target_distance_min_m: float | None
    target_distance_mean_m: float | None
    gate_completion: float | None
    gate_active_ratio: float | None
    candidate_ratio: float | None
    chosen_gate_ratio: float | None
    candidate_without_choice_ratio: float | None
    gate_windows: int
    passed_gates: int | None
    mixed_switches: int | None
    mixed_aborts: int | None
    gate_score_max: float | None
    gate_score_mean: float | None
    gate_confidence_max: float | None
    structured_score_min: float | None
    planning_p95_ms: float | None
    step_p95_ms: float | None
    safety_stop: bool
    min_front_lidar_m: float | None
    min_lidar_m: float | None
    success_score: float


@dataclass(frozen=True)
class PreparedRun:
    path: Path
    data: dict[str, Any]
    history: list[dict[str, Any]]
    road: list[tuple[float, float]]
    body_width: float | None
    metrics: RunMetrics
    normalized_time: list[float]
    tracking_norm: list[float]
    progress_series: list[float]
    gate_windows: list[tuple[float, float]]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Generate thesis multilevel validation tables and SVG plots.",
    )
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument(
        "--report",
        nargs=3,
        metavar=("MODE", "LEVEL", "PATH"),
        action="append",
        default=[],
        help="Add/override a report entry, e.g. --report mixed hardware reports/run.json",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    report_map = _report_map_from_args(args.report)
    paths = [path for levels in report_map.values() for items in levels.values() for path in items]
    missing = [path for path in paths if not path.exists()]
    if missing:
        for path in missing:
            print(f"Missing report: {path}", file=sys.stderr)
        return 2

    runs = [
        prepare_run(mode, level, path)
        for mode, levels in report_map.items()
        for level, level_paths in levels.items()
        for path in level_paths
    ]
    args.output_dir.mkdir(parents=True, exist_ok=True)
    write_metrics(args.output_dir, runs)
    write_figures(args.output_dir, runs)
    write_markdown(args.output_dir / "multilevel_validation_summary.md", runs, paths)
    write_manifest(args.output_dir / "manifest.txt", paths)
    print(f"Generated multilevel validation outputs in {args.output_dir}")
    return 0


def _report_map_from_args(entries: list[list[str]]) -> dict[str, dict[str, list[Path]]]:
    report_map = {
        mode: {level: list(paths) for level, paths in levels.items()}
        for mode, levels in DEFAULT_REPORTS.items()
    }
    for mode, level, raw_path in entries:
        mode_key = mode.strip().lower()
        level_key = level.strip().lower()
        if mode_key not in report_map:
            report_map[mode_key] = {}
        if level_key not in report_map[mode_key]:
            report_map[mode_key][level_key] = []
        report_map[mode_key][level_key].append(Path(raw_path))
    return report_map


def prepare_run(mode: str, level: str, path: Path) -> PreparedRun:
    data = json.loads(path.read_text(encoding="utf-8"))
    source = "simulation" if "telemetry" in data else "hardware"
    history = list(data.get("telemetry") or data.get("history") or [])
    road = _road_points(data, source)
    body_width = _body_width(data, source)
    run_id = _run_id(path)
    times = [_sample_time(sample) for sample in history]
    times = [time for time in times if time is not None]
    t0 = min(times) if times else 0.0
    history_duration = max(times) - t0 if times else None
    summary_duration = _summary_duration(data)
    duration = summary_duration if summary_duration is not None else history_duration

    positions = [_position(sample, source) for sample in history]
    path_points = [point for point in positions if point is not None]
    path_length = _path_length(path_points)
    reference_length = _reference_length(data, source, road, mode)
    tracking_values = [_abs_metric(sample, "tracker_cross_track") for sample in history]
    tracking_values = [value for value in tracking_values if value is not None]
    if _use_road_deviation_fallback(mode, source, tracking_values, road):
        tracking_values = [
            value
            for value in (_distance_to_polyline(point, road) for point in path_points)
            if value is not None
        ]
    tracking_p95 = _percentile(tracking_values, 95.0)
    heading_values = [_abs_metric(sample, "tracker_heading_error_deg") for sample in history]
    heading_values = [value for value in heading_values if value is not None]
    yaw_rates = [_abs_metric(sample, "yaw_rate") for sample in history]
    yaw_rates = [value for value in yaw_rates if value is not None]
    speeds = [_abs_metric(sample, "speed") for sample in history]
    speeds = [value for value in speeds if value is not None]
    target_distances = [_positive_metric(sample, "chosen_gate_distance") for sample in history]
    gate_scores = [_positive_metric(sample, "mixed_gate_score") for sample in history]
    gate_confidences = [_positive_metric(sample, "mixed_gate_confidence") for sample in history]
    structured_scores = [_positive_metric(sample, "mixed_structured_score") for sample in history]
    planning_ms = [_positive_metric(sample, "planning_ms") for sample in history]
    step_ms = [_positive_metric(sample, "step_ms") for sample in history]
    front_lidar = [_positive_metric(sample, "front_lidar") for sample in history]
    min_lidar = [_positive_metric(sample, "min_lidar") for sample in history]
    candidate_values = [_number(sample.get("candidate_gates")) or 0.0 for sample in history]
    chosen_values = [_number(sample.get("chosen_gate_index")) for sample in history]
    mixed_switches = _max_int(history, "mixed_switches")
    mixed_aborts = _max_int(history, "mixed_aborts")
    passed_gates = _max_int(history, "passed_gates")
    expected_gates = _expected_gates(data, mode, level, passed_gates)
    gate_windows = _gate_windows(history)
    status = _status(data)
    safety_stop = _safety_stop_significant(history)
    progress_series = [
        _progress_norm(sample, point, road, reference_length, mode, passed_gates, expected_gates)
        for sample, point in zip(history, positions)
    ]
    progress_series = [value for value in progress_series if value is not None]
    normalized_time = [
        _safe_div((_sample_time(sample) or t0) - t0, history_duration or duration, 0.0)
        for sample in history
    ]
    tracking_norm = [
        _safe_div(abs(_number(sample.get("tracker_cross_track")) or 0.0), body_width, 0.0)
        for sample in history
    ]
    metrics = RunMetrics(
        mode=mode,
        level=level,
        run_id=run_id,
        source=source,
        status=status,
        samples=len(history),
        duration_s=duration,
        body_width_m=body_width,
        road_length_m=reference_length,
        path_length_m=path_length,
        path_over_reference=_safe_div(path_length, reference_length),
        final_goal_distance_m=_final_goal_distance(data, history),
        progress_norm=progress_series[-1] if progress_series else None,
        tracking_p95_m=tracking_p95,
        tracking_p95_body=_safe_div(tracking_p95, body_width),
        heading_p95_deg=_percentile(heading_values, 95.0),
        yaw_rate_p95_rad_s=_percentile(yaw_rates, 95.0),
        speed_mean_mps=_mean(speeds),
        path_speed_mps=_safe_div(path_length, duration),
        time_per_meter_s_m=_safe_div(duration, path_length),
        target_distance_min_m=_min_valid(target_distances),
        target_distance_mean_m=_mean(target_distances),
        gate_completion=_gate_completion(mode, status, passed_gates, expected_gates, gate_windows),
        gate_active_ratio=_safe_div(sum(end - start for start, end in gate_windows), duration),
        candidate_ratio=_safe_div(sum(1 for value in candidate_values if value > 0.0), len(history)),
        chosen_gate_ratio=_safe_div(sum(1 for value in chosen_values if value is not None and value >= 0.0), len(history)),
        candidate_without_choice_ratio=_candidate_without_choice_ratio(candidate_values, chosen_values),
        gate_windows=len(gate_windows),
        passed_gates=passed_gates,
        mixed_switches=mixed_switches,
        mixed_aborts=mixed_aborts,
        gate_score_max=_max_valid(gate_scores),
        gate_score_mean=_mean(gate_scores),
        gate_confidence_max=_max_valid(gate_confidences),
        structured_score_min=_min_valid(structured_scores),
        planning_p95_ms=_percentile(planning_ms, 95.0),
        step_p95_ms=_percentile(step_ms, 95.0),
        safety_stop=safety_stop,
        min_front_lidar_m=_min_valid(front_lidar),
        min_lidar_m=_min_valid(min_lidar),
        success_score=_success_score(status, safety_stop, data),
    )
    return PreparedRun(
        path=path,
        data=data,
        history=history,
        road=road,
        body_width=body_width,
        metrics=metrics,
        normalized_time=normalized_time,
        tracking_norm=tracking_norm,
        progress_series=progress_series,
        gate_windows=gate_windows,
    )


def write_metrics(output_dir: Path, runs: list[PreparedRun]) -> None:
    rows = [asdict(run.metrics) for run in runs]
    with (output_dir / "multilevel_metrics.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    (output_dir / "multilevel_metrics.json").write_text(json.dumps(rows, indent=2), encoding="utf-8")


def write_figures(output_dir: Path, runs: list[PreparedRun]) -> None:
    modes = ["structured", "unstructured", "mixed"]
    levels = ["ideal", "baseline", "hardware"]
    level_colors = {"ideal": "#2ca02c", "baseline": "#1f77b4", "hardware": "#d62728"}

    def by_level(metric: str) -> list[dict[str, object]]:
        groups = []
        for level in levels:
            values = []
            for mode in modes:
                selected = [getattr(run.metrics, metric) for run in runs if run.metrics.mode == mode and run.metrics.level == level]
                values.append(_mean([value for value in selected if value is not None]))
            groups.append({"label": level, "values": values, "color": level_colors[level]})
        return groups

    write_grouped_bar_chart(
        output_dir / "success_matrix.svg",
        title="Validation success score by mode and level",
        categories=modes,
        groups=by_level("success_score"),
        y_label="success score [0-1]",
    )
    write_grouped_bar_chart(
        output_dir / "trajectory_error_p95_m.svg",
        title="Trajectory tracking error p95",
        categories=modes,
        groups=by_level("tracking_p95_m"),
        y_label="p95 tracking error [m]",
    )
    write_grouped_bar_chart(
        output_dir / "tracking_p95_body.svg",
        title="Tracking error p95 normalized by robot width",
        categories=modes,
        groups=by_level("tracking_p95_body"),
        y_label="p95 tracking / body width",
    )
    write_grouped_bar_chart(
        output_dir / "gate_completion.svg",
        title="Gate completion by mode and level",
        categories=modes,
        groups=by_level("gate_completion"),
        y_label="passed gates / expected gates",
    )
    write_grouped_bar_chart(
        output_dir / "path_over_reference.svg",
        title="Path length normalized by reference",
        categories=modes,
        groups=by_level("path_over_reference"),
        y_label="path length / reference length",
    )
    write_grouped_bar_chart(
        output_dir / "duration.svg",
        title="Mission duration by mode and level",
        categories=modes,
        groups=by_level("duration_s"),
        y_label="duration [s]",
    )
    write_grouped_bar_chart(
        output_dir / "time_per_meter.svg",
        title="Execution time normalized by trajectory length",
        categories=modes,
        groups=by_level("time_per_meter_s_m"),
        y_label="time / path length [s/m]",
    )
    write_grouped_bar_chart(
        output_dir / "path_speed.svg",
        title="Mean path execution speed",
        categories=modes,
        groups=by_level("path_speed_mps"),
        y_label="path length / duration [m/s]",
    )
    write_grouped_bar_chart(
        output_dir / "planner_compute_p95.svg",
        title="Planner compute time p95",
        categories=modes,
        groups=by_level("planning_p95_ms"),
        y_label="planning p95 [ms]",
    )
    write_grouped_bar_chart(
        output_dir / "gate_target_min_distance.svg",
        title="Minimum distance to selected gate target",
        categories=modes,
        groups=by_level("target_distance_min_m"),
        y_label="min chosen gate distance [m]",
    )
    write_grouped_bar_chart(
        output_dir / "candidate_without_choice_ratio.svg",
        title="Candidate frames without selected gate",
        categories=modes,
        groups=by_level("candidate_without_choice_ratio"),
        y_label="candidate without choice ratio",
    )
    write_grouped_bar_chart(
        output_dir / "mixed_switches.svg",
        title="Road/gate switches",
        categories=modes,
        groups=by_level("mixed_switches"),
        y_label="switch count",
    )
    write_grouped_bar_chart(
        output_dir / "gate_score_max.svg",
        title="Winning gate salience upper bound",
        categories=modes,
        groups=by_level("gate_score_max"),
        y_label="max gate score",
    )

    for mode in modes:
        selected = [run for run in runs if run.metrics.mode == mode]
        labels = _plot_labels(selected)
        write_time_series_chart(
            output_dir / f"{mode}_tracking_normalized.svg",
            title=f"{mode.capitalize()} tracking normalized by robot width",
            x_label="normalized mission time",
            y_label="tracking / body width",
            series=[
                {
                    "label": labels[index],
                    "points": list(zip(run.normalized_time, run.tracking_norm)),
                }
                for index, run in enumerate(selected)
            ],
        )
        write_timeline_chart(
            output_dir / f"{mode}_gate_windows.svg",
            title=f"{mode.capitalize()} gate/candidate windows",
            rows=[
                {
                    "label": labels[index],
                    "duration_s": run.metrics.duration_s or 0.0,
                    "windows": run.gate_windows,
                    "markers": [],
                }
                for index, run in enumerate(selected)
            ],
        )


def write_markdown(path: Path, runs: list[PreparedRun], report_paths: list[Path]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        "# Pipeline di validazione multilivello per analisi sim-to-real",
        "",
        "Dataset rigenerato con tre livelli: L0 simulazione ideale su mappe confrontabili, L1 simulazione baseline/realistica e L2 hardware reale.",
        "",
        "## Livelli",
        "",
        "| livello | nome | cosa valida | domanda principale |",
        "| --- | --- | --- | --- |",
        "| L0 | Ideal simulation | planner puro, salience, scelta road/gate, target selection, smoothness e compute time | Il planner completa task sensati in condizioni perfette? |",
        "| L1 | Realistic/baseline simulation | planner piu sensori/modello robot hardware-like | Gli errori modellati spiegano il comportamento osservato? |",
        "| L2 | Hardware | sistema completo robot, terreno, attrito, vibrazioni, rumore e setup fisico | Il sistema resta robusto quando gli errori agiscono insieme? |",
        "",
        "## Reports",
        "",
    ]
    for report_path in report_paths:
        lines.append(f"- `{report_path}`")
    lines.extend(["", "## Metriche principali", ""])
    headers = [
        "mode",
        "level",
        "status",
        "duration_s",
        "progress",
        "tracking_p95_m",
        "heading_p95_deg",
        "yaw_rate_p95",
        "planning_p95_ms",
        "gate_completion",
        "gate_windows",
        "mixed_switches",
        "gate_score_max",
        "candidate_no_choice",
        "safety",
    ]
    lines.append("| " + " | ".join(headers) + " |")
    lines.append("|" + "|".join([" --- " for _ in headers]) + "|")
    for run in runs:
        m = run.metrics
        lines.append(
            "| "
            + " | ".join(
                [
                    m.mode,
                    m.level,
                    m.status,
                    _fmt(m.duration_s),
                    _fmt(m.progress_norm),
                    _fmt(m.tracking_p95_m),
                    _fmt(m.heading_p95_deg),
                    _fmt(m.yaw_rate_p95_rad_s),
                    _fmt(m.planning_p95_ms),
                    _fmt(m.gate_completion),
                    str(m.gate_windows),
                    _fmt(m.mixed_switches),
                    _fmt(m.gate_score_max),
                    _fmt(m.candidate_without_choice_ratio),
                    "yes" if m.safety_stop else "no",
                ]
            )
            + " |"
        )
    comparability_lines = _comparability_lines(runs)
    if comparability_lines:
        lines.extend(["", "## Note di comparabilita", ""])
        lines.extend(comparability_lines)
    lines.extend(["", "## Metriche geometriche e safety", ""])
    geometry_headers = [
        "mode",
        "level",
        "path/reference",
        "goal_dist_m",
        "min_gate_dist_m",
        "gate_active_ratio",
        "candidate_ratio",
        "min_front_lidar_m",
        "min_lidar_m",
        "step_p95_ms",
        "time_per_meter",
        "path_speed",
    ]
    lines.append("| " + " | ".join(geometry_headers) + " |")
    lines.append("|" + "|".join([" --- " for _ in geometry_headers]) + "|")
    for run in runs:
        m = run.metrics
        lines.append(
            "| "
            + " | ".join(
                [
                    m.mode,
                    m.level,
                    _fmt(m.path_over_reference),
                    _fmt(m.final_goal_distance_m),
                    _fmt(m.target_distance_min_m),
                    _fmt(m.gate_active_ratio),
                    _fmt(m.candidate_ratio),
                    _fmt(m.min_front_lidar_m),
                    _fmt(m.min_lidar_m),
                    _fmt(m.step_p95_ms),
                    _fmt(m.time_per_meter_s_m),
                    _fmt(m.path_speed_mps),
                ]
            )
            + " |"
        )
    variance_lines = _variance_lines(runs)
    if variance_lines:
        lines.extend(["", "## Varianza run-to-run", ""])
        lines.extend(variance_lines)
    lines.extend(
        [
            "",
            "## Interpretation",
            "",
        "- L0 e' l'upper bound algoritmico sullo stesso task di L1: sensori ideali, niente spike, niente drift e nessuna variabilita run-to-run attesa.",
            "- L1 e' il ponte sim-to-real: mantiene la pipeline simulata ma usa mappe, sensori e comportamento hardware-like per avvicinarsi al robot.",
            "- L2 misura il sistema completo: differenze residue sono attribuibili a terreno, attrito, micro-slittamenti, vibrazioni, setup ostacoli, LiDAR parziale e ritardi non modellati.",
            "- Nella mixed, `mixed_switches`, `gate_score_max`, `candidate_without_choice_ratio` e finestre gate leggono la parte road/gate oltre al solo `passed_gates`.",
            "",
            "Generated figures:",
            "",
            "- `success_matrix.svg`",
            "- `trajectory_error_p95_m.svg`",
            "- `tracking_p95_body.svg`",
            "- `gate_completion.svg`",
            "- `gate_target_min_distance.svg`",
            "- `candidate_without_choice_ratio.svg`",
            "- `mixed_switches.svg`",
            "- `gate_score_max.svg`",
            "- `planner_compute_p95.svg`",
            "- `path_over_reference.svg`",
            "- `duration.svg`",
            "- `time_per_meter.svg`",
            "- `path_speed.svg`",
            "- per-mode `*_tracking_normalized.svg` and `*_gate_windows.svg`",
            "",
        ]
    )
    path.write_text("\n".join(lines), encoding="utf-8")


def write_manifest(path: Path, report_paths: list[Path]) -> None:
    path.write_text("\n".join(str(report_path) for report_path in report_paths) + "\n", encoding="utf-8")


def _variance_lines(runs: list[PreparedRun]) -> list[str]:
    grouped: dict[tuple[str, str], list[RunMetrics]] = {}
    for run in runs:
        grouped.setdefault((run.metrics.mode, run.metrics.level), []).append(run.metrics)
    lines: list[str] = []
    for (mode, level), metrics in sorted(grouped.items()):
        if len(metrics) < 2:
            continue
        durations = [m.duration_s for m in metrics if m.duration_s is not None]
        tracking = [m.tracking_p95_m for m in metrics if m.tracking_p95_m is not None]
        progress = [m.progress_norm for m in metrics if m.progress_norm is not None]
        lines.append(
            "- "
            + f"`{mode}/{level}`: n={len(metrics)}, "
            + f"durata mean/range={_fmt(_mean(durations))}/{_fmt(_range(durations))} s, "
            + f"tracking p95 mean/range={_fmt(_mean(tracking))}/{_fmt(_range(tracking))} m, "
            + f"progress mean/range={_fmt(_mean(progress))}/{_fmt(_range(progress))}."
        )
    return lines


def _comparability_lines(runs: list[PreparedRun]) -> list[str]:
    lines: list[str] = []
    by_mode: dict[str, list[RunMetrics]] = {}
    for run in runs:
        by_mode.setdefault(run.metrics.mode, []).append(run.metrics)
    for mode, metrics in sorted(by_mode.items()):
        by_level = {m.level: m for m in metrics}
        ideal = by_level.get("ideal")
        baseline = by_level.get("baseline")
        if ideal is None or baseline is None:
            continue
        road_ratio = _max_min_ratio(
            [v for v in [ideal.road_length_m, baseline.road_length_m] if v is not None]
        )
        body_ratio = _max_min_ratio(
            [v for v in [ideal.body_width_m, baseline.body_width_m] if v is not None]
        )
        if (road_ratio is not None and road_ratio > 1.08) or (body_ratio is not None and body_ratio > 1.08):
            lines.append(
                "- "
                + f"`{mode}`: L0 e L1 usano una geometria/scala ancora non perfettamente allineata "
                + f"(road length ratio={_fmt(road_ratio)}, body width ratio={_fmt(body_ratio)}). "
                + "Usare le metriche normalizzate quando si confrontano i livelli."
            )
    return lines


def _use_road_deviation_fallback(
    mode: str,
    source: str,
    tracking_values: list[float],
    road: list[tuple[float, float]],
) -> bool:
    if mode != "mixed" or source != "hardware" or not road:
        return False
    return True


def _plot_labels(runs: list[PreparedRun]) -> list[str]:
    totals: dict[str, int] = {}
    seen: dict[str, int] = {}
    for run in runs:
        totals[run.metrics.level] = totals.get(run.metrics.level, 0) + 1
    labels: list[str] = []
    for run in runs:
        level = run.metrics.level
        seen[level] = seen.get(level, 0) + 1
        base = {
            "ideal": "L0 ideal",
            "baseline": "L1 baseline",
            "hardware": "L2 hardware",
        }.get(level, level)
        if totals.get(level, 0) > 1:
            base = f"{base} {seen[level]}"
        labels.append(base)
    return labels


def _max_min_ratio(values: list[float | None]) -> float | None:
    valid = [value for value in values if value is not None and value > 1e-9]
    if len(valid) < 2:
        return None
    return max(valid) / min(valid)


def _road_points(data: dict[str, Any], source: str) -> list[tuple[float, float]]:
    world = data.get("scenario", {}) if source == "simulation" else data.get("scene", {}).get("world", {})
    return [(_number(point.get("x")) or 0.0, _number(point.get("y")) or 0.0) for point in world.get("road_centerline", [])]


def _body_width(data: dict[str, Any], source: str) -> float | None:
    if source == "simulation":
        return _number(data.get("config", {}).get("vehicle_geometry", {}).get("body_width"))
    return _number(data.get("scene", {}).get("geometry", {}).get("body_width"))


def _position(sample: dict[str, Any], source: str) -> tuple[float, float] | None:
    x = _number(sample.get("x"))
    y = _number(sample.get("y"))
    if x is None:
        x = _number(sample.get("position_x"))
    if y is None:
        y = _number(sample.get("position_y"))
    if x is None or y is None:
        return None
    return (x, y)


def _status(data: dict[str, Any]) -> str:
    if data.get("status"):
        return str(data["status"])
    frame = data.get("frame", {})
    if frame.get("goal_reached"):
        return "goal_reached"
    if frame.get("safety_stop_active"):
        return "safety_stop"
    return "unknown"


def _run_id(path: Path) -> str:
    name = path.stem
    for prefix in ("thesis_planner_", "thesis_hardware_"):
        if name.startswith(prefix):
            return name[len(prefix):]
    return name


def _sample_time(sample: dict[str, Any]) -> float | None:
    return _number(sample.get("time"))


def _reference_length(data: dict[str, Any], source: str, road: list[tuple[float, float]], mode: str) -> float | None:
    if road:
        length = _path_length(road)
        if length > 1e-6:
            return length
    world = data.get("scenario", {}) if source == "simulation" else data.get("scene", {}).get("world", {})
    start = world.get("start", {})
    goal = world.get("goal", {})
    sx = _number(start.get("x"))
    sy = _number(start.get("y"))
    gx = _number(goal.get("x"))
    gy = _number(goal.get("y"))
    if None not in (sx, sy, gx, gy):
        direct = math.hypot(float(gx) - float(sx), float(gy) - float(sy))
        if direct > 1e-6:
            return direct
    return None if mode == "structured" else _path_length(_gate_route(world))


def _gate_route(world: dict[str, Any]) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    start = world.get("start", {})
    sx = _number(start.get("x"))
    sy = _number(start.get("y"))
    if sx is not None and sy is not None:
        points.append((sx, sy))
    for gate in world.get("gates", []):
        if gate.get("final"):
            continue
        pos = gate.get("position", {})
        x = _number(pos.get("x"))
        y = _number(pos.get("y"))
        if x is not None and y is not None:
            points.append((x, y))
    goal = world.get("goal", {})
    gx = _number(goal.get("x"))
    gy = _number(goal.get("y"))
    if gx is not None and gy is not None:
        points.append((gx, gy))
    return points


def _expected_gates(data: dict[str, Any], mode: str, level: str, passed_gates: int | None) -> int | None:
    if mode == "structured":
        return None
    source = "simulation" if "telemetry" in data else "hardware"
    world = data.get("scenario", {}) if source == "simulation" else data.get("scene", {}).get("world", {})
    non_final = sum(1 for gate in world.get("gates", []) if not gate.get("final"))
    if non_final > 0:
        return max(non_final, passed_gates or 0)
    if mode == "mixed":
        return max(2, passed_gates or 0) if level != "hardware" else None
    return max(2, passed_gates or 0)


def _gate_completion(
    mode: str,
    status: str,
    passed_gates: int | None,
    expected_gates: int | None,
    gate_windows: list[tuple[float, float]],
) -> float | None:
    if mode == "structured":
        return None
    direct_completion = _safe_div(passed_gates, expected_gates)
    if direct_completion is not None and direct_completion > 0.0:
        return min(1.0, direct_completion)
    if mode == "mixed" and status == "goal_reached" and gate_windows:
        return 1.0
    return direct_completion


def _progress_norm(
    sample: dict[str, Any],
    point: tuple[float, float] | None,
    road: list[tuple[float, float]],
    reference_length: float | None,
    mode: str,
    passed_gates: int | None,
    expected_gates: int | None,
) -> float | None:
    progress_m = _number(sample.get("structured_progress_s"))
    if mode != "unstructured" and progress_m is not None and reference_length and reference_length > 1e-6:
        return max(0.0, min(1.0, progress_m / reference_length))
    if point is not None and road and reference_length and reference_length > 1e-6:
        return max(0.0, min(1.0, _project_length_on_polyline(point, road) / reference_length))
    if mode != "structured":
        current_passed = _number(sample.get("passed_gates"))
        if current_passed is not None and expected_gates:
            return max(0.0, min(1.0, current_passed / expected_gates))
        if passed_gates is not None and expected_gates:
            return max(0.0, min(1.0, passed_gates / expected_gates))
    return None


def _final_goal_distance(data: dict[str, Any], history: list[dict[str, Any]]) -> float | None:
    summary_distance = _number(data.get("summary", {}).get("distance_to_goal"))
    if summary_distance is not None:
        return summary_distance
    for sample in reversed(history):
        value = _number(sample.get("distance_to_goal"))
        if value is not None:
            return value
    return _number(data.get("frame", {}).get("distance_to_goal"))


def _summary_duration(data: dict[str, Any]) -> float | None:
    summary = data.get("summary", {})
    for key in ("sim_time", "duration_s", "runtime_s"):
        value = _number(summary.get(key))
        if value is not None and value > 0.0:
            return value
    performance = data.get("performance", {})
    value = _number(performance.get("duration_s"))
    if value is not None and value > 0.0:
        return value
    return None


def _gate_windows(history: list[dict[str, Any]]) -> list[tuple[float, float]]:
    windows: list[tuple[float, float]] = []
    active_start: float | None = None
    last_time: float | None = None
    for sample in history:
        time = _sample_time(sample)
        if time is None:
            continue
        chosen = (_number(sample.get("chosen_gate_index")) or -1.0) >= 0.0
        candidate = (_number(sample.get("candidate_gates")) or 0.0) > 0.0
        mixed = (_number(sample.get("mixed_mode")) or 0.0) > 0.0
        active = chosen or mixed or candidate
        if active and active_start is None:
            active_start = time
        if not active and active_start is not None:
            windows.append((active_start, last_time if last_time is not None else time))
            active_start = None
        last_time = time
    if active_start is not None and last_time is not None:
        windows.append((active_start, last_time))
    return [(start, end) for start, end in windows if end >= start]


def _path_length(points: list[tuple[float, float]]) -> float | None:
    if len(points) < 2:
        return None
    return sum(math.hypot(bx - ax, by - ay) for (ax, ay), (bx, by) in zip(points, points[1:]))


def _distance_to_polyline(point: tuple[float, float], polyline: list[tuple[float, float]]) -> float | None:
    if not polyline:
        return None
    if len(polyline) == 1:
        return math.hypot(point[0] - polyline[0][0], point[1] - polyline[0][1])
    best = math.inf
    px, py = point
    for (ax, ay), (bx, by) in zip(polyline, polyline[1:]):
        vx = bx - ax
        vy = by - ay
        length2 = vx * vx + vy * vy
        t = 0.0 if length2 <= 1e-12 else max(0.0, min(1.0, ((px - ax) * vx + (py - ay) * vy) / length2))
        cx = ax + t * vx
        cy = ay + t * vy
        best = min(best, math.hypot(px - cx, py - cy))
    return best if math.isfinite(best) else None


def _project_length_on_polyline(point: tuple[float, float], polyline: list[tuple[float, float]]) -> float:
    if len(polyline) < 2:
        return 0.0
    best_distance = math.inf
    best_s = 0.0
    cumulative = 0.0
    px, py = point
    for (ax, ay), (bx, by) in zip(polyline, polyline[1:]):
        vx = bx - ax
        vy = by - ay
        length = math.hypot(vx, vy)
        length2 = length * length
        t = 0.0 if length2 <= 1e-12 else max(0.0, min(1.0, ((px - ax) * vx + (py - ay) * vy) / length2))
        cx = ax + t * vx
        cy = ay + t * vy
        distance = math.hypot(px - cx, py - cy)
        if distance < best_distance:
            best_distance = distance
            best_s = cumulative + t * length
        cumulative += length
    return best_s


def _success_score(status: str, safety_stop: bool, data: dict[str, Any]) -> float:
    if status == "goal_reached" and not safety_stop:
        return 1.0
    if status == "goal_reached":
        return 0.8
    if status in {"live", "stopped", "timeout"} and not safety_stop and not data.get("summary", {}).get("collision"):
        return 0.45
    return 0.0


def _safety_stop_significant(history: list[dict[str, Any]]) -> bool:
    active_times = [
        _sample_time(sample)
        for sample in history
        if bool(_number(sample.get("safety_stop_active")))
    ]
    active_times = [time for time in active_times if time is not None]
    if not active_times:
        return False
    # Single-frame hardware safety flags can appear during telemetry gaps.
    # Treat it as mission-level safety only if it persists for about one second
    # or if the final frame is still active.
    if (bool(_number(history[-1].get("safety_stop_active"))) if history else False):
        return True
    if len(active_times) < 8:
        return False
    return max(active_times) - min(active_times) >= 1.0


def _number(value: Any) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(number):
        return None
    return number


def _abs_metric(sample: dict[str, Any], key: str) -> float | None:
    value = _number(sample.get(key))
    return abs(value) if value is not None else None


def _positive_metric(sample: dict[str, Any], key: str) -> float | None:
    value = _number(sample.get(key))
    return value if value is not None and value >= 0.0 else None


def _min_valid(values: list[float | None]) -> float | None:
    valid = [value for value in values if value is not None]
    return min(valid) if valid else None


def _max_valid(values: list[float | None]) -> float | None:
    valid = [value for value in values if value is not None]
    return max(valid) if valid else None


def _candidate_without_choice_ratio(
    candidate_values: list[float],
    chosen_values: list[float | None],
) -> float | None:
    candidate_frames = 0
    missing_choice_frames = 0
    for candidate, chosen in zip(candidate_values, chosen_values):
        if candidate <= 0.0:
            continue
        candidate_frames += 1
        if chosen is None or chosen < 0.0:
            missing_choice_frames += 1
    return _safe_div(missing_choice_frames, candidate_frames)


def _max_int(history: list[dict[str, Any]], key: str) -> int | None:
    values = [_number(sample.get(key)) for sample in history]
    valid = [int(round(value)) for value in values if value is not None and value >= 0.0]
    return max(valid) if valid else None


def _mean(values: list[float | None]) -> float | None:
    valid = [value for value in values if value is not None]
    return sum(valid) / len(valid) if valid else None


def _range(values: list[float | None]) -> float | None:
    valid = [value for value in values if value is not None]
    return max(valid) - min(valid) if valid else None


def _percentile(values: list[float], percentile: float) -> float | None:
    if not values:
        return None
    sorted_values = sorted(values)
    index = (len(sorted_values) - 1) * percentile / 100.0
    lower = math.floor(index)
    upper = math.ceil(index)
    if lower == upper:
        return sorted_values[int(index)]
    return sorted_values[lower] * (upper - index) + sorted_values[upper] * (index - lower)


def _safe_div(numerator: float | int | None, denominator: float | int | None, default: float | None = None) -> float | None:
    if numerator is None or denominator is None:
        return default
    denominator = float(denominator)
    if abs(denominator) < 1e-12:
        return default
    return float(numerator) / denominator


def _fmt(value: float | int | None) -> str:
    if value is None:
        return "n.d."
    return f"{float(value):.3f}"


def _short_run(run_id: str) -> str:
    parts = run_id.split("_")
    if len(parts) >= 4 and parts[-2].isdigit():
        return "_".join(parts[-3:])
    return run_id[-18:]


if __name__ == "__main__":
    raise SystemExit(main())
