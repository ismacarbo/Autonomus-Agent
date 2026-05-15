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

from data_analisys.svg_charts import (  # noqa: E402
    write_grouped_bar_chart,
    write_time_series_chart,
    write_timeline_chart,
)


DEFAULT_SIMULATION = Path(
    "reports/thesis_planner_mixed_mixed_road_gate_validation_lidar_dynamic_gui_auto_20260514_163644_584.json"
)
DEFAULT_HARDWARE = [
    Path("reports/thesis_hardware_mixed_mixed_hardware_road_gate_gui_manual_20260512_052657_596.json"),
    Path("reports/thesis_hardware_mixed_mixed_hardware_road_gate_gui_manual_20260512_053236_711.json"),
    Path("reports/thesis_hardware_mixed_mixed_hardware_road_gate_gui_manual_20260512_052931_566.json"),
]
DEFAULT_OUTPUT_DIR = Path("data_analisys/outputs/mixed_model_validation_20260514")


@dataclass(frozen=True)
class MixedRunMetrics:
    run_id: str
    source: str
    status: str
    samples: int
    duration_s: float | None
    road_length_m: float | None
    path_length_m: float | None
    path_over_road: float | None
    final_progress_norm: float | None
    final_goal_distance_m: float | None
    final_road_deviation_m: float | None
    road_deviation_mean_m: float | None
    road_deviation_p95_m: float | None
    road_deviation_max_m: float | None
    road_deviation_p95_body: float | None
    gate_ratio: float | None
    gate_windows: int
    longest_gate_window_s: float | None
    candidate_ratio: float | None
    min_chosen_gate_distance_m: float | None
    reported_passed_gates: int | None
    mixed_switches: int | None
    mixed_aborts: int | None
    mixed_gate_score_max: float | None
    mixed_gate_confidence_max: float | None
    mixed_structured_score_min: float | None
    yaw_rate_p95_rad_s: float | None
    yaw_rate_max_rad_s: float | None
    speed_mean_mps: float | None
    speed_p95_mps: float | None
    front_lidar_min_m: float | None
    min_lidar_min_m: float | None


@dataclass(frozen=True)
class PreparedRun:
    path: Path
    run_id: str
    source: str
    data: dict[str, Any]
    history: list[dict[str, Any]]
    road: list[dict[str, float]]
    body_width: float
    metrics: MixedRunMetrics
    normalized_time: list[float]
    road_deviation_norm: list[float]
    progress_norm: list[float]
    gate_windows: list[tuple[float, float]]
    completion_times: list[float]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Generate normalized mixed simulation-vs-hardware metrics for thesis plots.",
    )
    parser.add_argument("--simulation", type=Path, default=DEFAULT_SIMULATION)
    parser.add_argument("--hardware", type=Path, action="append", default=[])
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    hardware_paths = args.hardware if args.hardware else DEFAULT_HARDWARE
    report_paths = [args.simulation, *hardware_paths]
    missing = [path for path in report_paths if not path.exists()]
    if missing:
        for path in missing:
            print(f"Missing report: {path}", file=sys.stderr)
        return 2

    runs = [prepare_run(path) for path in report_paths]
    args.output_dir.mkdir(parents=True, exist_ok=True)
    write_metrics(args.output_dir, runs)
    write_figures(args.output_dir, runs)
    write_markdown(args.output_dir / "mixed_model_validation_summary.md", runs, report_paths)
    write_manifest(args.output_dir / "manifest.txt", report_paths)
    print(f"Generated mixed model validation outputs in {args.output_dir}")
    return 0


def prepare_run(path: Path) -> PreparedRun:
    data = json.loads(path.read_text(encoding="utf-8"))
    source = "simulation" if "telemetry" in data else "hardware"
    history = list(data.get("telemetry") or data.get("history") or [])
    road = _road(data, source)
    body_width = _body_width(data, source)
    run_id = _run_id(path)
    road_length = _road_length(road)
    times = [_sample_time(sample) for sample in history]
    times = [time for time in times if time is not None]
    t0 = min(times) if times else 0.0
    duration = (max(times) - t0) if times else None

    deviations: list[float] = []
    progress_values: list[float] = []
    normalized_time: list[float] = []
    path_points: list[tuple[float, float]] = []
    gate_windows = _gate_windows(history, source)
    for sample in history:
        point = _position(sample, source)
        if point is None:
            continue
        path_points.append(point)
        deviation = _distance_to_polyline(point, road)
        deviations.append(deviation)
        progress_m = _sample_progress_m(sample, source, point, road)
        progress_values.append(_safe_div(progress_m, road_length, default=0.0))
        time = _sample_time(sample)
        normalized_time.append(_safe_div((time or t0) - t0, duration, default=0.0))

    speeds = [_abs_value(sample, "speed") for sample in history]
    yaw_rates = [_abs_value(sample, "yaw_rate") for sample in history]
    candidate_active = [_float(sample.get("candidate_gates")) or 0.0 for sample in history]
    chosen_distances = [
        value
        for value in (_float(sample.get("chosen_gate_distance")) for sample in history)
        if value is not None and value >= 0.0
    ]
    road_deviation_norm = [value / body_width for value in deviations] if body_width > 0 else deviations
    completion_times = _completion_times(history)
    path_length = _path_length(path_points)

    metrics = MixedRunMetrics(
        run_id=run_id,
        source=source,
        status=str(data.get("status") or ("goal_reached" if data.get("frame", {}).get("goal_reached") else "unknown")),
        samples=len(history),
        duration_s=duration,
        road_length_m=road_length,
        path_length_m=path_length,
        path_over_road=_safe_div(path_length, road_length),
        final_progress_norm=progress_values[-1] if progress_values else None,
        final_goal_distance_m=_final_goal_distance(data, history, source),
        final_road_deviation_m=deviations[-1] if deviations else None,
        road_deviation_mean_m=_mean(deviations),
        road_deviation_p95_m=_percentile(deviations, 95.0),
        road_deviation_max_m=max(deviations) if deviations else None,
        road_deviation_p95_body=_safe_div(_percentile(deviations, 95.0), body_width),
        gate_ratio=_safe_div(sum(end - start for start, end in gate_windows), duration),
        gate_windows=len(gate_windows),
        longest_gate_window_s=max((end - start for start, end in gate_windows), default=None),
        candidate_ratio=_safe_div(sum(1 for value in candidate_active if value > 0.0), len(history)),
        min_chosen_gate_distance_m=min(chosen_distances) if chosen_distances else None,
        reported_passed_gates=_max_int(history, "passed_gates"),
        mixed_switches=_max_int(history, "mixed_switches"),
        mixed_aborts=_max_int(history, "mixed_aborts"),
        mixed_gate_score_max=_max_metric(history, "mixed_gate_score"),
        mixed_gate_confidence_max=_max_metric(history, "mixed_gate_confidence"),
        mixed_structured_score_min=_min_metric(history, "mixed_structured_score"),
        yaw_rate_p95_rad_s=_percentile(yaw_rates, 95.0),
        yaw_rate_max_rad_s=max(yaw_rates) if yaw_rates else None,
        speed_mean_mps=_mean(speeds),
        speed_p95_mps=_percentile(speeds, 95.0),
        front_lidar_min_m=_min_metric(history, "front_lidar"),
        min_lidar_min_m=_min_metric(history, "min_lidar"),
    )
    return PreparedRun(
        path=path,
        run_id=run_id,
        source=source,
        data=data,
        history=history,
        road=road,
        body_width=body_width,
        metrics=metrics,
        normalized_time=normalized_time,
        road_deviation_norm=road_deviation_norm,
        progress_norm=progress_values,
        gate_windows=gate_windows,
        completion_times=completion_times,
    )


def write_metrics(output_dir: Path, runs: list[PreparedRun]) -> None:
    rows = [asdict(run.metrics) for run in runs]
    with (output_dir / "mixed_metrics.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    (output_dir / "mixed_metrics.json").write_text(json.dumps(rows, indent=2), encoding="utf-8")


def write_figures(output_dir: Path, runs: list[PreparedRun]) -> None:
    write_time_series_chart(
        output_dir / "road_deviation_normalized.svg",
        title="Mixed road deviation normalized by robot width",
        x_label="normalized mission time",
        y_label="road deviation / body width",
        series=[
            {
                "label": _short_label(run.run_id),
                "points": list(zip(run.normalized_time, run.road_deviation_norm)),
            }
            for run in runs
        ],
    )
    write_time_series_chart(
        output_dir / "progress_normalized.svg",
        title="Mixed structured progress",
        x_label="normalized mission time",
        y_label="progress / road length",
        series=[
            {
                "label": _short_label(run.run_id),
                "points": list(zip(run.normalized_time, run.progress_norm)),
            }
            for run in runs
        ],
    )
    write_timeline_chart(
        output_dir / "gate_windows.svg",
        title="Mixed gate windows",
        rows=[
            {
                "label": _short_label(run.run_id),
                "duration_s": run.metrics.duration_s or 0.0,
                "windows": run.gate_windows,
                "markers": [{"x": time, "color": "#d62728"} for time in run.completion_times],
            }
            for run in runs
        ],
    )
    labels = [_short_label(run.run_id) for run in runs]
    write_grouped_bar_chart(
        output_dir / "summary_normalized_metrics.svg",
        title="Mixed normalized comparison",
        categories=labels,
        y_label="normalized value",
        groups=[
            {
                "label": "progress",
                "values": [run.metrics.final_progress_norm for run in runs],
                "color": "#1f77b4",
            },
            {
                "label": "gate ratio",
                "values": [run.metrics.gate_ratio for run in runs],
                "color": "#ff7f0e",
            },
            {
                "label": "road dev p95/body",
                "values": [run.metrics.road_deviation_p95_body for run in runs],
                "color": "#2ca02c",
            },
        ],
    )


def write_markdown(path: Path, runs: list[PreparedRun], report_paths: list[Path]) -> None:
    lines = [
        "# Mixed model validation summary",
        "",
        "Report analizzati:",
        "",
        *[f"- `{item}`" for item in report_paths],
        "",
        "| Run | Tipo | Stato | Durata s | path/road | progress | gate ratio | road dev p95/body | passed gates | switches | aborts |",
        "| --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for run in runs:
        m = run.metrics
        lines.append(
            "| `{}` | {} | `{}` | {} | {} | {} | {} | {} | {} | {} | {} |".format(
                m.run_id,
                m.source,
                m.status,
                _fmt(m.duration_s),
                _fmt(m.path_over_road),
                _fmt(m.final_progress_norm),
                _fmt(m.gate_ratio),
                _fmt(m.road_deviation_p95_body),
                _fmt_int(m.reported_passed_gates),
                _fmt_int(m.mixed_switches),
                _fmt_int(m.mixed_aborts),
            )
        )
    lines.extend(
        [
            "",
            "Lettura consigliata:",
            "",
            "- usare la simulazione hardware-aligned come baseline confrontabile con i run reali;",
            "- usare la simulazione grande come spiegazione della logica score/switch;",
            "- nei report hardware vecchi, le finestre gate sono ricostruite da `candidate_gates` e `chosen_gate_distance`.",
            "",
        ]
    )
    path.write_text("\n".join(lines), encoding="utf-8")


def write_manifest(path: Path, report_paths: list[Path]) -> None:
    path.write_text("\n".join(str(item) for item in report_paths) + "\n", encoding="utf-8")


def _road(data: dict[str, Any], source: str) -> list[dict[str, float]]:
    if source == "simulation":
        return list(data.get("scenario", {}).get("road_centerline") or [])
    return list(data.get("scene", {}).get("world", {}).get("road_centerline") or [])


def _body_width(data: dict[str, Any], source: str) -> float:
    if source == "simulation":
        return _float(data.get("config", {}).get("vehicle_geometry", {}).get("body_width")) or 0.24
    return _float(data.get("scene", {}).get("geometry", {}).get("body_width")) or 0.24


def _position(sample: dict[str, Any], source: str) -> tuple[float, float] | None:
    x_key = "x" if source == "simulation" else "position_x"
    y_key = "y" if source == "simulation" else "position_y"
    x = _float(sample.get(x_key))
    y = _float(sample.get(y_key))
    return (x, y) if x is not None and y is not None else None


def _sample_time(sample: dict[str, Any]) -> float | None:
    return _float(sample.get("time"))


def _sample_progress_m(
    sample: dict[str, Any],
    source: str,
    point: tuple[float, float],
    road: list[dict[str, float]],
) -> float:
    if source == "hardware":
        progress = _float(sample.get("structured_progress_s"))
        if progress is not None:
            return progress
    return _project_progress_m(point, road)


def _final_goal_distance(data: dict[str, Any], history: list[dict[str, Any]], source: str) -> float | None:
    if source == "simulation":
        return _float(data.get("summary", {}).get("distance_to_goal"))
    frame_value = _float(data.get("frame", {}).get("distance_to_goal"))
    if frame_value is not None:
        return frame_value
    return _float(history[-1].get("distance_to_goal")) if history else None


def _gate_windows(history: list[dict[str, Any]], source: str) -> list[tuple[float, float]]:
    if source == "simulation":
        return _boolean_windows(history, lambda sample: (_float(sample.get("mixed_mode")) or 0.0) > 0.5)
    return _boolean_windows(
        history,
        lambda sample: (_float(sample.get("candidate_gates")) or 0.0) > 0.0
        and (_float(sample.get("chosen_gate_distance")) or -1.0) >= 0.0,
    )


def _boolean_windows(history: list[dict[str, Any]], predicate) -> list[tuple[float, float]]:
    windows: list[tuple[float, float]] = []
    start: float | None = None
    last_time: float | None = None
    for sample in history:
        time = _sample_time(sample)
        if time is None:
            continue
        active = bool(predicate(sample))
        if active and start is None:
            start = time
        if not active and start is not None:
            windows.append((start, last_time if last_time is not None else time))
            start = None
        last_time = time
    if start is not None and last_time is not None:
        windows.append((start, last_time))
    return windows


def _completion_times(history: list[dict[str, Any]]) -> list[float]:
    events: list[float] = []
    previous = 0
    for sample in history:
        current = int(_float(sample.get("passed_gates")) or 0.0)
        time = _sample_time(sample)
        if current > previous and time is not None:
            events.append(time)
        previous = current
    return events


def _road_length(road: list[dict[str, float]]) -> float | None:
    if len(road) < 2:
        return None
    return sum(
        math.hypot(road[i]["x"] - road[i - 1]["x"], road[i]["y"] - road[i - 1]["y"])
        for i in range(1, len(road))
    )


def _path_length(points: list[tuple[float, float]]) -> float | None:
    if len(points) < 2:
        return None
    return sum(
        math.hypot(points[i][0] - points[i - 1][0], points[i][1] - points[i - 1][1])
        for i in range(1, len(points))
    )


def _distance_to_polyline(point: tuple[float, float], road: list[dict[str, float]]) -> float:
    if len(road) < 2:
        return 0.0
    px, py = point
    return min(
        _distance_to_segment(px, py, road[i - 1]["x"], road[i - 1]["y"], road[i]["x"], road[i]["y"])
        for i in range(1, len(road))
    )


def _distance_to_segment(px: float, py: float, ax: float, ay: float, bx: float, by: float) -> float:
    vx = bx - ax
    vy = by - ay
    wx = px - ax
    wy = py - ay
    length_sq = vx * vx + vy * vy
    if length_sq <= 1e-12:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, (wx * vx + wy * vy) / length_sq))
    cx = ax + t * vx
    cy = ay + t * vy
    return math.hypot(px - cx, py - cy)


def _project_progress_m(point: tuple[float, float], road: list[dict[str, float]]) -> float:
    if len(road) < 2:
        return 0.0
    px, py = point
    best_distance = float("inf")
    best_progress = 0.0
    progress = 0.0
    for i in range(1, len(road)):
        ax = road[i - 1]["x"]
        ay = road[i - 1]["y"]
        bx = road[i]["x"]
        by = road[i]["y"]
        vx = bx - ax
        vy = by - ay
        segment_length = math.hypot(vx, vy)
        if segment_length <= 1e-12:
            continue
        t = max(0.0, min(1.0, ((px - ax) * vx + (py - ay) * vy) / (segment_length * segment_length)))
        cx = ax + t * vx
        cy = ay + t * vy
        distance = math.hypot(px - cx, py - cy)
        if distance < best_distance:
            best_distance = distance
            best_progress = progress + t * segment_length
        progress += segment_length
    return best_progress


def _float(value: object) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _abs_value(sample: dict[str, Any], key: str) -> float:
    value = _float(sample.get(key))
    return abs(value) if value is not None else 0.0


def _mean(values: list[float]) -> float | None:
    return sum(values) / len(values) if values else None


def _percentile(values: list[float], percentile: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    k = (len(ordered) - 1) * percentile / 100.0
    lo = math.floor(k)
    hi = math.ceil(k)
    if lo == hi:
        return ordered[lo]
    return ordered[lo] * (hi - k) + ordered[hi] * (k - lo)


def _safe_div(num: float | None, den: float | int | None, default: float | None = None) -> float | None:
    if num is None or den is None or abs(float(den)) <= 1e-12:
        return default
    return float(num) / float(den)


def _max_metric(history: list[dict[str, Any]], key: str) -> float | None:
    values = [_float(sample.get(key)) for sample in history]
    values = [value for value in values if value is not None]
    return max(values) if values else None


def _min_metric(history: list[dict[str, Any]], key: str) -> float | None:
    values = [_float(sample.get(key)) for sample in history]
    values = [value for value in values if value is not None and value >= 0.0]
    return min(values) if values else None


def _max_int(history: list[dict[str, Any]], key: str) -> int | None:
    value = _max_metric(history, key)
    return int(round(value)) if value is not None else None


def _run_id(path: Path) -> str:
    stem = path.stem
    parts = stem.split("_")
    return "_".join(parts[-4:]) if len(parts) >= 4 else stem


def _short_label(value: str, max_len: int = 24) -> str:
    return value if len(value) <= max_len else value[-max_len:]


def _fmt(value: float | None) -> str:
    return "n.d." if value is None else f"{value:.3f}"


def _fmt_int(value: int | None) -> str:
    return "n.d." if value is None else str(value)


if __name__ == "__main__":
    raise SystemExit(main())
