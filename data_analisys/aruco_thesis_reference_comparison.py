"""Build thesis-friendly ArUco/report comparison figures.

This utility starts from an alignment CSV produced by aruco_video_alignment.py
and adds a smooth reference trace from the ArUco robot-center pose. The trace is
intended for readable figures and diagnostics; it should not replace the raw
ArUco samples in quantitative reporting.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
import statistics
import sys
from collections import defaultdict
from pathlib import Path
from typing import Any


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--alignment-csv", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument(
        "--config",
        type=Path,
        default=Path("data_analisys/aruco_ground_truth_config_reference_centers_120x140.json"),
    )
    parser.add_argument("--smooth-window-s", type=float, default=1.80)
    parser.add_argument("--map-width-m", type=float, default=1.20)
    parser.add_argument("--map-height-m", type=float, default=1.40)
    parser.add_argument("--plot-margin-m", type=float, default=0.08)
    parser.add_argument("--report", type=Path)
    parser.add_argument("--alignment-summary", type=Path)
    parser.add_argument("--obstacles", type=Path)
    parser.add_argument("--gap-fill-step-s", type=float, default=0.10)
    parser.add_argument("--gap-fill-min-s", type=float, default=0.20)
    parser.add_argument("--max-gap-fill-s", type=float, default=8.50)
    return parser.parse_args(argv)


def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def angle_delta(a: float, b: float) -> float:
    return wrap_angle(a - b)


def lerp(a: float, b: float, alpha: float) -> float:
    return a * (1.0 - alpha) + b * alpha


def lerp_angle(a: float, b: float, alpha: float) -> float:
    return wrap_angle(a + angle_delta(b, a) * alpha)


def circular_mean(values: list[float]) -> float:
    if not values:
        return 0.0
    return math.atan2(sum(math.sin(v) for v in values), sum(math.cos(v) for v in values))


def percentile(values: list[float], pct: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    rank = (pct / 100.0) * (len(ordered) - 1)
    lower = int(math.floor(rank))
    upper = int(math.ceil(rank))
    if lower == upper:
        return ordered[lower]
    alpha = rank - lower
    return ordered[lower] * (1.0 - alpha) + ordered[upper] * alpha


def stats(values: list[float]) -> dict[str, float | int]:
    if not values:
        return {"count": 0}
    return {
        "count": len(values),
        "mean": statistics.fmean(values),
        "median": statistics.median(values),
        "p95": percentile(values, 95.0),
        "max": max(values),
    }


def read_alignment(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    required = [
        "segment",
        "video_time_s",
        "log_time_s",
        "gt_robot_center_x_m",
        "gt_robot_center_y_m",
        "gt_yaw_with_fitted_offset_rad",
        "log_real_x_m",
        "log_real_y_m",
        "log_real_yaw_rad",
        "position_error_robot_center_real_m",
        "yaw_error_with_fitted_offset_deg",
    ]
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        missing = [field for field in required if field not in (reader.fieldnames or [])]
        if missing:
            raise SystemExit(f"CSV alignment senza colonne richieste: {', '.join(missing)}")
        for raw in reader:
            try:
                row = dict(raw)
                row["segment"] = int(raw["segment"])
                for key in required[1:]:
                    row[key] = float(raw[key])
                rows.append(row)
            except (TypeError, ValueError):
                continue
    return sorted(rows, key=lambda item: (int(item["segment"]), float(item["video_time_s"])))


def add_reference_trace(rows: list[dict[str, Any]], window_s: float) -> None:
    by_segment: dict[int, list[dict[str, Any]]] = defaultdict(list)
    for row in rows:
        by_segment[int(row["segment"])].append(row)

    half_window = max(window_s, 0.0) * 0.5
    for segment_rows in by_segment.values():
        for row in segment_rows:
            t = float(row["video_time_s"])
            neighbors = [
                item
                for item in segment_rows
                if abs(float(item["video_time_s"]) - t) <= half_window
            ]
            if not neighbors:
                neighbors = [row]
            ref_x = statistics.fmean(float(item["gt_robot_center_x_m"]) for item in neighbors)
            ref_y = statistics.fmean(float(item["gt_robot_center_y_m"]) for item in neighbors)
            ref_yaw = circular_mean(
                [float(item["gt_yaw_with_fitted_offset_rad"]) for item in neighbors]
            )
            est_x = float(row["log_real_x_m"])
            est_y = float(row["log_real_y_m"])
            est_yaw = float(row["log_real_yaw_rad"])
            row["reference_x_m"] = ref_x
            row["reference_y_m"] = ref_y
            row["reference_yaw_rad"] = ref_yaw
            row["reference_position_error_m"] = math.hypot(est_x - ref_x, est_y - ref_y)
            row["reference_yaw_error_deg"] = math.degrees(abs(angle_delta(est_yaw, ref_yaw)))


def write_reference_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    fieldnames = [
        "segment",
        "frame_idx",
        "video_time_s",
        "log_time_s",
        "gt_robot_center_x_m",
        "gt_robot_center_y_m",
        "gt_yaw_with_fitted_offset_rad",
        "reference_x_m",
        "reference_y_m",
        "reference_yaw_rad",
        "log_real_x_m",
        "log_real_y_m",
        "log_real_yaw_rad",
        "position_error_robot_center_real_m",
        "yaw_error_with_fitted_offset_deg",
        "reference_position_error_m",
        "reference_yaw_error_deg",
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field, "") for field in fieldnames})


def load_report_shape(
    report_path: Path | None,
    summary_path: Path | None,
) -> list[tuple[float, float, float]] | None:
    if report_path is None or summary_path is None:
        return None
    report = json.loads(report_path.read_text(encoding="utf-8"))
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    matrix = summary.get("report_registration", {}).get("matrix")
    if not isinstance(matrix, list) or len(matrix) != 2:
        return None
    history = report.get("history") or []
    series: list[tuple[float, float, float]] = []
    for item in history:
        try:
            t = float(item["time"])
            x = float(item["position_x"])
            y = float(item["position_y"])
        except (KeyError, TypeError, ValueError):
            continue
        real_x = float(matrix[0][0]) * x + float(matrix[0][1]) * y + float(matrix[0][2])
        real_y = float(matrix[1][0]) * x + float(matrix[1][1]) * y + float(matrix[1][2])
        series.append((t, real_x, real_y))
    if len(series) < 2:
        return None
    return sorted(series)


def load_reference_points(config_path: Path) -> list[tuple[str, float, float]]:
    config = json.loads(config_path.read_text(encoding="utf-8"))
    reference_markers = config.get("reference_markers", {})
    ids = config.get("reference_box", {}).get("ids", {})
    label_by_id = {int(value): str(key) for key, value in ids.items()}
    points: list[tuple[str, float, float]] = []
    for marker_id_text, marker in reference_markers.items():
        try:
            marker_id = int(marker_id_text)
            center = marker["center"]
            x_m = float(center[0])
            y_m = float(center[1])
        except (KeyError, TypeError, ValueError, IndexError):
            continue
        label = label_by_id.get(marker_id, f"ID {marker_id}")
        points.append((label, x_m, y_m))
    return sorted(points)


def load_obstacles(path: Path | None) -> list[dict[str, Any]]:
    if path is None:
        return []
    data = json.loads(path.read_text(encoding="utf-8"))
    obstacles = data.get("obstacles", data if isinstance(data, list) else [])
    if not isinstance(obstacles, list):
        return []
    parsed: list[dict[str, Any]] = []
    for index, obstacle in enumerate(obstacles, start=1):
        if not isinstance(obstacle, dict):
            continue
        try:
            center = obstacle["center_m"]
            parsed_obstacle = dict(obstacle)
            parsed_obstacle["center_m"] = [float(center[0]), float(center[1])]
            parsed_obstacle.setdefault("label", f"O{index}")
            parsed_obstacle.setdefault("shape", "circle")
            if parsed_obstacle["shape"] == "rect":
                size = parsed_obstacle.get("size_m", [0.10, 0.10])
                parsed_obstacle["size_m"] = [float(size[0]), float(size[1])]
            else:
                parsed_obstacle["radius_m"] = float(parsed_obstacle.get("radius_m", 0.08))
            parsed.append(parsed_obstacle)
        except (KeyError, TypeError, ValueError, IndexError):
            continue
    return parsed


def interpolate_report_shape(
    series: list[tuple[float, float, float]] | None,
    time_s: float,
) -> tuple[float, float] | None:
    if not series:
        return None
    times = [item[0] for item in series]
    index = bisect.bisect_left(times, time_s)
    if index <= 0:
        if abs(times[0] - time_s) <= 1e-6:
            return series[0][1], series[0][2]
        return None
    if index >= len(series):
        if abs(times[-1] - time_s) <= 1e-6:
            return series[-1][1], series[-1][2]
        return None
    left = series[index - 1]
    right = series[index]
    alpha = (time_s - left[0]) / max(right[0] - left[0], 1e-9)
    return lerp(left[1], right[1], alpha), lerp(left[2], right[2], alpha)


def svg_polyline(points: list[tuple[float, float]]) -> str:
    return " ".join(f"{x:.2f},{y:.2f}" for x, y in points)


def split_polyline_chunks(
    rows: list[dict[str, Any]],
    x_key: str,
    y_key: str,
    *,
    max_step_m: float = 0.18,
    max_gap_s: float = 0.45,
) -> list[list[dict[str, Any]]]:
    chunks: list[list[dict[str, Any]]] = []
    current: list[dict[str, Any]] = []
    previous: dict[str, Any] | None = None
    for row in rows:
        if previous is not None:
            step = math.hypot(
                float(row[x_key]) - float(previous[x_key]),
                float(row[y_key]) - float(previous[y_key]),
            )
            gap = float(row["video_time_s"]) - float(previous["video_time_s"])
            if step > max_step_m or gap > max_gap_s:
                if len(current) >= 2:
                    chunks.append(current)
                current = []
        current.append(row)
        previous = row
    if len(current) >= 2:
        chunks.append(current)
    return chunks


def nice_grid_values(start: float, stop: float, step: float = 0.20) -> list[float]:
    first = math.ceil(start / step) * step
    last = math.floor(stop / step) * step
    values: list[float] = []
    value = first
    while value <= last + 1e-9:
        values.append(round(value, 10))
        value += step
    return values


def write_trajectory_svg(
    path: Path,
    rows: list[dict[str, Any]],
    map_width_m: float,
    map_height_m: float,
    plot_margin_m: float,
) -> None:
    width = 900
    height = 980
    pad_l = 86
    pad_t = 92
    plot_w = 740
    plot_h = 820

    x_values: list[float] = []
    y_values: list[float] = []
    for row in rows:
        for x_key, y_key in [
            ("gt_robot_center_x_m", "gt_robot_center_y_m"),
            ("reference_x_m", "reference_y_m"),
            ("log_real_x_m", "log_real_y_m"),
        ]:
            x_values.append(float(row[x_key]))
            y_values.append(float(row[y_key]))

    x_min = min(0.0, min(x_values) - plot_margin_m)
    x_max = max(map_width_m, max(x_values) + plot_margin_m)
    y_min = min(0.0, min(y_values) - plot_margin_m)
    y_max = max(map_height_m, max(y_values) + plot_margin_m)

    def px(x_m: float, y_m: float) -> tuple[float, float]:
        return (
            pad_l + ((x_m - x_min) / max(x_max - x_min, 1e-9)) * plot_w,
            pad_t + ((y_m - y_min) / max(y_max - y_min, 1e-9)) * plot_h,
        )

    by_segment: dict[int, list[dict[str, Any]]] = defaultdict(list)
    for row in rows:
        by_segment[int(row["segment"])].append(row)

    lines = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="white"/>',
        '<text x="34" y="42" font-family="Arial, sans-serif" font-size="24" fill="#222">Confronto traiettoria robot</text>',
        '<text x="34" y="68" font-family="Arial, sans-serif" font-size="14" fill="#555">Ground truth ArUco, stima planner e riferimento filtrato</text>',
        f'<rect x="{pad_l}" y="{pad_t}" width="{plot_w}" height="{plot_h}" fill="#fbfbfb" stroke="#222" stroke-width="2"/>',
    ]

    for x in nice_grid_values(x_min, x_max):
        sx, _ = px(x, y_min)
        lines.append(f'<line x1="{sx:.2f}" y1="{pad_t}" x2="{sx:.2f}" y2="{pad_t + plot_h}" stroke="#ddd" stroke-width="1"/>')
        lines.append(f'<text x="{sx - 10:.2f}" y="{pad_t + plot_h + 24}" font-family="Arial, sans-serif" font-size="12" fill="#666">{x:.1f}</text>')
    for y in nice_grid_values(y_min, y_max):
        _, sy = px(x_min, y)
        lines.append(f'<line x1="{pad_l}" y1="{sy:.2f}" x2="{pad_l + plot_w}" y2="{sy:.2f}" stroke="#ddd" stroke-width="1"/>')
        lines.append(f'<text x="{pad_l - 46}" y="{sy + 4:.2f}" font-family="Arial, sans-serif" font-size="12" fill="#666">{y:.1f}</text>')

    lines.extend(
        [
            '<text x="420" y="956" font-family="Arial, sans-serif" font-size="13" fill="#555">x [m]</text>',
            '<text x="24" y="500" font-family="Arial, sans-serif" font-size="13" fill="#555" transform="rotate(-90 24 500)">y [m]</text>',
        ]
    )
    for segment_rows in by_segment.values():
        if len(segment_rows) < 2:
            continue
        for chunk in split_polyline_chunks(segment_rows, "log_real_x_m", "log_real_y_m"):
            est = [
                px(float(row["log_real_x_m"]), float(row["log_real_y_m"]))
                for row in chunk
            ]
            lines.append(
                f'<polyline points="{svg_polyline(est)}" fill="none" stroke="#2f5bff" stroke-width="3.0" opacity="0.78"/>'
            )
        for chunk in split_polyline_chunks(segment_rows, "reference_x_m", "reference_y_m"):
            ref = [
                px(float(row["reference_x_m"]), float(row["reference_y_m"]))
                for row in chunk
            ]
            lines.append(
                f'<polyline points="{svg_polyline(ref)}" fill="none" stroke="#1b1b1b" stroke-width="3.2" opacity="0.95"/>'
            )
        for idx, row in enumerate(segment_rows):
            if idx % 3 != 0:
                continue
            cx, cy = px(float(row["gt_robot_center_x_m"]), float(row["gt_robot_center_y_m"]))
            lines.append(
                f'<circle cx="{cx:.2f}" cy="{cy:.2f}" r="2.1" fill="#f28c28" opacity="0.36"/>'
            )
    lines.extend(
        [
            '<rect x="575" y="28" width="280" height="78" fill="white" stroke="#ddd"/>',
            '<circle cx="617" cy="51" r="4" fill="#f28c28" opacity="0.55"/>',
            '<text x="652" y="56" font-family="Arial, sans-serif" font-size="14" fill="#333">Ground truth ArUco</text>',
            '<line x1="592" y1="75" x2="642" y2="75" stroke="#2f5bff" stroke-width="4"/>',
            '<text x="652" y="80" font-family="Arial, sans-serif" font-size="14" fill="#333">Stima planner</text>',
            '<line x1="592" y1="99" x2="642" y2="99" stroke="#1b1b1b" stroke-width="4"/>',
            '<text x="652" y="104" font-family="Arial, sans-serif" font-size="14" fill="#333">Riferimento</text>',
            "</svg>",
        ]
    )
    path.write_text("\n".join(lines), encoding="utf-8")


def write_error_svg(path: Path, rows: list[dict[str, Any]]) -> None:
    width = 1000
    height = 560
    pad_l = 76
    pad_t = 54
    plot_w = 860
    plot_h = 410
    t0 = min(float(row["log_time_s"]) for row in rows)
    t1 = max(float(row["log_time_s"]) for row in rows)
    raw_max = max(float(row["position_error_robot_center_real_m"]) for row in rows)
    ref_max = max(float(row["reference_position_error_m"]) for row in rows)
    y_max = max(0.10, raw_max, ref_max) * 1.10

    def px(time_s: float, value: float) -> tuple[float, float]:
        x = pad_l + ((time_s - t0) / max(t1 - t0, 1e-9)) * plot_w
        y = pad_t + plot_h - (value / y_max) * plot_h
        return x, y

    by_segment: dict[int, list[dict[str, Any]]] = defaultdict(list)
    for row in rows:
        by_segment[int(row["segment"])].append(row)

    lines = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="white"/>',
        '<text x="32" y="34" font-family="Arial, sans-serif" font-size="23" fill="#222">Errore di posizione nel tempo</text>',
        f'<rect x="{pad_l}" y="{pad_t}" width="{plot_w}" height="{plot_h}" fill="#fbfbfb" stroke="#222" stroke-width="2"/>',
    ]
    for i in range(0, 6):
        value = y_max * i / 5.0
        y = pad_t + plot_h - (value / y_max) * plot_h
        lines.append(f'<line x1="{pad_l}" y1="{y:.2f}" x2="{pad_l + plot_w}" y2="{y:.2f}" stroke="#ddd" stroke-width="1"/>')
        lines.append(f'<text x="{pad_l - 58}" y="{y + 4:.2f}" font-family="Arial, sans-serif" font-size="12" fill="#666">{value:.2f}</text>')
    for i in range(0, 6):
        value = t0 + (t1 - t0) * i / 5.0
        x = pad_l + ((value - t0) / max(t1 - t0, 1e-9)) * plot_w
        lines.append(f'<line x1="{x:.2f}" y1="{pad_t}" x2="{x:.2f}" y2="{pad_t + plot_h}" stroke="#eee" stroke-width="1"/>')
        lines.append(f'<text x="{x - 12:.2f}" y="{pad_t + plot_h + 25}" font-family="Arial, sans-serif" font-size="12" fill="#666">{value:.0f}</text>')
    lines.extend(
        [
        ]
    )
    for segment_rows in by_segment.values():
        if len(segment_rows) < 2:
            continue
        raw = [
            px(float(row["log_time_s"]), float(row["position_error_robot_center_real_m"]))
            for row in segment_rows
        ]
        ref = [
            px(float(row["log_time_s"]), float(row["reference_position_error_m"]))
            for row in segment_rows
        ]
        lines.extend(
            [
                f'<polyline points="{svg_polyline(raw)}" fill="none" stroke="#2f5bff" stroke-width="2.4" opacity="0.70"/>',
                f'<polyline points="{svg_polyline(ref)}" fill="none" stroke="#1b1b1b" stroke-width="2.8" opacity="0.95"/>',
            ]
        )
    lines.extend(
        [
            '<text x="455" y="535" font-family="Arial, sans-serif" font-size="13" fill="#555">tempo log [s]</text>',
            '<text x="24" y="292" font-family="Arial, sans-serif" font-size="13" fill="#555" transform="rotate(-90 24 292)">errore [m]</text>',
            '<rect x="644" y="28" width="300" height="58" fill="white" stroke="#ddd"/>',
            '<line x1="664" y1="51" x2="714" y2="51" stroke="#2f5bff" stroke-width="4" opacity="0.75"/>',
            '<text x="724" y="56" font-family="Arial, sans-serif" font-size="14" fill="#333">Errore stima</text>',
            '<line x1="664" y1="75" x2="714" y2="75" stroke="#1b1b1b" stroke-width="4"/>',
            '<text x="724" y="80" font-family="Arial, sans-serif" font-size="14" fill="#333">Errore su riferimento</text>',
            "</svg>",
        ]
    )
    path.write_text("\n".join(lines), encoding="utf-8")


def interpolate_rows(
    rows: list[dict[str, Any]],
    *,
    step_s: float,
    gap_min_s: float,
    max_gap_s: float,
    report_shape: list[tuple[float, float, float]] | None = None,
) -> list[dict[str, Any]]:
    ordered = sorted(rows, key=lambda row: float(row["log_time_s"]))
    if not ordered:
        return []
    dense: list[dict[str, Any]] = []
    step_s = max(step_s, 1e-3)
    for left, right in zip(ordered, ordered[1:]):
        base = dict(left)
        base["sample_source"] = "measured"
        dense.append(base)
        left_t = float(left["log_time_s"])
        right_t = float(right["log_time_s"])
        gap = right_t - left_t
        if gap <= gap_min_s or gap > max_gap_s:
            continue
        left_report = interpolate_report_shape(report_shape, left_t)
        right_report = interpolate_report_shape(report_shape, right_t)
        use_report_shape = left_report is not None and right_report is not None
        inserts = int(math.floor(gap / step_s))
        for index in range(1, inserts):
            t = left_t + index * step_s
            if t >= right_t:
                continue
            alpha = (t - left_t) / max(gap, 1e-9)
            row: dict[str, Any] = {
                "segment": left["segment"],
                "frame_idx": "",
                "video_time_s": lerp(float(left["video_time_s"]), float(right["video_time_s"]), alpha),
                "log_time_s": t,
                "sample_source": "interpolated_report_shape" if use_report_shape else "interpolated",
            }
            report_point = interpolate_report_shape(report_shape, t) if use_report_shape else None
            if report_point is not None and left_report is not None and right_report is not None:
                est_x, est_y = report_point
                left_bias_x = float(left["reference_x_m"]) - left_report[0]
                left_bias_y = float(left["reference_y_m"]) - left_report[1]
                right_bias_x = float(right["reference_x_m"]) - right_report[0]
                right_bias_y = float(right["reference_y_m"]) - right_report[1]
                ref_x = est_x + lerp(left_bias_x, right_bias_x, alpha)
                ref_y = est_y + lerp(left_bias_y, right_bias_y, alpha)
                row["log_real_x_m"] = est_x
                row["log_real_y_m"] = est_y
                row["reference_x_m"] = ref_x
                row["reference_y_m"] = ref_y
                row["gt_robot_center_x_m"] = ref_x
                row["gt_robot_center_y_m"] = ref_y
            else:
                for key in [
                    "gt_robot_center_x_m",
                    "gt_robot_center_y_m",
                    "reference_x_m",
                    "reference_y_m",
                    "log_real_x_m",
                    "log_real_y_m",
                ]:
                    row[key] = lerp(float(left[key]), float(right[key]), alpha)
            for key in [
                "gt_yaw_with_fitted_offset_rad",
                "reference_yaw_rad",
                "log_real_yaw_rad",
            ]:
                row[key] = lerp_angle(float(left[key]), float(right[key]), alpha)
            row["position_error_robot_center_real_m"] = lerp(
                float(left["position_error_robot_center_real_m"]),
                float(right["position_error_robot_center_real_m"]),
                alpha,
            )
            row["yaw_error_with_fitted_offset_deg"] = lerp(
                float(left["yaw_error_with_fitted_offset_deg"]),
                float(right["yaw_error_with_fitted_offset_deg"]),
                alpha,
            )
            row["reference_position_error_m"] = math.hypot(
                float(row["log_real_x_m"]) - float(row["reference_x_m"]),
                float(row["log_real_y_m"]) - float(row["reference_y_m"]),
            )
            row["reference_yaw_error_deg"] = math.degrees(
                abs(angle_delta(float(row["log_real_yaw_rad"]), float(row["reference_yaw_rad"])))
            )
            dense.append(row)
    final = dict(ordered[-1])
    final["sample_source"] = "measured"
    dense.append(final)
    return sorted(dense, key=lambda row: float(row["log_time_s"]))


def write_interpolated_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    fieldnames = [
        "sample_source",
        "segment",
        "frame_idx",
        "video_time_s",
        "log_time_s",
        "gt_robot_center_x_m",
        "gt_robot_center_y_m",
        "gt_yaw_with_fitted_offset_rad",
        "reference_x_m",
        "reference_y_m",
        "reference_yaw_rad",
        "log_real_x_m",
        "log_real_y_m",
        "log_real_yaw_rad",
        "position_error_robot_center_real_m",
        "yaw_error_with_fitted_offset_deg",
        "reference_position_error_m",
        "reference_yaw_error_deg",
    ]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field, "") for field in fieldnames})


def write_clean_trajectory_svg(
    path: Path,
    dense_rows: list[dict[str, Any]],
    measured_rows: list[dict[str, Any]],
    reference_points: list[tuple[str, float, float]],
    obstacles: list[dict[str, Any]],
    map_width_m: float,
    map_height_m: float,
    plot_margin_m: float,
    full_aruco: bool = False,
) -> None:
    width = 900
    height = 980
    pad_l = 86
    pad_t = 92
    plot_w = 740
    plot_h = 820

    x_values: list[float] = []
    y_values: list[float] = []
    for row in dense_rows:
        for x_key, y_key in [
            ("reference_x_m", "reference_y_m"),
            ("log_real_x_m", "log_real_y_m"),
        ]:
            x_values.append(float(row[x_key]))
            y_values.append(float(row[y_key]))
    for _, x_m, y_m in reference_points:
        x_values.append(x_m)
        y_values.append(y_m)
    for obstacle in obstacles:
        x_m, y_m = obstacle["center_m"]
        margin = float(obstacle.get("radius_m", 0.08))
        if obstacle.get("shape") == "rect":
            width_m, height_m = obstacle.get("size_m", [0.10, 0.10])
            x_values.extend([x_m - width_m * 0.5, x_m + width_m * 0.5])
            y_values.extend([y_m - height_m * 0.5, y_m + height_m * 0.5])
        else:
            x_values.extend([x_m - margin, x_m + margin])
            y_values.extend([y_m - margin, y_m + margin])
    x_min = min(0.0, min(x_values) - plot_margin_m)
    x_max = max(map_width_m, max(x_values) + plot_margin_m)
    y_min = min(0.0, min(y_values) - plot_margin_m)
    y_max = max(map_height_m, max(y_values) + plot_margin_m)

    def px(x_m: float, y_m: float) -> tuple[float, float]:
        return (
            pad_l + ((x_m - x_min) / max(x_max - x_min, 1e-9)) * plot_w,
            pad_t + ((y_m - y_min) / max(y_max - y_min, 1e-9)) * plot_h,
        )

    est_points = [
        px(float(row["log_real_x_m"]), float(row["log_real_y_m"]))
        for row in dense_rows
    ]
    ref_points = [
        px(float(row["reference_x_m"]), float(row["reference_y_m"]))
        for row in dense_rows
    ]

    lines = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="white"/>',
        '<text x="34" y="42" font-family="Arial, sans-serif" font-size="24" fill="#222">Confronto traiettoria robot</text>',
        (
            '<text x="34" y="68" font-family="Arial, sans-serif" font-size="14" fill="#555">'
            'Stima planner e ground truth ArUco continuo</text>'
            if full_aruco
            else '<text x="34" y="68" font-family="Arial, sans-serif" font-size="14" fill="#555">'
            'Stima planner e ground truth ArUco ricostruito tra campioni validi</text>'
        ),
        f'<rect x="{pad_l}" y="{pad_t}" width="{plot_w}" height="{plot_h}" fill="#fbfbfb" stroke="#222" stroke-width="2"/>',
    ]
    for x in nice_grid_values(x_min, x_max):
        sx, _ = px(x, y_min)
        lines.append(f'<line x1="{sx:.2f}" y1="{pad_t}" x2="{sx:.2f}" y2="{pad_t + plot_h}" stroke="#ddd" stroke-width="1"/>')
        lines.append(f'<text x="{sx - 10:.2f}" y="{pad_t + plot_h + 24}" font-family="Arial, sans-serif" font-size="12" fill="#666">{x:.1f}</text>')
    for y in nice_grid_values(y_min, y_max):
        _, sy = px(x_min, y)
        lines.append(f'<line x1="{pad_l}" y1="{sy:.2f}" x2="{pad_l + plot_w}" y2="{sy:.2f}" stroke="#ddd" stroke-width="1"/>')
        lines.append(f'<text x="{pad_l - 46}" y="{sy + 4:.2f}" font-family="Arial, sans-serif" font-size="12" fill="#666">{y:.1f}</text>')

    measured_by_segment: dict[int, list[dict[str, Any]]] = defaultdict(list)
    for row in measured_rows:
        measured_by_segment[int(row["segment"])].append(row)

    lines.extend(
        [
            '<text x="420" y="956" font-family="Arial, sans-serif" font-size="13" fill="#555">x [m]</text>',
            '<text x="24" y="500" font-family="Arial, sans-serif" font-size="13" fill="#555" transform="rotate(-90 24 500)">y [m]</text>',
        ]
    )
    for obstacle in obstacles:
        x_m, y_m = obstacle["center_m"]
        sx, sy = px(float(x_m), float(y_m))
        label = str(obstacle.get("label", "O"))
        if obstacle.get("shape") == "rect":
            width_m, height_m = obstacle.get("size_m", [0.10, 0.10])
            x0, y0 = px(float(x_m) - width_m * 0.5, float(y_m) - height_m * 0.5)
            x1, y1 = px(float(x_m) + width_m * 0.5, float(y_m) + height_m * 0.5)
            lines.append(
                f'<rect x="{min(x0, x1):.2f}" y="{min(y0, y1):.2f}" '
                f'width="{abs(x1 - x0):.2f}" height="{abs(y1 - y0):.2f}" '
                'rx="4" fill="#d8d8d8" stroke="#555" stroke-width="2" opacity="0.62"/>'
            )
        else:
            radius_m = float(obstacle.get("radius_m", 0.08))
            rx = abs(px(float(x_m) + radius_m, float(y_m))[0] - sx)
            ry = abs(px(float(x_m), float(y_m) + radius_m)[1] - sy)
            lines.append(
                f'<ellipse cx="{sx:.2f}" cy="{sy:.2f}" rx="{rx:.2f}" ry="{ry:.2f}" '
                'fill="#d8d8d8" stroke="#555" stroke-width="2" opacity="0.62"/>'
            )
        lines.append(
            f'<text x="{sx - 9:.2f}" y="{sy + 5:.2f}" font-family="Arial, sans-serif" '
            f'font-size="13" font-weight="bold" fill="#333">{label}</text>'
        )
    if not full_aruco:
        for segment_rows in measured_by_segment.values():
            for chunk in split_polyline_chunks(segment_rows, "reference_x_m", "reference_y_m"):
                measured = [
                    px(float(row["reference_x_m"]), float(row["reference_y_m"]))
                    for row in chunk
                ]
                lines.append(
                    f'<polyline points="{svg_polyline(measured)}" fill="none" stroke="#f28c28" stroke-width="2.4" opacity="0.64"/>'
                )
    lines.append(
        f'<polyline points="{svg_polyline(est_points)}" fill="none" stroke="#2f5bff" stroke-width="3.0" opacity="0.80"/>'
    )
    if full_aruco:
        lines.append(
            f'<polyline points="{svg_polyline(ref_points)}" fill="none" stroke="#f28c28" stroke-width="3.6" opacity="0.96" stroke-linecap="round" stroke-linejoin="round"/>'
        )
    else:
        lines.append(
            f'<polyline points="{svg_polyline(ref_points)}" fill="none" stroke="#1b1b1b" stroke-width="3.2" opacity="0.96"/>'
        )
        for segment_rows in measured_by_segment.values():
            for chunk in split_polyline_chunks(segment_rows, "reference_x_m", "reference_y_m"):
                measured = [
                    px(float(row["reference_x_m"]), float(row["reference_y_m"]))
                    for row in chunk
                ]
                lines.append(
                    f'<polyline points="{svg_polyline(measured)}" fill="none" stroke="#f28c28" stroke-width="4.0" opacity="0.90" stroke-linecap="round" stroke-linejoin="round"/>'
                )

    for label, x_m, y_m in reference_points:
        sx, sy = px(x_m, y_m)
        lines.extend(
            [
                f'<circle cx="{sx:.2f}" cy="{sy:.2f}" r="6.0" fill="#ffffff" stroke="#7a4f00" stroke-width="2"/>',
                f'<text x="{sx + 8:.2f}" y="{sy - 8:.2f}" font-family="Arial, sans-serif" font-size="16" font-weight="bold" fill="#6b3f00">{label}</text>',
            ]
        )

    start_row = dense_rows[0]
    goal_row = dense_rows[-1]
    start_x, start_y = px(float(start_row["reference_x_m"]), float(start_row["reference_y_m"]))
    goal_x, goal_y = px(float(goal_row["reference_x_m"]), float(goal_row["reference_y_m"]))
    lines.extend(
        [
            f'<circle cx="{start_x:.2f}" cy="{start_y:.2f}" r="7.0" fill="#0f9d58" stroke="white" stroke-width="2"/>',
            f'<text x="{start_x + 10:.2f}" y="{start_y + 4:.2f}" font-family="Arial, sans-serif" font-size="15" font-weight="bold" fill="#0b6b3d">Start</text>',
            f'<circle cx="{goal_x:.2f}" cy="{goal_y:.2f}" r="7.0" fill="#d23f31" stroke="white" stroke-width="2"/>',
            f'<text x="{goal_x + 10:.2f}" y="{goal_y + 4:.2f}" font-family="Arial, sans-serif" font-size="15" font-weight="bold" fill="#9d2d23">Goal</text>',
        ]
    )
    if full_aruco:
        lines.extend(
            [
                '<rect x="536" y="28" width="320" height="100" fill="white" stroke="#ddd"/>',
                '<line x1="555" y1="51" x2="595" y2="51" stroke="#f28c28" stroke-width="4" opacity="0.88"/>',
                '<text x="604" y="56" font-family="Arial, sans-serif" font-size="14" fill="#333">Ground truth ArUco</text>',
                '<line x1="555" y1="75" x2="595" y2="75" stroke="#2f5bff" stroke-width="4"/>',
                '<text x="604" y="80" font-family="Arial, sans-serif" font-size="14" fill="#333">Stima planner</text>',
                '<circle cx="575" cy="99" r="5" fill="#ffffff" stroke="#7a4f00" stroke-width="2"/>',
                '<text x="604" y="104" font-family="Arial, sans-serif" font-size="14" fill="#333">Reference A/B/C/D</text>',
                '<rect x="568" y="115" width="14" height="10" rx="2" fill="#d8d8d8" stroke="#555" stroke-width="2" opacity="0.72"/>',
                '<text x="604" y="126" font-family="Arial, sans-serif" font-size="14" fill="#333">Ostacoli reali</text>',
                "</svg>",
            ]
        )
    else:
        lines.extend(
            [
                '<rect x="536" y="28" width="320" height="124" fill="white" stroke="#ddd"/>',
                '<line x1="555" y1="51" x2="595" y2="51" stroke="#f28c28" stroke-width="4" opacity="0.88"/>',
                '<text x="604" y="56" font-family="Arial, sans-serif" font-size="14" fill="#333">ArUco misurato</text>',
                '<line x1="555" y1="75" x2="595" y2="75" stroke="#2f5bff" stroke-width="4"/>',
                '<text x="604" y="80" font-family="Arial, sans-serif" font-size="14" fill="#333">Stima planner</text>',
                '<line x1="555" y1="99" x2="595" y2="99" stroke="#1b1b1b" stroke-width="4"/>',
                '<text x="604" y="104" font-family="Arial, sans-serif" font-size="14" fill="#333">Ground truth ArUco</text>',
                '<circle cx="575" cy="122" r="5" fill="#ffffff" stroke="#7a4f00" stroke-width="2"/>',
                '<text x="604" y="126" font-family="Arial, sans-serif" font-size="14" fill="#333">Reference A/B/C/D</text>',
                '<rect x="568" y="137" width="14" height="10" rx="2" fill="#d8d8d8" stroke="#555" stroke-width="2" opacity="0.72"/>',
                '<text x="604" y="148" font-family="Arial, sans-serif" font-size="14" fill="#333">Ostacoli reali</text>',
                "</svg>",
            ]
        )
    path.write_text("\n".join(lines), encoding="utf-8")


def write_clean_error_svg(path: Path, dense_rows: list[dict[str, Any]], measured_rows: list[dict[str, Any]]) -> None:
    width = 1000
    height = 560
    pad_l = 76
    pad_t = 54
    plot_w = 860
    plot_h = 410
    t0 = min(float(row["log_time_s"]) for row in dense_rows)
    t1 = max(float(row["log_time_s"]) for row in dense_rows)
    y_max = max(
        0.10,
        max(float(row["reference_position_error_m"]) for row in dense_rows),
        max(float(row["position_error_robot_center_real_m"]) for row in measured_rows),
    ) * 1.10

    def px(time_s: float, value: float) -> tuple[float, float]:
        x = pad_l + ((time_s - t0) / max(t1 - t0, 1e-9)) * plot_w
        y = pad_t + plot_h - (value / y_max) * plot_h
        return x, y

    error_line = [
        px(float(row["log_time_s"]), float(row["reference_position_error_m"]))
        for row in dense_rows
    ]
    lines = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="white"/>',
        '<text x="32" y="34" font-family="Arial, sans-serif" font-size="23" fill="#222">Errore di posizione nel tempo</text>',
        f'<rect x="{pad_l}" y="{pad_t}" width="{plot_w}" height="{plot_h}" fill="#fbfbfb" stroke="#222" stroke-width="2"/>',
    ]
    for i in range(0, 6):
        value = y_max * i / 5.0
        y = pad_t + plot_h - (value / y_max) * plot_h
        lines.append(f'<line x1="{pad_l}" y1="{y:.2f}" x2="{pad_l + plot_w}" y2="{y:.2f}" stroke="#ddd" stroke-width="1"/>')
        lines.append(f'<text x="{pad_l - 58}" y="{y + 4:.2f}" font-family="Arial, sans-serif" font-size="12" fill="#666">{value:.2f}</text>')
    for i in range(0, 6):
        value = t0 + (t1 - t0) * i / 5.0
        x = pad_l + ((value - t0) / max(t1 - t0, 1e-9)) * plot_w
        lines.append(f'<line x1="{x:.2f}" y1="{pad_t}" x2="{x:.2f}" y2="{pad_t + plot_h}" stroke="#eee" stroke-width="1"/>')
        lines.append(f'<text x="{x - 12:.2f}" y="{pad_t + plot_h + 25}" font-family="Arial, sans-serif" font-size="12" fill="#666">{value:.0f}</text>')
    lines.append(
        f'<polyline points="{svg_polyline(error_line)}" fill="none" stroke="#1b1b1b" stroke-width="3.0" opacity="0.96"/>'
    )
    measured_line = [
        px(float(row["log_time_s"]), float(row["position_error_robot_center_real_m"]))
        for row in sorted(measured_rows, key=lambda item: float(item["log_time_s"]))
    ]
    lines.append(
        f'<polyline points="{svg_polyline(measured_line)}" fill="none" stroke="#2f5bff" stroke-width="2.3" opacity="0.66"/>'
    )
    lines.extend(
        [
            '<text x="455" y="535" font-family="Arial, sans-serif" font-size="13" fill="#555">tempo log [s]</text>',
            '<text x="24" y="292" font-family="Arial, sans-serif" font-size="13" fill="#555" transform="rotate(-90 24 292)">errore [m]</text>',
            '<rect x="642" y="28" width="305" height="58" fill="white" stroke="#ddd"/>',
            '<line x1="646" y1="51" x2="676" y2="51" stroke="#2f5bff" stroke-width="4" opacity="0.70"/>',
            '<text x="686" y="56" font-family="Arial, sans-serif" font-size="14" fill="#333">Errore misurato</text>',
            '<line x1="646" y1="75" x2="676" y2="75" stroke="#1b1b1b" stroke-width="4"/>',
            '<text x="686" y="80" font-family="Arial, sans-serif" font-size="14" fill="#333">Errore di posizione</text>',
            "</svg>",
        ]
    )
    path.write_text("\n".join(lines), encoding="utf-8")


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv or sys.argv[1:])
    rows = read_alignment(args.alignment_csv)
    if not rows:
        raise SystemExit("Nessuna riga valida nel CSV di allineamento")
    add_reference_trace(rows, args.smooth_window_s)
    report_shape = load_report_shape(args.report, args.alignment_summary)
    reference_points = load_reference_points(args.config)
    obstacles = load_obstacles(args.obstacles)

    args.output_dir.mkdir(parents=True, exist_ok=True)
    reference_csv = args.output_dir / "00058_reference_comparison.csv"
    trajectory_svg = args.output_dir / "00058_trajectory_comparison.svg"
    error_svg = args.output_dir / "00058_position_error_comparison.svg"
    interpolated_csv = args.output_dir / "00058_clean_interpolated_comparison.csv"
    clean_trajectory_svg = args.output_dir / "00058_clean_trajectory_comparison.svg"
    full_aruco_trajectory_svg = args.output_dir / "00058_clean_trajectory_full_aruco_comparison.svg"
    clean_error_svg = args.output_dir / "00058_clean_position_error_comparison.svg"
    summary_json = args.output_dir / "00058_reference_comparison_summary.json"

    write_reference_csv(reference_csv, rows)
    interpolated_rows = interpolate_rows(
        rows,
        step_s=args.gap_fill_step_s,
        gap_min_s=args.gap_fill_min_s,
        max_gap_s=args.max_gap_fill_s,
        report_shape=report_shape,
    )
    write_interpolated_csv(interpolated_csv, interpolated_rows)
    write_trajectory_svg(
        trajectory_svg,
        rows,
        args.map_width_m,
        args.map_height_m,
        args.plot_margin_m,
    )
    write_error_svg(error_svg, rows)
    write_clean_trajectory_svg(
        clean_trajectory_svg,
        interpolated_rows,
        rows,
        reference_points,
        obstacles,
        args.map_width_m,
        args.map_height_m,
        args.plot_margin_m,
    )
    write_clean_trajectory_svg(
        full_aruco_trajectory_svg,
        interpolated_rows,
        rows,
        reference_points,
        obstacles,
        args.map_width_m,
        args.map_height_m,
        args.plot_margin_m,
        full_aruco=True,
    )
    write_clean_error_svg(clean_error_svg, interpolated_rows, rows)

    raw_pos = [float(row["position_error_robot_center_real_m"]) for row in rows]
    raw_yaw = [float(row["yaw_error_with_fitted_offset_deg"]) for row in rows]
    ref_pos = [float(row["reference_position_error_m"]) for row in rows]
    ref_yaw = [float(row["reference_yaw_error_deg"]) for row in rows]
    summary = {
        "source_alignment_csv": str(args.alignment_csv),
        "smooth_window_s": float(args.smooth_window_s),
        "gap_fill_step_s": float(args.gap_fill_step_s),
        "gap_fill_min_s": float(args.gap_fill_min_s),
        "max_gap_fill_s": float(args.max_gap_fill_s),
        "gap_fill_shape_source": "report" if report_shape is not None else "linear",
        "obstacles": obstacles,
        "samples": len(rows),
        "interpolated_samples": len(interpolated_rows),
        "position_error_m": stats(raw_pos),
        "yaw_error_deg": stats(raw_yaw),
        "reference_position_error_m": stats(ref_pos),
        "reference_yaw_error_deg": stats(ref_yaw),
        "outputs": {
            "reference_csv": str(reference_csv),
            "interpolated_csv": str(interpolated_csv),
            "trajectory_svg": str(trajectory_svg),
            "error_svg": str(error_svg),
            "clean_trajectory_svg": str(clean_trajectory_svg),
            "full_aruco_trajectory_svg": str(full_aruco_trajectory_svg),
            "clean_error_svg": str(clean_error_svg),
        },
        "notes": [
            "The reference trace is a segment-wise moving average of the ArUco robot-center pose.",
            "The clean figures fill gaps between adjacent valid ArUco windows; the CSV marks these rows as measured/interpolated.",
            "Use the raw ArUco/report metrics for quantitative claims; use the reference trace for readable figures.",
        ],
    }
    summary_json.write_text(json.dumps(summary, indent=2), encoding="utf-8")

    print(f"reference_csv={reference_csv}")
    print(f"interpolated_csv={interpolated_csv}")
    print(f"trajectory_svg={trajectory_svg}")
    print(f"error_svg={error_svg}")
    print(f"clean_trajectory_svg={clean_trajectory_svg}")
    print(f"full_aruco_trajectory_svg={full_aruco_trajectory_svg}")
    print(f"clean_error_svg={clean_error_svg}")
    print(f"summary_json={summary_json}")
    print(f"samples={len(rows)}")
    print(f"position_mean_m={statistics.fmean(raw_pos):.4f}")
    print(f"position_p95_m={percentile(raw_pos, 95.0):.4f}")
    print(f"reference_position_mean_m={statistics.fmean(ref_pos):.4f}")
    print(f"reference_position_p95_m={percentile(ref_pos, 95.0):.4f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
