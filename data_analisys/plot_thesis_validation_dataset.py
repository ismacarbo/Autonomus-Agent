"""Generate summary SVG figures for the curated thesis validation dataset."""

from __future__ import annotations

import csv
import html
import json
import math
from collections import Counter
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_DATASET_DIR = REPO_ROOT / "datasets" / "thesis_validation"

SUBSET_LABELS = {
    "car_structured": "Car structured",
    "car_unstructured": "Car unstructured",
    "car_mixed": "Car mixed",
    "aruco_ground_truth": "ArUco ground truth",
    "tank_mixed": "Tank mixed",
    "simulation_support": "Simulation support",
}

CORE_SUBSET_LABELS = {
    "car_structured": "Auto str.",
    "car_unstructured": "Auto non str.",
    "car_mixed": "Auto mista",
    "aruco_ground_truth": "ArUco",
    "tank_mixed": "Cingolato",
    "simulation_support": "Sim. supporto",
}

SUBSET_COLORS = {
    "car_structured": "#2f6f9f",
    "car_unstructured": "#3a9c6b",
    "car_mixed": "#d99a2b",
    "aruco_ground_truth": "#1f1f1f",
    "tank_mixed": "#7d5fb2",
    "simulation_support": "#6c7a89",
}

METRICS = [
    ("duration_s", "Duration [s]", "duration_stats_by_subset"),
    ("estimated_path_length_m", "Estimated path length [m]", "path_length_stats_by_subset"),
    ("min_clearance_m", "Minimum LiDAR clearance [m]", "clearance_stats_by_subset"),
    ("max_cross_track_m", "Maximum cross-track error [m]", "cross_track_stats_by_subset"),
    ("heading_error_p95_deg", "Heading error p95 [deg]", "heading_p95_stats_by_subset"),
    ("planner_compute_p95_ms", "Planner compute p95 [ms]", "planner_time_stats_by_subset"),
]


@dataclass(frozen=True)
class MetricStats:
    count: int
    mean: float | None
    std: float | None
    minimum: float | None
    maximum: float | None


def main() -> int:
    dataset_dir = DEFAULT_DATASET_DIR
    rows = read_csv(dataset_dir / "metrics_summary.csv")
    manifest = read_csv(dataset_dir / "manifest.csv")
    archive = read_json(dataset_dir / "archive_summary.json")
    figures_dir = dataset_dir / "figures"
    figures_dir.mkdir(parents=True, exist_ok=True)

    stats = build_stats(rows)
    write_stats_outputs(dataset_dir, stats)
    write_success_rate_chart(figures_dir, manifest)
    write_subset_count_chart(figures_dir, manifest)
    write_archive_status_chart(figures_dir, archive)
    for metric, ylabel, filename in METRICS:
        write_metric_stats_chart(figures_dir, rows, metric, ylabel, filename)
    write_mixed_activity_chart(figures_dir, rows)
    write_overview_dashboard(figures_dir, rows, manifest)
    write_core_validation_statistics_chart(figures_dir, rows, manifest)
    update_dataset_readme(dataset_dir)
    print(f"Generated dataset figures in {figures_dir}")
    return 0


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def read_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def value(row: dict[str, str], key: str) -> float | None:
    raw = row.get(key, "").strip()
    if not raw:
        return None
    try:
        out = float(raw)
    except ValueError:
        return None
    return out if math.isfinite(out) else None


def ordered_subsets(rows: Iterable[dict[str, str]]) -> list[str]:
    seen: list[str] = []
    for row in rows:
        subset = row.get("subset", "")
        if subset and subset not in seen:
            seen.append(subset)
    return seen


def stats_for(values: list[float]) -> MetricStats:
    if not values:
        return MetricStats(0, None, None, None, None)
    mean = sum(values) / len(values)
    if len(values) > 1:
        variance = sum((item - mean) ** 2 for item in values) / (len(values) - 1)
        std = math.sqrt(variance)
    else:
        std = 0.0
    return MetricStats(len(values), mean, std, min(values), max(values))


def build_stats(rows: list[dict[str, str]]) -> dict[str, dict[str, MetricStats]]:
    out: dict[str, dict[str, MetricStats]] = {}
    subsets = ordered_subsets(rows)
    for metric, _, _ in METRICS:
        out[metric] = {}
        for subset in subsets:
            values = [v for row in rows if row.get("subset") == subset for v in [value(row, metric)] if v is not None]
            out[metric][subset] = stats_for(values)
    return out


def write_stats_outputs(dataset_dir: Path, stats: dict[str, dict[str, MetricStats]]) -> None:
    serializable: dict[str, dict[str, dict[str, float | int | None]]] = {}
    rows: list[dict[str, Any]] = []
    for metric, by_subset in stats.items():
        serializable[metric] = {}
        for subset, item in by_subset.items():
            entry = {
                "count": item.count,
                "mean": item.mean,
                "std": item.std,
                "min": item.minimum,
                "max": item.maximum,
            }
            serializable[metric][subset] = entry
            rows.append({"metric": metric, "subset": subset, **entry})

    (dataset_dir / "stats_summary.json").write_text(
        json.dumps(serializable, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    with (dataset_dir / "stats_summary.csv").open("w", newline="", encoding="utf-8") as f:
        fieldnames = ["metric", "subset", "count", "mean", "std", "min", "max"]
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: format_cell(row.get(key)) for key in fieldnames})


def write_success_rate_chart(figures_dir: Path, manifest: list[dict[str, str]]) -> None:
    subsets = ordered_subsets(manifest)
    rows = []
    for subset in subsets:
        subset_rows = [row for row in manifest if row.get("subset") == subset]
        success = sum(1 for row in subset_rows if row.get("status") == "goal_reached")
        total = len(subset_rows)
        rows.append(
            {
                "subset": subset,
                "value": 100.0 * success / total if total else 0.0,
                "count": total,
            }
        )
    write_simple_bar_chart(
        figures_dir / "success_rate_by_subset.svg",
        "Success rate of curated reports",
        "success rate [%]",
        rows,
        fixed_y_max=105.0,
    )


def write_subset_count_chart(figures_dir: Path, manifest: list[dict[str, str]]) -> None:
    subsets = ordered_subsets(manifest)
    counts = Counter(row.get("subset", "") for row in manifest)
    rows = [{"subset": subset, "value": float(counts[subset]), "count": counts[subset]} for subset in subsets]
    write_simple_bar_chart(
        figures_dir / "report_count_by_subset.svg",
        "Reports included by validation subset",
        "reports",
        rows,
    )


def write_metric_stats_chart(
    figures_dir: Path,
    rows: list[dict[str, str]],
    metric: str,
    ylabel: str,
    filename: str,
) -> None:
    subsets = ordered_subsets(rows)
    items = []
    max_y = 0.0
    for subset in subsets:
        values = [v for row in rows if row.get("subset") == subset for v in [value(row, metric)] if v is not None]
        item = stats_for(values)
        items.append((subset, values, item))
        for candidate in [item.maximum, (item.mean or 0.0) + (item.std or 0.0)]:
            if candidate is not None:
                max_y = max(max_y, candidate)

    width = 1120
    height = 560
    pad_l = 86
    pad_r = 34
    pad_t = 66
    pad_b = 126
    plot_w = width - pad_l - pad_r
    plot_h = height - pad_t - pad_b
    y_max = nice_upper(max_y)

    def sy(v: float) -> float:
        return pad_t + plot_h - (v / y_max) * plot_h

    parts = svg_start(width, height, title=ylabel)
    parts.extend(axis_grid(pad_l, pad_t, plot_w, plot_h, 0.0, y_max, ylabel))
    category_w = plot_w / max(1, len(items))
    bar_w = min(88.0, category_w * 0.45)

    for idx, (subset, values, item) in enumerate(items):
        cx = pad_l + category_w * (idx + 0.5)
        color = SUBSET_COLORS.get(subset, "#607d8b")
        label = SUBSET_LABELS.get(subset, subset)
        if item.mean is not None:
            bar_top = sy(item.mean)
            parts.append(
                f'<rect x="{cx - bar_w / 2:.2f}" y="{bar_top:.2f}" width="{bar_w:.2f}" '
                f'height="{pad_t + plot_h - bar_top:.2f}" fill="{color}" opacity="0.88"/>'
            )
            if item.std is not None:
                std_lo = max(0.0, item.mean - item.std)
                std_hi = item.mean + item.std
                parts.append(line(cx, sy(std_lo), cx, sy(std_hi), "#111827", 2.0))
                parts.append(line(cx - 12, sy(std_hi), cx + 12, sy(std_hi), "#111827", 2.0))
                parts.append(line(cx - 12, sy(std_lo), cx + 12, sy(std_lo), "#111827", 2.0))
            if item.minimum is not None and item.maximum is not None:
                x = cx + bar_w * 0.72
                parts.append(line(x, sy(item.minimum), x, sy(item.maximum), "#4b5563", 1.6, dash="4 3"))
                parts.append(line(x - 9, sy(item.minimum), x + 9, sy(item.minimum), "#4b5563", 1.6))
                parts.append(line(x - 9, sy(item.maximum), x + 9, sy(item.maximum), "#4b5563", 1.6))
            for sample_index, sample in enumerate(values):
                jitter = ((sample_index % 5) - 2) * 4
                parts.append(circle(cx + jitter, sy(sample), 3.0, "#111827", opacity=0.68))
            parts.append(text(cx, bar_top - 8, f"n={item.count}", size=12, anchor="middle", color="#374151"))
        else:
            parts.append(text(cx, sy(0) - 8, "n/a", size=12, anchor="middle", color="#374151"))
        label_lines(parts, label, cx, height - 84)

    parts.append(legend([(SUBSET_COLORS["car_structured"], "bar = mean"), ("#111827", "solid whisker = std"), ("#4b5563", "dashed whisker = min/max"), ("#111827", "dots = runs")], width - 300, pad_t + 10))
    parts.append("</svg>\n")
    (figures_dir / f"{filename}.svg").write_text("\n".join(parts), encoding="utf-8")


def write_archive_status_chart(figures_dir: Path, archive: dict[str, Any]) -> None:
    keys = ["current_reports", "local_backup_20260625"]
    labels = ["Current reports", "Local backup"]
    statuses = ["goal_reached", "collision", "timeout", "safety_stop", "live", "listening", "stopped"]
    colors = {
        "goal_reached": "#3a9c6b",
        "collision": "#c94c4c",
        "timeout": "#d99a2b",
        "safety_stop": "#7d5fb2",
        "live": "#6c7a89",
        "listening": "#9aa5b1",
        "stopped": "#3b3b3b",
    }
    width = 860
    height = 520
    pad_l = 86
    pad_r = 250
    pad_t = 66
    pad_b = 86
    plot_w = width - pad_l - pad_r
    plot_h = height - pad_t - pad_b
    category_w = plot_w / len(keys)
    parts = svg_start(width, height, title="Status distribution in available archives")
    parts.extend(axis_grid(pad_l, pad_t, plot_w, plot_h, 0.0, 100.0, "reports [%]"))
    for idx, key in enumerate(keys):
        total = max(1, int(archive.get(key, {}).get("json_reports", 0)))
        cx = pad_l + category_w * (idx + 0.5)
        bar_w = min(110, category_w * 0.48)
        current_top = pad_t + plot_h
        for status in statuses:
            count = archive.get(key, {}).get("status_counts", {}).get(status, 0)
            pct = 100.0 * count / total
            h = pct / 100.0 * plot_h
            y = current_top - h
            parts.append(
                f'<rect x="{cx - bar_w / 2:.2f}" y="{y:.2f}" width="{bar_w:.2f}" '
                f'height="{h:.2f}" fill="{colors[status]}" stroke="#ffffff" stroke-width="0.6"/>'
            )
            current_top = y
        label_lines(parts, labels[idx], cx, height - 54)
        parts.append(text(cx, pad_t + plot_h + 20, f"n={total}", size=12, anchor="middle", color="#374151"))
    parts.append(legend([(colors[s], s) for s in statuses], width - pad_r + 38, pad_t + 8))
    parts.append("</svg>\n")
    (figures_dir / "archive_status_distribution.svg").write_text("\n".join(parts), encoding="utf-8")


def write_mixed_activity_chart(figures_dir: Path, rows: list[dict[str, str]]) -> None:
    selected = [row for row in rows if row.get("subset") in {"aruco_ground_truth", "tank_mixed"}]
    switch_rows = []
    gate_rows = []
    for row in selected:
        duration = value(row, "duration_s") or 1.0
        gate_time = value(row, "gate_time_s") or 0.0
        switch_rows.append({"subset": short_run(row["run_id"]), "value": value(row, "switch_events") or 0.0, "count": 1})
        gate_rows.append({"subset": short_run(row["run_id"]), "value": 100.0 * gate_time / duration, "count": 1})

    write_dual_panel_bar_chart(
        figures_dir / "mixed_activity_hardware.svg",
        "Mixed-mode activity on hardware runs",
        ("Switch events", "events", switch_rows, "#d99a2b"),
        ("Gate time ratio", "time [%]", gate_rows, "#3a9c6b"),
    )


def write_core_validation_statistics_chart(
    figures_dir: Path,
    rows: list[dict[str, str]],
    manifest: list[dict[str, str]],
) -> None:
    subsets = ordered_subsets(manifest)
    success_rows = []
    for subset in subsets:
        subset_rows = [row for row in manifest if row.get("subset") == subset]
        success = sum(1 for row in subset_rows if row.get("status") == "goal_reached")
        total = len(subset_rows)
        success_rows.append({"subset": subset, "value": 100.0 * success / total if total else 0.0, "count": total})

    width = 1120
    height = 820
    parts = svg_start(width, height, title="Statistiche principali della validazione")
    draw_panel_simple_bars(
        parts,
        76,
        84,
        968,
        180,
        "Tasso di completamento",
        "successo [%]",
        success_rows,
        fixed_y_max=105.0,
    )
    draw_panel_metric_stats(
        parts,
        76,
        372,
        430,
        260,
        "Errore laterale massimo",
        "m",
        rows,
        subsets,
        "max_cross_track_m",
    )
    draw_panel_metric_stats(
        parts,
        614,
        372,
        430,
        260,
        "Distanza minima LiDAR",
        "m",
        rows,
        subsets,
        "min_clearance_m",
    )
    parts.append(
        text(
            width / 2,
            762,
            "Le barre indicano la media; i segmenti continui la deviazione standard; i segmenti tratteggiati l'intervallo min-max.",
            size=13,
            anchor="middle",
            color="#374151",
        )
    )
    parts.append("</svg>\n")
    (figures_dir / "core_validation_statistics.svg").write_text("\n".join(parts), encoding="utf-8")


def draw_panel_simple_bars(
    parts: list[str],
    x0: float,
    y0: float,
    plot_w: float,
    plot_h: float,
    title: str,
    ylabel: str,
    rows: list[dict[str, Any]],
    fixed_y_max: float | None = None,
) -> None:
    max_value = max([float(row["value"]) for row in rows] + [1.0])
    y_max = fixed_y_max or nice_upper(max_value)

    def sy(v: float) -> float:
        return y0 + plot_h - (v / y_max) * plot_h

    parts.append(text(x0 + plot_w / 2, y0 - 22, title, size=16, anchor="middle", weight="700"))
    parts.extend(axis_grid(x0, y0, plot_w, plot_h, 0.0, y_max, ylabel, title_inside=False))
    category_w = plot_w / max(1, len(rows))
    bar_w = min(74.0, category_w * 0.50)
    for idx, row in enumerate(rows):
        subset = str(row["subset"])
        cx = x0 + category_w * (idx + 0.5)
        v = float(row["value"])
        top = sy(v)
        color = SUBSET_COLORS.get(subset, "#607d8b")
        parts.append(
            f'<rect x="{cx - bar_w / 2:.2f}" y="{top:.2f}" width="{bar_w:.2f}" '
            f'height="{y0 + plot_h - top:.2f}" fill="{color}" opacity="0.88"/>'
        )
        parts.append(text(cx, top - 8, f"{v:.0f}%", size=11, anchor="middle"))
        parts.append(text(cx, y0 + plot_h + 18, f"n={row['count']}", size=10, anchor="middle", color="#4b5563"))
        label_lines(parts, CORE_SUBSET_LABELS.get(subset, subset), cx, y0 + plot_h + 36, size=10)


def draw_panel_metric_stats(
    parts: list[str],
    x0: float,
    y0: float,
    plot_w: float,
    plot_h: float,
    title: str,
    ylabel: str,
    rows: list[dict[str, str]],
    subsets: list[str],
    metric: str,
) -> None:
    entries = []
    max_y = 0.0
    for subset in subsets:
        values = [v for row in rows if row.get("subset") == subset for v in [value(row, metric)] if v is not None]
        item = stats_for(values)
        entries.append((subset, values, item))
        for candidate in [item.maximum, (item.mean or 0.0) + (item.std or 0.0)]:
            if candidate is not None:
                max_y = max(max_y, candidate)
    y_max = nice_upper(max_y)

    def sy(v: float) -> float:
        return y0 + plot_h - (v / y_max) * plot_h

    parts.append(text(x0 + plot_w / 2, y0 - 22, title, size=16, anchor="middle", weight="700"))
    parts.extend(axis_grid(x0, y0, plot_w, plot_h, 0.0, y_max, ylabel, title_inside=False))
    category_w = plot_w / max(1, len(entries))
    bar_w = min(46.0, category_w * 0.42)
    for idx, (subset, values, item) in enumerate(entries):
        cx = x0 + category_w * (idx + 0.5)
        color = SUBSET_COLORS.get(subset, "#607d8b")
        if item.mean is not None:
            top = sy(item.mean)
            parts.append(
                f'<rect x="{cx - bar_w / 2:.2f}" y="{top:.2f}" width="{bar_w:.2f}" '
                f'height="{y0 + plot_h - top:.2f}" fill="{color}" opacity="0.88"/>'
            )
            if item.std is not None:
                std_lo = max(0.0, item.mean - item.std)
                std_hi = item.mean + item.std
                parts.append(line(cx, sy(std_lo), cx, sy(std_hi), "#111827", 1.7))
                parts.append(line(cx - 8, sy(std_hi), cx + 8, sy(std_hi), "#111827", 1.7))
                parts.append(line(cx - 8, sy(std_lo), cx + 8, sy(std_lo), "#111827", 1.7))
            if item.minimum is not None and item.maximum is not None:
                x = cx + bar_w * 0.72
                parts.append(line(x, sy(item.minimum), x, sy(item.maximum), "#4b5563", 1.4, dash="4 3"))
                parts.append(line(x - 6, sy(item.minimum), x + 6, sy(item.minimum), "#4b5563", 1.4))
                parts.append(line(x - 6, sy(item.maximum), x + 6, sy(item.maximum), "#4b5563", 1.4))
            parts.append(text(cx, top - 8, f"n={item.count}", size=10, anchor="middle", color="#374151"))
        else:
            parts.append(text(cx, sy(0.0) - 8, "n/a", size=10, anchor="middle", color="#374151"))
        short = CORE_SUBSET_LABELS.get(subset, subset).replace(" ", "\n")
        label_lines(parts, short.replace("\n", " "), cx, y0 + plot_h + 24, size=9)


def write_overview_dashboard(figures_dir: Path, rows: list[dict[str, str]], manifest: list[dict[str, str]]) -> None:
    subsets = ordered_subsets(manifest)
    counts = Counter(row.get("subset", "") for row in manifest)
    panels = [
        ("Included reports", "count", [{"subset": s, "value": float(counts[s]), "count": counts[s]} for s in subsets]),
        ("Mean duration", "s", mean_rows(rows, subsets, "duration_s")),
        ("Mean path length", "m", mean_rows(rows, subsets, "estimated_path_length_m")),
        ("Mean minimum clearance", "m", mean_rows(rows, subsets, "min_clearance_m")),
    ]
    write_dashboard_svg(figures_dir / "dataset_statistics_overview.svg", panels)


def mean_rows(rows: list[dict[str, str]], subsets: list[str], metric: str) -> list[dict[str, Any]]:
    out = []
    for subset in subsets:
        values = [v for row in rows if row.get("subset") == subset for v in [value(row, metric)] if v is not None]
        out.append({"subset": subset, "value": stats_for(values).mean or 0.0, "count": len(values)})
    return out


def write_simple_bar_chart(
    path: Path,
    title: str,
    ylabel: str,
    rows: list[dict[str, Any]],
    fixed_y_max: float | None = None,
) -> None:
    width = 1040
    height = 500
    pad_l = 86
    pad_r = 34
    pad_t = 66
    pad_b = 118
    plot_w = width - pad_l - pad_r
    plot_h = height - pad_t - pad_b
    max_value = max([float(row["value"]) for row in rows] + [1.0])
    y_max = fixed_y_max or nice_upper(max_value)

    def sy(v: float) -> float:
        return pad_t + plot_h - (v / y_max) * plot_h

    parts = svg_start(width, height, title=title)
    parts.extend(axis_grid(pad_l, pad_t, plot_w, plot_h, 0.0, y_max, ylabel))
    category_w = plot_w / max(1, len(rows))
    bar_w = min(86.0, category_w * 0.5)
    for idx, row in enumerate(rows):
        subset = row["subset"]
        display_label = SUBSET_LABELS.get(subset, subset)
        cx = pad_l + category_w * (idx + 0.5)
        v = float(row["value"])
        top = sy(v)
        color = SUBSET_COLORS.get(subset, "#607d8b")
        parts.append(
            f'<rect x="{cx - bar_w / 2:.2f}" y="{top:.2f}" width="{bar_w:.2f}" '
            f'height="{pad_t + plot_h - top:.2f}" fill="{color}" opacity="0.88"/>'
        )
        parts.append(text(cx, top - 9, format_number(v), size=12, anchor="middle", color="#111827"))
        if row.get("count") is not None:
            parts.append(text(cx, top - 25, f"n={row['count']}", size=11, anchor="middle", color="#4b5563"))
        label_lines(parts, display_label, cx, height - 78)
    parts.append("</svg>\n")
    path.write_text("\n".join(parts), encoding="utf-8")


def write_dual_panel_bar_chart(
    path: Path,
    title: str,
    left: tuple[str, str, list[dict[str, Any]], str],
    right: tuple[str, str, list[dict[str, Any]], str],
) -> None:
    width = 1120
    height = 520
    parts = svg_start(width, height, title=title)
    panel_w = 470
    panel_h = 310
    draw_panel_bars(parts, 76, 86, panel_w, panel_h, *left)
    draw_panel_bars(parts, 614, 86, panel_w, panel_h, *right)
    parts.append("</svg>\n")
    path.write_text("\n".join(parts), encoding="utf-8")


def draw_panel_bars(
    parts: list[str],
    x0: float,
    y0: float,
    plot_w: float,
    plot_h: float,
    title: str,
    ylabel: str,
    rows: list[dict[str, Any]],
    color: str,
) -> None:
    max_value = max([float(row["value"]) for row in rows] + [1.0])
    y_max = nice_upper(max_value)

    def sy(v: float) -> float:
        return y0 + plot_h - (v / y_max) * plot_h

    parts.append(text(x0 + plot_w / 2, y0 - 22, title, size=15, anchor="middle", weight="700"))
    parts.extend(axis_grid(x0, y0, plot_w, plot_h, 0.0, y_max, ylabel, title_inside=False))
    category_w = plot_w / max(1, len(rows))
    bar_w = min(58.0, category_w * 0.48)
    for idx, row in enumerate(rows):
        cx = x0 + category_w * (idx + 0.5)
        v = float(row["value"])
        top = sy(v)
        parts.append(
            f'<rect x="{cx - bar_w / 2:.2f}" y="{top:.2f}" width="{bar_w:.2f}" '
            f'height="{y0 + plot_h - top:.2f}" fill="{color}" opacity="0.88"/>'
        )
        parts.append(text(cx, top - 8, format_number(v), size=11, anchor="middle"))
        label_lines(parts, str(row["subset"]), cx, y0 + plot_h + 26, size=10)


def write_dashboard_svg(path: Path, panels: list[tuple[str, str, list[dict[str, Any]]]]) -> None:
    width = 1160
    height = 760
    parts = svg_start(width, height, title="Dataset statistics overview")
    positions = [(70, 86), (620, 86), (70, 420), (620, 420)]
    for (title, ylabel, rows), (x0, y0) in zip(panels, positions):
        panel_rows = [
            {
                **row,
                "subset": SUBSET_LABELS.get(str(row["subset"]), str(row["subset"])),
            }
            for row in rows
        ]
        draw_panel_bars(parts, x0, y0, 460, 230, title, ylabel, panel_rows, "#2f6f9f")
    parts.append("</svg>\n")
    path.write_text("\n".join(parts), encoding="utf-8")


def axis_grid(
    x0: float,
    y0: float,
    plot_w: float,
    plot_h: float,
    y_min: float,
    y_max: float,
    ylabel: str,
    *,
    title_inside: bool = True,
) -> list[str]:
    parts = []
    for tick in ticks(y_min, y_max, 5):
        y = y0 + plot_h - ((tick - y_min) / max(y_max - y_min, 1e-9)) * plot_h
        parts.append(line(x0, y, x0 + plot_w, y, "#d9dee5", 0.8))
        parts.append(text(x0 - 10, y + 4, format_number(tick), size=11, anchor="end", color="#4b5563"))
    parts.append(line(x0, y0, x0, y0 + plot_h, "#1f2933", 1.2))
    parts.append(line(x0, y0 + plot_h, x0 + plot_w, y0 + plot_h, "#1f2933", 1.2))
    if title_inside:
        parts.append(text(22, y0 + plot_h / 2, ylabel, size=13, anchor="middle", rotate=-90))
    else:
        parts.append(text(x0 - 54, y0 + plot_h / 2, ylabel, size=12, anchor="middle", rotate=-90))
    return parts


def svg_start(width: int, height: int, *, title: str) -> list[str]:
    return [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        "<style>",
        "text{font-family:Arial,Helvetica,sans-serif;fill:#111827}",
        ".title{font-size:20px;font-weight:700}",
        "</style>",
        f'<rect width="{width}" height="{height}" fill="#ffffff"/>',
        text(width / 2, 34, title, size=20, anchor="middle", weight="700"),
    ]


def legend(items: list[tuple[str, str]], x: float, y: float) -> str:
    parts = [f'<g transform="translate({x:.2f},{y:.2f})">']
    for idx, (color, label) in enumerate(items):
        yy = idx * 22
        parts.append(f'<rect x="0" y="{yy:.2f}" width="13" height="13" fill="{color}" opacity="0.88"/>')
        parts.append(text(20, yy + 11, label, size=12, anchor="start"))
    parts.append("</g>")
    return "\n".join(parts)


def line(x1: float, y1: float, x2: float, y2: float, color: str, width: float, dash: str | None = None) -> str:
    dash_attr = f' stroke-dasharray="{dash}"' if dash else ""
    return f'<line x1="{x1:.2f}" y1="{y1:.2f}" x2="{x2:.2f}" y2="{y2:.2f}" stroke="{color}" stroke-width="{width:.2f}"{dash_attr}/>'


def circle(x: float, y: float, r: float, color: str, opacity: float = 1.0) -> str:
    return f'<circle cx="{x:.2f}" cy="{y:.2f}" r="{r:.2f}" fill="{color}" opacity="{opacity:.2f}"/>'


def text(
    x: float,
    y: float,
    label: str,
    *,
    size: int = 12,
    anchor: str = "start",
    color: str = "#111827",
    weight: str = "400",
    rotate: float | None = None,
) -> str:
    transform = f' transform="rotate({rotate:.1f} {x:.2f} {y:.2f})"' if rotate is not None else ""
    return (
        f'<text x="{x:.2f}" y="{y:.2f}" font-size="{size}" text-anchor="{anchor}" '
        f'fill="{color}" font-weight="{weight}"{transform}>{html.escape(label)}</text>'
    )


def label_lines(parts: list[str], label: str, x: float, y: float, *, size: int = 11) -> None:
    words = label.replace("_", " ").split()
    if len(words) <= 2:
        parts.append(text(x, y, " ".join(words), size=size, anchor="middle", color="#374151"))
        return
    mid = math.ceil(len(words) / 2)
    parts.append(text(x, y, " ".join(words[:mid]), size=size, anchor="middle", color="#374151"))
    parts.append(text(x, y + size + 3, " ".join(words[mid:]), size=size, anchor="middle", color="#374151"))


def ticks(min_value: float, max_value: float, count: int) -> list[float]:
    if count <= 1:
        return [min_value, max_value]
    return [min_value + (max_value - min_value) * idx / (count - 1) for idx in range(count)]


def nice_upper(value: float) -> float:
    if value <= 0:
        return 1.0
    exponent = math.floor(math.log10(value))
    base = 10 ** exponent
    scaled = value / base
    if scaled <= 1.2:
        nice = 1.5
    elif scaled <= 2.0:
        nice = 2.5
    elif scaled <= 5.0:
        nice = 5.0
    else:
        nice = 10.0
    return nice * base


def format_number(value: float) -> str:
    if abs(value) >= 100:
        return f"{value:.0f}"
    if abs(value) >= 10:
        return f"{value:.1f}"
    if abs(value) >= 1:
        return f"{value:.2f}".rstrip("0").rstrip(".")
    return f"{value:.3f}".rstrip("0").rstrip(".")


def format_cell(item: Any) -> Any:
    if item is None:
        return ""
    if isinstance(item, float):
        return f"{item:.6f}".rstrip("0").rstrip(".")
    return item


def short_run(run_id: str) -> str:
    parts = run_id.split("_")
    if len(parts) >= 3:
        return "_".join(parts[-3:])
    return run_id


def update_dataset_readme(dataset_dir: Path) -> None:
    readme = dataset_dir / "README.md"
    text_value = readme.read_text(encoding="utf-8")
    block = """\n## Figures And Statistics\n\nGenerate summary figures with:\n\n```bash\npython3 -m data_analisys.plot_thesis_validation_dataset\n```\n\nOutputs are written to `figures/` as SVG files and include success rate, archive\nstatus distribution, duration/path/clearance statistics, planner timing, and\nmixed-mode activity. Machine-readable aggregate statistics are saved in\n`stats_summary.csv` and `stats_summary.json`.\n"""
    if "## Figures And Statistics" not in text_value:
        text_value = text_value.rstrip() + "\n" + block
        readme.write_text(text_value, encoding="utf-8")


if __name__ == "__main__":
    raise SystemExit(main())
