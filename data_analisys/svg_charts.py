from __future__ import annotations

import html
import math
from pathlib import Path
from typing import Iterable


DEFAULT_COLORS = [
    "#1f77b4",
    "#d62728",
    "#2ca02c",
    "#9467bd",
    "#ff7f0e",
    "#17becf",
    "#4c566a",
]


def write_time_series_chart(
    path: str | Path,
    *,
    title: str,
    x_label: str,
    y_label: str,
    series: list[dict[str, object]],
    markers: list[dict[str, object]] | None = None,
    width: int = 980,
    height: int = 440,
) -> None:
    prepared = []
    for index, item in enumerate(series):
        points = _valid_points(item.get("points", []))
        if not points:
            continue
        prepared.append(
            {
                "label": str(item.get("label", f"series {index + 1}")),
                "points": points,
                "color": str(item.get("color", DEFAULT_COLORS[index % len(DEFAULT_COLORS)])),
            }
        )

    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    if not prepared:
        output_path.write_text(_empty_svg(title, width, height), encoding="utf-8")
        return

    all_x = [x for item in prepared for x, _ in item["points"]]
    all_y = [y for item in prepared for _, y in item["points"]]
    x_min, x_max = _padded_range(min(all_x), max(all_x), pad_fraction=0.02)
    y_min, y_max = _padded_range(min(all_y), max(all_y), pad_fraction=0.08)

    margin_left = 78
    margin_right = 28
    margin_top = 56
    margin_bottom = 64
    plot_w = width - margin_left - margin_right
    plot_h = height - margin_top - margin_bottom

    def sx(x_value: float) -> float:
        return margin_left + (x_value - x_min) / (x_max - x_min) * plot_w

    def sy(y_value: float) -> float:
        return margin_top + plot_h - (y_value - y_min) / (y_max - y_min) * plot_h

    svg: list[str] = [_svg_header(width, height), _style()]
    svg.append(f'<text class="title" x="{width / 2:.1f}" y="28" text-anchor="middle">{_esc(title)}</text>')
    svg.extend(_grid_and_axes(margin_left, margin_top, plot_w, plot_h, x_min, x_max, y_min, y_max, sx, sy))

    for item in prepared:
        points_attr = " ".join(f"{sx(x):.2f},{sy(y):.2f}" for x, y in item["points"])
        svg.append(
            f'<polyline class="line" points="{points_attr}" stroke="{item["color"]}" />'
        )

    for marker in markers or []:
        value = _as_float(marker.get("x"))
        if value is None or value < x_min or value > x_max:
            continue
        x_pos = sx(value)
        label = str(marker.get("label", "marker"))
        color = str(marker.get("color", "#111827"))
        svg.append(f'<line class="marker" x1="{x_pos:.2f}" y1="{margin_top}" x2="{x_pos:.2f}" y2="{margin_top + plot_h}" stroke="{color}" />')
        svg.append(f'<text class="marker-label" x="{x_pos + 5:.2f}" y="{margin_top + 14}" fill="{color}">{_esc(label)}</text>')

    svg.append(f'<text class="axis-label" x="{width / 2:.1f}" y="{height - 18}" text-anchor="middle">{_esc(x_label)}</text>')
    svg.append(f'<text class="axis-label" transform="translate(18 {height / 2:.1f}) rotate(-90)" text-anchor="middle">{_esc(y_label)}</text>')
    svg.extend(_legend(prepared, width - margin_right - 190, margin_top + 8))
    svg.append("</svg>\n")
    output_path.write_text("\n".join(svg), encoding="utf-8")


def write_xy_chart(
    path: str | Path,
    *,
    title: str,
    points: Iterable[tuple[float, float]],
    width: int = 720,
    height: int = 640,
) -> None:
    pts = _valid_points(points)
    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    if not pts:
        output_path.write_text(_empty_svg(title, width, height), encoding="utf-8")
        return

    x_min, x_max = _padded_range(min(x for x, _ in pts), max(x for x, _ in pts), pad_fraction=0.08)
    y_min, y_max = _padded_range(min(y for _, y in pts), max(y for _, y in pts), pad_fraction=0.08)
    margin_left = 72
    margin_right = 32
    margin_top = 56
    margin_bottom = 62
    plot_w = width - margin_left - margin_right
    plot_h = height - margin_top - margin_bottom

    def sx(x_value: float) -> float:
        return margin_left + (x_value - x_min) / (x_max - x_min) * plot_w

    def sy(y_value: float) -> float:
        return margin_top + plot_h - (y_value - y_min) / (y_max - y_min) * plot_h

    points_attr = " ".join(f"{sx(x):.2f},{sy(y):.2f}" for x, y in pts)
    start_x, start_y = pts[0]
    end_x, end_y = pts[-1]

    svg = [_svg_header(width, height), _style()]
    svg.append(f'<text class="title" x="{width / 2:.1f}" y="28" text-anchor="middle">{_esc(title)}</text>')
    svg.extend(_grid_and_axes(margin_left, margin_top, plot_w, plot_h, x_min, x_max, y_min, y_max, sx, sy))
    svg.append(f'<polyline class="line" points="{points_attr}" stroke="#1f77b4" />')
    svg.append(f'<circle cx="{sx(start_x):.2f}" cy="{sy(start_y):.2f}" r="5" fill="#2ca02c" />')
    svg.append(f'<circle cx="{sx(end_x):.2f}" cy="{sy(end_y):.2f}" r="5" fill="#d62728" />')
    svg.append(f'<text class="axis-label" x="{width / 2:.1f}" y="{height - 18}" text-anchor="middle">x [m]</text>')
    svg.append(f'<text class="axis-label" transform="translate(18 {height / 2:.1f}) rotate(-90)" text-anchor="middle">y [m]</text>')
    svg.extend(_legend([
        {"label": "trajectory", "color": "#1f77b4"},
        {"label": "start", "color": "#2ca02c"},
        {"label": "end", "color": "#d62728"},
    ], width - margin_right - 170, margin_top + 8))
    svg.append("</svg>\n")
    output_path.write_text("\n".join(svg), encoding="utf-8")


def write_grouped_bar_chart(
    path: str | Path,
    *,
    title: str,
    categories: list[str],
    groups: list[dict[str, object]],
    y_label: str,
    width: int = 980,
    height: int = 460,
) -> None:
    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    values = []
    for group in groups:
        values.extend(_as_float(value) for value in group.get("values", []))
    values = [value for value in values if value is not None]
    if not categories or not groups or not values:
        output_path.write_text(_empty_svg(title, width, height), encoding="utf-8")
        return

    y_min = 0.0
    _, y_max = _padded_range(0.0, max(values), pad_fraction=0.12)
    margin_left = 78
    margin_right = 32
    margin_top = 58
    margin_bottom = 96
    plot_w = width - margin_left - margin_right
    plot_h = height - margin_top - margin_bottom
    category_w = plot_w / max(1, len(categories))
    bar_gap = 4
    bar_w = max(4.0, (category_w - 18) / max(1, len(groups)) - bar_gap)

    def sy(y_value: float) -> float:
        return margin_top + plot_h - (y_value - y_min) / (y_max - y_min) * plot_h

    svg = [_svg_header(width, height), _style()]
    svg.append(f'<text class="title" x="{width / 2:.1f}" y="28" text-anchor="middle">{_esc(title)}</text>')
    svg.extend(_horizontal_grid(margin_left, margin_top, plot_w, plot_h, y_min, y_max, sy))
    svg.append(f'<line class="axis" x1="{margin_left}" y1="{margin_top + plot_h}" x2="{margin_left + plot_w}" y2="{margin_top + plot_h}" />')
    svg.append(f'<line class="axis" x1="{margin_left}" y1="{margin_top}" x2="{margin_left}" y2="{margin_top + plot_h}" />')

    for cat_index, category in enumerate(categories):
        base_x = margin_left + cat_index * category_w + 9
        label_x = margin_left + (cat_index + 0.5) * category_w
        svg.append(f'<text class="tick-label" x="{label_x:.2f}" y="{height - 74}" text-anchor="middle" transform="rotate(-28 {label_x:.2f} {height - 74})">{_esc(_short_label(category))}</text>')
        for group_index, group in enumerate(groups):
            group_values = group.get("values", [])
            value = _as_float(group_values[cat_index]) if cat_index < len(group_values) else None
            if value is None:
                continue
            color = str(group.get("color", DEFAULT_COLORS[group_index % len(DEFAULT_COLORS)]))
            x_pos = base_x + group_index * (bar_w + bar_gap)
            y_pos = sy(value)
            svg.append(f'<rect class="bar" x="{x_pos:.2f}" y="{y_pos:.2f}" width="{bar_w:.2f}" height="{margin_top + plot_h - y_pos:.2f}" fill="{color}" />')

    svg.append(f'<text class="axis-label" transform="translate(18 {height / 2:.1f}) rotate(-90)" text-anchor="middle">{_esc(y_label)}</text>')
    svg.extend(_legend(groups, width - margin_right - 220, margin_top + 8))
    svg.append("</svg>\n")
    output_path.write_text("\n".join(svg), encoding="utf-8")


def write_timeline_chart(
    path: str | Path,
    *,
    title: str,
    rows: list[dict[str, object]],
    width: int = 980,
    row_height: int = 44,
) -> None:
    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        output_path.write_text(_empty_svg(title, width, 260), encoding="utf-8")
        return

    height = 112 + row_height * len(rows)
    margin_left = 186
    margin_right = 34
    margin_top = 58
    margin_bottom = 54
    plot_w = width - margin_left - margin_right
    max_time = max((_as_float(row.get("duration_s")) or 0.0) for row in rows)
    if max_time <= 0:
        max_time = 1.0

    def sx(time_s: float) -> float:
        return margin_left + time_s / max_time * plot_w

    svg = [_svg_header(width, height), _style()]
    svg.append(f'<text class="title" x="{width / 2:.1f}" y="28" text-anchor="middle">{_esc(title)}</text>')

    for tick in _ticks(0.0, max_time, 6):
        x_pos = sx(tick)
        svg.append(f'<line class="grid" x1="{x_pos:.2f}" y1="{margin_top - 8}" x2="{x_pos:.2f}" y2="{height - margin_bottom}" />')
        svg.append(f'<text class="tick-label" x="{x_pos:.2f}" y="{height - 24}" text-anchor="middle">{_fmt(tick)}</text>')

    for row_index, row in enumerate(rows):
        y_mid = margin_top + row_index * row_height + row_height / 2
        svg.append(f'<text class="row-label" x="{margin_left - 12}" y="{y_mid + 4:.2f}" text-anchor="end">{_esc(_short_label(str(row.get("label", ""))))}</text>')
        duration = _as_float(row.get("duration_s")) or 0.0
        svg.append(f'<line class="axis-light" x1="{margin_left}" y1="{y_mid:.2f}" x2="{sx(duration):.2f}" y2="{y_mid:.2f}" />')
        for start, end in row.get("windows", []):
            x_start = sx(float(start))
            x_end = sx(float(end))
            svg.append(f'<rect class="timeline-window" x="{x_start:.2f}" y="{y_mid - 9:.2f}" width="{max(1.0, x_end - x_start):.2f}" height="18" />')
        for marker in row.get("markers", []):
            x_value = _as_float(marker.get("x"))
            if x_value is None:
                continue
            color = str(marker.get("color", "#d62728"))
            x_pos = sx(x_value)
            svg.append(f'<line class="marker" x1="{x_pos:.2f}" y1="{y_mid - 15:.2f}" x2="{x_pos:.2f}" y2="{y_mid + 15:.2f}" stroke="{color}" />')

    svg.append(f'<text class="axis-label" x="{margin_left + plot_w / 2:.1f}" y="{height - 6}" text-anchor="middle">time [s]</text>')
    legend_items = [{"label": "planner reference", "color": "#1f77b4"}]
    if any(row.get("markers") for row in rows):
        legend_items.append({"label": "gate completion", "color": "#d62728"})
    svg.extend(_legend(legend_items, width - margin_right - 230, margin_top - 6))
    svg.append("</svg>\n")
    output_path.write_text("\n".join(svg), encoding="utf-8")


def _svg_header(width: int, height: int) -> str:
    return f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">'


def _style() -> str:
    return """<style>
    svg { background: #ffffff; font-family: Inter, ui-sans-serif, system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif; }
    .title { font-size: 26px; font-weight: 700; fill: #111827; }
    .axis, .axis-light { stroke: #374151; stroke-width: 1.2; }
    .axis-light { stroke: #9ca3af; }
    .grid { stroke: #e5e7eb; stroke-width: 1; }
    .tick-label, .axis-label, .row-label, .marker-label { fill: #4b5563; font-size: 20px; }
    .axis-label { font-weight: 600; }
    .row-label { font-weight: 600; }
    .line { fill: none; stroke-width: 2.2; stroke-linejoin: round; stroke-linecap: round; }
    .marker { stroke-width: 1.4; stroke-dasharray: 5 4; }
    .timeline-window { fill: #1f77b4; opacity: 0.78; rx: 3; }
    .bar { opacity: 0.88; }
    .legend-text { fill: #374151; font-size: 18px; }
    .legend-box { fill: #ffffff; stroke: #e5e7eb; stroke-width: 1; rx: 6; }
    </style>"""


def _grid_and_axes(
    margin_left: int,
    margin_top: int,
    plot_w: int,
    plot_h: int,
    x_min: float,
    x_max: float,
    y_min: float,
    y_max: float,
    sx,
    sy,
) -> list[str]:
    svg: list[str] = []
    svg.extend(_horizontal_grid(margin_left, margin_top, plot_w, plot_h, y_min, y_max, sy))
    for tick in _ticks(x_min, x_max, 6):
        x_pos = sx(tick)
        svg.append(f'<line class="grid" x1="{x_pos:.2f}" y1="{margin_top}" x2="{x_pos:.2f}" y2="{margin_top + plot_h}" />')
        svg.append(f'<text class="tick-label" x="{x_pos:.2f}" y="{margin_top + plot_h + 20}" text-anchor="middle">{_fmt(tick)}</text>')
    svg.append(f'<line class="axis" x1="{margin_left}" y1="{margin_top + plot_h}" x2="{margin_left + plot_w}" y2="{margin_top + plot_h}" />')
    svg.append(f'<line class="axis" x1="{margin_left}" y1="{margin_top}" x2="{margin_left}" y2="{margin_top + plot_h}" />')
    return svg


def _horizontal_grid(
    margin_left: int,
    margin_top: int,
    plot_w: int,
    plot_h: int,
    y_min: float,
    y_max: float,
    sy,
) -> list[str]:
    svg: list[str] = []
    for tick in _ticks(y_min, y_max, 6):
        y_pos = sy(tick)
        svg.append(f'<line class="grid" x1="{margin_left}" y1="{y_pos:.2f}" x2="{margin_left + plot_w}" y2="{y_pos:.2f}" />')
        svg.append(f'<text class="tick-label" x="{margin_left - 8}" y="{y_pos + 4:.2f}" text-anchor="end">{_fmt(tick)}</text>')
    return svg


def _legend(items: Iterable[dict[str, object]], x: float, y: float) -> list[str]:
    prepared = list(items)
    height = 24 + 22 * len(prepared)
    svg = [f'<rect class="legend-box" x="{x}" y="{y}" width="205" height="{height}" />']
    for index, item in enumerate(prepared):
        y_pos = y + 22 + index * 22
        color = str(item.get("color", DEFAULT_COLORS[index % len(DEFAULT_COLORS)]))
        label = str(item.get("label", "series"))
        svg.append(f'<line x1="{x + 12}" y1="{y_pos - 4}" x2="{x + 34}" y2="{y_pos - 4}" stroke="{color}" stroke-width="3" />')
        svg.append(f'<text class="legend-text" x="{x + 42}" y="{y_pos}">{_esc(label)}</text>')
    return svg


def _ticks(min_value: float, max_value: float, count: int) -> list[float]:
    if count <= 1 or min_value == max_value:
        return [min_value]
    return [min_value + (max_value - min_value) * index / (count - 1) for index in range(count)]


def _padded_range(min_value: float, max_value: float, pad_fraction: float) -> tuple[float, float]:
    if min_value == max_value:
        pad = max(1.0, abs(min_value) * 0.2)
        return min_value - pad, max_value + pad
    span = max_value - min_value
    pad = span * pad_fraction
    return min_value - pad, max_value + pad


def _valid_points(points: Iterable[object]) -> list[tuple[float, float]]:
    valid: list[tuple[float, float]] = []
    for point in points:
        try:
            x_value, y_value = point
        except (TypeError, ValueError):
            continue
        x_float = _as_float(x_value)
        y_float = _as_float(y_value)
        if x_float is not None and y_float is not None:
            valid.append((x_float, y_float))
    return valid


def _as_float(value: object) -> float | None:
    try:
        numeric = float(value)
    except (TypeError, ValueError):
        return None
    if math.isnan(numeric) or math.isinf(numeric):
        return None
    return numeric


def _fmt(value: float) -> str:
    if abs(value) >= 100:
        return f"{value:.0f}"
    if abs(value) >= 10:
        return f"{value:.1f}"
    return f"{value:.2f}"


def _esc(value: object) -> str:
    return html.escape(str(value), quote=True)


def _short_label(value: str, max_len: int = 22) -> str:
    if len(value) <= max_len:
        return value
    return value[-max_len:]


def _empty_svg(title: str, width: int, height: int) -> str:
    return "\n".join(
        [
            _svg_header(width, height),
            _style(),
            f'<text class="title" x="{width / 2:.1f}" y="28" text-anchor="middle">{_esc(title)}</text>',
            f'<text class="axis-label" x="{width / 2:.1f}" y="{height / 2:.1f}" text-anchor="middle">No data available</text>',
            "</svg>\n",
        ]
    )
