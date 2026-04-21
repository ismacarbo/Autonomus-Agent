from __future__ import annotations

import csv
import json
import math
import re
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Iterable


RUN_SUFFIX_RE = re.compile(r"(20\d{6}_\d{6}(?:_\d{3})?)")


@dataclass(frozen=True)
class RunReport:
    path: Path
    run_id: str
    history: list[dict[str, Any]]
    performance: dict[str, Any]
    status: Any
    frame: dict[str, Any]
    scene: dict[str, Any]
    raw_summary: dict[str, Any]


@dataclass(frozen=True)
class RunSummary:
    run_id: str
    report_path: str
    samples: int
    duration_s: float | None
    first_reference_s: float | None
    first_gate_completion_s: float | None
    reference_windows: int
    longest_reference_window_s: float | None
    final_controller_status_flags: int | None
    final_controller_error_code: int | None
    min_lidar_m: float | None
    min_front_lidar_m: float | None
    max_candidate_gates: float | None
    max_visible_gates: float | None
    avg_planning_ms: float | None
    max_planning_ms: float | None
    avg_lidar_ms: float | None
    max_lidar_ms: float | None
    avg_step_ms: float | None
    max_step_ms: float | None


def load_report(path: str | Path) -> RunReport:
    report_path = Path(path)
    with report_path.open("r", encoding="utf-8") as handle:
        raw = json.load(handle)

    history = raw.get("history") or raw.get("telemetry") or []
    if not isinstance(history, list):
        raise ValueError(f"{report_path} has an invalid history field")

    return RunReport(
        path=report_path,
        run_id=run_id_from_path(report_path),
        history=history,
        performance=raw.get("performance") or {},
        status=raw.get("status"),
        frame=raw.get("frame") or {},
        scene=raw.get("scene") or {},
        raw_summary=raw.get("summary") or {},
    )


def run_id_from_path(path: str | Path) -> str:
    name = Path(path).stem
    match = RUN_SUFFIX_RE.search(name)
    return match.group(1) if match else name


def discover_reports_from_status_doc(
    status_doc: str | Path,
    reports_dir: str | Path,
) -> tuple[list[Path], list[str]]:
    """Find JSON reports referenced by run suffixes in a status markdown file."""
    doc_path = Path(status_doc)
    text = doc_path.read_text(encoding="utf-8")
    reports_root = Path(reports_dir)

    suffixes: list[str] = []
    for match in RUN_SUFFIX_RE.finditer(text):
        suffix = match.group(1)
        if suffix not in suffixes:
            suffixes.append(suffix)

    found: list[Path] = []
    missing: list[str] = []
    for suffix in suffixes:
        matches = sorted(reports_root.glob(f"*{suffix}.json"))
        if matches:
            found.extend(matches)
        else:
            missing.append(suffix)

    deduped: list[Path] = []
    seen: set[Path] = set()
    for path in found:
        resolved = path.resolve()
        if resolved not in seen:
            deduped.append(path)
            seen.add(resolved)
    return deduped, missing


def as_float(value: Any) -> float | None:
    if isinstance(value, bool):
        return 1.0 if value else 0.0
    if isinstance(value, (int, float)):
        numeric = float(value)
    elif isinstance(value, str):
        try:
            numeric = float(value)
        except ValueError:
            return None
    else:
        return None

    if math.isnan(numeric) or math.isinf(numeric):
        return None
    return numeric


def as_int(value: Any) -> int | None:
    numeric = as_float(value)
    return int(numeric) if numeric is not None else None


def time_range(report: RunReport) -> tuple[float | None, float | None]:
    times = [t for t in (as_float(sample.get("time")) for sample in report.history) if t is not None]
    if not times:
        return None, None
    return min(times), max(times)


def numeric_values(
    report: RunReport,
    key: str,
    *,
    valid_nonnegative: bool = False,
) -> list[float]:
    values: list[float] = []
    for sample in report.history:
        value = as_float(sample.get(key))
        if value is None:
            continue
        if valid_nonnegative and value < 0:
            continue
        values.append(value)
    return values


def time_series(
    report: RunReport,
    key: str,
    *,
    valid_nonnegative: bool = False,
) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    for sample in report.history:
        time = as_float(sample.get("time"))
        value = as_float(sample.get(key))
        if time is None or value is None:
            continue
        if valid_nonnegative and value < 0:
            continue
        points.append((time, value))
    return points


def xy_series(report: RunReport) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    for sample in report.history:
        x_value = as_float(sample.get("position_x"))
        y_value = as_float(sample.get("position_y"))
        if x_value is not None and y_value is not None:
            points.append((x_value, y_value))
    return points


def reference_windows(report: RunReport, key: str = "planner_has_reference") -> list[tuple[float, float]]:
    windows: list[tuple[float, float]] = []
    start: float | None = None
    last_time: float | None = None

    for sample in report.history:
        time = as_float(sample.get("time"))
        if time is None:
            continue
        flag = (as_float(sample.get(key)) or 0.0) > 0.5
        if flag and start is None:
            start = time
        elif not flag and start is not None:
            windows.append((start, last_time if last_time is not None else time))
            start = None
        last_time = time

    if start is not None and last_time is not None:
        windows.append((start, last_time))
    return windows


def first_true_time(report: RunReport, key: str) -> float | None:
    for sample in report.history:
        time = as_float(sample.get("time"))
        value = as_float(sample.get(key))
        if time is not None and value is not None and value > 0.5:
            return time
    return None


def first_gate_completion_time(report: RunReport, tolerance: float = 1e-6) -> float | None:
    for sample in report.history:
        time = as_float(sample.get("time"))
        distance = as_float(sample.get("distance_to_goal"))
        if time is None or distance is None:
            continue
        if 0.0 <= distance <= tolerance:
            return time
    return None


def summarize_report(report: RunReport) -> RunSummary:
    start, end = time_range(report)
    windows = reference_windows(report)
    final = report.history[-1] if report.history else {}
    performance = report.performance

    return RunSummary(
        run_id=report.run_id,
        report_path=str(report.path),
        samples=len(report.history),
        duration_s=(end - start) if start is not None and end is not None else None,
        first_reference_s=first_true_time(report, "planner_has_reference"),
        first_gate_completion_s=first_gate_completion_time(report),
        reference_windows=len(windows),
        longest_reference_window_s=max((end - start for start, end in windows), default=None),
        final_controller_status_flags=as_int(final.get("controller_status_flags")),
        final_controller_error_code=as_int(final.get("controller_error_code")),
        min_lidar_m=_min_or_none(numeric_values(report, "min_lidar", valid_nonnegative=True)),
        min_front_lidar_m=_min_or_none(numeric_values(report, "front_lidar", valid_nonnegative=True)),
        max_candidate_gates=_max_or_none(numeric_values(report, "candidate_gates", valid_nonnegative=True)),
        max_visible_gates=_max_or_none(numeric_values(report, "visible_gates", valid_nonnegative=True)),
        avg_planning_ms=_nested_metric(performance, "planning_ms", "avg"),
        max_planning_ms=_nested_metric(performance, "planning_ms", "max"),
        avg_lidar_ms=_nested_metric(performance, "lidar_ms", "avg"),
        max_lidar_ms=_nested_metric(performance, "lidar_ms", "max"),
        avg_step_ms=_nested_metric(performance, "step_ms", "avg"),
        max_step_ms=_nested_metric(performance, "step_ms", "max"),
    )


def write_summaries_csv(path: str | Path, summaries: Iterable[RunSummary]) -> None:
    rows = [asdict(summary) for summary in summaries]
    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        output_path.write_text("", encoding="utf-8")
        return

    with output_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def write_summaries_json(path: str | Path, summaries: Iterable[RunSummary]) -> None:
    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    rows = [asdict(summary) for summary in summaries]
    output_path.write_text(json.dumps(rows, indent=2), encoding="utf-8")


def _nested_metric(performance: dict[str, Any], family: str, metric: str) -> float | None:
    value = performance.get(family)
    if not isinstance(value, dict):
        return None
    return as_float(value.get(metric))


def _min_or_none(values: list[float]) -> float | None:
    return min(values) if values else None


def _max_or_none(values: list[float]) -> float | None:
    return max(values) if values else None
