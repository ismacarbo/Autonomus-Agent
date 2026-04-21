from __future__ import annotations

import argparse
import csv
import glob
import json
import math
import sys
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from data_analisys.report_analysis import (
    RunReport,
    as_float,
    first_gate_completion_time,
    load_report,
    reference_windows,
    summarize_report,
    time_range,
    time_series,
)
from data_analisys.svg_charts import write_grouped_bar_chart, write_time_series_chart


DEFAULT_SIM_BASELINE = Path("reports/thesis_planner_unstructured_wide_slalom_gui_auto_20260317_032610_597.json")
DEFAULT_ROBOT_RUN = Path("reports/thesis_hardware_unstructured_manual_gate_editor_gui_manual_20260418_181318_932.json")
DEFAULT_OUTPUT_DIR = Path("data_analisys/outputs/unstructured_model_validation")
DEFAULT_AGGREGATE_OUTPUT_DIR = Path("data_analisys/outputs/unstructured_model_validation_aggregate")
DEFAULT_CONFIG = Path("data_analisys/unstructured_validation_config.json")
WINDOW_MODES = ("full", "until_first_gate", "longest_reference", "reference_only", "custom")


@dataclass(frozen=True)
class SignalStats:
    count: int
    mean: float | None
    rms: float | None
    min_value: float | None
    max_value: float | None
    final_value: float | None


@dataclass(frozen=True)
class FitResult:
    signal: str
    input_signal: str
    output_signal: str
    samples: int
    dt_s: float | None
    a: float | None
    b: float | None
    gain: float | None
    tau_s: float | None
    rmse: float | None
    note: str


@dataclass(frozen=True)
class WindowSelection:
    mode: str
    start_s: float | None
    end_s: float | None
    input_samples: int
    output_samples: int
    note: str


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Create an unstructured simulation-vs-robot baseline comparison.",
    )
    parser.add_argument("--sim-baseline", type=Path, default=DEFAULT_SIM_BASELINE)
    parser.add_argument("--robot-run", type=Path, default=DEFAULT_ROBOT_RUN)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--window", choices=WINDOW_MODES, default="full")
    parser.add_argument("--custom-start", type=float, default=None)
    parser.add_argument("--custom-end", type=float, default=None)
    parser.add_argument(
        "--robot-glob",
        default=None,
        help="When set, run aggregate validation for all robot reports matching this glob.",
    )
    parser.add_argument("--aggregate-output-dir", type=Path, default=DEFAULT_AGGREGATE_OUTPUT_DIR)
    parser.add_argument(
        "--grid-size",
        type=int,
        default=250,
        help="Number of normalized-time samples used for shape comparison.",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    apply_config_defaults(args)
    if args.robot_glob:
        robot_runs = [Path(path) for path in sorted(glob.glob(args.robot_glob))]
        run_multi_run_validation(
            args.sim_baseline,
            robot_runs,
            args.aggregate_output_dir,
            args.window,
            args.custom_start,
            args.custom_end,
            args.grid_size,
        )
        print(f"Wrote aggregate validation bundle to {args.aggregate_output_dir}")
        print(f"Simulation baseline: {args.sim_baseline}")
        print(f"Robot glob: {args.robot_glob}")
        print(f"Robot runs: {len(robot_runs)}")
        return 0

    run_model_validation(
        args.sim_baseline,
        args.robot_run,
        args.output_dir,
        args.grid_size,
        args.window,
        args.custom_start,
        args.custom_end,
    )

    print(f"Wrote unstructured model validation bundle to {args.output_dir}")
    print(f"Simulation baseline: {args.sim_baseline}")
    print(f"Robot run: {args.robot_run}")
    return 0


def apply_config_defaults(args: argparse.Namespace) -> None:
    if args.config is None or not args.config.exists():
        return
    config = json.loads(args.config.read_text(encoding="utf-8"))
    if args.sim_baseline == DEFAULT_SIM_BASELINE and config.get("simulation_baseline"):
        args.sim_baseline = Path(config["simulation_baseline"])
    if args.robot_run == DEFAULT_ROBOT_RUN and config.get("robot_validation_run"):
        args.robot_run = Path(config["robot_validation_run"])
    if args.window == "full" and config.get("default_analysis_window"):
        args.window = config["default_analysis_window"]


def run_model_validation(
    sim_baseline: Path,
    robot_run: Path,
    output_dir: Path,
    grid_size: int = 250,
    analysis_window: str = "full",
    custom_start: float | None = None,
    custom_end: float | None = None,
) -> tuple[dict[str, Any], dict[str, Any]]:
    sim_raw = load_report(sim_baseline)
    robot_raw = load_report(robot_run)
    sim, sim_window = select_analysis_window(sim_raw, analysis_window, custom_start, custom_end)
    robot, robot_window = select_analysis_window(robot_raw, analysis_window, custom_start, custom_end)
    output_dir.mkdir(parents=True, exist_ok=True)

    comparison = build_comparison(sim, robot, grid_size, analysis_window, sim_window, robot_window)
    fits = fit_reports(sim, robot)

    write_json(output_dir / "comparison.json", comparison)
    write_json(output_dir / "fit_results.json", fits)
    write_metrics_csv(output_dir / "comparison_metrics.csv", comparison)
    write_fit_csv(output_dir / "fit_results.csv", fits)
    write_manifest(output_dir / "manifest.txt", sim_baseline, robot_run, analysis_window, sim_window, robot_window)
    write_plots(output_dir, sim, robot, comparison)
    return comparison, fits


def run_multi_run_validation(
    sim_baseline: Path,
    robot_runs: list[Path],
    output_dir: Path,
    analysis_window: str = "full",
    custom_start: float | None = None,
    custom_end: float | None = None,
    grid_size: int = 250,
) -> dict[str, Any]:
    sim_raw = load_report(sim_baseline)
    sim, sim_window = select_analysis_window(sim_raw, analysis_window, custom_start, custom_end)
    output_dir.mkdir(parents=True, exist_ok=True)

    rows: list[dict[str, Any]] = []
    results: list[dict[str, Any]] = []
    for robot_path in robot_runs:
        robot_raw = load_report(robot_path)
        robot, robot_window = select_analysis_window(robot_raw, analysis_window, custom_start, custom_end)
        comparison = build_comparison(sim, robot, grid_size, analysis_window, sim_window, robot_window)
        fits = fit_reports(sim, robot)
        errors = comparison["aligned_normalized_time_error"]
        tracking_robot = comparison["tracking_error"]["robot"]
        row = {
            "robot_run_id": robot.run_id,
            "robot_path": str(robot_path),
            "window_mode": analysis_window,
            "robot_window_samples": robot_window.output_samples,
            "robot_window_start_s": robot_window.start_s,
            "robot_window_end_s": robot_window.end_s,
            "robot_summary_duration_s": comparison["robot"]["summary"]["duration_s"],
            "robot_first_gate_completion_s": comparison["robot"]["summary"]["first_gate_completion_s"],
            "speed_rmse_vs_sim": errors["speed"]["rmse"],
            "yaw_rate_rmse_vs_sim": errors["yaw_rate"]["rmse"],
            "distance_norm_rmse_vs_sim": errors["distance_to_goal_norm"]["rmse"],
            "heading_rmse_vs_sim": errors["tracker_heading_error_deg"]["rmse"],
            "planning_rmse_vs_sim": errors["planning_ms"]["rmse"],
            "step_rmse_vs_sim": errors["step_ms"]["rmse"],
            "robot_speed_target_rms": tracking_robot["speed_minus_target_speed"]["rms"],
            "robot_yaw_target_rms": tracking_robot["yaw_rate_minus_target_yaw_rate"]["rms"],
            "robot_speed_fit_tau_s": fits["robot_speed_pwm_fit"]["tau_s"],
            "robot_speed_fit_gain": fits["robot_speed_pwm_fit"]["gain"],
            "robot_speed_fit_rmse": fits["robot_speed_pwm_fit"]["rmse"],
            "robot_yaw_fit_tau_s": fits["robot_yaw_pwm_fit"]["tau_s"],
            "robot_yaw_fit_gain": fits["robot_yaw_pwm_fit"]["gain"],
            "robot_yaw_fit_rmse": fits["robot_yaw_pwm_fit"]["rmse"],
        }
        rows.append(row)
        results.append({"row": row, "comparison": comparison, "fits": fits})

    aggregate = {
        "simulation": {
            "path": str(sim_baseline),
            "run_id": sim.run_id,
            "window": asdict(sim_window),
        },
        "analysis_window": analysis_window,
        "robot_run_count": len(rows),
        "rows": rows,
        "aggregate_stats": aggregate_rows(rows),
    }
    write_json(output_dir / "aggregate_validation.json", aggregate)
    write_aggregate_csv(output_dir / "aggregate_validation.csv", rows)
    write_aggregate_plots(output_dir, rows)
    return aggregate


def fit_reports(sim, robot) -> dict[str, Any]:
    return {
        "sim_speed_pwm_fit": asdict(fit_first_order_response(sim, "speed_from_pwm", pwm_mean_abs_series(sim), signal_series(sim, "speed"))),
        "robot_speed_pwm_fit": asdict(fit_first_order_response(robot, "speed_from_pwm", pwm_mean_abs_series(robot), signal_series(robot, "speed"))),
        "sim_yaw_pwm_fit": asdict(fit_first_order_response(sim, "yaw_rate_from_diff_pwm", pwm_diff_series(sim), yaw_rate_series(sim))),
        "robot_yaw_pwm_fit": asdict(fit_first_order_response(robot, "yaw_rate_from_diff_pwm", pwm_diff_series(robot), yaw_rate_series(robot))),
    }


def select_analysis_window(
    report: RunReport,
    mode: str,
    custom_start: float | None = None,
    custom_end: float | None = None,
) -> tuple[RunReport, WindowSelection]:
    if mode not in WINDOW_MODES:
        raise ValueError(f"Unknown analysis window: {mode}")
    start, end = time_range(report)
    selected_start = start
    selected_end = end
    note = "full report"

    if mode == "until_first_gate":
        completion = first_gate_completion_time(report)
        if completion is not None:
            selected_end = completion
            note = "start to first gate completion"
        else:
            note = "first gate completion not found; using full report"
    elif mode == "longest_reference":
        windows = reference_windows(report)
        if windows:
            selected_start, selected_end = max(windows, key=lambda item: item[1] - item[0])
            note = "longest planner_has_reference window"
        else:
            note = "reference window not found; using full report"
    elif mode == "custom":
        selected_start = custom_start if custom_start is not None else start
        selected_end = custom_end if custom_end is not None else end
        note = "custom time range"

    if mode == "reference_only":
        filtered_history = [
            sample for sample in report.history
            if (as_float(sample.get("planner_has_reference")) or 0.0) > 0.5
        ]
        if filtered_history:
            filtered = report_with_history(report, filtered_history)
            filtered_start, filtered_end = time_range(filtered)
            return filtered, WindowSelection(
                mode,
                filtered_start,
                filtered_end,
                len(report.history),
                len(filtered_history),
                "all samples with planner_has_reference true",
            )
        note = "reference samples not found; using full report"

    filtered_history = filter_history_by_time(report.history, selected_start, selected_end)
    if not filtered_history:
        filtered_history = report.history[:]
        selected_start, selected_end = start, end
        note = f"{note}; selected range was empty, using full report"
    filtered = report_with_history(report, filtered_history)
    actual_start, actual_end = time_range(filtered)
    return filtered, WindowSelection(
        mode,
        actual_start,
        actual_end,
        len(report.history),
        len(filtered_history),
        note,
    )


def filter_history_by_time(
    history: list[dict[str, Any]],
    start: float | None,
    end: float | None,
) -> list[dict[str, Any]]:
    filtered: list[dict[str, Any]] = []
    for sample in history:
        time = as_float(sample.get("time"))
        if time is None:
            continue
        if start is not None and time < start:
            continue
        if end is not None and time > end:
            continue
        filtered.append(sample)
    return filtered


def report_with_history(report: RunReport, history: list[dict[str, Any]]) -> RunReport:
    return RunReport(
        path=report.path,
        run_id=report.run_id,
        history=history,
        performance=report.performance,
        status=report.status,
        frame=report.frame,
        scene=report.scene,
        raw_summary=report.raw_summary,
    )


def build_comparison(
    sim,
    robot,
    grid_size: int,
    analysis_window: str = "full",
    sim_window: WindowSelection | None = None,
    robot_window: WindowSelection | None = None,
) -> dict[str, Any]:
    sim_summary = summarize_report(sim)
    robot_summary = summarize_report(robot)
    signals = [
        "speed",
        "accel",
        "yaw_rate",
        "distance_to_goal_norm",
        "tracker_cross_track",
        "tracker_heading_error_deg",
        "planning_ms",
        "step_ms",
    ]

    sim_signal_stats = {signal: asdict(stats_for_series(resolve_signal(sim, signal))) for signal in signals}
    robot_signal_stats = {signal: asdict(stats_for_series(resolve_signal(robot, signal))) for signal in signals}

    aligned_errors: dict[str, Any] = {}
    for signal in signals:
        sim_points = normalize_time(resolve_signal(sim, signal))
        robot_points = normalize_time(resolve_signal(robot, signal))
        aligned_errors[signal] = aligned_signal_error(sim_points, robot_points, grid_size)

    robot_tracking = {
        "speed_minus_target_speed": asdict(stats_for_series(error_series(robot, "speed", "target_speed"))),
        "yaw_rate_minus_target_yaw_rate": asdict(stats_for_series(error_series(robot, "yaw_rate", "target_yaw_rate"))),
    }
    sim_tracking = {
        "steer_angle_minus_target_steer_angle": asdict(stats_for_series(error_series(sim, "steer_angle", "target_steer_angle"))),
        "tracker_cross_track": asdict(stats_for_series(resolve_signal(sim, "tracker_cross_track"))),
        "tracker_heading_error_deg": asdict(stats_for_series(resolve_signal(sim, "tracker_heading_error_deg"))),
    }

    return {
        "simulation": {
            "path": str(sim.path),
            "run_id": sim.run_id,
            "status": sim.status,
            "summary": asdict(sim_summary),
            "raw_summary": sim.raw_summary,
            "signals": sim_signal_stats,
        },
        "robot": {
            "path": str(robot.path),
            "run_id": robot.run_id,
            "status": robot.status,
            "summary": asdict(robot_summary),
            "signals": robot_signal_stats,
        },
        "aligned_normalized_time_error": aligned_errors,
        "analysis_window": {
            "mode": analysis_window,
            "simulation": asdict(sim_window) if sim_window is not None else None,
            "robot": asdict(robot_window) if robot_window is not None else None,
        },
        "tracking_error": {
            "simulation": sim_tracking,
            "robot": robot_tracking,
        },
        "notes": [
            "The first baseline compares normalized signal shapes because the simulated and physical unstructured maps are not the same environment.",
            "distance_to_goal_norm is normalized by each run initial valid distance_to_goal value.",
            "The PWM fits are first-order discrete approximations intended as fitting scaffolding, not final vehicle identification.",
        ],
    }


def signal_series(report, key: str) -> list[tuple[float, float]]:
    if key == "yaw_rate":
        return yaw_rate_series(report)
    return time_series(report, key)


def resolve_signal(report, signal: str) -> list[tuple[float, float]]:
    if signal == "yaw_rate":
        return yaw_rate_series(report)
    if signal == "distance_to_goal_norm":
        return normalized_distance_to_goal(report)
    return time_series(report, signal)


def yaw_rate_series(report) -> list[tuple[float, float]]:
    direct = time_series(report, "yaw_rate")
    if direct:
        return direct
    points: list[tuple[float, float]] = []
    for sample in report.history:
        time = as_float(sample.get("time"))
        speed = as_float(sample.get("speed"))
        curvature = as_float(sample.get("curvature"))
        if time is not None and speed is not None and curvature is not None:
            points.append((time, speed * curvature))
    return points


def normalized_distance_to_goal(report) -> list[tuple[float, float]]:
    points = [(t, v) for t, v in time_series(report, "distance_to_goal") if v >= 0]
    if not points:
        return []
    initial = next((value for _, value in points if value > 1e-9), None)
    if initial is None:
        return [(time, 0.0) for time, _ in points]
    return [(time, value / initial) for time, value in points]


def pwm_mean_abs_series(report) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    for sample in report.history:
        time = as_float(sample.get("time"))
        left = first_available(sample, ["controller_pwm_left", "pwm_left", "left_pwm"])
        right = first_available(sample, ["controller_pwm_right", "pwm_right", "right_pwm"])
        if time is not None and left is not None and right is not None:
            points.append((time, 0.5 * (abs(left) + abs(right))))
    return points


def pwm_diff_series(report) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    for sample in report.history:
        time = as_float(sample.get("time"))
        left = first_available(sample, ["controller_pwm_left", "pwm_left", "left_pwm"])
        right = first_available(sample, ["controller_pwm_right", "pwm_right", "right_pwm"])
        if time is not None and left is not None and right is not None:
            points.append((time, 0.5 * (right - left)))
    return points


def first_available(sample: dict[str, Any], keys: list[str]) -> float | None:
    for key in keys:
        value = as_float(sample.get(key))
        if value is not None:
            return value
    return None


def error_series(report, measured_key: str, target_key: str) -> list[tuple[float, float]]:
    target = dict(time_series(report, target_key))
    errors: list[tuple[float, float]] = []
    for time, measured in signal_series(report, measured_key):
        if time in target:
            errors.append((time, measured - target[time]))
    return errors


def stats_for_series(points: list[tuple[float, float]]) -> SignalStats:
    values = [value for _, value in points if math.isfinite(value)]
    if not values:
        return SignalStats(0, None, None, None, None, None)
    mean = sum(values) / len(values)
    rms = math.sqrt(sum(value * value for value in values) / len(values))
    return SignalStats(len(values), mean, rms, min(values), max(values), values[-1])


def normalize_time(points: list[tuple[float, float]]) -> list[tuple[float, float]]:
    if not points:
        return []
    start = points[0][0]
    end = points[-1][0]
    if end == start:
        return [(0.0, value) for _, value in points]
    return [((time - start) / (end - start), value) for time, value in points]


def aligned_signal_error(
    reference_points: list[tuple[float, float]],
    measured_points: list[tuple[float, float]],
    grid_size: int,
) -> dict[str, Any]:
    if len(reference_points) < 2 or len(measured_points) < 2:
        return {"count": 0, "bias": None, "rmse": None, "max_abs": None}
    errors: list[float] = []
    for index in range(max(2, grid_size)):
        x = index / (max(2, grid_size) - 1)
        ref = interpolate(reference_points, x)
        measured = interpolate(measured_points, x)
        if ref is not None and measured is not None:
            errors.append(measured - ref)
    if not errors:
        return {"count": 0, "bias": None, "rmse": None, "max_abs": None}
    bias = sum(errors) / len(errors)
    rmse = math.sqrt(sum(error * error for error in errors) / len(errors))
    max_abs = max(abs(error) for error in errors)
    return {"count": len(errors), "bias": bias, "rmse": rmse, "max_abs": max_abs}


def interpolate(points: list[tuple[float, float]], x: float) -> float | None:
    if not points:
        return None
    if x <= points[0][0]:
        return points[0][1]
    if x >= points[-1][0]:
        return points[-1][1]
    cursor = 1
    while cursor < len(points) and points[cursor][0] < x:
        cursor += 1
    x0, y0 = points[cursor - 1]
    x1, y1 = points[cursor]
    if x1 == x0:
        return y1
    alpha = (x - x0) / (x1 - x0)
    return y0 + alpha * (y1 - y0)


def fit_first_order_response(
    report,
    signal_name: str,
    input_points: list[tuple[float, float]],
    output_points: list[tuple[float, float]],
) -> FitResult:
    aligned = align_on_times(input_points, output_points)
    if len(aligned) < 4:
        return FitResult(signal_name, "pwm", "output", len(aligned), None, None, None, None, None, None, "not enough samples")

    times = [row[0] for row in aligned]
    inputs = [row[1] for row in aligned]
    outputs = [row[2] for row in aligned]
    x1 = outputs[:-1]
    x2 = inputs[:-1]
    y = outputs[1:]
    n = len(y)
    sx1 = sum(x1)
    sx2 = sum(x2)
    sy = sum(y)
    sx11 = sum(value * value for value in x1)
    sx22 = sum(value * value for value in x2)
    sx12 = sum(a * b for a, b in zip(x1, x2))
    sx1y = sum(a * b for a, b in zip(x1, y))
    sx2y = sum(a * b for a, b in zip(x2, y))

    # Least squares for y[k+1] = a*y[k] + b*u[k] + c.
    matrix = [
        [sx11, sx12, sx1],
        [sx12, sx22, sx2],
        [sx1, sx2, float(n)],
    ]
    vector = [sx1y, sx2y, sy]
    solved = solve_3x3(matrix, vector)
    if solved is None:
        return FitResult(signal_name, "pwm", "output", len(aligned), median_dt(times), None, None, None, None, None, "singular least-squares system")
    a, b, c = solved
    predictions = [a * yy + b * uu + c for yy, uu in zip(x1, x2)]
    rmse = math.sqrt(sum((pred - real) ** 2 for pred, real in zip(predictions, y)) / len(y))
    dt = median_dt(times)
    gain = b / (1.0 - a) if abs(1.0 - a) > 1e-9 else None
    tau = None
    if dt is not None and 0.0 < a < 1.0:
        tau = -dt / math.log(a)
    note = "first-order discrete fit y[k+1] = a*y[k] + b*u[k] + c"
    return FitResult(signal_name, "pwm", "output", len(aligned), dt, a, b, gain, tau, rmse, note)


def align_on_times(
    input_points: list[tuple[float, float]],
    output_points: list[tuple[float, float]],
) -> list[tuple[float, float, float]]:
    if not input_points or not output_points:
        return []
    input_norm = dict(input_points)
    aligned: list[tuple[float, float, float]] = []
    for time, output in output_points:
        if time in input_norm:
            aligned.append((time, input_norm[time], output))
    if len(aligned) >= 4:
        return aligned
    normalized_input = normalize_time(input_points)
    normalized_output = normalize_time(output_points)
    rebuilt: list[tuple[float, float, float]] = []
    for time, output in normalized_output:
        input_value = interpolate(normalized_input, time)
        if input_value is not None:
            rebuilt.append((time, input_value, output))
    return rebuilt


def median_dt(times: list[float]) -> float | None:
    deltas = [b - a for a, b in zip(times, times[1:]) if b > a]
    if not deltas:
        return None
    deltas.sort()
    mid = len(deltas) // 2
    if len(deltas) % 2:
        return deltas[mid]
    return 0.5 * (deltas[mid - 1] + deltas[mid])


def solve_3x3(matrix: list[list[float]], vector: list[float]) -> tuple[float, float, float] | None:
    a = [row[:] + [rhs] for row, rhs in zip(matrix, vector)]
    for col in range(3):
        pivot = max(range(col, 3), key=lambda row: abs(a[row][col]))
        if abs(a[pivot][col]) < 1e-12:
            return None
        a[col], a[pivot] = a[pivot], a[col]
        pivot_value = a[col][col]
        for j in range(col, 4):
            a[col][j] /= pivot_value
        for row in range(3):
            if row == col:
                continue
            factor = a[row][col]
            for j in range(col, 4):
                a[row][j] -= factor * a[col][j]
    return a[0][3], a[1][3], a[2][3]


def write_json(path: Path, payload: Any) -> None:
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def write_metrics_csv(path: Path, comparison: dict[str, Any]) -> None:
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["scope", "signal", "metric", "value"])
        for scope in ["simulation", "robot"]:
            for signal, stats in comparison[scope]["signals"].items():
                for metric, value in stats.items():
                    writer.writerow([scope, signal, metric, value])
        for signal, stats in comparison["aligned_normalized_time_error"].items():
            for metric, value in stats.items():
                writer.writerow(["aligned_error", signal, metric, value])
        for scope, signals in comparison["tracking_error"].items():
            for signal, stats in signals.items():
                for metric, value in stats.items():
                    writer.writerow([f"tracking_{scope}", signal, metric, value])


def write_fit_csv(path: Path, fits: dict[str, Any]) -> None:
    rows = list(fits.values())
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def write_manifest(
    path: Path,
    sim_baseline: Path,
    robot_run: Path,
    analysis_window: str,
    sim_window: WindowSelection,
    robot_window: WindowSelection,
) -> None:
    lines = [
        "Unstructured simulation-vs-robot model validation",
        "",
        f"Simulation baseline: {sim_baseline}",
        f"Robot run: {robot_run}",
        f"Analysis window: {analysis_window}",
        f"Simulation selected samples: {sim_window.output_samples}/{sim_window.input_samples} ({sim_window.note})",
        f"Robot selected samples: {robot_window.output_samples}/{robot_window.input_samples} ({robot_window.note})",
        "",
        "Interpretation:",
        "- The current default simulation and robot runs are not the same physical map.",
        "- Use normalized-time and normalized-distance comparisons for the first thesis baseline.",
        "- For final validation, rerun simulation and hardware with the same gate/map protocol.",
    ]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def aggregate_rows(rows: list[dict[str, Any]]) -> dict[str, dict[str, Any]]:
    metrics = [
        "speed_rmse_vs_sim",
        "yaw_rate_rmse_vs_sim",
        "distance_norm_rmse_vs_sim",
        "heading_rmse_vs_sim",
        "robot_speed_target_rms",
        "robot_yaw_target_rms",
        "robot_speed_fit_tau_s",
        "robot_yaw_fit_tau_s",
    ]
    return {metric: aggregate_metric([row.get(metric) for row in rows]) for metric in metrics}


def aggregate_metric(values: list[Any]) -> dict[str, Any]:
    numeric = [float(value) for value in values if isinstance(value, (int, float)) and math.isfinite(float(value))]
    if not numeric:
        return {"count": 0, "mean": None, "std": None, "min": None, "max": None}
    mean = sum(numeric) / len(numeric)
    variance = sum((value - mean) ** 2 for value in numeric) / len(numeric)
    return {
        "count": len(numeric),
        "mean": mean,
        "std": math.sqrt(variance),
        "min": min(numeric),
        "max": max(numeric),
    }


def write_aggregate_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def write_aggregate_plots(output_dir: Path, rows: list[dict[str, Any]]) -> None:
    if not rows:
        return
    categories = [row["robot_run_id"] for row in rows]
    write_grouped_bar_chart(
        output_dir / "aggregate_error_summary.svg",
        title="Multi-run normalized error against simulation",
        categories=categories,
        y_label="RMSE",
        groups=[
            {"label": "speed", "values": [row["speed_rmse_vs_sim"] for row in rows], "color": "#1f77b4"},
            {"label": "yaw-rate", "values": [row["yaw_rate_rmse_vs_sim"] for row in rows], "color": "#d62728"},
            {"label": "distance", "values": [row["distance_norm_rmse_vs_sim"] for row in rows], "color": "#2ca02c"},
        ],
    )
    write_grouped_bar_chart(
        output_dir / "aggregate_tracking_summary.svg",
        title="Multi-run robot tracking error",
        categories=categories,
        y_label="RMS",
        groups=[
            {"label": "speed-target", "values": [row["robot_speed_target_rms"] for row in rows], "color": "#1f77b4"},
            {"label": "yaw-target", "values": [row["robot_yaw_target_rms"] for row in rows], "color": "#d62728"},
        ],
    )


def write_plots(output_dir: Path, sim, robot, comparison: dict[str, Any]) -> None:
    sim_label = f"sim {sim.run_id}"
    robot_label = f"robot {robot.run_id}"

    write_time_series_chart(
        output_dir / "speed_comparison.svg",
        title="Simulation vs robot speed",
        x_label="normalized time",
        y_label="speed [m/s]",
        series=[
            {"label": sim_label, "points": normalize_time(signal_series(sim, "speed")), "color": "#1f77b4"},
            {"label": robot_label, "points": normalize_time(signal_series(robot, "speed")), "color": "#d62728"},
        ],
    )
    write_time_series_chart(
        output_dir / "yaw_rate_comparison.svg",
        title="Simulation vs robot yaw-rate",
        x_label="normalized time",
        y_label="yaw-rate [rad/s]",
        series=[
            {"label": sim_label, "points": normalize_time(yaw_rate_series(sim)), "color": "#1f77b4"},
            {"label": robot_label, "points": normalize_time(yaw_rate_series(robot)), "color": "#d62728"},
        ],
    )
    write_time_series_chart(
        output_dir / "distance_to_goal_normalized.svg",
        title="Normalized distance-to-goal",
        x_label="normalized time",
        y_label="distance / initial distance",
        series=[
            {"label": sim_label, "points": normalize_time(normalized_distance_to_goal(sim)), "color": "#1f77b4"},
            {"label": robot_label, "points": normalize_time(normalized_distance_to_goal(robot)), "color": "#d62728"},
        ],
    )
    write_time_series_chart(
        output_dir / "tracking_error_comparison.svg",
        title="Tracking / estimation error signals",
        x_label="normalized time",
        y_label="error",
        series=[
            {"label": "sim cross-track [m]", "points": normalize_time(resolve_signal(sim, "tracker_cross_track")), "color": "#1f77b4"},
            {"label": "sim heading [deg]", "points": normalize_time(resolve_signal(sim, "tracker_heading_error_deg")), "color": "#2ca02c"},
            {"label": "robot speed-target [m/s]", "points": normalize_time(error_series(robot, "speed", "target_speed")), "color": "#d62728"},
            {"label": "robot yaw-target [rad/s]", "points": normalize_time(error_series(robot, "yaw_rate", "target_yaw_rate")), "color": "#9467bd"},
        ],
    )
    write_time_series_chart(
        output_dir / "timing_comparison.svg",
        title="Simulation vs robot loop timing",
        x_label="normalized time",
        y_label="milliseconds",
        series=[
            {"label": "sim planning", "points": normalize_time(resolve_signal(sim, "planning_ms")), "color": "#1f77b4"},
            {"label": "robot planning", "points": normalize_time(resolve_signal(robot, "planning_ms")), "color": "#d62728"},
            {"label": "sim step", "points": normalize_time(resolve_signal(sim, "step_ms")), "color": "#17becf"},
            {"label": "robot step", "points": normalize_time(resolve_signal(robot, "step_ms")), "color": "#ff7f0e"},
        ],
    )

    categories = ["speed", "yaw_rate", "distance", "cross_track", "heading"]
    errors = comparison["aligned_normalized_time_error"]
    write_grouped_bar_chart(
        output_dir / "aligned_error_summary.svg",
        title="Normalized-time robot minus simulation error",
        categories=categories,
        y_label="RMSE",
        groups=[
            {
                "label": "rmse",
                "values": [
                    errors["speed"]["rmse"],
                    errors["yaw_rate"]["rmse"],
                    errors["distance_to_goal_norm"]["rmse"],
                    errors["tracker_cross_track"]["rmse"],
                    errors["tracker_heading_error_deg"]["rmse"],
                ],
                "color": "#d62728",
            }
        ],
    )


if __name__ == "__main__":
    raise SystemExit(main())
