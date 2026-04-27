from __future__ import annotations

import argparse
import csv
import glob
import itertools
import json
import subprocess
import sys
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from data_analisys.model_validation import fit_reports, load_report, run_model_validation, run_multi_run_validation
from data_analisys.validation_presets import load_preset_config, preset_names


DEFAULT_SIM_BIN = Path("build/simulator/thesis_planner_sim")
DEFAULT_OUTPUT_DIR = Path("data_analisys/outputs/structured_validation_road_tuning")
DEFAULT_PRESET = "structured_validation_road"


@dataclass(frozen=True)
class TuningCandidate:
    max_curvature: float
    max_steer_rate: float
    yaw_feedback_gain: float
    left_pwm_scale: float
    right_pwm_scale: float
    yaw_response_scale: float


@dataclass(frozen=True)
class TuningResult:
    rank_score: float
    robot_run_count: int
    sim_status: str
    sim_report: str | None
    validation_dir: str | None
    max_curvature: float
    max_steer_rate: float
    yaw_feedback_gain: float
    left_pwm_scale: float
    right_pwm_scale: float
    yaw_response_scale: float
    speed_rmse: float | None
    speed_rmse_max: float | None
    yaw_rate_rmse: float | None
    yaw_rate_rmse_max: float | None
    yaw_unwrapped_rmse: float | None
    yaw_unwrapped_rmse_max: float | None
    distance_rmse: float | None
    heading_rmse_deg: float | None
    heading_rmse_deg_max: float | None
    sim_speed_mean: float | None
    robot_speed_mean: float | None
    sim_yaw_fit_a: float | None
    sim_yaw_fit_b: float | None
    sim_yaw_fit_c: float | None
    sim_yaw_fit_gain: float | None
    sim_yaw_fit_tau_s: float | None
    sim_yaw_fit_rmse: float | None
    note: str


def parse_float_list(value: str) -> list[float]:
    parts = [item.strip() for item in value.split(",")]
    return [float(item) for item in parts if item]


def parse_scale_pairs(value: str) -> list[tuple[float, float]]:
    pairs: list[tuple[float, float]] = []
    for item in value.split(","):
        item = item.strip()
        if not item:
            continue
        if ":" not in item:
            raise ValueError(f"Scale pair '{item}' must use left:right format")
        left, right = item.split(":", 1)
        pairs.append((float(left), float(right)))
    return pairs


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run a structured-road simulator tuning sweep against a real-robot baseline.",
    )
    parser.add_argument("--preset", choices=preset_names(), default=DEFAULT_PRESET)
    parser.add_argument("--sim-bin", type=Path, default=DEFAULT_SIM_BIN)
    parser.add_argument("--robot-run", type=Path, default=None)
    parser.add_argument("--robot-glob", default=None)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--structured-map", default="validation")
    parser.add_argument("--max-steps", type=int, default=1200)
    parser.add_argument("--window", default="full")
    parser.add_argument("--grid-size", type=int, default=250)
    parser.add_argument("--max-curvature-values", default="10,14,18")
    parser.add_argument("--max-steer-rate-values", default="4.5,7.5,10.0")
    parser.add_argument("--yaw-feedback-gain-values", default="60,90,120,150")
    parser.add_argument("--pwm-scale-pairs", default="1.0:1.0")
    parser.add_argument("--yaw-response-scale-values", default="0.16,0.24,0.32,0.40")
    parser.add_argument("--speed-estimate-per-pwm", type=float, default=None)
    parser.add_argument("--pwm-slew-rate", type=float, default=None)
    parser.add_argument("--motor-time-constant", type=float, default=None)
    parser.add_argument("--max-linear-speed", type=float, default=None)
    parser.add_argument("--cruise-speed-limit", type=float, default=None)
    parser.add_argument("--min-effective-pwm", type=float, default=None)
    parser.add_argument("--limit", type=int, default=None, help="Optionally cap the number of candidates.")
    return parser


def build_candidates(args: argparse.Namespace) -> list[TuningCandidate]:
    curvatures = parse_float_list(args.max_curvature_values)
    steer_rates = parse_float_list(args.max_steer_rate_values)
    yaw_gains = parse_float_list(args.yaw_feedback_gain_values)
    scale_pairs = parse_scale_pairs(args.pwm_scale_pairs)
    yaw_response_scales = parse_float_list(args.yaw_response_scale_values)

    candidates = [
        TuningCandidate(
            max_curvature=max_curvature,
            max_steer_rate=max_steer_rate,
            yaw_feedback_gain=yaw_feedback_gain,
            left_pwm_scale=left_scale,
            right_pwm_scale=right_scale,
            yaw_response_scale=yaw_response_scale,
        )
        for max_curvature, max_steer_rate, yaw_feedback_gain, (left_scale, right_scale), yaw_response_scale in itertools.product(
            curvatures, steer_rates, yaw_gains, scale_pairs, yaw_response_scales
        )
    ]
    if args.limit is not None:
        return candidates[: max(args.limit, 0)]
    return candidates


def apply_fixed_overrides(command: list[str], args: argparse.Namespace) -> None:
    optional_args = {
        "speed-estimate-per-pwm": args.speed_estimate_per_pwm,
        "pwm-slew-rate": args.pwm_slew_rate,
        "motor-time-constant": args.motor_time_constant,
        "max-linear-speed": args.max_linear_speed,
        "cruise-speed-limit": args.cruise_speed_limit,
        "min-effective-pwm": args.min_effective_pwm,
    }
    for name, value in optional_args.items():
        if value is not None:
            command.extend([f"--{name}", f"{value}"])


def parse_headless_stdout(stdout: str) -> dict[str, str]:
    parsed: dict[str, str] = {}
    for line in stdout.splitlines():
        if "=" not in line:
            continue
        key, value = line.split("=", 1)
        parsed[key.strip()] = value.strip()
    return parsed


def safe_nested_value(payload: dict[str, Any], *keys: str) -> float | None:
    current: Any = payload
    for key in keys:
        if not isinstance(current, dict):
            return None
        current = current.get(key)
    if current is None:
        return None
    try:
        return float(current)
    except (TypeError, ValueError):
        return None


def safe_aggregate_stat(payload: dict[str, Any], metric: str, stat: str = "mean") -> float | None:
    return safe_nested_value(payload, "aggregate_stats", metric, stat)


def score_candidate(comparison: dict[str, Any], fits: dict[str, Any], sim_status: str) -> tuple[float, str]:
    speed_rmse = safe_nested_value(comparison, "aligned_normalized_time_error", "speed", "rmse") or 10.0
    yaw_rmse = safe_nested_value(comparison, "aligned_normalized_time_error", "yaw_rate", "rmse") or 10.0
    yaw_unwrapped_rmse = safe_nested_value(comparison, "aligned_normalized_time_error", "yaw_unwrapped", "rmse") or 10.0
    distance_rmse = safe_nested_value(comparison, "aligned_normalized_time_error", "distance_to_goal_norm", "rmse") or 10.0
    heading_rmse = safe_nested_value(comparison, "aligned_normalized_time_error", "tracker_heading_error_deg", "rmse") or 180.0

    score = (
        1.2 * (yaw_rmse / 0.28) +
        1.2 * (yaw_unwrapped_rmse / 0.40) +
        0.8 * (heading_rmse / 15.0) +
        0.4 * (speed_rmse / 0.05) +
        0.4 * (distance_rmse / 0.15)
    )

    notes: list[str] = []
    if sim_status != "goal_reached":
        score += 4.0
        notes.append(f"sim_status={sim_status}")

    sim_yaw_fit_a = safe_nested_value(fits, "sim_yaw_pwm_fit", "a")
    sim_yaw_fit_b = safe_nested_value(fits, "sim_yaw_pwm_fit", "b")
    sim_yaw_fit_tau = safe_nested_value(fits, "sim_yaw_pwm_fit", "tau_s")
    sim_yaw_fit_gain = safe_nested_value(fits, "sim_yaw_pwm_fit", "gain")
    if sim_yaw_fit_a is None:
        score += 2.0
        notes.append("sim_yaw_fit_singular")
    else:
        score += 1.2 * band_penalty(sim_yaw_fit_a, 0.75, 0.92, 0.10)
        if sim_yaw_fit_a <= 0.0 or sim_yaw_fit_a >= 1.0:
            score += 3.0
            notes.append(f"yaw_a_unstable={sim_yaw_fit_a:.3g}")

    if sim_yaw_fit_b is None:
        score += 2.0
        notes.append("sim_yaw_b_missing")
    else:
        score += 1.5 * band_penalty(sim_yaw_fit_b, 0.0004, 0.0028, 0.0010)
        if sim_yaw_fit_b <= 0.0:
            score += 3.0
            notes.append(f"yaw_b_nonpositive={sim_yaw_fit_b:.3g}")

    score += band_penalty(sim_yaw_fit_tau, 0.25, 0.70, 0.20)
    score += band_penalty(sim_yaw_fit_gain, 0.0040, 0.0130, 0.0060)

    return score, ", ".join(notes) or "ok"


def score_aggregate_candidate(aggregate: dict[str, Any], fits: dict[str, Any], sim_status: str) -> tuple[float, str]:
    speed_rmse = safe_aggregate_stat(aggregate, "speed_rmse_vs_sim", "mean") or 10.0
    speed_rmse_max = safe_aggregate_stat(aggregate, "speed_rmse_vs_sim", "max") or 10.0
    yaw_rmse = safe_aggregate_stat(aggregate, "yaw_rate_rmse_vs_sim", "mean") or 10.0
    yaw_rmse_max = safe_aggregate_stat(aggregate, "yaw_rate_rmse_vs_sim", "max") or 10.0
    yaw_unwrapped_rmse = safe_aggregate_stat(aggregate, "yaw_unwrapped_rmse_vs_sim", "mean") or 10.0
    yaw_unwrapped_rmse_max = safe_aggregate_stat(aggregate, "yaw_unwrapped_rmse_vs_sim", "max") or 10.0
    distance_rmse = safe_aggregate_stat(aggregate, "distance_norm_rmse_vs_sim", "mean") or 10.0
    heading_rmse = safe_aggregate_stat(aggregate, "heading_rmse_vs_sim", "mean") or 180.0
    heading_rmse_max = safe_aggregate_stat(aggregate, "heading_rmse_vs_sim", "max") or 180.0

    score = (
        1.3 * (yaw_rmse / 0.22) +
        1.1 * (yaw_rmse_max / 0.32) +
        1.3 * (yaw_unwrapped_rmse / 0.70) +
        0.9 * (yaw_unwrapped_rmse_max / 1.00) +
        0.9 * (heading_rmse / 38.0) +
        0.5 * (heading_rmse_max / 50.0) +
        0.5 * (speed_rmse / 0.012) +
        0.2 * (speed_rmse_max / 0.015) +
        0.5 * (distance_rmse / 0.12)
    )

    notes: list[str] = []
    if sim_status != "goal_reached":
        score += 4.0
        notes.append(f"sim_status={sim_status}")

    sim_yaw_fit_a = safe_nested_value(fits, "sim_yaw_pwm_fit", "a")
    sim_yaw_fit_b = safe_nested_value(fits, "sim_yaw_pwm_fit", "b")
    sim_yaw_fit_tau = safe_nested_value(fits, "sim_yaw_pwm_fit", "tau_s")
    sim_yaw_fit_gain = safe_nested_value(fits, "sim_yaw_pwm_fit", "gain")
    if sim_yaw_fit_a is None:
        score += 2.0
        notes.append("sim_yaw_fit_singular")
    else:
        score += 1.0 * band_penalty(sim_yaw_fit_a, 0.70, 0.95, 0.12)
        if sim_yaw_fit_a <= 0.0 or sim_yaw_fit_a >= 1.0:
            score += 3.0
            notes.append(f"yaw_a_unstable={sim_yaw_fit_a:.3g}")
    if sim_yaw_fit_b is None:
        score += 2.0
        notes.append("sim_yaw_b_missing")
    else:
        score += 1.0 * band_penalty(sim_yaw_fit_b, 0.0005, 0.0030, 0.0010)
        if sim_yaw_fit_b <= 0.0:
            score += 3.0
            notes.append(f"yaw_b_nonpositive={sim_yaw_fit_b:.3g}")
    score += 0.5 * band_penalty(sim_yaw_fit_tau, 0.20, 1.20, 0.30)
    score += 0.5 * band_penalty(sim_yaw_fit_gain, 0.0040, 0.0160, 0.0060)
    return score, ", ".join(notes) or "ok"


def band_penalty(value: float | None, low: float, high: float, scale: float) -> float:
    if value is None:
        return 1.0
    if low <= value <= high:
        return 0.0
    if value < low:
        return min((low - value) / max(scale, 1e-9), 5.0)
    return min((value - high) / max(scale, 1e-9), 5.0)


def write_csv(path: Path, rows: list[TuningResult]) -> None:
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    headers = list(asdict(rows[0]).keys())
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=headers)
        writer.writeheader()
        for row in rows:
            writer.writerow(asdict(row))


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if not args.sim_bin.exists():
        raise SystemExit(f"Simulator binary not found: {args.sim_bin}")

    config = load_preset_config(args.preset)
    robot_run = args.robot_run or Path(str(config.get("robot_validation_run") or ""))
    robot_runs: list[Path]
    if args.robot_glob:
        robot_runs = [Path(path) for path in sorted(glob.glob(args.robot_glob))]
        if not robot_runs:
            raise SystemExit(f"No robot reports matched: {args.robot_glob}")
    else:
        robot_runs = [robot_run]

    if not robot_run.exists():
        if robot_runs:
            robot_run = robot_runs[-1]
        else:
            raise SystemExit(f"Robot baseline not found: {robot_run}")

    output_dir = args.output_dir
    output_dir.mkdir(parents=True, exist_ok=True)

    candidates = build_candidates(args)
    if not candidates:
        raise SystemExit("No tuning candidates generated.")

    results: list[TuningResult] = []
    for index, candidate in enumerate(candidates, start=1):
        command = [
            str(args.sim_bin),
            "--headless",
            "--scenario",
            "structured",
            "--structured-map",
            args.structured_map,
            "--max-steps",
            str(args.max_steps),
            "--max-curvature",
            str(candidate.max_curvature),
            "--max-steer-rate",
            str(candidate.max_steer_rate),
            "--yaw-feedback-gain",
            str(candidate.yaw_feedback_gain),
            "--left-pwm-scale",
            str(candidate.left_pwm_scale),
            "--right-pwm-scale",
            str(candidate.right_pwm_scale),
            "--yaw-response-scale",
            str(candidate.yaw_response_scale),
        ]
        apply_fixed_overrides(command, args)

        print(f"[{index}/{len(candidates)}] running", " ".join(command))
        completed = subprocess.run(
            command,
            cwd=Path.cwd(),
            text=True,
            capture_output=True,
            check=False,
        )
        parsed = parse_headless_stdout(completed.stdout)
        sim_status = parsed.get("status", "launch_failed")
        sim_report = parsed.get("report_json")

        if sim_report is None or not Path(sim_report).exists():
            results.append(
                TuningResult(
                    rank_score=999.0,
                    robot_run_count=len(robot_runs),
                    sim_status=sim_status,
                    sim_report=sim_report,
                    validation_dir=None,
                    max_curvature=candidate.max_curvature,
                    max_steer_rate=candidate.max_steer_rate,
                    yaw_feedback_gain=candidate.yaw_feedback_gain,
                    left_pwm_scale=candidate.left_pwm_scale,
                    right_pwm_scale=candidate.right_pwm_scale,
                    yaw_response_scale=candidate.yaw_response_scale,
                    speed_rmse=None,
                    speed_rmse_max=None,
                    yaw_rate_rmse=None,
                    yaw_rate_rmse_max=None,
                    yaw_unwrapped_rmse=None,
                    yaw_unwrapped_rmse_max=None,
                    distance_rmse=None,
                    heading_rmse_deg=None,
                    heading_rmse_deg_max=None,
                    sim_speed_mean=None,
                    robot_speed_mean=None,
                    sim_yaw_fit_a=None,
                    sim_yaw_fit_b=None,
                    sim_yaw_fit_c=None,
                    sim_yaw_fit_gain=None,
                    sim_yaw_fit_tau_s=None,
                    sim_yaw_fit_rmse=None,
                    note="missing_report_json",
                )
            )
            continue

        candidate_slug = (
            f"curv_{candidate.max_curvature:.2f}_"
            f"steer_{candidate.max_steer_rate:.2f}_"
            f"yaw_{candidate.yaw_feedback_gain:.1f}_"
            f"scale_{candidate.left_pwm_scale:.2f}_{candidate.right_pwm_scale:.2f}_"
            f"yawresp_{candidate.yaw_response_scale:.2f}"
        ).replace(".", "p")
        validation_dir = output_dir / candidate_slug
        sim_path = Path(sim_report)
        sim_loaded = load_report(sim_path)
        primary_robot_loaded = load_report(robot_run)
        fits = fit_reports(sim_loaded, primary_robot_loaded)
        extra_notes = [
            "Ranking score is heuristic and prioritizes yaw-rate shape, yaw accumulation, heading error, and completion status.",
            f"Candidate overrides: max_curvature={candidate.max_curvature}, "
            f"max_steer_rate={candidate.max_steer_rate}, yaw_feedback_gain={candidate.yaw_feedback_gain}, "
            f"left_pwm_scale={candidate.left_pwm_scale}, right_pwm_scale={candidate.right_pwm_scale}, "
            f"yaw_response_scale={candidate.yaw_response_scale}.",
        ]
        if args.robot_glob:
            aggregate = run_multi_run_validation(
                sim_path,
                robot_runs,
                validation_dir,
                analysis_window=args.window,
                grid_size=args.grid_size,
                scenario_label=f"{config.get('label', args.preset)} tuning sweep",
                extra_notes=extra_notes,
            )
            rank_score, note = score_aggregate_candidate(aggregate, fits, sim_status)
            speed_rmse = safe_aggregate_stat(aggregate, "speed_rmse_vs_sim", "mean")
            speed_rmse_max = safe_aggregate_stat(aggregate, "speed_rmse_vs_sim", "max")
            yaw_rate_rmse = safe_aggregate_stat(aggregate, "yaw_rate_rmse_vs_sim", "mean")
            yaw_rate_rmse_max = safe_aggregate_stat(aggregate, "yaw_rate_rmse_vs_sim", "max")
            yaw_unwrapped_rmse = safe_aggregate_stat(aggregate, "yaw_unwrapped_rmse_vs_sim", "mean")
            yaw_unwrapped_rmse_max = safe_aggregate_stat(aggregate, "yaw_unwrapped_rmse_vs_sim", "max")
            distance_rmse = safe_aggregate_stat(aggregate, "distance_norm_rmse_vs_sim", "mean")
            heading_rmse_deg = safe_aggregate_stat(aggregate, "heading_rmse_vs_sim", "mean")
            heading_rmse_deg_max = safe_aggregate_stat(aggregate, "heading_rmse_vs_sim", "max")
            sim_speed_mean = None
            robot_speed_mean = None
        else:
            comparison, fits = run_model_validation(
                sim_path,
                robot_run,
                validation_dir,
                grid_size=args.grid_size,
                analysis_window=args.window,
                scenario_label=f"{config.get('label', args.preset)} tuning sweep",
                extra_notes=extra_notes,
            )
            rank_score, note = score_candidate(comparison, fits, sim_status)
            speed_rmse = safe_nested_value(comparison, "aligned_normalized_time_error", "speed", "rmse")
            speed_rmse_max = safe_nested_value(comparison, "aligned_normalized_time_error", "speed", "max_abs")
            yaw_rate_rmse = safe_nested_value(comparison, "aligned_normalized_time_error", "yaw_rate", "rmse")
            yaw_rate_rmse_max = safe_nested_value(comparison, "aligned_normalized_time_error", "yaw_rate", "max_abs")
            yaw_unwrapped_rmse = safe_nested_value(comparison, "aligned_normalized_time_error", "yaw_unwrapped", "rmse")
            yaw_unwrapped_rmse_max = safe_nested_value(comparison, "aligned_normalized_time_error", "yaw_unwrapped", "max_abs")
            distance_rmse = safe_nested_value(comparison, "aligned_normalized_time_error", "distance_to_goal_norm", "rmse")
            heading_rmse_deg = safe_nested_value(comparison, "aligned_normalized_time_error", "tracker_heading_error_deg", "rmse")
            heading_rmse_deg_max = safe_nested_value(comparison, "aligned_normalized_time_error", "tracker_heading_error_deg", "max_abs")
            sim_speed_mean = safe_nested_value(comparison, "simulation", "signals", "speed", "mean")
            robot_speed_mean = safe_nested_value(comparison, "robot", "signals", "speed", "mean")

        results.append(
            TuningResult(
                rank_score=rank_score,
                robot_run_count=len(robot_runs),
                sim_status=sim_status,
                sim_report=sim_report,
                validation_dir=str(validation_dir),
                max_curvature=candidate.max_curvature,
                max_steer_rate=candidate.max_steer_rate,
                yaw_feedback_gain=candidate.yaw_feedback_gain,
                left_pwm_scale=candidate.left_pwm_scale,
                right_pwm_scale=candidate.right_pwm_scale,
                yaw_response_scale=candidate.yaw_response_scale,
                speed_rmse=speed_rmse,
                speed_rmse_max=speed_rmse_max,
                yaw_rate_rmse=yaw_rate_rmse,
                yaw_rate_rmse_max=yaw_rate_rmse_max,
                yaw_unwrapped_rmse=yaw_unwrapped_rmse,
                yaw_unwrapped_rmse_max=yaw_unwrapped_rmse_max,
                distance_rmse=distance_rmse,
                heading_rmse_deg=heading_rmse_deg,
                heading_rmse_deg_max=heading_rmse_deg_max,
                sim_speed_mean=sim_speed_mean,
                robot_speed_mean=robot_speed_mean,
                sim_yaw_fit_a=safe_nested_value(fits, "sim_yaw_pwm_fit", "a"),
                sim_yaw_fit_b=safe_nested_value(fits, "sim_yaw_pwm_fit", "b"),
                sim_yaw_fit_c=safe_nested_value(fits, "sim_yaw_pwm_fit", "c"),
                sim_yaw_fit_gain=safe_nested_value(fits, "sim_yaw_pwm_fit", "gain"),
                sim_yaw_fit_tau_s=safe_nested_value(fits, "sim_yaw_pwm_fit", "tau_s"),
                sim_yaw_fit_rmse=safe_nested_value(fits, "sim_yaw_pwm_fit", "rmse"),
                note=note,
            )
        )

    results.sort(key=lambda item: item.rank_score)
    write_csv(output_dir / "tuning_summary.csv", results)
    (output_dir / "tuning_summary.json").write_text(
        json.dumps([asdict(row) for row in results], indent=2),
        encoding="utf-8",
    )

    if results:
        best = results[0]
        (output_dir / "best_candidate.json").write_text(
            json.dumps(asdict(best), indent=2),
            encoding="utf-8",
        )
        print("Best candidate:")
        print(json.dumps(asdict(best), indent=2))
    print(f"Wrote tuning sweep to {output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
