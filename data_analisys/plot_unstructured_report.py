from __future__ import annotations

import argparse
import sys
from pathlib import Path

if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from data_analisys.report_analysis import (  # noqa: E402
    discover_reports_from_status_doc,
    load_report,
    reference_windows,
    summarize_report,
    time_range,
    time_series,
    write_summaries_csv,
    write_summaries_json,
    xy_series,
)
from data_analisys.svg_charts import (  # noqa: E402
    write_grouped_bar_chart,
    write_time_series_chart,
    write_timeline_chart,
    write_xy_chart,
)


DEFAULT_STATUS_DOC = Path("documentation/Unstructured_Hardware_Status_20260418.md")
DEFAULT_REPORTS_DIR = Path("reports")
DEFAULT_OUTPUT_DIR = Path("data_analisys/outputs/unstructured_20260418")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Plot unstructured hardware report highlights from thesis JSON reports.",
    )
    parser.add_argument(
        "--status-doc",
        type=Path,
        default=DEFAULT_STATUS_DOC,
        help="Markdown status document used to discover representative run IDs.",
    )
    parser.add_argument(
        "--reports-dir",
        type=Path,
        default=DEFAULT_REPORTS_DIR,
        help="Directory containing thesis JSON reports.",
    )
    parser.add_argument(
        "--report",
        type=Path,
        action="append",
        default=[],
        help="Explicit report JSON path. Can be passed more than once.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=DEFAULT_OUTPUT_DIR,
        help="Destination directory for SVG plots and summary tables.",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    args.output_dir.mkdir(parents=True, exist_ok=True)

    missing: list[str] = []
    if args.report:
        report_paths = args.report
    else:
        report_paths, missing = discover_reports_from_status_doc(args.status_doc, args.reports_dir)

    if not report_paths:
        print("No reports found. Pass --report or check --status-doc / --reports-dir.", file=sys.stderr)
        return 2

    reports = [load_report(path) for path in report_paths]
    summaries = [summarize_report(report) for report in reports]

    write_summaries_csv(args.output_dir / "summary.csv", summaries)
    write_summaries_json(args.output_dir / "summary.json", summaries)

    for report in reports:
        plot_run(report, args.output_dir / report.run_id)

    plot_summary(reports, summaries, args.output_dir)
    write_manifest(args.output_dir / "manifest.txt", report_paths, missing)

    print(f"Generated {len(reports)} report analysis bundle(s) in {args.output_dir}")
    if missing:
        print("Missing report suffixes from status doc: " + ", ".join(missing))
    return 0


def plot_run(report, output_dir: Path) -> None:
    completion_time = summarize_report(report).first_gate_completion_s
    markers = []
    if completion_time is not None:
        markers.append({"x": completion_time, "label": "gate completion", "color": "#d62728"})

    write_time_series_chart(
        output_dir / "gate_distance.svg",
        title=f"{report.run_id} gate distance",
        x_label="time [s]",
        y_label="distance [m]",
        series=[
            {
                "label": "distance_to_goal",
                "points": time_series(report, "distance_to_goal", valid_nonnegative=True),
                "color": "#1f77b4",
            },
            {
                "label": "chosen_gate_distance",
                "points": time_series(report, "chosen_gate_distance", valid_nonnegative=True),
                "color": "#ff7f0e",
            },
        ],
        markers=markers,
    )

    write_time_series_chart(
        output_dir / "gate_reference.svg",
        title=f"{report.run_id} gate visibility and reference",
        x_label="time [s]",
        y_label="count / flag",
        series=[
            {"label": "candidate_gates", "points": time_series(report, "candidate_gates"), "color": "#1f77b4"},
            {"label": "visible_gates", "points": time_series(report, "visible_gates"), "color": "#2ca02c"},
            {"label": "planner_has_reference", "points": time_series(report, "planner_has_reference"), "color": "#d62728"},
            {"label": "chosen_gate_index", "points": time_series(report, "chosen_gate_index"), "color": "#9467bd"},
        ],
        markers=markers,
    )

    write_time_series_chart(
        output_dir / "lidar_clearance.svg",
        title=f"{report.run_id} lidar clearance",
        x_label="time [s]",
        y_label="meters",
        series=[
            {"label": "min_lidar", "points": time_series(report, "min_lidar", valid_nonnegative=True), "color": "#1f77b4"},
            {"label": "front_lidar", "points": time_series(report, "front_lidar", valid_nonnegative=True), "color": "#2ca02c"},
        ],
        markers=markers,
    )

    write_time_series_chart(
        output_dir / "lidar_close_samples.svg",
        title=f"{report.run_id} close lidar samples",
        x_label="time [s]",
        y_label="samples",
        series=[
            {"label": "close_lidar_samples", "points": time_series(report, "close_lidar_samples"), "color": "#d62728"},
            {"label": "front_close_samples", "points": time_series(report, "front_close_lidar_samples"), "color": "#9467bd"},
        ],
        markers=markers,
    )

    write_time_series_chart(
        output_dir / "controller_pwm.svg",
        title=f"{report.run_id} controller PWM",
        x_label="time [s]",
        y_label="PWM",
        series=[
            {"label": "controller_pwm_left", "points": time_series(report, "controller_pwm_left"), "color": "#1f77b4"},
            {"label": "controller_pwm_right", "points": time_series(report, "controller_pwm_right"), "color": "#d62728"},
            {"label": "target_pwm_left", "points": time_series(report, "controller_target_pwm_left"), "color": "#17becf"},
            {"label": "target_pwm_right", "points": time_series(report, "controller_target_pwm_right"), "color": "#ff7f0e"},
        ],
        markers=markers,
    )

    write_time_series_chart(
        output_dir / "controller_status.svg",
        title=f"{report.run_id} controller status",
        x_label="time [s]",
        y_label="flags / cycles",
        series=[
            {"label": "status_flags", "points": time_series(report, "controller_status_flags"), "color": "#1f77b4"},
            {"label": "motor_flags", "points": time_series(report, "controller_motor_flags"), "color": "#2ca02c"},
            {"label": "error_code", "points": time_series(report, "controller_error_code"), "color": "#d62728"},
            {"label": "no_motion_cycles", "points": time_series(report, "no_motion_cycles"), "color": "#9467bd"},
        ],
        markers=markers,
    )

    write_time_series_chart(
        output_dir / "motion_state.svg",
        title=f"{report.run_id} motion state",
        x_label="time [s]",
        y_label="m/s and rad/s",
        series=[
            {"label": "speed", "points": time_series(report, "speed"), "color": "#1f77b4"},
            {"label": "target_speed", "points": time_series(report, "target_speed"), "color": "#17becf"},
            {"label": "yaw_rate", "points": time_series(report, "yaw_rate"), "color": "#d62728"},
            {"label": "target_yaw_rate", "points": time_series(report, "target_yaw_rate"), "color": "#ff7f0e"},
        ],
        markers=markers,
    )

    write_time_series_chart(
        output_dir / "timing.svg",
        title=f"{report.run_id} loop timing",
        x_label="time [s]",
        y_label="milliseconds",
        series=[
            {"label": "planning_ms", "points": time_series(report, "planning_ms"), "color": "#1f77b4"},
            {"label": "lidar_ms", "points": time_series(report, "lidar_ms"), "color": "#2ca02c"},
            {"label": "step_ms", "points": time_series(report, "step_ms"), "color": "#d62728"},
            {"label": "tracking_ms", "points": time_series(report, "tracking_ms"), "color": "#9467bd"},
        ],
        markers=markers,
    )

    write_xy_chart(
        output_dir / "trajectory_xy.svg",
        title=f"{report.run_id} estimated trajectory",
        points=xy_series(report),
    )


def plot_summary(reports, summaries, output_dir: Path) -> None:
    categories = [summary.run_id for summary in summaries]
    write_grouped_bar_chart(
        output_dir / "summary_gate_timing.svg",
        title="Unstructured hardware gate timing",
        categories=categories,
        y_label="seconds",
        groups=[
            {
                "label": "first completion",
                "values": [summary.first_gate_completion_s for summary in summaries],
                "color": "#d62728",
            },
            {
                "label": "longest reference",
                "values": [summary.longest_reference_window_s for summary in summaries],
                "color": "#1f77b4",
            },
        ],
    )

    write_grouped_bar_chart(
        output_dir / "summary_loop_timing.svg",
        title="Unstructured hardware loop timing",
        categories=categories,
        y_label="milliseconds",
        groups=[
            {"label": "avg planning", "values": [summary.avg_planning_ms for summary in summaries], "color": "#1f77b4"},
            {"label": "avg lidar", "values": [summary.avg_lidar_ms for summary in summaries], "color": "#2ca02c"},
            {"label": "avg step", "values": [summary.avg_step_ms for summary in summaries], "color": "#d62728"},
        ],
    )

    timeline_rows = []
    for report, summary in zip(reports, summaries):
        _, end_time = time_range(report)
        markers = []
        if summary.first_gate_completion_s is not None:
            markers.append({"x": summary.first_gate_completion_s, "color": "#d62728"})
        timeline_rows.append(
            {
                "label": summary.run_id,
                "duration_s": end_time or summary.duration_s or 0.0,
                "windows": reference_windows(report),
                "markers": markers,
            }
        )

    write_timeline_chart(
        output_dir / "summary_reference_windows.svg",
        title="Planner reference windows and gate completion",
        rows=timeline_rows,
    )


def write_manifest(path: Path, report_paths: list[Path], missing: list[str]) -> None:
    lines = [
        "Unstructured hardware analysis manifest",
        "",
        "Reports:",
    ]
    lines.extend(f"- {report_path}" for report_path in report_paths)
    if missing:
        lines.extend(["", "Missing suffixes from status document:"])
        lines.extend(f"- {suffix}" for suffix in missing)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


if __name__ == "__main__":
    raise SystemExit(main())
