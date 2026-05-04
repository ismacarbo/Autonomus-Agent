from __future__ import annotations

import argparse
import json
import sys
from dataclasses import asdict
from pathlib import Path
from typing import Any, Callable

if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from data_analisys.report_analysis import as_float, load_report, time_series  # noqa: E402
from data_analisys.unstructured_dynamic_gate_analysis import (  # noqa: E402
    DEFAULT_HARDWARE_REPORTS,
    DEFAULT_OUTPUT_DIR,
    DEFAULT_SIM_REPORT,
    DynamicGateMetrics,
    boolean_windows,
    compute_metrics,
    write_metrics_csv,
    write_thesis_markdown,
)

try:
    from PyQt5 import QtCore, QtWidgets
except ImportError:
    QtCore = None
    QtWidgets = None

try:
    from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg, NavigationToolbar2QT
    from matplotlib.figure import Figure
except ImportError:
    FigureCanvasQTAgg = None
    NavigationToolbar2QT = None
    Figure = None


APP_TITLE = "Unstructured Dynamic Gate Comparison"
DEFAULT_COLORS = {
    "simulation": "#1f77b4",
    "hardware": "#d62728",
    "candidate": "#9ca3af",
    "reference": "#1f77b4",
    "completion": "#d62728",
    "green": "#2ca02c",
    "orange": "#ff7f0e",
    "purple": "#9467bd",
    "cyan": "#17becf",
}


PLOT_VIEWS = [
    ("overview", "Overview dashboard"),
    ("reference_windows", "Reference windows"),
    ("lock_quality", "Lock quality"),
    ("lock_timing", "Lock timing"),
    ("gate_distance_overlay", "Gate distance overlay"),
    ("selected_gate_state", "Selected run gate state"),
    ("motion_command", "Selected run motion"),
    ("lidar_clearance", "Selected run LiDAR clearance"),
    ("loop_timing", "Loop timing"),
    ("trajectory_xy", "Trajectory XY"),
]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Open a Matplotlib/PyQt comparison app for unstructured dynamic-gate reports.",
    )
    parser.add_argument("--simulation", type=Path, default=DEFAULT_SIM_REPORT)
    parser.add_argument("--hardware", type=Path, action="append", default=[])
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if QtWidgets is None or FigureCanvasQTAgg is None or Figure is None:
        print(
            "Missing GUI dependencies. Install them with:\n"
            "  python -m pip install -r data_analisys/requirements.txt\n"
            "or:\n"
            "  python -m pip install PyQt5 matplotlib",
            file=sys.stderr,
        )
        return 2

    app = QtWidgets.QApplication(sys.argv[:1])
    app.setApplicationName(APP_TITLE)
    window = DynamicGateComparisonWindow(
        simulation_path=args.simulation,
        hardware_paths=args.hardware if args.hardware else DEFAULT_HARDWARE_REPORTS,
        output_dir=args.output_dir,
    )
    window.resize(1500, 920)
    window.show()
    return int(app.exec_())


class LoadedReport:
    def __init__(self, path: Path, report: Any, metrics: DynamicGateMetrics):
        self.path = path
        self.report = report
        self.metrics = metrics

    @property
    def label(self) -> str:
        return f"{self.metrics.source}: {self.metrics.run_id}"


if QtWidgets is not None and FigureCanvasQTAgg is not None and Figure is not None:

    class DynamicGateComparisonWindow(QtWidgets.QMainWindow):
        def __init__(self, simulation_path: Path, hardware_paths: list[Path], output_dir: Path):
            super().__init__()
            self.simulation_path = simulation_path
            self.hardware_paths = hardware_paths
            self.output_dir = output_dir
            self.loaded: list[LoadedReport] = []
            self.figure = Figure(figsize=(10, 6), constrained_layout=True)
            self.canvas = FigureCanvasQTAgg(self.figure)
            self.toolbar = NavigationToolbar2QT(self.canvas, self)

            self.setWindowTitle(APP_TITLE)
            self._build_ui()
            self.load_reports()

        def _build_ui(self) -> None:
            root = QtWidgets.QWidget()
            self.setCentralWidget(root)
            layout = QtWidgets.QHBoxLayout(root)
            layout.setContentsMargins(12, 12, 12, 12)
            layout.setSpacing(12)

            sidebar = QtWidgets.QFrame()
            sidebar.setMinimumWidth(420)
            sidebar.setMaximumWidth(500)
            side = QtWidgets.QVBoxLayout(sidebar)
            side.setContentsMargins(12, 12, 12, 12)
            side.setSpacing(10)
            layout.addWidget(sidebar)

            title = QtWidgets.QLabel("Dynamic Gate Analysis")
            title.setObjectName("title")
            side.addWidget(title)

            side.addWidget(QtWidgets.QLabel("Simulation report"))
            sim_row = QtWidgets.QHBoxLayout()
            self.sim_edit = QtWidgets.QLineEdit(str(self.simulation_path))
            self.sim_browse = QtWidgets.QPushButton("Browse")
            self.sim_browse.clicked.connect(lambda: self.browse_file(self.sim_edit))
            sim_row.addWidget(self.sim_edit, 1)
            sim_row.addWidget(self.sim_browse)
            side.addLayout(sim_row)

            side.addWidget(QtWidgets.QLabel("Hardware reports, one per line"))
            self.hardware_edit = QtWidgets.QPlainTextEdit("\n".join(str(path) for path in self.hardware_paths))
            self.hardware_edit.setMinimumHeight(116)
            side.addWidget(self.hardware_edit)

            output_row = QtWidgets.QHBoxLayout()
            self.output_edit = QtWidgets.QLineEdit(str(self.output_dir))
            self.output_browse = QtWidgets.QPushButton("Output")
            self.output_browse.clicked.connect(self.browse_output_dir)
            output_row.addWidget(self.output_edit, 1)
            output_row.addWidget(self.output_browse)
            side.addLayout(output_row)

            load_row = QtWidgets.QHBoxLayout()
            self.load_button = QtWidgets.QPushButton("Load / Refresh")
            self.load_button.clicked.connect(self.load_reports)
            self.export_button = QtWidgets.QPushButton("Export data")
            self.export_button.clicked.connect(self.export_analysis)
            load_row.addWidget(self.load_button)
            load_row.addWidget(self.export_button)
            side.addLayout(load_row)

            self.view_combo = QtWidgets.QComboBox()
            for _, label in PLOT_VIEWS:
                self.view_combo.addItem(label)
            self.view_combo.currentIndexChanged.connect(self.redraw)
            side.addWidget(QtWidgets.QLabel("Plot view"))
            side.addWidget(self.view_combo)

            self.run_combo = QtWidgets.QComboBox()
            self.run_combo.currentIndexChanged.connect(self.redraw)
            side.addWidget(QtWidgets.QLabel("Selected run for detailed plots"))
            side.addWidget(self.run_combo)

            save_row = QtWidgets.QHBoxLayout()
            self.format_combo = QtWidgets.QComboBox()
            self.format_combo.addItems(["png", "svg", "pdf"])
            self.save_current_button = QtWidgets.QPushButton("Save current")
            self.save_current_button.clicked.connect(self.save_current_dialog)
            self.save_all_button = QtWidgets.QPushButton("Save all")
            self.save_all_button.clicked.connect(self.save_all_figures)
            save_row.addWidget(self.format_combo)
            save_row.addWidget(self.save_current_button)
            save_row.addWidget(self.save_all_button)
            side.addLayout(save_row)

            self.status_label = QtWidgets.QLabel("")
            self.status_label.setWordWrap(True)
            side.addWidget(self.status_label)

            self.metrics_table = QtWidgets.QTableWidget()
            self.metrics_table.setColumnCount(6)
            self.metrics_table.setHorizontalHeaderLabels(
                ["Run", "Source", "Status", "Lock %", "Longest lock", "Min gate"]
            )
            self.metrics_table.horizontalHeader().setStretchLastSection(True)
            self.metrics_table.setMinimumHeight(220)
            side.addWidget(self.metrics_table, 1)

            main_panel = QtWidgets.QWidget()
            main = QtWidgets.QVBoxLayout(main_panel)
            main.setContentsMargins(0, 0, 0, 0)
            main.setSpacing(8)
            layout.addWidget(main_panel, 1)
            main.addWidget(self.toolbar)
            main.addWidget(self.canvas, 1)

            self.notes = QtWidgets.QPlainTextEdit()
            self.notes.setReadOnly(True)
            self.notes.setMaximumHeight(160)
            main.addWidget(self.notes)

            self.setStyleSheet(
                """
                QLabel#title { font-size: 20px; font-weight: 700; }
                QFrame { background: #f9fafb; border: 1px solid #e5e7eb; border-radius: 6px; }
                QLineEdit, QPlainTextEdit, QComboBox, QTableWidget {
                    background: white; border: 1px solid #d1d5db; border-radius: 4px; padding: 4px;
                }
                QPushButton { padding: 7px 10px; }
                """
            )

        def browse_file(self, target: QtWidgets.QLineEdit) -> None:
            path, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Open report", str(Path.cwd()), "JSON (*.json)")
            if path:
                target.setText(path)

        def browse_output_dir(self) -> None:
            path = QtWidgets.QFileDialog.getExistingDirectory(self, "Output directory", str(Path.cwd()))
            if path:
                self.output_edit.setText(path)

        def load_reports(self) -> None:
            try:
                sim_path = Path(self.sim_edit.text()).expanduser()
                hardware_paths = [
                    Path(line.strip()).expanduser()
                    for line in self.hardware_edit.toPlainText().splitlines()
                    if line.strip()
                ]
                paths = [sim_path, *hardware_paths]
                missing = [path for path in paths if not path.exists()]
                if missing:
                    raise FileNotFoundError("Missing reports:\n" + "\n".join(str(path) for path in missing))
                self.output_dir = Path(self.output_edit.text()).expanduser()
                loaded: list[LoadedReport] = []
                for path in paths:
                    report = load_report(path)
                    loaded.append(LoadedReport(path, report, compute_metrics(report)))
                self.loaded = loaded
                self.populate_run_combo()
                self.populate_metrics_table()
                self.update_notes()
                self.status_label.setText(f"Loaded {len(self.loaded)} reports.")
                self.redraw()
            except Exception as exc:  # noqa: BLE001
                QtWidgets.QMessageBox.critical(self, "Load failed", str(exc))
                self.status_label.setText(f"Load failed: {exc}")

        def populate_run_combo(self) -> None:
            self.run_combo.blockSignals(True)
            self.run_combo.clear()
            for item in self.loaded:
                self.run_combo.addItem(item.label)
            self.run_combo.blockSignals(False)

        def populate_metrics_table(self) -> None:
            self.metrics_table.setRowCount(len(self.loaded))
            for row, item in enumerate(self.loaded):
                metric = item.metrics
                values = [
                    metric.run_id,
                    metric.source,
                    metric.status,
                    fmt(metric.reference_coverage_pct, "%"),
                    fmt(metric.longest_reference_window_s, "s"),
                    fmt(metric.min_chosen_gate_distance_m, "m"),
                ]
                for col, value in enumerate(values):
                    cell = QtWidgets.QTableWidgetItem(value)
                    cell.setFlags(cell.flags() & ~QtCore.Qt.ItemIsEditable)
                    self.metrics_table.setItem(row, col, cell)
            self.metrics_table.resizeColumnsToContents()

        def update_notes(self) -> None:
            hardware = [item.metrics for item in self.loaded if item.metrics.source == "hardware"]
            sim = next((item.metrics for item in self.loaded if item.metrics.source == "simulation"), None)
            avg_lock = average([item.reference_coverage_pct for item in hardware])
            avg_ratio = average([item.reference_over_candidate_pct for item in hardware])
            lines = [
                "Confronto coerente per unstructured: candidate gates, lock/reference, reacquisizione, clearance e timing.",
                "Il confronto XY punto-a-punto non e' usato perche i gate sono generati dinamicamente dal LiDAR.",
            ]
            if sim is not None:
                lines.append(
                    f"Simulazione: {sim.run_id}, status={sim.status}, durata={fmt(sim.mission_duration_s, 's')}, "
                    f"lock coverage={fmt(sim.reference_coverage_pct, '%')}."
                )
            if hardware:
                lines.append(
                    f"Hardware: lock coverage media={fmt(avg_lock, '%')}, "
                    f"reference/candidate medio={fmt(avg_ratio, '%')}."
                )
            self.notes.setPlainText("\n".join(lines))

        def current_view_id(self) -> str:
            index = self.view_combo.currentIndex()
            return PLOT_VIEWS[index][0] if 0 <= index < len(PLOT_VIEWS) else PLOT_VIEWS[0][0]

        def selected_loaded(self) -> LoadedReport | None:
            index = self.run_combo.currentIndex()
            if 0 <= index < len(self.loaded):
                return self.loaded[index]
            return self.loaded[0] if self.loaded else None

        def redraw(self) -> None:
            if not self.loaded:
                return
            self.figure.clear()
            view_id = self.current_view_id()
            plotters: dict[str, Callable[[Figure], None]] = {
                "overview": self.plot_overview,
                "reference_windows": self.plot_reference_windows,
                "lock_quality": self.plot_lock_quality,
                "lock_timing": self.plot_lock_timing,
                "gate_distance_overlay": self.plot_gate_distance_overlay,
                "selected_gate_state": self.plot_selected_gate_state,
                "motion_command": self.plot_motion_command,
                "lidar_clearance": self.plot_lidar_clearance,
                "loop_timing": self.plot_loop_timing,
                "trajectory_xy": self.plot_trajectory_xy,
            }
            plotters.get(view_id, self.plot_overview)(self.figure)
            self.canvas.draw_idle()

        def plot_overview(self, fig: Figure) -> None:
            metrics = [item.metrics for item in self.loaded]
            labels = short_labels(metrics)
            colors = [source_color(metric) for metric in metrics]
            axes = fig.subplots(2, 2)
            bar_plot(axes[0][0], labels, [m.mission_duration_s for m in metrics], colors, "Mission duration", "s")
            bar_plot(axes[0][1], labels, [m.reference_coverage_pct for m in metrics], colors, "Reference coverage", "%")
            bar_plot(axes[1][0], labels, [m.longest_reference_window_s for m in metrics], colors, "Longest lock window", "s")
            bar_plot(axes[1][1], labels, [m.min_chosen_gate_distance_m for m in metrics], colors, "Min chosen-gate distance", "m")
            fig.suptitle("Unstructured dynamic-gate validation overview")

        def plot_reference_windows(self, fig: Figure) -> None:
            ax = fig.add_subplot(111)
            y_positions = list(range(len(self.loaded)))
            for y, item in zip(y_positions, self.loaded):
                metric = item.metrics
                origin = metric.telemetry_start_s or 0.0
                duration = metric.telemetry_duration_s or 0.0
                ax.hlines(y, 0.0, duration, color="#d1d5db", linewidth=3)
                for start, end in boolean_windows(item.report, "candidate_gates"):
                    ax.broken_barh([(start - origin, end - start)], (y - 0.25, 0.18), facecolors=DEFAULT_COLORS["candidate"])
                for start, end in boolean_windows(item.report, "planner_has_reference"):
                    ax.broken_barh([(start - origin, end - start)], (y - 0.08, 0.28), facecolors=DEFAULT_COLORS["reference"])
                if metric.first_completion_rel_s is not None:
                    ax.vlines(metric.first_completion_rel_s, y - 0.36, y + 0.36, color=DEFAULT_COLORS["completion"], linewidth=2)
            ax.set_yticks(y_positions)
            ax.set_yticklabels([item.metrics.run_id for item in self.loaded])
            ax.set_xlabel("relative time [s]")
            ax.set_title("Candidate and locked-reference windows")
            ax.grid(axis="x", color="#e5e7eb")
            ax.legend(handles=legend_handles([
                ("candidate", DEFAULT_COLORS["candidate"]),
                ("reference", DEFAULT_COLORS["reference"]),
                ("completion", DEFAULT_COLORS["completion"]),
            ]), loc="lower right")

        def plot_lock_quality(self, fig: Figure) -> None:
            ax = fig.add_subplot(111)
            metrics = [item.metrics for item in self.loaded]
            grouped_bars(
                ax,
                short_labels(metrics),
                [
                    ("candidate coverage", [m.candidate_coverage_pct for m in metrics], DEFAULT_COLORS["purple"]),
                    ("reference coverage", [m.reference_coverage_pct for m in metrics], DEFAULT_COLORS["reference"]),
                    ("reference/candidate", [m.reference_over_candidate_pct for m in metrics], DEFAULT_COLORS["green"]),
                ],
                "Lock quality [%]",
            )

        def plot_lock_timing(self, fig: Figure) -> None:
            ax = fig.add_subplot(111)
            metrics = [item.metrics for item in self.loaded]
            grouped_bars(
                ax,
                short_labels(metrics),
                [
                    ("first lock", [m.first_lock_rel_s for m in metrics], DEFAULT_COLORS["purple"]),
                    ("longest lock", [m.longest_reference_window_s for m in metrics], DEFAULT_COLORS["reference"]),
                    ("max reacquisition gap", [m.longest_reacquisition_gap_s for m in metrics], DEFAULT_COLORS["orange"]),
                ],
                "Lock timing [s]",
            )

        def plot_gate_distance_overlay(self, fig: Figure) -> None:
            ax = fig.add_subplot(111)
            for item in self.loaded:
                points = normalized_points(item.report, "chosen_gate_distance", valid_nonnegative=True)
                if not points:
                    continue
                xs, ys = zip(*points)
                ax.plot(xs, ys, label=item.label, color=source_color(item.metrics), linewidth=2.0, alpha=0.88)
            ax.set_xlabel("normalized exported window")
            ax.set_ylabel("chosen gate distance [m]")
            ax.set_title("Chosen dynamic-gate distance")
            ax.grid(True, color="#e5e7eb")
            ax.legend()

        def plot_selected_gate_state(self, fig: Figure) -> None:
            item = self.selected_loaded()
            if item is None:
                return
            ax = fig.add_subplot(111)
            for key, color, label in [
                ("candidate_gates", DEFAULT_COLORS["purple"], "candidate gates"),
                ("planner_has_reference", DEFAULT_COLORS["reference"], "planner reference"),
                ("chosen_gate_index", DEFAULT_COLORS["orange"], "chosen index"),
            ]:
                points = relative_points(item.report, key)
                if points:
                    xs, ys = zip(*points)
                    ax.step(xs, ys, where="post", label=label, color=color, linewidth=2.0)
            if item.metrics.first_completion_rel_s is not None:
                ax.axvline(item.metrics.first_completion_rel_s, color=DEFAULT_COLORS["completion"], linestyle="--", label="completion")
            ax.set_xlabel("relative time [s]")
            ax.set_ylabel("count / flag / index")
            ax.set_title(f"{item.metrics.run_id} dynamic gate state")
            ax.grid(True, color="#e5e7eb")
            ax.legend()

        def plot_motion_command(self, fig: Figure) -> None:
            item = self.selected_loaded()
            if item is None:
                return
            ax = fig.add_subplot(111)
            for key, color, label in [
                ("speed", DEFAULT_COLORS["reference"], "speed"),
                ("target_speed", DEFAULT_COLORS["cyan"], "target speed"),
                ("yaw_rate", DEFAULT_COLORS["completion"], "yaw rate"),
                ("target_yaw_rate", DEFAULT_COLORS["orange"], "target yaw rate"),
            ]:
                points = relative_points(item.report, key)
                if points:
                    xs, ys = zip(*points)
                    ax.plot(xs, ys, label=label, color=color, linewidth=1.8)
            ax.set_xlabel("relative time [s]")
            ax.set_ylabel("m/s and rad/s")
            ax.set_title(f"{item.metrics.run_id} motion command tracking")
            ax.grid(True, color="#e5e7eb")
            ax.legend()

        def plot_lidar_clearance(self, fig: Figure) -> None:
            item = self.selected_loaded()
            if item is None:
                return
            ax = fig.add_subplot(111)
            for key, color, label in [
                ("min_lidar", DEFAULT_COLORS["reference"], "min LiDAR"),
                ("front_lidar", DEFAULT_COLORS["completion"], "front LiDAR"),
            ]:
                points = relative_points(item.report, key, valid_nonnegative=True)
                if points:
                    xs, ys = zip(*points)
                    ax.plot(xs, ys, label=label, color=color, linewidth=1.9)
            if not ax.lines:
                ax.text(0.5, 0.5, "This report does not export min_lidar/front_lidar.", ha="center", va="center", transform=ax.transAxes)
            ax.set_xlabel("relative time [s]")
            ax.set_ylabel("clearance [m]")
            ax.set_title(f"{item.metrics.run_id} LiDAR clearance")
            ax.grid(True, color="#e5e7eb")
            if ax.lines:
                ax.legend()

        def plot_loop_timing(self, fig: Figure) -> None:
            ax = fig.add_subplot(111)
            metrics = [item.metrics for item in self.loaded]
            grouped_bars(
                ax,
                short_labels(metrics),
                [
                    ("avg planning", [m.avg_planning_ms for m in metrics], DEFAULT_COLORS["reference"]),
                    ("avg lidar", [m.avg_lidar_ms for m in metrics], DEFAULT_COLORS["green"]),
                    ("avg step", [m.avg_step_ms for m in metrics], DEFAULT_COLORS["completion"]),
                ],
                "Loop timing [ms]",
            )

        def plot_trajectory_xy(self, fig: Figure) -> None:
            ax = fig.add_subplot(111)
            for item in self.loaded:
                points = xy_points(item.report)
                if not points:
                    continue
                xs, ys = zip(*points)
                ax.plot(xs, ys, label=item.label, color=source_color(item.metrics), linewidth=2.0, alpha=0.85)
                ax.scatter([xs[0]], [ys[0]], color=source_color(item.metrics), marker="o", s=28)
                ax.scatter([xs[-1]], [ys[-1]], color=source_color(item.metrics), marker="x", s=42)
            ax.set_xlabel("x [m]")
            ax.set_ylabel("y [m]")
            ax.set_title("Estimated / simulated trajectories")
            ax.grid(True, color="#e5e7eb")
            ax.axis("equal")
            ax.legend()

        def save_current_dialog(self) -> None:
            fmt_name = self.format_combo.currentText()
            default_name = f"{self.current_view_id()}.{fmt_name}"
            path, _ = QtWidgets.QFileDialog.getSaveFileName(
                self,
                "Save current figure",
                str(self.output_dir / "matplotlib_figures" / default_name),
                "Images (*.png *.svg *.pdf)",
            )
            if path:
                self.figure.savefig(path, dpi=220, bbox_inches="tight")
                self.status_label.setText(f"Saved {path}")

        def save_all_figures(self) -> None:
            if not self.loaded:
                return
            fmt_name = self.format_combo.currentText()
            output_dir = Path(self.output_edit.text()).expanduser() / "matplotlib_figures"
            output_dir.mkdir(parents=True, exist_ok=True)
            original_view = self.view_combo.currentIndex()
            original_run = self.run_combo.currentIndex()
            saved = 0
            detailed_views = {"selected_gate_state", "motion_command", "lidar_clearance"}
            for view_index, (view_id, _) in enumerate(PLOT_VIEWS):
                self.view_combo.setCurrentIndex(view_index)
                if view_id in detailed_views:
                    for run_index, item in enumerate(self.loaded):
                        self.run_combo.setCurrentIndex(run_index)
                        self.redraw()
                        self.figure.savefig(output_dir / f"{view_id}_{item.metrics.run_id}.{fmt_name}", dpi=220, bbox_inches="tight")
                        saved += 1
                else:
                    self.redraw()
                    self.figure.savefig(output_dir / f"{view_id}.{fmt_name}", dpi=220, bbox_inches="tight")
                    saved += 1
            self.view_combo.setCurrentIndex(original_view)
            self.run_combo.setCurrentIndex(original_run)
            self.redraw()
            self.status_label.setText(f"Saved {saved} Matplotlib figures in {output_dir}")

        def export_analysis(self) -> None:
            if not self.loaded:
                return
            output_dir = Path(self.output_edit.text()).expanduser()
            output_dir.mkdir(parents=True, exist_ok=True)
            metrics = [item.metrics for item in self.loaded]
            paths = [item.path for item in self.loaded]
            write_metrics_csv(output_dir / "dynamic_gate_metrics.csv", metrics)
            (output_dir / "dynamic_gate_metrics.json").write_text(
                json.dumps([asdict(metric) for metric in metrics], indent=2),
                encoding="utf-8",
            )
            write_thesis_markdown(output_dir / "thesis_unstructured_dynamic_gate_analysis.md", metrics, paths)
            self.status_label.setText(f"Exported metrics and thesis markdown to {output_dir}")


def relative_points(report: Any, key: str, *, valid_nonnegative: bool = False) -> list[tuple[float, float]]:
    points = time_series(report, key, valid_nonnegative=valid_nonnegative)
    if not points:
        return []
    origin = points[0][0]
    return [(time - origin, value) for time, value in points]


def normalized_points(report: Any, key: str, *, valid_nonnegative: bool = False) -> list[tuple[float, float]]:
    points = time_series(report, key, valid_nonnegative=valid_nonnegative)
    if len(points) < 2:
        return []
    start = points[0][0]
    duration = points[-1][0] - start
    if duration <= 0.0:
        return []
    return [((time - start) / duration, value) for time, value in points]


def xy_points(report: Any) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    for sample in report.history:
        x_value = as_float(sample.get("x"))
        y_value = as_float(sample.get("y"))
        if x_value is None or y_value is None:
            x_value = as_float(sample.get("position_x"))
            y_value = as_float(sample.get("position_y"))
        if x_value is not None and y_value is not None:
            points.append((x_value, y_value))
    return points


def source_color(metric: DynamicGateMetrics) -> str:
    if metric.source == "simulation":
        return DEFAULT_COLORS["simulation"]
    return DEFAULT_COLORS["hardware"]


def short_labels(metrics: list[DynamicGateMetrics]) -> list[str]:
    return [metric.run_id[-11:] for metric in metrics]


def grouped_bars(ax: Any, labels: list[str], groups: list[tuple[str, list[float | None], str]], ylabel: str) -> None:
    x_positions = list(range(len(labels)))
    width = 0.78 / max(len(groups), 1)
    for group_index, (label, values, color) in enumerate(groups):
        offset = (group_index - (len(groups) - 1) / 2.0) * width
        clean_values = [0.0 if value is None else value for value in values]
        bars = ax.bar([x + offset for x in x_positions], clean_values, width=width, label=label, color=color)
        for bar, value in zip(bars, values):
            if value is None:
                continue
            ax.text(bar.get_x() + bar.get_width() / 2.0, bar.get_height(), fmt(value), ha="center", va="bottom", fontsize=8)
    ax.set_xticks(x_positions)
    ax.set_xticklabels(labels, rotation=20, ha="right")
    ax.set_ylabel(ylabel)
    ax.grid(axis="y", color="#e5e7eb")
    ax.legend()


def bar_plot(ax: Any, labels: list[str], values: list[float | None], colors: list[str], title: str, ylabel: str) -> None:
    clean_values = [0.0 if value is None else value for value in values]
    bars = ax.bar(labels, clean_values, color=colors)
    for bar, value in zip(bars, values):
        if value is None:
            continue
        ax.text(bar.get_x() + bar.get_width() / 2.0, bar.get_height(), fmt(value), ha="center", va="bottom", fontsize=8)
    ax.set_title(title)
    ax.set_ylabel(ylabel)
    ax.tick_params(axis="x", rotation=20)
    ax.grid(axis="y", color="#e5e7eb")


def legend_handles(items: list[tuple[str, str]]) -> list[Any]:
    from matplotlib.lines import Line2D

    return [Line2D([0], [0], color=color, lw=4, label=label) for label, color in items]


def average(values: list[float | None]) -> float | None:
    clean = [value for value in values if value is not None]
    return sum(clean) / len(clean) if clean else None


def fmt(value: float | None, unit: str = "") -> str:
    if value is None:
        return "n/a"
    absolute = abs(value)
    if absolute >= 100:
        text = f"{value:.1f}"
    elif absolute >= 10:
        text = f"{value:.2f}"
    else:
        text = f"{value:.3f}"
    return f"{text} {unit}" if unit else text


if __name__ == "__main__":
    raise SystemExit(main())
