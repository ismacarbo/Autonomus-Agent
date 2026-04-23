from __future__ import annotations

import argparse
import glob
import sys
from dataclasses import asdict
from pathlib import Path
from typing import Any

if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from data_analisys.report_analysis import (  # noqa: E402
    RunReport,
    as_float,
    first_gate_completion_time,
    load_report,
    summarize_report,
    time_series,
    xy_series,
)
from data_analisys.model_validation import (  # noqa: E402
    DEFAULT_AGGREGATE_OUTPUT_DIR,
    DEFAULT_OUTPUT_DIR as DEFAULT_VALIDATION_OUTPUT_DIR,
    DEFAULT_ROBOT_RUN,
    DEFAULT_SIM_BASELINE,
    WINDOW_MODES,
    error_series,
    normalize_time,
    normalized_distance_to_goal,
    resolve_signal,
    run_multi_run_validation,
    run_model_validation,
    signal_series,
    yaw_rate_series,
)
from data_analisys.validation_presets import (  # noqa: E402
    DEFAULT_PRESET,
    list_presets,
    load_preset_config,
)

try:
    from PyQt5 import QtCore, QtGui, QtWidgets
except ImportError:
    QtCore = None
    QtGui = None
    QtWidgets = None


DEFAULT_REPORTS_DIR = Path("reports")
CHARTS = [
    ("gate_distance", "Gate distance"),
    ("gate_reference", "Gate reference"),
    ("lidar_clearance", "LiDAR clearance"),
    ("lidar_samples", "Close samples"),
    ("controller_pwm", "PWM"),
    ("controller_status", "Controller"),
    ("motion_state", "Motion"),
    ("timing", "Timing"),
    ("trajectory_xy", "Trajectory XY"),
]
COLORS = [
    "#1f77b4",
    "#d62728",
    "#2ca02c",
    "#9467bd",
    "#ff7f0e",
    "#17becf",
    "#4c566a",
]
VALIDATION_PRESETS = list_presets()


def validation_notes_text(config: dict[str, Any] | None) -> str:
    lines: list[str] = []
    if config:
        label = str(config.get("label") or "")
        description = str(config.get("description") or "")
        if label:
            lines.append(f"Preset: {label}")
        if description:
            lines.append(description)
        preset_notes = [str(note) for note in config.get("notes") or []]
        if preset_notes:
            lines.append("")
            lines.append("Note scenario:")
            lines.extend(f"- {note}" for note in preset_notes)
        lines.append("")
    lines.extend(
        [
            "Fitting attuale:",
            "y[k+1] = a*y[k] + b*u[k] + c",
            "",
            "Per velocita: u e il PWM medio assoluto.",
            "Per yaw-rate: u e il PWM differenziale.",
            "c rappresenta un bias statico: attrito, trim, offset o drift residuo.",
            "La stima e una regressione least-squares discreta di primo ordine.",
        ]
    )
    return "\n".join(lines)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Open a PyQt5 desktop viewer for hardware/planner test reports.",
    )
    parser.add_argument("--reports-dir", type=Path, default=DEFAULT_REPORTS_DIR)
    parser.add_argument("--open", type=Path, default=None, help="Report JSON to open at startup.")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if QtWidgets is None:
        print(
            "PyQt5 is not installed. Install it with:\n"
            "  python -m pip install -r data_analisys/requirements.txt\n"
            "or:\n"
            "  python -m pip install PyQt5",
            file=sys.stderr,
        )
        return 2

    app = QtWidgets.QApplication(sys.argv[:1])
    app.setApplicationName("Autonomous Agent Test Report Viewer")
    window = ReportViewerWindow(args.reports_dir.resolve())
    window.resize(1420, 860)
    window.show()
    if args.open:
        window.load_report_path(args.open)
    return int(app.exec_())


class ReportItem:
    def __init__(self, path: Path, report: RunReport | None, error: str | None = None):
        self.path = path
        self.report = report
        self.error = error
        self.summary = summarize_report(report) if report is not None else None
        self.scenario = infer_scenario(path.name)

    @property
    def run_id(self) -> str:
        return self.summary.run_id if self.summary is not None else self.path.stem

    @property
    def samples(self) -> int:
        return self.summary.samples if self.summary is not None else 0


if QtWidgets is not None:

    class ReportViewerWindow(QtWidgets.QMainWindow):
        def __init__(self, reports_dir: Path):
            super().__init__()
            self.reports_dir = reports_dir
            self.report_items: list[ReportItem] = []
            self.current_item: ReportItem | None = None
            self.validation_sim: RunReport | None = None
            self.validation_robot: RunReport | None = None
            self.validation_comparison: dict[str, Any] | None = None
            self.validation_fits: dict[str, Any] | None = None
            self.validation_presets = VALIDATION_PRESETS

            self.setWindowTitle("Test Report Viewer - PyQt5")
            self._build_ui()
            self.apply_validation_preset(DEFAULT_PRESET)
            self.refresh_reports()

        def _build_ui(self) -> None:
            central = QtWidgets.QWidget()
            self.setCentralWidget(central)
            root = QtWidgets.QHBoxLayout(central)
            root.setContentsMargins(12, 12, 12, 12)
            root.setSpacing(12)

            sidebar = QtWidgets.QFrame()
            sidebar.setObjectName("sidebar")
            sidebar.setMinimumWidth(350)
            sidebar.setMaximumWidth(440)
            sidebar_layout = QtWidgets.QVBoxLayout(sidebar)
            sidebar_layout.setContentsMargins(12, 12, 12, 12)
            sidebar_layout.setSpacing(10)
            root.addWidget(sidebar)

            title = QtWidgets.QLabel("Report Viewer")
            title.setObjectName("title")
            sidebar_layout.addWidget(title)
            self.reports_dir_label = QtWidgets.QLabel(str(self.reports_dir))
            self.reports_dir_label.setObjectName("muted")
            self.reports_dir_label.setWordWrap(True)
            sidebar_layout.addWidget(self.reports_dir_label)

            self.filter_edit = QtWidgets.QLineEdit()
            self.filter_edit.setPlaceholderText("Filtra per run, scenario, data")
            self.filter_edit.textChanged.connect(self.populate_report_list)
            sidebar_layout.addWidget(self.filter_edit)

            buttons = QtWidgets.QHBoxLayout()
            self.refresh_button = QtWidgets.QPushButton("Refresh")
            self.refresh_button.clicked.connect(self.refresh_reports)
            self.open_button = QtWidgets.QPushButton("Apri JSON")
            self.open_button.clicked.connect(self.open_report_dialog)
            buttons.addWidget(self.refresh_button)
            buttons.addWidget(self.open_button)
            sidebar_layout.addLayout(buttons)

            self.path_edit = QtWidgets.QLineEdit()
            self.path_edit.setPlaceholderText("Percorso JSON opzionale")
            self.path_edit.returnPressed.connect(self.open_path_from_edit)
            sidebar_layout.addWidget(self.path_edit)

            quick_buttons = QtWidgets.QHBoxLayout()
            self.hardware_button = QtWidgets.QPushButton("hardware")
            self.hardware_button.clicked.connect(lambda: self.set_filter_token("hardware"))
            self.structured_button = QtWidgets.QPushButton("structured")
            self.structured_button.clicked.connect(lambda: self.set_filter_token("structured"))
            self.unstructured_button = QtWidgets.QPushButton("unstructured")
            self.unstructured_button.clicked.connect(lambda: self.set_filter_token("unstructured"))
            quick_buttons.addWidget(self.hardware_button)
            quick_buttons.addWidget(self.structured_button)
            quick_buttons.addWidget(self.unstructured_button)
            sidebar_layout.addLayout(quick_buttons)

            self.report_list = QtWidgets.QListWidget()
            self.report_list.currentRowChanged.connect(self.report_row_changed)
            sidebar_layout.addWidget(self.report_list, 1)

            main_panel = QtWidgets.QWidget()
            main_layout = QtWidgets.QVBoxLayout(main_panel)
            main_layout.setContentsMargins(0, 0, 0, 0)
            main_layout.setSpacing(12)
            root.addWidget(main_panel, 1)

            self.run_title = QtWidgets.QLabel("Nessun report selezionato")
            self.run_title.setObjectName("title")
            main_layout.addWidget(self.run_title)
            self.status_label = QtWidgets.QLabel("Scegli un report dalla lista o apri un JSON.")
            self.status_label.setObjectName("muted")
            self.status_label.setWordWrap(True)
            main_layout.addWidget(self.status_label)

            self.workspace_tabs = QtWidgets.QTabWidget()
            main_layout.addWidget(self.workspace_tabs, 1)

            report_tab = QtWidgets.QWidget()
            report_layout = QtWidgets.QVBoxLayout(report_tab)
            report_layout.setContentsMargins(0, 0, 0, 0)
            report_layout.setSpacing(12)
            self.workspace_tabs.addTab(report_tab, "Report analysis")

            self.cards_layout = QtWidgets.QGridLayout()
            self.cards_layout.setSpacing(8)
            self.cards: dict[str, tuple[Any, Any]] = {}
            card_specs = [
                ("Samples", "samples"),
                ("Duration", "duration_s"),
                ("Gate completion", "first_gate_completion_s"),
                ("Longest reference", "longest_reference_window_s"),
                ("Min LiDAR", "min_lidar_m"),
                ("Status flags", "final_controller_status_flags"),
            ]
            for index, (label, key) in enumerate(card_specs):
                card = QtWidgets.QFrame()
                card.setObjectName("card")
                layout = QtWidgets.QVBoxLayout(card)
                layout.setContentsMargins(10, 8, 10, 8)
                label_widget = QtWidgets.QLabel(label)
                label_widget.setObjectName("muted")
                value_widget = QtWidgets.QLabel("-")
                value_widget.setObjectName("metric")
                layout.addWidget(label_widget)
                layout.addWidget(value_widget)
                self.cards[key] = (label_widget, value_widget)
                self.cards_layout.addWidget(card, 0, index)
            report_layout.addLayout(self.cards_layout)

            self.chart_tabs = QtWidgets.QTabBar()
            self.chart_tabs.setExpanding(False)
            for _, label in CHARTS:
                self.chart_tabs.addTab(label)
            self.chart_tabs.currentChanged.connect(self.chart_changed)
            report_layout.addWidget(self.chart_tabs)

            self.chart_canvas = ChartCanvas()
            report_layout.addWidget(self.chart_canvas, 1)

            details_box = QtWidgets.QFrame()
            details_box.setObjectName("details")
            details_layout = QtWidgets.QGridLayout(details_box)
            details_layout.setContentsMargins(10, 8, 10, 8)
            details_layout.setSpacing(10)
            self.detail_labels: dict[str, Any] = {}
            detail_names = [
                "Path",
                "Controller error",
                "Reference windows",
                "Min front LiDAR",
                "Avg planning",
                "Avg LiDAR",
                "Avg step",
                "Status",
            ]
            for index, key in enumerate(detail_names):
                key_label = QtWidgets.QLabel(key)
                key_label.setObjectName("muted")
                value_label = QtWidgets.QLabel("-")
                value_label.setWordWrap(True)
                row = index // 4
                col = (index % 4) * 2
                details_layout.addWidget(key_label, row, col)
                details_layout.addWidget(value_label, row, col + 1)
                self.detail_labels[key] = value_label
            report_layout.addWidget(details_box)

            self.validation_tab = self.build_validation_tab()
            self.workspace_tabs.addTab(self.validation_tab, "Model validation")

            self.setStyleSheet(
                """
                QMainWindow { background: #f5f7fb; }
                QFrame#sidebar { background: #eef2f7; border: 1px solid #d7dce5; border-radius: 8px; }
                QLabel#title { font-size: 20px; font-weight: 750; color: #111827; }
                QLabel#muted { color: #6b7280; font-size: 12px; }
                QLabel#metric { font-size: 19px; font-weight: 750; color: #111827; }
                QFrame#card, QFrame#details { background: #ffffff; border: 1px solid #d7dce5; border-radius: 8px; }
                QListWidget { background: #ffffff; border: 1px solid #d7dce5; border-radius: 8px; }
                QListWidget::item { padding: 8px; border-bottom: 1px solid #edf0f5; }
                QListWidget::item:selected { background: #dceeff; color: #111827; }
                QLineEdit { background: #ffffff; border: 1px solid #d7dce5; border-radius: 6px; padding: 8px; }
                QPushButton { background: #ffffff; border: 1px solid #d7dce5; border-radius: 6px; padding: 8px; }
                QPushButton:hover { background: #f7fafc; }
                QTabBar::tab { background: #ffffff; border: 1px solid #d7dce5; border-radius: 6px; padding: 8px 11px; margin-right: 6px; }
                QTabBar::tab:selected { background: #1f77b4; border-color: #1f77b4; color: #ffffff; }
                """
            )

        def build_validation_tab(self):
            tab = QtWidgets.QWidget()
            layout = QtWidgets.QVBoxLayout(tab)
            layout.setContentsMargins(0, 0, 0, 0)
            layout.setSpacing(10)

            controls = QtWidgets.QFrame()
            controls.setObjectName("details")
            controls_layout = QtWidgets.QGridLayout(controls)
            controls_layout.setContentsMargins(10, 8, 10, 8)
            controls_layout.setSpacing(8)

            self.sim_baseline_edit = QtWidgets.QLineEdit(str(DEFAULT_SIM_BASELINE))
            self.robot_run_edit = QtWidgets.QLineEdit(str(DEFAULT_ROBOT_RUN))
            self.validation_output_edit = QtWidgets.QLineEdit(str(DEFAULT_VALIDATION_OUTPUT_DIR))
            self.aggregate_glob_edit = QtWidgets.QLineEdit("reports/thesis_hardware_unstructured_manual_gate_editor_gui_manual_20260418*.json")
            self.aggregate_output_edit = QtWidgets.QLineEdit(str(DEFAULT_AGGREGATE_OUTPUT_DIR))
            self.analysis_window_combo = QtWidgets.QComboBox()
            self.analysis_window_combo.addItems(list(WINDOW_MODES))
            self.analysis_window_combo.setCurrentText("until_first_gate")
            self.validation_preset_combo = QtWidgets.QComboBox()
            for preset in self.validation_presets:
                self.validation_preset_combo.addItem(preset.label, preset.name)
            self.validation_preset_combo.currentIndexChanged.connect(lambda _: self.validation_preset_changed())
            self.custom_start_spin = QtWidgets.QDoubleSpinBox()
            self.custom_start_spin.setRange(0.0, 100000.0)
            self.custom_start_spin.setDecimals(3)
            self.custom_start_spin.setValue(0.0)
            self.custom_end_spin = QtWidgets.QDoubleSpinBox()
            self.custom_end_spin.setRange(0.0, 100000.0)
            self.custom_end_spin.setDecimals(3)
            self.custom_end_spin.setValue(30.0)
            self.grid_size_spin = QtWidgets.QSpinBox()
            self.grid_size_spin.setRange(20, 5000)
            self.grid_size_spin.setValue(250)

            controls_layout.addWidget(QtWidgets.QLabel("Preset"), 0, 0)
            controls_layout.addWidget(self.validation_preset_combo, 0, 1)
            controls_layout.addWidget(QtWidgets.QLabel("Simulation baseline"), 1, 0)
            controls_layout.addWidget(self.sim_baseline_edit, 1, 1)
            controls_layout.addWidget(self.make_button("Browse", self.browse_sim_baseline), 1, 2)
            controls_layout.addWidget(self.make_button("Use selected", self.use_selected_as_sim), 1, 3)
            controls_layout.addWidget(QtWidgets.QLabel("Robot run"), 2, 0)
            controls_layout.addWidget(self.robot_run_edit, 2, 1)
            controls_layout.addWidget(self.make_button("Browse", self.browse_robot_run), 2, 2)
            controls_layout.addWidget(self.make_button("Use selected", self.use_selected_as_robot), 2, 3)
            controls_layout.addWidget(QtWidgets.QLabel("Output dir"), 3, 0)
            controls_layout.addWidget(self.validation_output_edit, 3, 1)
            controls_layout.addWidget(self.make_button("Browse", self.browse_validation_output), 3, 2)
            controls_layout.addWidget(self.make_button("Open folder", self.open_validation_output), 3, 3)
            controls_layout.addWidget(QtWidgets.QLabel("Analysis window"), 4, 0)
            controls_layout.addWidget(self.analysis_window_combo, 4, 1)
            controls_layout.addWidget(QtWidgets.QLabel("Custom start/end"), 4, 2)
            custom_times = QtWidgets.QHBoxLayout()
            custom_times.addWidget(self.custom_start_spin)
            custom_times.addWidget(self.custom_end_spin)
            controls_layout.addLayout(custom_times, 4, 3)
            controls_layout.addWidget(QtWidgets.QLabel("Grid size"), 5, 0)
            controls_layout.addWidget(self.grid_size_spin, 5, 1)
            self.run_validation_button = self.make_button("Run validation", self.run_validation_from_ui)
            controls_layout.addWidget(self.run_validation_button, 5, 2, 1, 2)
            controls_layout.addWidget(QtWidgets.QLabel("Robot glob"), 6, 0)
            controls_layout.addWidget(self.aggregate_glob_edit, 6, 1)
            controls_layout.addWidget(QtWidgets.QLabel("Aggregate output"), 7, 0)
            controls_layout.addWidget(self.aggregate_output_edit, 7, 1)
            controls_layout.addWidget(self.make_button("Run aggregate", self.run_aggregate_from_ui), 7, 2, 1, 2)
            layout.addWidget(controls)

            self.validation_summary = QtWidgets.QLabel(
                "Seleziona una baseline simulata e una run reale, poi lancia la validazione."
            )
            self.validation_summary.setObjectName("muted")
            self.validation_summary.setWordWrap(True)
            layout.addWidget(self.validation_summary)

            self.validation_plot_combo = QtWidgets.QComboBox()
            self.validation_plot_combo.addItems(
                [
                    "speed",
                    "yaw_rate",
                    "distance_to_goal_norm",
                    "tracking_error",
                    "timing",
                ]
            )
            self.validation_plot_combo.currentTextChanged.connect(self.update_validation_plot)

            plot_panel = QtWidgets.QWidget()
            plot_layout = QtWidgets.QVBoxLayout(plot_panel)
            plot_layout.setContentsMargins(0, 0, 0, 0)
            plot_layout.setSpacing(8)
            plot_layout.addWidget(self.validation_plot_combo)
            self.validation_canvas = ComparisonCanvas()
            plot_layout.addWidget(self.validation_canvas, 1)

            self.validation_result_tabs = QtWidgets.QTabWidget()
            self.validation_metrics_table = QtWidgets.QTableWidget()
            self.validation_fit_table = QtWidgets.QTableWidget()
            self.validation_aggregate_table = QtWidgets.QTableWidget()
            self.validation_notes = QtWidgets.QTextEdit()
            self.validation_notes.setReadOnly(True)
            self.validation_notes.setPlainText(validation_notes_text(None))
            self.validation_result_tabs.addTab(self.validation_metrics_table, "Metrics")
            self.validation_result_tabs.addTab(self.validation_fit_table, "Fits")
            self.validation_result_tabs.addTab(self.validation_aggregate_table, "Aggregate")
            self.validation_result_tabs.addTab(self.validation_notes, "Fitting notes")
            self.validation_result_tabs.setMinimumHeight(250)

            splitter = QtWidgets.QSplitter(QtCore.Qt.Vertical)
            splitter.setChildrenCollapsible(False)
            splitter.addWidget(plot_panel)
            splitter.addWidget(self.validation_result_tabs)
            splitter.setStretchFactor(0, 3)
            splitter.setStretchFactor(1, 2)
            layout.addWidget(splitter, 1)
            return tab

        def selected_preset_name(self) -> str:
            preset_name = self.validation_preset_combo.currentData()
            return str(preset_name or DEFAULT_PRESET)

        def selected_preset_config(self) -> dict[str, Any]:
            return load_preset_config(self.selected_preset_name())

        def validation_preset_changed(self) -> None:
            self.apply_validation_preset(self.selected_preset_name())

        def apply_validation_preset(self, preset_name: str) -> None:
            config = load_preset_config(preset_name)
            simulation_baseline = config.get("simulation_baseline")
            robot_run = config.get("robot_validation_run")
            output_dir = config.get("output_dir")
            aggregate_output_dir = config.get("aggregate_output_dir")
            robot_glob = config.get("default_robot_glob")
            analysis_window = config.get("default_analysis_window")

            if simulation_baseline:
                self.sim_baseline_edit.setText(str(simulation_baseline))
            if robot_run:
                self.robot_run_edit.setText(str(robot_run))
            if output_dir:
                self.validation_output_edit.setText(str(output_dir))
            if aggregate_output_dir:
                self.aggregate_output_edit.setText(str(aggregate_output_dir))
            if robot_glob:
                self.aggregate_glob_edit.setText(str(robot_glob))
            if analysis_window in WINDOW_MODES:
                self.analysis_window_combo.setCurrentText(str(analysis_window))

            self.validation_notes.setPlainText(validation_notes_text(config))
            label = str(config.get("label") or preset_name)
            description = str(config.get("description") or "")
            self.validation_summary.setText(f"Preset caricato: {label}. {description}".strip())

        def make_button(self, label: str, callback):
            button = QtWidgets.QPushButton(label)
            button.clicked.connect(callback)
            return button

        def refresh_reports(self) -> None:
            self.report_items = []
            if self.reports_dir.exists():
                for path in sorted(self.reports_dir.glob("*.json")):
                    try:
                        report = load_report(path)
                        item = ReportItem(path, report)
                    except Exception as exc:
                        item = ReportItem(path, None, str(exc))
                    self.report_items.append(item)
            self.report_items.sort(key=lambda item: (item.run_id[:1].isdigit(), item.run_id), reverse=True)
            self.populate_report_list()
            self.status_label.setText(f"{len(self.report_items)} report trovati in {self.reports_dir}")

        def populate_report_list(self) -> None:
            self.report_list.blockSignals(True)
            self.report_list.clear()
            query = self.filter_edit.text().strip().lower()
            tokens = [token for token in query.split() if token]
            for item in self.report_items:
                haystack = f"{item.run_id} {item.path.name} {item.scenario}".lower()
                if tokens and not all(token in haystack for token in tokens):
                    continue
                list_item = QtWidgets.QListWidgetItem()
                list_item.setText(self.item_label(item))
                list_item.setData(QtCore.Qt.UserRole, str(item.path))
                if item.error:
                    list_item.setToolTip(item.error)
                self.report_list.addItem(list_item)
            self.report_list.blockSignals(False)

        def item_label(self, item: ReportItem) -> str:
            if item.summary is None:
                return f"{item.run_id}\n{item.scenario} - errore lettura"
            completion = fmt(item.summary.first_gate_completion_s, "s")
            duration = fmt(item.summary.duration_s, "s")
            return f"{item.run_id}\n{item.scenario} - {item.samples} samples - {duration} - gate {completion}"

        def report_row_changed(self, row: int) -> None:
            if row < 0:
                return
            list_item = self.report_list.item(row)
            if list_item is None:
                return
            path = Path(list_item.data(QtCore.Qt.UserRole))
            self.load_report_path(path)

        def load_report_path(self, path: Path) -> None:
            try:
                report = load_report(path)
                item = ReportItem(path, report)
            except Exception as exc:
                self.show_error("Errore lettura report", str(exc))
                return
            self.current_item = item
            self.path_edit.setText(str(path))
            self.render_report()

        def open_report_dialog(self) -> None:
            file_path, _ = QtWidgets.QFileDialog.getOpenFileName(
                self,
                "Apri report JSON",
                str(self.reports_dir),
                "JSON reports (*.json);;All files (*)",
            )
            if file_path:
                self.load_report_path(Path(file_path))

        def open_path_from_edit(self) -> None:
            raw = self.path_edit.text().strip()
            if not raw:
                return
            path = Path(raw)
            if not path.is_absolute():
                candidates = [Path.cwd() / path, self.reports_dir / path]
                path = next((candidate for candidate in candidates if candidate.exists()), path)
            self.load_report_path(path)

        def set_filter_token(self, token: str) -> None:
            current = self.filter_edit.text().strip()
            if token in current.lower().split():
                tokens = [part for part in current.split() if part.lower() != token]
                self.filter_edit.setText(" ".join(tokens))
            else:
                self.filter_edit.setText((current + " " + token).strip())

        def chart_changed(self, index: int) -> None:
            if self.current_item is None:
                return
            chart_id = CHARTS[index][0]
            self.chart_canvas.set_chart(self.current_item.report, chart_id)

        def render_report(self) -> None:
            if self.current_item is None or self.current_item.report is None or self.current_item.summary is None:
                return
            item = self.current_item
            summary = item.summary
            self.run_title.setText(item.run_id)
            self.status_label.setText(str(item.path))
            units = {
                "duration_s": "s",
                "first_gate_completion_s": "s",
                "longest_reference_window_s": "s",
                "min_lidar_m": "m",
            }
            summary_dict = asdict(summary)
            for key, (_, value_widget) in self.cards.items():
                value_widget.setText(fmt(summary_dict.get(key), units.get(key, "")))

            details = {
                "Path": str(item.path),
                "Controller error": fmt(summary.final_controller_error_code),
                "Reference windows": fmt(summary.reference_windows),
                "Min front LiDAR": fmt(summary.min_front_lidar_m, "m"),
                "Avg planning": fmt(summary.avg_planning_ms, "ms"),
                "Avg LiDAR": fmt(summary.avg_lidar_ms, "ms"),
                "Avg step": fmt(summary.avg_step_ms, "ms"),
                "Status": str(item.report.status),
            }
            for key, value in details.items():
                self.detail_labels[key].setText(value)

            chart_id = CHARTS[self.chart_tabs.currentIndex()][0]
            self.chart_canvas.set_chart(item.report, chart_id)

        def show_error(self, title: str, message: str) -> None:
            QtWidgets.QMessageBox.critical(self, title, message)

        def browse_sim_baseline(self) -> None:
            self.browse_report_into(self.sim_baseline_edit)

        def browse_robot_run(self) -> None:
            self.browse_report_into(self.robot_run_edit)

        def browse_report_into(self, edit: Any) -> None:
            file_path, _ = QtWidgets.QFileDialog.getOpenFileName(
                self,
                "Scegli report JSON",
                str(self.reports_dir),
                "JSON reports (*.json);;All files (*)",
            )
            if file_path:
                edit.setText(file_path)

        def browse_validation_output(self) -> None:
            directory = QtWidgets.QFileDialog.getExistingDirectory(
                self,
                "Scegli cartella output",
                str(Path(self.validation_output_edit.text()).parent),
            )
            if directory:
                self.validation_output_edit.setText(directory)

        def use_selected_as_sim(self) -> None:
            if self.current_item is None:
                self.show_error("Nessun report selezionato", "Seleziona un report dalla lista.")
                return
            self.sim_baseline_edit.setText(str(self.current_item.path))

        def use_selected_as_robot(self) -> None:
            if self.current_item is None:
                self.show_error("Nessun report selezionato", "Seleziona un report dalla lista.")
                return
            self.robot_run_edit.setText(str(self.current_item.path))

        def run_validation_from_ui(self) -> None:
            sim_path = Path(self.sim_baseline_edit.text().strip())
            robot_path = Path(self.robot_run_edit.text().strip())
            output_dir = Path(self.validation_output_edit.text().strip())
            window_mode, custom_start, custom_end = self.selected_window_params()
            preset_config = self.selected_preset_config()
            if not sim_path.exists():
                self.show_error("Baseline mancante", f"Non trovo il report simulato:\n{sim_path}")
                return
            if not robot_path.exists():
                self.show_error("Run reale mancante", f"Non trovo il report reale:\n{robot_path}")
                return
            self.run_validation_button.setEnabled(False)
            self.validation_summary.setText("Validazione in corso...")
            QtWidgets.QApplication.setOverrideCursor(QtCore.Qt.WaitCursor)
            try:
                comparison, fits = run_model_validation(
                    sim_path,
                    robot_path,
                    output_dir,
                    self.grid_size_spin.value(),
                    window_mode,
                    custom_start,
                    custom_end,
                    scenario_label=str(preset_config.get("label") or self.selected_preset_name()),
                    extra_notes=[str(note) for note in preset_config.get("notes") or []],
                )
                self.validation_sim = load_report(sim_path)
                self.validation_robot = load_report(robot_path)
                self.validation_comparison = comparison
                self.validation_fits = fits
                self.populate_validation_tables()
                self.update_validation_plot()
                self.validation_result_tabs.setCurrentWidget(self.validation_fit_table)
                self.validation_summary.setText(
                    f"Validazione completata ({window_mode}). Output: {output_dir.resolve()}"
                )
            except Exception as exc:
                self.show_error("Errore validazione", str(exc))
                self.validation_summary.setText("Validazione fallita.")
            finally:
                QtWidgets.QApplication.restoreOverrideCursor()
                self.run_validation_button.setEnabled(True)

        def selected_window_params(self) -> tuple[str, float | None, float | None]:
            mode = self.analysis_window_combo.currentText()
            if mode == "custom":
                return mode, self.custom_start_spin.value(), self.custom_end_spin.value()
            return mode, None, None

        def run_aggregate_from_ui(self) -> None:
            sim_path = Path(self.sim_baseline_edit.text().strip())
            output_dir = Path(self.aggregate_output_edit.text().strip())
            robot_glob = self.aggregate_glob_edit.text().strip()
            window_mode, custom_start, custom_end = self.selected_window_params()
            preset_config = self.selected_preset_config()
            if not sim_path.exists():
                self.show_error("Baseline mancante", f"Non trovo il report simulato:\n{sim_path}")
                return
            robot_runs = sorted(Path(path) for path in glob.glob(robot_glob))
            if not robot_runs:
                self.show_error("Nessuna run trovata", f"Il glob non ha trovato report:\n{robot_glob}")
                return
            QtWidgets.QApplication.setOverrideCursor(QtCore.Qt.WaitCursor)
            try:
                aggregate = run_multi_run_validation(
                    sim_path,
                    robot_runs,
                    output_dir,
                    window_mode,
                    custom_start,
                    custom_end,
                    self.grid_size_spin.value(),
                    scenario_label=str(preset_config.get("label") or self.selected_preset_name()),
                    extra_notes=[str(note) for note in preset_config.get("notes") or []],
                )
                self.populate_aggregate_table(aggregate.get("rows", []))
                self.validation_result_tabs.setCurrentWidget(self.validation_aggregate_table)
                self.validation_summary.setText(
                    f"Aggregato completato ({len(robot_runs)} run, {window_mode}). Output: {output_dir.resolve()}"
                )
            except Exception as exc:
                self.show_error("Errore aggregato", str(exc))
                self.validation_summary.setText("Aggregato fallito.")
            finally:
                QtWidgets.QApplication.restoreOverrideCursor()

        def open_validation_output(self) -> None:
            output_dir = Path(self.validation_output_edit.text().strip()).resolve()
            output_dir.mkdir(parents=True, exist_ok=True)
            QtGui.QDesktopServices.openUrl(QtCore.QUrl.fromLocalFile(str(output_dir)))

        def populate_validation_tables(self) -> None:
            if self.validation_comparison is None or self.validation_fits is None:
                return
            metric_rows: list[list[str]] = []
            for scope in ["simulation", "robot"]:
                summary = self.validation_comparison[scope]["summary"]
                metric_rows.extend(
                    [
                        [scope, "summary", "run_id", str(self.validation_comparison[scope]["run_id"])],
                        [scope, "summary", "status", str(self.validation_comparison[scope]["status"])],
                        [scope, "summary", "samples", fmt(summary.get("samples"))],
                        [scope, "summary", "duration_s", fmt(summary.get("duration_s"), "s")],
                    ]
                )
                for signal, stats in self.validation_comparison[scope]["signals"].items():
                    metric_rows.append([scope, signal, "rms", fmt(stats.get("rms"))])
                    metric_rows.append([scope, signal, "mean", fmt(stats.get("mean"))])
                    metric_rows.append([scope, signal, "max", fmt(stats.get("max_value"))])
            for signal, stats in self.validation_comparison["aligned_normalized_time_error"].items():
                metric_rows.append(["aligned_error", signal, "bias", fmt(stats.get("bias"))])
                metric_rows.append(["aligned_error", signal, "rmse", fmt(stats.get("rmse"))])
                metric_rows.append(["aligned_error", signal, "max_abs", fmt(stats.get("max_abs"))])
            self.fill_table(self.validation_metrics_table, ["scope", "signal", "metric", "value"], metric_rows)

            fit_rows: list[list[str]] = []
            for name, fit in self.validation_fits.items():
                fit_rows.append(
                    [
                        name,
                        str(fit.get("signal")),
                        fmt(fit.get("samples")),
                        fmt(fit.get("dt_s"), "s"),
                        fmt(fit.get("a")),
                        fmt(fit.get("b")),
                        fmt(fit.get("c")),
                        fmt(fit.get("gain")),
                        fmt(fit.get("tau_s"), "s"),
                        fmt(fit.get("rmse")),
                        str(fit.get("note")),
                    ]
                )
            self.fill_table(
                self.validation_fit_table,
                ["fit", "signal", "samples", "dt", "a", "b", "c", "gain", "tau", "rmse", "note"],
                fit_rows,
            )

        def populate_aggregate_table(self, rows: list[dict[str, Any]]) -> None:
            headers = [
                "run",
                "samples",
                "window",
                "speed rmse",
                "yaw rmse",
                "distance rmse",
                "speed target rms",
                "yaw target rms",
                "speed tau",
                "yaw tau",
            ]
            table_rows = [
                [
                    str(row.get("robot_run_id")),
                    fmt(row.get("robot_window_samples")),
                    str(row.get("window_mode")),
                    fmt(row.get("speed_rmse_vs_sim")),
                    fmt(row.get("yaw_rate_rmse_vs_sim")),
                    fmt(row.get("distance_norm_rmse_vs_sim")),
                    fmt(row.get("robot_speed_target_rms")),
                    fmt(row.get("robot_yaw_target_rms")),
                    fmt(row.get("robot_speed_fit_tau_s"), "s"),
                    fmt(row.get("robot_yaw_fit_tau_s"), "s"),
                ]
                for row in rows
            ]
            self.fill_table(self.validation_aggregate_table, headers, table_rows)

        def fill_table(self, table: Any, headers: list[str], rows: list[list[str]]) -> None:
            table.clear()
            table.setColumnCount(len(headers))
            table.setRowCount(len(rows))
            table.setHorizontalHeaderLabels(headers)
            for row_index, row in enumerate(rows):
                for col_index, value in enumerate(row):
                    item = QtWidgets.QTableWidgetItem(value)
                    item.setFlags(item.flags() & ~QtCore.Qt.ItemIsEditable)
                    table.setItem(row_index, col_index, item)
            table.resizeColumnsToContents()
            table.horizontalHeader().setStretchLastSection(True)

        def update_validation_plot(self) -> None:
            self.validation_canvas.set_validation(
                self.validation_sim,
                self.validation_robot,
                self.validation_plot_combo.currentText(),
            )


    class ChartCanvas(QtWidgets.QWidget):
        def __init__(self):
            super().__init__()
            self.report: RunReport | None = None
            self.chart_id = "gate_distance"
            self.setMinimumHeight(320)

        def set_chart(self, report: RunReport | None, chart_id: str) -> None:
            self.report = report
            self.chart_id = chart_id
            self.update()

        def paintEvent(self, event) -> None:
            painter = QtGui.QPainter(self)
            painter.setRenderHint(QtGui.QPainter.Antialiasing)
            rect = self.rect().adjusted(0, 0, -1, -1)
            painter.fillRect(rect, QtGui.QColor("#ffffff"))
            painter.setPen(QtGui.QPen(QtGui.QColor("#d7dce5"), 1))
            painter.drawRoundedRect(rect, 8, 8)

            if self.report is None:
                self.draw_empty(painter, rect, "Seleziona un report per iniziare")
                return
            if self.chart_id == "trajectory_xy":
                self.draw_xy(painter, rect, xy_series(self.report))
                return
            series = chart_series(self.report, self.chart_id)
            if not series:
                self.draw_empty(painter, rect, "Nessuna serie disponibile per questo grafico")
                return
            self.draw_time_series(painter, rect, series)

        def draw_empty(self, painter, rect, message: str) -> None:
            painter.setPen(QtGui.QColor("#6b7280"))
            painter.setFont(QtGui.QFont("Sans Serif", 13))
            painter.drawText(rect, QtCore.Qt.AlignCenter, message)

        def draw_time_series(self, painter, rect, series: list[dict[str, Any]]) -> None:
            plot = rect.adjusted(74, 42, -28, -58)
            all_points = [point for item in series for point in item["points"]]
            xs = [point[0] for point in all_points]
            ys = [point[1] for point in all_points]
            x_min, x_max = padded_range(min(xs), max(xs), 0.02)
            y_min, y_max = padded_range(min(ys), max(ys), 0.08)
            sx = lambda x: plot.left() + (x - x_min) / (x_max - x_min) * plot.width()
            sy = lambda y: plot.bottom() - (y - y_min) / (y_max - y_min) * plot.height()

            self.draw_axes(painter, rect, plot, x_min, x_max, y_min, y_max, sx, sy, "time [s]")
            for item in series:
                self.draw_polyline(painter, item["points"], item["color"], sx, sy)

            completion = first_gate_completion_time(self.report)
            if completion is not None and x_min <= completion <= x_max:
                x = sx(completion)
                pen = QtGui.QPen(QtGui.QColor("#d62728"), 1.5)
                pen.setStyle(QtCore.Qt.DashLine)
                painter.setPen(pen)
                painter.drawLine(int(x), plot.top(), int(x), plot.bottom())
                painter.setPen(QtGui.QColor("#d62728"))
                painter.drawText(int(x) + 6, plot.top() + 17, "gate completion")

            self.draw_legend(painter, rect, series)

        def draw_xy(self, painter, rect, points: list[tuple[float, float]]) -> None:
            if not points:
                self.draw_empty(painter, rect, "Nessuna traiettoria XY disponibile")
                return
            plot = rect.adjusted(74, 42, -28, -58)
            xs = [point[0] for point in points]
            ys = [point[1] for point in points]
            x_min, x_max = padded_range(min(xs), max(xs), 0.08)
            y_min, y_max = padded_range(min(ys), max(ys), 0.08)
            sx = lambda x: plot.left() + (x - x_min) / (x_max - x_min) * plot.width()
            sy = lambda y: plot.bottom() - (y - y_min) / (y_max - y_min) * plot.height()
            self.draw_axes(painter, rect, plot, x_min, x_max, y_min, y_max, sx, sy, "x [m]", "y [m]")
            self.draw_polyline(painter, points, "#1f77b4", sx, sy)
            self.draw_dot(painter, sx(points[0][0]), sy(points[0][1]), "#2ca02c")
            self.draw_dot(painter, sx(points[-1][0]), sy(points[-1][1]), "#d62728")
            self.draw_legend(
                painter,
                rect,
                [
                    {"label": "trajectory", "color": "#1f77b4"},
                    {"label": "start", "color": "#2ca02c"},
                    {"label": "end", "color": "#d62728"},
                ],
            )

        def draw_axes(self, painter, rect, plot, x_min, x_max, y_min, y_max, sx, sy, x_label: str, y_label: str = "") -> None:
            painter.setFont(QtGui.QFont("Sans Serif", 9))
            painter.setPen(QtGui.QPen(QtGui.QColor("#e5e7eb"), 1))
            for value in ticks(y_min, y_max, 6):
                y = sy(value)
                painter.drawLine(plot.left(), int(y), plot.right(), int(y))
                painter.setPen(QtGui.QColor("#4b5563"))
                painter.drawText(rect.left() + 8, int(y) + 4, axis_fmt(value))
                painter.setPen(QtGui.QPen(QtGui.QColor("#e5e7eb"), 1))
            for value in ticks(x_min, x_max, 6):
                x = sx(value)
                painter.drawLine(int(x), plot.top(), int(x), plot.bottom())
                painter.setPen(QtGui.QColor("#4b5563"))
                painter.drawText(int(x) - 18, plot.bottom() + 22, axis_fmt(value))
                painter.setPen(QtGui.QPen(QtGui.QColor("#e5e7eb"), 1))
            painter.setPen(QtGui.QPen(QtGui.QColor("#374151"), 1.2))
            painter.drawLine(plot.left(), plot.top(), plot.left(), plot.bottom())
            painter.drawLine(plot.left(), plot.bottom(), plot.right(), plot.bottom())
            painter.setPen(QtGui.QColor("#374151"))
            painter.setFont(QtGui.QFont("Sans Serif", 9, QtGui.QFont.Bold))
            painter.drawText(plot.center().x() - 34, rect.bottom() - 20, x_label)
            if y_label:
                painter.save()
                painter.translate(rect.left() + 20, plot.center().y() + 28)
                painter.rotate(-90)
                painter.drawText(0, 0, y_label)
                painter.restore()

        def draw_polyline(self, painter, points, color: str, sx, sy) -> None:
            if len(points) < 2:
                return
            path = QtGui.QPainterPath()
            path.moveTo(sx(points[0][0]), sy(points[0][1]))
            for x, y in points[1:]:
                path.lineTo(sx(x), sy(y))
            painter.setPen(QtGui.QPen(QtGui.QColor(color), 2.2))
            painter.drawPath(path)

        def draw_dot(self, painter, x: float, y: float, color: str) -> None:
            painter.setBrush(QtGui.QColor(color))
            painter.setPen(QtGui.QPen(QtGui.QColor(color), 1))
            painter.drawEllipse(QtCore.QPointF(x, y), 5, 5)

        def draw_legend(self, painter, rect, series: list[dict[str, Any]]) -> None:
            box_w = 206
            box_h = 24 + 22 * len(series)
            x = rect.right() - box_w - 18
            y = rect.top() + 18
            box = QtCore.QRectF(x, y, box_w, box_h)
            painter.setBrush(QtGui.QColor(255, 255, 255, 235))
            painter.setPen(QtGui.QPen(QtGui.QColor("#e5e7eb"), 1))
            painter.drawRoundedRect(box, 7, 7)
            painter.setFont(QtGui.QFont("Sans Serif", 9))
            for index, item in enumerate(series):
                row_y = y + 22 + index * 22
                painter.setPen(QtGui.QPen(QtGui.QColor(item["color"]), 3))
                painter.drawLine(int(x + 12), int(row_y - 4), int(x + 34), int(row_y - 4))
                painter.setPen(QtGui.QColor("#374151"))
                painter.drawText(int(x + 42), int(row_y), str(item["label"]))


    class ComparisonCanvas(ChartCanvas):
        def __init__(self):
            super().__init__()
            self.sim_report: RunReport | None = None
            self.robot_report: RunReport | None = None
            self.mode = "speed"

        def set_validation(
            self,
            sim_report: RunReport | None,
            robot_report: RunReport | None,
            mode: str,
        ) -> None:
            self.sim_report = sim_report
            self.robot_report = robot_report
            self.mode = mode
            self.update()

        def paintEvent(self, event) -> None:
            painter = QtGui.QPainter(self)
            painter.setRenderHint(QtGui.QPainter.Antialiasing)
            rect = self.rect().adjusted(0, 0, -1, -1)
            painter.fillRect(rect, QtGui.QColor("#ffffff"))
            painter.setPen(QtGui.QPen(QtGui.QColor("#d7dce5"), 1))
            painter.drawRoundedRect(rect, 8, 8)
            if self.sim_report is None or self.robot_report is None:
                self.draw_empty(painter, rect, "Lancia una validazione per visualizzare il confronto")
                return
            series = self.comparison_series()
            if not series:
                self.draw_empty(painter, rect, "Nessun segnale disponibile per questo confronto")
                return
            self.draw_comparison_series(painter, rect, series)

        def comparison_series(self) -> list[dict[str, Any]]:
            sim = self.sim_report
            robot = self.robot_report
            if sim is None or robot is None:
                return []
            if self.mode == "speed":
                return [
                    {"label": f"sim {sim.run_id}", "points": normalize_time(signal_series(sim, "speed")), "color": COLORS[0]},
                    {"label": f"robot {robot.run_id}", "points": normalize_time(signal_series(robot, "speed")), "color": COLORS[1]},
                ]
            if self.mode == "yaw_rate":
                return [
                    {"label": f"sim {sim.run_id}", "points": normalize_time(yaw_rate_series(sim)), "color": COLORS[0]},
                    {"label": f"robot {robot.run_id}", "points": normalize_time(yaw_rate_series(robot)), "color": COLORS[1]},
                ]
            if self.mode == "distance_to_goal_norm":
                return [
                    {"label": "sim distance", "points": normalize_time(normalized_distance_to_goal(sim)), "color": COLORS[0]},
                    {"label": "robot distance", "points": normalize_time(normalized_distance_to_goal(robot)), "color": COLORS[1]},
                ]
            if self.mode == "tracking_error":
                return [
                    {"label": "sim cross-track [m]", "points": normalize_time(resolve_signal(sim, "tracker_cross_track")), "color": COLORS[0]},
                    {"label": "sim heading [deg]", "points": normalize_time(resolve_signal(sim, "tracker_heading_error_deg")), "color": COLORS[2]},
                    {"label": "robot speed-target", "points": normalize_time(error_series(robot, "speed", "target_speed")), "color": COLORS[1]},
                    {"label": "robot yaw-target", "points": normalize_time(error_series(robot, "yaw_rate", "target_yaw_rate")), "color": COLORS[3]},
                ]
            if self.mode == "timing":
                return [
                    {"label": "sim planning", "points": normalize_time(resolve_signal(sim, "planning_ms")), "color": COLORS[0]},
                    {"label": "robot planning", "points": normalize_time(resolve_signal(robot, "planning_ms")), "color": COLORS[1]},
                    {"label": "sim step", "points": normalize_time(resolve_signal(sim, "step_ms")), "color": COLORS[5]},
                    {"label": "robot step", "points": normalize_time(resolve_signal(robot, "step_ms")), "color": COLORS[4]},
                ]
            return []

        def draw_comparison_series(self, painter, rect, series: list[dict[str, Any]]) -> None:
            series = [item for item in series if item["points"]]
            if not series:
                self.draw_empty(painter, rect, "Nessuna serie disponibile")
                return
            plot = rect.adjusted(74, 42, -28, -58)
            all_points = [point for item in series for point in item["points"]]
            xs = [point[0] for point in all_points]
            ys = [point[1] for point in all_points]
            x_min, x_max = padded_range(min(xs), max(xs), 0.02)
            y_min, y_max = padded_range(min(ys), max(ys), 0.08)
            sx = lambda x: plot.left() + (x - x_min) / (x_max - x_min) * plot.width()
            sy = lambda y: plot.bottom() - (y - y_min) / (y_max - y_min) * plot.height()
            self.draw_axes(painter, rect, plot, x_min, x_max, y_min, y_max, sx, sy, "normalized time", "value")
            for item in series:
                self.draw_polyline(painter, item["points"], item["color"], sx, sy)
            self.draw_legend(painter, rect, series)


def chart_series(report: RunReport, chart_id: str) -> list[dict[str, Any]]:
    specs = {
        "gate_distance": [
            ("distance_to_goal", "distance_to_goal", COLORS[0], True),
            ("chosen_gate_distance", "chosen_gate_distance", COLORS[4], True),
        ],
        "gate_reference": [
            ("candidate_gates", "candidate_gates", COLORS[0], False),
            ("visible_gates", "visible_gates", COLORS[2], False),
            ("planner_has_reference", "planner_has_reference", COLORS[1], False),
            ("chosen_gate_index", "chosen_gate_index", COLORS[3], False),
        ],
        "lidar_clearance": [
            ("min_lidar", "min_lidar", COLORS[0], True),
            ("front_lidar", "front_lidar", COLORS[2], True),
        ],
        "lidar_samples": [
            ("close_lidar_samples", "close_lidar_samples", COLORS[1], False),
            ("front_close_samples", "front_close_lidar_samples", COLORS[3], False),
            ("lidar_samples", "lidar_samples", COLORS[0], False),
        ],
        "controller_pwm": [
            ("pwm_left", "controller_pwm_left", COLORS[0], False),
            ("pwm_right", "controller_pwm_right", COLORS[1], False),
            ("target_left", "controller_target_pwm_left", COLORS[5], False),
            ("target_right", "controller_target_pwm_right", COLORS[4], False),
        ],
        "controller_status": [
            ("status_flags", "controller_status_flags", COLORS[0], False),
            ("motor_flags", "controller_motor_flags", COLORS[2], False),
            ("error_code", "controller_error_code", COLORS[1], False),
            ("no_motion_cycles", "no_motion_cycles", COLORS[3], False),
        ],
        "motion_state": [
            ("speed", "speed", COLORS[0], False),
            ("target_speed", "target_speed", COLORS[5], False),
            ("yaw_rate", "yaw_rate", COLORS[1], False),
            ("target_yaw_rate", "target_yaw_rate", COLORS[4], False),
        ],
        "timing": [
            ("planning_ms", "planning_ms", COLORS[0], False),
            ("lidar_ms", "lidar_ms", COLORS[2], False),
            ("step_ms", "step_ms", COLORS[1], False),
            ("tracking_ms", "tracking_ms", COLORS[3], False),
        ],
    }.get(chart_id, [])

    series: list[dict[str, Any]] = []
    for label, key, color, nonnegative in specs:
        points = time_series(report, key, valid_nonnegative=nonnegative)
        if points:
            series.append({"label": label, "points": points, "color": color})
    return series


def infer_scenario(name: str) -> str:
    lower = name.lower()
    parts = []
    if "hardware" in lower:
        parts.append("hardware")
    elif "planner" in lower:
        parts.append("planner")
    if "unstructured" in lower:
        parts.append("unstructured")
    elif "structured" in lower:
        parts.append("structured")
    if "headless" in lower:
        parts.append("headless")
    elif "gui" in lower:
        parts.append("gui")
    return " / ".join(parts) if parts else "report"


def fmt(value: Any, unit: str = "") -> str:
    numeric = as_float(value)
    if numeric is None:
        return "-"
    absolute = abs(numeric)
    if absolute >= 100:
        text = f"{numeric:.0f}"
    elif absolute >= 10:
        text = f"{numeric:.2f}"
    else:
        text = f"{numeric:.3f}"
    return f"{text} {unit}" if unit else text


def axis_fmt(value: float) -> str:
    absolute = abs(value)
    if absolute >= 100:
        return f"{value:.0f}"
    if absolute >= 10:
        return f"{value:.1f}"
    return f"{value:.2f}"


def ticks(min_value: float, max_value: float, count: int) -> list[float]:
    if min_value == max_value:
        return [min_value]
    return [min_value + (max_value - min_value) * index / (count - 1) for index in range(count)]


def padded_range(min_value: float, max_value: float, fraction: float) -> tuple[float, float]:
    if min_value == max_value:
        pad = max(1.0, abs(min_value) * 0.2)
        return min_value - pad, max_value + pad
    pad = (max_value - min_value) * fraction
    return min_value - pad, max_value + pad


if __name__ == "__main__":
    raise SystemExit(main())
