#!/usr/bin/env python3
"""Generate presentation-ready multilevel validation charts from curated reports."""

from __future__ import annotations

import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT / "datasets" / "thesis_validation" / "raw_reports"
OUTPUT = Path("/home/isma/Downloads/assets_presentation")

COLORS = {"L0 ideale": "#20bf63", "L1 baseline": "#3d7ee8", "L2 hardware": "#f33f43"}
MODES = ["Strutturata", "Non strutturata", "Mista"]

REPORTS = {
    "Strutturata": {
        "L0 ideale": [DATA / "car_structured/thesis_planner_structured_ideal_validation_road_ideal_headless_20260516_112936_522.json"],
        "L1 baseline": [DATA / "car_structured/thesis_planner_structured_validation_road_headless_20260515_084019_344.json"],
        "L2 hardware": [DATA / "car_structured/thesis_hardware_structured_validation_road_gui_manual_20260424_005850_634.json"],
    },
    "Non strutturata": {
        "L0 ideale": [DATA / "car_unstructured/thesis_planner_unstructured_ideal_hardware_lab_lidar_dynamic_ideal_headless_20260516_112926_925.json"],
        "L1 baseline": [DATA / "car_unstructured/thesis_planner_unstructured_hardware_lab_lidar_dynamic_headless_20260515_083542_221.json"],
        "L2 hardware": [DATA / "car_unstructured/thesis_hardware_unstructured_manual_gate_editor_gui_manual_20260513_045435_200.json"],
    },
    "Mista": {
        "L0 ideale": [DATA / "car_mixed/thesis_planner_mixed_mixed_ideal_hardware_aligned_lidar_dynamic_ideal_headless_20260516_112925_029.json"],
        "L1 baseline": [DATA / "car_mixed/thesis_planner_mixed_mixed_hardware_aligned_lidar_dynamic_headless_20260515_083538_038.json"],
        "L2 hardware": [
            DATA / "car_mixed/thesis_hardware_mixed_mixed_hardware_road_gate_gui_manual_20260512_052657_596.json",
            DATA / "car_mixed/thesis_hardware_mixed_mixed_hardware_road_gate_gui_manual_20260512_053236_711.json",
        ],
    },
}

# Paired runs from the same recent canonical matrix. These make L1 a genuine
# Sim-Calibrated level rather than the nearly ideal historical reference run.
RECENT_SIM_REPORTS = {
    "Strutturata": {
        "L0 ideale": [ROOT / "reports/thesis_planner_structured_validation_road_ideal_headless_20260713_000432_951.json"],
        "L1 calibrata": [ROOT / "reports/thesis_planner_structured_validation_road_calibrated_headless_20260713_000516_169.json"],
    },
    "Non strutturata": {
        "L0 ideale": [ROOT / "reports/thesis_planner_unstructured_ideal_hardware_lab_lidar_dynamic_ideal_headless_20260713_000554_682.json"],
        "L1 calibrata": [ROOT / "reports/thesis_planner_unstructured_ideal_hardware_lab_lidar_dynamic_calibrated_headless_20260713_000600_630.json"],
    },
    "Mista": {
        "L0 ideale": [ROOT / "reports/thesis_planner_mixed_mixed_hardware_aligned_lidar_dynamic_ideal_headless_20260713_000611_795.json"],
        "L1 calibrata": [ROOT / "reports/thesis_planner_mixed_mixed_hardware_aligned_lidar_dynamic_calibrated_headless_20260713_000614_210.json"],
    },
}


def load(path: Path) -> dict:
    with path.open(encoding="utf-8") as stream:
        return json.load(stream)


def samples(report: dict) -> list[dict]:
    return report.get("telemetry", report.get("history", []))


def duration(report: dict) -> float:
    rows = samples(report)
    return float(rows[-1]["time"] - rows[0]["time"])


def tracking_p95(report: dict) -> float:
    values = [abs(float(row["tracker_cross_track"])) for row in samples(report)
              if row.get("tracker_cross_track") is not None]
    return float(np.percentile(values, 95))


def heading_p95(report: dict) -> float:
    values = [abs(float(row["tracker_heading_error_deg"])) for row in samples(report)
              if row.get("tracker_heading_error_deg") is not None]
    return float(np.percentile(values, 95))


def mean_metric(paths: list[Path], metric) -> float:
    return float(np.mean([metric(load(path)) for path in paths]))


def style_axis(ax) -> None:
    ax.spines[["top", "right"]].set_visible(False)
    ax.spines[["left", "bottom"]].set_color("#4b5568")
    ax.spines[["left", "bottom"]].set_linewidth(1.8)
    ax.grid(axis="y", color="#d8dfeb", linewidth=1.0)
    ax.set_axisbelow(True)
    ax.tick_params(colors="#46546c", labelsize=14, length=0, pad=8)


def add_bar_labels(ax, bars, decimals: int, suffix: str) -> None:
    for bar in bars:
        value = bar.get_height()
        ax.text(bar.get_x() + bar.get_width() / 2, value, f"{value:.{decimals}f}{suffix}",
                ha="center", va="bottom", fontsize=14, fontweight="bold", color="#172033")


def create_combined_chart() -> None:
    fig = plt.figure(figsize=(16, 9), dpi=120, facecolor="#f7f9fc")
    grid = fig.add_gridspec(2, 1, height_ratios=[1.05, 1.15], hspace=0.38,
                           left=0.09, right=0.96, top=0.94, bottom=0.10)
    ax_bar = fig.add_subplot(grid[0])
    ax_yaw = fig.add_subplot(grid[1])

    levels = list(COLORS)
    x = np.arange(len(MODES))
    width = 0.22
    for idx, level in enumerate(levels):
        values = [mean_metric(REPORTS[mode][level], tracking_p95) for mode in MODES]
        bars = ax_bar.bar(x + (idx - 1) * width, values, width, label=level,
                          color=COLORS[level], edgecolor="none")
        add_bar_labels(ax_bar, bars, 3, " m")
    ax_bar.set_title("Tracking error nelle 3 modalità", loc="left", fontsize=24, fontweight="bold", color="#111827", pad=12)
    ax_bar.set_ylabel("errore p95 [m]", fontsize=16, color="#46546c", labelpad=12)
    ax_bar.set_xticks(x, MODES, fontsize=16)
    ax_bar.set_ylim(0, 0.145)
    ax_bar.legend(ncols=3, frameon=False, fontsize=14, loc="upper right")
    style_axis(ax_bar)

    # Draw L0 last: its clean signal lies close to zero and would otherwise be
    # hidden by the denser L1/L2 traces.
    for level in ["L2 hardware", "L1 baseline", "L0 ideale"]:
        report = load(REPORTS["Mista"][level][0])
        rows = samples(report)
        time = np.asarray([float(row["time"]) for row in rows])
        time = (time - time[0]) / max(time[-1] - time[0], 1e-9)
        yaw_rate = np.asarray([float(row.get("yaw_rate", 0.0)) for row in rows])
        is_l0 = level == "L0 ideale"
        ax_yaw.plot(time, yaw_rate, color=COLORS[level],
                    linewidth=3.2 if is_l0 else 2.0,
                    linestyle="--" if is_l0 else "-",
                    zorder=5 if is_l0 else 3,
                    label=level, alpha=1.0 if is_l0 else 0.92)
    ax_yaw.axhline(0, color="#8490a5", linewidth=1)
    ax_yaw.set_title("Yaw rate in modalità mixed", loc="left",
                     fontsize=24, fontweight="bold", color="#111827", pad=12)
    ax_yaw.set_xlabel("tempo normalizzato della missione", fontsize=16, color="#46546c", labelpad=10)
    ax_yaw.set_ylabel("yaw-rate [rad/s]", fontsize=16, color="#46546c", labelpad=12)
    ax_yaw.set_xlim(0, 1)
    ax_yaw.set_ylim(-1.05, 1.25)
    ax_yaw.set_xticks([0, 0.25, 0.5, 0.75, 1])
    ax_yaw.legend(ncols=3, frameon=False, fontsize=14, loc="upper right")
    style_axis(ax_yaw)

    fig.savefig(OUTPUT / "tracking_error_yaw_rate_combined.png", facecolor=fig.get_facecolor())
    plt.close(fig)


def create_tracking_chart() -> None:
    fig, ax = plt.subplots(figsize=(16, 9), dpi=120, facecolor="#f7f9fc")
    fig.subplots_adjust(left=0.10, right=0.96, top=0.72, bottom=0.17)
    levels = list(COLORS)
    x = np.arange(len(MODES))
    width = 0.22
    for idx, level in enumerate(levels):
        values = [mean_metric(REPORTS[mode][level], tracking_p95) for mode in MODES]
        bars = ax.bar(x + (idx - 1) * width, values, width, label=level,
                      color=COLORS[level], edgecolor="none")
        add_bar_labels(ax, bars, 3, " m")
    fig.text(0.10, 0.91, "Errore di tracking p95 per modalità e livello", ha="left",
             fontsize=30, fontweight="bold", color="#111827")
    fig.text(0.10, 0.865,
             "Valori ricalcolati direttamente dai report validanti; L2 mista è la media di due prove.",
             ha="left", fontsize=16, color="#51617a")
    ax.set_ylabel("errore p95 [m]", fontsize=18, color="#46546c", labelpad=14)
    ax.set_xticks(x, MODES, fontsize=18)
    ax.set_ylim(0, 0.145)
    ax.legend(ncols=3, frameon=False, fontsize=16, loc="upper center", bbox_to_anchor=(0.5, 1.16))
    style_axis(ax)
    fig.savefig(OUTPUT / "multilevel_tracking_error_l0_l1_l2_large_font_corrected.png",
                facecolor=fig.get_facecolor())
    plt.close(fig)


def create_heading_yaw_chart() -> None:
    fig = plt.figure(figsize=(16, 9), dpi=120, facecolor="#f7f9fc")
    grid = fig.add_gridspec(2, 1, height_ratios=[1.05, 1.15], hspace=0.38,
                           left=0.09, right=0.96, top=0.94, bottom=0.10)
    ax_bar = fig.add_subplot(grid[0])
    ax_yaw = fig.add_subplot(grid[1])

    levels = list(COLORS)
    x = np.arange(len(MODES))
    width = 0.22
    for idx, level in enumerate(levels):
        values = [mean_metric(REPORTS[mode][level], heading_p95) for mode in MODES]
        bars = ax_bar.bar(x + (idx - 1) * width, values, width, label=level,
                          color=COLORS[level], edgecolor="none")
        add_bar_labels(ax_bar, bars, 1, "°")
    ax_bar.set_title("Heading error nelle 3 modalità", loc="left", fontsize=24,
                     fontweight="bold", color="#111827", pad=12)
    ax_bar.set_ylabel("errore p95 [°]", fontsize=16, color="#46546c", labelpad=12)
    ax_bar.set_xticks(x, MODES, fontsize=16)
    ax_bar.set_ylim(0, 86)
    ax_bar.legend(ncols=3, frameon=False, fontsize=14, loc="upper left",
                  bbox_to_anchor=(0.34, 1.0))
    style_axis(ax_bar)

    # Draw L0 last so the near-zero ideal trace remains visible.
    for level in ["L2 hardware", "L1 baseline", "L0 ideale"]:
        report = load(REPORTS["Mista"][level][0])
        rows = samples(report)
        time = np.asarray([float(row["time"]) for row in rows])
        time = (time - time[0]) / max(time[-1] - time[0], 1e-9)
        yaw_rate = np.asarray([float(row.get("yaw_rate", 0.0)) for row in rows])
        is_l0 = level == "L0 ideale"
        ax_yaw.plot(time, yaw_rate, color=COLORS[level],
                    linewidth=3.2 if is_l0 else 2.0,
                    linestyle="--" if is_l0 else "-",
                    zorder=5 if is_l0 else 3,
                    label=level, alpha=1.0 if is_l0 else 0.92)
    ax_yaw.axhline(0, color="#8490a5", linewidth=1)
    ax_yaw.set_title("Yaw rate in modalità mixed", loc="left", fontsize=24,
                     fontweight="bold", color="#111827", pad=12)
    ax_yaw.set_xlabel("tempo normalizzato della missione", fontsize=16,
                      color="#46546c", labelpad=10)
    ax_yaw.set_ylabel("yaw-rate [rad/s]", fontsize=16, color="#46546c", labelpad=12)
    ax_yaw.set_xlim(0, 1)
    ax_yaw.set_ylim(-1.05, 1.25)
    ax_yaw.set_xticks([0, 0.25, 0.5, 0.75, 1])
    ax_yaw.legend(ncols=3, frameon=False, fontsize=14, loc="upper right")
    style_axis(ax_yaw)

    fig.savefig(OUTPUT / "heading_error_yaw_rate_combined.png", facecolor=fig.get_facecolor())
    plt.close(fig)


def create_heading_yaw_calibrated_chart() -> None:
    fig = plt.figure(figsize=(16, 9), dpi=120, facecolor="#f7f9fc")
    grid = fig.add_gridspec(2, 1, height_ratios=[1.05, 1.15], hspace=0.38,
                           left=0.09, right=0.96, top=0.94, bottom=0.10)
    ax_bar = fig.add_subplot(grid[0])
    ax_yaw = fig.add_subplot(grid[1])
    levels = ["L0 ideale", "L1 calibrata", "L2 hardware"]
    colors = {"L0 ideale": COLORS["L0 ideale"], "L1 calibrata": COLORS["L1 baseline"],
              "L2 hardware": COLORS["L2 hardware"]}
    x = np.arange(len(MODES))
    width = 0.22

    for idx, level in enumerate(levels):
        if level == "L2 hardware":
            values = [mean_metric(REPORTS[mode][level], heading_p95) for mode in MODES]
        else:
            values = [mean_metric(RECENT_SIM_REPORTS[mode][level], heading_p95) for mode in MODES]
        bars = ax_bar.bar(x + (idx - 1) * width, values, width, label=level,
                          color=colors[level], edgecolor="none")
        add_bar_labels(ax_bar, bars, 1, "°")
    ax_bar.set_title("Heading error nelle 3 modalità", loc="left", fontsize=24,
                     fontweight="bold", color="#111827", pad=12)
    ax_bar.set_ylabel("errore p95 [°]", fontsize=16, color="#46546c", labelpad=12)
    ax_bar.set_xticks(x, MODES, fontsize=16)
    ax_bar.set_ylim(0, 86)
    ax_bar.legend(ncols=3, frameon=False, fontsize=14, loc="upper left",
                  bbox_to_anchor=(0.34, 1.0))
    style_axis(ax_bar)

    yaw_sources = {
        "L0 ideale": RECENT_SIM_REPORTS["Mista"]["L0 ideale"][0],
        "L1 calibrata": RECENT_SIM_REPORTS["Mista"]["L1 calibrata"][0],
        "L2 hardware": REPORTS["Mista"]["L2 hardware"][0],
    }
    for level in levels:
        rows = samples(load(yaw_sources[level]))
        time = np.asarray([float(row["time"]) for row in rows])
        time = (time - time[0]) / max(time[-1] - time[0], 1e-9)
        yaw_rate = np.asarray([float(row.get("yaw_rate", 0.0)) for row in rows])
        ax_yaw.plot(time, yaw_rate, color=colors[level], linewidth=2.2,
                    label=level, alpha=0.94)
    ax_yaw.axhline(0, color="#8490a5", linewidth=1)
    ax_yaw.set_title("Yaw rate in modalità mixed", loc="left", fontsize=24,
                     fontweight="bold", color="#111827", pad=12)
    ax_yaw.set_xlabel("tempo normalizzato della missione", fontsize=16,
                      color="#46546c", labelpad=10)
    ax_yaw.set_ylabel("yaw-rate [rad/s]", fontsize=16, color="#46546c", labelpad=12)
    ax_yaw.set_xlim(0, 1)
    ax_yaw.set_ylim(-1.05, 1.25)
    ax_yaw.set_xticks([0, 0.25, 0.5, 0.75, 1])
    ax_yaw.legend(ncols=3, frameon=False, fontsize=14, loc="upper right")
    style_axis(ax_yaw)
    fig.savefig(OUTPUT / "heading_error_yaw_rate_sim_calibrated.png", facecolor=fig.get_facecolor())
    plt.close(fig)


def main() -> None:
    OUTPUT.mkdir(parents=True, exist_ok=True)
    create_combined_chart()
    create_tracking_chart()
    create_heading_yaw_chart()
    create_heading_yaw_calibrated_chart()
    for mode in MODES:
        values = []
        for level in COLORS:
            paths = REPORTS[mode][level]
            values.append((level, mean_metric(paths, duration), mean_metric(paths, tracking_p95)))
        print(mode, values)


if __name__ == "__main__":
    main()
