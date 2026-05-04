from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from data_analisys.report_analysis import as_float, load_report, run_id_from_path, time_series  # noqa: E402
from data_analisys.svg_charts import write_grouped_bar_chart, write_time_series_chart, write_timeline_chart  # noqa: E402


DEFAULT_SIM_REPORT = Path("reports/thesis_planner_unstructured_robot_validation_lidar_dynamic_gui_20260503_150430_614.json")
DEFAULT_HARDWARE_REPORTS = [
    Path("reports/thesis_hardware_unstructured_manual_gate_editor_gui_manual_20260430_002215_861.json"),
    Path("reports/thesis_hardware_unstructured_manual_gate_editor_gui_manual_20260430_002347_391.json"),
    Path("reports/thesis_hardware_unstructured_manual_gate_editor_gui_manual_20260430_012235_253.json"),
]
DEFAULT_OUTPUT_DIR = Path("data_analisys/outputs/unstructured_dynamic_gate_validation_20260503")


@dataclass(frozen=True)
class DynamicGateMetrics:
    run_id: str
    source: str
    status: str
    goal_reached: bool
    mission_duration_s: float | None
    telemetry_start_s: float | None
    telemetry_end_s: float | None
    telemetry_duration_s: float | None
    first_candidate_rel_s: float | None
    first_lock_rel_s: float | None
    candidate_to_lock_delay_s: float | None
    candidate_windows: int
    reference_windows: int
    total_candidate_time_s: float
    total_reference_time_s: float
    candidate_coverage_pct: float | None
    reference_coverage_pct: float | None
    reference_over_candidate_pct: float | None
    longest_reference_window_s: float | None
    longest_reacquisition_gap_s: float | None
    min_chosen_gate_distance_m: float | None
    final_chosen_gate_distance_m: float | None
    first_completion_rel_s: float | None
    completion_events_in_history: int
    reported_passed_gates: int | None
    min_lidar_m: float | None
    min_front_lidar_m: float | None
    avg_planning_ms: float | None
    max_planning_ms: float | None
    avg_lidar_ms: float | None
    max_lidar_ms: float | None
    avg_step_ms: float | None
    max_step_ms: float | None


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Analyze dynamic LiDAR-gate unstructured simulation and hardware reports for thesis tables.",
    )
    parser.add_argument("--simulation", type=Path, default=DEFAULT_SIM_REPORT)
    parser.add_argument("--hardware", type=Path, action="append", default=[])
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    hardware_reports = args.hardware if args.hardware else DEFAULT_HARDWARE_REPORTS
    report_paths = [args.simulation, *hardware_reports]

    missing = [path for path in report_paths if not path.exists()]
    if missing:
        for path in missing:
            print(f"Missing report: {path}", file=sys.stderr)
        return 2

    args.output_dir.mkdir(parents=True, exist_ok=True)
    reports = [load_report(path) for path in report_paths]
    metrics = [compute_metrics(report) for report in reports]

    write_metrics_csv(args.output_dir / "dynamic_gate_metrics.csv", metrics)
    (args.output_dir / "dynamic_gate_metrics.json").write_text(
        json.dumps([asdict(item) for item in metrics], indent=2),
        encoding="utf-8",
    )
    write_figures(reports, metrics, args.output_dir)
    write_thesis_markdown(args.output_dir / "thesis_unstructured_dynamic_gate_analysis.md", metrics, report_paths)
    write_manifest(args.output_dir / "manifest.txt", report_paths)

    print(f"Generated dynamic-gate thesis analysis in {args.output_dir}")
    return 0


def compute_metrics(report) -> DynamicGateMetrics:
    history = report.history
    run_id = report.run_id
    source = "simulation" if "thesis_planner" in report.path.name else "hardware"
    status = str(report.status or ("goal_reached" if report.frame.get("goal_reached") else "unknown"))
    goal_reached = status == "goal_reached" or bool(report.frame.get("goal_reached"))
    times = [_time(sample) for sample in history]
    times = [time for time in times if time is not None]
    start_time = min(times) if times else None
    end_time = max(times) if times else None
    telemetry_duration = (end_time - start_time) if start_time is not None and end_time is not None else None
    mission_duration = _mission_duration(report, end_time)

    candidate_windows = boolean_windows(report, "candidate_gates")
    reference_windows = boolean_windows(report, "planner_has_reference")
    total_candidate_time = sum(end - start for start, end in candidate_windows)
    total_reference_time = sum(end - start for start, end in reference_windows)
    first_candidate = candidate_windows[0][0] if candidate_windows else None
    first_lock = reference_windows[0][0] if reference_windows else None
    first_completion = first_zero_event_time(report)
    completion_events = zero_event_times(report)
    positive_gate_distances = [
        value
        for value in (_value(sample, "chosen_gate_distance") for sample in history)
        if value is not None and value >= 0.0
    ]

    return DynamicGateMetrics(
        run_id=run_id,
        source=source,
        status=status,
        goal_reached=goal_reached,
        mission_duration_s=mission_duration,
        telemetry_start_s=start_time,
        telemetry_end_s=end_time,
        telemetry_duration_s=telemetry_duration,
        first_candidate_rel_s=_rel(first_candidate, start_time),
        first_lock_rel_s=_rel(first_lock, start_time),
        candidate_to_lock_delay_s=(first_lock - first_candidate)
        if first_candidate is not None and first_lock is not None
        else None,
        candidate_windows=len(candidate_windows),
        reference_windows=len(reference_windows),
        total_candidate_time_s=total_candidate_time,
        total_reference_time_s=total_reference_time,
        candidate_coverage_pct=_pct(total_candidate_time, telemetry_duration),
        reference_coverage_pct=_pct(total_reference_time, telemetry_duration),
        reference_over_candidate_pct=_pct(total_reference_time, total_candidate_time),
        longest_reference_window_s=max((end - start for start, end in reference_windows), default=None),
        longest_reacquisition_gap_s=longest_gap_between(reference_windows),
        min_chosen_gate_distance_m=min(positive_gate_distances) if positive_gate_distances else None,
        final_chosen_gate_distance_m=positive_gate_distances[-1] if positive_gate_distances else None,
        first_completion_rel_s=_rel(first_completion, start_time),
        completion_events_in_history=len(completion_events),
        reported_passed_gates=_reported_passed_gates(report),
        min_lidar_m=_min_metric(report, "min_lidar"),
        min_front_lidar_m=_min_metric(report, "front_lidar"),
        avg_planning_ms=_performance(report, "planning_ms", "avg"),
        max_planning_ms=_performance(report, "planning_ms", "max"),
        avg_lidar_ms=_performance(report, "lidar_ms", "avg"),
        max_lidar_ms=_performance(report, "lidar_ms", "max"),
        avg_step_ms=_performance(report, "step_ms", "avg"),
        max_step_ms=_performance(report, "step_ms", "max"),
    )


def boolean_windows(report, key: str) -> list[tuple[float, float]]:
    windows: list[tuple[float, float]] = []
    start: float | None = None
    last_time: float | None = None
    for sample in report.history:
        time = _time(sample)
        if time is None:
            continue
        value = _value(sample, key) or 0.0
        active = value > 0.5
        if active and start is None:
            start = time
        elif not active and start is not None:
            windows.append((start, last_time if last_time is not None else time))
            start = None
        last_time = time
    if start is not None and last_time is not None:
        windows.append((start, last_time))
    return windows


def zero_event_times(report) -> list[float]:
    events: list[float] = []
    previous_zero = False
    for sample in report.history:
        time = _time(sample)
        distance = _value(sample, "distance_to_goal")
        current_zero = distance is not None and 0.0 <= distance <= 1e-4
        if current_zero and not previous_zero and time is not None:
            events.append(time)
        previous_zero = current_zero
    return events


def first_zero_event_time(report) -> float | None:
    events = zero_event_times(report)
    return events[0] if events else None


def longest_gap_between(windows: list[tuple[float, float]]) -> float | None:
    if len(windows) < 2:
        return None
    gaps = [max(0.0, windows[i][0] - windows[i - 1][1]) for i in range(1, len(windows))]
    return max(gaps) if gaps else None


def write_metrics_csv(path: Path, metrics: list[DynamicGateMetrics]) -> None:
    rows = [asdict(item) for item in metrics]
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def write_figures(reports, metrics: list[DynamicGateMetrics], output_dir: Path) -> None:
    labels = [_short_label(item.run_id) for item in metrics]
    write_grouped_bar_chart(
        output_dir / "lock_quality.svg",
        title="Dynamic gate lock quality",
        categories=labels,
        y_label="percent",
        groups=[
            {"label": "candidate coverage", "values": [m.candidate_coverage_pct for m in metrics], "color": "#1f77b4"},
            {"label": "reference coverage", "values": [m.reference_coverage_pct for m in metrics], "color": "#d62728"},
            {"label": "reference / candidates", "values": [m.reference_over_candidate_pct for m in metrics], "color": "#2ca02c"},
        ],
    )
    write_grouped_bar_chart(
        output_dir / "lock_timing.svg",
        title="Dynamic gate lock timing",
        categories=labels,
        y_label="seconds",
        groups=[
            {"label": "first lock", "values": [m.first_lock_rel_s for m in metrics], "color": "#9467bd"},
            {"label": "longest lock", "values": [m.longest_reference_window_s for m in metrics], "color": "#1f77b4"},
            {"label": "longest reacquisition gap", "values": [m.longest_reacquisition_gap_s for m in metrics], "color": "#ff7f0e"},
        ],
    )
    hardware_metrics = [metric for metric in metrics if metric.source == "hardware"]
    hardware_labels = [_short_label(item.run_id) for item in hardware_metrics]
    write_grouped_bar_chart(
        output_dir / "hardware_lidar_clearance.svg",
        title="Hardware LiDAR clearance",
        categories=hardware_labels,
        y_label="meters",
        groups=[
            {"label": "min LiDAR", "values": [m.min_lidar_m for m in hardware_metrics], "color": "#1f77b4"},
            {"label": "front LiDAR", "values": [m.min_front_lidar_m for m in hardware_metrics], "color": "#d62728"},
        ],
    )
    write_grouped_bar_chart(
        output_dir / "loop_timing.svg",
        title="Planner loop timing",
        categories=labels,
        y_label="milliseconds",
        groups=[
            {"label": "avg planning", "values": [m.avg_planning_ms for m in metrics], "color": "#1f77b4"},
            {"label": "avg lidar", "values": [m.avg_lidar_ms for m in metrics], "color": "#2ca02c"},
            {"label": "avg step", "values": [m.avg_step_ms for m in metrics], "color": "#d62728"},
        ],
    )

    timeline_rows = []
    for report, metric in zip(reports, metrics):
        start = metric.telemetry_start_s or 0.0
        windows = [(s - start, e - start) for s, e in boolean_windows(report, "planner_has_reference")]
        markers = []
        if metric.first_completion_rel_s is not None:
            markers.append({"x": metric.first_completion_rel_s, "color": "#d62728"})
        timeline_rows.append(
            {
                "label": metric.run_id,
                "duration_s": metric.telemetry_duration_s or 0.0,
                "windows": windows,
                "markers": markers,
            }
        )
    write_timeline_chart(
        output_dir / "reference_windows.svg",
        title="Dynamic gate reference windows",
        rows=timeline_rows,
    )

    for report, metric in zip(reports, metrics):
        markers = []
        if metric.first_completion_rel_s is not None:
            start = metric.telemetry_start_s or 0.0
            markers.append({"x": start + metric.first_completion_rel_s, "label": "completion", "color": "#d62728"})
        run_dir = output_dir / metric.run_id
        write_time_series_chart(
            run_dir / "gate_lock_state.svg",
            title=f"{metric.run_id} dynamic gate lock state",
            x_label="time [s]",
            y_label="count / flag / index",
            series=[
                {"label": "candidate_gates", "points": time_series(report, "candidate_gates"), "color": "#1f77b4"},
                {"label": "planner_has_reference", "points": time_series(report, "planner_has_reference"), "color": "#d62728"},
                {"label": "chosen_gate_index", "points": time_series(report, "chosen_gate_index"), "color": "#9467bd"},
            ],
            markers=markers,
        )
        write_time_series_chart(
            run_dir / "gate_distance.svg",
            title=f"{metric.run_id} gate distance",
            x_label="time [s]",
            y_label="meters",
            series=[
                {"label": "distance_to_goal", "points": time_series(report, "distance_to_goal", valid_nonnegative=True), "color": "#1f77b4"},
                {"label": "chosen_gate_distance", "points": time_series(report, "chosen_gate_distance", valid_nonnegative=True), "color": "#ff7f0e"},
            ],
            markers=markers,
        )
        write_time_series_chart(
            run_dir / "motion_command.svg",
            title=f"{metric.run_id} motion command",
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


def write_thesis_markdown(path: Path, metrics: list[DynamicGateMetrics], report_paths: list[Path]) -> None:
    sim = next((item for item in metrics if item.source == "simulation"), None)
    hardware = [item for item in metrics if item.source == "hardware"]
    lines: list[str] = [
        "# Analisi dati - validazione unstructured con gate dinamici LiDAR",
        "",
        "Questa analisi confronta la simulazione unstructured a gate dinamici LiDAR con i migliori run hardware disponibili. Il confronto non usa l'errore punto-a-punto della modalita structured: nella modalita unstructured i gate sono generati online dalla percezione LiDAR e non esiste una sequenza nominale identica tra simulazione e robot. Le metriche confrontabili sono quindi quelle della pipeline percezione-decisione-controllo: disponibilita dei candidati, lock del gate, continuita della reference, tempi di reacquisizione, clearance LiDAR e timing del ciclo.",
        "",
        "## Dataset",
        "",
        "| Run | Sorgente | Stato | Durata missione [s] | Finestra telemetria [s] | Note |",
        "|---|---:|---:|---:|---:|---|",
    ]
    for metric, report_path in zip(metrics, report_paths):
        note = "simulazione robot_validation con gate LiDAR dinamici" if metric.source == "simulation" else "hardware con completamento multi-gate"
        lines.append(
            f"| `{metric.run_id}` | {metric.source} | {metric.status} | {_fmt(metric.mission_duration_s)} | {_fmt(metric.telemetry_duration_s)} | {note} |"
        )

    lines.extend(
        [
            "",
            "## Metriche principali",
            "",
            "| Run | primo lock [s] | finestre lock | lock totale [s] | copertura lock [%] | lock piu lungo [s] | gap reacq. max [s] | min dist. gate [m] | eventi completion | min front LiDAR [m] |",
            "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
        ]
    )
    for metric in metrics:
        lines.append(
            "| "
            f"`{metric.run_id}` | "
            f"{_fmt(metric.first_lock_rel_s)} | "
            f"{metric.reference_windows} | "
            f"{_fmt(metric.total_reference_time_s)} | "
            f"{_fmt(metric.reference_coverage_pct)} | "
            f"{_fmt(metric.longest_reference_window_s)} | "
            f"{_fmt(metric.longest_reacquisition_gap_s)} | "
            f"{_fmt(metric.min_chosen_gate_distance_m)} | "
            f"{metric.completion_events_in_history} | "
            f"{_fmt(metric.min_front_lidar_m)} |"
        )

    lines.extend(
        [
            "",
            "Nota: la colonna `eventi completion` conta le transizioni `distance_to_goal = 0` presenti nella telemetria esportata. Nei report hardware selezionati il runner conclude la missione dopo due gate dinamici passati, ma il JSON salva in modo esplicito solo l'evento finale di completamento.",
        ]
    )
    lines.extend(chart_explanation_lines())

    if hardware:
        avg_lock_coverage = _avg([item.reference_coverage_pct for item in hardware])
        avg_lock_ratio = _avg([item.reference_over_candidate_pct for item in hardware])
        best_hw = min(hardware, key=lambda item: item.mission_duration_s or math.inf)
    else:
        avg_lock_coverage = None
        avg_lock_ratio = None
        best_hw = None

    lines.extend(
        [
            "",
            "## Lettura per la tesi",
            "",
            "- La simulazione `robot_validation` arriva al goal senza collisione e mantiene una reference LiDAR dinamica continua nella finestra esportata. Questo mostra che, nel mondo grande, la catena candidate gate -> reference -> tracking e' stabile quando il LiDAR produce una direzione libera consistente.",
            "- Nei run hardware il criterio di successo e' diverso: il runner termina dopo due gate dinamici passati. Per questo il numero `passed_gates` della simulazione non e' direttamente comparabile con l'hardware; e' piu corretto confrontare lock, continuita della reference e reacquisizione.",
            f"- Sui tre run hardware migliori la copertura media della reference e' circa {_fmt(avg_lock_coverage)}% della telemetria disponibile, mentre circa {_fmt(avg_lock_ratio)}% del tempo con candidati produce anche una reference tracciabile.",
        ]
    )
    if best_hw is not None:
        lines.append(
            f"- Il run hardware piu rapido nel gruppo e' `{best_hw.run_id}`, con completamento a circa {_fmt(best_hw.mission_duration_s)} s e una distanza minima dal gate scelto di {_fmt(best_hw.min_chosen_gate_distance_m)} m."
        )
    if sim is not None:
        lines.append(
            f"- Nel report simulato la finestra esportata parte a { _fmt(sim.telemetry_start_s) } s; le metriche di lock sono quindi riferite alla parte finale della missione, mentre la durata missione completa e' { _fmt(sim.mission_duration_s) } s."
        )
    lines.extend(
        [
            "- La criticita residua emersa dai dati hardware non e' l'assenza di candidati, ma la stabilita dopo perdita/reacquisizione: i gap di reacquisizione raggiungono decine di secondi nei run piu lenti. Questo giustifica la scelta di presentare la validazione come pipeline percettiva locale, non come tracciamento di una traiettoria globale prefissata.",
            "",
            "## Figure generate",
            "",
            "- `reference_windows.svg`: finestre temporali con reference attiva.",
            "- `lock_quality.svg`: percentuale di tempo con candidati e reference.",
            "- `lock_timing.svg`: primo lock, lock piu lungo e gap massimo di reacquisizione.",
            "- `hardware_lidar_clearance.svg`: clearance LiDAR minima sui run hardware, con valori mancanti per la simulazione se il report non esporta `min_lidar`.",
            "- sottocartelle per-run: stato del lock, distanza dal gate e comandi di moto.",
            "",
            "## Report usati",
            "",
        ]
    )
    lines.extend(f"- `{path}`" for path in report_paths)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def chart_explanation_lines() -> list[str]:
    return [
        "",
        "## Come leggere i grafici della app",
        "",
        "Nei grafici generati dalla app `Unstructured Dynamic Gate Comparison`, le barre blu indicano la simulazione e le barre rosse indicano i run hardware. La differenza di colore serve a separare due domini sperimentali diversi: la simulazione usa una mappa `robot_validation` grande, mentre i run hardware sono prove reali in scala ridotta con gate ricavati online dal LiDAR. Per questo le grandezze vanno interpretate come indicatori della pipeline, non come confronto metrico diretto di una stessa traiettoria.",
        "",
        "### Overview dashboard",
        "",
        "`Mission duration` misura il tempo totale fino alla condizione `goal_reached`. Il valore serve soprattutto a verificare che ogni run sia chiuso correttamente e a confrontare run dello stesso dominio. Non va usato da solo per dire che simulazione e hardware hanno prestazioni temporali equivalenti, perche' scala della mappa, percorso e criterio di completamento sono diversi.",
        "",
        "`Reference coverage` e' la percentuale della finestra di telemetria in cui `planner_has_reference = 1`, cioe' il planner ha un gate dinamico scelto e una traiettoria/reference da seguire. Un valore alto indica che il sistema passa poco tempo in scansione, attesa o reacquisizione.",
        "",
        "`Longest lock window` misura la durata massima di una finestra continua con reference attiva. E' una misura di persistenza del gate scelto: se il valore e' alto, il target LiDAR non viene perso immediatamente e il planner riesce a seguirlo per un tratto significativo.",
        "",
        "`Min chosen-gate distance` e' la distanza minima registrata tra il robot e il gate dinamico scelto. Nei run hardware e' un buon indicatore di avvicinamento al piano/alla regione di attraversamento. Tra simulazione e hardware va letta solo qualitativamente, perche' i criteri di completamento non sono identici.",
        "",
        "### Reference windows",
        "",
        "Il grafico `Reference windows` mostra quando esistono candidati e quando uno di questi diventa reference attiva. Le finestre candidate indicano che il LiDAR ha trovato aperture geometricamente plausibili. Le finestre reference indicano che il planner ha selezionato un candidato stabile e lo sta usando per costruire la traiettoria. Le linee verticali di completamento marcano l'istante in cui il runner considera raggiunto il criterio di missione.",
        "",
        "### Lock quality",
        "",
        "`Lock quality` confronta il tempo con candidati, il tempo con reference e il rapporto `reference / candidates`. Il primo valore misura quanto spesso la percezione produce aperture; il secondo misura quanto spesso il planner ha effettivamente un target utilizzabile; il terzo misura l'efficienza della trasformazione candidato -> lock.",
        "",
        "### Lock timing",
        "",
        "`Lock timing` mostra il primo lock, la finestra di lock piu lunga e il gap massimo di reacquisizione. Il gap massimo di reacquisizione e' il principale indicatore di debolezza residua: misura quanto tempo il robot rimane senza reference tra due finestre utili.",
        "",
        "### Gate distance overlay",
        "",
        "`Gate distance overlay` sovrappone la distanza dal gate scelto su tempo normalizzato. Serve a confrontare la forma qualitativa dell'avvicinamento: una curva che decresce in modo regolare indica che il target scelto rimane coerente con il moto del robot.",
        "",
        "### Selected run gate state",
        "",
        "`Selected run gate state` mostra, per un singolo run, `candidate_gates`, `planner_has_reference` e `chosen_gate_index`. E' utile per capire se il robot e' senza riferimento per mancanza di candidati, per mancata conferma del candidato o per perdita del gate scelto.",
        "",
        "### Motion command",
        "",
        "`Motion command` confronta velocita, velocita target, yaw-rate e yaw-rate target. Fasi con `target_speed` vicino a zero e yaw-rate non nullo corrispondono tipicamente a scansione o reacquisizione. Fasi con `target_speed` positivo e reference attiva corrispondono all'avvicinamento al gate.",
        "",
        "### LiDAR clearance",
        "",
        "`LiDAR clearance` mostra la distanza minima LiDAR e la distanza frontale. Nei report hardware e' un indicatore di sicurezza: valori molto bassi segnalano passaggi vicino agli ostacoli o contatti potenziali. Se un report simulato non esporta `min_lidar` e `front_lidar`, questo grafico va letto solo sui run hardware.",
        "",
        "### Loop timing",
        "",
        "`Loop timing` confronta i tempi medi di planning, LiDAR e step. Serve a verificare che il comportamento osservato non dipenda da sovraccarico computazionale. Nei report hardware il tempo di ciclo include comunicazione, acquisizione LiDAR e aggiornamento del runner reale, quindi e' naturalmente piu alto della simulazione.",
        "",
        "### Trajectory XY",
        "",
        "`Trajectory XY` mostra la traiettoria stimata nel piano. E' utile come supporto visivo, ma non e' la metrica principale di confronto per questa validazione: simulazione e hardware non hanno la stessa mappa ne' gli stessi gate fisici.",
    ]


def write_manifest(path: Path, report_paths: list[Path]) -> None:
    lines = ["Unstructured dynamic gate thesis analysis", "", "Reports:"]
    lines.extend(f"- {report_path}" for report_path in report_paths)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _mission_duration(report, telemetry_end: float | None) -> float | None:
    value = _nested(report.raw_summary, "sim_time")
    if value is not None:
        return value
    value = _nested(report.frame, "sim_time")
    if value is not None:
        return value
    return telemetry_end


def _reported_passed_gates(report) -> int | None:
    value = _nested(report.raw_summary, "passed_gates")
    if value is None:
        return None
    return int(round(value))


def _performance(report, family: str, metric: str) -> float | None:
    value = report.performance.get(family)
    if not isinstance(value, dict):
        return None
    return as_float(value.get(metric))


def _min_metric(report, key: str) -> float | None:
    values = [
        value
        for value in (_value(sample, key) for sample in report.history)
        if value is not None and value >= 0.0
    ]
    return min(values) if values else None


def _time(sample: dict[str, Any]) -> float | None:
    return _value(sample, "time")


def _value(sample: dict[str, Any], key: str) -> float | None:
    return as_float(sample.get(key))


def _nested(data: dict[str, Any], key: str) -> float | None:
    return as_float(data.get(key)) if isinstance(data, dict) else None


def _rel(value: float | None, origin: float | None) -> float | None:
    if value is None or origin is None:
        return None
    return value - origin


def _pct(value: float | None, total: float | None) -> float | None:
    if value is None or total is None or total <= 0.0:
        return None
    return 100.0 * value / total


def _avg(values: list[float | None]) -> float | None:
    clean = [value for value in values if value is not None]
    return sum(clean) / len(clean) if clean else None


def _fmt(value: float | None) -> str:
    if value is None:
        return "n/a"
    if abs(value) >= 100.0:
        return f"{value:.1f}"
    if abs(value) >= 10.0:
        return f"{value:.2f}"
    return f"{value:.3f}"


def _short_label(run_id: str) -> str:
    return run_id[-11:] if len(run_id) > 11 else run_id


if __name__ == "__main__":
    raise SystemExit(main())
