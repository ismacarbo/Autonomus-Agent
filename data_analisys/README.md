# Data analisys

Questa cartella contiene una pipeline Python per analizzare i report JSON dei test
hardware, visualizzarli in una app desktop e produrre grafici SVG statici.

## App desktop locale

Il viewer principale e una app PyQt5 locale:

```bash
python -m data_analisys.report_viewer_qt
```

Se PyQt5 non e installato:

```bash
python -m pip install -r data_analisys/requirements.txt
```

Dalla UI puoi:

- tenere la finestra sempre aperta
- ricaricare la lista dei JSON in `reports/`
- filtrare per run, data, hardware, structured/unstructured
- aprire un report tramite file picker o percorso manuale
- cambiare grafico senza rigenerare gli SVG batch
- lanciare la validazione modello nella tab `Model validation`
- scegliere un preset di baseline: `unstructured`, `structured_validation_road`, `structured_figure_eight`
- confrontare baseline simulata e run reale con metriche, fit e plot nella stessa finestra
- vedere sempre i tab `Metrics`, `Fits`, `Aggregate` e `Fitting notes` nella sezione `Model validation`
- scegliere la finestra di analisi: `full`, `until_first_gate`, `longest_reference`, `reference_only`, `custom`
- lanciare un aggregato multi-run con un glob sui report reali

Per aprire subito un report:

```bash
python -m data_analisys.report_viewer_qt \
  --open reports/thesis_hardware_unstructured_manual_gate_editor_gui_manual_20260418_181318_932.json
```

Per puntare a una cartella diversa:

```bash
python -m data_analisys.report_viewer_qt --reports-dir /path/ai/report
```

## Export SVG batch

Per generare figure statiche:

```bash
python -m data_analisys.plot_unstructured_report
```

Di default lo script legge:

- `documentation/Unstructured_Hardware_Status_20260418.md`
- i report JSON corrispondenti dentro `reports/`
- scrive tutto in `data_analisys/outputs/unstructured_20260418/`

## Cosa genera

Per ogni run:

- `gate_distance.svg`: distanza dal gate scelto e tempo di completamento
- `gate_reference.svg`: candidate gates, visible gates, reference attiva e indice scelto
- `lidar_clearance.svg`: clearance LiDAR minima e frontale
- `lidar_close_samples.svg`: campioni LiDAR ravvicinati
- `controller_pwm.svg`: PWM applicati e target
- `controller_status.svg`: flag, errori e cicli senza moto
- `motion_state.svg`: velocita e yaw-rate misurati/target
- `timing.svg`: planning, LiDAR, tracking e step time
- `trajectory_xy.svg`: traiettoria stimata nel piano

Per il confronto tra run:

- `summary.csv`
- `summary.json`
- `summary_gate_timing.svg`
- `summary_loop_timing.svg`
- `summary_reference_windows.svg`

## Uso su un singolo report

```bash
python -m data_analisys.plot_unstructured_report \
  --report reports/thesis_hardware_unstructured_manual_gate_editor_gui_manual_20260418_181318_932.json \
  --output-dir data_analisys/outputs/one_run
```

## Baseline simulazione vs robot

La stessa analisi e disponibile nella tab `Model validation` della app PyQt5.
Per eseguirla da terminale:

```bash
python -m data_analisys.model_validation
```

Di default confronta:

- baseline simulata: `reports/thesis_planner_unstructured_wide_slalom_gui_auto_20260317_032610_597.json`
- run reale riuscita: `reports/thesis_hardware_unstructured_manual_gate_editor_gui_manual_20260418_181318_932.json`
- configurazione: `data_analisys/unstructured_validation_config.json`
- finestra di default: `until_first_gate`

Output:

- `data_analisys/outputs/unstructured_model_validation/comparison.json`
- `data_analisys/outputs/unstructured_model_validation/comparison_metrics.csv`
- `data_analisys/outputs/unstructured_model_validation/fit_results.json`
- `data_analisys/outputs/unstructured_model_validation/*.svg`

Puoi cambiare i report:

```bash
python -m data_analisys.model_validation \
  --sim-baseline reports/thesis_planner_unstructured_wide_slalom_gui_auto_20260317_032610_597.json \
  --robot-run reports/thesis_hardware_unstructured_manual_gate_editor_gui_manual_20260418_181318_932.json \
  --window until_first_gate
```

Preset disponibili:

```bash
python -m data_analisys.model_validation --list-presets
```

Esempi strutturati:

```bash
python -m data_analisys.model_validation --preset structured_validation_road

python -m data_analisys.model_validation \
  --preset structured_validation_road \
  --robot-glob 'reports/thesis_hardware_structured_validation_road_gui_manual_20260422*.json'

python -m data_analisys.model_validation --preset structured_figure_eight
```

Sweep di tuning structured:

```bash
python -m data_analisys.tune_structured_sim --limit 8
```

Lo sweep usa il simulatore headless sulla `validation road`, applica override del plant
e classifica i candidati rispetto alla baseline reale structured.

Note sui preset strutturati:

- `structured_validation_road` e gia una baseline buona e coerente
- `structured_figure_eight` e provvisorio ma utile per iniziare a leggere il mismatch
- nei report del 22-23 aprile non c'e un `circle_loop` hardware nominato esplicitamente; il test structured piu vicino salvato in JSON e `figure_eight`

Finestre disponibili:

- `full`: tutta la run
- `until_first_gate`: dall'inizio al primo gate completato, se presente
- `longest_reference`: finestra continua piu lunga con `planner_has_reference`
- `reference_only`: tutti i campioni con `planner_has_reference`
- `custom`: intervallo manuale con `--custom-start` e `--custom-end`

Aggregato multi-run:

```bash
python -m data_analisys.model_validation \
  --sim-baseline reports/thesis_planner_unstructured_wide_slalom_gui_auto_20260317_032610_597.json \
  --robot-glob 'reports/thesis_hardware_unstructured_manual_gate_editor_gui_manual_20260418*.json' \
  --window until_first_gate
```

Nota importante: il confronto di default usa tempo normalizzato e distanza-to-goal
normalizzata perche la mappa simulata e il test reale non sono ancora lo stesso ambiente.
Per una validazione finale forte, andranno raccolti run simulati e reali con lo stesso
protocollo di gate/mappa.

## Analisi unstructured con gate dinamici LiDAR

Per la validazione unstructured con gate generati dal LiDAR, il confronto punto-a-punto
non e' equivalente alla modalita structured. Usa invece la pipeline dedicata:

```bash
python -m data_analisys.unstructured_dynamic_gate_analysis
```

Per aprire una app Matplotlib interattiva con salvataggio PNG/SVG/PDF:

```bash
python -m data_analisys.unstructured_dynamic_gate_app
```

Output principali:

- `data_analisys/outputs/unstructured_dynamic_gate_validation_20260503/dynamic_gate_metrics.csv`
- `data_analisys/outputs/unstructured_dynamic_gate_validation_20260503/thesis_unstructured_dynamic_gate_analysis.md`
- figure SVG su lock del gate, reference windows, timing e clearance
- copia stabile in `documentation/Unstructured_Dynamic_Gate_Data_Analysis_20260503.md`

## Analisi mixed simulazione vs hardware

Per la modalita mixed usa la pipeline dedicata, che normalizza progresso,
deviazione dalla road e finestre gate tra simulazione e run reali:

```bash
python -m data_analisys.mixed_model_validation \
  --simulation reports/<mixed_hardware_aligned_sim>.json \
  --hardware reports/thesis_hardware_mixed_mixed_hardware_road_gate_gui_manual_20260512_052657_596.json \
  --hardware reports/thesis_hardware_mixed_mixed_hardware_road_gate_gui_manual_20260512_053236_711.json
```

Output principali:

- `data_analisys/outputs/mixed_model_validation_20260514/mixed_metrics.csv`
- `data_analisys/outputs/mixed_model_validation_20260514/mixed_metrics.json`
- `data_analisys/outputs/mixed_model_validation_20260514/mixed_model_validation_summary.md`
- figure SVG su `road_deviation_normalized`, `progress_normalized`, `gate_windows` e metriche aggregate

Per generare una baseline simulata piu vicina ai run reali mixed:

```bash
build/simulator/thesis_planner_sim \
  --headless \
  --scenario mixed \
  --mixed-map hardware_aligned \
  --dynamic-lidar-gates \
  --max-steps 2400
```

## Idea del fitting

Il fitting iniziale e volutamente semplice e ispezionabile. Per ogni segnale trattiamo il
robot come un sistema discreto del primo ordine:

```text
y[k+1] = a*y[k] + b*u[k] + c
```

Dove:

- `y` e il segnale misurato da predire
- `u` e un comando derivato dal PWM
- `a` misura quanta inerzia/memoria ha il sistema
- `b` misura quanto il comando influenza il prossimo campione
- `c` assorbe bias, offset statici e drift lento

Per ora vengono stimati:

- velocita da `PWM medio assoluto`
- yaw-rate da `PWM differenziale`

Dai parametri ricaviamo anche:

- `c`, mostrato anche nella app, utile per leggere deadzone residua, attrito statico o trim
- `gain = b / (1 - a)`, se il sistema e stabile
- `tau = -dt / log(a)`, una costante di tempo approssimata quando `0 < a < 1`
- `rmse`, errore medio quadratico della previsione one-step

Questa non e ancora l'identificazione finale del modello veicolo: e uno scaffold robusto
per capire se i segnali sono coerenti, se serve un ritardo ingresso, se il PWM e un buon
input e quanto il reale si discosta dalla simulazione.

## Prossimo passo naturale

Quando iniziamo il fitting, conviene aggiungere un modulo separato, per esempio
`fit_models.py`, che consumi `summary.csv` o le serie temporali estratte dai JSON.
La separazione e intenzionale: prima consolidiamo metriche e plot, poi scegliamo i
modelli di fitting senza mischiare visualizzazione e stima.
