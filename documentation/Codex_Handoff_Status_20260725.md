# Stato di handoff Codex - 2026-07-25

Questo file e' il punto di ripartenza per una nuova chat. Lo stato qui sotto e'
successivo al refactor MVC e alla matrice finale del 25 luglio 2026.

## Ambito concordato

La regressione obbligatoria e' stata ristretta ai preset realmente usati:

- structured: `Validation Road` e `Circle Loop`;
- unstructured: tutti i preset deterministici (`Robot Validation`, `Tight
  Corridor`, `Wide Slalom`, `Lower Bypass`, `Hardware Lab`, `Ideal Hardware
  Lab`); il Manual Gate Editor non ha una geometria riproducibile;
- mixed: `Closed Obstacle Road` e `Hardware Lab`;
- entrambi i robot e i livelli Sim-Ideal/Sim-Calibrated.

ZigZag, Hardware Track, Figure Eight e Tank Circuit restano preset legacy ma
non sono un gate di regressione. Le loro geometrie sperimentali sono state
ripristinate; non proseguire la taratura di questi preset senza una richiesta
esplicita.

## Refactor MVC completato

I nuovi confini sono descritti in `documentation/MVC_Code_Map.md`; la guida
dettagliata ai flussi, ai simboli principali e ai punti di modifica e' in
`documentation/Guida_Tecnica_Punti_Principali_Codice_20260726.md`.

Model:

- `model/navigation`: velocita' corpo/ruote, PWM, MPC, EKF e dinamica;
- `model/perception`: LiDAR simulato e point map;
- `model/reporting`: PNG SLAM di riferimento;
- `model/route`: proiezione continua e protezione anti short-track;
- `model/world`: scelta preset, scala, layout, raycast e collisioni.

Controller:

- `controller/application`: parsing CLI e report simulato;
- `controller/hardware_app`: composition root del runner hardware;
- `controller/hardware_io`: seriale, RSP-v1, LiDAR, posa esterna e robot bridge;
- `controller/hardware_planner`: orchestrazione runtime e telemetria hardware;
- `controller/simulation_planner`: orchestrazione e telemetria simulate;
- `controller/slam`: bridge opzionale verso slam_toolbox.

View/ViewModel:

- `view/canvas`: canvas comune;
- `view/live_stream`: stream della vista hardware;
- `view/simulator_app`: ViewModel, composition root e pannelli ImGui.

Gli header sono organizzati sotto `simulator/include/mvc/model`, `controller` e
`view`; gli smoke test sono in `simulator/src/tests`. Non restano sorgenti o
header C++ sciolti nelle radici `simulator/src` e `simulator/include`.

`view/simulator_app/main.cpp` non contiene parsing CLI, rasterizzatore PNG o report JSON
simulato, geometria canvas o scelta/normalizzazione degli scenari. Il runner
hardware usa lo stesso Actuation Model per il PWM.

### Split dei monoliti in package - 2026-07-26

I sette file sopra 1.000 righe sono stati sostituiti da wrapper `.cpp` piccoli
e partizioni nominate `parts/*.inc`: hardware planner, simulation planner,
simulator app, live stream, world, hardware app e hardware protocol smoke.
La concatenazione delle partizioni e' stata verificata tramite SHA-256 prima
della sostituzione. Il contenuto e l'ordine testuale erano identici alle
sorgenti precedenti; soltanto include e percorsi sono poi stati aggiornati.

La mappa dei package e' in `simulator/src/README.md`; il dettaglio del runner
reale e' in `simulator/src/controller/hardware_planner/README.md`. IntelliSense
usa `build/compile_commands.json`, include path espliciti e associa `*.inc` al
linguaggio C++ tramite `.vscode/c_cpp_properties.json` e `.vscode/settings.json`.

Verifica post-split:

```text
build                                      = riuscita
CTest                                      = 4/4
structured Validation + Circle             = 8/8
matrice primaria                           = 31 goal / 9 collisioni note
artefatti JSON + SLAM PNG                  = 40/40
runner hardware sintetico Hardware Lab     = 4/4 goal reached
```

### Refactor strutturale finale delle directory

Il 25 luglio 2026 anche tutti i sorgenti e gli header legacy rimasti nelle
radici sono stati spostati nei rispettivi confini MVC. Il cambiamento ha
riguardato soltanto percorsi, direttive `#include` e riferimenti CMake; corpi di
funzione, costanti, protocolli e parametri hardware non sono stati modificati.

Verifica successiva allo spostamento:

```text
file C++ direttamente in simulator/src     = 0
header direttamente in simulator/include   = 0
build                                       = riuscita
CTest                                       = 4/4
matrice primaria                            = 31 goal / 9 collisioni note
artefatti JSON + SLAM PNG                   = 40/40
runner hardware sintetico Hardware Lab      = 4/4 goal reached
```

## Scala metrica verificata

I 40 report della matrice confermano per entrambi i robot e in tutte le
modalita':

```text
body_length = 0.25 m
body_width  = 0.15 m
wheelbase   = 0.18 m
track       = 0.13 m
```

Structured usa arena `1.20 x 1.20 m` e centerline con span massimo `0.90 m`.
Le arene unstructured/mixed mantengono dimensioni diverse per preservare
varchi e ostacoli, ma la scala e il footprint del robot non cambiano piu'.

## Build e test unitari

Comandi eseguiti:

```sh
cmake -S . -B build -DBUILD_TESTING=ON
cmake --build build -j2
ctest --test-dir build --output-on-failure
```

Esito finale: `4/4` test passati.

```text
thesis_structured_path_projection
thesis_actuation_model_mapping
thesis_external_pose_udp_roundtrip
thesis_world_stream_roundtrip
```

Il vecchio smoke della world stream attendeva ancora `0.50 m`; e' stato
aggiornato alla specifica corrente `0.90 m`.

## Matrice del simulatore principale

Script:

```sh
./scripts/run_primary_mvc_regression_matrix.sh
```

Risultato completo:

```text
results/primary_mvc_regression_matrix/summary.tsv
```

Esito aggregato: `31/40 goal_reached`, `9/40 collision`.

| Gruppo | Goal | Totale | Nota |
| --- | ---: | ---: | --- |
| Structured Validation + Circle | 8 | 8 | tutti robot/livelli passano |
| Unstructured Robot Validation | 3 | 4 | tank calibrated esce dal bordo dopo 4 gate |
| Unstructured Tight Corridor | 1 | 4 | tre contatti con obstacle 4 |
| Unstructured Wide Slalom | 3 | 4 | car ideal esce dal bordo dopo 4 gate |
| Unstructured Lower Bypass | 2 | 4 | car ideal e tank calibrated escono dal bordo |
| Unstructured Hardware + Ideal Lab | 8 | 8 | tutti passano |
| Mixed Closed Obstacle Road | 2 | 4 | tank 2/2; car 0/2 |
| Mixed Hardware Lab | 4 | 4 | tutti passano |

Tutti i `40/40` casi, inclusi quelli conclusi per collisione, hanno prodotto
entrambi gli artefatti non vuoti:

```text
<report>.json
<report>_slam_reference.png
```

## Collisioni da affrontare insieme

Non sono state mascherate con tolleranze piu' larghe.

1. Quattro fallimenti unstructured sono uscite dal bordo quando il robot ha
   gia' passato quattro gate: Robot Validation tank calibrated, Wide Slalom
   car ideal, Lower Bypass car ideal e tank calibrated. Il goal/finale e' troppo
   vicino al bordo rispetto al footprint orientato.
2. Tre fallimenti Tight Corridor sono contatti coerenti con `obstacle[4]`, non
   errori di report: car calibrated e tank ideal/calibrated.
3. Closed Obstacle Road car ideal/calibrated arriva sul bordo superiore; la
   calibrated interseca anche `obstacle[3]`. Il tank completa entrambi i livelli.

Uno smoke separato del runner hardware sintetico sulla Closed Obstacle Road car
ha riprodotto la debolezza: timeout a `240 s`, safety stop LiDAR a `0.1745 m`,
`passed_gates=0`. Non e' quindi soltanto un artefatto della GUI o del refactor.

Come concordato con l'utente, fermarsi prima di modificare queste tarature e
scegliere insieme se intervenire su geometria/goal, margine del gate finale o
traiettoria di bypass della car.

## Smoke runner hardware riusciti

Risultato:

```text
results/primary_hardware_runner_smoke/summary.tsv
```

| Modalita' | Preset | Robot | Esito | Tempo | Gate |
| --- | --- | --- | --- | ---: | ---: |
| Unstructured | Hardware Lab | Car | goal reached | 51.0 s | 2 |
| Unstructured | Hardware Lab | Tank | goal reached | 52.7 s | 2 |
| Mixed | Hardware Lab | Car | goal reached | 17.6 s | 2 |
| Mixed | Hardware Lab | Tank | goal reached | 61.8 s | 2 |

## Report scientifici e `TO REPLACE`

Il refactor e gli smoke software non autorizzano la sostituzione dei risultati
scientifici mancanti. Per rimuovere tutti i `TO REPLACE` restano necessarie le
prove conservate, inclusi fallimenti e timeout, con scenario e posa iniziale
accoppiati. Il conteggio discusso e':

```text
solo real: 3 modalita' x 2 robot x 10 tentativi = 60 prove
Sim-Ideal + Sim-Calibrated + Real accoppiati     = 180 esecuzioni
```

Il motion capture serve per i risultati con riferimento esterno e relativa
incertezza. Non presentare la matrice software come sostituto di queste prove.

## Comandi di ripartenza

```sh
cd /home/isma/Desktop/Autonomus-Agent
git status --short
cmake --build build -j2
ctest --test-dir build --output-on-failure
column -t -s $'\t' results/primary_mvc_regression_matrix/summary.tsv
```

Prima di nuovi cambiamenti leggere `documentation/MVC_Code_Map.md`. La worktree
contiene modifiche precedenti dell'utente e un submodule gia' sporco; non usare
reset distruttivi e non alterare file fuori dallo scope.
