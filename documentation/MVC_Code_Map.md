# Mappa del codice MVC e dei flussi runtime

Questa nota descrive l'architettura corrente del simulatore e del runner hardware.
I nomi di funzione sono intenzionalmente riportati oltre ai file: sono riferimenti
piu' stabili dei numeri di riga durante i refactor successivi.

Per la spiegazione operativa completa dei cicli simulato e hardware, dei dati
scambiati fra i package e dei punti esatti in cui intervenire, vedere
`documentation/Guida_Tecnica_Punti_Principali_Codice_20260726.md`.

## Confini MVC

```text
Model
  simulator/src/model/
    navigation/           attuazione, MPC, EKF e dinamica car/tank
    perception/           LiDAR simulato e accumulo endpoint SLAM
    reporting/            raster PNG associato ai report
    route/                proiezione e campionamento dei circuiti chiusi
    world/                preset, scala, raycast e collisioni

Controller
  simulator/src/controller/
    application/          opzioni applicative e report simulati
    hardware_app/         composition root hardware
    hardware_io/          seriale, RSP-v1, RPLidar e bridge del robot
    hardware_planner/     planner, localizzazione, gap workflow e controllo
    simulation_planner/   planner, mixed arbitration e sensori simulati
    slam/                 bridge opzionale verso slam_toolbox

View
  simulator/src/view/
    canvas/               disegno condiviso del mondo e del robot
    live_stream/          trasporto della vista hardware live
    simulator_app/        composition root, pannelli ImGui ed editor

Test
  simulator/src/tests/
    unit/                 contratti dei Model
    integration/          roundtrip UDP e world stream
    hardware_protocol/    smoke standalone del protocollo hardware
```

Gli header seguono lo stesso ordinamento sotto `simulator/include/mvc`. La mappa
rapida dei package e' anche in `simulator/src/README.md`. I composition root
sono `view/simulator_app/main.cpp` e `controller/hardware_app/main.cpp`.

I monoliti storici sono ora piccoli wrapper `.cpp` che includono partizioni
nominate in `parts/*.inc`. Le partizioni restano nella stessa translation unit
e nello stesso ordine testuale della sorgente precedente: lo split migliora la
navigazione senza cambiare linkage o comportamento, in particolare nel codice
hardware.

## Flusso simulato

```text
CLI/GUI
  -> mvc::controller::parse_app_options
  -> mvc::model::make_world_from_mode
  -> mvc::model::fit_simulation_structured_world
  -> PlannerDrivenVehicleSim::step
       -> update_dynamic_lidar_gates
       -> plan_if_needed / sel_jr
       -> update_selected_trajectory
       -> ModelPredictivePathFollower::solve
       -> VehicleDynamicsModel::step
       -> mvc::model::acquire_simulated_lidar_scan
       -> StateEstimatorEkf
       -> completion contract + telemetry
  -> mvc::controller::write_json_report
  -> mvc::model::write_slam_reference_png
```

## Flusso hardware

```text
RSP serial + RPLidar A1
  -> RealRobotBridge::poll
  -> RealRobotObservation
  -> HardwarePlannerRunner::step_with_observation
       -> update_estimate_from_observation
       -> update_lidar_hits_world
       -> rebuild_dynamic_gap_gates
       -> update_unstructured_gap_workflow
       -> plan_if_needed / sel_jr
       -> compute_control_command
       -> mvc::model::wheel_speeds_from_body
       -> mvc::model::wheel_speed_to_pwm
  -> RealRobotBridge::send_pwm
  -> RSPSerialBridge::send_pwm
  -> firmware RSP-v1
```

## Dove si trovano i pezzi importanti

| Funzione | File e simboli principali | Dato in uscita / consumatore |
| --- | --- | --- |
| Lista dei gate percepiti in simulazione | `simulator/src/controller/simulation_planner/parts/dynamic_gates.inc`: `extract_dynamic_lidar_gate_candidates`, `update_dynamic_lidar_gates`, `active_gate_indices` | aggiorna `gates_`; `plan_if_needed` costruisce il vettore di gate passato a `sel_jr` |
| Lista dei gate percepiti su robot | `simulator/src/controller/hardware_planner/parts/gap_detection.inc` e `gap_workflow.inc`: `rebuild_dynamic_gap_gates`, `update_unstructured_gap_workflow`, `active_gate_indices` | aggiorna `gates_`; `plan_if_needed` passa soltanto i gate attivi al planner |
| Gate scelto e stato | `chosen_gate_index_`, `visible_gate_indices_`, flag `gate::passed/choose/too_far/final` negli header `hardware_planner/runner.h` e `simulation_planner/simulator.h` | follower locale, GUI, telemetria e criterio di missione |
| Scansione LiDAR simulata | `simulator/src/model/perception/perception_model.cpp`: `acquire_simulated_lidar_scan` | `PlannerDrivenVehicleSim::update_lidar` |
| Protocollo RPLidar A1 | `simulator/src/controller/hardware_io/rplidar_a1.cpp`: `start_scan`, `read_scan_node`, `grab_scan`, `process_scan_packet` | `RealRobotObservation::lidar_scan` |
| Trasformazione scan base-link -> world | `HardwarePlannerRunner::update_lidar_hits_world` | `lidar_hits_`, diagnostica e mapping |
| Primitive del planner | `PlannerDrivenVehicleSim::plan_if_needed` e `HardwarePlannerRunner::plan_if_needed`; planner esterno in `third_party/progettotesi/src/lib/` | jerk longitudinale `j`, rate della curvatura/yaw `r`, indice gate |
| Riferimento e tracking | `hardware_planner/parts/trajectory.inc`, `simulation_planner/parts/trajectory.inc`; `model/navigation/mpc_path_follower.cpp`: `ModelPredictivePathFollower::solve` | velocita', accelerazione, sterzo o yaw-rate desiderati |
| Circuiti chiusi e anti short-track | `simulator/src/model/route/structured_path_model.cpp`: `project_closed_path`, `sample_closed_path_span`; contratto finale in `simulation_planner/parts/runtime.inc` | progressione monotona, riferimento locale, giro completo vicino allo start |
| Mapping primitive -> ruote -> PWM | `simulator/src/model/navigation/actuation_model.cpp`: `wheel_speeds_from_body`, `wheel_speed_to_pwm` | PWM sinistro e destro con clamp, slew, dead-zone e boost di partenza |
| Invio PWM al controller | `hardware_planner/parts/control.inc`, `RealRobotBridge::send_pwm`, `hardware_io/rsp_serial_bridge.cpp`: `RSPSerialBridge::send_pwm` | frame RSP-v1 `SET_MOTOR_PWM` |
| Applicazione PWM nel firmware | `low_level/car/rspV1.cpp`: `set_targets`, `step_towards`, `set_motor_hw` | direzione pin e `analogWrite`; stessa logica e' replicata nello sketch Arduino |
| Costruzione geometrica dei preset | `simulator/src/model/world/parts/*_presets.inc` e `geometry_support.inc` | `WorldMap` metrico grezzo |
| Scala metrica comune | `simulator/src/model/world/scenario_model.cpp`: `fit_simulation_structured_world`, `fit_hardware_structured_world`; layout runtime in `world/parts/runtime_layout.inc` | arena structured 1.20 x 1.20 m, percorso 0.90 m, footprint 0.25 x 0.15 m per entrambi i robot |
| Raycast e collisioni | `simulator/src/model/world/world_geometry_model.cpp`: `WorldMap::raycast`, `line_of_sight`, `collides` | LiDAR, sicurezza e simulazione fisica |
| Input sensori del base link | `simulator/src/controller/hardware_io/real_robot_bridge.cpp`: `RealRobotBridge::poll`; `hardware_planner/parts/sensing_runtime.inc`: `step_with_observation` | encoder, IMU, stato controller e scan LiDAR sincronizzati in `RealRobotObservation` |
| Fusione della posa | `hardware_planner/parts/localization.inc`, `simulator/src/model/navigation/state_estimator_ekf.cpp` | `navigation_position_`, yaw, velocita', accelerazione e incertezza |
| SLAM nativo | `mvc::model::accumulate_slam_endpoints`; bridge opzionale in `controller/slam/slam_toolbox_bridge.cpp` | point cloud/occupancy per diagnostica, mai feedback nascosto al planner |
| Disegno mondo e robot | `simulator/src/view/canvas/world_canvas_view.cpp` | canvas ImGui comune a simulazione e hardware |
| Telemetria | `simulation_planner/telemetry_controller.cpp`, `hardware_planner/telemetry_controller.cpp` | history e `SimulationReport`/`HardwarePlannerReport` |
| JSON e immagine SLAM simulata | `controller/application/simulation_report_controller.cpp`; `model/reporting/slam_reference_exporter.cpp` | coppia `report.json` + `report_slam_reference.png` |
| Report hardware | `view/simulator_app/parts/reporting.inc`, alimentato dal ViewModel in `include/mvc/view/simulator_app/hardware_view_model.h` | JSON hardware e PNG della ricostruzione LiDAR |
| Streaming GUI hardware | `view/live_stream/stream.cpp` e `view/canvas/world_canvas_view.cpp` | snapshot world/robot/LiDAR con la stessa scala della simulazione |

## Gate: dalla percezione al planner

In simulazione, `update_dynamic_lidar_gates` riceve i candidati prodotti da
`extract_dynamic_lidar_gate_candidates`, applica stabilita', ampiezza minima,
hold temporale e deduplicazione, quindi aggiorna `gates_`. Su hardware lo stesso
contratto e' implementato da `rebuild_dynamic_gap_gates` e dalla macchina a
stati `update_unstructured_gap_workflow`. In entrambi i runner:

1. `active_gate_indices` esclude gate passati, finali o fuori finestra;
2. `plan_if_needed` copia soltanto gli elementi attivi in `active_gates`;
3. `sel_jr(..., active_gates, ...)` restituisce primitive e indice locale;
4. l'indice viene rimappato su `gates_` e memorizzato in
   `chosen_gate_index_`;
5. il gate finale resta un target fisico e non viene contato come attraversato
   a distanza.

## Sensori del base link

`RealRobotObservation` e' il confine fra I/O e controllo. Contiene:

- telemetria RSP-v1 del controller, inclusi tick/delta encoder, PWM applicati,
  stato di sicurezza e IMU quando disponibile;
- scan LiDAR nel frame del sensore;
- timestamp monotono host e indicatori di freschezza.

`update_estimate_from_observation` converte encoder e IMU nel frame base-link e
aggiorna l'EKF. `update_lidar_hits_world` applica extrinseci e posa stimata per
la visualizzazione/mappa. Il riferimento motion-capture o ArUco e' esportato
come ground truth indipendente e non entra nel controllo.

## Report e artefatto SLAM

Ogni percorso che salva un JSON chiama anche l'esportatore del Model. Il nome
e' deterministico:

```text
<stem>.json
<stem>_slam_reference.png
```

Il PNG contiene geometria di riferimento, occupazione LiDAR, trail, start e
goal. Il salvataggio viene considerato riuscito soltanto se entrambi gli
artefatti sono stati scritti.

## Test di regressione

```sh
cmake -S . -B build -DBUILD_TESTING=ON
cmake --build build -j2
ctest --test-dir build --output-on-failure
./scripts/run_primary_mvc_regression_matrix.sh
```

I test unitari MVC includono il mapping PWM e la proiezione su circuito chiuso.
Quest'ultimo verifica esplicitamente che la proiezione non attraversi lo start
prima che il robot vi sia fisicamente ritornato. La matrice applicativa copre
Validation Road e Circle Loop in structured, tutti i preset unstructured
deterministici e Closed Obstacle Road/Hardware Lab in mixed. I preset structured
legacy non usati nelle prove non fanno parte del gate di regressione.
