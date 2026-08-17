# Mappa della pipeline: gate, clotoide, MPC e PWM

Aggiornata al 28 luglio 2026. Questa nota indica i punti da aprire per seguire
una decisione dalla misura hardware fino ai motori. La pipeline hardware e'
assemblata da:

- `simulator/src/controller/hardware_app/main.cpp`: opzioni, profilo robot,
  connessione e streaming GUI;
- `simulator/src/controller/hardware_planner/runner.cpp`: include i moduli del
  controller nell'ordine `support`, `lifecycle`, `planning`, `localization`,
  `gap_detection`, `gap_workflow`, `trajectory`, `control`, `sensing_runtime`;
- `simulator/include/mvc/controller/hardware_planner/runner.h`: strutture di
  configurazione, stato persistente e interfaccia pubblica del runner.

## Flusso completo di un ciclo hardware

Il punto migliore da cui iniziare e' `HardwarePlannerRunner::step()` in
`simulator/src/controller/hardware_planner/parts/sensing_runtime.inc`.

```text
seriale MCU + LiDAR
        |
        v
RealRobotObservation e controllo timestamp
        |
        v
encoder + IMU -> EKF -> posa stimata
        |
        +---- LiDAR -> hit globali -> gate candidati persistenti
        |                              |
        |                              v
        |                    lista di gate attivi
        |                              |
        v                              v
stato del planner -----------------> sel_jr()
                                      |
                            j/r + gate scelto + clotoide
                                      |
                                      v
                         campionamento della clotoide
                                      |
                                      v
                              MPC path follower
                                      |
                         target speed + target yaw rate
                                      |
                                      v
                         velocita' ruote sinistra/destra
                                      |
                                      v
                         feed-forward + feedback -> PWM
                                      |
                                      v
                         RSP seriale -> microcontrollore
```

Dentro `step()` l'ordine effettivo e':

1. sincronizzazione e scarto delle misure vecchie;
2. `update_estimate_from_observation()`;
3. `correct_pose_with_lidar()` e `update_lidar_hits_world()`;
4. `rebuild_dynamic_gap_gates()` e `update_unstructured_gap_workflow()`;
5. `sync_planner_from_estimate()` e `refresh_gate_diagnostics()`;
6. `plan_if_needed()`;
7. `update_planner_references()` e `update_selected_trajectory()`;
8. `compute_control_command()`;
9. `send_last_control_command()`.

## LiDAR e costruzione geometrica dei gate

### Acquisizione dello scan

- `simulator/src/controller/hardware_io/rplidar_a1.cpp`: protocollo e lettura
  dei nodi del RPLidar A1;
- `RealRobotBridge::read_lidar_scan()` e `refresh_lidar_snapshot()` in
  `simulator/src/controller/hardware_io/real_robot_bridge.cpp`: creano lo scan
  con timestamp iniziale, finale, centrale, durata e flag di riuso;
- `HardwarePlannerRunner::update_lidar_hits_world()` in
  `parts/localization.inc`: trasforma i raggi dalla posa del LiDAR al frame
  globale e aggiorna la ricostruzione locale.

### Estrazione dei varchi

Il punto principale e' `HardwarePlannerRunner::rebuild_dynamic_gap_gates()` in
`simulator/src/controller/hardware_planner/parts/gap_detection.inc`.

La funzione:

1. elimina misure invalide e self-hit;
2. ordina i raggi per angolo e applica filtro mediano/outlier;
3. individua settori liberi delimitati da due ritorni LiDAR;
4. verifica che la larghezza sia maggiore del footprint e del margine;
5. calcola il centro geometrico reale del varco come punto medio cartesiano:

   ```text
   aperture_center = 0.5 * (left_boundary.hit + right_boundary.hit)
   ```

   Non viene usata la semplice media degli angoli, che sposterebbe il centro
   verso il palo piu' vicino;
6. colloca `target` poco oltre l'apertura, lungo la direzione del centro;
7. controlla segmento libero, occupancy grid, clearance e supporto dello scan;
8. calcola uno score con larghezza, profondita', distanza, clearance,
   allineamento, progresso, continuita' e copertura laterale;
9. associa il candidato a un `DynamicGapTrack` persistente e filtra posizione,
   punto di attraversamento, larghezza e confidenza su piu' scansioni;
10. pubblica soltanto track confermate nelle liste `gate_specs_` e `gates_`.

Le due liste hanno scopi diversi:

- `gate_specs_` e' la rappresentazione applicativa: nome/ID persistente,
  `position`, `anchor_position` al centro dell'apertura, heading e flag final;
- `gates_` e' la `std::vector<gate>` compatibile con il planner originale e
  contiene coordinate, clotoide associata, `passed`, `choose`, `too_far` e
  `final`.

Lo stato delle liste si trova in `runner.h`; e' accessibile anche tramite
`gates()`, `gate_specs()` e `visible_gate_indices()`.

## Lista effettivamente inviata al planner

Il passaggio esatto e' in `HardwarePlannerRunner::plan_if_needed()` dentro
`simulator/src/controller/hardware_planner/parts/planning.inc`:

```cpp
const std::vector<int> active_indices = active_gate_indices();
std::vector<gate> active_gates;
for (int index : active_indices) {
    active_gates.push_back(gates_[static_cast<size_t>(index)]);
}

std::vector<double> commands = sel_jr(
    false, step_count_, false, nullptr, true, &active_gates,
    sim_, x0_, g_x0_, cl_, planner_traj_);
```

Quindi la lista realmente passata a `sel_jr()` e' `active_gates`, non
direttamente `gate_specs_`.

`active_gate_indices()` si trova in `parts/lifecycle.inc` e accetta soltanto
gate non passati e non marcati `too_far`. Dopo il ritorno del planner:

- `commands[0]` e' `j`;
- `commands[1]` e' `r`;
- `commands[2]` e' l'indice locale del gate scelto;
- l'indice locale viene rimappato nell'indice globale `chosen_gate_index_`.

`refresh_gate_diagnostics()` aggiorna `visible_gate_indices_` e legge il flag
`gate.choose` scritto dal planner.

Quando un gate dinamico e' stato confermato e bloccato,
`publish_locked_gap_goal()` in `parts/gap_workflow.inc` sostituisce
temporaneamente entrambe le liste con un solo gate. In questo modo un candidato
nuovo non puo' far oscillare l'obiettivo prima dell'attraversamento fisico.
`set_locked_gap_goal()` conserva inoltre:

- centro dell'apertura;
- target oltre il varco;
- normale del varco usata come heading terminale;
- larghezza utile del corridoio;
- posa e istante in cui e' iniziato l'inseguimento.

Il conteggio del passaggio e il rejoin mixed sono in
`count_mixed_gate_crossing_if_needed()` nello stesso file.

## Planner originale e significato di j/r

L'interfaccia e' `sel_jr()` in:

- `third_party/progettotesi/include/action_selection.h`;
- `third_party/progettotesi/src/lib/action_selection.cpp`.

Il planner costruisce un motor cortex per ogni comportamento/gate, combina le
salienze e sceglie l'azione con costo minimo. La creazione della clotoide verso
ogni gate e' in `gate_behaviour()` dentro
`third_party/progettotesi/src/lib/behaviours.cpp`.

Significato corretto:

- `j`: jerk longitudinale, cioe' variazione dell'accelerazione;
- `r`: jerk laterale del planner originale, non raggio di sterzata;
- `planner_traj_.long_traj` e `planner_traj_.lat_traj`: polinomi temporali
  scelti insieme all'azione;
- `cl_`: clotoide del comportamento/gate selezionato.

In `update_planner_references()` (`parts/trajectory.inc`) `last_j_` viene
integrato in accelerazione e poi in `planner_speed_ref_`. Sul cingolato la
primitiva laterale viene campionata e integrata in accelerazione di yaw e yaw
rate. Sulla car `r` non viene convertito direttamente in PWM: la clotoide scelta
diventa il riferimento geometrico che viene inseguito dall'MPC.

## Creazione e campionamento della clotoide

La clotoide scelta dal planner e' in `cl_.prev_road`. La conversione in una
traiettoria utilizzabile dal controller avviene in
`HardwarePlannerRunner::update_selected_trajectory()` dentro
`simulator/src/controller/hardware_planner/parts/trajectory.inc`.

La funzione:

1. proietta la posa stimata sulla clotoide e ricava ascissa `s` e offset
   laterale;
2. sceglie intervallo di lookahead e numero di campioni;
3. chiama `build_reference_waypoints()`;
4. produce `reference_trajectory_`, composta da posizione, yaw, curvatura e
   velocita' di riferimento;
5. produce `planned_trajectory_`, usata principalmente per GUI e report.

Nel Manual Gate Editor l'anchor MPC viene azzerato quando viene rigenerata la
clotoide locale, per evitare che l'indice della traiettoria precedente faccia
saltare direttamente al vecchio endpoint.

## Inseguimento della clotoide con MPC

I file principali sono:

- configurazione per modalita': `HardwarePlannerRunner::reset()` in
  `parts/lifecycle.inc`;
- algoritmo: `simulator/src/model/navigation/mpc_path_follower.cpp`;
- interfaccia e pesi: `simulator/include/mvc/model/navigation/mpc_path_follower.h`;
- applicazione del risultato: `HardwarePlannerRunner::compute_control_command()`
  in `parts/control.inc`.

`ModelPredictivePathFollower::solve()`:

1. cerca l'anchor piu' vicino senza tornare indietro arbitrariamente;
2. seleziona un punto preview lungo la clotoide;
3. calcola errore trasversale e di heading;
4. prova combinazioni di accelerazione e steering-rate per la car, oppure
   accelerazione e yaw-acceleration per il tank;
5. simula ogni combinazione lungo l'orizzonte;
6. minimizza costo di cross-track, heading, velocita', steering/yaw e sforzo;
7. restituisce `MpcCommand` con accelerazione, steering-rate, velocita', yaw,
   errori e indice anchor.

Per la car, `compute_control_command()` integra lo steering-rate, ricava:

```text
curvature = tan(steer_angle) / wheelbase
target_yaw_rate = target_speed * curvature
```

Nel normale inseguimento del Manual Gate Editor il comando proviene dall'MPC.
Il controllo diretto di yaw rimane una recovery per errore di heading molto
grande, ricerca senza gate o ostacolo; non e' il percorso nominale della
clotoide.

## Da velocita'/yaw ai PWM

Il mapping hardware e' nella seconda meta' di
`HardwarePlannerRunner::compute_control_command()` in `parts/control.inc`.

Prima vengono calcolate le velocita' fisiche dei due lati:

```text
v_left  = v - omega * track_width / 2
v_right = v + omega * track_width / 2
```

La funzione pura e' `wheel_speeds_from_body()` in:

- `simulator/include/mvc/model/navigation/actuation_model.h`;
- `simulator/src/model/navigation/actuation_model.cpp`.

`wheel_speed_to_pwm()` applica minimo efficace, bias, guadagno e scala del
singolo attuatore. In seguito `control.inc` aggiunge:

- feed-forward velocita' ruota -> PWM;
- feedback PI usando le velocita' misurate dagli encoder;
- fallback lineare/yaw se gli encoder non sono validi;
- rilevamento stallo e break-away PWM;
- limiti, slew-rate e bande PWM efficaci;
- differenza PWM minima nelle curve compatte, per evitare che una clotoide
  venga appiattita a `70/70`.

I parametri misurati della car sono in `config/robots/car_calibrated.json`.
`apply_robot_calibration_profile()` in
`simulator/src/controller/hardware_app/parts/stream_config.inc` li copia in
`HardwarePlannerConfig`.

L'ultimo passaggio host e' `send_last_control_command()` in
`parts/sensing_runtime.inc`:

- senza PID MCU invia `last_command_.pwm_left/right` in modalita'
  `SafeDirectPwm`;
- con `--mcu-wheel-pid` invia le velocita' ruota in m/s;
- per la car reale applica una sola volta lo scambio dei canali elettrici
  registrato da `controller_motor_channels_swapped`; encoder, planner e
  localizzazione restano sempre nel frame fisico sinistra/destra.

`RSPSerialBridge::send_pwm()` in
`simulator/src/controller/hardware_io/rsp_serial_bridge.cpp` limita a
`[-255, 255]`, codifica `MotorCmd` e scrive il frame seriale.

## Firmware low-level

Il firmware principale della car e':

`low_level/car/rspV1_arduino/rspV1_arduino.ino`

Punti principali:

- `handle_motor_cmd()`: riceve il payload RSP e sceglie direct PWM,
  safe-direct PWM o wheel velocity;
- `velocity_pid_output()`: PID di velocita' ruota, usato soltanto nella modalita'
  wheel velocity;
- `update_motors()`: watchdog, slew e aggiornamento ciclico dei PWM;
- `set_motor_hw()`: ultimo mapping su pin/direzione fisica;
- `update_encoders()` e `send_encoder_telemetry()`: acquisizione e telemetria
  degli encoder;
- `send_motor_state()`: PWM target e realmente applicati.

Con il vecchio firmware e senza `--mcu-wheel-pid`, la pipeline si ferma al
ramo safe-direct PWM; il PID presente nel sorgente non viene utilizzato.

Il firmware tank equivalente e' in
`low_level/tank/rspV1_tank/rspV1_tank.ino`.

## Sensori, sincronizzazione e localizzazione

- parsing RSP e sincronizzazione clock MCU/host:
  `RSPSerialBridge::handle_frame()` e `synchronize_mcu_time()` in
  `rsp_serial_bridge.cpp`;
- snapshot comune controller + LiDAR: `RealRobotObservation` in
  `simulator/include/mvc/controller/hardware_io/real_robot_bridge.h`;
- controllo di eta' e scarto misure stale: `HardwarePlannerRunner::step()`;
- encoder/IMU -> posa: `update_estimate_from_observation()` in
  `parts/localization.inc`;
- modello EKF: `simulator/src/model/navigation/state_estimator_ekf.cpp`;
- scan matching locale: `correct_pose_with_lidar()` in `parts/localization.inc`;
- correzione bounded da SLAM Toolbox: `apply_slam_pose_correction()` nello
  stesso file.

La conversione encoder usa raggio ruota, tick/giro e track width del profilo
calibrato. La posa esterna rimane ground truth/validazione e non sostituisce il
controllo nominale.

## SLAM, GUI, telemetria e report

- client C++ verso il sidecar: `simulator/src/controller/slam/slam_toolbox_bridge.cpp`;
- bridge ROS 2/SLAM Toolbox: `tools/slam_toolbox_bridge/bridge_node.py`;
- parametri SLAM: `tools/slam_toolbox_bridge/mapper_params.yaml`;
- stato e metriche del planner hardware:
  `simulator/src/controller/hardware_planner/telemetry_controller.cpp`;
- serializzazione live verso GUI:
  `simulator/src/view/live_stream/stream.cpp`;
- report hardware JSON/CSV/Markdown e PNG SLAM:
  `simulator/src/view/simulator_app/parts/reporting.inc`;
- pulsanti di esportazione del bundle:
  `simulator/src/view/simulator_app/parts/mission_panels.inc`.

`planner_has_reference`, `chosen_gate_index`, `candidate_gates`, target yaw,
PWM pianificati, PWM del controller, tick encoder, timing sensori e stato SLAM
sono i campi piu' utili per seguire una run end-to-end.

## Geometria delle mappe e del robot

- modello centrale di mondo: `simulator/include/mvc/model/world/world.h`;
- costruzione per modalita': `simulator/src/model/world/scenario_model.cpp`;
- Structured: `simulator/src/model/world/parts/structured_presets.inc`;
- Unstructured: `simulator/src/model/world/parts/unstructured_presets.inc`;
- Mixed/Closed Obstacle Road: `simulator/src/model/world/parts/mixed_presets.inc`;
- primitive e normalizzazione geometrica:
  `simulator/src/model/world/parts/geometry_support.inc`;
- modifiche runtime/editor e sanitizzazione:
  `simulator/src/model/world/parts/runtime_layout.inc`;
- collisioni e ray casting LiDAR:
  `simulator/src/model/world/world_geometry_model.cpp`;
- dimensioni e calibrazione car/tank: `config/robots/*.json`.

## Equivalenti nella simulazione

La stessa separazione esiste nella pipeline simulata:

- gate LiDAR persistenti:
  `simulator/src/controller/simulation_planner/parts/dynamic_gates.inc`;
- lista gate e chiamata a `sel_jr()`:
  `parts/planning_sensors.inc`;
- arbitraggio road/gate/rejoin mixed:
  `parts/mixed_arbitration.inc`;
- campionamento clotoide e MPC: `parts/trajectory.inc`;
- ciclo e applicazione comando: `parts/runtime.inc`;
- plant car/tank, ruote, PWM, motori ed encoder:
  `simulator/src/model/navigation/vehicle_dynamics.cpp`.

## Diagnosi rapida per sintomo

| Sintomo | Primo punto da controllare |
| --- | --- |
| Nessun gate rilevato | `gap_detection.inc`, scan e `gate_specs_` |
| Gate geometrico errato | `aperture_center`, target e score in `gap_detection.inc` |
| Gate cambia continuamente | `DynamicGapTrack`, lock e `gap_workflow.inc` |
| Planner sceglie il gate sbagliato | `active_gates`, `sel_jr()` e `commands[2]` |
| Clotoide disegnata male | `gate_behaviour()` e `cl_.prev_road` |
| Clotoide corretta ma robot va dritto | `MpcCommand`, target yaw, PWM differenziale e `send_last_control_command()` |
| Robot gira nella direzione opposta | ordine canali nel profilo e telemetria encoder |
| Posa deriva | tick/giro, raggio, segni encoder, IMU, timestamp ed EKF |
| Scala mappa errata | preset, `scenario_model.cpp`, editor e profilo robot |
| PNG SLAM incoerente | sidecar, `slam_toolbox_bridge.cpp` e `reporting.inc` |

