# Guida tecnica ai punti principali del codice

Data: 2026-07-26

Questa nota e' il riferimento operativo per capire il simulatore, il runner
hardware e il percorso completo dei dati. Descrive dove si trovano i componenti
e come cooperano durante un singolo ciclo di controllo.

I nomi dei simboli sono preferiti ai numeri di riga: restano validi anche quando
una funzione viene spostata. I file `parts/*.inc` sono partizioni di una stessa
translation unit e vengono inclusi dal piccolo wrapper `.cpp` del package.

Salvo quando il percorso inizia esplicitamente da `simulator/`, tutti i percorsi
di sorgenti `.cpp` e `parts/*.inc` citati nella guida sono relativi a
`simulator/src/`; i percorsi degli header sono relativi a
`simulator/include/mvc/`.

## Percorso di lettura consigliato

Per capire il progetto senza leggerlo tutto in ordine:

1. `simulator/include/mvc/model/world/world.h`: tipi geometrici e `WorldMap`;
2. `simulator/include/mvc/controller/simulation_planner/simulator.h`: stato del
   simulatore e configurazione;
3. `simulator/include/mvc/controller/hardware_planner/runner.h`: configurazione,
   diagnostica e stato hardware;
4. `controller/simulation_planner/parts/runtime.inc`: ordine di un ciclo
   simulato;
5. `controller/hardware_planner/parts/sensing_runtime.inc`: ordine di un ciclo
   hardware;
6. `controller/hardware_planner/parts/planning.inc` e
   `controller/simulation_planner/parts/planning_sensors.inc`: ingresso nel
   planner;
7. `model/navigation/mpc_path_follower.cpp`: inseguimento della traiettoria;
8. `controller/hardware_planner/parts/control.inc`: produzione dei PWM.

## Organizzazione dei package

```text
simulator/src/
  controller/
    application/          opzioni e report simulati
    hardware_app/         avvio del runner hardware
    hardware_io/          seriale, RSP-v1, LiDAR e robot bridge
    hardware_planner/     controllo completo del robot reale
    simulation_planner/   controllo completo del simulatore
    slam/                 bridge opzionale verso slam_toolbox
  model/
    navigation/           dinamica, EKF, MPC, ruote e PWM
    perception/           modello del LiDAR simulato
    reporting/            raster SLAM PNG
    route/                progressione sui circuiti chiusi
    world/                mappe, scala, raycast e collisioni
  view/
    canvas/               disegno comune del mondo
    live_stream/          trasporto della vista hardware
    simulator_app/        GUI, editor e composizione dell'applicazione
  tests/
    unit/                 contratti dei Model
    integration/          roundtrip dei bridge
    hardware_protocol/    smoke standalone hardware
```

Gli header pubblici replicano questa struttura sotto
`simulator/include/mvc/`. La mappa sintetica dei package e' in
`simulator/src/README.md`.

## Tipi che costituiscono i confini del sistema

| Tipo | Dove | Ruolo |
| --- | --- | --- |
| `WorldMap` | `model/world/world.h` | bounds, start, goal, strada, ostacoli, gate e preset |
| `SimConfig` | `controller/simulation_planner/simulator.h` | sensori, livello ideal/calibrated, robot, timing e tuning simulato |
| `HardwarePlannerConfig` | `controller/hardware_planner/runner.h` | geometria, localizzazione, gap extraction, PWM e safety hardware |
| `RealRobotObservation` | `controller/hardware_io/real_robot_bridge.h` | snapshot sincronizzato di telemetria controller e scansione LiDAR |
| `HardwarePlannerEstimate` | `controller/hardware_planner/runner.h` | posa e stato cinematico stimati |
| `HardwareControlCommand` | `controller/hardware_planner/runner.h` | target cinematici e PWM finali |
| `TelemetrySample` | `controller/simulation_planner/simulator.h` | campione diagnostico del simulatore |
| `HardwareTelemetrySample` | `controller/hardware_planner/runner.h` | campione diagnostico hardware |
| `LiveSceneSnapshot` | `view/live_stream/live_view_stream.h` | parte statica della scena remota |
| `LiveFrameSnapshot` | `view/live_stream/live_view_stream.h` | stato runtime inviato alla GUI |

`WorldMap` e' la sorgente di verita' geometrica. Il planner non legge
direttamente la GUI e il firmware non legge direttamente la mappa: ogni livello
riceve una rappresentazione esplicita dal livello precedente.

## I due flussi end-to-end

### Ciclo simulato

Il punto centrale e' `PlannerDrivenVehicleSim::step` in
`controller/simulation_planner/parts/runtime.inc`.

```text
WorldMap
  -> aggiornamento gate/ostacoli dinamici
  -> percezione dei gate e arbitraggio mixed
  -> sel_jr: primitive planner j/r
  -> generazione del riferimento locale
  -> ModelPredictivePathFollower
  -> VehicleDynamicsModel
  -> LiDAR simulato e navigazione stimata
  -> collisione/completamento
  -> telemetria e report
```

L'ordine effettivo del ciclo e':

1. `WorldMap::update_gate_layout` aggiorna eventuali gate mobili;
2. `sync_gate_specs_from_world` mantiene sincronizzati gate scenario e planner;
3. `update_dynamic_lidar_gates` aggiorna i gate percepiti;
4. `update_mixed_arbitration` decide se usare strada o bypass;
5. `plan_if_needed` invoca il planner bio-inspired;
6. `update_planner_references` integra le primitive nel tempo;
7. `update_selected_trajectory` produce waypoint locali;
8. il follower calcola accelerazione, sterzo o yaw-rate;
9. `VehicleDynamicsModel::step` muove il robot fisico simulato;
10. `update_lidar` genera la scansione;
11. `update_navigation_state` aggiorna la posa percepita dal controllo;
12. `update_slam_map` accumula gli endpoint occupati;
13. vengono valutati collisione e contratto di completamento;
14. `update_telemetry` salva il campione del ciclo.

### Ciclo hardware

Il punto centrale e' `HardwarePlannerRunner::step_with_observation` in
`controller/hardware_planner/parts/sensing_runtime.inc`.

```text
Seriale RSP + RPLidar
  -> RealRobotBridge
  -> RealRobotObservation
  -> encoder/IMU/EKF
  -> LiDAR, occupancy e gate dinamici
  -> sel_jr: primitive planner j/r
  -> riferimento + MPC
  -> velocita' ruote + PWM
  -> RSPSerialBridge::send_pwm
  -> firmware
```

L'ordine effettivo e':

1. `RealRobotBridge::pump` legge controller e LiDAR;
2. `update_estimate_from_observation` fonde encoder e IMU;
3. `correct_pose_with_lidar` applica una correzione opzionale e limitata;
4. `update_lidar_hits_world` trasforma gli hit nel frame mappa;
5. `rebuild_dynamic_gap_gates` aggiorna i candidati attraversabili;
6. `update_unstructured_gap_workflow` gestisce scan, lock e attraversamento;
7. `sync_planner_from_estimate` copia lo stato stimato nel formato del planner;
8. `plan_if_needed` calcola `j`, `r` e l'eventuale gate scelto;
9. `update_planner_references` e `update_selected_trajectory` preparano il
   riferimento locale;
10. `compute_control_command` calcola target cinematici e PWM;
11. viene valutato il completamento fisico della missione;
12. `push_history` salva la telemetria;
13. soltanto alla fine `RealRobotBridge::send_pwm` invia il comando.

Il parametro `send_pwm` di `step_with_observation` permette di usare lo stesso
flusso con osservazioni sintetiche senza comandare una porta reale.

## Costruzione e scala delle mappe

### Selezione del mondo

File: `model/world/scenario_model.cpp`.

`mvc::model::make_world_from_mode` seleziona il costruttore corretto:

- structured: `WorldMap::structured_demo`;
- unstructured: `WorldMap::unstructured_demo`;
- mixed: uno dei costruttori `mixed_*_demo`.

Le geometrie sono divise in:

- `model/world/parts/geometry_support.inc`: helper geometrici;
- `unstructured_presets.inc`: scenari unstructured;
- `structured_presets.inc`: scenari structured;
- `mixed_presets.inc`: scenari mixed;
- `runtime_layout.inc`: aggiornamento gate/ostacoli ed editor.

### Normalizzazione metrica

`fit_hardware_structured_world` e `fit_simulation_structured_world` passano
entrambi da `normalize_structured_world`. Per i preset structured non custom:

Le dimensioni definitive sono centralizzate in
`model/navigation/vehicle_dynamics.h`, namespace `measured_robot`, e vengono
riusate da simulatore, runner hardware, collisioni, self-hit LiDAR e profili:

| Grandezza | Car | Tank |
| --- | ---: | ---: |
| body, lunghezza x larghezza | 0.250 x 0.150 m | 0.165 x 0.146 m |
| ruota / cingolo | diametro 0.0654 m | 0.158 x 0.0447 m |
| raggio ruota / raggio efficace | 0.0327 m, misurato | 0.032 m, provvisorio |
| distanza fra centri cingoli | - | 0.1013 m |

Per il tank `0.1013 = 0.146 - 0.0447`. La misura 0.032 m non e' una
dimensione esterna del cingolo: e' il raggio cinematico efficace ancora da
identificare con encoder e distanza reale. Il profilo lo dichiara infatti con
`wheel_radius_calibrated=false`.

I preset structured usati sono normalizzati in un'arena 1.20 x 1.20 m, con
span utile della strada pari a 0.90 m. Wheelbase e limiti cinematici restano
parametri del modello e del profilo, non vengono ricavati dal disegno GUI.

Le mappe unstructured e mixed non vengono miniaturizzate ciecamente: devono
preservare varchi compatibili con il footprint fisico. `validate_hardware_world`
controlla dimensioni, start/goal e margini; `sanitize_hardware_unstructured_world`
rimuove contenuto editor incompatibile quando il flusso richiede percezione
LiDAR pura.

### Raycast e collisioni

File: `model/world/world_geometry_model.cpp`.

- `WorldMap::raycast` interseca i raggi con bounds e ostacoli;
- `WorldMap::line_of_sight` verifica un segmento con padding;
- `WorldMap::collides` usa il poligono orientato del robot, non soltanto il suo
  centro.

Per questo una collisione puo' verificarsi anche quando `(x,y)` e' ancora
dentro l'arena: un angolo del footprint puo' gia' aver superato il bordo o
intersecato un rettangolo.

## LiDAR simulato

File: `model/perception/perception_model.cpp`.

`range_sensor_spec` traduce il profilo selezionato in numero di raggi, campo
visivo e portata. Il profilo RPLidar A1 usa 360 raggi su 360 gradi.

`acquire_simulated_lidar_scan`:

1. rende visibili gli ostacoli di percezione coerenti con tempo e progresso;
2. chiama `WorldMap::raycast` dalla posa fisica del sensore;
3. in Sim-Ideal restituisce direttamente gli hit;
4. in Sim-Calibrated applica rumore gaussiano alla distanza e dropout;
5. ricostruisce il punto finale dopo la perturbazione.

`accumulate_slam_endpoints` riproietta gli hit dal frame fisico del LiDAR nel
frame della posa di navigazione. I punti vengono quantizzati in celle, deduplicati
e limitati a `maximum_points`. Questa point map e' diagnostica: non sostituisce
la posa vera del simulatore.

## LiDAR reale e base-link

### Protocollo del sensore

File: `controller/hardware_io/rplidar_a1.cpp`.

`RPLidarA1::start_scan` avvia motore e stream. `grab_scan` legge e valida i
pacchetti, ricostruisce angolo, distanza e quality e restituisce
`std::vector<ScanPoint>`.

### Aggregazione dei sensori

File: `controller/hardware_io/real_robot_bridge.cpp`.

`RealRobotBridge` unisce due canali indipendenti:

- `RSPSerialBridge`: telemetria del base-link/controller;
- `RPLidarA1`: scansione del sensore di distanza.

Il risultato e' un `RealRobotObservation` contenente timestamp host,
`ControllerTelemetry`, scan LiDAR e flag di validita'. Una scansione buona viene
conservata per un breve fallback; se lo stream cade, il bridge prova letture
meno restrittive, usa temporaneamente la cache e programma la riconnessione.

### Sincronizzazione temporale

La sincronizzazione non usa l'istante in cui il planner legge lo snapshot come
timestamp di tutte le misure:

1. `RSPSerialBridge::synchronize_mcu_time` stima l'offset MCU-host da
   `mcu_time_ms` e timestamp monotono di ricezione;
2. l'offset viene filtrato e il residuo alimenta `mcu_clock_jitter_s`;
3. IMU, encoder, motor state, safety e heartbeat conservano sia timestamp di
   ricezione sia timestamp host sincronizzato;
4. `RealRobotBridge::capture_lidar_scan` conserva start, end, midpoint e durata
   originale dello scan;
5. `step_with_observation` misura l'eta' effettiva e scarta controller o LiDAR
   oltre `max_controller_age_s` / `max_lidar_age_s`;
6. `update_lidar_hits_world` interpola indietro posa e yaw per ogni beam lungo
   la durata della scansione quando `motion_compensate_scan=true`.

Le metriche `controller_age_s`, `lidar_age_s`, `lidar_scan_duration_s`,
`mcu_clock_offset_s`, `mcu_clock_jitter_s`, `stale_controller_drops` e
`stale_lidar_drops` sono disponibili nella diagnostica hardware. Una scansione
riutilizzata e' marcata esplicitamente; non viene fatta passare per nuova.

### Trasformazione base-link -> world

`HardwarePlannerRunner::scan_point_world_hit` e
`update_lidar_hits_world`, rispettivamente in `sensing_runtime.inc` e
`localization.inc`, applicano:

1. convenzione angolare e possibile flip del sensore;
2. offset estrinseco `lidar_x/y/yaw_offset` rispetto al base-link;
3. posa stimata `(x,y,yaw)` del robot;
4. filtro dei self-hit sul footprint;
5. trasformazione nel frame della mappa.

Gli hit alimentano visualizzazione, occupancy persistente, scan matching e
ricerca dei varchi.

## Telemetria RSP-v1, encoder e IMU

Package: `controller/hardware_io/`.

```text
SerialPort
  -> RSP StreamDecoder
  -> RSPSerialBridge::handle_frame
  -> ControllerTelemetry
  -> RealRobotObservation
```

- `serial_port.cpp` gestisce file descriptor, baudrate, timeout e read/write;
- `rsp_protocol.cpp` implementa frame, CRC, payload e decoder incrementale;
- `rsp_serial_bridge.cpp` gestisce ACK/errori, heartbeat, modalità, stop e PWM;
- `RealRobotBridge::refresh_controller_snapshot` copia l'ultimo snapshot
  completo nell'osservazione.

I dati principali usati dal controllo sono tick e delta encoder, `enc_dt_ms`,
flag di direzione, yaw/yaw-rate IMU, PWM applicati e target, safety flags,
motor flags, status flags ed error code.

## Stima della posa ed EKF

### Preparazione delle misure

File: `controller/hardware_planner/parts/localization.inc`.

`update_estimate_from_observation`:

1. converte yaw e yaw-rate IMU da milliradianti;
2. inizializza un offset tra frame IMU e frame mondo;
3. limita spike oltre l'inviluppo fisico del robot;
4. assorbe salti discontinui dello yaw nell'offset per non spostare la posa;
5. calcola i delta encoder e ne controlla la plausibilita';
6. converte tick in metri con raggio ruota e tick/giro;
7. ricava velocita' lineare e yaw-rate differenziale;
8. se gli encoder non sono affidabili, usa un fallback prudente basato su
   comando, PWM e IMU;
9. esegue predict e update dell'EKF.

### Stato del filtro

File: `model/navigation/state_estimator_ekf.cpp`.

`KinematicBicycleEkf` mantiene:

```text
[x, y, yaw, velocita', yaw_rate]
```

- `predict` integra odometria e propaga la covarianza con il rumore `Q`;
- `update_imu` corregge yaw e yaw-rate con rumore `R`;
- `update_lidar_pose` corregge posizione e, quando autorizzato, yaw;
- `sync_estimate_from_ekf_state` pubblica il risultato al runner.

Il bias giroscopico e' uno stato adattivo separato: viene appreso soltanto
quando encoder e yaw-rate indicano robot quasi fermo, poi viene sottratto alla
misura. Il rumore di processo cresce con `abs(yaw_rate)`; per il tank usa
`tracked_turn_process_noise_gain`, perche' skid e rotazioni sul posto hanno
incertezza strutturalmente maggiore. Le correzioni LiDAR sono protette da gate
di Mahalanobis e contatori accepted/rejected; una sequenza di rifiuti abilita
solo un recovery piccolo e controllato.

Il predict integra la velocita' nel frame del base-link. Non introduce uno
sideslip dedotto soltanto dalla curvatura: encoder e gyro non osservano tale
variabile e l'iniezione precedente produceva deriva anche in Sim-Ideal.

### Correzione LiDAR

`HardwarePlannerRunner::correct_pose_with_lidar` esegue una ricerca locale
attorno alla stima e minimizza lo score degli hit contro mappa o occupancy.
La correzione viene rifiutata se lo scan e' insufficiente, la policy non la
consente, il miglioramento e' debole o il robot sta ruotando in una condizione
che renderebbe lo scan matching instabile. Nei mondi compatti lo spostamento
longitudinale/laterale e' limitato esplicitamente.

In unstructured la point map nasce dalla stessa posa stimata: per evitare un
feedback circolare, le correzioni sono piccole e ammesse soprattutto durante
avanzamento reale.

### SLAM Toolbox

Il client opzionale e' in `controller/slam/slam_toolbox_bridge.cpp`; il sidecar
ROS 2 e la configurazione sono sotto `tools/slam_toolbox_bridge/`. Il runner
invia odometria, timestamp midpoint e scan, e riceve occupancy, posa corretta e
diagnostica del grafo.

Senza `--slam-pose-feedback` il canale e' soltanto diagnostico. Con
`--slam-bridge-port 9760 --slam-pose-feedback`,
`HardwarePlannerRunner::apply_slam_pose_correction` limita ogni innovazione a
6 cm e 0.08 rad, passa ancora dal gate statistico dell'EKF e rifiuta divergenze
oltre 0.75 m / 0.90 rad. Il recovery, dopo rifiuti consecutivi, e' limitato a
1.5 cm e 0.02 rad. La posa motion-capture rimane comunque indipendente e non
entra in questa catena.

### Posa esterna

`ExternalPoseUdpReceiver` e' in `controller/hardware_io/external_pose_receiver.cpp`.
`HardwarePlannerRunner::set_external_pose_reference` memorizza la misura, mentre
`controller/hardware_planner/telemetry_controller.cpp` calcola errore XY/yaw e
qualita'.

La posa esterna non viene usata da `compute_control_command`, dal planner o
dall'EKF: e' ground truth indipendente per report e validazione scientifica.

## Gate: rappresentazioni e flusso verso il planner

Esistono due rappresentazioni correlate:

- `GateSpec`: descrizione geometrica del mondo, adatta a editor e streaming;
- `gate`: struttura richiesta dal planner bio-inspired, con flag
  `passed`, `choose`, `too_far` e `final`.

`sync_gate_specs_from_world` costruisce la seconda dalla prima. I gate LiDAR
aggiornano entrambe per mantenere GUI, diagnostica e planner coerenti.

### Gate dinamici in simulazione

File: `controller/simulation_planner/parts/dynamic_gates.inc`.

`update_dynamic_lidar_gates` riceve i candidati estratti dagli hit, ordina per
score e pubblica o aggiorna il gate attivo. Applica:

- identita' numerica persistente `DynamicLidarGateTrack::id`;
- match spaziale con hit/miss e confidenza del track;
- smoothing di posizione, heading, larghezza e score;
- stabilita' minima prima della selezione;
- lock del track scelto fino all'attraversamento fisico;
- cambio soltanto verso candidati stabilmente migliori;
- rimozione immediata del track passato, per non ripubblicarlo;
- regole specifiche mixed per non perdere il bypass attivo.

Il target coincidente con il goal e' riconosciuto dalla posizione. Resta un
target selezionabile dal planner legacy, ma
`update_unstructured_gate_progress` lo esclude dal conteggio degli intermedi.
La traiettoria finale ha un fallback locale dedicato: il planner non puo'
rifiutare il goal e lasciare il robot in coast verso i bounds.

`update_unstructured_gate_progress` considera il passaggio lungo l'asse della
rotta e la distanza laterale, non soltanto un raggio dal centro. Nei preset
deterministici gli anchor semantici vengono pubblicati soltanto quando almeno
tre beam LiDAR confermano spazio libero oltre il target; non vengono mescolati
con settori arbitrari che cambierebbero il numero di gate della mappa.

### Gap extraction sul robot

File: `controller/hardware_planner/parts/gap_detection.inc`.

`rebuild_dynamic_gap_gates` realizza la pipeline completa:

1. filtra scan invalidi, range e self-hit;
2. ordina i beam per angolo;
3. applica filtro mediano;
4. elimina spike lontani incoerenti con i vicini;
5. costruisce una occupancy grid locale;
6. marca campioni liberi lungo ogni raggio e occupati agli endpoint;
7. gonfia gli ostacoli usando footprint e clearance;
8. individua settori liberi e discontinuita' di profondita';
9. misura la larghezza fisica dell'apertura;
10. scarta target troppo stretti, fuori bounds o con segmento occupato;
11. assegna uno score usando progresso, distanza, allineamento al goal,
    clearance e continuita';
12. associa i candidati a `DynamicGapTrack` persistenti;
13. richiede hit/score sufficienti prima della conferma;
14. conserva per alcuni frame un track valido quando lo scan e' parzialmente
    occluso;
15. pubblica in `gate_specs_` soltanto i candidati confermati.

Nel mixed il detector viene attivato quando la strada davanti ha pressione
LiDAR o block score sufficiente. Questo evita che spazi laterali innocui
distolgano continuamente il robot dalla centerline.

### Macchina a stati dei varchi

File: `controller/hardware_planner/parts/gap_workflow.inc`.

Il workflow e':

```text
startup scan
  -> candidato confermato
  -> locked gap goal
  -> allineamento/avvicinamento
  -> attraversamento fisico
  -> incremento passed_gates
  -> nuova scansione oppure completamento
```

`set_locked_gap_goal` salva posizione iniziale, asse di approccio e larghezza
del corridoio. `publish_locked_gap_goal` espone un solo gate stabile al planner.
`locked_gap_longitudinal_progress` e `locked_gap_lateral_offset` misurano il
robot nel frame del varco.

Il passaggio viene contato soltanto quando il frontale ha superato il piano del
gate entro il limite laterale, oppure quando una tolleranza compatta e
controllata conferma un attraversamento quasi completo. Tempo minimo e distanza
percorsa dal lock impediscono successi immediati.

## Mixed: strada strutturata e bypass

### Simulatore

File: `controller/simulation_planner/parts/mixed_arbitration.inc`.

`compute_mixed_gate_score` valuta necessita' e confidenza del bypass;
`compute_mixed_structured_score` valuta la possibilita' di restare sulla strada.
`update_mixed_arbitration` applica soglie, hold e hysteresis per scegliere tra:

- road mode: riferimento clothoid/centerline;
- gate mode: target LiDAR locale;
- rejoin: ritorno graduale alla strada dopo il gate.

Per Closed Obstacle Road la pipeline completa e':

```text
LiDAR -> gate track persistente -> mixed score/hysteresis -> gate lock
      -> traiettoria locale -> attraversamento -> rejoin sempre in avanti
      -> centerline -> MPC
```

La selezione rispetta il contratto del detector: score numerico minore significa
candidato migliore. Durante gate e rejoin la velocita' viene ridotta usando
clearance e curvatura. Il rejoin usa una cubica locale con endpoint davanti al
progresso corrente, non la proiezione globale piu' vicina. Hardware Lab, che ha
una strada aperta, usa un rejoin distinto verso `road_rejoin` e non applica il
wrap dei circuiti chiusi.

### Hardware

`compute_mixed_road_forward_clearance` e
`compute_mixed_road_block_score` sono in
`controller/hardware_planner/parts/lifecycle.inc`.
Quando il fronte e' bloccato, `gap_detection.inc` cerca un bypass e
`gap_workflow.inc` lo blocca fino all'attraversamento. Terminato il gate,
`trajectory.inc` ricostruisce il riferimento sulla strada.

Se ricompare wandering su Closed Obstacle Road, l'ordine diagnostico e':

1. `dynamic_gate_track_id`, confidenza e target switch;
2. `mixed_gate_score` contro `mixed_structured_score`;
3. stato road/gate/rejoin e durata del rejoin;
4. progresso structured prima/dopo il rejoin;
5. target curvature, heading error MPC e clearance minima.

Non va corretto nel mapping PWM finche' il target selezionato cambia posizione
o modalità: il PWM e' uno strato successivo.

## Planner bio-inspired e primitive

L'implementazione esterna e' in `third_party/progettotesi/src/lib/`; l'entry
point e' `sel_jr`, dichiarato in `action_selection.h`.

I due adapter sono:

- `HardwarePlannerRunner::plan_if_needed` in
  `controller/hardware_planner/parts/planning.inc`;
- `PlannerDrivenVehicleSim::plan_if_needed` in
  `controller/simulation_planner/parts/planning_sensors.inc`.

Prima della chiamata, `active_gate_indices` esclude gate passati, finalizzati o
fuori finestra. Gli elementi attivi vengono copiati in un vettore compatto,
perche' l'indice restituito da `sel_jr` e' locale a quel vettore.

`sel_jr` viene chiamato in due modalità:

```text
road_beh=true   -> road_info/clothoid, nessun vettore gate
gate_beh=true   -> vettore dei soli gate attivi
```

Il ritorno e' interpretato come:

```text
commands[0] = j  jerk longitudinale
commands[1] = r  variazione/accelerazione del comando angolare
commands[2] = indice locale del gate scelto, quando presente
```

`j` e `r` vengono verificati, limitati e memorizzati. L'indice locale viene
rimappato su `gates_`, producendo `chosen_gate_index_`. Questa variabile e' il
collegamento esplicito tra decisione del planner, traiettoria, GUI e telemetria.

## Dalle primitive alla traiettoria

File:

- `controller/hardware_planner/parts/trajectory.inc`;
- `controller/simulation_planner/parts/state_reference.inc`;
- `controller/simulation_planner/parts/trajectory.inc`.

`update_planner_references` integra nel tempo le primitive correnti e costruisce
velocita', accelerazione e yaw-rate desiderati.

`update_selected_trajectory` sceglie la geometria:

- strada: campioni della clothoid/centerline davanti al robot;
- gate: segmento o curva locale verso `chosen_gate_index_`;
- mixed rejoin: campioni che riportano progressivamente alla centerline.

Il risultato e' una `std::vector<ReferenceWaypoint>` con posizione, yaw,
curvatura e speed reference. Questa e' l'interfaccia verso il follower: il
planner decide l'intenzione, il riferimento la rende geometricamente seguibile.

### Protezione anti short-track

File: `model/route/structured_path_model.cpp`.

`project_closed_path` proietta il robot sulla polyline chiusa usando `s_hint`.
La ricerca e' limitata in avanti e indietro e rifiuta un attraversamento dello
start quando il robot non e' fisicamente vicino allo start. In questo modo una
centerline che si avvicina o si incrocia non puo' far saltare il progresso a
fine giro.

`sample_closed_path_span` campiona un arco continuo anche quando `s` supera la
lunghezza e deve fare wrap.

## Follower MPC

File: `model/navigation/mpc_path_follower.cpp`.

`ModelPredictivePathFollower::solve`:

1. trova il waypoint anchor vicino alla posa, usando un hint locale;
2. sceglie un preview in avanti per evitare inseguimento miope;
3. calcola errore laterale e heading nel frame del riferimento;
4. combina preview curvature, curvatura della strada ed errori;
5. genera una famiglia discreta di comandi candidati;
6. simula ogni candidato per l'orizzonte configurato;
7. accumula costi di cross-track, heading, velocita' e sforzo;
8. restituisce il candidato a costo minimo.

Per la car i candidati sono accelerazione e steer-rate e vengono integrati con
il modello bicycle. Per il tank sono accelerazione e yaw-acceleration e vengono
integrati con un modello differenziale. L'uscita include target speed,
steer-angle o yaw-rate, costo ed errori diagnostici.

## Dinamica dei robot

File: `model/navigation/vehicle_dynamics.cpp`.

`VehicleDynamicsModel` e le implementazioni car/tank mantengono lo stato fisico
simulato. La car usa wheelbase, steering e curvatura; il tank usa velocita'
lineare e yaw-rate differenziale. `VehicleGeometry` contiene anche footprint,
limiti di accelerazione, velocita', sterzo e ruote.

Il footprint orientato pubblicato dal modello viene passato a
`WorldMap::collides`.

## Profili calibrated versionati

I profili sono:

- `config/robots/car_calibrated.json`;
- `config/robots/tank_calibrated.json`.

Il loader e' `model/navigation/robot_calibration_profile.cpp`. Valida schema,
modello e valori positivi, calcola un hash FNV-1a del contenuto e applica in un
solo punto geometria, encoder, attuazione, ritardo, rumori IMU/LiDAR e limiti.
Il simulatore carica automaticamente il profilo del robot in Sim-Calibrated;
il runner hardware permette override con `--calibration-profile PATH`.

Report e live scene conservano `profile_name`, `profile_version`, path e hash.
Il JSON simulato salva anche `configuration_hash`, seed e provenance del
planner. I parametri di attuazione attuali sono baseline iniziali: il campo
`measurement_status` dichiara esplicitamente che PWM, costante motore e raggio
efficace tank richiedono ancora identificazione con dati reali.

## Da target cinematici a PWM

Il punto principale e' `HardwarePlannerRunner::compute_control_command` in
`controller/hardware_planner/parts/control.inc`.

La funzione lavora a strati:

1. determina target speed, curvature e yaw-rate dalla traiettoria/MPC;
2. applica regole di scan iniziale, recovery, pivot e gate lock;
3. valuta stop LiDAR, safety flags e distanza frontale;
4. limita i target alla geometria del veicolo;
5. converte velocita' corpo e yaw-rate in velocita' ruota;
6. calcola feed-forward PWM;
7. aggiunge feedback da velocita' ruote/encoder quando disponibile;
8. applica correzione lineare e angolare;
9. gestisce dead-zone, boost di partenza e autorita' della ruota esterna;
10. applica clamp e slew-rate finale.

Le trasformazioni pure sono in `model/navigation/actuation_model.cpp`.

### Cinematica differenziale

Con `b` uguale a meta' track:

```text
v_left  = v - omega * b
v_right = v + omega * b
```

La funzione e' `wheel_speeds_from_body`.

### Mapping velocita' -> PWM

`wheel_speed_to_pwm` applica scala canale, dead-zone, bias e gain, poi limita a
`max_pwm`. Gli helper successivi hanno responsabilita' separate:

- `clamp_motion_pwm_band`: banda minima/massima;
- `apply_start_motion_boost`: supera l'attrito statico;
- `enforce_forward_tracked_turn_authority`: mantiene differenza fra ruota
  interna ed esterna sul tank;
- `slew_limit_pwm`: limita la variazione fra cicli;
- `wheel_speed_from_pwm_estimate`: fallback diagnostico/odometrico.

Il comando finale passa da:

```text
HardwarePlannerRunner
  -> RealRobotBridge::send_pwm
  -> RSPSerialBridge::send_pwm
  -> frame SET_MOTOR_PWM
  -> firmware RSP-v1
```

Il firmware applica direzione, ramping e `analogWrite` nei file sotto
`low_level/`. I cambiamenti al mapping host non devono essere confusi con il
ramping implementato dal controller fisico.

### Controllo ruote chiuso sul microcontrollore

I firmware principali sono:

- car: `low_level/car/rspV1_arduino/rspV1_arduino.ino`;
- tank: `low_level/tank/rspV1_tank/rspV1_tank.ino`;
- copia tank mantenuta identica: `low_level/tank/tank_firmware/tank_firmware.ino`.

Con l'opzione host `--mcu-wheel-pid`, il payload `MOTOR_CMD` usa
`CONTROL_MODE_WHEEL_VELOCITY = 0x02` e i due `int16` sono target firmati in
mm/s, non PWM. Il firmware esegue ogni 20 ms:

```text
target velocita' ruota
  -> feed-forward
  + Kp * errore encoder
  + Ki * integrale anti-windup
  -> target PWM
  -> ramping, safety e analogWrite
```

I parametri RSP `0x0B..0x0F` configurano `Kp`, `Ki`, ticks/giro, raggio in
micrometri e feed-forward. Al connect il runner invia ticks/giro e raggio del
profilo; inoltre rifiuta firmware troppo vecchi. Stop, timeout e cambio modalita'
azzerano target, integrali e snapshot encoder. `MOTOR_FLAG` bit 5 segnala che
l'anello chiuso e' attivo.

Il mapping PWM host resta disponibile come modalita' legacy e come riferimento
feed-forward/calibrazione. Prima del flash occorre compilare con Arduino CLI o
IDE per la scheda reale e verificare segno encoder, ticks/giro e direzione di
entrambe le ruote su cavalletto.

## Safety e gestione degli errori

La sicurezza e' distribuita su più livelli:

- validazione dei frame e heartbeat in `hardware_io`;
- plausibilita' encoder e spike IMU in `localization.inc`;
- self-hit, min range e front sector nel LiDAR;
- stop/recovery in `control.inc`;
- clamp PWM e slew nell'Actuation Model;
- safety flags e watchdog nel firmware.

Se il goal e' raggiunto, il runner azzera il comando. Durante disconnect o
eccezioni il composition root hardware invia stop quando la connessione lo
consente.

## Contratti di completamento

### Structured

Il completamento non e' una semplice distanza dallo start. In
`controller/simulation_planner/parts/runtime.inc` e
`controller/hardware_planner/parts/sensing_runtime.inc` vengono combinati:

- progresso cumulativo sulla strada;
- lunghezza target del giro;
- ritorno fisico nella finestra dello start;
- distanza dalla posa finale;
- cross-track e heading, soprattutto per il tank.

Questo impedisce lo short-track: essere vicini allo start prima del giro non
equivale a completare la missione.

### Unstructured

Un gate locked deve essere superato longitudinalmente e restare entro il
corridoio laterale. Dopo ogni passaggio il sistema riparte dalla fase di scan.
Nel simulatore il conteggio degli intermedi non sostituisce la distanza dal
goal. Sul runner hardware, dopo almeno un varco reale, il goal globale viene
ammesso come target successivo soltanto se scan corrente, occupancy e mappa di
percezione confermano libero l'intero segmento: la missione termina quindi
nella regione fisica del goal, non su un secondo settore libero arbitrario.

### Mixed

Il completamento richiede insieme:

- progresso sufficiente sulla strada;
- nessun bypass ancora attivo;
- sequenza di gate richiesta completata;
- ritorno/start o goal fisico coerente con la topologia;
- velocita' sufficientemente bassa quando applicabile.

## Telemetria e report

### Simulazione

- telemetria: `controller/simulation_planner/telemetry_controller.cpp`;
- JSON: `controller/application/simulation_report_controller.cpp`;
- PNG: `model/reporting/slam_reference_exporter.cpp`.

`write_json_report` salva configurazione, scenario, risultato, performance e
telemetria campionata. Dopo aver chiuso correttamente il JSON costruisce il nome
`<stem>_slam_reference.png` e chiama l'esportatore. Il salvataggio e' riuscito
soltanto se entrambi gli artefatti sono stati scritti.

Il PNG contiene free-space rays, occupancy, ostacoli/strada di riferimento,
trail stimato, start, goal e posizione finale.

### Robustezza statistica

`scripts/run_statistical_robustness_matrix.sh` esegue di default 20 seed per
ogni combinazione primaria car/tank e ideal/calibrated. Le variabili
`SEEDS`, `FIRST_SEED`, `MAX_STEPS`, `TIMEOUT_SECONDS`, `MIN_SUCCESS_RATE` e
`OUTPUT_DIR` permettono di estendere la campagna a 50 seed senza cambiare il
codice.

`runs.tsv` conserva combinazione, seed, status, return code e report; ogni
report contiene hash configurazione/profilo, collision cause, minimum clearance,
gate switch, tempo road/gate/rejoin, path length, errore laterale, variazione
steering e timing planner. `aggregate.tsv` calcola success rate per famiglia e
fallisce sotto la soglia, 0.95 di default. Gli artefatti JSON e PNG restano la
sorgente per analisi piu' severe, ad esempio 99%, zero collisioni e p95 sotto il
periodo del ciclo.

La regressione breve e riproducibile e'
`scripts/run_primary_mvc_regression_matrix.sh`: usa soltanto Structured
Validation/Circle, tutte le Unstructured deterministiche e Mixed
Closed-Obstacle/Hardware, con limite 4000 step.

### Hardware

- storico/report: `controller/hardware_planner/telemetry_controller.cpp`;
- ricostruzione: `view/simulator_app/parts/state_support.inc`;
- JSON/CSV/PNG: `view/simulator_app/parts/reporting.inc`.

La ricostruzione hardware usa gli hit LiDAR e la posa stimata. La posa esterna,
quando presente, resta un canale distinto e permette di calcolare gli errori.

## GUI, editor e live stream

### GUI

Package: `view/simulator_app/`.

- `view/simulator_app/parts/application.inc`: `main`, headless e ciclo
  SDL/ImGui;
- `configuration.inc`: applicazione preset e livelli simulativi;
- `map_editor.inc`: selezione e trascinamento degli elementi;
- `telemetry_views.inc`: grafici;
- `mission_panels.inc`: workflow simulazione/hardware;
- `control_panel.inc`: controlli principali;
- `reporting.inc`: esportazione hardware.

La GUI non implementa il planner: legge Controller e ViewModel e invia soltanto
azioni esplicite come reset, selezione mondo o salvataggio.

### Live stream hardware

Package: `view/live_stream/`.

- `serialization.inc`: codec binario di scena e frame;
- `socket_transport.inc`: socket e packet envelope;
- `structured_remap.inc`: normalizzazione della sola visualizzazione;
- `snapshots.inc`: conversione del runner in snapshot;
- `client.inc` e `server.inc`: comunicazione bidirezionale.

`LiveSceneSnapshot` contiene geometria relativamente statica. `LiveFrameSnapshot`
contiene robot, gate, LiDAR, comando e telemetria del ciclo. La remap structured
e' solo grafica: non cambia coordinate del planner o del robot reale.

## Entry point e configurazione

### Simulatore

`view/simulator_app/parts/application.inc` contiene `main`, `run_headless` e
`run_gui`. Il parsing condiviso e' in
`controller/application/app_options.cpp`.

### Hardware

`controller/hardware_app/parts/entrypoint.inc` crea:

1. `RealRobotBridge::Options`;
2. `HardwarePlannerConfig`;
3. `WorldMap` selezionata e validata;
4. `HardwarePlannerRunner`;
5. ricevitore posa esterna e live stream, se abilitati.

`controller/hardware_app/parts/options.inc` contiene parsing e help;
`controller/hardware_app/parts/stream_config.inc` applica profilo robot,
scenario e streaming; `controller/hardware_app/parts/simulated_plant.inc`
implementa lo smoke sintetico senza porte fisiche.

## Dove intervenire per una modifica

| Obiettivo | Primo file da aprire |
| --- | --- |
| Cambiare geometria di una mappa | `model/world/parts/*_presets.inc` |
| Cambiare scala structured | `model/world/scenario_model.cpp` |
| Cambiare raycast/collisione | `model/world/world_geometry_model.cpp` |
| Cambiare rumore LiDAR simulato | `model/perception/perception_model.cpp` |
| Cambiare protocollo RPLidar | `controller/hardware_io/rplidar_a1.cpp` |
| Cambiare decodifica telemetria RSP | `controller/hardware_io/rsp_protocol.cpp` e `rsp_serial_bridge.cpp` |
| Cambiare estrinseci LiDAR | `HardwarePlannerConfig::localization` e `localization.inc` |
| Cambiare filtro encoder/IMU | `controller/hardware_planner/parts/localization.inc` |
| Cambiare EKF | `model/navigation/state_estimator_ekf.cpp` |
| Cambiare estrazione dei varchi | `controller/hardware_planner/parts/gap_detection.inc` |
| Cambiare quando un gate e' passato | `controller/hardware_planner/parts/gap_workflow.inc` o `controller/simulation_planner/parts/dynamic_gates.inc` |
| Cambiare arbitraggio mixed | `controller/simulation_planner/parts/mixed_arbitration.inc` |
| Cambiare input al planner | i due `plan_if_needed` |
| Cambiare traiettoria locale | `controller/hardware_planner/parts/trajectory.inc` e `controller/simulation_planner/parts/trajectory.inc` |
| Cambiare MPC | `model/navigation/mpc_path_follower.cpp` |
| Cambiare PWM | `controller/hardware_planner/parts/control.inc` e `model/navigation/actuation_model.cpp` |
| Cambiare criterio goal | i due file `*runtime.inc` |
| Cambiare JSON simulato | `controller/application/simulation_report_controller.cpp` |
| Cambiare PNG SLAM | `model/reporting/slam_reference_exporter.cpp` |
| Cambiare GUI | `view/simulator_app/parts/` |
| Cambiare stream | `view/live_stream/parts/` |

## Regole per estendere il codice senza rompere i confini

1. Una trasformazione matematica pura va nel Model.
2. Una decisione sull'ordine del ciclo va nel Controller.
3. Disegno, pannelli e serializzazione della vista vanno nella View.
4. Il planner non deve leggere ImGui, socket o porte seriali.
5. Il firmware non deve conoscere `WorldMap` o gate.
6. La posa esterna deve restare ground truth, salvo una scelta sperimentale
   esplicita e documentata.
7. Un gate finale non deve essere contato come intermedio.
8. Un report JSON deve continuare a produrre il PNG SLAM gemello.
9. Le modifiche hardware richiedono almeno gli smoke sintetici prima di una
   prova fisica.

## Verifica minima dopo una modifica

```sh
cmake -S . -B build -DBUILD_TESTING=ON
cmake --build build -j2
ctest --test-dir build --output-on-failure
./scripts/run_primary_mvc_regression_matrix.sh
```

Per i package hardware eseguire inoltre gli smoke sintetici Hardware Lab con
`thesis_robot_runner --simulate` per car e tank, sia unstructured sia mixed.

Baseline al 2026-07-26:

```text
CTest                                  4/4
Structured Validation + Circle         8/8
Unstructured deterministiche          24/24
Mixed Closed Obstacle + Hardware        8/8
Matrice primaria                       40/40 goal reached
JSON + SLAM PNG                        40/40
Hardware Lab runner sintetico          4/4 goal reached
Closed Obstacle multi-seed             20/20 (5 seed per combinazione)
```

La campagna statistica completa resta configurata a 20 seed per combinazione e
va eseguita separatamente prima della raccolta dati definitiva. I firmware PID
sono stati controllati strutturalmente ma non compilati in questa workstation:
`arduino-cli` non e' installato e il core/board reali devono essere selezionati
prima del flash.
