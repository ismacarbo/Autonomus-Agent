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

```text
arena              1.20 x 1.20 m
span della strada  0.90 m
body robot         0.25 x 0.15 m
wheelbase          0.18 m
track              0.13 m
```

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

- match spaziale con il gate precedente;
- smoothing di posizione, heading, larghezza e score;
- hold temporale quando il candidato scompare per pochi frame;
- regole specifiche di mixed per non perdere il bypass attivo;
- distinzione tra gate intermedio e target finale.

Un candidato coincidente con il goal e' marcato `final`: resta selezionabile ma
non viene contato come gate attraversato con la tolleranza larga degli
intermedi.

`update_unstructured_gate_progress` considera il passaggio lungo l'asse della
rotta e la distanza laterale, non soltanto un raggio dal centro.

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

### Hardware

`compute_mixed_road_forward_clearance` e
`compute_mixed_road_block_score` sono in
`controller/hardware_planner/parts/lifecycle.inc`.
Quando il fronte e' bloccato, `gap_detection.inc` cerca un bypass e
`gap_workflow.inc` lo blocca fino all'attraversamento. Terminato il gate,
`trajectory.inc` ricostruisce il riferimento sulla strada.

Il wandering della car su Closed Obstacle Road va quindi cercato in tre punti,
in quest'ordine:

1. stabilita' del candidato in `gap_detection.inc`;
2. durata del lock/rejoin in `gap_workflow.inc`;
3. transizione strada-gate in `mixed_arbitration.inc` per simulazione o nei
   block score hardware.

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

Il goal e' raggiunto dopo il numero richiesto di varchi fisicamente attraversati.
Un gate locked deve essere superato longitudinalmente e restare entro il
corridoio laterale. Dopo ogni passaggio il sistema riparte dalla fase di scan.

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
Matrice primaria                       31 goal / 9 collisioni note
JSON + SLAM PNG                        40/40
Hardware Lab runner sintetico          4/4 goal reached
```

Le nove collisioni note non sono state introdotte dal refactor: sono i casi
gia' classificati di bordo finale, Tight Corridor e Closed Obstacle Road car.
