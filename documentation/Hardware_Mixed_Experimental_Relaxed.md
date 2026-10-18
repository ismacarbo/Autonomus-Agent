# Mixed hardware — EXPERIMENTAL_RELAXED, 18 ottobre 2026

**Aggiornamento dopo la prova delle 20:35:** [rimozione del veto dell'inviluppo prudenziale e del corridoio virtuale](Hardware_Mixed_Footprint_20261018_203702.md). La configurazione sperimentale usa ora `clearance_policy physical_footprint`; i margini descritti sotto appartengono alla prima iterazione.

**Aggiornamento dopo la prova delle 20:10:** [correzione dello stop e degli impulsi di partenza](Hardware_Mixed_Stop_20261018_201108.md). Il seguito documenta la prima iterazione; la correzione aggiunge primitive di lunghezza adattiva, riacquisizione immediata e regolazione della marcia lenta.

Implementazione esclusiva del percorso hardware. La configurazione storica resta **STRICT**; la nuova configurazione esplicita è `config/mixed_experimental_relaxed.conf`. Il log identifica la modalità come `MIXED_EXPERIMENTAL_RELAXED`. Non sono stati aperti dispositivi né avviati motori. Simulation planner, preset simulati, GUI e firmware non sono stati modificati.

## Risultato e limiti del confronto

La ricerca ora produce segmenti ammissibili con lo stesso limite operativo **1,80 m⁻¹**, velocità **0,04 m/s**, corpo **0,25 × 0,15 m**, passo **0,18 m**, sterzo massimo **0,52 rad** e gli stessi margini del confronto. Il miglioramento iniziale non dipende dall'aumento dello sterzo o dalla riduzione della sagoma.

| Test offline | STRICT | EXPERIMENTAL_RELAXED |
|---|---:|---:|
| Cinque gate rigenerati dalle ultime cinque scansioni dello snapshot 10:18:39 | 0 accettati / 315 valutazioni | 259 accettati / 1.015 valutazioni |
| Gate selezionabili su questi dati | 0 | 5, tramite punti intermedi |
| Attraversamenti completi previsti in questi segmenti registrati | 0 accettati | 0: orizzonte locale di 0,75 m |
| Gate sintetico con strada troppo corta per il ritorno G2 | 0 / 63 | 42 / 161; selezionato attraversamento senza rejoin preventivo |
| Rientro sintetico progressivo, n iniziale 0,30 m | — | Quattro segmenti da 0,45 m: n ≈ 0,245; 0,178; 0,116; 0,041 m |

Questi sono **conteggi di valutazioni**, con famiglie e prefissi condivisi, non altrettante manovre fisiche indipendenti. Il test del gate completo usa `local_horizon_m=1.50`; il test registrato usa il default sperimentale di 0,75 m. Le prove di rientro concatenano matematicamente gli endpoint, senza simulare una dinamica o un controller fisico.

Il confronto storico rimane 0 accettati su 16.254 valutazioni in 258 ricerche. Non è possibile riprodurre esattamente tutte quelle ricerche: il log non conserva per ciascuna posa, raggi e geometria completa dei gate. Il nuovo confronto usa **gli stessi input per entrambe le politiche**: 1.286 endpoint ricostruiti, cinque scansioni fra 54,204593 e 54,799871 s, posa finale registrata e gate riestratti una volta. La visibilità resta `obstacles_only` in entrambi i casi. Gli endpoint esportati non contengono qualità raw e durata originale di ciascuna scansione; il confronto geometrico imposta l'inflazione di moto a zero, coerentemente con robot fermo ma senza certificare ogni incertezza del runtime. L'hardware continua invece a calcolarla dai dati live.

Fonti e SHA-256 sono nel [manifest](analysis_20261018/experimental_relaxed/fixture_manifest.json). Il fixture versionato si rigenera con `python3 tools/extract_mixed_relaxed_fixture.py` dai dati originali in `reports/`. I cinque gate rigenerati non sono presentati come gli stessi ID del log storico.

Figure locali: [segmenti e LiDAR](analysis_20261018/experimental_relaxed/recorded_paths.png), [curvature](analysis_20261018/experimental_relaxed/curvature_comparison.png). Artefatti numerici e log sono nella stessa directory, non sono input del runtime hardware.

## Filtri e politiche

| Condizione | STRICT | EXPERIMENTAL_RELAXED |
|---|---|---|
| Ritorno completo verificato prima di scegliere un gate | Obbligatorio | Preferenza; assenza annotata come `no_rejoin_after_gate` |
| Attraversamento completo in una sola reference | Obbligatorio | Possibili avvicinamenti intermedi arrestabili |
| Orientamento del passaggio | `cos(delta)>0.7` | Verso di attraversamento e sagoma effettiva nel passaggio; eliminata soglia angolare arbitraria |
| Offset e orientamento dell'uscita | Famiglia di 63 G2 e corridoio finale | G2 più primitive libere; la larghezza del gate si verifica mentre la sagoma interseca il piano, non lungo un corridoio virtuale dopo l'uscita |
| Linea retta LiDAR→target, cella target libera, supporto mappa | Prefiltri | Non bloccanti: decide la traiettoria curva verificata, anche se termina prima del target lontano |
| Distanza target rilevato | Finestra prudenziale precedente | Range realmente disponibile al sensore; nessun gate inventato senza estremi osservati |
| Score e conferme del tracking gate | Soglia e conferme precedenti | Una osservazione corrente sufficiente a proporre il gate; score usato per ordinamento |
| Gate già passato / minimo avanzamento Frenet del suo anchor | Veto | Penalità per gate già passato; nessun minimo arbitrario di s per la proposta |
| Cooldown di acquisizione/riprova | 0,60 s | Nuova scansione e veicolo assestato richiesti; niente cooldown addizionale |
| Timeout gate | Stop precedente | Arresto della reference e nuova acquisizione; non impegno illimitato nello stesso gate |
| Rejoin n=0, delta_yaw=0 in una G2 | Target rigido | Costo terminale su n e delta_yaw, con più segmenti possibili |
| Lunghezza eccessiva G2 | Filtro detour precedente | Nessun veto euristico del rapporto lunghezza/distanza; limite assoluto di risorse e reference troncata all'orizzonte locale |

Rimangono la strada persistente e il goal originale `(1.30,0)`. `ReferenceRoad::project` conserva la finestra locale di Frenet; `targets` mantiene i target sulla strada, e in modalità rilassata si considera anche l'ultimo punto disponibile. Il punto finale di una primitiva libera non deve coincidere con la strada. Vicino agli estremi della strada, un residuo tangenziale non è più automaticamente classificato come guasto di localizzazione; i controlli indipendenti di salto posa rimangono. Il rejoin si dichiara concluso solo sulla **posa realmente stimata**, vicina alla curva, con n/yaw entro tolleranza, strada successiva verificata e tre conferme. In RELAXED non si impone ulteriore avanzamento in s alla conclusione, che al goal non sarebbe disponibile. La sola proiezione sul goal di una posa oltre il goal non completa il rientro.

## Primitive, arresto e controllo

Si riutilizza la libreria di clotoidi già presente. La nuova ricerca sceglie una curvatura raggiungibile, la raccorda alla curvatura iniziale con una rampa limitata dallo slew dello sterzo e dall'accelerazione angolare, quindi prosegue con un arco. Non impone posizione, yaw e curvatura finale zero simultaneamente. Le primitive sono integrate esattamente dalla libreria e concatenate senza discontinuità di posizione, tangente o curvatura.

La ricerca locale ha tre stadi, sette scelte di curvatura per stadio, al massimo dodici nodi diversi mantenuti: **massimo 140 estensioni**, oltre alle 63 connessioni G2. I target laterali e di uscita delle primitive risultano dall'integrazione del veicolo, non dalla replica della stessa griglia di endpoint. Collision checking, geometria e ricerca di primitive sono condivisi da avvicinamento e rejoin. La salience precedente rimane attiva in STRICT; in RELAXED si evita quella chiamata aggiuntiva, usando il costo locale.

Ogni candidato eseguibile deve superare:

- Curvatura operativa e meccanica, yaw rate, slew dello sterzo e accelerazione angolare.
- Sagoma completa contro gli endpoint occupati freschi, con margini ostacolo/posa, discretizzazione e incertezza di moto; bounds di missione.
- Controllo separato di una continuazione di arresto all'endpoint, lungo la curvatura finale: `d_stop = v * reaction_time + v² / (2 * braking_decel)`, usando il massimo fra velocità richiesta e misurata.

Con 0,04 m/s, 0,40 s e 0,20 m/s² la distanza nominale è **0,020 m**, oltre alla sagoma inflazionata. È il modello configurato di arresto, da verificare sul mezzo; non una decelerazione misurata in questa sessione. La sagoma base verificata rimane 0,43 × 0,33 m prima dell'inflazione di moto. La politica `obstacles_only` controlla ostacoli misurati senza certificare come libero lo spazio mai osservato; `visibility_policy strict` rimane disponibile anche in RELAXED.

L'MPC riceve solo reference ammesse. Le reference locali hanno velocità finale zero e un profilo di rallentamento che comprende la latenza di reazione. Il runtime verifica nuovamente la parte rimanente su osservazioni aggiornate; prima di ogni comando controlla anche l'arco di arresto della velocità richiesta e di quella misurata. Alla fine del segmento servono una nuova osservazione e assestamento prima della reference successiva. Un ostacolo nuovo invalida la reference. Se mancano gate, la stessa ricerca può trovare un segmento locale verso una nuova osservazione; se non esiste, il robot resta fermo.

Emergency stop, watchdog/firmware, fault e STOP richiesti dal controller, limiti PWM, età di LiDAR/IMU/telemetria, matching iniziale, controlli di localizzazione/slip e controlli prima della trasmissione restano attivi. Nessuna curva diagnostica passa automaticamente all'MPC.

## Curvatura e analisi non eseguibile

La formulazione è dimensionalmente coerente: curvatura in m⁻¹, `yaw_rate=v*k`, `steer=atan(wheelbase*k)`, slew `v*wheelbase*dk/ds / (1+(wheelbase*k)²)`. I picchi delle G2 sono valutati anche agli estremi di ciascuna clotoide, quindi non si perdono picchi in raccordi più corti del passo di campionamento. Il problema osservato è soprattutto la scelta di condizioni terminali, non una correzione di unità.

- **Limite meccanico del modello:** `tan(0.52)/0.18 = 3.1809 m⁻¹`. Non equivale a una nuova calibrazione fisica dello sterzo.
- **Limite operativo:** `operational_curvature_limit`, inizialmente 1,80 m⁻¹. Se omesso eredita il profilo. Il valore applicato è sempre il minimo con il limite meccanico e viene usato anche dall'MPC e dal controllo hardware.
- **Vincolo del generatore:** eliminata l'obbligatorietà della singola G2 con tutte le condizioni finali; rampe e archi restano entro i limiti.
- **Euristiche:** detour, allineamento terminale e rejoin preventivo non autorizzano a superare alcun limite fisico.

`unconstrained_candidate_analysis` restituisce un tipo diagnostico separato da `PathResult`, privo di `valid()`. Il parser hardware rifiuta di selezionarlo come politica. Il test eseguibile esporta le curve senza aprire trasporti hardware:

```sh
build-hardware-mixed/simulator/thesis_hardware_mixed_relaxed_test \
  --unconstrained-candidate-analysis /tmp/mixed_geometry_only.csv
```

Nel caso matematico stretto il solver propone un picco di 12,8696 m⁻¹: la curva viene esportata come **non eseguibile** e `connect` la rifiuta. Gli artefatti del confronto includono anche le G2 geometriche verso i gate ricostruiti, i limiti e la loro differenza, separati dalle reference localmente validate. Il campo diagnostico `rejection=accepted`, se presente, riguarda solo la cinematica della connessione: non verifica ostacoli né autorizza movimento; la curva diagnostica rimane marcata `executable=0`. Una validazione sullo snapshot non sostituisce la rivalidazione live.

## Verifiche e riproduzione senza hardware

Build locale Release su Linux x86_64/GCC 15.2, GUI disabilitata. Non è una compilazione effettuata sul Pi. **Esito: 5/5 test CTest passati in 1,64 s**, oltre a `--mixed-options` e alle verifiche CLI di entrambe le configurazioni senza connessione hardware. [Log test](analysis_20261018/experimental_relaxed/ctest.log), [build finale](analysis_20261018/experimental_relaxed/build-final.log), [confronto numerico](analysis_20261018/experimental_relaxed/offline-comparison.log). Eseguire soltanto questa selezione offline, non l'intera suite che include scenari simulati:

```sh
cmake -S . -B build-hardware-mixed \
  -DTHESIS_BUILD_SIM_GUI=OFF -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release
cmake --build build-hardware-mixed -j 2 --target \
  thesis_robot_runner thesis_lidar_start_matcher \
  thesis_hardware_mixed_navigation_test thesis_hardware_mixed_relaxed_test \
  thesis_start_matching_runtime_smoke thesis_slam_observation_smoke \
  thesis_hardware_runtime_settings_smoke
ctest --test-dir build-hardware-mixed --output-on-failure \
  -R '^(thesis_hardware_mixed_navigation_offline|thesis_hardware_mixed_relaxed_offline|thesis_start_matching_runtime|thesis_slam_observation|thesis_trial_recording)$'
build-hardware-mixed/simulator/thesis_hardware_runtime_settings_smoke --mixed-options
build-hardware-mixed/simulator/thesis_hardware_mixed_relaxed_test \
  --artifacts documentation/analysis_20261018/experimental_relaxed/curves
python3 tools/plot_mixed_relaxed_results.py
```

Copertura nuova: confronto registrato A/B, attraversamento senza rejoin anticipato, segmento intermedio, esclusione per coda di arresto, collisione, scansioni scadute e spazio ignoto con visibilità strict, integrazione differenziale yaw/curvatura, slew ai raccordi, rientro multi segmento, mancata falsa conclusione oltre il goal, separazione della geometria diagnostica, parsing delle politiche, riferimento all'MPC, stop finale, rinnovo solo con nuova osservazione, limite sperimentale di velocità e interblocchi sensori/controller. La suite STRICT precedente resta in esecuzione.

Le cinque ricerche RELAXED del confronto hanno richiesto **65,13 ms complessivi** su questa macchina x86_64 (singolo campione di tempo, senza I/O seriale). Il costo è limitato per numero di nodi e gate, ma va misurato sul Pi 5: `planning_ms`, `step_ms` e le età sensore nei CSV sono le misure operative. Una ricerca locale finita non garantisce completezza, assenza di minimi locali o riuscita della missione; non sono state aggiunte retromarcia o pianificazione globale.

## Prossima prova fisica sul Raspberry Pi 5

Compilare sul Pi con i comandi precedenti. Il target iniziale proposto è **0,04 m/s**, con cap applicato dalla configurazione sperimentale anche se viene richiesto un cruise maggiore. I dati precedenti, tutti a velocità zero, non permettono di certificare che sia la minima velocità stabile del veicolo. Verificare velocità misurata e risposta alla richiesta di arresto durante una prova supervisionata; non abbassare ulteriormente il target se il mezzo entra nella zona di stallo/deadband.

Impostare porte stabili, reference dello start attuale e host SLAM:

```sh
MIXED_BIN="$PWD/build-hardware-mixed/simulator"
MIXED_REFERENCE="$PWD/datasets/localization/car_mixed_local_start.csv"
MIXED_CONTROLLER=/dev/ttyACM0
MIXED_LIDAR=/dev/ttyUSB0
MIXED_SLAM_HOST=127.0.0.1
mkdir -p reports/mixed-real

# Sola verifica configurazione: non connette l'hardware.
"$MIXED_BIN/thesis_robot_runner" \
  --scenario mixed --mixed-map goal_directed_dynamic \
  --mixed-config config/mixed_experimental_relaxed.conf \
  --vehicle-model car --calibration-profile config/robots/car_calibrated.json \
  --start-reference "$MIXED_REFERENCE" --cruise-speed 0.04 --check-mixed-config
```

Attendere `mixed_planning_policy=MIXED_EXPERIMENTAL_RELAXED`, `mixed_effective_speed=0.04`, curvatura operativa 1,8 e meccanica circa 3,1809. L'assenza della reference può lasciare valida la configurazione, ma impedisce la connessione/marcia: occorre acquisirla. Per una nuova posa iniziale, con trazione disabilitata e robot fermo, usare un nuovo nome di reference:

```sh
mkdir -p datasets/localization
# Apre soltanto il LiDAR. Il test evita di sovrascrivere una cattura esistente.
test ! -e "$MIXED_REFERENCE" && \
"$MIXED_BIN/thesis_lidar_start_matcher" \
  --lidar-port "$MIXED_LIDAR" --vehicle-model car \
  --lidar-x 0.075 --lidar-y 0.040 --lidar-yaw-deg 0 \
  --min-range 0.12 --max-range 2.50 \
  --capture-reference "$MIXED_REFERENCE" --scan-count 10
```

Avviare il bridge SLAM esistente in un altro terminale/macchina: `./tools/slam_toolbox_bridge/run.sh`. Impostare `MIXED_SLAM_HOST` di conseguenza. Il comando seguente è riservato all'operatore e **può muovere il robot appena il matching è confermato**. Tenere l'arresto fisico immediatamente accessibile e supervisionare tutta la prova. Non è stato eseguito in questa sessione.

```sh
MIXED_RUN="$(TZ=Europe/Rome date +%Y%m%d_%H%M%S)_mixed_relaxed"
printf '%s\n' "$MIXED_RUN" > reports/mixed-real/latest_run.txt
"$MIXED_BIN/thesis_robot_runner" \
  --controller-port "$MIXED_CONTROLLER" --lidar-port "$MIXED_LIDAR" \
  --scenario mixed --mixed-map goal_directed_dynamic \
  --mixed-config config/mixed_experimental_relaxed.conf \
  --vehicle-model car --calibration-profile config/robots/car_calibrated.json \
  --start-reference "$MIXED_REFERENCE" \
  --cruise-speed 0.04 --dt 0.10 --max-steps 600 \
  --slam-bridge-host "$MIXED_SLAM_HOST" --slam-bridge-port 9760 --slam-pose-feedback \
  --telemetry-csv "reports/mixed-real/${MIXED_RUN}.csv" \
  > "reports/mixed-real/${MIXED_RUN}.log" 2>&1
```

`Ctrl+C` nel terminale del runner richiede lo stop tramite il gestore esistente; anche `kill -TERM PID_DEL_RUNNER` usa quel percorso. Rimane necessario l'arresto fisico se processo o collegamento non rispondono. I 600 step non sono un limite temporale preciso. Non serve completare la missione per osservare il primo segmento utile.

Per un confronto STRICT, riposizionare sullo stesso start, conservare scena e velocità e cambiare soltanto `--mixed-config config/mixed_goal_directed_dynamic.conf`; usare un nuovo nome di run. Entrambe le configurazioni hanno `visibility_policy obstacles_only`, così il confronto misura la politica di pianificazione, non un cambio di copertura LiDAR.

## Raccolta e lettura dei log

```sh
tail -f "reports/mixed-real/${MIXED_RUN}.log"
rg 'mixed_planning_policy|mixed_search_cycle|mixed_gate_search|mixed_rejoin_search|mixed_observation_search|mixed_candidate|mixed_selected|mixed_transition' \
  "reports/mixed-real/${MIXED_RUN}.log"
python3 tools/analyze_hardware_mixed.py "reports/mixed-real/${MIXED_RUN}.csv" \
  --log "reports/mixed-real/${MIXED_RUN}.log" \
  --geometry-dir "reports/mixed-real/${MIXED_RUN}_geometry" \
  > "reports/mixed-real/${MIXED_RUN}_summary.json"
```

I log aggregano generati, cinematicamente ammessi, collision-free dopo i filtri precedenti, attraversamenti completi previsti, rejoin completi previsti e accettati RELAXED. Aggiungono picco di curvatura, target x/y/yaw, `strict_blocker`, motivo della scelta, stato e stop. `mixed_search_cycle` somma i gate di un ciclo; non sommarlo nuovamente alle righe per gate. `rejoin_*` nei conteggi gate è una ricerca annidata, da non aggiungere ai candidati gate. I campioni `mixed_reference` includono curvatura e velocità geometrica nominale; il profilo di rallentamento applicato dall'MPC e il moto effettivo vanno confrontati con i target e gli encoder nel CSV. Il tool di analisi ora aggiunge i totali per politica e tipo di ricerca, e continua a riconoscere i log storici.

## File interessati

- `simulator/include/mvc/controller/hardware_planner/{mixed_navigation.h,mixed_mission.h}`: politiche, parametri, diagnostica e API offline.
- `simulator/src/controller/hardware_planner/mixed_navigation.cpp`: primitive, ricerca condivisa, gate, rejoin e arrestabilità.
- `simulator/src/controller/hardware_planner/parts/{mixed_navigation.inc,gap_detection.inc,lifecycle.inc}`: macchina a stati, filtri percettivi della proposta, limiti effettivi e profili locali.
- `simulator/src/controller/hardware_app/parts/entrypoint.inc`: verifica configurazione senza connessione.
- `config/{mixed_goal_directed_dynamic.conf,mixed_experimental_relaxed.conf}`: STRICT esplicito e configurazione sperimentale.
- `simulator/src/tests/integration/hardware_mixed_relaxed_test.cpp`, relativo fixture, `simulator/CMakeLists.txt`: suite hardware offline dedicata.
- `tools/{extract_mixed_relaxed_fixture.py,plot_mixed_relaxed_results.py,analyze_hardware_mixed.py}`: riproducibilità e confronto dei log.
- Questo documento e relativa eccezione in `.gitignore`; artefatti locali in `documentation/analysis_20261018/experimental_relaxed/`.

`planning.inc`, `trajectory.inc` e `gap_workflow.inc` sono stati esaminati: il preset dinamico hardware li bypassa per selezione/reference e conteggio Mixed tramite il ramo dedicato in `sensing_runtime.inc`. Non serve duplicarvi la nuova politica. Il submodule `third_party/progettotesi` aveva modifiche preesistenti ed è stato lasciato intatto.
