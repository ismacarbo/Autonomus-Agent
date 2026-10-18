# Mixed hardware: correzione dell'inviluppo dopo la prova 20:35

La modalità sperimentale rimuove il veto del margine prudenziale e del corridoio virtuale del gate. Sulla scena registrata dello stop passa da **0 candidati accettati su 182 valutazioni a 212 su 350**, usando gli stessi due gate, le stesse cinque scansioni e lo stesso limite di curvatura di 1,80 m⁻¹. Sono segmenti locali verificati da **0,25 e 0,50 m**, non attraversamenti completati o una dimostrazione di riuscita della missione.

## Causa dello stop

Fonti: CSV/log del runner `20261018_203524_mixed_relaxed_fix` e snapshot `thesis_hardware_mixed_mixed_hardware_road_gate_gui_snapshot_20261018_203702_806`. Il runner contiene 283 campioni fino a circa 40,71 s; lo snapshot contiene 175 campioni fino a 25,164644 s. Il confronto geometrico usa l'ultima scena dello snapshot, mentre la cronologia usa il CSV completo.

- La ricerca reale funzionava: 461 valutazioni accettate, 12 attraversamenti previsti fra i candidati, due ripianificazioni. Il robot non ha però completato alcun gate.
- Il primo stop transitorio è `encoder_slip_guard` a 2,848 s. La velocità encoder massima è circa 0,214 m/s contro target 0,04 m/s; è quantizzata e non dimostra una velocità al suolo costante di quel valore. La regolazione PWM introdotta nella correzione precedente è conservata.
- A 7,238 s compare `braking_sweep_occupied_footprint`, poi ripartenza a 7,528 s. A 8,844 s ricompare lo stop di frenata; dal campione successivo prevale `current_occupied_footprint`, per 220 campioni. Il veto sulla posa attuale impedisce anche una nuova ricerca.
- Nelle ultime cinque scansioni (1.216 endpoint, memoria 0,60 s), **nessun endpoint occupa il corpo da 25 × 15 cm**. Il più vicino dista **10,75 cm dalla sagoma orientata**, ma 14 punti occupano il vecchio rettangolo di controllo da almeno **43 × 33 cm**.

Il rettangolo aggiungeva 3,5 cm di margine ostacolo, 2,5 cm di margine posa e 3 cm di discretizzazione **su ciascun lato**, anche da fermo. Agli angoli, 9 cm di padding rettangolare arrivano a circa 12,7 cm di distanza euclidea dal corpo. In moto si aggiungeva inoltre il massimo dell'incertezza di tutte le scansioni, calcolato con il range massimo di 2,5 m anche per punti vicini.

Questo dimostra il veto prudenziale nella scena di arresto persistente; non dimostra che ogni precedente frenata durante il moto fosse superflua. Il controllo della frenata rimane attivo.

![Sagoma reale, vecchio inviluppo e segmenti verificati](analysis_20261018/mixed_203702/footprint_and_paths.png)

## Modifiche

La configurazione `config/mixed_experimental_relaxed.conf` abilita esplicitamente:

```text
planning_policy EXPERIMENTAL_RELAXED
clearance_policy physical_footprint
collision_buffer_m 0.01
```

1. **Margini aggiuntivi non bloccanti:** per il footprint, `obstacle_margin_m` e `pose_margin_m` diventano una penalità del costo. Possono essere impostati anche a zero in questa politica. Non si gonfia più il corpo di 9 cm per lato come condizione necessaria a muoversi.
2. **Sagoma percorsa:** si misura la distanza euclidea degli endpoint dal rettangolo orientato reale. Rimangono un buffer di misura configurato a 1 cm, l'incertezza di moto di ciascun ritorno e una copertura numerica degli intervalli tra pose, inclusa la rotazione degli angoli del corpo. Da fermo non si aggiunge il padding della discretizzazione di un percorso inesistente. Il buffer è un parametro sperimentale, non una calibrazione metrologica effettuata in questa sessione.
3. **Incertezza per ritorno:** la parte traslazionale dipende dal moto durante scansione/trasporto; la parte rotazionale usa la distanza del singolo ritorno, includendo il braccio del LiDAR. L'incertezza di una scansione lontana non si applica indistintamente a tutti gli altri punti. I fixture storici senza componenti separate conservano il loro valore aggregato.
4. **Niente veto del corridoio virtuale:** il gate propone una destinazione; l'intero piano geometrico che lo contiene non diventa una parete. Una traiettoria libera può avanzare fuori dal corridoio dell'apertura per osservare e ripianificare. Un attraversamento viene contato soltanto se la sagoma passa davvero attraverso l'apertura e si libera dal piano. Il test evita anche falsi conteggi.
5. **Larghezza coerente:** l'estrattore usa corpo più buffer, senza aggiungere i vecchi margini di comfort. Restano gli estremi LiDAR realmente misurati. Le preferenze opzionali di rejoin anticipato sono limitate a otto verifiche per ricerca gate in questa politica; esaurire il budget non elimina il candidato.
6. **Diagnostica:** startup/check-config riportano `mixed_clearance_policy=physical_footprint`; gli stop di footprint e frenata riportano punto ostacolo, distanza dal corpo e margine richiesto. `virtual_gate_corridor_veto=0` conferma la rimozione del veto.

STRICT e la politica `conservative_envelope` mantengono il controllo precedente. Se `planning_policy` è STRICT, il nuovo campo non indebolisce la sagoma conservativa. Visibilità `obstacles_only`, controllo dei dati freschi, goal e strada persistente restano quelli della prova. `obstacles_only` controlla gli ostacoli misurati: non certifica lo spazio senza osservazioni come libero.

Restano collisioni del corpo e della traiettoria, coda di arresto alla fine del segmento, frenata verificata a ogni comando usando anche il moto misurato, velocità sperimentale 0,04 m/s, curvatura/sterzo e limiti degli attuatori, interblocchi sensori e localizzazione, arresto di emergenza, watchdog, guardia encoder e timeout di stallo. I parametri di frenata non sono stati ricalibrati. Nessuna modifica a simulation planner, preset simulati, GUI o firmware; il submodule già modificato dall'utente è rimasto intatto.

## Verifica riproducibile

| Stessa scena registrata, stessi due gate | Vecchio inviluppo | Nuova politica |
|---|---:|---:|
| Candidati valutati | 182 | 350 |
| Cinematicamente ammessi | 65 | 231 |
| Accettati dopo collisioni e arresto | 0 | 212 |
| Gate con segmento locale selezionabile | 0 | 2 |
| Attraversamenti completi / rejoin completi nei segmenti | 0 / 0 | 0 / 0 |

I segmenti selezionati hanno picchi di curvatura 1,80 e 1,20 m⁻¹. Le due ricerche richiedono circa **57 ms complessivi sul PC x86_64** in una misurazione; non è una misura sul Pi. L'aumento delle valutazioni deriva dalla possibilità di espandere prefissi che prima venivano eliminati subito. I conteggi non rappresentano altrettante manovre indipendenti e non vanno sommati ai 461 accettati della prova fisica precedente.

Il test attraversa anche il runtime hardware con osservazioni iniettate: la nuova reference arriva al controllore con target positivo e velocità terminale zero. Non apre dispositivi. Le curve geometriche non ammissibili rimangono confinate all'analisi offline.

Build Release hardware riuscita sul PC, inclusi runner, acquisitore LiDAR e harness. Cinque CTest passati, più `--mixed-options` e check-config STRICT/RELAXED. Le regressioni includono corpo occupato, buffer minimo, collisione fra campioni e angoli in rotazione, incertezza per punto, sensori scaduti, collisione nella coda di frenata, gate virtuale senza falso attraversamento, emergency stop e comunicazione persa. Restano validi i casi precedenti: 359 accettati sulla scena mattutina e 18 sulla scena delle 20:11 con politica conservativa e ricerca adattiva.

Rigenerazione del nuovo fixture dai file originali in `reports/`:

```sh
python3 tools/extract_mixed_stop_fixture.py \
  --snapshot thesis_hardware_mixed_mixed_hardware_road_gate_gui_snapshot_20261018_203702_806 \
  --fixture mixed_20261018_203702_footprint_scans.txt \
  --analysis-dir documentation/analysis_20261018/mixed_203702
build-hardware-mixed/simulator/thesis_hardware_mixed_relaxed_test \
  --artifacts documentation/analysis_20261018/mixed_203702/paths
```

Il fixture conserva endpoint esportati, posa e incertezza aggregata, non i pacchetti raw originali. I gate sono riestratti e condivisi fra i confronti; non si afferma di riprodurre tutti i cicli della missione. Fonti/hash: [manifest](analysis_20261018/mixed_203702/fixture_manifest.json). Evidenze: [geometria](analysis_20261018/mixed_203702/footprint_evidence.json), [regressione](analysis_20261018/mixed_203702/regression.log), [CTest](analysis_20261018/mixed_203702/ctest.log), [riepilogo runner](analysis_20261018/mixed_203702/runner_summary.json).

## Aggiornare il Pi

La build già presente sul Pi precede questa correzione: occorre trasferire i sorgenti aggiornati e ricompilare. Le modifiche locali non vengono trasferite da un semplice `git pull` finché non sono pubblicate. La patch incrementale è basata su `c73f599` e non include il submodule modificato.

Dal PC:

```sh
cd /home/isma/Desktop/Autonomus-Agent
scp reports/mixed_footprint_fix_20261018_203702.patch ismadev@100.87.133.15:/tmp/
```

Sul Pi, con runner fermo, applicare una sola volta:

```sh
cd ~/Desktop/Autonomus-Agent
git apply --check /tmp/mixed_footprint_fix_20261018_203702.patch && \
git apply /tmp/mixed_footprint_fix_20261018_203702.patch

cmake -S . -B build-hardware-mixed \
  -DTHESIS_BUILD_SIM_GUI=OFF -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release
cmake --build build-hardware-mixed -j2 --target \
  thesis_robot_runner thesis_lidar_start_matcher \
  thesis_hardware_mixed_relaxed_test thesis_hardware_mixed_navigation_test \
  thesis_start_matching_runtime_smoke thesis_slam_observation_smoke \
  thesis_hardware_runtime_settings_smoke
ctest --test-dir build-hardware-mixed --output-on-failure \
  -R '^(thesis_hardware_mixed_navigation_offline|thesis_hardware_mixed_relaxed_offline|thesis_start_matching_runtime|thesis_slam_observation|thesis_trial_recording)$'
build-hardware-mixed/simulator/thesis_hardware_runtime_settings_smoke --mixed-options
```

Se la patch risulta già applicata o la revisione differisce, controllare il diff senza forzare un reset. La vecchia patch `mixed_stop_fix_20261018_201108.patch` non contiene questa modifica.

Verifica senza connessione hardware:

```sh
build-hardware-mixed/simulator/thesis_robot_runner \
  --scenario mixed --mixed-map goal_directed_dynamic \
  --mixed-config config/mixed_experimental_relaxed.conf \
  --vehicle-model car --calibration-profile config/robots/car_calibrated.json \
  --cruise-speed 0.04 --check-mixed-config
```

Devono comparire `mixed_clearance_policy=physical_footprint`, `collision_buffer_m=0.01` e `virtual_gate_corridor_veto=0`. `--check-mixed-config` non abilita la marcia e non sostituisce il matching live.

## Nuova prova e log

Ripartire dallo start iniziale con una nuova cattura, non dalla posa avanzata vicino all'ostacolo. La procedura di [avvio manuale](Hardware_Mixed_Experimental_Relaxed.md#prossima-prova-fisica-sul-raspberry-pi-5) resta valida con la configurazione aggiornata. Usare porte e host PC della prova precedente; per il bridge remoto non lasciare `127.0.0.1`. I comandi seguenti assumono che `MIXED_CONTROLLER`, `MIXED_LIDAR` e `MIXED_PC_IP` siano già impostati come nella prova 20:35.

Sul Pi, nuova reference senza sovrascrivere quella precedente:

```sh
cd ~/Desktop/Autonomus-Agent
: "${MIXED_LIDAR:?Impostare la porta LiDAR usata nella prova precedente}"
MIXED_BIN="$PWD/build-hardware-mixed/simulator"
MIXED_CONFIG="$PWD/config/mixed_experimental_relaxed.conf"
MIXED_RUN="$(TZ=Europe/Rome date +%Y%m%d_%H%M%S)_mixed_footprint"
MIXED_REFERENCE="$PWD/datasets/localization/${MIXED_RUN}_start.csv"
mkdir -p reports/mixed-real datasets/localization
printf '%s\n' "$MIXED_REFERENCE" > reports/mixed-real/latest_reference.txt
"$MIXED_BIN/thesis_lidar_start_matcher" \
  --lidar-port "$MIXED_LIDAR" --vehicle-model car \
  --lidar-x 0.075 --lidar-y 0.040 --lidar-yaw-deg 0 \
  --min-range 0.12 --max-range 2.50 \
  --capture-reference "$MIXED_REFERENCE" --scan-count 10
```

Il robot deve restare fermo durante la cattura; il matcher non apre il controllore motori. Il successivo comando **può muovere il robot**, solo per avvio manuale dell'operatore dopo `status=reference_captured`, con GUI e bridge SLAM già attivi, prova supervisionata e arresto fisico accessibile. Il target resta 0,04 m/s. Nessun comando di movimento è stato eseguito durante questo sviluppo.

```sh
: "${MIXED_CONTROLLER:?Impostare la porta controllore}"
: "${MIXED_PC_IP:?Impostare IP del PC con bridge SLAM e GUI}"
printf '%s\n' "$MIXED_RUN" > reports/mixed-real/latest_run.txt
cp "$MIXED_CONFIG" "reports/mixed-real/${MIXED_RUN}.conf"
printf '%s\n' "$MIXED_REFERENCE" > "reports/mixed-real/${MIXED_RUN}_reference.txt"
git rev-parse HEAD > "reports/mixed-real/${MIXED_RUN}_revision.txt"
git diff -- . ':!third_party/progettotesi' > "reports/mixed-real/${MIXED_RUN}_source.diff"
"$MIXED_BIN/thesis_robot_runner" \
  --controller-port "$MIXED_CONTROLLER" --lidar-port "$MIXED_LIDAR" \
  --scenario mixed --mixed-map goal_directed_dynamic --mixed-config "$MIXED_CONFIG" \
  --vehicle-model car --calibration-profile config/robots/car_calibrated.json \
  --start-reference "$MIXED_REFERENCE" --cruise-speed 0.04 --dt 0.10 --max-steps 600 \
  --slam-bridge-host "$MIXED_PC_IP" --slam-bridge-port 9760 --slam-pose-feedback \
  --stream-host "$MIXED_PC_IP" --stream-port 9559 \
  --telemetry-csv "reports/mixed-real/${MIXED_RUN}.csv" \
  > "reports/mixed-real/${MIXED_RUN}.log" 2>&1
```

Per fermare il runner usare `Ctrl+C`; se processo/collegamento non risponde usare l'arresto fisico. Da un altro terminale sul Pi si possono seguire i log:

```sh
cd ~/Desktop/Autonomus-Agent
MIXED_RUN="$(cat reports/mixed-real/latest_run.txt)"
tail -f "reports/mixed-real/${MIXED_RUN}.log"
```

A prova terminata:

```sh
python3 tools/analyze_hardware_mixed.py "reports/mixed-real/${MIXED_RUN}.csv" \
  --log "reports/mixed-real/${MIXED_RUN}.log" \
  --geometry-dir "reports/mixed-real/${MIXED_RUN}_geometry" \
  > "reports/mixed-real/${MIXED_RUN}_summary.json"
```

Conservare CSV/log, configurazione, diff, reference/raw e snapshot GUI. Per questa modifica guardare soprattutto `mixed_footprint` e `mixed_braking`: ora distinguono la distanza reale del punto dal margine richiesto. Per confrontare la sola politica di clearance creare una copia della stessa configurazione sostituendo `physical_footprint` con `conservative_envelope`; STRICT cambia anche la filosofia di selezione.

## File modificati

- `config/mixed_experimental_relaxed.conf`.
- `simulator/include/mvc/controller/hardware_planner/{mixed_navigation.h,mixed_mission.h}`.
- `simulator/src/controller/hardware_planner/mixed_navigation.cpp` e `parts/{mixed_navigation.inc,gap_detection.inc}`.
- `simulator/src/controller/hardware_app/parts/entrypoint.inc`.
- `simulator/src/tests/integration/hardware_mixed_relaxed_test.cpp` e `data/mixed_20261018_203702_footprint_scans.txt`.
- `tools/extract_mixed_stop_fixture.py`, questo documento, il collegamento nella guida sperimentale e `.gitignore`.
