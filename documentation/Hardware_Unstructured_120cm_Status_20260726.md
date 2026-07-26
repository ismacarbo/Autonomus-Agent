# Stato hardware car e arena unstructured 1,20 m - 2026-07-26

## Esito corrente

La calibrazione metrica della robot car e' ora attiva nel profilo
`car_measured_2026_07` versione `1.1.0`, hash
`fnv1a64:d26d3757ec4a6bde`. Il controllo chiuso di velocita' sul micro non e'
attivo: i run descritti qui usano ancora il firmware `1.2` e il percorso PWM
compatibile con il firmware precedente.

I due nuovi bundle structured terminano entrambi in `goal_reached` e confermano
che la correzione encoder da `38 tick/giro` ha eliminato il precedente errore
metrico di circa 9,5 volte.

| Bundle | Tempo | Campioni | Distanza encoder | Traiettoria stimata | Max CTE | Max errore heading | Step medio / max |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `thesis_hardware_structured_validation_road_gui_bundle_20260726_195156_019` | 83,743 s | 941 | 6,767 m | 4,467 m | 0,063 m | 84,49 deg | 37,25 / 56,83 ms |
| `thesis_hardware_structured_validation_road_gui_bundle_20260726_200321_070` | 62,015 s | 707 | 6,372 m | 4,179 m | 0,060 m | 78,05 deg | 36,69 / 53,13 ms |

Il secondo e' il run migliore corrente: completa la missione circa 21,7 s prima
e riduce leggermente CTE ed errore di heading. Il primo run contiene interventi
manuali per riportare il robot dentro l'area; gli spike e le distanze di quel
bundle non sono quindi metriche utilizzabili per una validazione formale. Anche
il secondo deve essere ripetuto senza contatto manuale prima di entrare nella
batteria statistica del paper. Il divario ancora ampio tra distanza encoder e
traiettoria stimata e' coerente con riposizionamenti/slittamento e rende
necessario il confronto con il ground truth esterno.

Nei campioni compare inoltre `controller_error_code = 7` nonostante il moto e il
completamento corretti. Prima della raccolta formale va verificato il significato
di quel codice nel firmware `1.2`; non va confuso con un fallimento del planner,
ma deve essere documentato o eliminato.

## Nuova arena hardware unstructured

Il runner hardware e il workspace hardware della GUI usano ora lo stesso
riferimento metrico della structured:

```text
arena:                       1,20 x 1,20 m
span massimo del contenuto:  0,80 m
cornice dell'envelope:        0,20 m per lato
robot car:                    0,25 x 0,15 m (invariato)
raggio ruota:                 0,0327 m (invariato)
```

La cornice da 20 cm e' intenzionale. Una prima prova con contenuto da 0,90 m
lasciava troppo poco spazio allo start di Hardware Lab per una rotazione con il
padding di collisione e il plant urtava il bordo dopo 7,2 s. Non e' stato
miniaturizzato il robot per nascondere il problema.

La trasformazione:

- conserva l'aspect ratio di ogni preset;
- centra il contenuto nella nuova arena;
- trasforma coerentemente start, goal, ostacoli, gate, ampiezze dinamiche e
  soglie di attivazione espresse in metri;
- conserva nome/identita' e comportamento del preset;
- e' idempotente, quindi sincronizzare o applicare piu volte la stessa mappa non
  la restringe ulteriormente;
- si applica al runner hardware e alle mappe preparate/sincronizzate dalla GUI;
- non comprime i preset del simulatore principale: in questo modo Tight
  Corridor e gli altri scenari statistici non acquisiscono varchi piu stretti
  del footprint fisico.

Per `hardware_lab`, il layout hardware risultante e':

```text
bounds:  (0,000, 0,000) -> (1,200, 1,200) m
start:   (0,283, 0,541) m
goal:    (0,896, 0,585) m
scala del contenuto originale: 0,296296
```

Gli ostacoli del preset restano riferimenti di scenario per il plant sintetico.
Quando la GUI prepara una mappa hardware LiDAR-only, la sanitizzazione continua
a rimuovere la geometria nota: i gate operativi vengono quindi estratti dalle
scansioni reali, non letti dalla mappa.

## Profilo LiDAR/gate per l'arena compatta

La vecchia configurazione unstructured aveva un margine dinamico di `1,50 m` e
range coerenti con l'arena da `2,70 x 2,40 m`. Dentro una mappa da 1,20 m questo
impediva la conferma di un target stabile e il robot restava in scansione.

Per le sole missioni hardware unstructured con span non superiore a 1,25 m sono
ora applicati:

```text
LiDAR/planning range:          1,20 m
obstacle stop distance:        0,16 m
dynamic bounds margin:         0,10 m
free-distance threshold:       0,24 m
minimum gate width:            max(body width + 0,06 m, 0,21 m)
target distance:               0,18 ... 0,58 m
target/path clearance:         body_width/2 + 0,020 / 0,010 m
startup scan:                  1,20 s
gate track minimum hits:       1
gate track confirm/hold:       1,15 / 0,55
gate richiesti per completare: 2
```

Questa configurazione modifica soltanto percezione e selezione dei gate alla
nuova scala. Non cambia mapping cinematico, MPC, conversione PWM, protocollo RSP
o firmware.

## Verifiche eseguite

Build completata per:

```text
thesis_robot_runner
thesis_planner_sim
thesis_world_stream_smoke
```

CTest:

```text
5/5 passed
```

La regressione world-stream verifica in un solo test tutti i preset
unstructured riproducibili (`robot_validation`, `tight`, `slalom`, `lower`,
`hardware_lab`, `ideal`): bounds `1,20 x 1,20 m`, start/goal compatibili con il
raggio circoscritto del robot, identita' del preset conservata e trasformazione
idempotente.

End-to-end finale con pipeline hardware sintetica, car e Hardware Lab:

```text
status:                  goal_reached
tempo:                   25,4 s
step:                    254
gate attraversati:       2
collisione:              no
safety stop:             no
min LiDAR:               0,158 m
LiDAR disponibile:       si
dynamic gate pipeline:   attiva
```

In unstructured `goal_reached` certifica il completamento dei due gate dinamici,
non l'arrivo esatto al marker globale: questa semantica va mantenuta distinta
dalla chiusura del giro structured nel paper.

## Prossimo test reale consigliato

Sul Raspberry, dalla root del repository:

```sh
cmake --build build-ninja --target thesis_robot_runner thesis_planner_sim -j4

./build-ninja/simulator/thesis_robot_runner \
  --controller-port /dev/ttyACM0 \
  --lidar-port /dev/ttyUSB0 \
  --scenario unstructured \
  --unstructured-map hardware_lab \
  --vehicle-model car \
  --stream-host 100.66.27.57 \
  --stream-port 9559
```

Non aggiungere `--mcu-velocity-closed-loop` finche' il micro mantiene il firmware
precedente. Prima dell'avvio controllare nella GUI che il canvas riporti
`1,20 x 1,20 m`, che lo start sia circa `(0,283, 0,541)` e che il LiDAR produca
360 campioni validi. Il primo run reale deve essere interrotto se il robot esce
dalla cornice o se la distanza frontale scende sotto il margine fisico scelto;
un eventuale riposizionamento manuale rende la prova esplorativa e non formale.

## Punti principali del codice modificati

- normalizzazione geometrica generica:
  `simulator/src/model/world/parts/names_math.inc`, funzione
  `normalize_unstructured_world`;
- policy arena hardware:
  `simulator/src/model/world/scenario_model.cpp`, funzione
  `fit_hardware_unstructured_world`;
- costruzione del mondo e profilo compatto del runner:
  `simulator/src/controller/hardware_app/parts/options.inc`;
- riapplicazione del profilo dopo calibrazione e sincronizzazione GUI:
  `entrypoint.inc` e `stream_config.inc` nello stesso package hardware app;
- regressione collettiva:
  `simulator/src/tests/integration/world_stream_smoke_test.cpp`.
