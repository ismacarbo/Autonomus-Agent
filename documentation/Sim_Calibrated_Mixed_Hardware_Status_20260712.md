# Sim-Calibrated e mixed hardware - stato 2026-07-12

> Aggiornamento: per lo stato successivo al refactor MVC, la scala structured
> corrente e la matrice del 25 luglio 2026 vedere
> `documentation/Codex_Handoff_Status_20260725.md`. Le tabelle di smoke qui
> sotto sono mantenute come storico del 12 luglio.

Questa nota descrive l'aggiornamento della pipeline sim-to-real successivo alla
tesi. L'obiettivo e' preparare una campagna di prove accoppiate e ripetute,
senza reinterpretare i report storici come un benchmark controllato.

## Geometria comune

Il preset mixed hardware-aligned usa ora una scala metrica coerente:

```text
arena:                 1.20 x 1.20 m
robot car:             0.25 x 0.15 m
robot cingolato:       0.25 x 0.15 m
wheelbase nominale:    0.18 m
track width:           0.13 m
start:                 (0.15, 0.60) m
goal:                  (1.00, 0.60) m
```

L'obiettivo e' mantenuto a `20 cm` dal bordo destro. In questo modo il centro
del robot non viene portato in una regione incompatibile con semilunghezza del
corpo e soglia LiDAR di arresto.

L'ostacolo sintetico del preset `hardware_aligned` interseca la centerline e
obbliga il planner mixed a produrre una deviazione:

```text
obstacle: x=[0.570, 0.700] m, y=[0.080, 0.630] m
```

Anche tutti i preset structured integrati vengono ora trasformati dalla stessa
funzione condivisa, indipendentemente dal profilo del robot:

```text
arena structured:       1.20 x 1.20 m
span massimo del percorso strutturato: 0.90 m
```

La trasformazione viene applicata a start, goal, centerline, ostacoli e gate.
Il preset unstructured non viene modificato. Prima di questa correzione il
runner hardware conteneva una seconda normalizzazione storica (`0.40 m` per la
car e `1.00 m` per il tank), che rendeva non confrontabili le due esecuzioni.

## Sim-Calibrated

`Sim-Calibrated` e' selezionabile dalla GUI e dalla CLI:

```sh
./build/simulator/thesis_planner_sim \
  --headless \
  --scenario mixed \
  --mixed-map hardware_aligned \
  --vehicle-model car \
  --calibrated-sim
```

Il livello e' deterministico e usa seed `20260711`. I parametri esportati nel
report sono:

| Proprietà | Valore |
| --- | ---: |
| Ritardo comando | `1` step (`50 ms` con `dt=0.05 s`) |
| Rumore distanza encoder | `0.00035 m` std |
| Scala encoder sinistra/destra | `0.995 / 1.005` |
| Rumore yaw IMU | `0.004 rad` std |
| Rumore yaw-rate IMU | `0.012 rad/s` std |
| Random walk bias IMU | `0.00008 rad` per aggiornamento |
| Rumore range LiDAR | `0.005 m` std |
| Dropout LiDAR | `0.006` |
| Scala attuatori sinistra/destra | `0.985 / 1.015` |
| Scala costante di tempo motore | `1.15` |

Questi valori sono prior hardware-informed. Non sono ancora il risultato di
una identificazione statistica indipendente e devono essere aggiornati con le
nuove prove ripetute.

## Correzioni mixed hardware

Sono state corrette quattro cause distinte.

1. In modalita' mixed la proiezione sulla centerline veniva usata come
   pseudo-misura di posa. Quando un gate veniva rilasciato, l'EKF veniva quindi
   richiamato artificialmente sulla strada. Ora la posa mixed prosegue da
   encoder e IMU; il LiDAR resta una sorgente di percezione per i gate.

2. Il gate veniva cancellato quando, durante la rotazione, l'ostacolo usciva
   dal settore LiDAR frontale. Un target gia' bloccato e' ora mantenuto per una
   finestra temporale controllata e validato prima della ricostruzione dei
   candidati.

3. Un singolo target lontano produceva una congiungente che attraversava
   l'ostacolo. Il bypass compatto usa ora due stadi: spostamento laterale prima
   del blocco, quindi avanzamento parallelo alla centerline fino a quando il
   retro del robot ha superato l'ingombro.

4. Dopo il bypass il riferimento rettilineo conservava la tangente finale e
   non garantiva il rientro laterale. La missione usa un secondo target per
   certificare il ricongiungimento al corridoio strutturato.

I report hardware archiviati rendono visibile il difetto precedente. Nella run
car `20260621_204747_088` il gate era selezionato in `246/565` campioni e si
osservavano `46` commutazioni, ma `passed_gates=0`. Nelle tre run tank del
`20260622` i gate erano presenti e visibili, con `7`, `20` e `40`
commutazioni, ma anche in quel caso il vecchio contatore restava a zero. Il
problema non era quindi assenza di percezione: il riferimento gate veniva
rilasciato e il controllo tornava ripetutamente alla centerline.

Il conteggio `passed_gates=2` nel preset compatto indica quindi:

```text
1: completamento del bypass fisico
2: ricongiungimento al riferimento strutturato
```

## Cingolato e pivot

Il follower locale e' ora un unico MPC con due modelli di predizione: bicicletta
per la car e uniciclo per il cingolato. Per il modello cingolato, quando
l'errore di heading supera la soglia il riferimento lineare viene posto a zero
e l'MPC mantiene un riferimento di yaw-rate. Il mixer differenziale finale
produce:

```text
v_left  = v - omega * track_width / 2
v_right = v + omega * track_width / 2
```

Con `v=0`, i due cingoli ricevono velocita' di segno opposto e il robot ruota
sul posto. Il mapping yaw/track non sostituisce quindi il controllore: converte
l'uscita del modello MPC uniciclo nei due canali di attuazione.

## Frontend

La GUI mostra ora:

- livello `Sim-Ideal`, `Sim-Calibrated` o `Sim-Reference`;
- dimensioni dell'arena;
- footprint del robot;
- stato dell'arbitro mixed (`structured road` o `gate bypass`);
- gli stessi dati di scala nel pannello hardware live;
- mappa di riferimento e ricostruzione LiDAR accumulata affiancate, sia in
  simulazione sia nello stream hardware;
- gate come segmento orientato secondo l'heading del target, non come cerchio;
- rendering LiDAR identico nei due workspace;
- grafici diagnostici separati per unita' fisica.

L'interfaccia usa il nome `Autonomous Reasoning Engine`. I preset sono raccolti
nel tab Scenario: sono stati rimossi soltanto i pulsanti rapidi duplicati,
mentre tutti i preset restano selezionabili nei menu a tendina. In particolare
la modalita' mixed conserva `Closed Obstacle Road`. I quattro riepiloghi
ridondanti del vecchio Mission Desk e i blocchi Mission Readiness sono stati
rimossi dal rendering; il preflight hardware non occupa la vista della run.

La vista di mappatura ha due backend dichiarati esplicitamente:

- fallback nativo: ricostruzione 2-D degli endpoint LiDAR con posa stimata;
- sidecar ROS 2: `slam_toolbox` con scan matching, loop closure e
  ottimizzazione del pose graph tramite Ceres.

Il bridge usa UDP locale e restituisce occupancy grid e posa corretta alla sola
vista diagnostica. Non modifica planner o MPC. Se il sidecar non risponde, la
GUI torna al fallback e lo indica nel titolo, senza presentarlo come SLAM.

## Test eseguiti

Build:

```sh
cmake -S . -B build
cmake --build build --target \
  thesis_planner_sim thesis_robot_runner thesis_world_stream_smoke \
  thesis_external_pose_smoke -j2
```

Round-trip della mappa sul formato usato dallo streaming:

```text
world_stream_smoke: ok
payload_bytes=788
road_points=24
obstacles=1
```

`ctest --test-dir build -R 'thesis_(external_pose|world_stream)' --output-on-failure`:

```text
2/2 tests passed
```

Smoke runner hardware con sensori sintetici:

| Profilo | Esito | Tempo | Gate | Cicli pivot | PWM opposti |
| --- | --- | ---: | ---: | ---: | ---: |
| Car | goal reached | `17.6 s` | `2` | `33` | `31` |
| Cingolato | goal reached | `61.8 s` | `2` | `102` | `450` |

Anche il percorso structured allora normalizzato a `0.50 m` era stato eseguito
nel runner hardware sintetico con LiDAR di mapping attivo. Questi due tempi
sono storici e non vanno attribuiti alla scala corrente `0.90 m`:

| Profilo | Esito | Tempo | Span percorso | Arena |
| --- | --- | ---: | ---: | ---: |
| Car | goal reached | `30.7 s` | `0.50 m` | `1.20 x 1.20 m` |
| Cingolato | goal reached | `27.4 s` | `0.50 m` | `1.20 x 1.20 m` |

Lo smoke esporta separatamente posa stimata e posa fisica sintetica. Questo
permette di osservare la deriva della baseline senza usarne il ground truth
come feedback del controllore.

Matrice canonica del simulatore principale, prodotta da
`scripts/run_canonical_simulation_matrix.sh`:

| Modalita' | Robot | Livello | Esito | Tempo | Gate |
| --- | --- | --- | --- | ---: | ---: |
| Structured | Car | Sim-Ideal | goal reached | `68.05 s` | `0` |
| Structured | Car | Sim-Calibrated | goal reached | `62.25 s` | `0` |
| Structured | Cingolato | Sim-Ideal | goal reached | `17.1 s` | `0` |
| Structured | Cingolato | Sim-Calibrated | goal reached | `11.25 s` | `0` |
| Unstructured | Car | Sim-Ideal | goal reached | `26.55 s` | `2` |
| Unstructured | Car | Sim-Calibrated | goal reached | `30.2 s` | `2` |
| Unstructured | Cingolato | Sim-Ideal | goal reached | `24.45 s` | `2` |
| Unstructured | Cingolato | Sim-Calibrated | goal reached | `25.25 s` | `2` |
| Mixed | Car | Sim-Ideal | goal reached | `13.4 s` | `1` |
| Mixed | Car | Sim-Calibrated | goal reached | `14.65 s` | `1` |
| Mixed | Cingolato | Sim-Ideal | goal reached | `11.7 s` | `1` |
| Mixed | Cingolato | Sim-Calibrated | goal reached | `12.1 s` | `1` |

Il risultato completo e' in
`results/canonical_simulation_matrix/summary.tsv`. I due eseguibili hanno
criteri diagnostici diversi: nel runner hardware il secondo passaggio
certifica il rientro sulla strada, mentre nel simulatore principale viene
contato il solo bypass fisico.

## Draft paper

La versione nel template IEEE/ICRA si trova in:

```text
paper/icra/main.tex
paper/icra/main.pdf
```

Il draft usa il vocabolario di un paper e separa esplicitamente:

- risultati storici selezionati;
- nuova implementazione Sim-Calibrated;
- smoke software;
- prove fisiche da ripetere con protocollo accoppiato.

Le sezioni che richiedono nuovi dati sono marcate nel titolo. Il confronto
fiduciale usa i valori ricavati dal file sorgente della prova: media `0.253 m`,
mediana `0.212 m`, RMSE `0.295 m` e p95 `0.524 m` su `1123` campioni
temporalmente correlati. Il grafico viene generato da
`paper/icra/plot_pilot_external_pose.py`: i vuoti vengono interpolati soltanto
per continuita' visiva, mentre le metriche restano calcolate sui `1123`
campioni misurati. Il PDF ha quattro pagine di contenuto e una quinta pagina di
riferimenti; il pannello dovra' essere rimpiazzato dai trial motion-capture
sincronizzati.

## Lavoro ancora necessario

Prima di presentare i risultati come benchmark scientifico servono:

- tutti i tentativi, inclusi fallimenti e timeout;
- almeno 5-10 ripetizioni per ogni condizione accoppiata;
- identificazione indipendente dei parametri Sim-Calibrated;
- geometria, start, goal e ostacoli identici tra livelli;
- incertezza e calibrazione indipendente del riferimento motion capture;
- metriche con media, deviazione standard, p95 e intervalli di confidenza;
- verifica sul robot reale delle nuove soglie mixed e del pivot del cingolato.

Il canale UDP per il motion capture e' gia' integrato e documentato in
`documentation/Motion_Capture_Hardware_Reference.md`. Il riferimento esterno
viene esportato e visualizzato, ma non entra nel loop di controllo.
