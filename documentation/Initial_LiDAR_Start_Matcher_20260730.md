# Initial LiDAR start matcher

## Obiettivo

`thesis_lidar_start_matcher` verifica che il robot sia nella stessa posa prima
di ogni prova. In questa prima fase il risultato non viene ancora applicato
all'EKF durante il moto: il tool serve a validare separatamente acquisizione,
geometria LiDAR, scan matching, qualità e ripetibilità dello start.

Il programma apre esclusivamente `/dev/ttyUSB0` (o la porta LiDAR indicata). Non
apre il controller, non invia PWM e non può muovere i motori.

## Perché il matcher è nativo e non usa subito `/map`

Il bridge ROS 2 attuale avvia SLAM Toolbox in mapping online e resetta la
sessione quando cambia l'identificatore della run. Questo è corretto per creare
una mappa, ma non assegna una posa globale ripetibile: il primo scan diventa
sempre il nuovo zero.

Per validare lo start serve invece una reference congelata. Il matcher C++ usa
una point cloud acquisita nello start nominale e cerca direttamente la
trasformazione corrente-reference. Non richiede Docker, ROS 2 o rete ed è
riutilizzabile in seguito come sorgente di misura per l'EKF.

SLAM Toolbox resta indicato per la fase online successiva:

```text
encoder + IMU -> EKF locale -> odom
LiDAR + odom  -> SLAM Toolbox -> map->odom
posa planner  = map->odom * odom->base_link
```

## File principali

- API e strutture dati:
  `simulator/include/mvc/model/navigation/initial_lidar_matcher.h`
- ricerca scan-to-reference, score, covarianza e file reference:
  `simulator/src/model/navigation/initial_lidar_matcher.cpp`
- acquisizione RPLidar, CLI e report:
  `simulator/src/controller/hardware_calibration/lidar_start_matcher_main.cpp`
- launcher per la robot car:
  `scripts/run_car_lidar_start_matcher.sh`
- matrice simulata multi-seed:
  `scripts/run_sim_lidar_start_matrix.sh`
- test sintetico:
  `simulator/src/tests/unit/initial_lidar_matcher_test.cpp`

## Build

Sul Raspberry Pi, dalla root del repository:

```bash
cmake --build build-ninja --target thesis_lidar_start_matcher -j4
```

La build locale alternativa è:

```bash
cmake --build build --target thesis_lidar_start_matcher -j4
```

## 1. Creazione della reference

Collocare la car esattamente nella posa fisica che deve diventare lo start di
tutte le prove. L'ambiente deve essere già nella configurazione definitiva e il
robot deve restare fermo.

```bash
./scripts/run_car_lidar_start_matcher.sh /dev/ttyUSB0 \
  --capture-reference datasets/localization/car_validation_start.csv \
  --scan-count 10
```

La directory `datasets/` è ignorata da Git: la point cloud è un dato
sperimentale locale e non viene inclusa quando si pusha soltanto il codice.

La reference contiene:

- point cloud voxelizzata nel frame `base_link` dello start nominale;
- offset e orientamento LiDAR;
- inversione della direzione angolare;
- range minimo e massimo;
- dimensioni del footprint usate per eliminare self-hit;
- numero di scan aggregati e risoluzione dei voxel.
- seriale, firmware e revisione hardware del LiDAR;
- esito del controllo di stabilità tra scan pari e dispari;
- percorso del log raw e hash FNV-1a a 64 bit di metadati e point cloud.

La cattura richiede almeno quattro scan. Prima di salvare la reference divide
gli scan in due insiemi indipendenti (pari/dispari), li registra tra loro e
accetta la reference soltanto se la differenza resta entro 2 cm e 2 gradi e il
match supera le soglie geometriche. Se il robot si muove, qualcosa attraversa
la scena o i ritorni sono insufficienti, il file reference non viene creato;
resta invece il `*_raw_scans.csv` per diagnosticare la cattura rifiutata.

Una reference v2 viene riletta immediatamente dopo la scrittura. Durante le
prove successive sono obbligatori sia hash integro sia seriale LiDAR uguale a
quello della cattura. L'hash rileva modifiche accidentali o file incompleti;
non è una firma crittografica.

I valori predefiniti della car coincidono con la configurazione hardware:

```text
body:       0.25 x 0.15 m
lidar x:    0.075 m
lidar y:    0.040 m
lidar yaw:  162 deg
mirror:     enabled
```

Se la posizione fisica del LiDAR è diversa, correggere i valori già durante la
cattura, per esempio:

```bash
./scripts/run_car_lidar_start_matcher.sh /dev/ttyUSB0 \
  --capture-reference datasets/localization/car_validation_start.csv \
  --lidar-x 0.075 --lidar-y 0.040 --lidar-yaw-deg 162
```

La modalità match rilegge la geometria dal file reference, evitando che una run
usi accidentalmente extrinsic differenti.

## 2. Verifica di una nuova posa iniziale

Spostare e riposizionare il robot, lasciarlo completamente fermo e lanciare:

```bash
./scripts/run_car_lidar_start_matcher.sh /dev/ttyUSB0 \
  --reference datasets/localization/car_validation_start.csv \
  --scan-count 8 \
  --confirmations 3
```

L'output principale è:

```text
status=start_pose_confirmed
start_pose_accepted=1
confirmation_position_spread_m=...
confirmation_yaw_spread_deg=...
offset_x_m=...
offset_y_m=...
offset_translation_m=...
offset_yaw_deg=...
rmse_m=...
inlier_ratio=...
ambiguity_margin=...
confidence=...
observability_x=...
observability_y=...
observability_yaw=...
total_compute_ms=...
```

La decisione positiva non dipende da una sola acquisizione. Il programma crea
tre nuvole indipendenti, esegue tre match e abilita lo start soltanto se:

1. tutti e tre i match sono validi;
2. tutti e tre sono individualmente dentro le soglie;
3. la dispersione è al massimo 2,5 cm e 2 gradi.

Un singolo rifiuto oppure risultati tra loro incoerenti mantengono lo start
disabilitato. Il tool continua ad aprire esclusivamente il LiDAR: anche in caso
positivo non invia comandi al controller.

La trasformazione stimata porta la point cloud del robot corrente nel frame
della reference. Di conseguenza:

- `offset_x_m > 0`: lo start corrente è davanti allo start nominale;
- `offset_y_m > 0`: lo start corrente è a sinistra dello start nominale;
- `offset_yaw_deg > 0`: il robot è ruotato in senso antiorario;
- `offset_translation_m`: errore planare complessivo.

## Soglie iniziali

Le soglie predefinite sono:

```text
ricerca x/y:             +/- 0.30 m
ricerca yaw:             +/- 30 deg
offset start massimo:       0.08 m
errore yaw massimo:         8 deg
RMSE massimo:               0.045 m
inlier ratio minimo:        0.55
ambiguity margin minimo:    0.025
dispersione conferme:        0.025 m / 2 deg
timeout per match:           20000 ms
```

Queste tolleranze servono per la prima validazione. Dopo 10-20 riposizionamenti
reali conviene ridurre progressivamente lo start a circa 2-3 cm e 2-3 gradi, se
la distribuzione osservata lo consente:

```bash
./scripts/run_car_lidar_start_matcher.sh /dev/ttyUSB0 \
  --reference datasets/localization/car_validation_start.csv \
  --max-start-offset 0.03 \
  --max-start-yaw-deg 3
```

## Score e sicurezza

Per ogni candidato `(x,y,yaw)` il matcher:

1. trasforma gli endpoint dello scan corrente nel frame reference;
2. cerca per ogni punto il vicino più prossimo nella reference;
3. rifiuta corrispondenze oltre la distanza massima;
4. scarta la coda peggiore dei residui per tollerare outlier;
5. calcola RMSE, percentuale di inlier e score con penalità per punti non
   associati;
6. esegue una ricerca coarse-to-fine;
7. confronta la soluzione con una soluzione geometricamente distinta per
   rilevare ambiguità;
8. produce una covarianza conservativa derivata dai residui e dal numero di
   corrispondenze.
9. misura la crescita locale del costo in `x`, `y` e `yaw`, esponendola come
   osservabilità e gonfiando la covarianza lungo direzioni deboli;
10. interrompe coarse/fine search al timeout e rifiuta la misura invece di
    usare un risultato parziale.

La covarianza attuale è una stima diagnostica, non ancora la matrice ottenuta
dall'Hessiana di un ICP continuo. Prima dell'iniezione nell'EKF verrà calibrata
sui dati reali.

Un ambiente circolare o fortemente simmetrico può avere RMSE basso ma
`ambiguity_margin` insufficiente. In quel caso il rifiuto è corretto: bisogna
aggiungere uno o più elementi geometrici statici e asimmetrici visibili dal
LiDAR, senza abbassare alla cieca la soglia.

## Artefatti prodotti

Ogni verifica genera in `reports/`:

- `thesis_hardware_lidar_start_match_*.json`: risultato, soglie, sensore,
  tre risultati individuali, consenso, osservabilità, tempi, covarianza e
  percorsi degli artefatti;
- `*_raw_scans.csv`: tutti i fasci, inclusi quelli scartati, qualità, range,
  motivo del rifiuto e timestamp host di inizio/fine scan. Il timestamp per
  fascio è marcato `beam_timestamp_interpolated=1`, perché RPLidar A1 non lo
  fornisce nativamente;
- `*_clouds.csv`: point cloud reference, corrente non allineata e corrente
  allineata;
- `*_comparison.ply`: confronto visualizzabile in CloudCompare/Meshlab:
  reference rossa, scan corrente blu, scan allineata verde.

Codici di uscita:

- `0`: posa valida e dentro le tolleranze;
- `3`: match valido ma qualità o posa iniziale rifiutata;
- `4`: impossibile calcolare il match, per esempio per punti insufficienti;
- `1`: errore di porta, LiDAR, file o report.

## Passo successivo dopo la prova hardware

Quando il matcher sarà stabile su più riposizionamenti, la sua trasformazione
verrà composta con lo start della `WorldMap`:

```text
T_map_current = T_map_nominal_start * T_reference_current
```

Quella posa inizializzerà l'EKF prima di abilitare il controllo. Solo dopo questa
validazione lo stesso frontend verrà eseguito al timestamp degli scan durante la
missione, oppure usato per inizializzare una mappa serializzata di SLAM Toolbox.

## Confronto con la simulazione

La simulazione usa la stessa classe `InitialLidarMatcher` e gli stessi criteri
di accettazione del tool hardware. Non esiste una copia semplificata
dell'algoritmo.

L'eseguibile dedicato:

```text
thesis_sim_lidar_start_matcher
```

esegue questa sequenza:

```text
posa nominale WorldMap
  -> scansioni LiDAR reference

posa nominale + offset ground truth noto
  -> nuove scansioni con rumore/dropout
  -> matcher condiviso
  -> offset stimato
  -> errore rispetto al ground truth
```

Build:

```bash
cmake --build build-ninja --target thesis_sim_lidar_start_matcher -j4
```

Esempio sulla Closed Obstacle Road:

```bash
./scripts/run_sim_lidar_start_matcher.sh \
  --scenario mixed \
  --mixed-map obstacle \
  --offset-x 0.035 \
  --offset-y -0.025 \
  --offset-yaw-deg 3.5 \
  --seed 20260730
```

Gli altri preset principali possono essere verificati con:

```bash
./scripts/run_sim_lidar_start_matcher.sh \
  --scenario structured --structured-map validation --seed 11

./scripts/run_sim_lidar_start_matcher.sh \
  --scenario structured --structured-map circle --seed 12

./scripts/run_sim_lidar_start_matcher.sh \
  --scenario unstructured --unstructured-map hardware_lab --seed 13

./scripts/run_sim_lidar_start_matcher.sh \
  --scenario mixed --mixed-map hardware --seed 14
```

Per separare errore numerico e rumore del sensore:

```bash
# LiDAR senza rumore né dropout
./scripts/run_sim_lidar_start_matcher.sh --ideal --seed 1

# Profilo RPLidar calibrato
./scripts/run_sim_lidar_start_matcher.sh \
  --calibrated --range-noise 0.005 --dropout 0.006 --seed 1
```

Il JSON simulato aggiunge ai campi hardware equivalenti:

- offset iniziale ground truth;
- offset stimato;
- errore `x/y` e norma dell'errore planare;
- errore yaw in radianti e gradi;
- seed, rumore, dropout e modalità ideal/calibrated.

Anche gli artefatti hanno lo stesso formato CSV/PLY. La reference simulata è
salvata automaticamente accanto al report; con `--reference-out PATH` può
essere esportata in un percorso esplicito.

La prima matrice calibrated verificata il 30 luglio 2026 ha prodotto:

| Modalità | Errore posizione | Errore yaw | Esito |
| --- | ---: | ---: | --- |
| Structured Validation | 0,5 cm | 0,75 deg | accepted |
| Structured Circle | 0,5 cm | 0,75 deg | accepted |
| Unstructured Hardware Lab | 1,0 cm | 0,00 deg | accepted |
| Mixed Hardware | 0,5 cm | 0,25 deg | accepted |
| Mixed Closed Obstacle | 0,0 cm | 0,25 deg | accepted |

## Matrice statistica prima dell'hardware

Il launcher seguente esegue per default 20 start casuali sulla Closed Obstacle
Road. Ogni start usa tre point cloud indipendenti, quindi la decisione è la
stessa del tool hardware:

```bash
./scripts/run_sim_lidar_start_matrix.sh --seed 20260806
```

Per una campagna definitiva da 50 seed:

```bash
./scripts/run_sim_lidar_start_matrix.sh \
  --runs 50 \
  --seed 20260806 \
  --output-dir reports/lidar_start_matrix_50
```

Il campionamento è stratificato: alterna pose sicuramente ammesse e pose oltre
la soglia, queste ultime alternativamente per traslazione e yaw. In questo modo
anche una matrice breve contiene casi positivi e negativi. Il report
`*_matrix.csv` conserva ground truth, decisione attesa, decisione ottenuta,
classificazione, errore, dispersione delle conferme, osservabilità, covarianza,
tempo ed elementi valutati per ogni run. Il JSON riassume:

- true/false positive e true/false negative;
- stime invalide o accettate con errore eccessivo;
- errore medio e p95 di posizione e yaw;
- tempo medio e p95 del matcher.

Il comando termina con errore se compare un falso positivo, un risultato
invalido o una posa accettata ma inaccurata; i falsi rifiuti producono il codice
3. La smoke matrix verificata il 6 agosto 2026 sulla Closed Obstacle Road ha
classificato correttamente 6/6 start casuali, senza falsi positivi, falsi
negativi o stime inaccurate. La matrice stratificata finale ha inoltre
classificato correttamente 2/2 start ammessi e 2/2 start da rifiutare. Questi
sono controlli di integrazione, non un risultato statistico sufficiente per il
paper.
