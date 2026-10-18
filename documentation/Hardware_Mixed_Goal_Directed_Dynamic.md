# MixedGoalDirectedDynamic — hardware reale

Implementazione del 17 ottobre 2026. Nessuna prova fisica è stata eseguita durante lo sviluppo. Il preset si seleziona dal launcher `thesis_robot_runner`, con `--scenario mixed --mixed-map goal_directed_dynamic`. L'alias `MixedGoalDirectedDynamic` è equivalente. `--simulate` viene rifiutato per questo preset.

## Aggiornamento del 18 ottobre: copertura LiDAR opzionale

**Correzione successiva ai report `094142_980` e `094252_934`:** il gate viene ora selezionato soltanto se, oltre all'attraversamento, esiste una connessione verificata alla strada dalla posa prevista di liberazione del passaggio. Usa gli stessi target Frenet e le stesse clotoidi del rejoin effettivo. Questa verifica può rifiutare gate prima accettati: sul rettilineo da 1,30 m alcuni bypass non lasciano spazio per il rientro del car. Le condizioni vengono comunque ricontrollate dal vivo; la previsione non sostituisce le misure successive.

Un rejoin che fallisce esclusivamente per geometria/dinamica produce `rejoin_kinematics_unreachable`. Il motivo resta stabile durante l'intervallo di retry; `replan_interval_pending` e `waiting_for_fresh_scan` distinguono la pausa temporale dalla vera attesa di una scansione. Il log `mixed_rejoin_search` riporta strada residua, `n`, errore angolare e distribuzione dei rifiuti; ogni `mixed_candidate` di rejoin aggiunge picco di curvatura e limite. Lo stop resta recuperabile se nuove osservazioni/pose consentono una soluzione. Perdita sensori e collisioni conservano la priorità.

L'odometria della nuova Mixed integra ora anche i primi due intervalli encoder **già validati**: prima venivano persi durante lo streak iniziale, senza recuperare la distanza alla terza misura. Restano esclusi pacchetti non ready, scaduti, ripetuti, in overflow o cinematicamente implausibili; non viene stimato movimento dal PWM. La modifica è limitata a `mixed_dynamic.enabled` e non cambia il filtro dei runtime precedenti. [Analisi dei due report e comandi di aggiornamento](../reports/mixed_replan_20261018/Diagnosi_e_fix.md).

Su richiesta dell'operatore, `config/mixed_goal_directed_dynamic.conf` contiene ora **`visibility_policy obstacles_only`**. La presenza di settori senza raggi non causa più `current_unknown_footprint` o `unknown_footprint`: il controllo iniziale, ROAD, gate, rejoin e arco di arresto verificano gli ostacoli misurati senza richiedere copertura completa della sagoma. Le celle sconosciute restano sconosciute; non vengono inventati raggi o marcate celle libere. Un ostacolo in una zona non osservata può quindi sfuggire al controllo.

Restano attivi collisioni con ostacoli misurati, ingombro e margini, limiti cinematici/dinamici, scansioni fresche e sufficienti, matching, localizzazione, watchdog, STOP e perdita della comunicazione. Il parametro non modifica velocità, geometria, raggio minimo o posizione iniziale. La missione resta quella completa da 130 cm.

Per ripristinare la verifica di copertura impostare **`visibility_policy strict`** nello stesso file e riavviare manualmente il runner. `strict` resta il default del codice per le configurazioni che omettono il parametro. Il log iniziale della configurazione aggiornata deve riportare:

```text
mixed_visibility_policy=obstacles_only full_coverage_required=0 measured_collision_checks=1
```

Replay offline iniziale dei dati del report `20261017_163834_681`, prima della verifica preventiva del ritorno: superato il controllo iniziale, rilevato l'ostacolo, trovate 9 connessioni ammissibili su 63 per un gate misurato e raggiunto `GATE_FOLLOWING`. La verifica aggiunta successivamente rifiuta quelle nove connessioni per assenza di ritorno verificato sulla strada corta; la sagoma iniziale continua a passare. Sul CSV raw copiato dall'operatore, `--check-mixed-scan` termina con exit 0 e `mixed_start_footprint=measured_obstacles_clear full_visibility_verified=0`; gli stessi dati in modalità `strict` restituiscono exit 3, `unknown_footprint`. Sono verifiche offline, senza dispositivi o movimento: non dimostrano il completamento fisico del bypass o del rejoin.

[Comandi di trasferimento, build e missione completa](../reports/mixed_relax_20261018/Modifica_e_comandi.md). Le diagnosi delle sezioni storiche in fondo descrivono il comportamento precedente con `strict`.

## Confini della modifica

Il simulation planner, i suoi preset, il rendering, la GUI e i modelli fisici restano invariati. La directory storicamente denominata `simulator/` contiene anche il runtime del Raspberry Pi: le modifiche riguardano quel runtime e i suoi test offline. L'unica nuova API nel modello condiviso è `WorldMap::hardware_mixed_mission`, implementata nella libreria hardware: costruisce una mappa missione con strada, start, goal e limiti numerici, senza ostacoli, gate o template. Nel frame locale i limiti non rappresentano il perimetro fisico registrato. Non viene registrata fra i preset simulati. I vecchi preset hardware mantengono la modalità precedente, perché `mixed_dynamic.enabled` è falso per default.

Il nuovo percorso non esegue le vecchie euristiche Mixed per alternare checkpoint di bypass/rientro, non limita il numero di gate, non usa la parità dei gate e non richiama il raycast della mappa per conoscere gli ostacoli. Il detector esistente conserva filtri, tracking temporale e identità `gap_track_N`; per il nuovo preset sono esclusi i target laterali artificiali e i candidati senza apertura effettivamente misurata.

## Architettura e stati

`HardwarePlannerRunner::step_with_observation` mantiene acquisizione, stima, filtri di freschezza, MPC, conversione PWM/velocità ruote e telemetria. Se il preset è abilitato, la pianificazione passa alla macchina a stati in `parts/mixed_navigation.inc`.

| Stato | Comportamento e uscita |
|---|---|
| `ROAD_FOLLOWING` | Proietta la posa sulla strada persistente, campiona il tratto avanti, applica la politica di visibilità configurata e riutilizza `sel_jr` structured e MPC. Un ostacolo percepito nel corridoio attiva l'acquisizione. |
| `GATE_ACQUISITION` | Arresto; usa scansioni nuove per confermare aperture. I candidati devono avanzare lungo la strada, superare i controlli G2/cinematici/percezione e non ripetere aperture già passate. La preferenza di `sel_jr`/salience non può annullare un rifiuto. |
| `GATE_FOLLOWING` | ID, piano del gate, normale e larghezza restano bloccati. La traiettoria viene mantenuta se verificata; il passaggio richiede direzione coerente, sagoma compresa nell'apertura al passaggio del piano e parte posteriore oltre il piano con margine. La posa oltre il gate da sola non prova l'attraversamento. |
| `ROAD_REJOINING` | Rimuove il riferimento del gate e genera una nuova connessione alla strada originale. Non torna implicitamente al riferimento structured. |
| `SAFE_STOP` | PWM e target ruote a zero. La fase da riprendere resta distinta dallo stop; si rivaluta soltanto con dati validi. Un salto di posa richiede reset/riavvio da una posa iniziale nota. Il timeout del gate resta fermo fino all'intervento dell'operatore. |
| `GOAL_REACHED` | Richiede progressione finale, distanza dal goal globale, errore laterale, orientamento e velocità ridotta. Gate e rejoin non completano la missione. |

Dopo il gate viene conservata la posizione dell'apertura attraversata. Il contatore aumenta solo per attraversamenti fisici; un rejoin ha un contatore separato. I track vengono ricreati per il prossimo ostacolo con ID crescenti e le aperture passate vengono escluse entro un raggio configurabile. Non esiste un numero di gate richiesto per finire.

Dal fix del report `20261017_160347_774`, ogni gate misurato viene valutato con fino a 63 connessioni G2: tre distanze d'uscita, tre posizioni laterali entro la larghezza disponibile e sette orientamenti finali. L'orientamento finale della curva può differire dalla normale del passaggio; la normale, il piano e l'identità del gate restano quelli misurati. Il candidato deve attraversare il piano nel verso corretto, con sagoma e margini dentro l'apertura, e terminare con il retro libero. Restano obbligatorie curvatura, dinamica e collisioni; l'osservabilità dell'intero percorso è richiesta soltanto con `strict`. Anche il replan dello stesso gate usa questa ricerca, conservando lock e storia dell'attraversamento. La curva viene mantenuta quando resta valida e la ricerca viene limitata dal periodo di replanning.

## Strada persistente e Frenet

La strada è una spline G2 aperta, costruita con la stessa libreria Clothoids già usata dal planner. È conservata separatamente da `cl_`, dalla curva del gate e dalla curva di rientro. Le modifiche del planner locale non la sovrascrivono.

La proiezione cerca la distanza minima nel solo intervallo `[s_precedente - projection_back_m, s_precedente + projection_forward_m + spostamento]`, con affinamento numerico. La ricerca globale non può sostituire il risultato: questo impedisce di saltare su un ramo vicino o su un'altra parte di una strada intersecante. Il riferimento iniziale è `s=0`; un riposizionamento distante non viene interpretato come avanzamento della missione.

Nel punto proiettato si calcolano tangente `theta(s)`, `s`, `n = -sin(theta) dx + cos(theta) dy` e `delta_yaw = wrap(yaw - theta)`. Il valore `n` è positivo a sinistra della strada. Su un percorso curvo si usa la tangente della spline, non quella della traiettoria di bypass.

I target sono `s_i = min(lunghezza_strada, s0 + lookahead_i)`, con offset positivi e duplicati rimossi. Per ciascuno vengono costruiti posizione, tangente e curvatura della strada. La connessione è una spline di tre clotoidi G2 (`G2solve3arc`) dalla posa e curvatura correnti alla posa e curvatura del target. Non si cerca un'intersezione della vecchia curva del gate.

La selezione valuta lunghezza, integrale della curvatura al quadrato, clearance, avanzamento curvilineo e distanza residua dal goal. Endpoint e tangente sono vincoli, non soltanto termini di costo. Il tratto stradale successivo al target deve superare gli stessi controlli di collisione e la politica di visibilità scelta. Un target vicino non fattibile non impedisce di esaminare quelli successivi.

Le connessioni devono rispettare raggio/sterzo del modello car-like oppure curvatura/yaw rate del modello tracked. Gli estremi e la derivata della curvatura sono verificati su ogni clotoide, anche se più corta del passo di campionamento. Sono controllate accelerazione angolare e velocità di sterzata alla velocità prevista e a quella misurata. La regolarità geometrica da sola non autorizza il movimento. Non vengono amplificati i limiti di curvatura del car-like attraverso le euristiche dei vecchi preset compatti.

## Percezione, stabilità e arresto

La validazione usa raggi LiDAR reali con qualità e range validi. Due raggi vicini delimitano un settore conservativo osservato; intervalli angolari grandi e ritorni mancanti restano sconosciuti. La memoria locale conserva scansioni per un tempo limitato. Gli endpoint occupati prevalgono su osservazioni precedenti di spazio libero. Non viene usata la ground truth. Con `frame start_reference`, i bounds sono un inviluppo numerico finito, non il perimetro fisico registrato. Pareti e barriere sono rilevate dal LiDAR e sottoposte agli stessi controlli di sagoma degli altri ostacoli. Con `frame arena` si possono ancora fornire limiti fisici misurati nello stesso frame della strada.

Si verificano sagome rettangolari orientate, riempite con campioni, con margine ostacoli, margine posa, compensazione della discretizzazione e inflazione per il moto durante scansione e trasporto. La sola posizione del centro non basta. Con `strict`, la porzione già occupata dal corpo del robot è esclusa dalla richiesta di visibilità, ma non dal controllo di collisione con ritorni validi; tutto lo spazio aggiuntivo deve risultare osservato. Con `obstacles_only` si conserva il controllo di collisione sulla stessa sagoma con margini, senza richiedere copertura dei settori mancanti.

La sagoma attuale viene verificata prima della ricerca dei gate. `current_occupied_footprint` significa che un ritorno misurato entra nella sagoma con margine; `current_unknown_footprint`, presente soltanto con `strict`, significa che una parte del margine non è osservata. Il log `mixed_footprint` riporta il punto responsabile in coordinate di missione. Queste cause non vengono più nascoste da `no_verified_gate`. Le osservazioni fresche possono risolvere uno stop per copertura temporaneamente insufficiente; un settore cieco persistente resta sconosciuto anche con `obstacles_only`, ma in tale modalità non causa da solo uno stop.

La curva locale resta stabile finché il corridoio rimanente è valido e il robot la segue entro le soglie. Ogni ciclo la ricontrolla. Un nuovo ostacolo sulla curva fa tornare all'acquisizione; dati insufficienti causano stop. Una curva sostitutiva viene generata su scansione nuova, dopo il periodo minimo e con velocità lineare e angolare quasi nulle, per rendere conservativa la discontinuità del riferimento. Il tracking MPC e i limiti di variazione dei comandi restano attivi.

Il rejoin termina solo dopo tre scansioni nuove che confermano insieme `|n| < 0.045 m`, `|delta_yaw| < 0.12 rad`, almeno `0.15 m` di progressione dalla fase di rientro (ridotta alla distanza del target se termina prima sul goal) e raggiungimento della progressione del target, con strada successiva verificata. Durante la conferma il robot si ferma.

Sono mantenute le protezioni preesistenti. Il nuovo percorso aggiunge controlli su encoder/IMU, età delle scansioni, campioni validi, covarianza, salti della posa, geofence e assenza di riferimento. Ricontrolla anche l'arco del comando e quello del moto misurato per l'intervallo di reazione più frenata. Una pianificazione che esaurisce il margine di freschezza dei sensori non invia movimento. Il comando precedente non viene reinviato se i sensori sono già scaduti. Il riarmo automatico del controller è escluso per questo preset; STOP e fault del controller non vengono cancellati dal planner.

## Configurazione e localizzazione

L'arena comunicata dall'operatore misura **200 × 170 cm**, con **pareti/barriere visibili all'altezza del LiDAR**. Il robot può essere collocato nella posa iniziale scelta dall'operatore, senza coordinate obbligatorie `(35,85)` cm. La sagoma con margine deve essere priva di ostacoli misurati; la copertura completa è richiesta soltanto con `strict`. Con i parametri correnti il margine base è 9 cm dalla carrozzeria (3,5 cm ostacoli + 2,5 cm posa + 3 cm discretizzazione), oltre all'eventuale inflazione per il moto.

La prima cattura LiDAR definisce il sistema di riferimento della missione:

- origine `(0,0)` = centro del robot nella posa di cattura;
- asse +X = direzione del muso nella posa di cattura;
- asse +Y = sinistra del robot nella posa di cattura;
- strada nominale = rettilineo da `(0,0)` a `(1.30,0)` m;
- goal globale = **130 cm avanti rispetto alla posa di cattura**, invariato durante bypass e rejoin.

`config/mixed_goal_directed_dynamic.conf` contiene:

```text
frame start_reference
arena_size 2.00 1.70
road 0 0
road 1.30 0
lookaheads 0.35 0.55 0.80 1.10 1.50
visibility_policy obstacles_only
```

Seguono i parametri di sicurezza riportati sotto. Per cambiare la distanza del goal basta cambiare l'ultima coordinata X della strada. Le coordinate sono relative alla cattura, non a un angolo dell'arena. I lookahead vengono limitati al goal e deduplicati: non sono checkpoint obbligatori. Non è presente una missione di prova breve.

**Come vengono trattati i bordi:** dimensioni dell'arena e scan matching non forniscono da soli la posa del rettangolo nel frame del robot. Non viene quindi inventato un rettangolo centrato sullo start. `WorldMap::bounds` contiene un inviluppo finito `[-D,+D] × [-D,+D]`, con `D = hypot(2.00,1.70) ≈ 2.625 m`: qualsiasi punto dell'arena può distare al massimo una diagonale dallo start, qualunque sia l'orientamento iniziale. È un limite numerico esterno, **non una garanzia di permanenza dentro un rettangolo fisico registrato**. Nessuno spazio dell'inviluppo è dichiarato libero. Gli endpoint misurati delle pareti/barriere vengono controllati contro ogni sagoma futura con margine; con `obstacles_only` i tratti non osservati non bloccano da soli il percorso. Un bordo solo disegnato a terra non sarebbe rilevabile. Non è implementato un riconoscimento automatico dei quattro lati dell'arena.

Per missioni con coordinate fisiche note resta supportato `frame arena` insieme a `bounds min_x min_y max_x max_y`; il parser rifiuta di mescolare `frame start_reference` e bounds fisici. Le missioni precedenti con bounds espliciti senza `frame` mantengono il frame arena. Non vengono disabilitati controlli di collisione, freschezza, localizzazione, sagoma o frenata.

Il profilo car misura 25 × 15 cm e conserva curvatura massima 1.8 m⁻¹, raggio minimo circa 56 cm. Lo start è libero, ma ciò non garantisce che goal e aggiramenti siano raggiungibili da qualsiasi posa: puntare il muso lungo la linea desiderata, lasciando spazio fisico per arrivare a 130 cm e aggirare gli ostacoli. Un ostacolo o una parete misurati possono imporre SAFE_STOP; lo spazio non osservato lo impone soltanto con `strict`. Non vengono ridotti margini o raggio minimo per far entrare la manovra.

Il matcher iniziale è **obbligatorio per questo preset**: `--no-start-matching` viene rifiutato. La nuova reference car dedicata è `datasets/localization/car_mixed_local_start.csv`; il vecchio file collegato allo start `(35,85)` non viene ereditato. La prima cattura nella posizione liberamente scelta stabilisce lo start. Le scansioni delle esecuzioni successive correggono la posa nel frame della cattura, senza spostare la reference road o il goal. Per scegliere un nuovo start fisico, acquisire una nuova reference, conservando quella precedente con un nome diverso. Per ripetere la stessa missione, riutilizzare la reference e riposizionare vicino allo stesso punto.

Se la reference manca, è corrotta, instabile o ha estrinseci incompatibili, `connect()` rifiuta la connessione prima di aprire controller e LiDAR. Il file valido da solo non autorizza i motori: il runner richiede tre conferme indipendenti del matching a robot fermo. Non serve eseguire un secondo comando standalone di verifica. Le soglie esistenti sono 8 cm / 8° di offset massimo e 2.5 cm / 2° di dispersione fra conferme, relative alla cattura scelta, non a coordinate fisse dell'arena. Non sono garanzie di accuratezza.

**Gli ostacoli possono essere già presenti lungo la linea quando si acquisisce la reference.** Tenere fermi robot e scena durante la cattura. La point cloud serve esclusivamente al matcher iniziale: non viene inserita come mappa di ostacoli o sequenza di gate nel planner. La navigazione usa il LiDAR live. Se in una prova successiva gli ostacoli vengono spostati, il matching può ancora riuscire con strutture stabili sufficienti, oppure essere rifiutato se la scena è troppo diversa/ambigua; non viene aggirato un rifiuto.

Il profilo `config/robots/car_calibrated.json` usa yaw LiDAR **0°**, angoli raw specchiati, traslazione X=0.075 m / Y=0.040 m. I comandi sotto impostano questi estrinseci per la cattura: il default storico del tool generico, 162°, non è coerente con questo profilo. Una modifica del montaggio richiede calibrazione e nuova reference.

Il matcher corregge soltanto lo start. SLAM Toolbox resta il backend online: `--slam-pose-feedback` applica le correzioni limitate già supportate. Il rettilineo da 130 cm a 0.04 m/s richiede almeno 32.5 s, oltre al tempo dei bypass; l'odometria pura può superare la soglia di incertezza prima del goal. La localizzazione non affidabile causa stop; non si alzano le soglie per nascondere la deriva.

| Parametro | Default | Motivazione/calibrazione |
|---|---:|---|
| `--cruise-speed` | 0.06 m/s | Iniziare a 0.04; il nuovo preset rifiuta valori oltre 0.12. Verificare velocità encoder e velocità fisica. |
| `lookaheads` | 0.35…1.50 m | Spazio di manovra, raggio minimo, lunghezza ostacoli e portata realmente osservabile. |
| `road_preview_m` | 0.65 m | Rilevare il blocco prima di arrivare nel corridoio di arresto. |
| `obstacle_margin_m` | 0.035 m | Ingombro, rumore LiDAR, errore di tracking. Si aggiungono posa e discretizzazione. |
| `pose_margin_m` | 0.025 m | Errore massimo atteso della posa rispetto alla strada e alle osservazioni memorizzate. |
| `braking_decel_mps2` | 0.20 m/s² | Deve essere una decelerazione realmente ottenibile sul fondo/carico usato. |
| `reaction_time_s` | 0.40 s | Comprende latenza sensori, elaborazione e comando; misurare sul Pi. |
| `max_yaw_accel_rad_s2` | 0.60 rad/s² | Limite conservativo tracked e limite dinamico delle connessioni. |
| `observation_memory_s` | 0.60 s | Memoria corta; occorrono nuove osservazioni quando scade. |
| `visibility_policy` | `strict` nel codice; `obstacles_only` nel file della missione | Con `obstacles_only` i settori non osservati non impediscono la traiettoria; restano collisioni, margini e freschezza. |
| `max_beam_gap_rad` | 0.055 rad | Definisce la copertura osservata; blocca le traiettorie solo con `strict`. |
| `max_pose_sigma_m`, `max_yaw_sigma_rad` | 0.60 m, 0.35 rad | Guardie sull'incertezza dell'EKF esistente; calibrare insieme alla localizzazione. |
| `projection_back_m`, `projection_forward_m` | 0.08, 0.25 m | Impediscono salti di ramo senza bloccare la normale progressione. |
| `gate_timeout_s`, `gate_reject_radius_m` | 30 s, 0.22 m | Lock bloccato e riacquisizione di aperture già superate. |
| `gate_heading_search_rad` | 0.60 rad | Ampiezza degli orientamenti finali alternativi; non cambia la normale del gate o i limiti del car. |
| `gate_exit_search_m` | 0.30 m | Estensione aggiuntiva campionata oltre l'uscita minima del gate; tutta la curva viene controllata secondo la politica scelta. |

Gli altri parametri configurabili sono elencati in `mixed_mission.h` / `hardware_mixed::Config`. Il profilo robot car/tank continua a determinare ingombro, encoder, diametro ruote, carreggiata e attuazione. Non usare lo stesso profilo per entrambi i robot.

## Comandi sul Raspberry Pi: compilazione e configurazione

Dalla root del repository, nello stesso terminale per conservare le variabili:

```sh
cmake -S . -B build-hardware-mixed -DTHESIS_BUILD_SIM_GUI=OFF -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release
cmake --build build-hardware-mixed --target thesis_robot_runner thesis_lidar_start_matcher thesis_hardware_mixed_navigation_test thesis_hardware_runtime_settings_smoke -j 2
ctest --test-dir build-hardware-mixed -R '^thesis_hardware_mixed_navigation_offline$' --output-on-failure
build-hardware-mixed/simulator/thesis_hardware_runtime_settings_smoke --mixed-options

MIXED_BIN="$PWD/build-hardware-mixed/simulator"
MIXED_REFERENCE="$PWD/datasets/localization/car_mixed_local_start.csv"
mkdir -p datasets/localization reports/mixed-real
ls -l /dev/serial/by-id/
```

Impostare le porte identificate. Questi valori sono esempi; preferire i link stabili in `/dev/serial/by-id/`:

```sh
MIXED_CONTROLLER=/dev/ttyACM0
MIXED_LIDAR=/dev/ttyUSB0

"$MIXED_BIN/thesis_robot_runner" \
  --scenario mixed --mixed-map goal_directed_dynamic \
  --mixed-config config/mixed_goal_directed_dynamic.conf \
  --vehicle-model car --calibration-profile config/robots/car_calibrated.json \
  --start-reference "$MIXED_REFERENCE" --check-mixed-config
```

`--check-mixed-config` non apre dispositivi. Attendere `mixed_frame=start_reference`, `mixed_bounds_kind=unregistered_local_envelope`, start `(0,0)` e goal `(1.3,0)`. Prima della cattura è normale `start_reference_loaded=0`: exit 0 riguarda la validità geometrica/configurazione, non la disponibilità allo start. Non eseguire l'intera suite CTest, che include test simulati.

## Una cattura nella posa scelta

Collocare il robot dove si desidera partire, con il muso nella direzione del rettilineo. Gli ostacoli possono essere già collocati lungo la linea. Con trazione disabilitata tenere il robot fermo; chiudere eventuali programmi che occupano il LiDAR. La posa di cattura diventa il riferimento, senza misurare coordinate rispetto all'arena.

```sh
test ! -e "$MIXED_REFERENCE" && \
"$MIXED_BIN/thesis_lidar_start_matcher" \
  --lidar-port "$MIXED_LIDAR" --vehicle-model car \
  --lidar-x 0.075 --lidar-y 0.040 --lidar-yaw-deg 0 \
  --min-range 0.12 --max-range 2.50 \
  --capture-reference "$MIXED_REFERENCE" --scan-count 10
```

Richiedere exit 0, `status=reference_captured` e `reference_integrity_verified=1`. Il tool controlla stabilità e salva i raggi raw, apre solo il LiDAR e stampa `motor_controller_opened=0`; il rotore LiDAR può girare. Il `test` evita di sovrascrivere una reference già presente. Per un nuovo start, scegliere un nuovo percorso in `MIXED_REFERENCE` e ripetere la cattura. Non serve una nuova cattura per ogni run dallo stesso start.

## Missione effettiva con ostacoli, avvio manuale

Avviare il bridge SLAM esistente sulla macchina con Docker, in un terminale separato:

```sh
./tools/slam_toolbox_bridge/run.sh
```

Nel terminale del Pi:

```sh
MIXED_SLAM_HOST=127.0.0.1
```

Se il bridge gira sul PC, sostituire `127.0.0.1` con l'IP del PC; usa UDP 9760. Il bridge non comanda i motori. Le correzioni restano soggette alle guardie esistenti. Controllare nel CSV `slam_toolbox_connected`, `slam_pose_valid`, innovazioni e `slam_correction_accepted`; il semplice avvio del container non prova che la localizzazione sia affidabile.

Il comando seguente è la **missione completa da 130 cm, con ostacoli già presenti**. Può muovere il robot automaticamente appena il matcher accetta lo start. Avviarlo solo quando pronti, con arresto fisico disponibile; lasciare il robot fermo durante il matching.

```sh
MIXED_RUN="$(date +%Y%m%d_%H%M%S)_mixed_obstacles"
printf '%s\n' "$MIXED_RUN" > reports/mixed-real/latest_run.txt
"$MIXED_BIN/thesis_robot_runner" \
  --controller-port "$MIXED_CONTROLLER" --lidar-port "$MIXED_LIDAR" \
  --scenario mixed --mixed-map goal_directed_dynamic \
  --mixed-config config/mixed_goal_directed_dynamic.conf \
  --vehicle-model car --calibration-profile config/robots/car_calibrated.json \
  --start-reference "$MIXED_REFERENCE" \
  --cruise-speed 0.04 --dt 0.10 --max-steps 3000 \
  --slam-bridge-host "$MIXED_SLAM_HOST" --slam-bridge-port 9760 --slam-pose-feedback \
  --telemetry-csv "reports/mixed-real/${MIXED_RUN}.csv" \
  > "reports/mixed-real/${MIXED_RUN}.log" 2>&1
```

Il runner esegue automaticamente le tre conferme iniziali, poi ROAD → GATE → REJOIN → ROAD fino al goal globale, se esistono soluzioni verificate. Matching, ostacoli e rientri possono richiedere soste. `--max-steps` non è una durata garantita né un criterio di successo. Nessuna prova breve è richiesta da questa procedura; nessuna esecuzione fisica è stata avviata durante lo sviluppo.

## Arresto e log

`Ctrl+C` nel terminale del runner oppure `kill -TERM PID_DEL_RUNNER` richiedono l'arresto attraverso gli handler esistenti; l'uscita invia PWM zero e STOP. Tenere disponibile l'arresto fisico se processo o collegamento si bloccano. Dopo un arresto, riposizionare sullo start e riavviare; non riarmare alla cieca a metà strada. Per arrestare anche il bridge: `docker stop thesis-slam-toolbox-bridge` (fermare prima il robot). Chiudere il solo visualizzatore dei log non ferma il robot.

Da un secondo terminale, nella root del repository:

```sh
MIXED_RUN=$(cat reports/mixed-real/latest_run.txt)
tail -f "reports/mixed-real/${MIXED_RUN}.log"
```

Al termine della prova:

```sh
MIXED_RUN=$(cat reports/mixed-real/latest_run.txt)
python3 tools/analyze_hardware_mixed.py "reports/mixed-real/${MIXED_RUN}.csv" \
  --log "reports/mixed-real/${MIXED_RUN}.log" \
  --geometry-dir "reports/mixed-real/${MIXED_RUN}_geometry" \
  > "reports/mixed-real/${MIXED_RUN}_summary.json"
```

CSV: stato, causa stop, posa/yaw, `s/n/delta_yaw`, goal, ID gate, contatori, target/s_target, candidati, replan, velocità, comandi, età sensori, latenze, offset e hash reference, qualità matching e confronto SLAM. Il log aggiunge transizioni, rifiuti dei candidati, strada (`mixed_road`) e riferimenti locali (`mixed_reference`). La traiettoria eseguita è nelle colonne `position_x/position_y/yaw`. Il recorder rifiuta di sovrascrivere CSV; usare nomi nuovi anche per i riepiloghi. Il riepilogo non certifica riuscita o sicurezza fisica.

`mixed_gate_search` riporta il numero reale di connessioni provate per gate e il conteggio di ogni risultato: `curvature_or_yaw_rate`, `aperture_crossing_rejected`, `occupied_footprint`, `unknown_footprint`, `accepted`. `mixed_candidate ... result=accepted` aggiunge punto e orientamento finale selezionati. Per vedere le nuove diagnosi: `rg 'mixed_footprint|mixed_gate_search|mixed_candidate|mixed_transition' reports/mixed-real/*.log`.

Il report fisico `20261017_160347_774` ha matcher accettato, SLAM collegato (187 aggiornamenti) e distanza dalla carrozzeria di circa 30 cm dalla parete. Rimane un intervallo di circa 15° privo di ritorni utilizzabili attorno a −120° nel frame robot, che interseca il margine laterale/posteriore. Anche le ultime cinque scansioni negli ultimi 0,60 s non verificano quella zona. La build diagnostica precedente lo segnalava come `current_unknown_footprint`; la stessa causa resta bloccante con `strict`, mentre la configurazione `obstacles_only` del 18 ottobre la disattiva. Il JSON GUI non permette di distinguere una schermatura del LiDAR da ritorni scartati per range/qualità/sagoma.

Per analizzare il CSV raw già prodotto dalla cattura iniziale, senza aprire dispositivi o muovere il robot, sul Pi si può usare:

```sh
python3 tools/analyze_hardware_mixed.py \
  --raw-start-scan "${MIXED_REFERENCE%.csv}_raw_scans.csv" \
  --raw-sector-deg 110 130
```

Con il profilo car 1.8.0 gli angoli raw +110…+130° corrispondono al settore robot −130…−110°. Il riepilogo mostra ritorni per scansione, qualità zero, motivi di esclusione della cattura e range. I filtri della cattura e quelli della navigazione differiscono; matching riuscito non certifica copertura della sagoma. Se manca la visibilità fisica occorre liberare la visuale del sensore, verificare il montaggio e, se cambia, aggiornare gli estrinseci e riacquisire la reference. Se il CSV mostra ritorni scartati si può investigare il filtro specifico, senza dichiarare libero lo spazio mancante.

## Protocollo A–F

| Prova | Preparazione manuale | Evidenza da controllare |
|---|---|---|
| A | Strada libera | ROAD, nessun gate/rejoin spurio; fine solo sul goal globale. |
| B | Un ostacolo non presente nella configurazione | ROAD → acquisizione → gate → rejoin → ROAD; sagoma libera e goal invariato. |
| C | Due ostacoli separati | Due cicli, ID distinti, due gate e due rejoin, nessun limite a due gate nel codice. |
| D | Spostare fisicamente l'ostacolo e ripartire | Nessuna modifica di coordinate ostacolo/checkpoint; target diverso derivato dalle misure. |
| E | Impedire il primo rientro | Rifiuto motivato del primo candidato e selezione più avanti oppure SAFE_STOP. |
| F | Passaggio stretto / osservazioni insufficienti | Stop se apertura misurata troppo stretta, LiDAR assente/scaduto o punti validi insufficienti. Un settore mancante da solo causa stop soltanto con `strict`. |

Annotare misura e posizione fisica degli ostacoli solo nel resoconto della prova, non nella configurazione operativa. Non introdurre ostacoli durante un movimento senza una procedura controllata: per le prime prove fermare e riavviare.

## Limitazioni da verificare sul robot

- Non è un global planner: se nessuna connessione locale supera i controlli configurati, si ferma. Con `obstacles_only` una connessione può attraversare spazio non osservato; un ostacolo non misurato può sfuggire al controllo. Non implementa un recovery in retromarcia nuovo.
- Il piano di un gate può non liberare un ostacolo lungo. Il rejoin successivo deve essere verificato; può terminare in stop in presenza di occlusioni. Non viene ipotizzata la forma completa dell'ostacolo da una scansione.
- Il LiDAR è planare: oggetti fuori piano, superfici problematiche e dettagli fra i raggi non sono certificati dal controllo geometrico. Serve validazione reale del sensore e dei margini.
- I limiti di frenata, errore posa e tracking sono parametri di ingegneria da misurare. La verifica offline non certifica accelerazioni o clearance effettive della piattaforma.
- La memoria delle osservazioni scade rapidamente. `strict` può produrre stop per copertura insufficiente; `obstacles_only` disattiva quel criterio senza modificare la mappa o aumentare la durata della memoria.
- Il tracking viene mantenuto fra replanning; la sostituzione di una curva avviene dopo decelerazione/arresto. Non si promette un bypass senza soste.
- ROAD e rejoin sono prioritariamente per una strada aperta. La proiezione locale previene salti ma non risolve una localizzazione iniziale sbagliata o una topologia globale ambigua.
- La stima odometrica può superare il limite di covarianza prima della fine della missione lunga. SLAM feedback e calibrazione sono parte della validazione fisica necessaria.
- La configurazione live di mappa/profilo è rifiutata nel nuovo preset. Ripartire dal launcher per cambiarla; nessuna nuova GUI è stata introdotta.
- Compilazione e tempi misurati nell'ambiente di sviluppo non sostituiscono compilazione, latenza e prove sul Raspberry Pi 5.

## Verifiche dello sviluppo prima della modifica del 18 ottobre

Le sezioni seguenti conservano risultati e diagnosi storici con copertura `strict`. Le indicazioni per recuperare la copertura non sono un prerequisito aggiuntivo della modalità `obstacles_only` richiesta successivamente; per i comandi attuali vedere l’aggiornamento in apertura.

Eseguite il 17 ottobre 2026 nell'ambiente di sviluppo **Linux x86_64, GCC 15.2, Release**, con GUI disabilitata. Compilati con successo `thesis_robot_runner`, `thesis_hardware_mixed_navigation_test` e gli eseguibili necessari ai controlli sotto. Non è una compilazione eseguita sul Pi né una prova di movimento.

| Verifica | Risultato reale |
|---|---|
| `thesis_hardware_mixed_navigation_offline` | PASS, 0.52 s |
| `thesis_structured_path_projection` | PASS |
| `thesis_actuation_model_mapping` | PASS |
| `thesis_straight_line_calibration_math` | PASS |
| `thesis_drivetrain_diagnostics_logic` | PASS |
| `thesis_initial_lidar_matcher_math` | PASS, 0.48 s |
| Harness hardware `--trial-recording` | PASS, exit 0 |
| Harness hardware `--actuation-guards` | PASS, exit 0 |
| Harness hardware `--structured-mixed` | PASS, exit 0 |
| Harness hardware `--mixed-options` | PASS, exit 0 |
| `--check-mixed-config`, car, missione nel frame della reference | PASS, nessuna connessione hardware; reference reale ancora da acquisire |
| Tool CSV ed esportazione geometria su fixture sintetica | PASS |
| `git diff --check` | PASS |

La selezione CTest ha superato **6/6 test**, tempo totale riportato **1.00 s**. I log sono in `build-hardware-mixed/validation/` (`local-frame-build.log`, `local-frame-offline-tests.log`, `local-frame-options.log`, `local-frame-config.log`; i log precedenti `arena-*` restano conservati per le verifiche precedenti). Si è verificata anche l'assenza di differenze nei sorgenti di simulation planner, GUI/rendering, preset WorldMap simulati, `simulated_plant.inc` e modello fisico condiviso. Il submodule `third_party/progettotesi` era già modificato all'inizio ed è stato lasciato intatto.

Il nuovo test verifica segno/tangente Frenet, proiezione limitata dalla progressione, strada curva, target futuri, clotoidi valide e limiti cinematici/dinamici, collisioni, spazio ignoto e memoria scaduta, attraversamento con sagoma e direzione, conferma del rejoin, due cicli distinti, scelta di un candidato successivo, rientro impossibile, nuovo ostacolo sul rejoin, apertura ricavata da raggi sintetici e comandi reali del runner su osservazioni iniettate per car e tank, priorità MPC sul gate verificato, attesa dell’arresto angolare e rientro vicino al goal finale. L'aggiornamento dell'arena verifica anche il controllo hardware sul rettilineo da 130 cm, il rifiuto di una strada con sagoma fuori area, il blocco di `connect()` senza reference o matcher e il rifiuto CLI di `--no-start-matching` per questo preset. Il passaggio al frame locale verifica la partenza da `(0,0)`, la correzione della sola posa tramite tre risultati di matching concordi, la persistenza di strada e goal e il rifiuto di configurazioni che confondono bounds fisici e frame della reference. Le pose e i raggi sono fixture matematiche: nessun plant, simulation planner o trasporto seriale viene eseguito. Questi test non certificano le prove fisiche A–F.

Per ripetere gli harness senza simulatore:

```sh
cmake --build build-hardware-mixed --target thesis_hardware_runtime_settings_smoke -j 2
build-hardware-mixed/simulator/thesis_hardware_runtime_settings_smoke --trial-recording
build-hardware-mixed/simulator/thesis_hardware_runtime_settings_smoke --actuation-guards
build-hardware-mixed/simulator/thesis_hardware_runtime_settings_smoke --structured-mixed
build-hardware-mixed/simulator/thesis_hardware_runtime_settings_smoke --mixed-options
```

Aggiornamento dopo il report fisico `20261017_160347_774`: compilazione Release riuscita di runner e test hardware; **3/3 test offline passati in 0,91 s** (`thesis_hardware_mixed_navigation_offline`: 0,66 s; `thesis_start_matching_runtime`: 0,25 s; `thesis_slam_observation`: 0,00 s). La suite Mixed include connessioni alternative con limite car invariato, rifiuti per apertura stretta/collisioni/spazio ignoto, replan MPC con normale e ID stabili, e le cinque scansioni misurate del report. Quest'ultima regressione richiede lo stop `current_unknown_footprint`, non una falsa autorizzazione al moto. Verificati anche parser della missione senza connessione hardware e riepilogo Python dei CSV raw (qualità zero, self hit, settore assente, wrap angolare, schema errato). Log della build e dei test: `build-hardware-mixed/validation/gate-search-fix-{build,tests}.log`. Nessuna esecuzione sul Raspberry e nessun movimento fisico effettuati per questa correzione.

## Verifica offline della visibilità iniziale (report delle 16:38)

Diagnosi storica in modalità `strict`; il comando usa la politica presente nel file passato con `--mixed-config`.

Il report `20261017_163834_681` conferma matching accettato (0,966124), SLAM collegato (83 aggiornamenti), 148 campioni `current_unknown_footprint` dopo 15 campioni di attesa iniziale. Non c'è movimento. Il punto che blocca la sagoma è circa `(-0.0586364,-0.165)` m: rispetto al LiDAR si trova a −123,10°, verso destra e dietro. Nella scansione finale i raggi esportati che lo delimitano sono −126,23° e −112,70°, separati da 13,53°; il limite resta 3,15°. La ricostruzione GUI contiene punti già filtrati, non tutti i ritorni raw.

**Contare i punti in un settore non certifica che il settore sia interamente osservato.** I 55 ritorni accettati dalla cattura nell'intervallo raw 110–130° possono trovarsi ai bordi della lacuna centrale. Il riepilogo Python ora mostra angolo, range, qualità e motivo del filtro per ogni ritorno del settore, separatamente per scansione.

Il runner aggiunge `--check-mixed-scan`: legge un CSV raw di una singola cattura stazionaria con la stessa conversione, specchiatura, filtri e controllo della sagoma del runtime Mixed. La memoria rimane di 0,60 s e non viene fusa tutta la cattura in una mappa permanentemente libera. Le colonne `accepted` del matcher non vengono usate come autorizzazione: per esempio qualità zero viene rifiutata dal planner anche se il matcher aveva conservato il punto. Ritorni positivi oltre la portata del planner sono limitati alla portata come nel runtime, senza creare un ostacolo artificiale sul limite. Un CSV con tempi incoerenti o acquisizioni distinte viene rifiutato.

Dopo aver aggiornato i sorgenti sul Raspberry, con il runner precedente fermo:

```sh
cd ~/Desktop/Autonomus-Agent
cmake --build build-hardware-mixed \
  --target thesis_robot_runner thesis_hardware_mixed_navigation_test thesis_hardware_runtime_settings_smoke -j 2 && \
ctest --test-dir build-hardware-mixed \
  -R '^thesis_hardware_mixed_navigation_offline$' --output-on-failure && \
build-hardware-mixed/simulator/thesis_hardware_runtime_settings_smoke --mixed-options
```

Nel terminale che conserva `MIXED_REFERENCE`, verificare **la cattura già salvata**, senza avviare controller, LiDAR, rete o motori:

```sh
build-hardware-mixed/simulator/thesis_robot_runner \
  --scenario mixed --mixed-map goal_directed_dynamic \
  --mixed-config config/mixed_goal_directed_dynamic.conf \
  --vehicle-model car --calibration-profile config/robots/car_calibrated.json \
  --check-mixed-scan "${MIXED_REFERENCE%.csv}_raw_scans.csv"
```

L'output riporta `hardware_connected=0 motor_controller_opened=0 motion_authorized=0`, i conteggi dei filtri e, se manca copertura, i due raggi che delimitano il punto responsabile del rifiuto. Exit code **3** e `mixed_start_footprint=blocked` sono un risultato diagnostico, non un errore di avvio. Exit code **0** riporta `observed_clear` con `strict`, oppure `measured_obstacles_clear full_visibility_verified=0` con `obstacles_only`. Questi risultati descrivono soltanto l'ultima sagoma registrata: non certificano la strada, i gate, lo stato presente dei sensori o il successo di una missione. Non è una prova breve con movimento.

Il log live aggiunge `mixed_visibility cause=beam_gap|range_occluded|no_fresh_beams`, estremi del settore, range, età e conteggi dei filtri quando compare lo stop. Se la cattura supera il controllo ma le scansioni live no, confrontare queste informazioni: una reference storica non sostituisce osservazioni live mancanti.

Per risolvere un settore effettivamente non osservato occorre intervenire sulle misure: verificare la linea di vista del LiDAR nel settore indicato, possibili schermature di cavi/supporti e superfici che non restituiscono misure valide. Dai soli endpoint esportati non si distingue una schermatura da perdita in acquisizione o ritorni invalidi. Non alzare la soglia angolare né ridurre i margini per aggirare lo stop. Se si cambia montaggio, correggere gli estrinseci e acquisire una nuova reference; se si cambia posa, catturare una nuova reference. Lo start resta libero.

La modifica diagnostica precedente non cambiava i criteri di movimento: lo stesso insieme incompleto richiedeva ancora stop. La successiva politica `obstacles_only`, descritta in apertura, modifica esplicitamente quel comportamento; la partenza fisica resta da verificare. Simulation planner, preset e GUI restano invariati.

Verifica locale Linux x86_64: build Release del runner e degli harness riuscita; **3/3 CTest offline passati in 0,86 s**, più `thesis_hardware_runtime_settings_smoke --mixed-options` (parser e controllo raw). La CLI reale è stata eseguita senza porte su fixture di scansione completa, settore mancante, qualità zero e replay degli endpoint esportati: rispettivamente exit code 0, 3, 3, 3, sempre `motion_authorized=0`. Il replay degli endpoint non ricostruisce ritorni raw non esportati. Artefatti in `reports/mixed_visibility_20261017/`. Nessuna build sul Pi o prova fisica è stata eseguita dall’assistente.

## Elenco dei file della modifica

Ogni modifica sotto `simulator/` appartiene all'infrastruttura hardware condivisa o ai suoi test; nessuna modifica al comportamento simulato viene abilitata.

| File | Scopo |
|---|---|
| `.gitignore` | Esclusione build dedicata e inclusione di questa guida. |
| `config/mixed_goal_directed_dynamic.conf` | Missione e parametri reali, senza ostacoli/gate. |
| `documentation/Hardware_Mixed_Goal_Directed_Dynamic.md` | Architettura, istruzioni, limiti e risultati. |
| `tools/analyze_hardware_mixed.py` | Riepilogo CSV ed esportazione geometria dai log. |
| `simulator/CMakeLists.txt` | Sorgente nella libreria hardware e target CTest offline. |
| `simulator/include/mvc/model/world/world.h` | Dichiarazione della sola factory hardware, senza preset simulato. |
| `simulator/include/mvc/controller/hardware_planner/mixed_mission.h` | Factory della missione e lettura configurazione. |
| `simulator/include/mvc/controller/hardware_planner/mixed_navigation.h` | Stati, configurazione e utility hardware. |
| `simulator/include/mvc/controller/hardware_planner/runner.h` | Stato persistente e telemetria del runner. |
| `simulator/src/controller/hardware_planner/mixed_navigation.cpp` | Frenet limitato, G2, sagoma, spazio osservato, criteri geometrici. |
| `simulator/src/controller/hardware_planner/parts/mixed_navigation.inc` | Macchina a stati, selezione, replan e guardia comandi. |
| `simulator/src/controller/hardware_planner/runner.cpp` | Inclusione della nuova parte hardware. |
| `simulator/src/controller/hardware_planner/parts/lifecycle.inc` | Reset, limiti cinematici, configurazione MPC e percezione hardware. |
| `simulator/src/controller/hardware_planner/parts/sensing_runtime.inc` | Dispatch esplicito, sensori, freschezza prima dell'invio e goal globale. |
| `simulator/src/controller/hardware_planner/parts/control.inc` | Inibizione riferimenti mancanti, riconoscimento rejoin e limiti comandi. |
| `simulator/src/controller/hardware_planner/parts/gap_detection.inc` | Esclusione dei target artificiali/euristiche compatte per il nuovo preset. |
| `simulator/src/controller/hardware_planner/parts/gap_workflow.inc` | Bypass del vecchio conteggio Mixed e memoria del piano attraversato. |
| `simulator/src/controller/hardware_planner/parts/exploration.inc` | Riutilizzo del matcher iniziale nella nuova modalità hardware. |
| `simulator/src/controller/hardware_planner/telemetry_controller.cpp` | Campi e stato Mixed nella telemetria esistente. |
| `simulator/src/controller/hardware_app/parts/options.inc` | Preset, file missione e opzione di validazione senza dispositivi. |
| `simulator/src/controller/hardware_app/parts/entrypoint.inc` | Configurazione dedicata dopo il profilo fisico car/tank. |
| `simulator/src/controller/hardware_app/parts/stream_config.inc` | Protezione della missione hardware da sostituzioni remote del preset. |
| `simulator/src/controller/hardware_app/parts/trial_recording.inc` | Colonne Mixed nel recorder CSV esistente. |
| `simulator/src/tests/integration/hardware_mixed_navigation_test.cpp` | Test matematici, percezione e stati senza movimento/simulatore. |
| `simulator/src/tests/integration/hardware_runtime_settings_test.cpp` | Controlli delle nuove opzioni hardware. |

`planning.inc` e `trajectory.inc` sono stati analizzati e rimangono invariati: il dispatch hardware sceglie la macchina a stati dedicata prima di entrarvi; i vecchi runtime continuano a usarli come prima.

La [figura della scansione](../reports/mixed_visibility_20261017/visibility_163834.png) suggerisce che la lacuna verso destra/dietro coincida con un’interruzione dei ritorni fra due tratti del perimetro. È un’inferenza dalla point cloud, non una prova di una schermatura o di un varco fisico. Controllare quel tratto e, se manca una superficie rilevabile, completarlo con una barriera stabile e opaca visibile all’altezza del LiDAR, fuori dalla sagoma del robot. Acquisire poi una nuova reference e verificarla offline. Questo intervento aggiunge misure reali; non dichiara libero spazio non osservato.


## Aggiornamento 18 ottobre 2026: CSV raw analizzato

Il raw della cattura `20261017_160242_473086656`, ora disponibile sul PC, è stato verificato con la CLI offline. La conversione Mixed conserva tutti i 2.582 ritorni, senza scarti per qualità, range minimo o sagoma. I 10 ritorni oltre 2,5 m sono già conservati dal planner come raggi limitati alla portata: il filtro di range del matcher non è la causa dello stop. Nessun ritorno raw appare fra 117 e 123° nelle dieci scansioni. La migliore coppia recente attorno al punto bloccante è separata da 7,50°, contro 3,15° ammessi. Il controllo restituisce `unknown_footprint` senza aprire dispositivi.

Restano 13 campioni non verificati del margine sulle ultime cinque scansioni: 8 posteriori destri e 5 anteriori destri. Occorre controllare entrambe le direzioni, non soltanto la prima riportata nel log. Il raw esportato non contiene i nodi a distanza zero, quindi non dimostra da solo se manchino ritorni fisici o nodi del flusso seriale. Prima della successiva missione controllare visibilità e continuità delle barriere all'altezza del laser, acquisire una nuova reference stazionaria e verificarla offline. Non aumentare la soglia né ridurre il margine per forzare il movimento.

Nuova regressione sulle cinque scansioni raw originali (1.293 punti), build locale dell'harness e `--mixed-options`: PASS. Nessuna modifica ai controlli di movimento o al simulatore. [Analisi completa e comandi della cattura](../reports/mixed_raw_20261018/Analisi_raw_e_prossimo_controllo.md).
