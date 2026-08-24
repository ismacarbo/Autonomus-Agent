# Stato hardware unstructured con firmware PWM legacy — 2026-08-18

## Run analizzate

- `thesis_hardware_unstructured_manual_gate_editor_gui_bundle_20260817_235211_151`
- `thesis_hardware_unstructured_manual_gate_editor_gui_manual_20260817_235326_607`

Il matcher LiDAR iniziale non e' la causa del fallimento. Le tre acquisizioni
nella posa di riferimento sono state accettate con RMSE circa 1,3 mm, inlier
ratio 1, confidenza circa 0,986 e offset nullo. Il controllo negativo ha
stimato uno spostamento di 0,337 m e lo ha rifiutato correttamente con exit code
3.

## Diagnosi dei due report

Entrambi i report dichiarano `mcu_velocity_closed_loop: false`: il PID del
firmware 1.3 non e' attivo e non puo' avere causato il problema.

Nel primo run la ruota sinistra produce tick in 28 campioni, la destra in 118;
nel secondo sono 21 contro 133. Nei cicli in cui il planner richiede almeno 60
PWM alla ruota sinistra, questa rimane senza nuovi tick rispettivamente in 93 e
116 campioni. La destra continua invece a muoversi. Il controllo globale di
stallo osservava la velocita' massima delle due ruote: la ruota destra in moto
mascherava quindi quella sinistra ferma e `stall_boost_active` non si attivava.

La conseguenza e' una falsa traslazione encoder: la posa arriva a x=1,85 m e
x=1,70 m in un'arena larga solo 1,20 m. La point cloud, i gate persistenti e la
clotoide vengono trasformati con questa posa deformata; il gate resta
selezionato ma non puo' essere attraversato geometricamente.

Il report conteneva inoltre un gate finale a `(1,76, 1,00)` pur avendo bounds
`[0, 1,20]`. Il mondo Manual Editor veniva trasmesso con lista gate vuota, ma
la deserializzazione richiamava `set_gate_behavior()` e ricreava il vecchio
template non normalizzato. Il gate non era la causa primaria dello slittamento,
ma rendeva scena e report incoerenti.

## Correzioni implementate

Le modifiche sono limitate alla car in `Unstructured Gates` con arena non
superiore a 1,25 m. Structured, Mixed e tank non usano questa nuova politica.

1. Il rilevamento di stallo per ruota usa direttamente delta encoder e periodo
   encoder anche quando la readiness dell'odometria non ha ancora completato
   la propria streak.
2. La ruota ferma riceve un impulso indipendente a 110 PWM per due cicli. Il
   minimo normale resta 70 PWM: 110 e' break-away, non cruise continuo.
3. Se una ruota gira, l'altra e' quasi ferma e l'IMU non conferma la rotazione
   encoder, la traslazione odometrica viene attenuata e lo yaw-rate viene preso
   dall'IMU finche' entrambe le ruote recuperano trazione.
4. Il footprint stimato non puo' continuare a uscire dai bounds del mondo.
5. La distanza di arresto LiDAR della car unstructured compatta passa da 0,16
   a 0,19 m per includere offset LiDAR, parte anteriore del corpo e un periodo
   di comando PWM.
6. La ricerca resta ad arco in avanti. Solo dopo 1,2 s di blocco frontale viene
   eseguito un breve arco in retromarcia verso il lato piu' libero; non viene
   riabilitata la rotazione indefinita sul posto.
7. Il round-trip di un Manual Editor senza gate non puo' piu' ricreare un gate
   template. E' presente un test di regressione dedicato.
8. Ogni campione JSON/CSV ora include `stall_boost_active`,
   `left_wheel_stall_cycles`, `right_wheel_stall_cycles` ed
   `encoder_slip_guard_active`.
9. Il protocollo live sale alla versione 11: runner Pi e GUI workstation devono
   essere ricompilati dallo stesso commit.
10. E' stata riparata la dichiarazione spezzata di `RSP_MSG_HEARTBEAT_CMD` nel
    sorgente firmware 1.3. Non e' necessario flasharlo per provare il percorso
    PWM legacy.

## Verifica software

- build completa riuscita con GCC 15;
- CTest: 7/7 passati;
- tutti i sei preset unstructured deterministici della car, Ideal e
  Calibrated: 12/12 `goal_reached`;
- Structured Validation e Circle, Ideal e Calibrated: 4/4 `goal_reached`;
- Mixed Hardware, Ideal e Calibrated: 2/2 `goal_reached`;
- Closed Obstacle Road Ideal: `goal_reached`;
- Closed Obstacle Road Calibrated raggiunge ancora il limite di 4000 step. E'
  una regressione preesistente del caso mixed calibrated e non dipende dal
  ramo unstructured/PWM modificato qui.

Il runner hardware sintetico del preset Hardware Lab non e' un criterio di
goal affidabile per il Manual Editor reale: usa gli ostacoli virtuali del
preset, mentre il percorso hardware elimina quegli ostacoli e ricostruisce le
aperture dal LiDAR. Ha comunque verificato che il nuovo safety stop interviene
prima di continuare ad avanzare contro un ostacolo. La validazione conclusiva
richiede una nuova acquisizione fisica Manual Editor.

## Ordine della prossima prova hardware

Sulla workstation e sul Raspberry Pi, dopo lo stesso `git pull`, ricompilare:

```bash
cmake --build build-ninja --target thesis_robot_runner thesis_planner_sim -j4
```

Avviare la GUI sulla workstation, poi verificare lo start sul Pi:

```bash
./scripts/run_car_lidar_start_matcher.sh /dev/ttyUSB0 \
  --reference datasets/localization/car_unstructured_hardware_lab_start.csv \
  --scan-count 8 \
  --confirmations 3 \
  --max-start-offset 0.03 \
  --max-start-yaw-deg 3
```

Procedere soltanto con `start_pose_accepted=1` ed exit code 0. Avviare quindi
il runner senza `--mcu-wheel-pid`, usando il Manual Gate Editor della GUI:

```bash
./build-ninja/simulator/thesis_robot_runner \
  --controller-port /dev/ttyACM0 \
  --lidar-port /dev/ttyUSB0 \
  --scenario unstructured \
  --unstructured-map custom \
  --vehicle-model car \
  --pose-fusion auto \
  --stream-host 100.66.27.57 \
  --stream-port 9559
```

Nel nuovo report devono risultare:

- `mcu_velocity_closed_loop: false`;
- nessun gate statico fuori dai bounds nella scena;
- impulsi `stall_boost_active=1` solo quando una ruota non produce tick;
- riattivazione dei tick della ruota ferma dopo il boost;
- `encoder_slip_guard_active=1` soltanto nei campioni di slittamento grave;
- posa e point cloud che restano coerenti nell'arena 1,20 x 1,20 m;
- una clotoide mantenuta fino all'attraversamento del gate.

Per la prima prova e' consigliato fermarsi dopo il primo gate valido e salvare
subito il bundle, prima di eseguire una missione lunga.

## Aggiornamento drivetrain 2026-08-24 (firmware 1.4)

Il mapping fisico definitivo e' `RF=A2`, `RR=A0`, `LF=A1`, `LR=A3` con
`PWMA=destra` e `PWMB=sinistra`. Le prove one-side sono tutte passate e il
segno BNO080 e' stato validato in entrambe le direzioni.

Sweep con ruote sollevate, durata 1,25 s:

| PWM | tick sinistra | tick destra |
|---:|---:|---:|
| 70 | 45 | 38 |
| 85 | 57 | 47 |
| 100 | 67 | 57 |
| 115 | 70 | 66 |
| 130 | 78 | 75 |

Le regressioni sono `L=0,5266667*PWM+10,7333333` e
`R=0,6200000*PWM-5,4000000`. Per ottenere a destra la risposta canonica
sinistra si applica quindi:

```text
PWM_R = 0,8494623656 * PWM_base + 26,0215053763
```

La prova a terra ha misurato `+2,18 deg` con `60/100` e `-31,97 deg` con
`100/60`; l'osservazione fisica ha confermato che la prima virata era davvero
debole. Il problema non e' il segno IMU ma l'autorita' fortemente asimmetrica
del drivetrain sotto carico.

Il profilo car 1.6 applica ora la calibrazione affine dopo la somma di
feed-forward e feedback. Il runner chiude inoltre un PI esterno sullo yaw-rate
BNO080 anche quando gli encoder sono validi, poi lascia al PI per-ruota il
compito di realizzare le velocita' corrette. In modalita' unstructured le due
ruote restano forward-only; la massima autorita' differenziale del gate e'
70 PWM per yaw positivo e 40 PWM per yaw negativo. I vecchi scale identificati
con A0/A1 incrociati sono stati azzerati.

Il PID velocita' interno al micro resta disattivato: prima di usare
`--mcu-wheel-pid` devono essere identificati separatamente `FF`, `KP` e `KI`.
