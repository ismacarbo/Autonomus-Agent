# Diagnostica hardware motori, encoder e IMU

Questo test serve a determinare se il mancato inseguimento dei gate dipende dalla
catena di attuazione e misura. Esclude completamente planner, LiDAR, clotoide e
MPC e usa il firmware RSP gia installato sulla car: **non occorre flashare il
microcontrollore**.

Il test invia PWM diretti e registra:

- comando logico sinistro e destro;
- PWM target e PWM applicati dal microcontrollore;
- tick encoder sinistro e destro;
- yaw e yaw-rate IMU;
- flag ed errori del controller.

## Build

```sh
cmake --build build-ninja --target thesis_drivetrain_diagnostics
```

## Test 1: motori ed encoder con ruote sollevate

Bloccare la car su un supporto stabile, con tutte le ruote libere. Avviare:

```sh
./scripts/run_car_drivetrain_diagnostics.sh /dev/ttyACM0 --suite encoders
```

La sequenza e controllata dall'operatore:

1. robot fermo: nessun encoder deve avanzare e lo yaw non deve derivare;
2. `LEFT_ONLY`: devono girare soltanto le ruote fisiche sinistre e deve avanzare
   soltanto l'encoder logico sinistro;
3. `RIGHT_ONLY`: devono girare soltanto le ruote fisiche destre e deve avanzare
   soltanto l'encoder logico destro.

Dopo ogni comando monolaterale il programma chiede quale lato fisico sia stato
osservato. In questo modo il report distingue un encoder fermo da uno swap dei
canali. Ogni moto dura di default 1,25 s; `CTRL+C` invia l'arresto.

## Test 2: encoder e IMU a terra

Usare un pavimento libero e avviare:

```sh
./scripts/run_car_drivetrain_diagnostics.sh /dev/ttyACM0 --suite imu
```

Le fasi sono:

1. `IMU_STATIONARY`: yaw stabile a motori fermi;
2. `STRAIGHT`: PWM uguale, tick simili e variazione di yaw contenuta;
3. `POSITIVE_YAW`: destra piu veloce della sinistra, yaw IMU positivo;
4. `NEGATIVE_YAW`: sinistra piu veloce della destra, yaw IMU negativo.

Ogni fase parte solo dopo `INVIO`, dura automaticamente 1,25 s e poi arresta i
motori. Per eseguire entrambe le suite nella stessa sessione:

```sh
./scripts/run_car_drivetrain_diagnostics.sh /dev/ttyACM0 --suite all
```

Parametri prudenti di default: PWM 100 e differenziale 25. Possono essere
modificati, ad esempio:

```sh
./scripts/run_car_drivetrain_diagnostics.sh /dev/ttyACM0 \
  --suite imu --pwm 110 --turn-delta 30 --duration 1.0
```

## Risultati

La cartella `reports/` riceve tre file con prefisso
`thesis_hardware_drivetrain_diagnostic_`:

- CSV con tutti i campioni;
- JSON con risultati e cause automatiche;
- Markdown con la tabella riassuntiva.

Le cause principali sono esplicite, ad esempio:

- `left_encoder_did_not_move`: encoder sinistro assente durante `LEFT_ONLY`;
- `encoder_channels_appear_swapped`: risponde il canale opposto;
- `left_side_stalled`: il lato sinistro non avanza sotto carico;
- `imu_yaw_sign_is_wrong`: segno IMU contrario al differenziale comandato;
- `imu_yaw_response_is_too_small`: la car riceve il differenziale ma non ruota;
- `severe_encoder_imbalance`: risposta dei due lati incompatibile con moto dritto.

Se una fase monolaterale fallisce, non ha senso modificare l'MPC: prima va
corretta la mappatura o la misura hardware. Se le fasi monolaterali passano ma
falliscono quelle a terra, il problema e piu probabilmente soglia PWM sotto
carico, attrito, alimentazione o controllo open-loop.
