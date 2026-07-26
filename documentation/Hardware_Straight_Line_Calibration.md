# Calibrazione rettilinea hardware della robot car

Questo test esclude planner, LiDAR, MPC e PID. Il Raspberry Pi invia lo stesso
PWM diretto ai due lati tramite il protocollo RSP gia' usato dal runner e salva
tick encoder, distanza nominale, yaw IMU e PWM realmente applicato.

## Preparazione

1. Usare il firmware RSP gia' validato sulla car.
2. Scegliere un tratto libero e rettilineo.
3. Marcare a terra il punto iniziale del centro dell'asse del robot.
4. Tenere accessibili alimentazione e tastiera del Raspberry Pi.

Il test si arresta premendo `INVIO`, con `CTRL+C` oppure dopo il timeout di
sicurezza. Il watchdog del microcontrollore resta attivo.

## Build e avvio

```sh
cmake --build build-ninja --target thesis_straight_line_calibration
./scripts/run_car_straight_calibration.sh /dev/ttyACM0
```

I default della car sono PWM `90`, raggio `0.0327 m`, `360` tick/giro nominali
e timeout massimo di `20 s`. Per cambiare solo il PWM:

```sh
./scripts/run_car_straight_calibration.sh /dev/ttyACM0 --pwm 100
```

Quando il robot raggiunge il punto finale, premere `INVIO`. Misurare quindi la
distanza percorsa dallo stesso riferimento fisico usato all'inizio e inserirla
in metri. Il test genera CSV, JSON e Markdown nella cartella `reports/`.

## Interpretazione

Il fattore metrico calcolato e':

```text
fattore = distanza_fisica / distanza_encoder_nominale
```

La conversione metri/tick si moltiplica per questo fattore. Poiche' il raggio
fisico della car e' gia' misurato (`0.0327 m`), il valore da trasferire nel
profilo e' principalmente il numero effettivo di tick/giro calcolato dal test.
Il raggio equivalente viene mostrato solo come diagnostica. Il fattore non e'
direttamente un fattore di scala della mappa. Prima di restringere la strada
bisogna verificare anche:

- differenza percentuale tra tick sinistri e destri;
- variazione di yaw durante il comando uguale;
- eventuale encoder che resta fermo o conta in modo anomalo.

Una forte asimmetria indica che una singola misura di distanza non basta:
occorre prima controllare mappatura degli encoder, tick/giro e risposta dei due
lati. Solo dopo la calibrazione metrica si confronta l'ingombro della
centerline con lo spazio fisico e si decide l'eventuale riduzione della pista.
