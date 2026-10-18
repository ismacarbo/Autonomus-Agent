# Mixed hardware: correzione dopo la prova 20:10 del 18 ottobre 2026

La ricerca a lunghezza fissa è stata riprodotta sulla scena finale e corretta. Gli stessi due gate e le stesse cinque scansioni danno **0 accettati / 140 valutazioni** con espansioni minime da 25 cm e **18 accettati / 644 valutazioni** con la ricerca adattiva, mantenendo l'orizzonte massimo a 0,75 m e la curvatura a 1,80 m⁻¹. I segmenti selezionati sono lunghi 6,25 e 11,25 cm; comprendono una verifica separata della continuazione di arresto. Non sono attraversamenti completi né una dimostrazione della riuscita della missione.

## Cosa ha fermato il robot

La reference iniziale e il matching funzionano. Il bundle contiene 355 campioni fino a 51,888 s, mentre il riepilogo del runner incollato contiene 440 campioni: sono finestre diverse. Su questo PC non erano presenti il CSV/log originali del runner della nuova prova. La ricostruzione usa il JSON e gli endpoint LiDAR del bundle `thesis_hardware_mixed_mixed_hardware_road_gate_gui_snapshot_20261018_201108_709`, senza inventare raggi mancanti.

| Tempo | Evidenza |
|---|---|
| 2,35 s | Selezione di un gate e primi comandi; la modalità rilassata aveva effettivamente sbloccato la partenza. |
| 2,80 s | PWM sinistro 0, destro 119: impulso di avviamento su una sola ruota. |
| 3,10 s | `encoder_slip_guard_active=1`, stop temporaneo etichettato genericamente `localization_unreliable_reset_required`. |
| 3,99 s | Velocità encoder 0,216 m/s contro target 0,04 m/s; frenata PWM 0/0. |
| 6,64 s | `local_replan_settling`, poi arresto persistente. |
| 7,08 s | `no_verified_gate_reference`; il gate resta bloccato fino al timeout. |
| 32,50 s | Riacquisizione dopo timeout; poi `no_verified_gate`. |

Durante il successivo arresto IMU, encoder, LiDAR e SLAM restano disponibili. Non è un guasto persistente del matching o della comunicazione. La velocità encoder è quantizzata (38 tick/giro): 4 tick in 100 ms corrispondono a circa 0,216 m/s. Non è una misura continua della velocità al suolo; l'operatore ha comunque confermato scatti e rotazioni brusche.

Due difetti concorrono al comportamento osservato:

1. **Ricerca locale con passo minimo artificiale.** Le tre espansioni avevano ciascuna lunghezza `local_horizon_m / 3`, quindi almeno 25 cm nella configurazione utilizzata. Dopo l'avanzamento, tutte le prime espansioni fallivano e nessuna seconda primitiva poteva essere esplorata. Sulla scena finale il gate superiore dà 51 rifiuti cinematici e 19 per apertura; quello inferiore 63 cinematici, 6 per apertura e 1 per ingombro occupato. Un segmento più corto può ancora essere arrestabile.
2. **Attuazione discontinua a bassa velocità.** Il limite sul target non limitava l'avanzamento accumulato durante gli impulsi PWM. Il recupero di stallo poteva comandare una ruota a 110 PWM canonici, circa 119 sul driver destro calibrato, tenendo ferma l'altra. Dopo una frenata, un singolo intervallo encoder a zero consentiva di ripartire subito, pur avendo già percorso più distanza di quella richiesta. I dati non permettono di attribuire tutta la differenza di posa al solo PWM: sono presenti anche correzioni SLAM.

Figure: [velocità/PWM/yaw registrati](analysis_20261018/mixed_201108/measured_motion.png), [scena finale e segmenti verificati](analysis_20261018/mixed_201108/stopped_scene.png).

## Correzioni implementate

- **Lunghezza adattiva:** se nessuna delle sette curvature produce un'espansione valida, la lunghezza viene dimezzata fino a `local_min_segment_m`, inizialmente 0,05 m. Si mantengono le espansioni lunghe dove funzionano. Tre livelli, dodici nodi mantenuti e al massimo quattro lunghezze per nodo limitano il costo: massimo 560 estensioni di primitive più 63 G2 per gate. Non sono traiettorie fisiche indipendenti.
- **Riacquisizione:** un gate senza nuova reference locale viene rilasciato subito; il ciclo successivo deve avere una scansione fresca e veicolo assestato. Non si attende più il timeout di 30 s. Se tutti i gate falliscono, viene tentata anche la ricerca locale di osservazione/rientro già usata in assenza di gate.
- **Partenza graduale in EXPERIMENTAL_RELAXED:** il car con controllo PWM host usa una rampa di 35 PWM canonici/s, prima della trasformazione calibrata dei driver. Non usa più il recupero con impulso su una sola ruota e arresto dell'altra. Rimane il tetto di avviamento già configurato e lo stop per stallo persistente. Il minimo PWM calibrato viene preservato anche dopo lo slew finale.
- **Regolazione dell'avanzamento:** si confronta la distanza encoder accumulata con l'integrale della velocità richiesta. Oltre un tick di anticipo entrambe le ruote vengono mantenute a zero fino al recupero del margine; un singolo pacchetto a zero non cancella il debito. Gli intervalli duplicati non vengono conteggiati due volte e lo spostamento durante uno stop viene contabilizzato. Non si accumula credito illimitato mentre il veicolo è fermo.
- **Diagnostica:** `mixed_local_reference_end` distingue errore geometrico, distanza dalla reference e fine segmento; `mixed_crawl_pacing` registra le attese del regolatore. Lo stop transitorio per encoder/slittamento ora si chiama `encoder_slip_guard`, distinto dagli errori di localizzazione. La lunghezza selezionata compare nel log dei candidati.

La modalità STRICT conserva il percorso di selezione e attuazione precedente. La nuova regolazione PWM è limitata a Mixed EXPERIMENTAL_RELAXED, car-like, controllo host; non modifica il controllo velocità MCU. Non sono cambiati simulatore, GUI, firmware o profilo di calibrazione. Il submodule con la patch preesistente rimane intatto.

Rimangono ingombro e margini, collision checking, coda di arresto, limiti di curvatura/sterzo, interblocchi sensori, emergency stop, watchdog, frenata per overspeed e timeout di stallo. I parametri di arresto e della rampa non costituiscono una nuova calibrazione fisica: la regolarità del moto a 0,04 m/s va verificata sul robot.

## Verifiche

- Build Release x86_64 del runner hardware, acquisitore LiDAR e harness offline riuscita.
- Cinque test CTest hardware offline passati; parser/scansioni `--mixed-options` passati.
- `--actuation-guards` e `--structured-mixed` passati su osservazioni iniettate: nessun dispositivo o simulation planner avviato.
- Regressione della scena finale con 1.228 endpoint in cinque scansioni. Include inflazione del moto ricavata dalla telemetria esportata, validazione della sagoma, limiti delle primitive e coda di arresto. I gate sono riestratti una volta e condivisi fra le due ricerche.
- Test del nuovo controllo: rampa calibrata, assenza dell'impulso su una ruota, sequenza encoder quattro tick → zero, pacchetti duplicati, ripartenza dopo assestamento e stop per stallo conservato.
- La precedente regressione della scena mattutina rimane valida: 359 valutazioni accettate su cinque gate con la nuova ricerca, contro zero STRICT. Il suo confronto precedente aveva 259 accettati; gli input e i limiti fisici sono conservati.

I log e le curve numeriche sono in `documentation/analysis_20261018/mixed_201108/`. [Manifest delle fonti](analysis_20261018/mixed_201108/fixture_manifest.json), [test](analysis_20261018/mixed_201108/ctest.log), [regressione](analysis_20261018/mixed_201108/regression.log). Rigenerazione del fixture: `python3 tools/extract_mixed_stop_fixture.py` con i file originali in `reports/`.

## Applicazione sul Pi e prossima prova

Il percorso corretto del repository sul Pi, confermato dal terminale dell'operatore, è **`~/Desktop/Autonomus-Agent`**. L'errore `cd ~/Autonomus-Agent` era successivo alla prova e non ne ha causato lo stop.

Le modifiche sono locali; `git pull` da solo non le trasferisce finché non vengono pubblicate. È disponibile il pacchetto locale `reports/mixed_stop_fix_20261018_201108.patch`. Il bundle mostra il Pi all'indirizzo Tailscale `100.87.133.15` e l'utente del terminale è `ismadev`.

Dal PC, per trasferire la patch senza pubblicare commit:

```sh
cd /home/isma/Desktop/Autonomus-Agent
scp reports/mixed_stop_fix_20261018_201108.patch ismadev@100.87.133.15:/tmp/
```

Sul Pi, a runner arrestato:

```sh
cd ~/Desktop/Autonomus-Agent
git apply --check /tmp/mixed_stop_fix_20261018_201108.patch
git apply /tmp/mixed_stop_fix_20261018_201108.patch

cmake -S . -B build-hardware-mixed \
  -DTHESIS_BUILD_SIM_GUI=OFF -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release
cmake --build build-hardware-mixed -j 2 --target \
  thesis_robot_runner thesis_lidar_start_matcher \
  thesis_hardware_mixed_relaxed_test thesis_hardware_mixed_navigation_test \
  thesis_start_matching_runtime_smoke thesis_slam_observation_smoke \
  thesis_hardware_runtime_settings_smoke
ctest --test-dir build-hardware-mixed --output-on-failure \
  -R '^(thesis_hardware_mixed_navigation_offline|thesis_hardware_mixed_relaxed_offline|thesis_start_matching_runtime|thesis_slam_observation|thesis_trial_recording)$'
build-hardware-mixed/simulator/thesis_hardware_runtime_settings_smoke --mixed-options
```

Applicare la patch una sola volta; se `git apply --check` fallisce, non forzare un reset del repository. Una patch già applicata o un'altra revisione richiedono il controllo del diff.

Ripartire dallo start iniziale con una nuova cattura LiDAR, non dalla posa avanzata vicino all'ostacolo. Utilizzare la stessa procedura di [avvio manuale](Hardware_Mixed_Experimental_Relaxed.md), configurazione `config/mixed_experimental_relaxed.conf` e target 0,04 m/s. Il log deve mostrare `min_segment=0.05 crawl_pwm_rise_per_s=35`. Il goal resta `(1.30,0)` nel frame della nuova reference. La GUI esistente e il bridge SLAM rimangono compatibili.

Per la prossima analisi copiare anche **CSV e log del runner**, insieme a configurazione, reference/raw di cattura e bundle GUI: permettono di distinguere le cause esatte dei rifiuti e l'istante di invalidazione. La patch non è stata eseguita sul robot e nessun comando di movimento è stato avviato durante lo sviluppo.
