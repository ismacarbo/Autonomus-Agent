# Hardware planner package

`runner.cpp` e' il punto di compilazione del package e include queste
responsabilita' nell'ordine originale:

| Partizione | Responsabilita' |
| --- | --- |
| `parts/support.inc` | helper geometrici, cinematici e occupancy grid locali |
| `parts/lifecycle.inc` | connessione, reset, configurazione, gate e stato planner |
| `parts/planning.inc` | invocazione del planner bio-inspired |
| `parts/localization.inc` | encoder/IMU, EKF e correzione LiDAR |
| `parts/gap_detection.inc` | estrazione, filtro e tracking dei varchi LiDAR |
| `parts/gap_workflow.inc` | macchina a stati unstructured/mixed |
| `parts/trajectory.inc` | riferimento locale e traiettoria selezionata |
| `parts/control.inc` | follower, ruote, PWM e safety stop |
| `parts/sensing_runtime.inc` | metriche LiDAR, step e ciclo run |

L'interfaccia pubblica e' in
`simulator/include/mvc/controller/hardware_planner/runner.h`; la telemetria e'
implementata da `telemetry_controller.cpp`.
