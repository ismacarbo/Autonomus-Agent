# Package map

Il codice runtime segue MVC e, dentro ciascun livello, e' separato per
responsabilita'. Gli header pubblici replicano la stessa organizzazione sotto
`simulator/include/mvc`.

```text
controller/
  application/          opzioni applicative e report simulati
  hardware_app/         composition root del runner hardware
  hardware_io/          seriale, RSP-v1, RPLidar, posa esterna e robot bridge
  hardware_planner/     orchestrazione, percezione e controllo del robot reale
  simulation_planner/   orchestrazione del planner nel simulatore
  slam/                 adattatore opzionale verso slam_toolbox

model/
  navigation/           dinamica, EKF, MPC e conversione ruote/PWM
  perception/           scansione LiDAR simulata e point map
  reporting/            immagine SLAM associata ai report
  route/                progressione e proiezione dei percorsi structured
  world/                preset, scala, raycast e collisioni

view/
  canvas/               rendering condiviso di mondo e robot
  live_stream/          protocollo della vista hardware remota
  simulator_app/        GUI, pannelli, editor e composition root SDL/ImGui

tests/
  unit/                 contratti dei Model
  integration/          roundtrip UDP e world stream
  hardware_protocol/    smoke standalone del protocollo hardware
```

I file `parts/*.inc` sono partizioni di implementazione incluse, nell'ordine
esplicito, dal piccolo file `.cpp` del package. Non sono translation unit
autonome. Questa scelta rende navigabili i vecchi monoliti senza cambiare
linkage, ordine del codice o comportamento, dettaglio particolarmente
importante per il runner hardware.

La descrizione approfondita dei flussi runtime e dei principali simboli e' in
`documentation/Guida_Tecnica_Punti_Principali_Codice_20260726.md`.
