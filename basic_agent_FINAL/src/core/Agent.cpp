#include "../../include/core/Agent.h"

#include <cstdio>

Agent::Agent(const char* host, int port) { socket.open(host, port); }

ManoeuvreMsg Agent::process(const ScenarioMsg& s) {
  Manoeuvre target = planner.plan(s);
  ManoeuvreMsg out{};
  out.accelCmd = controller.computeAccel(s.vehicleState, target);
  out.steerCmd = controller.computeSteer(s.vehicleState, target);
  return out;
}

void Agent::run() {
  ScenarioMsg s{};
  while (true) {
    if (!socket.receiveScenario(s)) break;
    ManoeuvreMsg cmd = process(s);
    socket.sendManoeuvre(cmd);
  }
  socket.close();
}
