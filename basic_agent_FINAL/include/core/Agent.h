#pragma once
#include "core/Controller.h"
#include "core/Planner.h"
#include "net/UDPSocket.h"

class Agent {
 public:
  Agent(const char* host, int port);
  void run();

  ManoeuvreMsg process(const ScenarioMsg& s);

 private:
  UDPSocket socket;
  Planner planner;
  Controller controller;
};