#pragma once
#include "core/TrafficLightLogic.h"
#include "model/Msgs.h"

class Planner {
 public:
  Manoeuvre plan(const ScenarioMsg& s);

 private:
  Manoeuvre buildLongitudinal(const ScenarioMsg& s);
  Manoeuvre buildLateral(const ScenarioMsg& s);
};
