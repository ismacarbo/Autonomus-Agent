#pragma once
#include "../model/Msgs.h"

class Planner {
 public:
  Manoeuvre plan(const ScenarioMsg& s);

 private:
  Manoeuvre buildLongitudinal(const ScenarioMsg& s);
  void buildLateral(const ScenarioMsg /*s*/) { /* TODO in futuro */ }
};
