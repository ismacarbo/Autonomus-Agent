#include "../../include/core/Planner.h"

extern "C" {
#include "primitives.h"
}

#include <algorithm>
#include <cmath>

Manoeuvre Planner::plan(const ScenarioMsg& s) { return buildLongitudinal(s); }

Manoeuvre Planner::buildLongitudinal(const ScenarioMsg& s) {
  Manoeuvre out;
  const double minAcc = -10.0, maxAcc = 5.0;
  const double acc = std::clamp(s.vehicleState.a, minAcc, maxAcc);
  const double vel = s.vehicleState.v;

  // simple lookahead
  const double lookahead = std::max(50.0, s.vehicleState.v * 5.0);
  double coeffsT2[6]{}, coeffsT1[6]{};
  double v2 = 0, T2 = 0, v1 = 0, T1 = 0;

  if (s.trafficLight.state == "unknown" || s.trafficLight.dist >= lookahead) {
    pass_primitive(vel, acc, lookahead, s.vehicleState.v, s.vehicleState.v, 0, 0, coeffsT2, &v2, &T2, coeffsT1, &v1,
                   &T1);
    out.sf = lookahead;
    out.vf = v2;
    out.tf = T2;
    out.af = 0.0;
  }
}