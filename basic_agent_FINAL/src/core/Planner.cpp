#include "core/Planner.h"

#include <algorithm>
#include <cmath>

extern "C" {
#include "primitives.h"
}

Manoeuvre Planner::plan(const ScenarioMsg& s) { return buildLongitudinal(s); }

Manoeuvre Planner::buildLongitudinal(const ScenarioMsg& s) {
  Manoeuvre out{};
  const double minAcc = -10.0, maxAcc = 5.0;
  const double acc = std::max(minAcc, std::min(s.vehicleState.a, maxAcc));
  const double vel = s.vehicleState.v;

  TLParam P{};
  const double lookahead = std::max(50.0, s.vehicleState.v * 5.0);

  double coeffsT2[6]{}, coeffsT1[6]{};
  double v2 = 0, T2 = 0, v1 = 0, T1 = 0;

  double pass_dist = 0.0, stop_dist = 0.0;
  if (s.trafficLight.nr != 0) {
    pass_dist = s.trafficLight.dist;
    stop_dist = s.trafficLight.dist - P.Ss * 0.5;
  }

  if (s.trafficLight.nr == 0 || pass_dist >= lookahead) {
    pass_primitive(vel, acc, lookahead, s.vehicleState.requestedCruising, s.vehicleState.requestedCruising, 0.0, 0.0,
                   coeffsT2, &v2, &T2, coeffsT1, &v1, &T1);
    out.sf = lookahead;
    out.vf = v2;
    out.tf = T2;
    out.af = 0.0;
    for (int i = 0; i < 6; ++i) out.coeff[i] = coeffsT2[i];
    return out;
  }

  const auto W = TrafficLightLogic::compute(s, P);

  if (pass_dist < P.Ss && W.phase == 1) {
    pass_primitive(vel, acc, lookahead, s.vehicleState.requestedCruising, s.vehicleState.requestedCruising, 0.0, 0.0,
                   coeffsT2, &v2, &T2, coeffsT1, &v1, &T1);
    out.sf = lookahead;
    out.vf = v1;
    out.tf = T1;
    for (int i = 0; i < 6; ++i) out.coeff[i] = coeffsT1[i];
    return out;
  }

  pass_primitive(vel, acc, pass_dist, P.Vmin, P.Vmax, W.Tmin, W.Tmax, coeffsT2, &v2, &T2, coeffsT1, &v1, &T1);

  if (v1 == 0) {
    if (stop_dist > 0) {
      double bests = 0, bestT = 0;
      stop_primitive(vel, acc, stop_dist, out.coeff, &bests, &bestT);
      out.sf = bests;
      out.vf = 0;
      out.tf = bestT;
      out.af = 0;
    } else {
    }
    return out;
  }

  const bool chooseT1 = std::fabs(coeffsT2[3]) > std::fabs(coeffsT1[3]);
  const double* bestC = chooseT1 ? coeffsT1 : coeffsT2;
  out.sf = pass_dist;
  out.vf = chooseT1 ? v1 : v2;
  out.tf = chooseT1 ? T1 : T2;
  for (int i = 0; i < 6; ++i) out.coeff[i] = bestC[i];
  return out;
}
