#include "../core/TrafficLightLogic.h"

#include <algorithm>

TLWindows TrafficLightLogic::compute(const ScenarioMsg& s, const TLParam& p) {
  TLWindows w{};
  if (s.trafficLight.nr == 0) {
    return w;
  }

  const double Vmin = std::max(p.Vmin, 1.0);
  const double Ts = p.Ss / Vmin;    // time to cover safety space at min speed
  const double Tin = p.Sin / Vmin;  // time to cover min space to stop at

  if (s.trafficLight.currentState == TLState::GREEN && s.trafficLight.next1 == TLState::YELLOW) {
    // green phase
    w.Tmin = std::max(0.0, s.trafficLight.t1 - Ts);
    w.Tmax = std::max(0.0, s.trafficLight.t1 - Tin);
    w.phase = 1;
  } else if (s.trafficLight.currentState == TLState::YELLOW && s.trafficLight.next1 == TLState::RED) {
    // yellow phase
    w.Tmin = 0.0;
    w.Tmax = std::max(0.0, s.trafficLight.t2 - Tin);
    w.phase = 2;
  } else if (s.trafficLight.currentState == TLState::RED && s.trafficLight.next1 == TLState::GREEN) {
    // red phase
    w.Tmin = std::max(0.0, s.trafficLight.t2 + s.trafficLight.t3 - Ts);
    w.Tmax = std::max(0.0, s.trafficLight.t2 + s.trafficLight.t3 - Tin);
    w.phase = 3;
  } else {
    // unknown or unsupported phase
    w.Tmin = 0.0;
    w.Tmax = 0.0;
    w.phase = 0;
  }
  return w;
}
