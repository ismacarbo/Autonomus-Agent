#pragma once
#include "../model/Msgs.h"

struct TLParam {
  double Ss{5.0};     // safety space [m]
  double Sin{10.0};   // min space to stop [m]
  double Vmin{3.0};   // min speed to consider [m/s]
  double Vmax{15.0};  // max speed to consider [m/s]
};

struct TLWindows {
  double Tmin{0.0};
  double Tmax{0.0};
  int phase{0};
};

struct TrafficLightLogic {
  static TLWindows compute(const ScenarioMsg& s, const TLParam& p);
};