#include "../../include/core/Controller.h"

#include <algorithm>

double Controller::computeAccel(const VehicleState& x, const Manoeuvre& /*m*/) {
  // simple acceleration if v<20 m/s
  const double v_ref = 20.0;
  const double err = v_ref - x.v;

  integ_v += err * 0.05;  // DT fixed as main
  const double deriv = (err - prev_err_v) / 0.05;

  double req_acc = kp_v * err + ki_v * integ_v + kd_v * deriv;
  req_acc = std::clamp(req_acc, minAcc, maxAcc);
  prev_err_v = err;

  // anti-windup
  if ((req_acc == minAcc && err < 0) || (req_acc == maxAcc && err > 0)) {
    integ_v -= err * 0.05;
  }
  return req_acc;
}