#pragma once
#include "../model/Msgs.h"

class Controller {
 public:
  // PID longitudinal
  double computeAccel(const VehicleState& x, const Manoeuvre& m);

  // TODO: implement pure pursuit/stanley
  double computeSteer(const VehicleState& /*x*/, const Manoeuvre& /*m*/) { return 0.0; }

 private:
  // PID PARAMS
  // kp v: how much rects immediatly for the velocity error
  // ki v: accumulates the error in time to reset error at start
  // kd v: derivative
  // integ v: integrator state
  // prev err v: previous error
  double kp_v{0.02}, ki_v{1.0}, kd_v{0.0};
  double integ_v{0.0}, prev_err_v{0.0};

  // saturations of the actuators
  const double minAcc{-10.0}, maxAcc{5.0};
};