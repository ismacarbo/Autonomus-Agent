#pragma once

#include <utility>

#include "mvc/model/navigation/vehicle_dynamics.h"

namespace thesis_sim::mvc::model {

// Pure conversion functions used between planner/controller outputs and the
// two physical motor channels.  Keeping these functions free of transport and
// planner state makes the PWM contract independently testable.
double wheel_speed_from_pwm_estimate(int pwm,
                                     double channel_scale,
                                     const VehicleGeometry& geometry);

std::pair<double, double> wheel_speeds_from_body(double speed_mps,
                                                 double yaw_rate_rad_s,
                                                 double half_track_m);

int wheel_speed_to_pwm(double wheel_speed_mps,
                       double channel_scale,
                       int min_effective_pwm,
                       int max_pwm,
                       double speed_to_pwm_gain,
                       double speed_to_pwm_bias);

int clamp_motion_pwm_band(int pwm, int min_pwm, int max_pwm);
int slew_limit_pwm(int previous_pwm, int target_pwm, int max_delta);

void scale_motion_pwm_pair_to_full_scale(int max_pwm,
                                         int* pwm_left,
                                         int* pwm_right);

void apply_start_motion_boost(int min_pwm,
                              int* pwm_left,
                              int* pwm_right);

void enforce_forward_tracked_turn_authority(double target_yaw_rate,
                                            int outer_min_pwm,
                                            int inner_min_pwm,
                                            int inner_max_pwm,
                                            int* pwm_left,
                                            int* pwm_right);

}  // namespace thesis_sim::mvc::model
