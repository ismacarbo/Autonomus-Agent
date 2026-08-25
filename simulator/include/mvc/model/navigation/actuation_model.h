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

// Applies a measured affine correction to the magnitude of an already
// computed PWM command while preserving stop and direction semantics.
int apply_signed_pwm_calibration(int pwm,
                                 double output_scale,
                                 double output_offset,
                                 int max_pwm);

int remove_signed_pwm_calibration(int calibrated_pwm,
                                  double output_scale,
                                  double output_offset,
                                  int max_pwm);

// Outer-loop correction used to turn an MPC/body yaw-rate request into the
// wheel-level yaw-rate request that compensates measured drivetrain slip.
double yaw_rate_target_with_feedback(double target_yaw_rate,
                                     double measured_yaw_rate,
                                     double yaw_error_integral,
                                     double proportional_gain,
                                     double integral_gain,
                                     double maximum_abs_yaw_rate);

double stabilize_yaw_rate_target(double requested_yaw_rate,
                                 double measured_yaw_rate,
                                 double yaw_error_integral,
                                 double proportional_gain,
                                 double integral_gain,
                                 double maximum_abs_yaw_rate,
                                 double maximum_feedback_correction,
                                 double previous_target_yaw_rate,
                                 double maximum_target_step,
                                 double sign_preservation_threshold);

int clamp_motion_pwm_band(int pwm, int min_pwm, int max_pwm);
int slew_limit_pwm(int previous_pwm, int target_pwm, int max_delta);

void scale_motion_pwm_pair_to_full_scale(int max_pwm,
                                         int* pwm_left,
                                         int* pwm_right);

void apply_start_motion_boost(int min_pwm,
                              int* pwm_left,
                              int* pwm_right);

void enforce_forward_differential_pwm(double target_yaw_rate,
                                      int minimum_pwm_delta,
                                      int max_pwm,
                                      int* pwm_left,
                                      int* pwm_right);

void enforce_forward_tracked_turn_authority(double target_yaw_rate,
                                            int outer_min_pwm,
                                            int inner_min_pwm,
                                            int inner_max_pwm,
                                            int* pwm_left,
                                            int* pwm_right);

}  // namespace thesis_sim::mvc::model
