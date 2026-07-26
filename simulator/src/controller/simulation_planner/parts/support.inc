#include "mvc/controller/simulation_planner/simulator.h"

#include "mvc/model/perception/perception_model.h"
#include "mvc/model/route/structured_path_model.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <utility>

namespace thesis_sim {

namespace {

constexpr int kArucoReferenceMixedRequiredGates = 4;

using mvc::model::RangeSensorSpec;
using mvc::model::ClosedPathProjection;
using mvc::model::project_closed_path;
using mvc::model::sample_closed_path;
using mvc::model::sample_closed_path_span;

mvc::model::SimulatedLidarConfig make_lidar_model_config(const SimConfig& config) {
    return {
        config.lidar_enabled,
        config.range_sensor_profile,
        config.lidar_beams,
        config.lidar_fov_rad,
        config.lidar_range,
        config.calibrated_conditions,
        config.lidar_range_noise_std_m,
        config.lidar_dropout_probability,
    };
}

struct LidarSearchWindow {
    double xy_window = 0.0;
    double xy_step = 0.0;
    double yaw_window = 0.0;
    double yaw_step = 0.0;
};

double clamp_value(double value, double lo, double hi) {
    return std::max(lo, std::min(value, hi));
}

bool is_finite_pair(double a, double b) {
    return std::isfinite(a) && std::isfinite(b);
}

double primitive_control_at(const traj_info& trajectory, double t, double fallback) {
    if (!std::isfinite(trajectory.c3) ||
        !std::isfinite(trajectory.c4) ||
        !std::isfinite(trajectory.c5)) {
        return fallback;
    }
    return trajectory.c3 + trajectory.c4 * t + 0.5 * trajectory.c5 * t * t;
}

double primitive_integral(const traj_info& trajectory, double t, double fallback_control) {
    if (!std::isfinite(trajectory.c3) ||
        !std::isfinite(trajectory.c4) ||
        !std::isfinite(trajectory.c5)) {
        return fallback_control * t;
    }
    return trajectory.c3 * t +
           0.5 * trajectory.c4 * t * t +
           (trajectory.c5 * t * t * t) / 6.0;
}

double primitive_sample_time(const traj_info& trajectory, double elapsed) {
    if (std::isfinite(trajectory.Tf) && trajectory.Tf > 1e-6) {
        return clamp_value(elapsed, 0.0, trajectory.Tf);
    }
    return std::max(0.0, elapsed);
}

double tracked_planner_yaw_accel_limit(const VehicleGeometry& geometry) {
    return clamp_value(2.0 * geometry.max_yaw_rate, 1.4, 5.0);
}

double elapsed_ms(std::chrono::steady_clock::time_point start,
                  std::chrono::steady_clock::time_point end) {
    return std::chrono::duration<double, std::milli>(end - start).count();
}

bool point_in_bounds(const Rect& bounds, const Vec2& p) {
    return p.x >= bounds.min_x && p.x <= bounds.max_x &&
           p.y >= bounds.min_y && p.y <= bounds.max_y;
}

bool point_in_expanded_rect(const Vec2& p, const Rect& rect, double padding) {
    return p.x >= rect.min_x - padding &&
           p.x <= rect.max_x + padding &&
           p.y >= rect.min_y - padding &&
           p.y <= rect.max_y + padding;
}

bool points_form_closed_loop(const std::vector<Vec2>& points, double threshold) {
    return points.size() >= 3 && distance(points.front(), points.back()) <= threshold;
}

bool road_environment(const WorldMap& world) {
    return world.environment_mode() == EnvironmentMode::StructuredRoad ||
           world.environment_mode() == EnvironmentMode::MixedRoadGates;
}

bool gate_environment(const WorldMap& world) {
    return world.environment_mode() == EnvironmentMode::UnstructuredGates ||
           world.environment_mode() == EnvironmentMode::MixedRoadGates;
}

double wrap_angle(double angle) {
    constexpr double kPi = 3.14159265358979323846;
    while (angle > kPi) {
        angle -= 2.0 * kPi;
    }
    while (angle < -kPi) {
        angle += 2.0 * kPi;
    }
    return angle;
}

double wrap_arc_length(double s, double s_max) {
    if (!(s_max > 1e-6)) {
        return 0.0;
    }
    double wrapped = std::fmod(s, s_max);
    if (wrapped < 0.0) {
        wrapped += s_max;
    }
    return wrapped;
}

double closed_loop_reference_span(double road_length, double lookahead_distance) {
    if (!(road_length > 1e-6)) {
        return 0.0;
    }
    if (road_length <= 5.0) {
        return clamp_value(0.75 * road_length, std::min(0.45, road_length), road_length);
    }
    return std::max(4.0, std::min(lookahead_distance, 0.75 * road_length));
}

double stabilize_structured_track_s(double candidate_s,
                                    double previous_s,
                                    double road_length,
                                    double max_forward_step,
                                    double* progress_delta) {
    if (!(road_length > 1e-6)) {
        if (progress_delta != nullptr) {
            *progress_delta = 0.0;
        }
        return 0.0;
    }

    const double safe_step = std::max(max_forward_step, 0.02);
    if (!std::isfinite(previous_s)) {
        if (progress_delta != nullptr) {
            *progress_delta = 0.0;
        }
        return wrap_arc_length(candidate_s, road_length);
    }

    const double previous_wrapped = wrap_arc_length(previous_s, road_length);
    double delta_s = candidate_s - previous_wrapped;
    if (delta_s > 0.5 * road_length) {
        delta_s -= road_length;
    } else if (delta_s < -0.5 * road_length) {
        delta_s += road_length;
    }

    if (delta_s < -0.15) {
        delta_s = 0.0;
    }
    delta_s = clamp_value(delta_s, 0.0, safe_step);
    if (progress_delta != nullptr) {
        *progress_delta = delta_s;
    }

    return wrap_arc_length(previous_wrapped + delta_s, road_length);
}

double closed_loop_forward_delta(double previous_s, double current_s, double road_length) {
    if (!(road_length > 1e-6) || !std::isfinite(previous_s) || !std::isfinite(current_s)) {
        return 0.0;
    }
    const double previous_wrapped = wrap_arc_length(previous_s, road_length);
    const double current_wrapped = wrap_arc_length(current_s, road_length);
    double delta_s = current_wrapped - previous_wrapped;
    if (delta_s < -0.15) {
        delta_s += road_length;
    }
    if (delta_s < 0.0 || delta_s > 0.75 * road_length) {
        return 0.0;
    }
    return delta_s;
}

bool compact_structured_world(const WorldMap& world) {
    if (!road_environment(world)) {
        return false;
    }
    const Rect& bounds = world.bounds();
    const double span = std::max(bounds.max_x - bounds.min_x, bounds.max_y - bounds.min_y);
    return span <= 0.75;
}

double world_span_m(const WorldMap& world) {
    const Rect& bounds = world.bounds();
    return std::max(bounds.max_x - bounds.min_x, bounds.max_y - bounds.min_y);
}

bool micro_structured_world(const WorldMap& world) {
    return compact_structured_world(world) && world_span_m(world) <= 0.35;
}

void apply_hardware_like_vehicle_tuning(const WorldMap& world,
                                        VehicleGeometry* geometry,
                                        SimConfig* config) {
    if (geometry == nullptr || config == nullptr) {
        return;
    }

    const bool structured = world.environment_mode() == EnvironmentMode::StructuredRoad;
    const bool unstructured = world.environment_mode() == EnvironmentMode::UnstructuredGates;
    const bool mixed = world.environment_mode() == EnvironmentMode::MixedRoadGates;
    if (!structured && !unstructured && !mixed) {
        return;
    }

    const double span = world_span_m(world);
    const bool compact = structured ? compact_structured_world(world) : span <= 5.0;
    const bool micro = structured ? micro_structured_world(world) : false;
    const bool validation_road =
        structured &&
        (world.structured_preset() == StructuredMapPreset::ValidationRoad ||
         world.structured_preset() == StructuredMapPreset::IdealCircle);
    const bool normalized_structured_road =
        structured && span >= 0.90 && span <= 1.25;
    const bool aruco_real_closed_mixed =
        mixed &&
        world.structured_preset() == StructuredMapPreset::TankCircuit &&
        distance(world.start(), world.goal()) <= 0.05 &&
        span <= 2.50;
    const double speed_cap = mixed ? 0.16
                                   : (normalized_structured_road ? 0.055
                                   : (micro ? (validation_road ? 0.020 : 0.025)
                                            : (validation_road
                                                   ? (normalized_structured_road ? 0.060 : 0.030)
                                                   : (compact ? 0.08 : 0.35))));
    const double speed_per_pwm = micro ? 0.0010 : (compact ? 0.0012 : 0.0016);
    const double motor_tau = micro ? 0.16 : 0.18;
    const double pwm_slew = micro ? 420.0 : (compact ? 480.0 : 450.0);

    if (unstructured || mixed) {
        geometry->wheelbase = 0.180;
        geometry->cg_to_front = 0.090;
        geometry->cg_to_rear = 0.090;
        geometry->track = 0.130;
        geometry->body_length = measured_robot::kCarBodyLengthM;
        geometry->body_width = measured_robot::kCarBodyWidthM;
        geometry->wheel_length = measured_robot::kCarWheelDiameterM;
        geometry->wheel_width = 0.020;
        geometry->wheel_radius = measured_robot::kCarWheelRadiusM;
        geometry->max_steer_angle = 0.950;
        geometry->max_steer_rate = 3.200;
        geometry->max_curvature = 4.200;
        geometry->max_yaw_rate = 3.000;
        geometry->min_effective_pwm = 45;
        geometry->speed_estimate_per_pwm = 0.0016;
        geometry->motor_time_constant = 0.240;
        geometry->pwm_slew_rate = 450.0;
        geometry->yaw_response_scale = 1.00;
        geometry->linear_feedback_gain = 75.0;
        geometry->yaw_feedback_gain = 35.0;
        if (aruco_real_closed_mixed) {
            geometry->track = 0.130;
            geometry->max_curvature = 6.0;
            geometry->max_yaw_rate = 3.4;
        }
    } else if (normalized_structured_road) {
        geometry->wheelbase = 0.180;
        geometry->cg_to_front = 0.090;
        geometry->cg_to_rear = 0.090;
        geometry->track = 0.130;
        geometry->body_length = measured_robot::kCarBodyLengthM;
        geometry->body_width = measured_robot::kCarBodyWidthM;
        geometry->wheel_length = measured_robot::kCarWheelDiameterM;
        geometry->wheel_width = 0.020;
        geometry->wheel_radius = measured_robot::kCarWheelRadiusM;
        geometry->max_steer_angle = 0.60;
        geometry->max_steer_rate = 2.80;
        geometry->max_curvature = 3.20;
        geometry->max_yaw_rate = 2.40;
        geometry->yaw_response_scale = 1.00;
        geometry->linear_feedback_gain = 75.0;
        geometry->yaw_feedback_gain = 45.0;
    } else if (micro) {
        geometry->wheelbase = 0.085;
        geometry->cg_to_front = 0.0425;
        geometry->cg_to_rear = 0.0425;
        geometry->track = 0.070;
        geometry->body_length = 0.095;
        geometry->body_width = 0.075;
        geometry->wheel_length = 0.022;
        geometry->wheel_width = 0.010;
        geometry->wheel_radius = 0.016;
        geometry->yaw_response_scale = 0.46;
        geometry->linear_feedback_gain = std::max(geometry->linear_feedback_gain, 105.0);
        geometry->yaw_feedback_gain =
            std::max(geometry->yaw_feedback_gain, validation_road ? 150.0 : 95.0);
    } else if (compact) {
        geometry->wheelbase = 0.09;
        geometry->cg_to_front = 0.045;
        geometry->cg_to_rear = 0.045;
        geometry->track = 0.080;
        geometry->body_length = 0.10;
        geometry->body_width = 0.08;
        geometry->wheel_length = 0.024;
        geometry->wheel_width = 0.011;
        geometry->wheel_radius = 0.017;
        geometry->yaw_response_scale = 0.60;
        geometry->linear_feedback_gain = std::max(geometry->linear_feedback_gain, 95.0);
        geometry->yaw_feedback_gain = std::max(geometry->yaw_feedback_gain, 120.0);
    }
    if (!unstructured && !mixed) {
        geometry->min_effective_pwm = std::max(geometry->min_effective_pwm, 55);
        geometry->speed_estimate_per_pwm = std::min(geometry->speed_estimate_per_pwm, speed_per_pwm);
        geometry->motor_time_constant = std::max(geometry->motor_time_constant, motor_tau);
        geometry->pwm_slew_rate = std::min(geometry->pwm_slew_rate, pwm_slew);
    }
    geometry->max_linear_speed = std::min(geometry->max_linear_speed, speed_cap);
    config->cruise_speed_limit = std::min(config->cruise_speed_limit, speed_cap);
}

void apply_tracked_vehicle_tuning(const WorldMap& world,
                                  VehicleGeometry* geometry,
                                  SimConfig* config) {
    if (geometry == nullptr || config == nullptr) {
        return;
    }

    const bool compact = compact_structured_world(world);
    const bool compact_gate_world =
        gate_environment(world) && world_span_m(world) <= 5.0;
    const bool micro = micro_structured_world(world);
    const bool validation_road =
        world.environment_mode() == EnvironmentMode::StructuredRoad &&
        (world.structured_preset() == StructuredMapPreset::ValidationRoad ||
         world.structured_preset() == StructuredMapPreset::IdealCircle);
    const bool normalized_hardware_structured =
        world.environment_mode() == EnvironmentMode::StructuredRoad &&
        world_span_m(world) >= 0.90 &&
        world_span_m(world) <= 1.25;
    const bool aruco_real_closed_mixed =
        world.environment_mode() == EnvironmentMode::MixedRoadGates &&
        world.structured_preset() == StructuredMapPreset::TankCircuit &&
        distance(world.start(), world.goal()) <= 0.05 &&
        world_span_m(world) <= 2.50;

    if (aruco_real_closed_mixed) {
        geometry->wheelbase = measured_robot::kTankBeltLengthM;
        geometry->cg_to_front = 0.5 * measured_robot::kTankBodyLengthM;
        geometry->cg_to_rear = 0.5 * measured_robot::kTankBodyLengthM;
        geometry->track = measured_robot::kTankTrackCenterDistanceM;
        geometry->body_length = measured_robot::kTankBodyLengthM;
        geometry->body_width = measured_robot::kTankBodyWidthM;
        geometry->wheel_length = measured_robot::kTankBeltLengthM;
        geometry->wheel_width = measured_robot::kTankBeltWidthM;
        geometry->wheel_radius = measured_robot::kTankEffectiveDriveRadiusDefaultM;
        geometry->encoder_ticks_per_revolution = 2400;
        geometry->max_linear_speed = 0.16;
        geometry->max_yaw_rate = 3.4;
        geometry->max_curvature = 8.0;
        geometry->max_steer_angle = 1.25;
        geometry->max_steer_rate = 5.0;
        geometry->speed_estimate_per_pwm = 0.0015;
        geometry->motor_time_constant = 0.20;
        geometry->pwm_slew_rate = 500.0;
        geometry->yaw_response_scale = 0.92;
        geometry->linear_feedback_gain = 80.0;
        geometry->yaw_feedback_gain = 80.0;
        geometry->min_effective_pwm = 44;
        config->cruise_speed_limit = std::min(config->cruise_speed_limit, 0.08);
    } else if (normalized_hardware_structured) {
        geometry->wheelbase = measured_robot::kTankBeltLengthM;
        geometry->cg_to_front = 0.5 * measured_robot::kTankBodyLengthM;
        geometry->cg_to_rear = 0.5 * measured_robot::kTankBodyLengthM;
        geometry->track = measured_robot::kTankTrackCenterDistanceM;
        geometry->body_length = measured_robot::kTankBodyLengthM;
        geometry->body_width = measured_robot::kTankBodyWidthM;
        geometry->wheel_length = measured_robot::kTankBeltLengthM;
        geometry->wheel_width = measured_robot::kTankBeltWidthM;
        geometry->wheel_radius = measured_robot::kTankEffectiveDriveRadiusDefaultM;
        geometry->encoder_ticks_per_revolution = 2400;
        geometry->max_linear_speed = 0.14;
        geometry->max_yaw_rate = 1.35;
        geometry->max_curvature = 6.0;
        geometry->max_steer_angle = 1.10;
        geometry->max_steer_rate = 4.20;
        geometry->speed_estimate_per_pwm = 0.0016;
        geometry->motor_time_constant = 0.24;
        geometry->pwm_slew_rate = 450.0;
        geometry->yaw_response_scale = 1.0;
        geometry->linear_feedback_gain = 50.0;
        geometry->yaw_feedback_gain = 32.0;
        geometry->min_effective_pwm = 48;
        config->cruise_speed_limit = std::min(config->cruise_speed_limit, 0.08);
    } else if (compact_gate_world) {
        geometry->wheelbase = measured_robot::kTankBeltLengthM;
        geometry->cg_to_front = 0.5 * measured_robot::kTankBodyLengthM;
        geometry->cg_to_rear = 0.5 * measured_robot::kTankBodyLengthM;
        geometry->track = measured_robot::kTankTrackCenterDistanceM;
        geometry->body_length = measured_robot::kTankBodyLengthM;
        geometry->body_width = measured_robot::kTankBodyWidthM;
        geometry->wheel_length = measured_robot::kTankBeltLengthM;
        geometry->wheel_width = measured_robot::kTankBeltWidthM;
        geometry->wheel_radius = measured_robot::kTankEffectiveDriveRadiusDefaultM;
        geometry->encoder_ticks_per_revolution = 2400;
        geometry->max_linear_speed = 0.14;
        geometry->max_yaw_rate = 1.80;
        geometry->max_curvature = 6.0;
        geometry->max_steer_angle = 1.15;
        geometry->max_steer_rate = 4.20;
        geometry->speed_estimate_per_pwm = 0.0016;
        geometry->motor_time_constant = 0.24;
        geometry->pwm_slew_rate = 450.0;
        geometry->yaw_response_scale = 1.0;
        geometry->linear_feedback_gain = 50.0;
        geometry->yaw_feedback_gain = 32.0;
        geometry->min_effective_pwm = 48;
        config->cruise_speed_limit = std::min(
            config->cruise_speed_limit,
            world.environment_mode() == EnvironmentMode::MixedRoadGates ? 0.065 : 0.080);
    } else if (micro) {
        geometry->wheelbase = 0.085;
        geometry->cg_to_front = 0.0425;
        geometry->cg_to_rear = 0.0425;
        geometry->track = 0.070;
        geometry->body_length = 0.110;
        geometry->body_width = 0.085;
        geometry->wheel_length = 0.090;
        geometry->wheel_width = 0.018;
        geometry->wheel_radius = 0.016;
        geometry->max_linear_speed = validation_road ? 0.024 : 0.032;
        geometry->max_yaw_rate = 7.5;
        geometry->max_curvature = validation_road ? 22.0 : 18.0;
        geometry->max_steer_angle = 1.35;
        geometry->max_steer_rate = validation_road ? 12.0 : 8.0;
        geometry->speed_estimate_per_pwm = 0.0010;
        geometry->motor_time_constant = 0.18;
        geometry->pwm_slew_rate = 430.0;
        geometry->yaw_response_scale = 0.72;
        geometry->linear_feedback_gain = 120.0;
        geometry->yaw_feedback_gain = validation_road ? 150.0 : 115.0;
        config->cruise_speed_limit = std::min(config->cruise_speed_limit, geometry->max_linear_speed);
    } else if (compact) {
        geometry->wheelbase = 0.105;
        geometry->cg_to_front = 0.0525;
        geometry->cg_to_rear = 0.0525;
        geometry->track = 0.085;
        geometry->body_length = 0.135;
        geometry->body_width = 0.095;
        geometry->wheel_length = 0.110;
        geometry->wheel_width = 0.022;
        geometry->wheel_radius = 0.017;
        geometry->max_linear_speed = 0.070;
        geometry->max_yaw_rate = 6.0;
        geometry->max_curvature = 16.0;
        geometry->max_steer_angle = 1.25;
        geometry->max_steer_rate = 8.0;
        geometry->speed_estimate_per_pwm = 0.0012;
        geometry->motor_time_constant = 0.20;
        geometry->pwm_slew_rate = 460.0;
        geometry->yaw_response_scale = 0.78;
        geometry->linear_feedback_gain = 105.0;
        geometry->yaw_feedback_gain = 125.0;
        config->cruise_speed_limit = std::min(config->cruise_speed_limit, geometry->max_linear_speed);
    } else {
        geometry->wheelbase = measured_robot::kTankBeltLengthM;
        geometry->cg_to_front = 0.5 * measured_robot::kTankBodyLengthM;
        geometry->cg_to_rear = 0.5 * measured_robot::kTankBodyLengthM;
        geometry->track = measured_robot::kTankTrackCenterDistanceM;
        geometry->body_length = measured_robot::kTankBodyLengthM;
        geometry->body_width = measured_robot::kTankBodyWidthM;
        geometry->wheel_length = measured_robot::kTankBeltLengthM;
        geometry->wheel_width = measured_robot::kTankBeltWidthM;
        geometry->wheel_radius = measured_robot::kTankEffectiveDriveRadiusDefaultM;
        geometry->max_linear_speed = 0.42;
        geometry->max_yaw_rate = 2.40;
        geometry->max_curvature = 7.0;
        geometry->max_steer_angle = 1.25;
        geometry->max_steer_rate = 4.5;
        geometry->speed_estimate_per_pwm = 0.0015;
        geometry->motor_time_constant = 0.24;
        geometry->pwm_slew_rate = 420.0;
        geometry->yaw_response_scale = 0.86;
        geometry->linear_feedback_gain = 80.0;
        geometry->yaw_feedback_gain = 70.0;
        config->cruise_speed_limit = std::min(config->cruise_speed_limit, 0.26);
    }

    if (!normalized_hardware_structured) {
        geometry->min_effective_pwm = std::max(geometry->min_effective_pwm, 50);
    }
    geometry->wheel_speed_to_pwm_gain = 190.0;
    geometry->wheel_speed_to_pwm_bias = 28.0;
    const bool compact_motion = compact || compact_gate_world;
    geometry->max_accel = std::min(geometry->max_accel, compact_motion ? 0.9 : 1.1);
    geometry->max_decel = std::min(geometry->max_decel, compact_motion ? 1.1 : 1.4);
}

void apply_vehicle_tuning_overrides(const VehicleTuningOverrides& overrides,
                                    VehicleGeometry* geometry,
                                    SimConfig* config) {
    if (geometry == nullptr || config == nullptr) {
        return;
    }

    if (overrides.wheelbase.has_value()) {
        geometry->wheelbase = *overrides.wheelbase;
        geometry->cg_to_front = 0.5 * geometry->wheelbase;
        geometry->cg_to_rear = 0.5 * geometry->wheelbase;
    }
    if (overrides.track.has_value()) {
        geometry->track = *overrides.track;
    }
    if (overrides.body_length.has_value()) {
        geometry->body_length = *overrides.body_length;
    }
    if (overrides.body_width.has_value()) {
        geometry->body_width = *overrides.body_width;
    }
    if (overrides.wheel_length.has_value()) {
        geometry->wheel_length = *overrides.wheel_length;
    }
    if (overrides.wheel_width.has_value()) {
        geometry->wheel_width = *overrides.wheel_width;
    }
    if (overrides.wheel_radius.has_value()) {
        geometry->wheel_radius = *overrides.wheel_radius;
    }
    if (overrides.encoder_ticks_per_revolution.has_value()) {
        geometry->encoder_ticks_per_revolution = static_cast<std::int32_t>(
            std::llround(*overrides.encoder_ticks_per_revolution));
    }

    if (overrides.min_effective_pwm.has_value()) {
        geometry->min_effective_pwm = static_cast<int>(std::lround(*overrides.min_effective_pwm));
    }
    if (overrides.speed_estimate_per_pwm.has_value()) {
        geometry->speed_estimate_per_pwm = *overrides.speed_estimate_per_pwm;
        if (geometry->speed_estimate_per_pwm > 1e-9) {
            // The calibration slope and its feed-forward inverse are one
            // contract. Keeping the legacy gain/bias while changing only the
            // forward plant made low-speed calibrated targets run 3-4x fast.
            geometry->wheel_speed_to_pwm_gain =
                1.0 / geometry->speed_estimate_per_pwm;
            geometry->wheel_speed_to_pwm_bias = 0.0;
        }
    }
    if (overrides.pwm_slew_rate.has_value()) {
        geometry->pwm_slew_rate = *overrides.pwm_slew_rate;
    }
    if (overrides.motor_time_constant.has_value()) {
        geometry->motor_time_constant = *overrides.motor_time_constant;
    }
    if (overrides.max_linear_speed.has_value()) {
        geometry->max_linear_speed = *overrides.max_linear_speed;
    }
    if (overrides.max_curvature.has_value()) {
        geometry->max_curvature = *overrides.max_curvature;
    }
    if (overrides.max_steer_angle.has_value()) {
        geometry->max_steer_angle = *overrides.max_steer_angle;
    }
    if (overrides.max_steer_rate.has_value()) {
        geometry->max_steer_rate = *overrides.max_steer_rate;
    }
    if (overrides.max_yaw_rate.has_value()) {
        geometry->max_yaw_rate = *overrides.max_yaw_rate;
    }
    if (overrides.linear_feedback_gain.has_value()) {
        geometry->linear_feedback_gain = *overrides.linear_feedback_gain;
    }
    if (overrides.yaw_feedback_gain.has_value()) {
        geometry->yaw_feedback_gain = *overrides.yaw_feedback_gain;
    }
    if (overrides.left_pwm_scale.has_value()) {
        geometry->left_pwm_scale = *overrides.left_pwm_scale;
    }
    if (overrides.right_pwm_scale.has_value()) {
        geometry->right_pwm_scale = *overrides.right_pwm_scale;
    }
    if (overrides.yaw_response_scale.has_value()) {
        geometry->yaw_response_scale = *overrides.yaw_response_scale;
    }
    if (overrides.cruise_speed_limit.has_value()) {
        config->cruise_speed_limit = *overrides.cruise_speed_limit;
    }
}

bool project_curvilinear_state(const clothoid_info& clothoid,
                               const Vec2& position,
                               double s_hint,
                               bool wrap_loop,
                               double* out_s,
                               double* out_n) {
    if (clothoid.prev_road.num_segments() <= 0 || !(clothoid.end_point_s > 0.0)) {
        return false;
    }

    const double road_length = clothoid.end_point_s;
    double best_s = std::numeric_limits<double>::quiet_NaN();
    double best_distance = std::numeric_limits<double>::infinity();
    double best_x = 0.0;
    double best_y = 0.0;

    auto try_sample = [&](double sample_s) {
        const double eval_s = wrap_loop ? wrap_arc_length(sample_s, road_length)
                                        : clamp_value(sample_s, 0.0, road_length);
        double x = 0.0;
        double y = 0.0;
        clothoid.prev_road.eval(eval_s, x, y);
        const double dst = std::hypot(position.x - x, position.y - y);
        if (dst < best_distance) {
            best_distance = dst;
            best_s = eval_s;
            best_x = x;
            best_y = y;
        }
    };

    if (std::isfinite(s_hint)) {
        double lo = wrap_loop ? wrap_arc_length(s_hint, road_length) - 2.0
                              : clamp_value(s_hint - 2.0, 0.0, road_length);
        double hi = wrap_loop ? wrap_arc_length(s_hint, road_length) + 8.0
                              : clamp_value(s_hint + 8.0, 0.0, road_length);
        double step = 0.25;
        for (int pass = 0; pass < 3; ++pass) {
            for (double sample_s = lo; sample_s <= hi + 0.5 * step; sample_s += step) {
                try_sample(sample_s);
            }
            if (!std::isfinite(best_s)) {
                continue;
            }
            if (wrap_loop) {
                lo = best_s - 2.0 * step;
                hi = best_s + 2.0 * step;
            } else {
                lo = clamp_value(best_s - 2.0 * step, 0.0, road_length);
                hi = clamp_value(best_s + 2.0 * step, 0.0, road_length);
            }
            step = std::max(step * 0.25, 0.02);
        }
    }

    double iso_x = 0.0;
    double iso_y = 0.0;
    double iso_s = 0.0;
    double iso_n = 0.0;
    double iso_dst = 0.0;
    const int found = clothoid.prev_road.closest_point_ISO(
        position.x,
        position.y,
        iso_x,
        iso_y,
        iso_s,
        iso_n,
        iso_dst);
    if (found >= 0 && std::isfinite(iso_s) && iso_dst < best_distance) {
        best_s = wrap_loop ? wrap_arc_length(iso_s, road_length) : clamp_value(iso_s, 0.0, road_length);
        best_distance = iso_dst;
        best_x = iso_x;
        best_y = iso_y;
    }

    if (wrap_loop && (!std::isfinite(best_s) || best_distance > 0.75)) {
        const double coarse_step = std::max(0.05, road_length / 240.0);
        for (double sample_s = 0.0; sample_s <= road_length + 0.5 * coarse_step; sample_s += coarse_step) {
            try_sample(sample_s);
        }
        if (std::isfinite(best_s)) {
            double lo = best_s - 2.0 * coarse_step;
            double hi = best_s + 2.0 * coarse_step;
            double step = std::max(0.01, coarse_step * 0.25);
            for (int pass = 0; pass < 2; ++pass) {
                for (double sample_s = lo; sample_s <= hi + 0.5 * step; sample_s += step) {
                    try_sample(sample_s);
                }
                lo = best_s - 2.0 * step;
                hi = best_s + 2.0 * step;
                step = std::max(0.005, step * 0.25);
            }
        }
    }

    if (!std::isfinite(best_s)) {
        return false;
    }

    const double heading = clothoid.prev_road.theta(best_s);
    const double dx = position.x - best_x;
    const double dy = position.y - best_y;
    if (out_s != nullptr) {
        *out_s = best_s;
    }
    if (out_n != nullptr) {
        *out_n = -std::sin(heading) * dx + std::cos(heading) * dy;
    }
    return true;
}

double steer_from_curvature(const VehicleGeometry& geometry, double curvature) {
    const double steer = std::atan(geometry.wheelbase * clamp_value(curvature, -geometry.max_curvature, geometry.max_curvature));
    return clamp_value(steer, -geometry.max_steer_angle, geometry.max_steer_angle);
}

std::vector<ReferenceWaypoint> build_reference_waypoints(const std::vector<Vec2>& points, double speed_ref) {
    std::vector<ReferenceWaypoint> reference;
    if (points.size() < 2) {
        return reference;
    }

    reference.reserve(points.size());
    std::vector<double> headings(points.size(), 0.0);
    for (size_t i = 0; i < points.size(); ++i) {
        const Vec2 prev = points[i > 0 ? i - 1 : i];
        const Vec2 next = points[i + 1 < points.size() ? i + 1 : i];
        headings[i] = std::atan2(next.y - prev.y, next.x - prev.x);
    }

    for (size_t i = 0; i < points.size(); ++i) {
        double curvature = 0.0;
        if (i > 0 && i + 1 < points.size()) {
            const double ds = std::max(distance(points[i + 1], points[i - 1]) * 0.5, 1e-3);
            curvature = wrap_angle(headings[i + 1] - headings[i - 1]) / ds;
        }
        reference.push_back({
            points[i],
            headings[i],
            curvature,
            speed_ref,
        });
    }
    return reference;
}

std::vector<ReferenceWaypoint> build_reference_waypoints(const clothoid_info& clothoid,
                                                         double s_start,
                                                         double s_end,
                                                         int sample_count,
                                                         double speed_ref,
                                                         bool wrap_loop = false) {
    std::vector<ReferenceWaypoint> reference;
    if (!(s_end > s_start + 1e-4) || sample_count < 2) {
        return reference;
    }

    const double road_length = std::max(clothoid.end_point_s, 1e-6);
    reference.reserve(static_cast<size_t>(sample_count));
    for (int i = 0; i < sample_count; ++i) {
        const double alpha =
            sample_count > 1 ? static_cast<double>(i) / static_cast<double>(sample_count - 1) : 0.0;
        const double sample_s = s_start + alpha * (s_end - s_start);
        const double s = wrap_loop
                             ? wrap_arc_length(sample_s, road_length)
                             : clamp_value(sample_s, 0.0, clothoid.end_point_s);
        double x = 0.0;
        double y = 0.0;
        clothoid.prev_road.eval(s, x, y);
        reference.push_back({
            {x, y},
            wrap_angle(clothoid.prev_road.theta(s)),
            clothoid.prev_road.kappa(s),
            speed_ref,
        });
    }
    return reference;
}

std::vector<Vec2> extract_reference_positions(const std::vector<ReferenceWaypoint>& reference) {
    std::vector<Vec2> points;
    points.reserve(reference.size());
    for (const ReferenceWaypoint& waypoint : reference) {
        points.push_back(waypoint.position);
    }
    return points;
}

VehicleModelState build_tracking_state(const VehicleModelState& vehicle_state,
                                       const Vec2& position,
                                       double yaw,
                                       double speed,
                                       double accel,
                                       double curvature,
                                       double yaw_rate) {
    VehicleModelState tracking_state = vehicle_state;
    tracking_state.position = position;
    tracking_state.yaw = yaw;
    tracking_state.speed = speed;
    tracking_state.accel = accel;
    tracking_state.curvature = curvature;
    tracking_state.yaw_rate = yaw_rate;
    return tracking_state;
}

double planner_speed_limit(double cruise_speed_limit, double curvature) {
    constexpr double kStraightCurvature = 0.1;
    if (!std::isfinite(curvature) || std::abs(curvature) < kStraightCurvature) {
        return cruise_speed_limit;
    }

    const double curvature_limit = 1.5 * std::pow(std::abs(curvature), -1.0 / 3.0);
    const double minimum_turn_speed = std::min(0.15, cruise_speed_limit);
    return clamp_value(curvature_limit, minimum_turn_speed, cruise_speed_limit);
}

double tracked_vehicle_speed_floor(const WorldMap& world, const VehicleGeometry& geometry) {
    if (micro_structured_world(world)) {
        return std::min(geometry.max_linear_speed, 0.018);
    }
    if (compact_structured_world(world)) {
        return std::min(geometry.max_linear_speed, 0.035);
    }
    if (gate_environment(world) && world_span_m(world) <= 5.0) {
        return std::min(
            geometry.max_linear_speed,
            world.environment_mode() == EnvironmentMode::MixedRoadGates ? 0.065 : 0.080);
    }
    return std::min(geometry.max_linear_speed, 0.12);
}

bool structured_road_is_closed_loop(const WorldMap& world) {
    return road_environment(world) &&
           points_form_closed_loop(world.road_centerline(), 0.45);
}

bool compact_mixed_closed_loop(const WorldMap& world) {
    return world.environment_mode() == EnvironmentMode::MixedRoadGates &&
           (structured_road_is_closed_loop(world) || distance(world.start(), world.goal()) <= 0.05) &&
           world_span_m(world) <= 2.50;
}

bool mixed_closed_gate_sequence(const WorldMap& world) {
    return world.environment_mode() == EnvironmentMode::MixedRoadGates &&
           (structured_road_is_closed_loop(world) || distance(world.start(), world.goal()) <= 0.05);
}

bool mixed_closed_semantic_gate_sequence(const WorldMap& world) {
    if (!mixed_closed_gate_sequence(world)) {
        return false;
    }
    return std::any_of(world.gates().begin(), world.gates().end(), [](const GateSpec& gate) {
        return !gate.final;
    });
}

bool compact_unstructured_world(const WorldMap& world) {
    return gate_environment(world) &&
           world_span_m(world) <= 5.0;
}

double distance_to_gate_point(const gate& candidate, const Vec2& position) {
    return std::hypot(candidate.x_pos - position.x, candidate.y_pos - position.y);
}

Vec2 gate_position(const gate& candidate) {
    return {candidate.x_pos, candidate.y_pos};
}

double dot_product(const Vec2& a, const Vec2& b) {
    return a.x * b.x + a.y * b.y;
}

double cross_product(const Vec2& a, const Vec2& b) {
    return a.x * b.y - a.y * b.x;
}

Vec2 subtract(const Vec2& a, const Vec2& b) {
    return {a.x - b.x, a.y - b.y};
}

}  // namespace

const char* range_sensor_profile_name(RangeSensorProfile profile) {
    switch (profile) {
        case RangeSensorProfile::IdealLidar2D:
            return "Ideal 2D LiDAR";
        case RangeSensorProfile::RplidarA1:
            return "RPLidar A1";
        case RangeSensorProfile::ShortRangeScanner:
            return "Short Range Scanner";
        default:
            return "Unknown";
    }
}
