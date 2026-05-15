#include "planner_sim.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <utility>

namespace thesis_sim {

namespace {

struct RangeSensorSpec {
    int beams = 0;
    double fov_rad = 0.0;
    double range = 0.0;
};

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
    if (unstructured && !compact) {
        return;
    }
    const bool validation_road =
        structured && world.structured_preset() == StructuredMapPreset::ValidationRoad;
    const double speed_cap = mixed ? 0.16
                                   : (micro ? (validation_road ? 0.020 : 0.025)
                                            : (validation_road ? 0.03 : (compact ? 0.08 : 0.35)));
    const double speed_per_pwm = micro ? 0.0010 : (compact ? 0.0012 : 0.0016);
    const double motor_tau = micro ? 0.16 : 0.18;
    const double pwm_slew = micro ? 420.0 : (compact ? 480.0 : 450.0);

    if (unstructured || mixed) {
        geometry->wheelbase = 0.300;
        geometry->cg_to_front = 0.150;
        geometry->cg_to_rear = 0.150;
        geometry->track = 0.280;
        geometry->body_length = 0.340;
        geometry->body_width = 0.240;
        geometry->wheel_length = 0.090;
        geometry->wheel_width = 0.030;
        geometry->wheel_radius = 0.040;
        geometry->max_steer_angle = mixed ? 0.950 : 0.520;
        geometry->max_steer_rate = mixed ? 3.200 : 1.800;
        geometry->max_curvature = mixed ? 4.200 : 1.800;
        geometry->max_yaw_rate = mixed ? 3.000 : 2.400;
        geometry->min_effective_pwm = 45;
        geometry->speed_estimate_per_pwm = 0.0016;
        geometry->motor_time_constant = 0.240;
        geometry->pwm_slew_rate = 450.0;
        geometry->yaw_response_scale = 1.00;
        geometry->linear_feedback_gain = 75.0;
        geometry->yaw_feedback_gain = 35.0;
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
    const bool micro = micro_structured_world(world);
    const bool validation_road =
        world.environment_mode() == EnvironmentMode::StructuredRoad &&
        world.structured_preset() == StructuredMapPreset::ValidationRoad;

    if (micro) {
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
        geometry->wheelbase = 0.260;
        geometry->cg_to_front = 0.130;
        geometry->cg_to_rear = 0.130;
        geometry->track = 0.280;
        geometry->body_length = 0.340;
        geometry->body_width = 0.240;
        geometry->wheel_length = 0.285;
        geometry->wheel_width = 0.055;
        geometry->wheel_radius = 0.040;
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

    geometry->min_effective_pwm = std::max(geometry->min_effective_pwm, 50);
    geometry->wheel_speed_to_pwm_gain = 190.0;
    geometry->wheel_speed_to_pwm_bias = 28.0;
    geometry->max_accel = std::min(geometry->max_accel, compact ? 0.9 : 1.1);
    geometry->max_decel = std::min(geometry->max_decel, compact ? 1.1 : 1.4);
}

void apply_vehicle_tuning_overrides(const VehicleTuningOverrides& overrides,
                                    VehicleGeometry* geometry,
                                    SimConfig* config) {
    if (geometry == nullptr || config == nullptr) {
        return;
    }

    if (overrides.min_effective_pwm.has_value()) {
        geometry->min_effective_pwm = static_cast<int>(std::lround(*overrides.min_effective_pwm));
    }
    if (overrides.speed_estimate_per_pwm.has_value()) {
        geometry->speed_estimate_per_pwm = *overrides.speed_estimate_per_pwm;
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

RangeSensorSpec make_range_sensor_spec(const SimConfig& config) {
    constexpr double kPi = 3.14159265358979323846;
    switch (config.range_sensor_profile) {
        case RangeSensorProfile::IdealLidar2D:
            return {std::max(config.lidar_beams, 1), config.lidar_fov_rad, config.lidar_range};
        case RangeSensorProfile::RplidarA1:
            return {360, 2.0 * kPi, 12.0};
        case RangeSensorProfile::ShortRangeScanner:
            return {121, 2.0 * kPi / 3.0, 4.5};
        default:
            return {std::max(config.lidar_beams, 1), config.lidar_fov_rad, config.lidar_range};
    }
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
    return std::min(geometry.max_linear_speed, 0.12);
}

bool structured_road_is_closed_loop(const WorldMap& world) {
    return road_environment(world) &&
           points_form_closed_loop(world.road_centerline(), 0.45);
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

const char* PlannerDrivenVehicleSim::localization_mode_name() const {
    if (config_.imu_enabled && config_.lidar_enabled) {
        return "Encoders + IMU + LiDAR";
    }
    if (config_.imu_enabled) {
        return "Encoders + IMU";
    }
    if (config_.lidar_enabled) {
        return "Encoders + LiDAR";
    }
    return "Encoders only";
}

PlannerDrivenVehicleSim::PlannerDrivenVehicleSim(WorldMap world, SimConfig config)
    : world_(std::move(world)),
      config_(config),
      null_stream_("/dev/null") {
    reset();
}

void PlannerDrivenVehicleSim::reset() {
    geometry_ = VehicleGeometry{};
    config_.cruise_speed_limit = SimConfig{}.cruise_speed_limit;
    if (config_.vehicle_model == VehicleModelKind::TrackedVehicle) {
        apply_tracked_vehicle_tuning(world_, &geometry_, &config_);
    } else {
        apply_hardware_like_vehicle_tuning(world_, &geometry_, &config_);
    }
    if (world_.environment_mode() == EnvironmentMode::MixedRoadGates) {
        config_.max_history = std::max(config_.max_history, 9000);
    }
    if (micro_structured_world(world_)) {
        const bool validation_road =
            world_.environment_mode() == EnvironmentMode::StructuredRoad &&
            world_.structured_preset() == StructuredMapPreset::ValidationRoad;
        geometry_.max_steer_angle = std::max(geometry_.max_steer_angle, 1.35);
        geometry_.max_steer_rate =
            std::max(geometry_.max_steer_rate, validation_road ? 12.0 : 8.00);
        geometry_.max_curvature =
            std::max(geometry_.max_curvature, validation_road ? 20.0 : 16.0);
        geometry_.max_yaw_rate = std::max(geometry_.max_yaw_rate, 8.0);
    } else if (compact_structured_world(world_)) {
        geometry_.max_steer_angle = std::max(geometry_.max_steer_angle, 1.25);
        geometry_.max_steer_rate = std::max(geometry_.max_steer_rate, 8.00);
        geometry_.max_curvature = std::max(geometry_.max_curvature, 16.0);
        geometry_.max_yaw_rate = std::max(geometry_.max_yaw_rate, 8.0);
    } else if (world_.environment_mode() == EnvironmentMode::MixedRoadGates) {
        geometry_.max_steer_angle = std::max(geometry_.max_steer_angle, 0.72);
        geometry_.max_steer_rate = std::max(geometry_.max_steer_rate, 2.40);
        geometry_.max_curvature = std::max(geometry_.max_curvature, 2.40);
        geometry_.max_yaw_rate = std::max(geometry_.max_yaw_rate, 2.80);
    }
    apply_vehicle_tuning_overrides(tuning_overrides_, &geometry_, &config_);
    rebuild_vehicle_model();
    MpcFollowerConfig mpc_config{};
    if (micro_structured_world(world_)) {
        mpc_config.preview_distance = 0.08;
        mpc_config.min_lookahead_distance = 0.05;
        mpc_config.max_steer_rate = 8.00;
        mpc_config.w_heading = 18.0;
        mpc_config.w_steer_rate = 0.020;
    } else if (compact_structured_world(world_)) {
        mpc_config.preview_distance = 0.20;
        mpc_config.min_lookahead_distance = 0.12;
        mpc_config.max_steer_rate = 4.50;
        mpc_config.w_heading = 14.0;
        mpc_config.w_steer_rate = 0.025;
    } else if (world_.environment_mode() == EnvironmentMode::MixedRoadGates) {
        mpc_config.preview_distance = 0.65;
        mpc_config.min_lookahead_distance = 0.32;
        mpc_config.max_steer_rate = 3.20;
        mpc_config.w_heading = 13.0;
        mpc_config.w_steer_rate = 0.030;
    }
    mpc_follower_ = KinematicBicycleMpcFollower(mpc_config);

    step_count_ = 0;
    sim_time_ = 0.0;
    last_j_ = 0.0;
    last_r_ = 0.0;
    chosen_gate_index_ = -1;
    dynamic_lidar_passed_gates_ = 0;
    dynamic_lidar_candidate_count_ = 0;
    dynamic_lidar_gate_hold_steps_ = 0;
    dynamic_lidar_stable_steps_ = 0;
    dynamic_lidar_active_gate_width_ = 0.0;
    dynamic_lidar_active_gate_score_ = 0.0;
    dynamic_lidar_passed_gate_positions_.clear();
    mixed_gate_mode_active_ = false;
    mixed_gate_rejoin_cooldown_steps_ = 0;
    mixed_switch_count_ = 0;
    mixed_abort_count_ = 0;
    mixed_gate_score_ = 0.0;
    mixed_structured_score_ = 1.0;
    mixed_gate_confidence_ = 0.0;
    goal_reached_ = false;
    collision_ = false;
    distance_to_goal_ = distance(world_.start(), world_.goal());
    min_lidar_distance_ = active_lidar_range();
    navigation_position_ = world_.start();
    navigation_yaw_ = world_.start_heading();
    navigation_yaw_rate_ = 0.0;
    navigation_curvature_ = 0.0;
    navigation_speed_ = 0.0;
    navigation_accel_ = 0.0;
    navigation_xy_error_ = 0.0;
    navigation_yaw_error_deg_ = 0.0;
    planner_speed_ref_ = 0.0;
    planner_accel_ref_ = 0.0;
    tracker_cross_track_error_ = 0.0;
    tracker_heading_error_deg_ = 0.0;
    planning_compute_ms_ = 0.0;
    tracking_compute_ms_ = 0.0;
    lidar_compute_ms_ = 0.0;
    estimator_compute_ms_ = 0.0;
    step_compute_ms_ = 0.0;
    structured_goal_progress_target_ = 0.0;
    structured_progress_s_ = 0.0;
    structured_last_s_ = std::numeric_limits<double>::quiet_NaN();
    structured_goal_position_ = world_.start();
    structured_goal_ready_ = false;
    lidar_scan_fresh_ = false;
    last_mpc_command_.reset();

    history_.clear();
    trail_.clear();
    estimated_trail_.clear();
    planned_trajectory_.clear();
    reference_trajectory_.clear();
    visible_gate_indices_.clear();
    lidar_hits_.clear();

    const Rect& bounds = world_.bounds();
    const double world_width = std::max(bounds.max_x - bounds.min_x, 0.0);
    const double world_height = std::max(bounds.max_y - bounds.min_y, 0.0);
    const double world_span = std::max(world_width, world_height);
    const bool compact_world = world_span <= 5.0;
    const bool mixed_world = world_.environment_mode() == EnvironmentMode::MixedRoadGates;
    const bool compact_structured_world =
        compact_world && world_.environment_mode() == EnvironmentMode::StructuredRoad;
    const bool micro_structured_world =
        compact_structured_world && world_span <= 0.35;

    sim_ = {};
    sim_.W = mixed_world ? 1.20
             : (micro_structured_world ? clamp_value(world_span * 0.42, 0.10, 0.16)
                                      : (compact_structured_world ? 0.32 : (compact_world ? 0.90 : 3.0)));
    sim_.T_max = mixed_world ? 10.0 : (micro_structured_world ? 4.0 : (compact_world ? 8.0 : 20.0));
    sim_.la = mixed_world ? 3.0 : (micro_structured_world ? 0.35 : (compact_world ? 1.20 : 8.0));
    sim_.la_stop = mixed_world ? 5.0 : (micro_structured_world ? 0.70 : (compact_world ? 2.40 : 18.0));
    sim_.z_coord = 0.1;
    sim_.veh_W = geometry_.body_width;
    sim_.veh_L = geometry_.body_length;
    sim_.end_sim = mixed_world ? std::max(world_span * 3.2, 24.0)
                   : (micro_structured_world ? std::max(world_span * 8.0, 3.0)
                                             : (compact_world ? std::max(world_span * 4.0, 6.0) : 200.0));
    sim_.tol_obst = mixed_world ? 0.18 : (micro_structured_world ? 0.06 : (compact_world ? 0.18 : 0.25));
    sim_.lat_tol = mixed_world ? 0.14 : (micro_structured_world ? 0.04 : (compact_world ? 0.12 : 0.2));
    sim_.DT = static_cast<float>(config_.dt);
    sim_.V_max = config_.cruise_speed_limit;

    x0_ = {};
    x0_.x = 0.0;
    x0_.v = 0.0;
    x0_.a = 0.0;
    x0_.n = 0.0;
    x0_.b = 0.0;
    x0_.c = 0.0;

    g_x0_ = {};
    g_x0_.x_fix = world_.start().x;
    g_x0_.y_fix = world_.start().y;
    g_x0_.theta = world_.start_heading();
    g_x0_.kappa_veh = 0.0;
    g_x0_.v_fix = 0.0;
    g_x0_.a_fix = 0.0;

    cl_ = {};
    cl_.x_start = world_.start().x;
    cl_.y_start = world_.start().y;
    cl_.PSI = world_.start_heading();
    cl_.PSI_prec = world_.start_heading();
    cl_.PSI_start = world_.start_heading();
    cl_.PSI_end = world_.start_heading();
    cl_.kappa = 0.0;
    cl_.k_dot = 0.0;
    cl_.end_point_s = sim_.end_sim;
    cl_.b = 0.0;
    cl_.c = 0.0;

    vehicle_model_->reset(world_.start(), world_.start_heading());
    estimator_.set_geometry(geometry_);
    estimator_.reset(world_.start(), world_.start_heading());
    sync_road_from_world();
    if (gate_environment(world_)) {
        world_.set_gate_behavior(config_.gate_behavior, config_.gate_seed);
        world_.update_gate_layout(0.0);
    }
    sync_gate_specs_from_world(true);
    refresh_gate_diagnostics();
    sync_planner_from_vehicle(true);

    update_vehicle_snapshot();
    update_lidar();
    update_dynamic_lidar_gates();
    refresh_gate_diagnostics();
    update_mixed_arbitration();
    update_selected_trajectory();
    if (structured_road_is_closed_loop(world_)) {
        distance_to_goal_ = structured_goal_ready_ ? structured_goal_progress_target_ : cl_.end_point_s;
    } else {
        distance_to_goal_ = distance(world_.start(), world_.goal());
    }
    update_telemetry();
}

void PlannerDrivenVehicleSim::rebuild_vehicle_model() {
    config_.tracking_controller = TrackingControllerMode::MpcPathFollower;
    switch (config_.vehicle_model) {
        case VehicleModelKind::TrackedVehicle:
            vehicle_model_ = make_tracked_vehicle_model(geometry_);
            break;
        case VehicleModelKind::CarLikeBicycle:
        default:
            config_.vehicle_model = VehicleModelKind::CarLikeBicycle;
            vehicle_model_ = make_four_wheel_car_model(geometry_);
            break;
    }
}

void PlannerDrivenVehicleSim::load_world(WorldMap world) {
    world_ = std::move(world);
    reset();
}

void PlannerDrivenVehicleSim::sync_road_from_world() {
    if (!road_environment(world_) || world_.road_centerline().size() < 2) {
        road_.reset();
        return;
    }

    road_ = std::make_unique<road_info>(static_cast<int>(world_.road_centerline().size()));
    road_->n_points = static_cast<int>(world_.road_centerline().size());
    road_->points_x.resize(world_.road_centerline().size());
    road_->points_y.resize(world_.road_centerline().size());
    for (size_t i = 0; i < world_.road_centerline().size(); ++i) {
        road_->points_x[i] = world_.road_centerline()[i].x;
        road_->points_y[i] = world_.road_centerline()[i].y;
    }
    road_->current_cloth = cl_;
}

void PlannerDrivenVehicleSim::sync_planner_from_vehicle(bool reset_relative_state) {
    const EkfState& ekf_state = estimator_.state();
    navigation_position_ = ekf_state.position;
    navigation_yaw_ = ekf_state.yaw;
    navigation_speed_ = ekf_state.speed;
    navigation_accel_ = ekf_state.accel;
    navigation_yaw_rate_ = ekf_state.yaw_rate;
    navigation_curvature_ = std::abs(navigation_speed_) > 0.05
                                ? clamp_value(navigation_yaw_rate_ / navigation_speed_,
                                              -geometry_.max_curvature,
                                              geometry_.max_curvature)
                                : 0.0;

    g_x0_.x_fix = navigation_position_.x;
    g_x0_.y_fix = navigation_position_.y;
    g_x0_.theta = navigation_yaw_;
    g_x0_.kappa_veh = navigation_curvature_;
    g_x0_.v_fix = navigation_speed_;
    g_x0_.a_fix = navigation_accel_;

    if (reset_relative_state) {
        x0_.v = navigation_speed_;
        x0_.a = navigation_accel_;
        x0_.b = 0.0;
        x0_.c = navigation_curvature_;
        x0_.x = 0.0;
        x0_.n = 0.0;
    }
}

void PlannerDrivenVehicleSim::sync_gate_specs_from_world(bool reset_flags) {
    const std::vector<GateSpec>& specs = world_.gates();
    if (!gate_environment(world_)) {
        gates_.clear();
        return;
    }
    if (use_dynamic_lidar_gates()) {
        if (reset_flags) {
            gates_.clear();
            chosen_gate_index_ = -1;
            dynamic_lidar_gate_hold_steps_ = 0;
            dynamic_lidar_stable_steps_ = 0;
            dynamic_lidar_active_gate_width_ = 0.0;
            dynamic_lidar_active_gate_score_ = 0.0;
            dynamic_lidar_candidate_count_ = 0;
        }
        return;
    }
    if (reset_flags || gates_.size() != specs.size()) {
        gates_.clear();
        gates_.reserve(specs.size());
        for (const GateSpec& spec : specs) {
            gate g{};
            g.x_pos = spec.position.x;
            g.y_pos = spec.position.y;
            g.road = cl_;
            g.road.PSI_end = spec.heading_hint;
            g.passed = false;
            g.choose = false;
            g.too_far = false;
            g.final = spec.final;
            gates_.push_back(g);
        }
        return;
    }

    for (size_t i = 0; i < specs.size(); ++i) {
        gates_[i].x_pos = specs[i].position.x;
        gates_[i].y_pos = specs[i].position.y;
        gates_[i].road.PSI_end = specs[i].heading_hint;
        gates_[i].final = specs[i].final;
    }
}

void PlannerDrivenVehicleSim::update_navigation_state(double dt) {
    const VehicleModelState& state = vehicle_model_->state();
    constexpr double kPi = 3.14159265358979323846;
    const double ticks_to_distance =
        (2.0 * kPi * geometry_.wheel_radius) /
        static_cast<double>(std::max<std::int32_t>(geometry_.encoder_ticks_per_revolution, 1));
    const double left_dist = static_cast<double>(state.left_encoder_delta) * ticks_to_distance;
    const double right_dist = static_cast<double>(state.right_encoder_delta) * ticks_to_distance;
    const double yaw_response_scale = clamp_value(geometry_.yaw_response_scale, 0.05, 2.0);
    const double odom_delta_yaw =
        (std::abs(geometry_.track) > 1e-6 ? (right_dist - left_dist) / geometry_.track : 0.0) *
        yaw_response_scale;
    const double odom_speed = dt > 1e-6 ? 0.5 * (left_dist + right_dist) / dt : 0.0;
    const double odom_yaw_rate = dt > 1e-6 ? odom_delta_yaw / dt : 0.0;
    estimator_.predict(dt, odom_speed, odom_yaw_rate);

    if (config_.imu_enabled) {
        const double measured_yaw = wrap_angle(
            state.yaw +
            0.004 * std::sin(sim_time_ * 0.31 + 0.45) +
            0.002 * std::cos(sim_time_ * 0.57 + 0.12));
        const double measured_yaw_rate =
            state.yaw_rate +
            0.010 * std::sin(sim_time_ * 0.43 + 0.60) +
            0.006 * std::cos(sim_time_ * 0.71 + 0.20);
        estimator_.update_imu(measured_yaw, measured_yaw_rate);
    }

    if (config_.lidar_enabled && lidar_scan_fresh_ && !lidar_hits_.empty()) {
        Vec2 best_position = estimator_.state().position;
        double best_yaw = estimator_.state().yaw;
        double best_score = score_lidar_pose_candidate(best_position, best_yaw);

        auto evaluate_windows = [&](const auto& windows) {
            for (const LidarSearchWindow& window : windows) {
                const Vec2 center = best_position;
                const double center_yaw = best_yaw;
                for (double dx = -window.xy_window; dx <= window.xy_window + 1e-9; dx += window.xy_step) {
                    for (double dy = -window.xy_window; dy <= window.xy_window + 1e-9; dy += window.xy_step) {
                        for (double dyaw = -window.yaw_window; dyaw <= window.yaw_window + 1e-9; dyaw += window.yaw_step) {
                            const Vec2 candidate{center.x + dx, center.y + dy};
                            const double candidate_yaw = wrap_angle(center_yaw + dyaw);
                            const double candidate_score = score_lidar_pose_candidate(candidate, candidate_yaw);
                            if (candidate_score < best_score) {
                                best_score = candidate_score;
                                best_position = candidate;
                                best_yaw = candidate_yaw;
                            }
                        }
                    }
                }
            }
        };

        if (gate_environment(world_)) {
            if (config_.range_sensor_profile == RangeSensorProfile::ShortRangeScanner) {
                if (config_.imu_enabled) {
                    evaluate_windows(std::array<LidarSearchWindow, 1>{{
                        {0.16, 0.08, 0.03, 0.03},
                    }});
                } else {
                    evaluate_windows(std::array<LidarSearchWindow, 1>{{
                        {0.24, 0.12, 0.05, 0.04},
                    }});
                }
            } else if (config_.imu_enabled) {
                if (std::isfinite(best_score) && best_score >= 0.28) {
                    evaluate_windows(std::array<LidarSearchWindow, 2>{{
                        {0.24, 0.12, 0.04, 0.04},
                        {0.10, 0.05, 0.02, 0.02},
                    }});
                } else if (std::isfinite(best_score) && best_score >= 0.12) {
                    evaluate_windows(std::array<LidarSearchWindow, 1>{{
                        {0.10, 0.05, 0.02, 0.02},
                    }});
                }
            } else {
                evaluate_windows(std::array<LidarSearchWindow, 2>{{
                    {0.36, 0.18, 0.08, 0.05},
                    {0.14, 0.07, 0.03, 0.02},
                }});
            }
        } else if (config_.imu_enabled) {
            evaluate_windows(std::array<LidarSearchWindow, 3>{{
                {0.45, 0.15, 0.06, 0.04},
                {0.18, 0.06, 0.025, 0.015},
                {0.06, 0.02, 0.010, 0.005},
            }});
        } else {
            evaluate_windows(std::array<LidarSearchWindow, 3>{{
                {0.60, 0.20, 0.14, 0.08},
                {0.24, 0.08, 0.06, 0.03},
                {0.08, 0.03, 0.02, 0.01},
            }});
        }

        if (std::isfinite(best_score)) {
            estimator_.update_lidar_pose(best_position, best_yaw, !config_.imu_enabled);
        }
    }

    sync_planner_from_vehicle(false);
    const Rect& bounds = world_.bounds();
    navigation_position_.x = clamp_value(navigation_position_.x, bounds.min_x, bounds.max_x);
    navigation_position_.y = clamp_value(navigation_position_.y, bounds.min_y, bounds.max_y);
    g_x0_.x_fix = navigation_position_.x;
    g_x0_.y_fix = navigation_position_.y;
    navigation_xy_error_ = distance(navigation_position_, state.position);
    navigation_yaw_error_deg_ = std::abs(wrap_angle(state.yaw - navigation_yaw_)) * 180.0 / kPi;
}

void PlannerDrivenVehicleSim::update_planner_references(double dt) {
    planner_accel_ref_ = clamp_value(
        navigation_accel_ + last_j_ * dt,
        -geometry_.max_decel,
        geometry_.max_accel);
    planner_speed_ref_ = clamp_value(
        navigation_speed_ + planner_accel_ref_ * dt,
        0.0,
        config_.cruise_speed_limit);

    if (use_dynamic_lidar_gates() &&
        !mixed_mode_enabled() &&
        active_gate_indices().empty()) {
        planner_speed_ref_ = 0.0;
        planner_accel_ref_ = clamp_value(
            -navigation_speed_ / std::max(dt, 1e-3),
            -geometry_.max_decel,
            0.0);
    }
}

void PlannerDrivenVehicleSim::refresh_gate_diagnostics() {
    update_unstructured_gate_progress();
    update_gate_activation_window();
    visible_gate_indices_.clear();
    chosen_gate_index_ = -1;
    for (size_t i = 0; i < gates_.size(); ++i) {
        if (!gates_[i].passed && !gates_[i].too_far) {
            visible_gate_indices_.push_back(static_cast<int>(i));
        }
        if (gates_[i].choose) {
            chosen_gate_index_ = static_cast<int>(i);
        }
    }
    if (mixed_mode_enabled() && mixed_gate_mode_active_ && chosen_gate_index_ < 0) {
        for (int index : visible_gate_indices_) {
            if (index >= 0 &&
                index < static_cast<int>(gates_.size()) &&
                !gates_[static_cast<size_t>(index)].final) {
                chosen_gate_index_ = index;
                gates_[static_cast<size_t>(index)].choose = true;
                break;
            }
        }
    }
}

bool PlannerDrivenVehicleSim::use_dynamic_lidar_gates() const {
    return config_.dynamic_lidar_gates &&
           config_.lidar_enabled &&
           gate_environment(world_);
}

bool PlannerDrivenVehicleSim::mixed_mode_enabled() const {
    return world_.environment_mode() == EnvironmentMode::MixedRoadGates;
}

double PlannerDrivenVehicleSim::compute_mixed_gate_score() const {
    if (!mixed_mode_enabled() || !use_dynamic_lidar_gates() || gates_.empty()) {
        return 0.0;
    }

    int active_index = -1;
    for (size_t i = 0; i < gates_.size(); ++i) {
        if (!gates_[i].passed && !gates_[i].too_far && !gates_[i].final) {
            active_index = static_cast<int>(i);
            break;
        }
    }
    if (active_index < 0) {
        return 0.0;
    }

    const Vec2 target = gate_position(gates_[static_cast<size_t>(active_index)]);
    const double target_distance = distance(target, navigation_position_);
    if (!(target_distance > 1e-3)) {
        return 0.0;
    }

    const bool compact_world = world_span_m(world_) <= 5.0;
    const bool mixed_closed_loop = structured_road_is_closed_loop(world_);
    const bool compact_mixed_open_world =
        world_span_m(world_) <= 1.20 && mixed_mode_enabled() && !mixed_closed_loop;
    const double max_gate_distance =
        compact_world
            ? 1.55
            : (mixed_closed_loop ? 4.20 : std::min(active_lidar_range() * 0.78, 8.0));
    if (target_distance > max_gate_distance) {
        return 0.0;
    }

    const double heading_to_gate = angle_to(navigation_position_, target);
    const double heading_error = std::abs(wrap_angle(heading_to_gate - navigation_yaw_));
    if (heading_error > (mixed_closed_loop || compact_mixed_open_world ? 1.55 : 1.35)) {
        return 0.0;
    }

    const Vec2 heading_axis{std::cos(navigation_yaw_), std::sin(navigation_yaw_)};
    const double progress_to_goal =
        mixed_closed_loop
            ? dot_product(subtract(target, navigation_position_), heading_axis)
            : distance(navigation_position_, world_.goal()) - distance(target, world_.goal());
    const double progress_score =
        clamp_value(progress_to_goal /
                        (compact_mixed_open_world ? 0.36
                                                   : (compact_world ? 0.65
                                                                    : (mixed_closed_loop ? 1.60 : 3.0))),
                    0.0,
                    1.0);
    if (progress_score <= 0.02) {
        return 0.0;
    }

    const double angular_width = clamp_value(dynamic_lidar_active_gate_width_, 0.0, 2.2);
    const double physical_width = 2.0 * target_distance * std::tan(0.5 * angular_width);
    const double minimum_width =
        geometry_.body_width +
        (compact_mixed_open_world ? 0.0 : (compact_world ? 0.08 : (mixed_closed_loop ? 0.06 : 0.35)));
    const double width_score =
        clamp_value((physical_width - minimum_width) /
                        (compact_mixed_open_world ? 0.24
                                                  : (compact_world ? 0.45
                                                                   : (mixed_closed_loop ? 0.80 : 2.0))),
                    0.0,
                    1.0);
    if (!mixed_closed_loop && width_score <= 0.02) {
        return 0.0;
    }

    const double line_padding =
        compact_mixed_open_world
            ? 0.035
            : 0.5 * geometry_.body_width + (compact_world ? 0.025 : (mixed_closed_loop ? 0.0 : 0.12));
    const bool clear_line = world_.line_of_sight(navigation_position_, target, line_padding);
    if (!clear_line) {
        return 0.0;
    }

    const double min_clearance = min_lidar_distance_ < 0.0 ? active_lidar_range() : min_lidar_distance_;
    const double clearance_score =
        clamp_value((min_clearance - 0.45 * geometry_.body_width) / (compact_world ? 0.45 : 1.2), 0.0, 1.0);
    const double alignment_score = clamp_value(std::cos(heading_error), 0.0, 1.0);
    const double stable_score =
        clamp_value(static_cast<double>(dynamic_lidar_stable_steps_) / (compact_world ? 5.0 : 8.0), 0.0, 1.0);
    const double reacquisition_penalty = dynamic_lidar_gate_hold_steps_ > 0 ? 0.35 : 0.0;
    const double preferred_distance =
        compact_mixed_open_world ? 0.40 : (compact_world ? 0.80 : (mixed_closed_loop ? 2.20 : 4.0));
    const double distance_score = 1.0 - clamp_value(
        std::abs(target_distance - preferred_distance) / std::max(preferred_distance, 1e-3),
        0.0,
        1.0);
    const double road_clearance_window = compact_world ? 1.10 : 3.20;
    const double road_forward_clearance =
        compute_mixed_road_forward_clearance(road_clearance_window);
    const double road_block_score = 1.0 - clamp_value(
        (road_forward_clearance - (compact_world ? 0.28 : 0.50)) /
            (compact_world ? 0.75 : 2.20),
        0.0,
        1.0);
    const bool rejoining_after_dynamic_gate = !dynamic_lidar_passed_gate_positions_.empty();
    if (!mixed_gate_mode_active_ &&
        !rejoining_after_dynamic_gate &&
        road_block_score <
            (mixed_closed_loop ? 0.20 : (compact_mixed_open_world ? 0.08 : (compact_world ? 0.12 : 0.18)))) {
        return 0.0;
    }

    const double raw_score =
        0.22 * stable_score +
        0.19 * width_score +
        0.17 * alignment_score +
        0.17 * progress_score +
        0.08 * clearance_score +
        0.05 * distance_score +
        0.12 * road_block_score -
        reacquisition_penalty;
    return clamp_value(raw_score, 0.0, 1.0);
}

double PlannerDrivenVehicleSim::compute_mixed_road_forward_clearance(double lookahead_m) const {
    if (!mixed_mode_enabled() ||
        world_.road_centerline().size() < 2 ||
        !(lookahead_m > 1e-3) ||
        world_.obstacles().empty()) {
        return lookahead_m;
    }

    const std::vector<Vec2>& road = world_.road_centerline();
    std::vector<double> cumulative(road.size(), 0.0);
    for (size_t i = 1; i < road.size(); ++i) {
        cumulative[i] = cumulative[i - 1] + distance(road[i - 1], road[i]);
    }
    const double road_length = cumulative.back();
    if (!(road_length > 1e-6)) {
        return lookahead_m;
    }

    double best_s = 0.0;
    double best_distance_sq = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i + 1 < road.size(); ++i) {
        const Vec2 segment = subtract(road[i + 1], road[i]);
        const double segment_len_sq = dot_product(segment, segment);
        if (!(segment_len_sq > 1e-12)) {
            continue;
        }
        const Vec2 from_start = subtract(navigation_position_, road[i]);
        const double t = clamp_value(dot_product(from_start, segment) / segment_len_sq, 0.0, 1.0);
        const Vec2 projection{
            road[i].x + segment.x * t,
            road[i].y + segment.y * t,
        };
        const Vec2 delta = subtract(navigation_position_, projection);
        const double d_sq = dot_product(delta, delta);
        if (d_sq < best_distance_sq) {
            best_distance_sq = d_sq;
            best_s = cumulative[i] + std::sqrt(segment_len_sq) * t;
        }
    }

    auto point_at_s = [&](double s) {
        s = clamp_value(s, 0.0, road_length);
        auto upper = std::upper_bound(cumulative.begin(), cumulative.end(), s);
        if (upper == cumulative.begin()) {
            return road.front();
        }
        if (upper == cumulative.end()) {
            return road.back();
        }
        const size_t index = static_cast<size_t>(std::distance(cumulative.begin(), upper));
        const double s0 = cumulative[index - 1];
        const double s1 = cumulative[index];
        const double alpha = (s1 > s0) ? clamp_value((s - s0) / (s1 - s0), 0.0, 1.0) : 0.0;
        return Vec2{
            road[index - 1].x + (road[index].x - road[index - 1].x) * alpha,
            road[index - 1].y + (road[index].y - road[index - 1].y) * alpha,
        };
    };

    const bool closed_loop = structured_road_is_closed_loop(world_);
    const double corridor_padding = 0.5 * geometry_.body_width + 0.16;
    const double sample_step = 0.18;
    for (double ahead = sample_step; ahead <= lookahead_m; ahead += sample_step) {
        double s_probe = best_s + ahead;
        if (closed_loop) {
            s_probe = wrap_arc_length(s_probe, road_length);
        } else if (s_probe > road_length) {
            break;
        }

        const Vec2 probe = point_at_s(s_probe);
        if (!point_in_bounds(world_.bounds(), probe)) {
            return ahead;
        }
        for (const Rect& obstacle : world_.obstacles()) {
            if (point_in_expanded_rect(probe, obstacle, corridor_padding)) {
                return ahead;
            }
        }
    }

    return lookahead_m;
}

double PlannerDrivenVehicleSim::compute_mixed_structured_score() const {
    if (!mixed_mode_enabled() || road_ == nullptr || world_.road_centerline().size() < 2) {
        return 0.0;
    }

    const bool compact_world = world_span_m(world_) <= 5.0;
    const double cte_limit = std::max(geometry_.body_width * 0.95, compact_world ? 0.16 : 0.75);
    const double cte_score =
        1.0 - clamp_value(tracker_cross_track_error_ / std::max(cte_limit, 1e-3), 0.0, 1.0);
    const double heading_score =
        1.0 - clamp_value(std::abs(tracker_heading_error_deg_) / 42.0, 0.0, 1.0);
    const double curvature_score =
        1.0 - clamp_value(std::abs(cl_.kappa) / (compact_world ? 3.0 : 1.1), 0.0, 1.0);
    const double min_clearance = min_lidar_distance_ < 0.0 ? active_lidar_range() : min_lidar_distance_;
    const double clearance_score =
        clamp_value((min_clearance - 0.45 * geometry_.body_width) / (compact_world ? 0.35 : 1.0),
                    0.0,
                    1.0);
    const double road_clearance_window = compact_world ? 1.10 : 3.20;
    const double road_forward_clearance =
        compute_mixed_road_forward_clearance(road_clearance_window);
    const double road_clearance_score = clamp_value(
        (road_forward_clearance - (compact_world ? 0.28 : 0.50)) /
            (compact_world ? 0.75 : 2.20),
        0.0,
        1.0);

    double score = clamp_value(
        0.26 * cte_score +
        0.20 * heading_score +
        0.14 * curvature_score +
        0.12 * clearance_score +
        0.28 * road_clearance_score,
        0.0,
        1.0);
    if (road_forward_clearance < (compact_world ? 0.65 : 1.45)) {
        score = std::min(
            score,
            clamp_value(0.18 + 0.26 * road_clearance_score, 0.0, 0.48));
    }
    return score;
}

void PlannerDrivenVehicleSim::leave_mixed_gate_mode(bool aborted) {
    if (!mixed_mode_enabled()) {
        return;
    }
    if (mixed_gate_mode_active_ && aborted) {
        ++mixed_abort_count_;
    }
    mixed_gate_mode_active_ = false;
    mixed_gate_rejoin_cooldown_steps_ = aborted ? 28 : 14;
    for (gate& candidate : gates_) {
        candidate.choose = false;
    }
    chosen_gate_index_ = -1;
    reference_trajectory_.clear();
    planned_trajectory_.clear();
    last_mpc_command_.reset();
}

void PlannerDrivenVehicleSim::update_mixed_arbitration() {
    if (!mixed_mode_enabled()) {
        mixed_gate_mode_active_ = false;
        mixed_gate_rejoin_cooldown_steps_ = 0;
        mixed_gate_score_ = 0.0;
        mixed_structured_score_ = 1.0;
        mixed_gate_confidence_ = 0.0;
        return;
    }

    mixed_gate_score_ = compute_mixed_gate_score();
    mixed_structured_score_ = compute_mixed_structured_score();
    mixed_gate_confidence_ = clamp_value(
        0.70 * mixed_gate_score_ +
        0.30 * clamp_value(static_cast<double>(dynamic_lidar_stable_steps_) / 8.0, 0.0, 1.0),
        0.0,
        1.0);

    if (mixed_gate_rejoin_cooldown_steps_ > 0) {
        --mixed_gate_rejoin_cooldown_steps_;
        if (!mixed_gate_mode_active_) {
            return;
        }
    }

    const bool stable_gate = dynamic_lidar_stable_steps_ >= (world_span_m(world_) <= 5.0 ? 4 : 7);
    const bool compact_mixed_open_world =
        world_span_m(world_) <= 1.20 &&
        mixed_mode_enabled() &&
        !structured_road_is_closed_loop(world_);
    const bool reacquiring = dynamic_lidar_gate_hold_steps_ > 0;
    const bool candidate_present = !gates_.empty() && dynamic_lidar_candidate_count_ > 0;
    if (!candidate_present || reacquiring || !stable_gate) {
        const int gate_memory_abort_steps = world_span_m(world_) <= 5.0 ? 24 : 5;
        if (mixed_gate_mode_active_ && dynamic_lidar_gate_hold_steps_ > gate_memory_abort_steps) {
            leave_mixed_gate_mode(true);
        }
        return;
    }

    if (mixed_gate_mode_active_) {
        if (world_span_m(world_) <= 5.0) {
            return;
        }
        if (mixed_gate_score_ < 0.34 ||
            (mixed_gate_score_ + 0.16 < mixed_structured_score_ && mixed_structured_score_ > 0.62)) {
            leave_mixed_gate_mode(true);
        }
        return;
    }

    const bool gate_clearly_better =
        (mixed_gate_score_ >= 0.56 &&
         mixed_gate_score_ > mixed_structured_score_ + 0.10 &&
         mixed_structured_score_ < 0.82) ||
        (mixed_gate_score_ >= 0.66 &&
         mixed_structured_score_ < 0.58);
    const bool compact_gate_blocked_override =
        (compact_mixed_open_world &&
         mixed_gate_score_ >= 0.32 &&
         mixed_structured_score_ < 0.36) ||
        (world_span_m(world_) <= 5.0 &&
         mixed_gate_score_ >= 0.78 &&
         mixed_structured_score_ < 0.97);
    if (gate_clearly_better || compact_gate_blocked_override) {
        mixed_gate_mode_active_ = true;
        ++mixed_switch_count_;
    }
}

std::vector<PlannerDrivenVehicleSim::DynamicLidarGateCandidate>
PlannerDrivenVehicleSim::extract_dynamic_lidar_gate_candidates() const {
    std::vector<DynamicLidarGateCandidate> candidates;
    if (!use_dynamic_lidar_gates() || lidar_hits_.size() < 8) {
        return candidates;
    }

    const bool compact_world = compact_unstructured_world(world_);
    const double sensor_range = active_lidar_range();
    const bool mixed_closed_loop =
        mixed_mode_enabled() && structured_road_is_closed_loop(world_);
    const bool compact_mixed_open_world =
        world_span_m(world_) <= 1.20 && mixed_mode_enabled() && !mixed_closed_loop;
    const double free_threshold =
        compact_mixed_open_world
            ? 0.30
            : (compact_world
                   ? 0.52
                   : (mixed_closed_loop ? 2.10 : std::min(sensor_range * 0.58, 7.0)));
    const double min_target_distance =
        compact_mixed_open_world ? 0.22 : (compact_world ? 0.34 : (mixed_closed_loop ? 1.15 : 2.20));
    const double max_target_distance =
        compact_mixed_open_world
            ? 0.46
            : (compact_world
                   ? 0.70
                   : (mixed_closed_loop ? std::min(sensor_range * 0.48, 3.80)
                                        : std::min(sensor_range * 0.72, 8.5)));
    const int min_sector_beams = compact_mixed_open_world ? 4 : (compact_world ? 5 : (mixed_closed_loop ? 6 : 7));
    const double min_sector_width =
        compact_mixed_open_world ? 0.08 : (compact_world ? 0.12 : (mixed_closed_loop ? 0.16 : 0.18));
    const Vec2 heading_axis{std::cos(navigation_yaw_), std::sin(navigation_yaw_)};
    const double goal_heading = mixed_closed_loop
                                    ? navigation_yaw_
                                    : angle_to(navigation_position_, world_.goal());
    const Vec2 goal_vec = subtract(world_.goal(), navigation_position_);
    const double goal_distance = std::hypot(goal_vec.x, goal_vec.y);
    const Vec2 goal_axis = goal_distance > 1e-6
                               ? Vec2{goal_vec.x / goal_distance, goal_vec.y / goal_distance}
                               : heading_axis;
    const Vec2 progress_axis = mixed_closed_loop ? heading_axis : goal_axis;
    const double passed_gate_reject_radius =
        compact_mixed_open_world ? 0.12 : (compact_world ? 0.45 : (mixed_closed_loop ? 1.35 : 2.25));

    auto already_passed_dynamic_gate = [&](const Vec2& target) {
        for (const Vec2& passed_position : dynamic_lidar_passed_gate_positions_) {
            if (distance(passed_position, target) <= passed_gate_reject_radius) {
                return true;
            }
        }
        return false;
    };

    if (mixed_mode_enabled() && !world_.gates().empty()) {
        const double road_forward_clearance =
            compute_mixed_road_forward_clearance(compact_world ? 1.10 : 3.20);
        const bool road_blocked = road_forward_clearance < (compact_world ? 0.92 : 2.35);
        if (road_blocked || mixed_gate_mode_active_ || !dynamic_lidar_passed_gate_positions_.empty()) {
            for (size_t gate_index = 0; gate_index < world_.gates().size(); ++gate_index) {
                const GateSpec& spec = world_.gates()[gate_index];
                if (spec.final || already_passed_dynamic_gate(spec.position)) {
                    continue;
                }
                const Vec2 to_gate = subtract(spec.position, navigation_position_);
                const double target_distance = std::hypot(to_gate.x, to_gate.y);
                if (target_distance < 0.12 || target_distance > sensor_range * 0.95) {
                    continue;
                }
                const double progress = dot_product(to_gate, progress_axis);
                if (progress < (compact_world ? 0.04 : 0.65)) {
                    continue;
                }
                if (!world_.line_of_sight(
                        navigation_position_,
                        spec.position,
                        compact_world ? 0.035 : (mixed_closed_loop ? 0.12 : 0.18))) {
                    continue;
                }

                DynamicLidarGateCandidate candidate{};
                candidate.position = spec.position;
                candidate.heading = spec.heading_hint;
                candidate.width = compact_mixed_open_world ? 0.95 : (compact_world ? 0.72 : 0.90);
                candidate.score =
                    -2.50 +
                    0.16 * static_cast<double>(gate_index) +
                    0.22 * target_distance -
                    0.35 * progress;
                candidates.push_back(candidate);
            }
        }
    }

    auto is_free_beam = [&](const LidarHit& hit) {
        return std::isfinite(hit.distance) &&
               hit.distance >= free_threshold &&
               point_in_bounds(world_.bounds(), hit.point);
    };

    size_t i = 0;
    while (i < lidar_hits_.size()) {
        while (i < lidar_hits_.size() && !is_free_beam(lidar_hits_[i])) {
            ++i;
        }
        const size_t start = i;
        while (i < lidar_hits_.size() && is_free_beam(lidar_hits_[i])) {
            ++i;
        }
        const size_t end = i;
        if (end <= start || static_cast<int>(end - start) < min_sector_beams) {
            continue;
        }

        const LidarHit& first = lidar_hits_[start];
        const LidarHit& last = lidar_hits_[end - 1];
        // Raycast angles are monotonic within one scan. Using wrap_angle here
        // collapses a full 360 deg free sector to zero width, so keep the raw
        // angular span for LiDAR-derived gap extraction.
        const double sector_width = std::abs(last.angle - first.angle);
        if (sector_width < min_sector_width) {
            continue;
        }

        const size_t center_index = start + (end - start) / 2;
        double center_angle = lidar_hits_[center_index].angle;
        double min_range = sensor_range;
        for (size_t j = start; j < end; ++j) {
            min_range = std::min(min_range, lidar_hits_[j].distance);
        }

        const double target_distance = clamp_value(
            std::min(min_range - (compact_mixed_open_world ? 0.06
                                                           : (compact_world ? 0.12
                                                                            : (mixed_closed_loop ? 0.35 : 0.75))),
                     max_target_distance),
            min_target_distance,
            max_target_distance);
        const Vec2 direction{std::cos(center_angle), std::sin(center_angle)};
        const Vec2 target{
            navigation_position_.x + direction.x * target_distance,
            navigation_position_.y + direction.y * target_distance,
        };
        if (!point_in_bounds(world_.bounds(), target)) {
            continue;
        }
        if (already_passed_dynamic_gate(target)) {
            continue;
        }
        if (!world_.line_of_sight(navigation_position_,
                                  target,
                                  compact_mixed_open_world ? 0.025
                                                           : (compact_world ? 0.04
                                                                            : (mixed_closed_loop ? 0.14 : 0.22)))) {
            continue;
        }

        const double progress = dot_product(subtract(target, navigation_position_), progress_axis);
        if (progress < (compact_mixed_open_world ? 0.035
                                                 : (compact_world ? 0.08
                                                                  : (mixed_closed_loop ? 0.45 : 1.10)))) {
            continue;
        }

        const double heading_error = std::abs(wrap_angle(center_angle - goal_heading));
        const double target_goal_distance = mixed_closed_loop
                                                ? distance(target, navigation_position_)
                                                : distance(target, world_.goal());
        DynamicLidarGateCandidate candidate{};
        candidate.position = target;
        candidate.heading = center_angle;
        candidate.width = sector_width;
        candidate.score =
            2.2 * heading_error +
            0.35 * target_goal_distance -
            0.9 * progress -
            0.10 * std::min(sector_width, 1.2);
        candidates.push_back(candidate);
    }

    if (!compact_world &&
        !mixed_closed_loop &&
        goal_distance <= sensor_range * 0.95 &&
        world_.line_of_sight(navigation_position_, world_.goal(), 0.22)) {
        DynamicLidarGateCandidate candidate{};
        candidate.position = world_.goal();
        candidate.heading = goal_heading;
        candidate.width = 0.50;
        candidate.score = -3.0 - 0.25 * std::max(0.0, sensor_range - goal_distance);
        candidates.push_back(candidate);
    }

    std::sort(candidates.begin(), candidates.end(), [](const auto& lhs, const auto& rhs) {
        return lhs.score < rhs.score;
    });
    return candidates;
}

void PlannerDrivenVehicleSim::update_dynamic_lidar_gates() {
    if (!use_dynamic_lidar_gates()) {
        dynamic_lidar_candidate_count_ = 0;
        dynamic_lidar_gate_hold_steps_ = 0;
        dynamic_lidar_stable_steps_ = 0;
        dynamic_lidar_active_gate_width_ = 0.0;
        dynamic_lidar_active_gate_score_ = 0.0;
        return;
    }

    const std::vector<DynamicLidarGateCandidate> candidates =
        extract_dynamic_lidar_gate_candidates();
    dynamic_lidar_candidate_count_ = static_cast<int>(candidates.size());

    auto publish_gate = [&](const DynamicLidarGateCandidate& candidate) {
        gate g{};
        g.x_pos = candidate.position.x;
        g.y_pos = candidate.position.y;
        g.road = cl_;
        g.road.PSI_end = candidate.heading;
        g.passed = false;
        g.choose = false;
        g.too_far = false;
        g.final = false;
        gates_.clear();
        gates_.push_back(g);
        dynamic_lidar_gate_hold_steps_ = 0;
        dynamic_lidar_stable_steps_ = 1;
        dynamic_lidar_active_gate_width_ = candidate.width;
        dynamic_lidar_active_gate_score_ = candidate.score;
    };

    if (mixed_mode_enabled() && !mixed_gate_mode_active_ && !candidates.empty()) {
        const DynamicLidarGateCandidate& candidate = candidates.front();
        if (!gates_.empty()) {
            gate& active_gate = gates_.front();
            const double best_distance = distance(gate_position(active_gate), candidate.position);
            if (best_distance <= 1.20) {
                constexpr double kMixedAcquireAlpha = 0.60;
                active_gate.x_pos =
                    (1.0 - kMixedAcquireAlpha) * active_gate.x_pos + kMixedAcquireAlpha * candidate.position.x;
                active_gate.y_pos =
                    (1.0 - kMixedAcquireAlpha) * active_gate.y_pos + kMixedAcquireAlpha * candidate.position.y;
                active_gate.road.PSI_end =
                    wrap_angle(0.75 * active_gate.road.PSI_end + 0.25 * candidate.heading);
                active_gate.passed = false;
                active_gate.final = false;
                active_gate.too_far = false;
                dynamic_lidar_gate_hold_steps_ = 0;
                dynamic_lidar_stable_steps_ = std::min(dynamic_lidar_stable_steps_ + 1, 1000000);
                dynamic_lidar_active_gate_width_ =
                    0.70 * dynamic_lidar_active_gate_width_ + 0.30 * candidate.width;
                dynamic_lidar_active_gate_score_ =
                    0.70 * dynamic_lidar_active_gate_score_ + 0.30 * candidate.score;
                return;
            }
        }
        publish_gate(candidate);
        return;
    }

    int active_index = -1;
    for (size_t i = 0; i < gates_.size(); ++i) {
        if (!gates_[i].passed && !gates_[i].final) {
            active_index = static_cast<int>(i);
            break;
        }
    }

    if (active_index >= 0) {
        gate& active_gate = gates_[static_cast<size_t>(active_index)];
        int best_match = -1;
        double best_match_distance = std::numeric_limits<double>::infinity();
        for (size_t i = 0; i < candidates.size(); ++i) {
            const double d = distance(gate_position(active_gate), candidates[i].position);
            if (d < best_match_distance) {
                best_match_distance = d;
                best_match = static_cast<int>(i);
            }
        }

        const bool compact_dynamic_world = compact_unstructured_world(world_);
        const bool mixed_dynamic_world = mixed_mode_enabled();
        const double match_radius =
            compact_dynamic_world ? 0.42 : (mixed_dynamic_world ? 1.65 : 2.40);
        if (best_match >= 0 && best_match_distance <= match_radius) {
            const DynamicLidarGateCandidate& candidate = candidates[static_cast<size_t>(best_match)];
            if (compact_dynamic_world || mixed_dynamic_world) {
                const double update_alpha = mixed_dynamic_world ? 0.45 : 0.30;
                active_gate.x_pos = (1.0 - update_alpha) * active_gate.x_pos + update_alpha * candidate.position.x;
                active_gate.y_pos = (1.0 - update_alpha) * active_gate.y_pos + update_alpha * candidate.position.y;
            }
            active_gate.road.PSI_end =
                wrap_angle(0.85 * active_gate.road.PSI_end + 0.15 * candidate.heading);
            active_gate.too_far = false;
            dynamic_lidar_gate_hold_steps_ = 0;
            dynamic_lidar_stable_steps_ = std::min(dynamic_lidar_stable_steps_ + 1, 1000000);
            dynamic_lidar_active_gate_width_ =
                0.75 * dynamic_lidar_active_gate_width_ + 0.25 * candidate.width;
            dynamic_lidar_active_gate_score_ =
                0.75 * dynamic_lidar_active_gate_score_ + 0.25 * candidate.score;
            return;
        }

        ++dynamic_lidar_gate_hold_steps_;
        const int hold_limit = compact_unstructured_world(world_) ? 28 : (mixed_mode_enabled() ? 12 : 36);
        if (dynamic_lidar_gate_hold_steps_ <= hold_limit) {
            active_gate.too_far = false;
            return;
        }

        gates_.clear();
        chosen_gate_index_ = -1;
        reference_trajectory_.clear();
        planned_trajectory_.clear();
        last_mpc_command_.reset();
        dynamic_lidar_gate_hold_steps_ = 0;
        dynamic_lidar_stable_steps_ = 0;
        dynamic_lidar_active_gate_width_ = 0.0;
        dynamic_lidar_active_gate_score_ = 0.0;
    }

    if (!candidates.empty()) {
        publish_gate(candidates.front());
    }
}

void PlannerDrivenVehicleSim::update_unstructured_gate_progress() {
    if (!gate_environment(world_) || gates_.empty()) {
        return;
    }

    const Vec2 vehicle_position = navigation_position_;
    const double world_span = world_span_m(world_);
    const bool compact_world = world_span <= 5.0;
    const double lane_width = std::max(sim_.W, compact_world ? 0.40 : 1.0);

    for (size_t i = 0; i < gates_.size(); ++i) {
        gate& candidate = gates_[i];
        if (candidate.passed || candidate.final) {
            continue;
        }

        const Vec2 current_gate = gate_position(candidate);
        const Vec2 previous_anchor = (i == 0) ? world_.start() : gate_position(gates_[i - 1]);
        const Vec2 next_anchor =
            (i + 1 < gates_.size()) ? gate_position(gates_[i + 1]) : world_.goal();

        Vec2 route_axis = subtract(next_anchor, previous_anchor);
        double route_length = std::hypot(route_axis.x, route_axis.y);
        if (!(route_length > 1e-6)) {
            route_axis = subtract(world_.goal(), previous_anchor);
            route_length = std::hypot(route_axis.x, route_axis.y);
        }
        if (!(route_length > 1e-6)) {
            if (use_dynamic_lidar_gates() && mixed_mode_enabled()) {
                route_axis = {
                    std::cos(candidate.road.PSI_end),
                    std::sin(candidate.road.PSI_end),
                };
                route_length = 1.0;
            } else {
                continue;
            }
        }
        route_axis.x /= route_length;
        route_axis.y /= route_length;

        const Vec2 gate_to_vehicle = subtract(vehicle_position, current_gate);
        const double gate_distance = std::hypot(gate_to_vehicle.x, gate_to_vehicle.y);
        const double signed_forward = dot_product(gate_to_vehicle, route_axis);
        const double lateral_from_gate_corridor = std::abs(cross_product(route_axis, gate_to_vehicle));
        const double previous_to_gate = distance(previous_anchor, current_gate);
        const double gate_to_next = distance(current_gate, next_anchor);
        const double local_spacing = std::min(previous_to_gate, gate_to_next);

        const bool dynamic_gates = use_dynamic_lidar_gates();
        const double pass_radius =
            compact_world
                ? clamp_value(0.16 + 0.12 * local_spacing, 0.22, 0.60)
                : (dynamic_gates
                       ? clamp_value(0.28 + 0.035 * local_spacing, 0.55, 1.05)
                       : clamp_value(0.55 + 0.08 * local_spacing, 0.85, 2.40));
        const double pass_lateral =
            compact_world
                ? clamp_value(0.65 * lane_width + 0.06 * local_spacing, 0.28, 0.75)
                : (dynamic_gates
                       ? clamp_value(0.45 * lane_width + 0.025 * local_spacing, 0.70, 1.40)
                       : clamp_value(0.75 * lane_width + 0.06 * local_spacing, 1.10, 3.00));
        const double forward_margin =
            compact_world
                ? clamp_value(0.06 + 0.04 * local_spacing, 0.08, 0.24)
                : (dynamic_gates
                       ? clamp_value(0.18 + 0.025 * local_spacing, 0.35, 0.80)
                       : clamp_value(0.25 + 0.035 * local_spacing, 0.40, 0.95));

        const bool reached_gate_region =
            !dynamic_gates && gate_distance <= pass_radius;
        const bool compact_mixed_dynamic_gate =
            dynamic_gates && mixed_mode_enabled() && compact_world;
        const bool compact_mixed_reached_gate =
            compact_mixed_dynamic_gate && gate_distance <= 0.14;
        const double compact_mixed_pass_radius =
            std::max(pass_radius, compact_mixed_dynamic_gate ? 0.38 : pass_radius);
        const bool crossed_gate_corridor =
            signed_forward >= forward_margin &&
            lateral_from_gate_corridor <= pass_lateral &&
            (!compact_mixed_dynamic_gate || gate_distance <= compact_mixed_pass_radius);

        if (reached_gate_region || compact_mixed_reached_gate || crossed_gate_corridor) {
            candidate.passed = true;
            candidate.choose = false;
            candidate.too_far = false;
            if (use_dynamic_lidar_gates()) {
                ++dynamic_lidar_passed_gates_;
                dynamic_lidar_passed_gate_positions_.push_back(current_gate);
                gates_.clear();
                chosen_gate_index_ = -1;
                reference_trajectory_.clear();
                planned_trajectory_.clear();
                last_mpc_command_.reset();
                dynamic_lidar_gate_hold_steps_ = 0;
                dynamic_lidar_stable_steps_ = 0;
                dynamic_lidar_active_gate_width_ = 0.0;
                dynamic_lidar_active_gate_score_ = 0.0;
                if (mixed_mode_enabled()) {
                    leave_mixed_gate_mode(false);
                }
                return;
            }
        }
    }
}

void PlannerDrivenVehicleSim::update_gate_activation_window() {
    if (!gate_environment(world_) || gates_.empty()) {
        return;
    }

    int primary_index = -1;
    for (size_t i = 0; i < gates_.size(); ++i) {
        if (!gates_[i].passed) {
            primary_index = static_cast<int>(i);
            break;
        }
    }

    if (primary_index < 0) {
        for (gate& candidate : gates_) {
            candidate.too_far = false;
            candidate.choose = false;
        }
        return;
    }

    const bool compact_world = world_span_m(world_) <= 5.0;
    const double activation_range =
        std::max(active_lidar_range() * 1.20, compact_world ? 1.0 : 6.0);
    for (size_t i = 0; i < gates_.size(); ++i) {
        if (gates_[i].passed) {
            gates_[i].too_far = false;
            gates_[i].choose = false;
            continue;
        }

        const int lookahead_index = static_cast<int>(i) - primary_index;
        const bool is_primary = lookahead_index == 0;
        const bool in_lookahead_window =
            lookahead_index > 0 &&
            lookahead_index <= 2 &&
            distance_to_gate_point(gates_[i], navigation_position_) <= activation_range;
        gates_[i].too_far = !(is_primary || in_lookahead_window);
        if (gates_[i].too_far) {
            gates_[i].choose = false;
        }
    }
}

std::vector<int> PlannerDrivenVehicleSim::active_gate_indices() const {
    std::vector<int> indices;
    if (!gate_environment(world_)) {
        return indices;
    }

    for (size_t i = 0; i < gates_.size(); ++i) {
        if (!gates_[i].passed && !gates_[i].too_far) {
            indices.push_back(static_cast<int>(i));
        }
    }
    return indices;
}

void PlannerDrivenVehicleSim::plan_if_needed() {
    if (step_count_ % planning_interval_steps() != 0) {
        return;
    }

    sim_.V_max = planner_speed_limit(config_.cruise_speed_limit, cl_.kappa);

    if (mixed_mode_enabled() && mixed_gate_mode_active_ && active_gate_indices().empty()) {
        leave_mixed_gate_mode(true);
    }

    if (road_environment(world_) &&
        road_ != nullptr &&
        (!mixed_mode_enabled() || !mixed_gate_mode_active_)) {
        std::vector<double> commands = sel_jr(
            false,
            step_count_,
            true,
            road_.get(),
            false,
            nullptr,
            sim_,
            x0_,
            g_x0_,
            cl_,
            null_stream_,
            null_stream_,
            null_stream_);

        double next_j = commands.size() > 0 ? commands[0] : 0.0;
        double next_r = commands.size() > 1 ? commands[1] : 0.0;
        if (!is_finite_pair(next_j, next_r)) {
            next_j = 0.0;
            next_r = 0.0;
        }

        visible_gate_indices_.clear();
        chosen_gate_index_ = -1;
        last_j_ = clamp_value(next_j, -3.5, 2.5);
        last_r_ = clamp_value(next_r, -1.4, 1.4);
        update_selected_trajectory();
        return;
    }

    if (mixed_mode_enabled() && road_ != nullptr && mixed_gate_mode_active_) {
        const std::vector<int> active_indices = active_gate_indices();
        if (active_indices.empty()) {
            leave_mixed_gate_mode(true);
            return;
        }
        std::vector<gate> active_gates;
        active_gates.reserve(active_indices.size());
        for (int index : active_indices) {
            active_gates.push_back(gates_[static_cast<size_t>(index)]);
        }

        std::vector<double> commands = sel_jr(
            false,
            step_count_,
            false,
            nullptr,
            true,
            &active_gates,
            sim_,
            x0_,
            g_x0_,
            cl_,
            null_stream_,
            null_stream_,
            null_stream_);

        double next_j = commands.size() > 0 ? commands[0] : 0.0;
        double next_r = commands.size() > 1 ? commands[1] : 0.0;
        int chosen_gate = commands.size() > 2 ? static_cast<int>(commands[2]) : -1;

        if (!is_finite_pair(next_j, next_r)) {
            next_j = 0.0;
            next_r = 0.0;
            chosen_gate = -1;
        }

        for (size_t i = 0; i < active_indices.size(); ++i) {
            gates_[static_cast<size_t>(active_indices[i])] = active_gates[i];
        }

        last_j_ = clamp_value(next_j, -3.5, 2.5);
        last_r_ = clamp_value(next_r, -1.4, 1.4);

        const bool compact_mixed_open_world =
            world_span_m(world_) <= 1.20 &&
            mixed_mode_enabled() &&
            !structured_road_is_closed_loop(world_);
        if (chosen_gate == 100 || chosen_gate < 0) {
            if (compact_mixed_open_world && !active_indices.empty()) {
                refresh_gate_diagnostics();
                chosen_gate_index_ = active_indices.front();
                mixed_gate_mode_active_ = true;
                last_j_ = std::max(last_j_, 0.0);
                last_r_ = 0.0;
                update_selected_trajectory();
                return;
            }
            leave_mixed_gate_mode(false);
            update_selected_trajectory();
            return;
        }

        int chosen_gate_global = -1;
        if (chosen_gate >= 0 && chosen_gate < static_cast<int>(active_indices.size())) {
            chosen_gate_global = active_indices[static_cast<size_t>(chosen_gate)];
        }
        refresh_gate_diagnostics();
        if (chosen_gate_global >= 0 && chosen_gate_global < static_cast<int>(gates_.size())) {
            chosen_gate_index_ = chosen_gate_global;
            mixed_gate_mode_active_ = true;
        }
        update_selected_trajectory();
        return;
    }

    const std::vector<int> active_indices = active_gate_indices();
    if (gates_.empty() || active_indices.empty()) {
        last_j_ = 0.0;
        last_r_ = 0.0;
        chosen_gate_index_ = -1;
        planned_trajectory_.clear();
        reference_trajectory_.clear();
        last_mpc_command_.reset();
        return;
    }

    std::vector<gate> active_gates;
    active_gates.reserve(active_indices.size());
    for (int index : active_indices) {
        active_gates.push_back(gates_[static_cast<size_t>(index)]);
    }

    std::vector<double> commands = sel_jr(
        false,
        step_count_,
        false,
        nullptr,
        true,
        &active_gates,
        sim_,
        x0_,
        g_x0_,
        cl_,
        null_stream_,
        null_stream_,
        null_stream_);

    double next_j = commands.size() > 0 ? commands[0] : 0.0;
    double next_r = commands.size() > 1 ? commands[1] : 0.0;
    int chosen_gate = commands.size() > 2 ? static_cast<int>(commands[2]) : -1;

    if (!is_finite_pair(next_j, next_r)) {
        next_j = 0.0;
        next_r = 0.0;
        chosen_gate = -1;
    }

    for (size_t i = 0; i < active_indices.size(); ++i) {
        gates_[static_cast<size_t>(active_indices[i])] = active_gates[i];
    }

    int chosen_gate_global = -1;
    if (chosen_gate >= 0 && chosen_gate < static_cast<int>(active_indices.size())) {
        chosen_gate_global = active_indices[static_cast<size_t>(chosen_gate)];
    }

    last_j_ = clamp_value(next_j, -3.5, 2.5);
    last_r_ = clamp_value(next_r, -1.4, 1.4);
    refresh_gate_diagnostics();
    if (chosen_gate_global >= 0 && chosen_gate_global < static_cast<int>(gates_.size())) {
        chosen_gate_index_ = chosen_gate_global;
    }
    update_selected_trajectory();
}

void PlannerDrivenVehicleSim::update_lidar() {
    if (!config_.lidar_enabled) {
        lidar_hits_.clear();
        min_lidar_distance_ = -1.0;
        lidar_scan_fresh_ = false;
        return;
    }

    lidar_scan_fresh_ = false;
    const bool force_refresh = step_count_ == 0 || lidar_hits_.empty();
    if (!force_refresh && (step_count_ % std::max(lidar_update_interval_steps(), 1)) != 0) {
        min_lidar_distance_ = compute_min_lidar();
        return;
    }

    const RangeSensorSpec sensor = make_range_sensor_spec(config_);
    const VehicleModelState& state = vehicle_model_->state();
    lidar_hits_ = world_.raycast(state.position, state.yaw, sensor.beams, sensor.fov_rad, sensor.range);
    min_lidar_distance_ = compute_min_lidar();
    lidar_scan_fresh_ = true;
}

void PlannerDrivenVehicleSim::update_vehicle_snapshot() {
    const VehicleModelState& model_state = vehicle_model_->state();
    vehicle_.position = model_state.position;
    vehicle_.yaw = model_state.yaw;
    vehicle_.speed = model_state.speed;
    vehicle_.accel = model_state.accel;
    vehicle_.curvature = model_state.curvature;
    vehicle_.steer_angle = model_state.steer_angle;
    vehicle_.yaw_rate = model_state.yaw_rate;
    vehicle_.sideslip = model_state.sideslip;
    vehicle_.left_wheel_speed = model_state.left_wheel_speed;
    vehicle_.right_wheel_speed = model_state.right_wheel_speed;
    vehicle_.target_speed = model_state.target_speed;
    vehicle_.target_yaw_rate = model_state.target_yaw_rate;
    vehicle_.target_steer_angle = model_state.target_steer_angle;
    vehicle_.left_encoder_ticks = model_state.left_encoder_ticks;
    vehicle_.right_encoder_ticks = model_state.right_encoder_ticks;
    vehicle_.left_encoder_delta = model_state.left_encoder_delta;
    vehicle_.right_encoder_delta = model_state.right_encoder_delta;
    vehicle_.left_pwm = model_state.left_pwm;
    vehicle_.right_pwm = model_state.right_pwm;
    vehicle_.encoder_dt_ms = model_state.encoder_dt_ms;
    vehicle_.model_name = vehicle_model_->name();
    vehicle_.body_corners = make_box_corners(vehicle_.position, vehicle_.yaw, geometry_.body_length, geometry_.body_width);

    const bool tracked_vehicle = config_.vehicle_model == VehicleModelKind::TrackedVehicle;
    const std::array<Vec2, 4> wheel_local = tracked_vehicle
        ? std::array<Vec2, 4>{{
              {0.0, geometry_.track * 0.5},
              {0.0, -geometry_.track * 0.5},
              {0.0, geometry_.track * 0.5},
              {0.0, -geometry_.track * 0.5},
          }}
        : std::array<Vec2, 4>{{
              {geometry_.wheelbase * 0.5, geometry_.track * 0.5},
              {geometry_.wheelbase * 0.5, -geometry_.track * 0.5},
              {-geometry_.wheelbase * 0.5, geometry_.track * 0.5},
              {-geometry_.wheelbase * 0.5, -geometry_.track * 0.5},
          }};

    for (size_t i = 0; i < wheel_local.size(); ++i) {
        const Vec2 global_offset = rotate(wheel_local[i], vehicle_.yaw);
        vehicle_.wheels[i].center = {vehicle_.position.x + global_offset.x, vehicle_.position.y + global_offset.y};
        const bool is_left = wheel_local[i].y >= 0.0;
        const bool is_front = wheel_local[i].x > 0.0;
        vehicle_.wheels[i].steering = !tracked_vehicle && is_front;
        vehicle_.wheels[i].yaw = vehicle_.yaw + (vehicle_.wheels[i].steering ? vehicle_.steer_angle : 0.0);
        vehicle_.wheels[i].speed = is_left ? vehicle_.left_wheel_speed : vehicle_.right_wheel_speed;
    }
}

double PlannerDrivenVehicleSim::compute_min_lidar() const {
    if (!config_.lidar_enabled) {
        return -1.0;
    }
    if (lidar_hits_.empty()) {
        return active_lidar_range();
    }
    double min_value = active_lidar_range();
    for (const LidarHit& hit : lidar_hits_) {
        min_value = std::min(min_value, hit.distance);
    }
    return min_value;
}

double PlannerDrivenVehicleSim::score_lidar_pose_candidate(const Vec2& position, double yaw) const {
    if (!config_.lidar_enabled || lidar_hits_.empty()) {
        return std::numeric_limits<double>::infinity();
    }
    if (!point_in_bounds(world_.bounds(), position)) {
        return std::numeric_limits<double>::infinity();
    }

    const RangeSensorSpec sensor = make_range_sensor_spec(config_);
    const int beam_count = std::max(sensor.beams, 1);
    if (static_cast<int>(lidar_hits_.size()) < beam_count) {
        return std::numeric_limits<double>::infinity();
    }
    const double start_offset = -sensor.fov_rad * 0.5;
    const double angle_step = beam_count > 1 ? sensor.fov_rad / static_cast<double>(beam_count - 1) : 0.0;
    const bool unstructured = gate_environment(world_);
    const bool short_unstructured =
        unstructured && config_.range_sensor_profile == RangeSensorProfile::ShortRangeScanner;
    const int stride = sensor.beams >= 300
                           ? (unstructured ? 10 : 6)
                           : (short_unstructured ? 6 : (unstructured ? 4 : 3));

    double score = 0.0;
    int used = 0;
    for (int i = 0; i < beam_count; i += stride) {
        const double beam_heading = yaw + start_offset + static_cast<double>(i) * angle_step;
        const std::vector<LidarHit> expected = world_.raycast(position, beam_heading, 1, 0.0, sensor.range);
        if (expected.empty()) {
            continue;
        }

        const LidarHit& measured = lidar_hits_[static_cast<size_t>(i)];
        const LidarHit& predicted = expected.front();
        const double measured_range = clamp_value(measured.distance, 0.0, sensor.range);
        if (measured_range <= 0.0) {
            continue;
        }

        const double predicted_range = clamp_value(predicted.distance, 0.0, sensor.range);
        const double range_error = predicted_range - measured_range;
        const double range_scale = std::max(
            0.18 + std::min(measured_range, predicted_range),
            0.20);
        double beam_score = (range_error * range_error) / (range_scale * range_scale);
        if (measured.hit != predicted.hit) {
            beam_score += 1.75;
        }
        score += beam_score;
        ++used;
    }

    if (used < std::max(12, beam_count / std::max(stride * 4, 1))) {
        return std::numeric_limits<double>::infinity();
    }
    return score / static_cast<double>(used);
}

int PlannerDrivenVehicleSim::active_lidar_beams() const {
    return make_range_sensor_spec(config_).beams;
}

double PlannerDrivenVehicleSim::active_lidar_fov_rad() const {
    return make_range_sensor_spec(config_).fov_rad;
}

double PlannerDrivenVehicleSim::active_lidar_range() const {
    return make_range_sensor_spec(config_).range;
}

void PlannerDrivenVehicleSim::set_sensor_suite(bool imu_enabled,
                                               bool lidar_enabled,
                                               RangeSensorProfile profile) {
    config_.imu_enabled = imu_enabled;
    config_.lidar_enabled = lidar_enabled;
    config_.range_sensor_profile = profile;
    update_lidar();
    update_navigation_state(config_.dt);
    sync_planner_from_vehicle(false);
    update_selected_trajectory();
}

void PlannerDrivenVehicleSim::set_dynamic_lidar_gates(bool enabled) {
    if (config_.dynamic_lidar_gates == enabled) {
        return;
    }
    config_.dynamic_lidar_gates = enabled;
    reset();
}

void PlannerDrivenVehicleSim::set_vehicle_stack(VehicleModelKind model, TrackingControllerMode controller) {
    config_.vehicle_model =
        model == VehicleModelKind::TrackedVehicle ? VehicleModelKind::TrackedVehicle
                                                  : VehicleModelKind::CarLikeBicycle;
    config_.tracking_controller =
        controller == TrackingControllerMode::PlannerCommand ? TrackingControllerMode::PlannerCommand
                                                             : TrackingControllerMode::MpcPathFollower;
    reset();
}

void PlannerDrivenVehicleSim::set_tuning_overrides(const VehicleTuningOverrides& overrides) {
    tuning_overrides_ = overrides;
    reset();
}

void PlannerDrivenVehicleSim::set_gate_behavior(GateBehaviorMode mode, std::uint32_t seed) {
    config_.gate_behavior = mode;
    config_.gate_seed = seed;
    world_.set_gate_behavior(mode, seed);
    world_.update_gate_layout(sim_time_);
    sync_gate_specs_from_world(false);
}

void PlannerDrivenVehicleSim::regenerate_gate_layout(std::uint32_t seed) {
    config_.gate_seed = seed;
    world_.reset_gate_layout(seed);
    world_.update_gate_layout(sim_time_);
    sync_gate_specs_from_world(false);
}

void PlannerDrivenVehicleSim::update_telemetry() {
    const double chosen_gate_distance =
        chosen_gate_index_ >= 0 && chosen_gate_index_ < static_cast<int>(gates_.size())
            ? distance_to_gate_point(gates_[static_cast<size_t>(chosen_gate_index_)], navigation_position_)
            : -1.0;
    history_.push_back({
        sim_time_,
        vehicle_.position.x,
        vehicle_.position.y,
        vehicle_.speed,
        vehicle_.accel,
        vehicle_.yaw,
        last_j_,
        vehicle_.curvature,
        vehicle_.yaw_rate,
        last_r_,
        vehicle_.steer_angle,
        vehicle_.target_steer_angle,
        vehicle_.sideslip,
        vehicle_.left_wheel_speed,
        vehicle_.right_wheel_speed,
        vehicle_.target_speed,
        vehicle_.target_yaw_rate,
        static_cast<double>(vehicle_.left_encoder_ticks),
        static_cast<double>(vehicle_.right_encoder_ticks),
        static_cast<double>(vehicle_.left_encoder_delta),
        static_cast<double>(vehicle_.right_encoder_delta),
        static_cast<double>(vehicle_.left_pwm),
        static_cast<double>(vehicle_.right_pwm),
        distance_to_goal_,
        min_lidar_distance_,
        navigation_xy_error_,
        navigation_yaw_error_deg_,
        planner_speed_ref_,
        last_mpc_command_.has_value() ? last_mpc_command_->accel_cmd : 0.0,
        last_mpc_command_.has_value() ? last_mpc_command_->steer_rate_cmd : 0.0,
        tracker_cross_track_error_,
        tracker_heading_error_deg_,
        planning_compute_ms_,
        tracking_compute_ms_,
        lidar_compute_ms_,
        estimator_compute_ms_,
        step_compute_ms_,
        static_cast<double>(visible_gate_indices_.size()),
        static_cast<double>(lidar_hits_.size()),
        static_cast<double>(chosen_gate_index_),
        chosen_gate_distance,
        static_cast<double>(count_passed_gates()),
        static_cast<double>(dynamic_lidar_candidate_count_),
        use_dynamic_lidar_gates() ? 1.0 : 0.0,
        !reference_trajectory_.empty() ? 1.0 : 0.0,
        mixed_gate_mode_active_ ? 1.0 : 0.0,
        mixed_gate_score_,
        mixed_structured_score_,
        mixed_gate_confidence_,
        static_cast<double>(mixed_switch_count_),
        static_cast<double>(mixed_abort_count_),
    });
    if (static_cast<int>(history_.size()) > config_.max_history) {
        history_.erase(history_.begin());
    }

    trail_.push_back(vehicle_.position);
    if (static_cast<int>(trail_.size()) > config_.max_history) {
        trail_.erase(trail_.begin());
    }

    estimated_trail_.push_back(navigation_position_);
    if (static_cast<int>(estimated_trail_.size()) > config_.max_history) {
        estimated_trail_.erase(estimated_trail_.begin());
    }
}

void PlannerDrivenVehicleSim::update_selected_trajectory() {
    planned_trajectory_.clear();
    reference_trajectory_.clear();
    const bool has_planner_clothoid =
        cl_.prev_road.num_segments() > 0 &&
        std::isfinite(cl_.end_point_s) &&
        cl_.end_point_s > 0.10;

    const bool mixed_gate_reference = mixed_mode_enabled() && mixed_gate_mode_active_;
    if (road_environment(world_) &&
        !mixed_gate_reference &&
        (!has_planner_clothoid || world_.road_centerline().size() < 2)) {
        planned_trajectory_ = world_.road_centerline();
        reference_trajectory_ = build_reference_waypoints(planned_trajectory_, planner_speed_ref_);
        return;
    }

    if ((world_.environment_mode() == EnvironmentMode::UnstructuredGates || mixed_gate_reference) &&
        chosen_gate_index_ < 0) {
        return;
    }
    if (mixed_gate_reference &&
        chosen_gate_index_ >= 0 &&
        chosen_gate_index_ < static_cast<int>(gates_.size())) {
        const gate& selected_gate = gates_[static_cast<size_t>(chosen_gate_index_)];
        const Vec2 start = navigation_position_;
        const Vec2 target = gate_position(selected_gate);
        const double gate_distance = distance(start, target);
        if (gate_distance >= 0.05) {
            const Vec2 gate_axis{
                (target.x - start.x) / gate_distance,
                (target.y - start.y) / gate_distance,
            };
            const Vec2 p1{
                start.x + gate_axis.x * 0.45 * gate_distance,
                start.y + gate_axis.y * 0.45 * gate_distance,
            };
            const Vec2 p2{
                target.x - gate_axis.x * 0.15 * gate_distance,
                target.y - gate_axis.y * 0.15 * gate_distance,
            };
            constexpr int kSamples = 28;
            planned_trajectory_.reserve(kSamples);
            for (int i = 0; i < kSamples; ++i) {
                const double t = static_cast<double>(i) / static_cast<double>(kSamples - 1);
                const double u = 1.0 - t;
                planned_trajectory_.push_back({
                    u * u * u * start.x +
                        3.0 * u * u * t * p1.x +
                        3.0 * u * t * t * p2.x +
                        t * t * t * target.x,
                    u * u * u * start.y +
                        3.0 * u * u * t * p1.y +
                        3.0 * u * t * t * p2.y +
                        t * t * t * target.y,
                });
            }
            reference_trajectory_ = build_reference_waypoints(planned_trajectory_, planner_speed_ref_);
        }
        return;
    }
    if (!has_planner_clothoid) {
        return;
    }

    const bool closed_structured_loop = structured_road_is_closed_loop(world_);
    double s_current = x0_.x;
    if (!std::isfinite(s_current)) {
        s_current = 0.0;
    }
    if (closed_structured_loop) {
        s_current = wrap_arc_length(s_current, cl_.end_point_s);
    } else {
        s_current = clamp_value(std::max(0.0, s_current), 0.0, cl_.end_point_s);
    }

    double structured_progress_delta = 0.0;
    double lateral_offset = 0.0;
    if (!project_curvilinear_state(cl_, navigation_position_, s_current, closed_structured_loop, &s_current, &lateral_offset)) {
        double x_on_path = 0.0;
        double y_on_path = 0.0;
        cl_.prev_road.eval(s_current, x_on_path, y_on_path);
        const double path_heading = wrap_angle(cl_.prev_road.theta(s_current));
        const double dx = navigation_position_.x - x_on_path;
        const double dy = navigation_position_.y - y_on_path;
        lateral_offset = -std::sin(path_heading) * dx + std::cos(path_heading) * dy;
    }
    if (closed_structured_loop) {
        const double max_forward_step =
            std::max(navigation_speed_ * std::max(config_.dt, 1e-3) + 0.20, 0.08);
        s_current = stabilize_structured_track_s(
            s_current,
            structured_last_s_,
            cl_.end_point_s,
            max_forward_step,
            &structured_progress_delta);
        double x_on_path = 0.0;
        double y_on_path = 0.0;
        cl_.prev_road.eval(s_current, x_on_path, y_on_path);
        const double path_heading = wrap_angle(cl_.prev_road.theta(s_current));
        const double dx = navigation_position_.x - x_on_path;
        const double dy = navigation_position_.y - y_on_path;
        lateral_offset = -std::sin(path_heading) * dx + std::cos(path_heading) * dy;
    }
    x0_.x = s_current;
    x0_.n = lateral_offset;
    x0_.v = navigation_speed_;
    x0_.a = navigation_accel_;
    x0_.c = navigation_curvature_;

    if (closed_structured_loop) {
        if (!structured_goal_ready_) {
            structured_goal_progress_target_ = std::max(0.0, cl_.end_point_s);
            structured_goal_position_ = world_.start();
            structured_goal_ready_ = structured_goal_progress_target_ > 0.0;
        }

        if (!std::isfinite(structured_last_s_)) {
            structured_progress_s_ = 0.0;
        } else {
            structured_progress_s_ = std::max(0.0, structured_progress_s_ + structured_progress_delta);
        }
        structured_last_s_ = s_current;
    }

    const bool unstructured =
        world_.environment_mode() == EnvironmentMode::UnstructuredGates || mixed_gate_reference;
    const double lookahead_distance = unstructured ? 22.0 : (closed_structured_loop ? 14.0 : 18.0);
    const int sample_count = unstructured ? 48 : (closed_structured_loop ? 64 : 36);
    double s_start = closed_structured_loop ? wrap_arc_length(s_current, cl_.end_point_s) : std::max(0.0, s_current);
    double s_end = 0.0;
    if (closed_structured_loop) {
        const double max_span = closed_loop_reference_span(cl_.end_point_s, lookahead_distance);
        s_end = s_start + max_span;
    } else {
        s_end = std::min(cl_.end_point_s, s_start + lookahead_distance);
        if (!(s_end > s_start + 0.10)) {
            s_start = std::max(0.0, cl_.end_point_s - 0.75);
            s_end = cl_.end_point_s;
        }
    }
    if (!(s_end > s_start + 0.10)) {
        return;
    }

    reference_trajectory_ = build_reference_waypoints(
        cl_,
        s_start,
        s_end,
        sample_count,
        planner_speed_ref_,
        closed_structured_loop);
    planned_trajectory_ = extract_reference_positions(reference_trajectory_);
}

int PlannerDrivenVehicleSim::planning_interval_steps() const {
    const int base_interval = std::max(config_.control_interval_steps, 1);
    if (gate_environment(world_) &&
        config_.tracking_controller == TrackingControllerMode::MpcPathFollower) {
        if (config_.range_sensor_profile == RangeSensorProfile::ShortRangeScanner) {
            return std::max(base_interval, 4);
        }
        if (config_.gate_behavior == GateBehaviorMode::Static) {
            return std::max(base_interval, 4);
        }
        return std::max(base_interval, 3);
    }
    return base_interval;
}

int PlannerDrivenVehicleSim::lidar_update_interval_steps() const {
    switch (config_.range_sensor_profile) {
        case RangeSensorProfile::IdealLidar2D:
            return 1;
        case RangeSensorProfile::RplidarA1:
            return gate_environment(world_) ? 3 : 2;
        case RangeSensorProfile::ShortRangeScanner:
            return gate_environment(world_) ? 2 : 1;
        default:
            return 1;
    }
}

void PlannerDrivenVehicleSim::step() {
    if (goal_reached_ || collision_) {
        return;
    }

    const auto step_start = std::chrono::steady_clock::now();

    world_.update_gate_layout(sim_time_);
    sync_gate_specs_from_world(false);
    update_dynamic_lidar_gates();
    refresh_gate_diagnostics();
    update_mixed_arbitration();

    const auto planning_start = std::chrono::steady_clock::now();
    plan_if_needed();
    update_planner_references(config_.dt);
    update_selected_trajectory();
    planning_compute_ms_ = elapsed_ms(planning_start, std::chrono::steady_clock::now());

    VehicleControlInput control{};
    control.jerk_cmd = last_j_;
    control.planner_r_cmd = last_r_;
    control.target_speed = planner_speed_ref_;
    const bool tracked_vehicle = config_.vehicle_model == VehicleModelKind::TrackedVehicle;
    const bool compact_mixed_open_step =
        mixed_mode_enabled() &&
        world_span_m(world_) <= 1.20 &&
        !structured_road_is_closed_loop(world_);
    const bool compact_mixed_gate_step =
        compact_mixed_open_step &&
        mixed_gate_mode_active_ &&
        chosen_gate_index_ >= 0 &&
        chosen_gate_index_ < static_cast<int>(gates_.size());
    double tracking_desired_speed = planner_speed_ref_;
    if ((world_.environment_mode() == EnvironmentMode::UnstructuredGates ||
         (mixed_mode_enabled() && mixed_gate_mode_active_)) &&
        !compact_unstructured_world(world_) &&
        reference_trajectory_.size() >= 2 &&
        distance_to_goal_ > 1.50) {
        tracking_desired_speed = std::max(
            tracking_desired_speed,
            std::min(0.30, config_.cruise_speed_limit));
    }
    if (compact_mixed_gate_step && reference_trajectory_.size() >= 2) {
        tracking_desired_speed = clamp_value(
            tracking_desired_speed,
            std::min(0.045, config_.cruise_speed_limit),
            std::min(0.070, config_.cruise_speed_limit));
    } else if (mixed_mode_enabled() && reference_trajectory_.size() >= 2) {
        tracking_desired_speed = std::max(
            tracking_desired_speed,
            std::min(0.12, config_.cruise_speed_limit));
    }
    if (tracked_vehicle && reference_trajectory_.size() >= 2) {
        tracking_desired_speed = std::max(
            tracking_desired_speed,
            tracked_vehicle_speed_floor(world_, geometry_));
    }
    tracker_cross_track_error_ = 0.0;
    tracker_heading_error_deg_ = 0.0;

    const auto tracking_start = std::chrono::steady_clock::now();
    if (config_.vehicle_model == VehicleModelKind::CarLikeBicycle ||
        tracked_vehicle) {
        const VehicleModelState tracking_state = build_tracking_state(
            vehicle_model_->state(),
            navigation_position_,
            navigation_yaw_,
            navigation_speed_,
            navigation_accel_,
            navigation_curvature_,
            navigation_yaw_rate_);
        if (config_.tracking_controller == TrackingControllerMode::MpcPathFollower &&
            reference_trajectory_.size() >= 2) {
            const MpcCommand mpc_command = mpc_follower_.solve(
                geometry_,
                tracking_state,
                reference_trajectory_,
                tracking_desired_speed,
                0);
            if (mpc_command.valid) {
                control.accel_cmd = mpc_command.accel_cmd;
                control.target_speed = mpc_command.target_speed;
                const double target_curvature = clamp_value(
                    std::tan(mpc_command.target_steer_angle) / std::max(geometry_.wheelbase, 1e-6),
                    -geometry_.max_curvature,
                    geometry_.max_curvature);
                control.target_curvature = target_curvature;
                if (tracked_vehicle) {
                    control.target_yaw_rate = clamp_value(
                        mpc_command.target_speed * target_curvature -
                            1.15 * mpc_command.heading_error -
                            0.20 * mpc_command.cross_track_error,
                        -geometry_.max_yaw_rate,
                        geometry_.max_yaw_rate);
                    control.target_steer_angle = 0.0;
                    control.steer_rate_cmd = 0.0;
                } else {
                    control.steer_rate_cmd = mpc_command.steer_rate_cmd;
                    control.target_steer_angle = mpc_command.target_steer_angle;
                    control.target_yaw_rate = control.target_speed * control.target_curvature;
                    if (compact_mixed_gate_step) {
                        const Vec2 target = gate_position(gates_[static_cast<size_t>(chosen_gate_index_)]);
                        const double direct_heading_error =
                            wrap_angle(angle_to(navigation_position_, target) - navigation_yaw_);
                        const double direct_steer = clamp_value(
                            1.35 * direct_heading_error,
                            -geometry_.max_steer_angle,
                            geometry_.max_steer_angle);
                        if (std::abs(direct_steer) > std::abs(control.target_steer_angle)) {
                            control.target_steer_angle = direct_steer;
                            control.steer_rate_cmd = clamp_value(
                                (direct_steer - tracking_state.steer_angle) / std::max(config_.dt, 1e-3),
                                -geometry_.max_steer_rate,
                                geometry_.max_steer_rate);
                            control.target_curvature = clamp_value(
                                std::tan(direct_steer) / std::max(geometry_.wheelbase, 1e-6),
                                -geometry_.max_curvature,
                                geometry_.max_curvature);
                            control.target_yaw_rate = control.target_speed * control.target_curvature;
                        }
                    }
                }
                tracker_cross_track_error_ = std::abs(mpc_command.cross_track_error);
                tracker_heading_error_deg_ = std::abs(mpc_command.heading_error) * 180.0 / 3.14159265358979323846;
                last_mpc_command_ = mpc_command;
            } else {
                last_mpc_command_.reset();
            }
        } else {
            const double target_curvature = !reference_trajectory_.empty() ? reference_trajectory_.front().curvature : 0.0;
            control.accel_cmd = clamp_value(
                (tracking_desired_speed - tracking_state.speed) / std::max(config_.dt, 1e-3),
                -geometry_.max_decel,
                geometry_.max_accel);
            control.target_speed = tracking_desired_speed;
            control.target_curvature = clamp_value(
                target_curvature,
                -geometry_.max_curvature,
                geometry_.max_curvature);
            if (tracked_vehicle) {
                control.target_yaw_rate = clamp_value(
                    tracking_desired_speed * control.target_curvature,
                    -geometry_.max_yaw_rate,
                    geometry_.max_yaw_rate);
                control.target_steer_angle = 0.0;
                control.steer_rate_cmd = 0.0;
            } else {
                control.target_steer_angle = steer_from_curvature(geometry_, target_curvature);
                control.steer_rate_cmd = clamp_value(
                    (control.target_steer_angle - tracking_state.steer_angle) / std::max(config_.dt, 1e-3),
                    -geometry_.max_steer_rate,
                    geometry_.max_steer_rate);
                control.target_yaw_rate = control.target_speed * control.target_curvature;
            }
            last_mpc_command_.reset();
        }
    } else {
        last_mpc_command_.reset();
    }
    tracking_compute_ms_ = elapsed_ms(tracking_start, std::chrono::steady_clock::now());

    vehicle_model_->step(config_.dt, control);
    sim_time_ += config_.dt;
    ++step_count_;

    const auto lidar_start = std::chrono::steady_clock::now();
    update_lidar();
    lidar_compute_ms_ = elapsed_ms(lidar_start, std::chrono::steady_clock::now());

    const auto estimator_start = std::chrono::steady_clock::now();
    update_navigation_state(config_.dt);
    estimator_compute_ms_ = elapsed_ms(estimator_start, std::chrono::steady_clock::now());
    sync_planner_from_vehicle(false);
    update_vehicle_snapshot();
    const bool compact_mixed_world =
        world_.environment_mode() == EnvironmentMode::MixedRoadGates &&
        world_span_m(world_) <= 1.20;
    const double collision_padding =
        (compact_structured_world(world_) || compact_mixed_world) ? 0.0 : 0.05;
    collision_ = world_.collides(vehicle_.body_corners, collision_padding);
    if (structured_road_is_closed_loop(world_)) {
        const bool tiny_indoor_loop = micro_structured_world(world_);
        const double wrapped_track_s =
            std::isfinite(x0_.x) ? wrap_arc_length(x0_.x, cl_.end_point_s) : 0.0;
        const double goal_position_distance =
            distance(vehicle_.position, structured_goal_ready_ ? structured_goal_position_ : world_.start());
        distance_to_goal_ = structured_goal_ready_
                                ? std::max(structured_goal_progress_target_ - structured_progress_s_, 0.0)
                                : cl_.end_point_s;
        const double progress_margin =
            tiny_indoor_loop ? std::clamp(0.11 * cl_.end_point_s, 0.065, 0.10)
                             : std::clamp(0.03 * std::max(cl_.end_point_s, 1.0), 0.05, 0.20);
        const double start_window =
            tiny_indoor_loop ? std::clamp(0.10 * cl_.end_point_s, 0.065, 0.10)
                             : std::clamp(0.04 * std::max(cl_.end_point_s, 1.0), 0.10, 0.35);
        const double goal_position_acceptance =
            tiny_indoor_loop ? std::clamp(0.28 * world_span_m(world_), 0.09, 0.11) : 0.35;
        const bool returned_to_start =
            wrapped_track_s <= start_window ||
            wrapped_track_s >= std::max(cl_.end_point_s - start_window, 0.0);
        const bool physically_near_start = goal_position_distance < goal_position_acceptance;
        goal_reached_ =
            structured_goal_ready_ &&
            structured_progress_s_ + progress_margin >= structured_goal_progress_target_ &&
            physically_near_start &&
            returned_to_start;
    } else {
        distance_to_goal_ = distance(vehicle_.position, world_.goal());
        const bool unstructured = world_.environment_mode() == EnvironmentMode::UnstructuredGates;
        const bool mixed = world_.environment_mode() == EnvironmentMode::MixedRoadGates;
        const bool compact_unstructured = unstructured && world_span_m(world_) <= 5.0;
        const bool compact_mixed = mixed && world_span_m(world_) <= 5.0;
        const bool lab_scale_mixed_open =
            mixed &&
            world_span_m(world_) <= 1.20 &&
            !structured_road_is_closed_loop(world_);
        if (unstructured && world_.unstructured_preset() == UnstructuredMapPreset::HardwareLab &&
            count_passed_gates() >= 2) {
            distance_to_goal_ = 0.0;
            goal_reached_ = true;
        } else {
            const double goal_acceptance =
                lab_scale_mixed_open
                    ? 0.14
                    : compact_mixed
                    ? std::clamp(0.08 * world_span_m(world_), 0.06, 0.12)
                    : (compact_unstructured
                           ? std::clamp(0.05 * world_span_m(world_), 0.08, 0.16)
                           : 0.75);
            goal_reached_ = distance_to_goal_ < goal_acceptance && std::abs(vehicle_.speed) < 0.15;
        }
    }
    step_compute_ms_ = elapsed_ms(step_start, std::chrono::steady_clock::now());
    update_telemetry();
}

SimulationReport PlannerDrivenVehicleSim::run_headless(int max_steps) {
    const int limit = max_steps > 0 ? max_steps : 6000;
    while (!goal_reached_ && !collision_ && step_count_ < limit) {
        step();
    }

    return {
        goal_reached_,
        collision_,
        step_count_,
        sim_time_,
        vehicle_.position,
        distance_to_goal_,
        count_passed_gates(),
    };
}

int PlannerDrivenVehicleSim::count_passed_gates() const {
    if (use_dynamic_lidar_gates()) {
        return dynamic_lidar_passed_gates_;
    }
    int total = 0;
    for (const gate& g : gates_) {
        if (g.passed) {
            ++total;
        }
    }
    return total;
}

}  // namespace thesis_sim
