#pragma once

#include "mvc/controller/hardware_planner/runner.h"

#include "mvc/model/navigation/actuation_model.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <thread>
#include <unordered_set>
#include <utility>

namespace thesis_sim {

namespace {

using thesis_sim::mvc::model::apply_start_motion_boost;
using thesis_sim::mvc::model::apply_signed_pwm_calibration;
using thesis_sim::mvc::model::clamp_motion_pwm_band;
using thesis_sim::mvc::model::enforce_forward_differential_pwm;
using thesis_sim::mvc::model::enforce_forward_tracked_turn_authority;
using thesis_sim::mvc::model::remove_signed_pwm_calibration;
using thesis_sim::mvc::model::scale_motion_pwm_pair_to_full_scale;
using thesis_sim::mvc::model::slew_limit_pwm;
using thesis_sim::mvc::model::wheel_speed_from_pwm_estimate;
using thesis_sim::mvc::model::wheel_speeds_from_body;
using thesis_sim::mvc::model::yaw_rate_target_with_feedback;
using thesis_sim::mvc::model::stabilize_yaw_rate_target;

constexpr double kPi = 3.14159265358979323846;
constexpr int kStableEncoderReadyFrames = 3;
constexpr double kEncoderWheelSpeedMargin = 1.35;

struct LidarSearchWindow {
    double xy_window = 0.0;
    double xy_step = 0.0;
    double yaw_window = 0.0;
    double yaw_step = 0.0;
};

bool world_has_structured_reference(const WorldMap& world) {
    return world.environment_mode() == EnvironmentMode::StructuredRoad ||
           world.environment_mode() == EnvironmentMode::MixedRoadGates;
}

bool world_has_dynamic_gap_layer(const WorldMap& world) {
    return world.environment_mode() == EnvironmentMode::UnstructuredGates ||
           world.environment_mode() == EnvironmentMode::MixedRoadGates;
}

bool world_is_mixed(const WorldMap& world) {
    return world.environment_mode() == EnvironmentMode::MixedRoadGates;
}

double clamp_value(double value, double lo, double hi) {
    return std::max(lo, std::min(value, hi));
}

double wrap_angle(double angle) {
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

double signed_wrapped_arc_delta(double from_s, double to_s, double s_max) {
    if (!(s_max > 1e-6)) {
        return 0.0;
    }
    double delta = wrap_arc_length(to_s, s_max) - wrap_arc_length(from_s, s_max);
    if (delta > 0.5 * s_max) {
        delta -= s_max;
    } else if (delta < -0.5 * s_max) {
        delta += s_max;
    }
    return delta;
}

double structured_road_length_m(const WorldMap& world) {
    if (!world_has_structured_reference(world) ||
        world.road_centerline().size() < 2) {
        return 0.0;
    }
    double total = 0.0;
    for (size_t i = 1; i < world.road_centerline().size(); ++i) {
        total += distance(world.road_centerline()[i - 1], world.road_centerline()[i]);
    }
    return total;
}

double closed_loop_reference_span(double road_length, double lookahead_distance) {
    if (!(road_length > 1e-6)) {
        return 0.0;
    }
    if (road_length <= 0.55) {
        return clamp_value(0.22 * road_length, std::min(0.08, road_length), std::min(0.12, road_length));
    }
    if (road_length <= 0.90) {
        return clamp_value(0.34 * road_length, std::min(0.16, road_length), std::min(0.24, road_length));
    }
    if (road_length <= 1.60) {
        return clamp_value(0.40 * road_length, 0.20, std::min(0.38, road_length));
    }
    if (road_length <= 5.0) {
        return clamp_value(0.55 * road_length, 0.24, std::min(0.90 * road_length, road_length));
    }
    return std::max(4.0, std::min(lookahead_distance, 0.75 * road_length));
}

double structured_tracking_speed_floor(const WorldMap& world, double speed_limit) {
    double course_span = 0.0;
    if (world_has_structured_reference(world) &&
        !world.road_centerline().empty()) {
        double min_x = world.road_centerline().front().x;
        double max_x = min_x;
        double min_y = world.road_centerline().front().y;
        double max_y = min_y;
        for (const Vec2& point : world.road_centerline()) {
            min_x = std::min(min_x, point.x);
            max_x = std::max(max_x, point.x);
            min_y = std::min(min_y, point.y);
            max_y = std::max(max_y, point.y);
        }
        course_span = std::max(max_x - min_x, max_y - min_y);
    }
    const Rect& bounds = world.bounds();
    const double span = std::max(bounds.max_x - bounds.min_x, bounds.max_y - bounds.min_y);
    const double effective_span = course_span > 1e-6 ? course_span : span;
    const double road_length = structured_road_length_m(world);
    const double closure_threshold = std::clamp(0.12 * std::max(effective_span, 0.10), 0.05, 0.18);
    const bool closed_loop =
        world.environment_mode() == EnvironmentMode::StructuredRoad &&
        world.road_centerline().size() >= 3 &&
        distance(world.start(), world.goal()) <= closure_threshold &&
        distance(world.road_centerline().front(), world.road_centerline().back()) <= closure_threshold;
    double floor = 0.060;
    if (closed_loop && road_length <= 0.90) {
        floor = 0.022;
    } else if (closed_loop && road_length <= 1.40) {
        floor = 0.024;
    } else if (effective_span <= 0.35) {
        floor = 0.028;
    } else if (effective_span <= 0.75) {
        floor = 0.050;
    }
    return clamp_value(floor, 0.0, std::max(speed_limit, 0.0));
}

double world_span_m(const WorldMap& world) {
    const Rect& bounds = world.bounds();
    return std::max(bounds.max_x - bounds.min_x, bounds.max_y - bounds.min_y);
}

bool hardware_unstructured_exploration_world(const WorldMap& world) {
    return world.environment_mode() == EnvironmentMode::UnstructuredGates;
}

bool open_ended_unstructured_exploration(const WorldMap& world) {
    // Hardware unstructured missions are perception-driven regardless of the
    // GUI preset.  Preset gates are stripped before execution, so using the
    // preset goal as a search heading would silently reintroduce a scripted
    // arc in HardwareLab. The only motion objectives here are live LiDAR
    // free-space exploration and LiDAR-confirmed physical apertures.
    return hardware_unstructured_exploration_world(world);
}

bool navigation_world_bounds_enabled(const WorldMap& world) {
    // In LiDAR-only exploration WorldMap::bounds is a finite canvas used by
    // the GUI and dense debug grids, not a sensed wall or a physical geofence.
    // Structured and mixed missions still use their authored bounds.
    return !open_ended_unstructured_exploration(world);
}

double structured_course_span_m(const WorldMap& world) {
    if (!world_has_structured_reference(world) ||
        world.road_centerline().empty()) {
        return world_span_m(world);
    }
    double min_x = world.road_centerline().front().x;
    double max_x = min_x;
    double min_y = world.road_centerline().front().y;
    double max_y = min_y;
    for (const Vec2& point : world.road_centerline()) {
        min_x = std::min(min_x, point.x);
        max_x = std::max(max_x, point.x);
        min_y = std::min(min_y, point.y);
        max_y = std::max(max_y, point.y);
    }
    return std::max(max_x - min_x, max_y - min_y);
}

bool compact_structured_world(const WorldMap& world) {
    if (!world_has_structured_reference(world)) {
        return false;
    }
    return structured_course_span_m(world) <= 0.90 || world_span_m(world) <= 0.75;
}

bool micro_structured_world(const WorldMap& world) {
    return world_has_structured_reference(world) &&
           structured_course_span_m(world) <= 0.55;
}

bool compact_mixed_open_road(const WorldMap& world);

double structured_goal_active_distance(const WorldMap& world, double configured_goal_tolerance) {
    if (micro_structured_world(world)) {
        return std::max(0.040, 0.16 * structured_course_span_m(world));
    }
    if (compact_mixed_open_road(world)) {
        return std::max(configured_goal_tolerance * 1.15, 0.09);
    }
    if (world_is_mixed(world) &&
        world_span_m(world) <= 2.50 &&
        structured_course_span_m(world) <= 1.20) {
        return std::max(configured_goal_tolerance * 1.5, 0.12);
    }
    return std::max(configured_goal_tolerance * 2.0, 0.25);
}

double structured_slowdown_radius(const WorldMap& world, double configured_slowdown_radius) {
    if (micro_structured_world(world)) {
        return clamp_value(0.65 * structured_course_span_m(world), 0.10, 0.22);
    }
    return std::max(configured_slowdown_radius, 0.25);
}

double deg_to_rad(double angle_deg) {
    return angle_deg * kPi / 180.0;
}

double monotonic_seconds() {
    using clock = std::chrono::steady_clock;
    return std::chrono::duration<double>(clock::now().time_since_epoch()).count();
}

int desired_controller_cmd_timeout_ms(double nominal_dt) {
    return std::max(
        900,
        static_cast<int>(std::lround(std::max(nominal_dt, 0.05) * 10.0 * 1000.0)));
}

double elapsed_ms(std::chrono::steady_clock::time_point start,
                  std::chrono::steady_clock::time_point end) {
    return std::chrono::duration<double, std::milli>(end - start).count();
}

double distance_sq(const Vec2& a, const Vec2& b) {
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    return dx * dx + dy * dy;
}

double vector_norm(const Vec2& value) {
    return std::hypot(value.x, value.y);
}

double dot_product(const Vec2& a, const Vec2& b) {
    return a.x * b.x + a.y * b.y;
}

Vec2 normalize_with_fallback(const Vec2& value, const Vec2& fallback) {
    const double value_norm = vector_norm(value);
    if (value_norm > 1e-6) {
        return {value.x / value_norm, value.y / value_norm};
    }

    const double fallback_norm = vector_norm(fallback);
    if (fallback_norm > 1e-6) {
        return {fallback.x / fallback_norm, fallback.y / fallback_norm};
    }

    return {1.0, 0.0};
}

bool is_inside_bounds(const WorldMap& world, const Vec2& position, double margin) {
    const Rect& bounds = world.bounds();
    const double min_x = bounds.min_x - margin;
    const double max_x = bounds.max_x + margin;
    const double min_y = bounds.min_y - margin;
    const double max_y = bounds.max_y + margin;
    return position.x >= min_x && position.x <= max_x &&
           position.y >= min_y && position.y <= max_y;
}

bool is_inside_bounds(const WorldMap& world, const Vec2& position) {
    return is_inside_bounds(world, position, 0.0);
}

bool point_in_expanded_rect(const Vec2& point, const Rect& rect, double padding) {
    return point.x >= rect.min_x - padding &&
           point.x <= rect.max_x + padding &&
           point.y >= rect.min_y - padding &&
           point.y <= rect.max_y + padding;
}

bool compact_hardware_mixed_lab_world(const WorldMap& world) {
    return world_is_mixed(world) &&
           world.unstructured_preset() == UnstructuredMapPreset::HardwareLab &&
           world_span_m(world) <= 0.85 &&
           !world.road_centerline().empty();
}

bool lidar_pressure_from_hits(const WorldMap& world,
                              const std::vector<LidarHit>& hits,
                              double yaw,
                              double max_distance,
                              double front_half_angle,
                              bool front_only) {
    const bool micro_hardware_lab = compact_hardware_mixed_lab_world(world);
    const int required_hits = micro_hardware_lab ? (front_only ? 2 : 4) : 1;
    int pressure_hits = 0;
    for (const LidarHit& hit : hits) {
        if (!hit.hit ||
            !(hit.distance > 0.0) ||
            hit.distance > max_distance) {
            continue;
        }
        if (front_only) {
            const double local_angle = std::abs(wrap_angle(hit.angle - yaw));
            if (local_angle > front_half_angle) {
                continue;
            }
        }
        ++pressure_hits;
        if (pressure_hits >= required_hits) {
            return true;
        }
    }
    return false;
}

double distance_to_polyline_m(const std::vector<Vec2>& points, const Vec2& position) {
    if (points.empty()) {
        return std::numeric_limits<double>::infinity();
    }
    if (points.size() == 1) {
        return distance(points.front(), position);
    }

    double best_distance_sq = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i + 1 < points.size(); ++i) {
        const Vec2 a = points[i];
        const Vec2 b = points[i + 1];
        const Vec2 segment{b.x - a.x, b.y - a.y};
        const double segment_len_sq = dot_product(segment, segment);
        if (!(segment_len_sq > 1e-12)) {
            continue;
        }
        const Vec2 from_a{position.x - a.x, position.y - a.y};
        const double t = clamp_value(dot_product(from_a, segment) / segment_len_sq, 0.0, 1.0);
        const Vec2 projected{a.x + segment.x * t, a.y + segment.y * t};
        best_distance_sq = std::min(best_distance_sq, distance_sq(position, projected));
    }
    return std::sqrt(best_distance_sq);
}

Vec2 clamp_point_to_bounds(const WorldMap& world, const Vec2& position, double margin = 0.0) {
    const Rect& bounds = world.bounds();
    const double min_x = bounds.min_x + margin;
    const double max_x = bounds.max_x - margin;
    const double min_y = bounds.min_y + margin;
    const double max_y = bounds.max_y - margin;
    return {
        clamp_value(position.x, std::min(min_x, max_x), std::max(min_x, max_x)),
        clamp_value(position.y, std::min(min_y, max_y), std::max(min_y, max_y)),
    };
}

Vec2 lidar_origin_world(const Vec2& base_position, double base_yaw, const LidarLocalizationConfig& cfg) {
    const Vec2 local_offset{cfg.lidar_x_offset, cfg.lidar_y_offset};
    const Vec2 rotated = rotate(local_offset, base_yaw);
    return {base_position.x + rotated.x, base_position.y + rotated.y};
}

double lidar_sensor_angle_rad(const LidarLocalizationConfig& cfg, double angle_deg) {
    const double raw_angle = deg_to_rad(angle_deg);
    return wrap_angle(cfg.lidar_flip_left_right ? -raw_angle : raw_angle);
}

double lidar_body_local_angle_rad(const LidarLocalizationConfig& cfg, double angle_deg) {
    return wrap_angle(cfg.lidar_yaw_offset + lidar_sensor_angle_rad(cfg, angle_deg));
}

double planning_lidar_range(const HardwarePlannerConfig& config) {
    return clamp_value(
        config.gap_extraction.planning_max_range_m,
        config.localization.min_valid_range_m + 0.05,
        config.localization.max_range_m);
}

double sector_max_clearance(const std::vector<LidarHit>& hits,
                            double yaw,
                            double center_local_angle,
                            double half_angle_width) {
    double best = 0.0;
    for (const LidarHit& hit : hits) {
        if (!hit.hit || !(hit.distance > 0.0)) {
            continue;
        }
        const double local_angle = wrap_angle(hit.angle - yaw);
        const double sector_delta = std::abs(wrap_angle(local_angle - center_local_angle));
        if (sector_delta <= half_angle_width) {
            best = std::max(best, hit.distance);
        }
    }
    return best;
}

double sector_min_clearance(const std::vector<LidarHit>& hits,
                            double yaw,
                            double center_local_angle,
                            double half_angle_width,
                            double fallback_clearance) {
    double best = std::numeric_limits<double>::infinity();
    for (const LidarHit& hit : hits) {
        if (!hit.hit || !(hit.distance > 0.0)) {
            continue;
        }
        const double local_angle = wrap_angle(hit.angle - yaw);
        const double sector_delta = std::abs(wrap_angle(local_angle - center_local_angle));
        if (sector_delta <= half_angle_width) {
            best = std::min(best, hit.distance);
        }
    }
    return std::isfinite(best) ? best : fallback_clearance;
}

int signum(int value) {
    return (value > 0) - (value < 0);
}

bool controller_encoders_ready(const ControllerTelemetry& telemetry) {
    const bool status_ready =
        (telemetry.status_flags & static_cast<std::uint16_t>(StatusFlag::EncodersReady)) != 0U;
    const bool left_valid =
        (telemetry.enc_flags & static_cast<std::uint16_t>(EncoderFlag::LeftValid)) != 0U;
    const bool right_valid =
        (telemetry.enc_flags & static_cast<std::uint16_t>(EncoderFlag::RightValid)) != 0U;
    return telemetry.have_encoder && status_ready && left_valid && right_valid && telemetry.enc_dt_ms > 0U;
}

bool encoder_flag_set(std::uint16_t flags, EncoderFlag flag) {
    return (flags & static_cast<std::uint16_t>(flag)) != 0U;
}

double signed_encoder_distance(std::int32_t delta_ticks, bool dir_negative, double ticks_to_distance) {
    // Current tank firmware sends monotonic tick counters plus direction flags.
    // Some intermediate test firmware sent signed counters; negative deltas are
    // already directional, so do not apply the flag a second time.
    if (delta_ticks < 0) {
        return static_cast<double>(delta_ticks) * ticks_to_distance;
    }

    const double signed_delta = dir_negative
        ? -static_cast<double>(delta_ticks)
        : static_cast<double>(delta_ticks);
    return signed_delta * ticks_to_distance;
}

double forward_speed_from_pwm_estimate(const ControllerTelemetry& telemetry,
                                       const VehicleGeometry& geometry,
                                       const MotorPwmMapperConfig& pwm_config) {
    const int canonical_left_pwm = remove_signed_pwm_calibration(
        telemetry.pwm_l,
        pwm_config.left_command_scale,
        pwm_config.left_command_offset,
        pwm_config.max_pwm);
    const int canonical_right_pwm = remove_signed_pwm_calibration(
        telemetry.pwm_r,
        pwm_config.right_command_scale,
        pwm_config.right_command_offset,
        pwm_config.max_pwm);
    const double left_speed = wheel_speed_from_pwm_estimate(
        canonical_left_pwm, geometry.left_pwm_scale, geometry);
    const double right_speed = wheel_speed_from_pwm_estimate(
        canonical_right_pwm, geometry.right_pwm_scale, geometry);
    return 0.5 * (left_speed + right_speed);
}

double structured_tracking_speed_scale(double distance_to_goal,
                                       double slowdown_radius,
                                       double heading_error_deg,
                                       double cross_track_error,
                                       double curvature,
                                       const VehicleGeometry& geometry) {
    const double goal_scale =
        0.55 + 0.45 * clamp_value(distance_to_goal / std::max(slowdown_radius, 0.25), 0.0, 1.0);
    const double heading_scale =
        1.0 - 0.55 * clamp_value(std::abs(heading_error_deg) / 45.0, 0.0, 1.0);
    const double cross_track_scale =
        1.0 - 0.45 * clamp_value(std::abs(cross_track_error) / 0.35, 0.0, 1.0);
    const double curvature_scale =
        1.0 - 0.30 * clamp_value(std::abs(curvature) / std::max(geometry.max_curvature, 1e-3), 0.0, 1.0);
    return clamp_value(goal_scale * heading_scale * cross_track_scale * curvature_scale, 0.28, 1.0);
}

VehicleGeometry make_vehicle_geometry(const HardwarePlannerConfig& config) {
    VehicleGeometry geometry{};
    geometry.wheelbase = config.drive.wheelbase;
    geometry.cg_to_front = config.drive.cg_to_front;
    geometry.cg_to_rear = config.drive.cg_to_rear;
    geometry.track = config.drive.track_width;
    geometry.body_length = config.drive.body_length;
    geometry.body_width = config.drive.body_width;
    geometry.wheel_length = config.drive.wheel_or_belt_length;
    geometry.wheel_width = config.drive.wheel_or_belt_width;
    geometry.wheel_radius = config.drive.wheel_radius;
    geometry.max_steer_angle = config.drive.max_steer_angle;
    geometry.max_steer_rate = config.drive.max_steer_rate;
    geometry.max_curvature = config.drive.max_curvature;
    geometry.max_linear_speed = config.drive.max_linear_speed;
    geometry.max_yaw_rate = config.drive.max_yaw_rate;
    geometry.max_accel = config.drive.max_accel;
    geometry.max_decel = config.drive.max_decel;
    geometry.max_pwm = config.pwm.max_pwm;
    geometry.min_effective_pwm = config.pwm.min_effective_pwm;
    geometry.wheel_speed_to_pwm_gain = config.pwm.wheel_speed_to_pwm_gain;
    geometry.wheel_speed_to_pwm_bias = config.pwm.wheel_speed_to_pwm_bias;
    geometry.speed_estimate_per_pwm =
        config.pwm.wheel_speed_to_pwm_gain > 1e-9
            ? 1.0 / config.pwm.wheel_speed_to_pwm_gain
            : 0.0;
    geometry.left_pwm_scale = config.pwm.left_scale;
    geometry.right_pwm_scale = config.pwm.right_scale;
    geometry.linear_feedback_gain = config.pwm.linear_feedback_gain;
    geometry.yaw_feedback_gain = config.pwm.yaw_feedback_gain;
    geometry.encoder_ticks_per_revolution = config.drive.encoder_ticks_per_revolution;
    return geometry;
}

bool project_curvilinear_state(const clothoid_info& clothoid,
                               const Vec2& position,
                               double s_hint,
                               bool wrap_loop,
                               bool prefer_hinted_loop_projection,
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

    const bool compact_closed_loop =
        prefer_hinted_loop_projection && wrap_loop && road_length <= 2.50;
    const double hint_s = std::isfinite(s_hint)
                              ? (wrap_loop ? wrap_arc_length(s_hint, road_length)
                                           : clamp_value(s_hint, 0.0, road_length))
                              : 0.0;
    const double backward_search =
        compact_closed_loop ? clamp_value(0.18 * road_length, 0.10, 0.22) : 2.0;
    const double forward_search =
        compact_closed_loop ? clamp_value(0.52 * road_length, 0.35, 0.70) : 8.0;
    if (std::isfinite(s_hint)) {
        double lo = wrap_loop ? hint_s - backward_search
                              : clamp_value(hint_s - backward_search, 0.0, road_length);
        double hi = wrap_loop ? hint_s + forward_search
                              : clamp_value(hint_s + forward_search, 0.0, road_length);
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
        const double candidate_s =
            wrap_loop ? wrap_arc_length(iso_s, road_length) : clamp_value(iso_s, 0.0, road_length);
        bool accept_iso_projection = true;
        if (compact_closed_loop && std::isfinite(s_hint)) {
            const double arc_delta = signed_wrapped_arc_delta(hint_s, candidate_s, road_length);
            accept_iso_projection =
                arc_delta >= -backward_search - 1e-6 &&
                arc_delta <= forward_search + 1e-6;
        }
        if (accept_iso_projection) {
            best_s = candidate_s;
            best_distance = iso_dst;
            best_x = iso_x;
            best_y = iso_y;
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
    const double steer = std::atan(
        geometry.wheelbase *
        clamp_value(curvature, -geometry.max_curvature, geometry.max_curvature));
    return clamp_value(steer, -geometry.max_steer_angle, geometry.max_steer_angle);
}

double curvature_from_steer(const VehicleGeometry& geometry, double steer_angle) {
    return clamp_value(
        std::tan(steer_angle) / std::max(geometry.wheelbase, 1e-6),
        -geometry.max_curvature,
        geometry.max_curvature);
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

VehicleModelState build_tracking_state(const Vec2& position,
                                       double yaw,
                                       double speed,
                                       double accel,
                                       double curvature,
                                       double yaw_rate,
                                       double steer_angle) {
    VehicleModelState tracking_state{};
    tracking_state.position = position;
    tracking_state.yaw = yaw;
    tracking_state.speed = speed;
    tracking_state.accel = accel;
    tracking_state.curvature = curvature;
    tracking_state.yaw_rate = yaw_rate;
    tracking_state.steer_angle = steer_angle;
    return tracking_state;
}

double planner_speed_limit(double cruise_speed_limit, double curvature) {
    constexpr double kStraightCurvature = 0.1;
    if (!std::isfinite(curvature) || std::abs(curvature) < kStraightCurvature) {
        return cruise_speed_limit;
    }

    const double curvature_limit = 1.5 * std::pow(std::abs(curvature), -1.0 / 3.0);
    return clamp_value(curvature_limit, 0.15, cruise_speed_limit);
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

bool points_form_closed_loop(const std::vector<Vec2>& points, double threshold) {
    return points.size() >= 3 && distance(points.front(), points.back()) <= threshold;
}

bool structured_road_is_closed_loop(const WorldMap& world) {
    if (!world_has_structured_reference(world) ||
        world.road_centerline().size() < 3) {
        return false;
    }
    const double course_span = structured_course_span_m(world);
    const double closure_threshold = std::clamp(0.12 * course_span, 0.05, 0.18);
    return distance(world.start(), world.goal()) <= closure_threshold &&
           points_form_closed_loop(world.road_centerline(), closure_threshold);
}

bool tiny_indoor_structured_loop(const WorldMap& world) {
    return structured_road_is_closed_loop(world) &&
           micro_structured_world(world) &&
           (structured_course_span_m(world) <= 0.32 ||
            structured_road_length_m(world) <= 0.95);
}

bool simple_single_turn_structured_loop(const WorldMap& world) {
    return structured_road_is_closed_loop(world) &&
           world.structured_preset() != StructuredMapPreset::FigureEight &&
           world.structured_preset() != StructuredMapPreset::TankCircuit &&
           world.structured_preset() != StructuredMapPreset::ZigZag;
}

double structured_loop_winding_sign(const WorldMap& world) {
    const std::vector<Vec2>& points = world.road_centerline();
    if (points.size() < 3) {
        return 0.0;
    }

    double signed_area_twice = 0.0;
    for (size_t i = 0; i < points.size(); ++i) {
        const Vec2& a = points[i];
        const Vec2& b = points[(i + 1) % points.size()];
        signed_area_twice += a.x * b.y - b.x * a.y;
    }
    if (std::abs(signed_area_twice) < 1e-9) {
        return 0.0;
    }
    return signed_area_twice > 0.0 ? 1.0 : -1.0;
}

bool compact_mixed_structured_loop(const WorldMap& world) {
    return world_is_mixed(world) &&
           structured_road_is_closed_loop(world) &&
           world_span_m(world) <= 2.50 &&
           structured_course_span_m(world) <= 1.20;
}

bool compact_mixed_open_road(const WorldMap& world) {
    return world_is_mixed(world) &&
           !structured_road_is_closed_loop(world) &&
           world_has_structured_reference(world) &&
           world_span_m(world) <= 2.50 &&
           structured_course_span_m(world) <= 1.20;
}

double compact_mixed_gate_acceptance_radius_m(const WorldMap& world) {
    if (!world_is_mixed(world)) {
        return 0.0;
    }
    const double span = world_span_m(world);
    if (span <= 0.65) {
        return clamp_value(0.07 * span, 0.032, 0.045);
    }
    return span <= 1.25 ? 0.10 : 0.12;
}

double compact_mixed_gate_validation_radius_m(const WorldMap& world) {
    const double nominal_radius = compact_mixed_gate_acceptance_radius_m(world);
    if (!(nominal_radius > 0.0)) {
        return 0.0;
    }
    return nominal_radius + (world_span_m(world) <= 0.65 ? 0.012 : (world_span_m(world) <= 1.25 ? 0.02 : 0.0));
}

double distance_to_gate_point(const gate& candidate, const Vec2& position) {
    return std::hypot(candidate.x_pos - position.x, candidate.y_pos - position.y);
}

std::uint64_t quantized_point_key(const Vec2& point, double resolution_m) {
    const double safe_resolution = std::max(resolution_m, 1e-3);
    const std::int32_t qx = static_cast<std::int32_t>(std::lround(point.x / safe_resolution));
    const std::int32_t qy = static_cast<std::int32_t>(std::lround(point.y / safe_resolution));
    return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(qx)) << 32U) |
           static_cast<std::uint32_t>(qy);
}

Vec2 quantized_point_center(const Vec2& point, double resolution_m) {
    const double safe_resolution = std::max(resolution_m, 1e-3);
    const double qx = std::round(point.x / safe_resolution) * safe_resolution;
    const double qy = std::round(point.y / safe_resolution) * safe_resolution;
    return {qx, qy};
}

double point_segment_distance_sq(const Vec2& point, const Vec2& a, const Vec2& b) {
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double segment_length_sq = dx * dx + dy * dy;
    if (segment_length_sq <= 1e-12) {
        return distance_sq(point, a);
    }

    const double t = clamp_value(
        ((point.x - a.x) * dx + (point.y - a.y) * dy) / segment_length_sq,
        0.0,
        1.0);
    const Vec2 projection{a.x + dx * t, a.y + dy * t};
    return distance_sq(point, projection);
}

struct LocalOccupancyGrid {
    double resolution_m = 0.05;
    int width = 0;
    int height = 0;
    Vec2 origin{};
    std::vector<std::uint16_t> free_votes;
    std::vector<std::uint16_t> occupied_votes;
    std::vector<std::uint8_t> inflated_mask;
    std::vector<std::uint8_t> free_mask;

    int index(int x, int y) const {
        return y * width + x;
    }

    bool valid_cell(int x, int y) const {
        return x >= 0 && x < width && y >= 0 && y < height;
    }

    bool world_to_cell(const Vec2& point, int* cell_x, int* cell_y) const {
        const int x = static_cast<int>(std::floor((point.x - origin.x) / resolution_m));
        const int y = static_cast<int>(std::floor((point.y - origin.y) / resolution_m));
        if (!valid_cell(x, y)) {
            return false;
        }
        if (cell_x != nullptr) {
            *cell_x = x;
        }
        if (cell_y != nullptr) {
            *cell_y = y;
        }
        return true;
    }

    Vec2 cell_center(int x, int y) const {
        return {
            origin.x + (static_cast<double>(x) + 0.5) * resolution_m,
            origin.y + (static_cast<double>(y) + 0.5) * resolution_m,
        };
    }

    void add_free_sample(const Vec2& point) {
        int x = 0;
        int y = 0;
        if (!world_to_cell(point, &x, &y)) {
            return;
        }
        std::uint16_t& value = free_votes[static_cast<size_t>(index(x, y))];
        value = static_cast<std::uint16_t>(std::min<int>(value + 1, std::numeric_limits<std::uint16_t>::max()));
    }

    void add_occupied_sample(const Vec2& point) {
        int x = 0;
        int y = 0;
        if (!world_to_cell(point, &x, &y)) {
            return;
        }
        std::uint16_t& value = occupied_votes[static_cast<size_t>(index(x, y))];
        value = static_cast<std::uint16_t>(std::min<int>(value + 3, std::numeric_limits<std::uint16_t>::max()));
    }

    bool is_inflated(const Vec2& point) const {
        int x = 0;
        int y = 0;
        if (!world_to_cell(point, &x, &y)) {
            return true;
        }
        return inflated_mask[static_cast<size_t>(index(x, y))] != 0U;
    }

    bool is_free(const Vec2& point) const {
        int x = 0;
        int y = 0;
        if (!world_to_cell(point, &x, &y)) {
            return false;
        }
        return free_mask[static_cast<size_t>(index(x, y))] != 0U;
    }

    void finalize(double inflate_radius_m) {
        inflated_mask.assign(free_votes.size(), 0U);
        free_mask.assign(free_votes.size(), 0U);

        const int inflation_cells = std::max(1, static_cast<int>(std::ceil(inflate_radius_m / std::max(resolution_m, 1e-3))));
        const int inflation_cells_sq = inflation_cells * inflation_cells;
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                if (occupied_votes[static_cast<size_t>(index(x, y))] == 0U) {
                    continue;
                }
                for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
                    for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
                        if (dx * dx + dy * dy > inflation_cells_sq) {
                            continue;
                        }
                        const int nx = x + dx;
                        const int ny = y + dy;
                        if (!valid_cell(nx, ny)) {
                            continue;
                        }
                        inflated_mask[static_cast<size_t>(index(nx, ny))] = 1U;
                    }
                }
            }
        }

        for (size_t i = 0; i < free_votes.size(); ++i) {
            free_mask[i] = free_votes[i] > 0U && inflated_mask[i] == 0U ? 1U : 0U;
        }
    }
};

template <typename Fn>
void sample_segment_points(const Vec2& start,
                           const Vec2& end,
                           double step_m,
                           bool include_endpoint,
                           Fn&& fn) {
    const double length = distance(start, end);
    if (!(length > 1e-6)) {
        if (include_endpoint) {
            fn(end);
        }
        return;
    }

    const int steps = std::max(1, static_cast<int>(std::ceil(length / std::max(step_m, 1e-3))));
    const int last_step = include_endpoint ? steps : steps - 1;
    for (int i = 0; i <= last_step; ++i) {
        const double alpha = clamp_value(
            static_cast<double>(i) / static_cast<double>(steps),
            0.0,
            1.0);
        fn({
            start.x + (end.x - start.x) * alpha,
            start.y + (end.y - start.y) * alpha,
        });
    }
}

bool grid_segment_is_clear(const LocalOccupancyGrid& grid, const Vec2& start, const Vec2& end) {
    const double length = distance(start, end);
    if (!(length > 1e-6)) {
        return true;
    }

    int unsupported_samples = 0;
    const double step_m = std::max(grid.resolution_m * 0.6, 0.025);
    const int steps = std::max(1, static_cast<int>(std::ceil(length / step_m)));
    for (int i = 1; i <= steps; ++i) {
        const double alpha = static_cast<double>(i) / static_cast<double>(steps);
        const Vec2 point{
            start.x + (end.x - start.x) * alpha,
            start.y + (end.y - start.y) * alpha,
        };
        if (grid.is_inflated(point)) {
            return false;
        }
        const double traveled = alpha * length;
        if (traveled > 0.12 && !grid.is_free(point)) {
            ++unsupported_samples;
            if (unsupported_samples > 1) {
                return false;
            }
        }
    }

    return true;
}

double grid_local_clearance_score(const LocalOccupancyGrid& grid,
                                  int cell_x,
                                  int cell_y,
                                  int radius_cells) {
    int considered = 0;
    int free_cells = 0;
    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
        for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
            if (dx * dx + dy * dy > radius_cells * radius_cells) {
                continue;
            }
            const int nx = cell_x + dx;
            const int ny = cell_y + dy;
            if (!grid.valid_cell(nx, ny)) {
                continue;
            }
            const size_t idx = static_cast<size_t>(grid.index(nx, ny));
            if (grid.inflated_mask[idx] != 0U) {
                continue;
            }
            ++considered;
            if (grid.free_mask[idx] != 0U) {
                ++free_cells;
            }
        }
    }

    if (considered <= 0) {
        return 0.0;
    }
    return static_cast<double>(free_cells) / static_cast<double>(considered);
}

}  // namespace

const char* hardware_localization_policy_name(HardwareLocalizationPolicy policy) {
    switch (policy) {
        case HardwareLocalizationPolicy::EncoderImuPrimary:
            return "Encoder/IMU primary";
        case HardwareLocalizationPolicy::LidarPoseCorrection:
            return "LiDAR pose correction";
        case HardwareLocalizationPolicy::Auto:
        default:
            return "Auto";
    }
}

const char* hardware_localization_policy_cli_name(HardwareLocalizationPolicy policy) {
    switch (policy) {
        case HardwareLocalizationPolicy::EncoderImuPrimary:
            return "encoder";
        case HardwareLocalizationPolicy::LidarPoseCorrection:
            return "lidar";
        case HardwareLocalizationPolicy::Auto:
        default:
            return "auto";
    }
}

}  // namespace thesis_sim
