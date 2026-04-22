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

bool points_form_closed_loop(const std::vector<Vec2>& points, double threshold) {
    return points.size() >= 3 && distance(points.front(), points.back()) <= threshold;
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

bool compact_structured_world(const WorldMap& world) {
    if (world.environment_mode() != EnvironmentMode::StructuredRoad) {
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
    return clamp_value(curvature_limit, 0.15, cruise_speed_limit);
}

bool structured_road_is_closed_loop(const WorldMap& world) {
    return world.environment_mode() == EnvironmentMode::StructuredRoad &&
           points_form_closed_loop(world.road_centerline(), 0.45);
}

double distance_to_gate_point(const gate& candidate, const Vec2& position) {
    return std::hypot(candidate.x_pos - position.x, candidate.y_pos - position.y);
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
    if (micro_structured_world(world_)) {
        geometry_.max_steer_angle = std::max(geometry_.max_steer_angle, 1.35);
        geometry_.max_steer_rate = std::max(geometry_.max_steer_rate, 8.00);
        geometry_.max_curvature = std::max(geometry_.max_curvature, 14.0);
    } else if (compact_structured_world(world_)) {
        geometry_.max_steer_angle = std::max(geometry_.max_steer_angle, 1.10);
        geometry_.max_steer_rate = std::max(geometry_.max_steer_rate, 4.50);
        geometry_.max_curvature = std::max(geometry_.max_curvature, 5.50);
    }
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
    }
    mpc_follower_ = KinematicBicycleMpcFollower(mpc_config);

    step_count_ = 0;
    sim_time_ = 0.0;
    last_j_ = 0.0;
    last_r_ = 0.0;
    chosen_gate_index_ = -1;
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
    const bool compact_structured_world =
        compact_world && world_.environment_mode() == EnvironmentMode::StructuredRoad;
    const bool micro_structured_world =
        compact_structured_world && world_span <= 0.35;

    sim_ = {};
    sim_.W = micro_structured_world ? clamp_value(world_span * 0.42, 0.10, 0.16)
             : (compact_structured_world ? 0.32 : (compact_world ? 0.90 : 3.0));
    sim_.T_max = micro_structured_world ? 4.0 : (compact_world ? 8.0 : 20.0);
    sim_.la = micro_structured_world ? 0.35 : (compact_world ? 1.20 : 8.0);
    sim_.la_stop = micro_structured_world ? 0.70 : (compact_world ? 2.40 : 18.0);
    sim_.z_coord = 0.1;
    sim_.veh_W = geometry_.body_width;
    sim_.veh_L = geometry_.body_length;
    sim_.end_sim = micro_structured_world ? std::max(world_span * 8.0, 3.0)
                   : (compact_world ? std::max(world_span * 4.0, 6.0) : 200.0);
    sim_.tol_obst = micro_structured_world ? 0.06 : (compact_world ? 0.18 : 0.25);
    sim_.lat_tol = micro_structured_world ? 0.04 : (compact_world ? 0.12 : 0.2);
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
    if (world_.environment_mode() == EnvironmentMode::UnstructuredGates) {
        world_.set_gate_behavior(config_.gate_behavior, config_.gate_seed);
        world_.update_gate_layout(0.0);
    }
    sync_gate_specs_from_world(true);
    refresh_gate_diagnostics();
    sync_planner_from_vehicle(true);

    update_vehicle_snapshot();
    update_lidar();
    update_selected_trajectory();
    if (structured_road_is_closed_loop(world_)) {
        distance_to_goal_ = structured_goal_ready_ ? structured_goal_progress_target_ : cl_.end_point_s;
    } else {
        distance_to_goal_ = distance(world_.start(), world_.goal());
    }
    update_telemetry();
}

void PlannerDrivenVehicleSim::rebuild_vehicle_model() {
    config_.vehicle_model = VehicleModelKind::CarLikeBicycle;
    config_.tracking_controller = TrackingControllerMode::MpcPathFollower;
    vehicle_model_ = make_four_wheel_car_model(geometry_);
}

void PlannerDrivenVehicleSim::load_world(WorldMap world) {
    world_ = std::move(world);
    reset();
}

void PlannerDrivenVehicleSim::sync_road_from_world() {
    if (world_.environment_mode() != EnvironmentMode::StructuredRoad || world_.road_centerline().size() < 2) {
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
    if (world_.environment_mode() != EnvironmentMode::UnstructuredGates) {
        gates_.clear();
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
    const double odom_delta_yaw = std::abs(geometry_.track) > 1e-6 ? (right_dist - left_dist) / geometry_.track : 0.0;
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

        if (world_.environment_mode() == EnvironmentMode::UnstructuredGates) {
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
}

void PlannerDrivenVehicleSim::refresh_gate_diagnostics() {
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
}

void PlannerDrivenVehicleSim::update_gate_activation_window() {
    if (world_.environment_mode() != EnvironmentMode::UnstructuredGates || gates_.empty()) {
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

    for (size_t i = 0; i < gates_.size(); ++i) {
        if (gates_[i].passed) {
            gates_[i].too_far = false;
            gates_[i].choose = false;
            continue;
        }

        const bool is_primary = static_cast<int>(i) == primary_index;
        gates_[i].too_far = !is_primary;
        if (gates_[i].too_far) {
            gates_[i].choose = false;
        }
    }
}

std::vector<int> PlannerDrivenVehicleSim::active_gate_indices() const {
    std::vector<int> indices;
    if (world_.environment_mode() != EnvironmentMode::UnstructuredGates) {
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

    if (world_.environment_mode() == EnvironmentMode::StructuredRoad && road_ != nullptr) {
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

    const std::vector<int> active_indices = active_gate_indices();
    if (gates_.empty() || active_indices.empty()) {
        last_j_ = 0.0;
        last_r_ = 0.0;
        chosen_gate_index_ = -1;
        planned_trajectory_.clear();
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

    const std::array<Vec2, 4> wheel_local{{
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
        vehicle_.wheels[i].steering = config_.vehicle_model == VehicleModelKind::CarLikeBicycle && is_front;
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
    const bool unstructured = world_.environment_mode() == EnvironmentMode::UnstructuredGates;
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

void PlannerDrivenVehicleSim::set_vehicle_stack(VehicleModelKind model, TrackingControllerMode controller) {
    (void)model;
    (void)controller;
    config_.vehicle_model = VehicleModelKind::CarLikeBicycle;
    config_.tracking_controller = TrackingControllerMode::MpcPathFollower;
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
    history_.push_back({
        sim_time_,
        vehicle_.speed,
        vehicle_.accel,
        last_j_,
        vehicle_.curvature,
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

    if (world_.environment_mode() == EnvironmentMode::StructuredRoad &&
        (!has_planner_clothoid || world_.road_centerline().size() < 2)) {
        planned_trajectory_ = world_.road_centerline();
        reference_trajectory_ = build_reference_waypoints(planned_trajectory_, planner_speed_ref_);
        return;
    }

    if (world_.environment_mode() == EnvironmentMode::UnstructuredGates && chosen_gate_index_ < 0) {
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
    x0_.x = s_current;
    x0_.n = lateral_offset;
    x0_.v = navigation_speed_;
    x0_.a = navigation_accel_;
    x0_.c = navigation_curvature_;

    if (closed_structured_loop) {
        if (!structured_goal_ready_) {
            const double goal_epsilon = std::clamp(0.02 * cl_.end_point_s, 0.25, 1.0);
            structured_goal_progress_target_ = std::max(0.0, cl_.end_point_s - goal_epsilon);
            structured_goal_position_ = world_.start();
            structured_goal_ready_ = structured_goal_progress_target_ > 0.0;
        }

        if (!std::isfinite(structured_last_s_)) {
            structured_last_s_ = s_current;
            structured_progress_s_ = 0.0;
        } else {
            double delta_s = s_current - structured_last_s_;
            if (delta_s < -0.5 * cl_.end_point_s) {
                delta_s += cl_.end_point_s;
            } else if (delta_s > 0.5 * cl_.end_point_s) {
                delta_s -= cl_.end_point_s;
            }
            structured_progress_s_ = std::max(0.0, structured_progress_s_ + delta_s);
            structured_last_s_ = s_current;
        }
    }

    const bool unstructured = world_.environment_mode() == EnvironmentMode::UnstructuredGates;
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
    if (world_.environment_mode() == EnvironmentMode::UnstructuredGates &&
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
            return world_.environment_mode() == EnvironmentMode::UnstructuredGates ? 3 : 2;
        case RangeSensorProfile::ShortRangeScanner:
            return world_.environment_mode() == EnvironmentMode::UnstructuredGates ? 2 : 1;
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
    refresh_gate_diagnostics();

    const auto planning_start = std::chrono::steady_clock::now();
    plan_if_needed();
    update_planner_references(config_.dt);
    update_selected_trajectory();
    planning_compute_ms_ = elapsed_ms(planning_start, std::chrono::steady_clock::now());

    VehicleControlInput control{};
    control.jerk_cmd = last_j_;
    control.planner_r_cmd = last_r_;
    control.target_speed = planner_speed_ref_;
    tracker_cross_track_error_ = 0.0;
    tracker_heading_error_deg_ = 0.0;

    const auto tracking_start = std::chrono::steady_clock::now();
    if (config_.vehicle_model == VehicleModelKind::CarLikeBicycle) {
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
                planner_speed_ref_,
                0);
            if (mpc_command.valid) {
                control.accel_cmd = mpc_command.accel_cmd;
                control.steer_rate_cmd = mpc_command.steer_rate_cmd;
                control.target_speed = mpc_command.target_speed;
                control.target_steer_angle = mpc_command.target_steer_angle;
                tracker_cross_track_error_ = std::abs(mpc_command.cross_track_error);
                tracker_heading_error_deg_ = std::abs(mpc_command.heading_error) * 180.0 / 3.14159265358979323846;
                last_mpc_command_ = mpc_command;
            } else {
                last_mpc_command_.reset();
            }
        } else {
            const double target_curvature = !reference_trajectory_.empty() ? reference_trajectory_.front().curvature : 0.0;
            control.accel_cmd = clamp_value(
                (planner_speed_ref_ - tracking_state.speed) / std::max(config_.dt, 1e-3),
                -geometry_.max_decel,
                geometry_.max_accel);
            control.target_steer_angle = steer_from_curvature(geometry_, target_curvature);
            control.steer_rate_cmd = clamp_value(
                (control.target_steer_angle - tracking_state.steer_angle) / std::max(config_.dt, 1e-3),
                -geometry_.max_steer_rate,
                geometry_.max_steer_rate);
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
    collision_ = world_.collides(vehicle_.body_corners);
    if (structured_road_is_closed_loop(world_)) {
        const double goal_position_distance =
            distance(vehicle_.position, structured_goal_ready_ ? structured_goal_position_ : world_.start());
        distance_to_goal_ = structured_goal_ready_
                                ? std::max(structured_goal_progress_target_ - structured_progress_s_, 0.0)
                                : cl_.end_point_s;
        const double progress_margin = std::max(0.8, 0.03 * std::max(cl_.end_point_s, 1.0));
        goal_reached_ = structured_goal_ready_ &&
                        structured_progress_s_ + progress_margin >= structured_goal_progress_target_ &&
                        goal_position_distance < 0.75;
    } else {
        distance_to_goal_ = distance(vehicle_.position, world_.goal());
        goal_reached_ = distance_to_goal_ < 0.75 && std::abs(vehicle_.speed) < 0.15;
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
    int total = 0;
    for (const gate& g : gates_) {
        if (g.passed) {
            ++total;
        }
    }
    return total;
}

}  // namespace thesis_sim
