#include "hardware_planner_runner.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <thread>
#include <utility>

namespace thesis_sim {

namespace {

constexpr double kPi = 3.14159265358979323846;

struct LidarSearchWindow {
    double xy_window = 0.0;
    double xy_step = 0.0;
    double yaw_window = 0.0;
    double yaw_step = 0.0;
};

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

double deg_to_rad(double angle_deg) {
    return angle_deg * kPi / 180.0;
}

double monotonic_seconds() {
    using clock = std::chrono::steady_clock;
    return std::chrono::duration<double>(clock::now().time_since_epoch()).count();
}

double elapsed_ms(std::chrono::steady_clock::time_point start,
                  std::chrono::steady_clock::time_point end) {
    return std::chrono::duration<double, std::milli>(end - start).count();
}

bool is_inside_bounds(const WorldMap& world, const Vec2& position) {
    const Rect& bounds = world.bounds();
    return position.x >= bounds.min_x && position.x <= bounds.max_x &&
           position.y >= bounds.min_y && position.y <= bounds.max_y;
}

Vec2 lidar_origin_world(const Vec2& base_position, double base_yaw, const LidarLocalizationConfig& cfg) {
    const Vec2 local_offset{cfg.lidar_x_offset, cfg.lidar_y_offset};
    const Vec2 rotated = rotate(local_offset, base_yaw);
    return {base_position.x + rotated.x, base_position.y + rotated.y};
}

VehicleGeometry make_vehicle_geometry(const HardwarePlannerConfig& config) {
    VehicleGeometry geometry{};
    geometry.wheelbase = config.drive.wheelbase;
    geometry.cg_to_front = config.drive.cg_to_front;
    geometry.cg_to_rear = config.drive.cg_to_rear;
    geometry.track = config.drive.track_width;
    geometry.body_length = config.drive.body_length;
    geometry.body_width = config.drive.body_width;
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

bool points_form_closed_loop(const std::vector<Vec2>& points, double threshold) {
    return points.size() >= 3 && distance(points.front(), points.back()) <= threshold;
}

bool structured_road_is_closed_loop(const WorldMap& world) {
    return world.environment_mode() == EnvironmentMode::StructuredRoad &&
           points_form_closed_loop(world.road_centerline(), 0.45);
}

}  // namespace

HardwarePlannerRunner::HardwarePlannerRunner(WorldMap world,
                                             RealRobotBridge::Options bridge_options,
                                             HardwarePlannerConfig config)
    : world_(std::move(world)),
      config_(config),
      geometry_(make_vehicle_geometry(config_)),
      bridge_(std::move(bridge_options)),
      null_stream_("/dev/null") {
    estimator_.set_geometry(geometry_);
    initialize_planner_state();
    reset();
}

void HardwarePlannerRunner::connect() {
    if (connected_) {
        return;
    }

    bridge_.connect(true);
    connected_ = true;

    if (bridge_.controller_connected()) {
        if (config_.auto_gyro_zero) {
            bridge_.gyro_zero();
        }
        if (config_.auto_set_autonomous_mode) {
            bridge_.set_mode(ControllerMode::Autonomous);
        }
    }

    const double deadline = monotonic_seconds() + 5.0;
    while (monotonic_seconds() < deadline) {
        bridge_.poll_controller(0.05);
        telemetry_ready_ = bridge_.observation().have_controller_telemetry;
        if (telemetry_ready_) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    if (!telemetry_ready_) {
        throw ProtocolResponseError("Controller telemetry not ready after connect");
    }

    reset();
    bridge_.send_pwm(0, 0, true);
}

void HardwarePlannerRunner::disconnect() {
    if (!connected_) {
        return;
    }

    try {
        if (bridge_.controller_connected()) {
            bridge_.send_pwm(0, 0, true);
            bridge_.stop(StopReason::UserRequest, false);
        }
    } catch (const std::exception&) {
    }

    bridge_.disconnect();
    connected_ = false;
}

void HardwarePlannerRunner::reset() {
    step_count_ = 0;
    sim_time_ = 0.0;
    last_j_ = 0.0;
    last_r_ = 0.0;
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
    chosen_gate_index_ = -1;
    goal_reached_ = false;
    safety_stop_active_ = false;
    yaw_offset_ = 0.0;
    last_raw_imu_yaw_ = 0.0;
    last_observation_time_ = 0.0;
    last_left_encoder_ticks_ = 0;
    last_right_encoder_ticks_ = 0;
    yaw_offset_initialized_ = false;
    have_raw_imu_yaw_ = false;
    encoder_ticks_initialized_ = false;
    history_.clear();
    trail_.clear();
    planned_trajectory_.clear();
    reference_trajectory_.clear();
    visible_gate_indices_.clear();
    lidar_hits_.clear();
    last_command_ = {};
    last_mpc_command_.reset();

    initialize_planner_state();
    sync_road_from_world();
    estimator_.set_geometry(geometry_);
    estimator_.reset(world_.start(), world_.start_heading());
    reset_pose(world_.start(), world_.start_heading());
}

void HardwarePlannerRunner::reset_pose(const Vec2& position, double heading) {
    estimate_ = {};
    estimate_.position = position;
    estimate_.yaw = heading;
    estimate_.min_lidar_distance = config_.localization.max_range_m;
    estimate_.front_lidar_distance = config_.localization.max_range_m;
    estimate_.localized = true;

    distance_to_goal_ = distance(position, world_.goal());
    planner_speed_ref_ = 0.0;
    planner_accel_ref_ = 0.0;
    tracker_cross_track_error_ = 0.0;
    tracker_heading_error_deg_ = 0.0;
    estimator_.reset(position, heading);
    sync_planner_from_estimate(true);
    sync_gate_specs_from_world(true);
    refresh_gate_diagnostics();
    update_selected_trajectory();
    if (structured_road_is_closed_loop(world_)) {
        distance_to_goal_ = structured_goal_ready_ ? structured_goal_progress_target_ : cl_.end_point_s;
    } else {
        distance_to_goal_ = distance(position, world_.goal());
    }
}

void HardwarePlannerRunner::initialize_planner_state() {
    const Rect& bounds = world_.bounds();
    const double world_width = std::max(bounds.max_x - bounds.min_x, 0.0);
    const double world_height = std::max(bounds.max_y - bounds.min_y, 0.0);
    const double world_span = std::max(world_width, world_height);
    const bool compact_world = world_span <= 5.0;

    sim_ = {};
    sim_.W = compact_world ? 0.90 : 3.0;
    sim_.T_max = compact_world ? 8.0 : 20.0;
    sim_.la = compact_world ? 1.20 : 8.0;
    sim_.la_stop = compact_world ? 2.40 : 18.0;
    sim_.z_coord = 0.1;
    sim_.veh_W = geometry_.body_width;
    sim_.veh_L = geometry_.body_length;
    sim_.end_sim = compact_world ? std::max(world_span * 4.0, 6.0) : 200.0;
    sim_.tol_obst = compact_world ? 0.18 : 0.25;
    sim_.lat_tol = compact_world ? 0.12 : 0.2;
    sim_.DT = static_cast<float>(config_.nominal_dt);
    sim_.V_max = config_.cruise_speed_limit;

    x0_ = {};
    g_x0_ = {};
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
}

void HardwarePlannerRunner::initialize_gates() {
    gates_.clear();
    for (const GateSpec& spec : world_.gates()) {
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
}

void HardwarePlannerRunner::sync_road_from_world() {
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

void HardwarePlannerRunner::sync_gate_specs_from_world(bool reset_flags) {
    const std::vector<GateSpec>& specs = world_.gates();
    if (world_.environment_mode() != EnvironmentMode::UnstructuredGates) {
        gates_.clear();
        return;
    }
    if (reset_flags || gates_.size() != specs.size()) {
        initialize_gates();
        return;
    }

    for (size_t i = 0; i < specs.size(); ++i) {
        gates_[i].x_pos = specs[i].position.x;
        gates_[i].y_pos = specs[i].position.y;
        gates_[i].road.PSI_end = specs[i].heading_hint;
        gates_[i].final = specs[i].final;
    }
}

void HardwarePlannerRunner::sync_planner_from_estimate(bool reset_relative_state) {
    g_x0_.x_fix = estimate_.position.x;
    g_x0_.y_fix = estimate_.position.y;
    g_x0_.theta = estimate_.yaw;
    g_x0_.kappa_veh = estimate_.curvature;
    g_x0_.v_fix = estimate_.speed;
    g_x0_.a_fix = estimate_.accel;

    x0_.v = estimate_.speed;
    x0_.a = estimate_.accel;
    x0_.c = estimate_.curvature;

    if (reset_relative_state) {
        x0_.x = 0.0;
        x0_.n = 0.0;
        x0_.b = 0.0;
    }
}

void HardwarePlannerRunner::update_speed_limit() {
    sim_.V_max = planner_speed_limit(config_.cruise_speed_limit, cl_.kappa);
}

void HardwarePlannerRunner::update_gate_activation_window() {
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

std::vector<int> HardwarePlannerRunner::active_gate_indices() const {
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

void HardwarePlannerRunner::refresh_gate_diagnostics() {
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

int HardwarePlannerRunner::planning_interval_steps() const {
    return std::max(config_.control_interval_steps, 1);
}

void HardwarePlannerRunner::plan_if_needed() {
    if (step_count_ % planning_interval_steps() != 0) {
        return;
    }

    update_speed_limit();
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
        reference_trajectory_.clear();
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

    if (!std::isfinite(next_j) || !std::isfinite(next_r)) {
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
}

void HardwarePlannerRunner::update_estimate_from_observation(const RealRobotObservation& observation, double dt) {
    if (!observation.have_controller_telemetry) {
        return;
    }

    const ControllerTelemetry& telemetry = observation.controller;
    const double imu_yaw = static_cast<double>(telemetry.yaw_mrad) / 1000.0;
    last_raw_imu_yaw_ = imu_yaw;
    have_raw_imu_yaw_ = true;

    if (!yaw_offset_initialized_) {
        yaw_offset_ = wrap_angle(estimator_.state().yaw - imu_yaw);
        yaw_offset_initialized_ = true;
    }

    if (!encoder_ticks_initialized_) {
        last_left_encoder_ticks_ = telemetry.ticks_left;
        last_right_encoder_ticks_ = telemetry.ticks_right;
        encoder_ticks_initialized_ = true;
    }

    const std::int32_t left_delta_ticks = telemetry.ticks_left - last_left_encoder_ticks_;
    const std::int32_t right_delta_ticks = telemetry.ticks_right - last_right_encoder_ticks_;
    last_left_encoder_ticks_ = telemetry.ticks_left;
    last_right_encoder_ticks_ = telemetry.ticks_right;

    const double encoder_dt =
        telemetry.enc_dt_ms > 0
            ? clamp_value(static_cast<double>(telemetry.enc_dt_ms) / 1000.0, 0.01, 0.25)
            : std::max(dt, 0.01);
    const double ticks_to_distance =
        (2.0 * kPi * geometry_.wheel_radius) /
        static_cast<double>(std::max<std::int32_t>(geometry_.encoder_ticks_per_revolution, 1));
    const double left_dist = static_cast<double>(left_delta_ticks) * ticks_to_distance;
    const double right_dist = static_cast<double>(right_delta_ticks) * ticks_to_distance;
    const double odom_delta_yaw =
        std::abs(geometry_.track) > 1e-6 ? (right_dist - left_dist) / geometry_.track : 0.0;
    const double odom_speed = encoder_dt > 1e-6 ? 0.5 * (left_dist + right_dist) / encoder_dt : 0.0;
    const double odom_yaw_rate = encoder_dt > 1e-6 ? odom_delta_yaw / encoder_dt : 0.0;
    estimator_.predict(encoder_dt, odom_speed, odom_yaw_rate);

    const double measured_yaw = wrap_angle(imu_yaw + yaw_offset_);
    const double measured_yaw_rate = static_cast<double>(telemetry.yaw_rate_mrad_s) / 1000.0;
    estimator_.update_imu(measured_yaw, measured_yaw_rate);

    const EkfState& state = estimator_.state();
    estimate_.position = state.position;
    estimate_.yaw = state.yaw;
    estimate_.speed = state.speed;
    estimate_.accel = state.accel;
    estimate_.yaw_rate = state.yaw_rate;
    estimate_.curvature = std::abs(state.speed) > 0.05
                              ? clamp_value(
                                    state.yaw_rate / state.speed,
                                    -geometry_.max_curvature,
                                    geometry_.max_curvature)
                              : 0.0;
    estimate_.localized = true;
}

void HardwarePlannerRunner::correct_pose_with_lidar(const std::vector<RPLidarA1::ScanPoint>& scan) {
    if (static_cast<int>(scan.size()) < config_.localization.min_scan_points) {
        return;
    }

    const double base_score = score_candidate_pose(estimate_.position, estimate_.yaw, scan);
    Vec2 best_position = estimate_.position;
    double best_yaw = estimate_.yaw;
    double best_score = base_score;

    const std::array<LidarSearchWindow, 2> search_windows{{
        {config_.localization.xy_search_window_m,
         config_.localization.xy_search_step_m,
         config_.localization.yaw_search_window_rad,
         config_.localization.yaw_search_step_rad},
        {std::max(config_.localization.xy_search_step_m, 0.08),
         std::max(config_.localization.xy_search_step_m * 0.5, 0.04),
         std::max(config_.localization.yaw_search_step_rad, 0.03),
         std::max(config_.localization.yaw_search_step_rad * 0.5, 0.015)},
    }};

    for (const LidarSearchWindow& window : search_windows) {
        const Vec2 center = best_position;
        const double center_yaw = best_yaw;
        for (double dx = -window.xy_window; dx <= window.xy_window + 1e-9; dx += window.xy_step) {
            for (double dy = -window.xy_window; dy <= window.xy_window + 1e-9; dy += window.xy_step) {
                for (double dyaw = -window.yaw_window; dyaw <= window.yaw_window + 1e-9; dyaw += window.yaw_step) {
                    const Vec2 candidate{center.x + dx, center.y + dy};
                    const double candidate_yaw = wrap_angle(center_yaw + dyaw);
                    const double score = score_candidate_pose(candidate, candidate_yaw, scan);
                    if (score < best_score) {
                        best_score = score;
                        best_position = candidate;
                        best_yaw = candidate_yaw;
                    }
                }
            }
        }
    }

    if (std::isfinite(best_score)) {
        estimator_.update_lidar_pose(best_position, best_yaw, !yaw_offset_initialized_);
        const EkfState& state = estimator_.state();
        estimate_.position = state.position;
        estimate_.yaw = state.yaw;
        estimate_.speed = state.speed;
        estimate_.accel = state.accel;
        estimate_.yaw_rate = state.yaw_rate;
        estimate_.curvature = std::abs(state.speed) > 0.05
                                  ? clamp_value(
                                        state.yaw_rate / state.speed,
                                        -geometry_.max_curvature,
                                        geometry_.max_curvature)
                                  : 0.0;
        estimate_.localized = true;
        if (have_raw_imu_yaw_) {
            yaw_offset_ = wrap_angle(estimate_.yaw - last_raw_imu_yaw_);
            yaw_offset_initialized_ = true;
        }
    }
}

double HardwarePlannerRunner::score_candidate_pose(const Vec2& position,
                                                   double yaw,
                                                   const std::vector<RPLidarA1::ScanPoint>& scan) const {
    if (!is_inside_bounds(world_, position)) {
        return std::numeric_limits<double>::infinity();
    }

    const Vec2 origin = lidar_origin_world(position, yaw, config_.localization);
    if (!is_inside_bounds(world_, origin)) {
        return std::numeric_limits<double>::infinity();
    }

    double score = 0.0;
    int used = 0;
    const int downsample = std::max(config_.localization.scan_downsample, 1);

    for (size_t i = 0; i < scan.size(); i += static_cast<size_t>(downsample)) {
        const RPLidarA1::ScanPoint& point = scan[i];
        if (point.distance_m <= 0.0 ||
            point.distance_m < config_.localization.min_valid_range_m ||
            point.distance_m > config_.localization.max_range_m) {
            continue;
        }

        const double beam_heading = wrap_angle(
            yaw + config_.localization.lidar_yaw_offset + deg_to_rad(point.angle_deg));
        const std::vector<LidarHit> expected = world_.raycast(
            origin,
            beam_heading,
            1,
            0.0,
            config_.localization.max_range_m);

        const double predicted_range = expected.empty()
                                           ? config_.localization.max_range_m
                                           : expected.front().distance;
        const double range_error = std::min(
            std::abs(predicted_range - point.distance_m),
            config_.localization.max_range_m);
        const double weight = 1.0 / (0.25 + point.distance_m);
        score += range_error * weight;
        ++used;
    }

    if (used < 6) {
        return std::numeric_limits<double>::infinity();
    }

    return score / static_cast<double>(used);
}

void HardwarePlannerRunner::update_lidar_hits_world(const std::vector<RPLidarA1::ScanPoint>& scan) {
    lidar_hits_.clear();
    lidar_hits_.reserve(scan.size());

    const Vec2 origin = lidar_origin_world(estimate_.position, estimate_.yaw, config_.localization);
    for (const RPLidarA1::ScanPoint& point : scan) {
        if (point.distance_m <= 0.0 || point.distance_m < config_.localization.min_valid_range_m) {
            continue;
        }
        const double angle_world = wrap_angle(
            estimate_.yaw + config_.localization.lidar_yaw_offset + deg_to_rad(point.angle_deg));
        const Vec2 hit{
            origin.x + std::cos(angle_world) * point.distance_m,
            origin.y + std::sin(angle_world) * point.distance_m,
        };
        lidar_hits_.push_back({angle_world, point.distance_m, hit, true});
    }

    estimate_.min_lidar_distance = compute_min_lidar_distance(scan);
    estimate_.front_lidar_distance = compute_front_lidar_distance(scan);
}

void HardwarePlannerRunner::update_planner_references(double dt) {
    planner_accel_ref_ = clamp_value(
        estimate_.accel + last_j_ * dt,
        -geometry_.max_decel,
        geometry_.max_accel);
    planner_speed_ref_ = clamp_value(
        estimate_.speed + planner_accel_ref_ * dt,
        0.0,
        config_.cruise_speed_limit);

    if (world_.environment_mode() == EnvironmentMode::StructuredRoad) {
        const double slowdown_radius = std::max(config_.goal_slowdown_radius_m, 0.25);
        const double alpha = clamp_value(distance_to_goal_ / slowdown_radius, 0.0, 1.0);
        double goal_speed_cap = 0.06 + alpha * (config_.cruise_speed_limit - 0.06);
        if (distance_to_goal_ < std::max(config_.goal_tolerance_m * 2.0, 0.35)) {
            goal_speed_cap = std::min(goal_speed_cap, 0.10);
        }
        planner_speed_ref_ = std::min(planner_speed_ref_, goal_speed_cap);
        return;
    }

    const int non_final_gate_count = std::max(static_cast<int>(world_.gates().size()) - 1, 0);
    if (count_passed_gates() >= non_final_gate_count && non_final_gate_count > 0) {
        const double slowdown_radius = std::max(config_.goal_slowdown_radius_m, 0.25);
        const double alpha = clamp_value(distance_to_goal_ / slowdown_radius, 0.0, 1.0);
        double goal_speed_cap = 0.06 + alpha * (config_.cruise_speed_limit - 0.06);
        if (distance_to_goal_ < std::max(config_.goal_tolerance_m * 2.0, 0.35)) {
            goal_speed_cap = std::min(goal_speed_cap, 0.10);
        }
        planner_speed_ref_ = std::min(planner_speed_ref_, goal_speed_cap);
    }
}

void HardwarePlannerRunner::update_selected_trajectory() {
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
    if (!project_curvilinear_state(cl_, estimate_.position, s_current, closed_structured_loop, &s_current, &lateral_offset)) {
        double x_on_path = 0.0;
        double y_on_path = 0.0;
        cl_.prev_road.eval(s_current, x_on_path, y_on_path);
        const double path_heading = wrap_angle(cl_.prev_road.theta(s_current));
        const double dx = estimate_.position.x - x_on_path;
        const double dy = estimate_.position.y - y_on_path;
        lateral_offset = -std::sin(path_heading) * dx + std::cos(path_heading) * dy;
    }

    x0_.x = s_current;
    x0_.n = lateral_offset;
    x0_.v = estimate_.speed;
    x0_.a = estimate_.accel;
    x0_.c = estimate_.curvature;

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
        const double max_span = std::max(4.0, std::min(lookahead_distance, 0.75 * cl_.end_point_s));
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

void HardwarePlannerRunner::compute_control_command(double dt) {
    last_command_ = {};
    safety_stop_active_ = false;
    tracker_cross_track_error_ = 0.0;
    tracker_heading_error_deg_ = 0.0;

    last_command_.target_speed = planner_speed_ref_;
    last_command_.target_curvature = reference_trajectory_.empty() ? estimate_.curvature : reference_trajectory_.front().curvature;

    const VehicleModelState tracking_state = build_tracking_state(
        estimate_.position,
        estimate_.yaw,
        estimate_.speed,
        estimate_.accel,
        estimate_.curvature,
        estimate_.yaw_rate,
        steer_from_curvature(geometry_, estimate_.curvature));

    if (reference_trajectory_.size() >= 2) {
        const MpcCommand mpc_command = mpc_follower_.solve(
            geometry_,
            tracking_state,
            reference_trajectory_,
            planner_speed_ref_,
            0);
        if (mpc_command.valid) {
            last_command_.target_speed = clamp_value(
                mpc_command.target_speed,
                0.0,
                std::min(config_.cruise_speed_limit, geometry_.max_linear_speed));
            last_command_.target_curvature = curvature_from_steer(geometry_, mpc_command.target_steer_angle);
            tracker_cross_track_error_ = std::abs(mpc_command.cross_track_error);
            tracker_heading_error_deg_ =
                std::abs(mpc_command.heading_error) * 180.0 / kPi;
            last_mpc_command_ = mpc_command;
        } else {
            last_mpc_command_.reset();
        }
    } else {
        last_mpc_command_.reset();
    }

    if (estimate_.front_lidar_distance > 0.0 &&
        estimate_.front_lidar_distance < config_.localization.obstacle_stop_distance_m) {
        safety_stop_active_ = true;
        last_command_.safety_stop = true;
        last_command_.target_speed = 0.0;
    }

    last_command_.target_speed = clamp_value(
        last_command_.target_speed,
        0.0,
        geometry_.max_linear_speed);
    last_command_.target_curvature = clamp_value(
        last_command_.target_curvature,
        -geometry_.max_curvature,
        geometry_.max_curvature);
    last_command_.target_yaw_rate = clamp_value(
        last_command_.target_speed * last_command_.target_curvature,
        -config_.drive.max_yaw_rate,
        config_.drive.max_yaw_rate);

    const double half_track = config_.drive.track_width * 0.5;
    const double left_wheel_speed = last_command_.target_speed - last_command_.target_yaw_rate * half_track;
    const double right_wheel_speed = last_command_.target_speed + last_command_.target_yaw_rate * half_track;

    const int ff_left = wheel_speed_to_pwm(left_wheel_speed, config_.pwm.left_scale);
    const int ff_right = wheel_speed_to_pwm(right_wheel_speed, config_.pwm.right_scale);

    const int fb_linear = static_cast<int>(std::lround(
        config_.pwm.linear_feedback_gain * (last_command_.target_speed - estimate_.speed)));
    const int fb_yaw = static_cast<int>(std::lround(
        config_.pwm.yaw_feedback_gain * (last_command_.target_yaw_rate - estimate_.yaw_rate)));

    last_command_.pwm_left = static_cast<int>(clamp_value(
        static_cast<double>(ff_left + fb_linear - fb_yaw),
        -config_.pwm.max_pwm,
        config_.pwm.max_pwm));
    last_command_.pwm_right = static_cast<int>(clamp_value(
        static_cast<double>(ff_right + fb_linear + fb_yaw),
        -config_.pwm.max_pwm,
        config_.pwm.max_pwm));
}

void HardwarePlannerRunner::push_history() {
    history_.push_back({
        sim_time_,
        estimate_.speed,
        estimate_.accel,
        estimate_.yaw_rate,
        last_j_,
        last_r_,
        last_command_.target_speed,
        last_command_.target_yaw_rate,
        estimate_.curvature,
        distance_to_goal_,
        estimate_.min_lidar_distance,
        estimate_.front_lidar_distance,
        planner_speed_ref_,
        tracker_cross_track_error_,
        tracker_heading_error_deg_,
        planning_compute_ms_,
        tracking_compute_ms_,
        lidar_compute_ms_,
        estimator_compute_ms_,
        step_compute_ms_,
        static_cast<double>(visible_gate_indices_.size()),
        last_command_.pwm_left,
        last_command_.pwm_right,
    });

    if (static_cast<int>(history_.size()) > config_.max_history) {
        history_.erase(history_.begin());
    }

    trail_.push_back(estimate_.position);
    if (static_cast<int>(trail_.size()) > config_.max_history) {
        trail_.erase(trail_.begin());
    }
}

double HardwarePlannerRunner::compute_min_lidar_distance(const std::vector<RPLidarA1::ScanPoint>& scan) const {
    double min_range = config_.localization.max_range_m;
    for (const RPLidarA1::ScanPoint& point : scan) {
        if (point.distance_m >= config_.localization.min_valid_range_m &&
            point.distance_m <= config_.localization.max_range_m) {
            min_range = std::min(min_range, point.distance_m);
        }
    }
    return min_range;
}

double HardwarePlannerRunner::compute_front_lidar_distance(const std::vector<RPLidarA1::ScanPoint>& scan) const {
    double min_range = config_.localization.max_range_m;
    for (const RPLidarA1::ScanPoint& point : scan) {
        if (point.distance_m <= 0.0 ||
            point.distance_m < config_.localization.min_valid_range_m ||
            point.distance_m > config_.localization.max_range_m) {
            continue;
        }
        const double local_angle = wrap_angle(config_.localization.lidar_yaw_offset + deg_to_rad(point.angle_deg));
        if (std::abs(local_angle) <= config_.localization.front_sector_half_angle_rad) {
            min_range = std::min(min_range, point.distance_m);
        }
    }
    return min_range;
}

int HardwarePlannerRunner::wheel_speed_to_pwm(double wheel_speed_mps, double scale) const {
    if (std::abs(wheel_speed_mps) < 1e-4) {
        return 0;
    }

    const double sign = wheel_speed_mps >= 0.0 ? 1.0 : -1.0;
    double pwm = config_.pwm.wheel_speed_to_pwm_bias +
                 std::abs(wheel_speed_mps) * config_.pwm.wheel_speed_to_pwm_gain;
    pwm *= scale;
    pwm = clamp_value(
        pwm,
        static_cast<double>(config_.pwm.min_effective_pwm),
        static_cast<double>(config_.pwm.max_pwm));
    return static_cast<int>(std::lround(sign * pwm));
}

void HardwarePlannerRunner::step() {
    if (!connected_) {
        throw ProtocolResponseError("HardwarePlannerRunner not connected");
    }

    bridge_.pump(std::min(0.05, config_.nominal_dt * 0.5), config_.localization.min_scan_points, true);
    telemetry_ready_ = bridge_.observation().have_controller_telemetry;
    if (!telemetry_ready_) {
        return;
    }

    const RealRobotObservation& observation = bridge_.observation();
    const double now = observation.host_timestamp_s;
    const double dt = last_observation_time_ > 0.0
                          ? clamp_value(now - last_observation_time_, 0.02, 0.25)
                          : config_.nominal_dt;
    last_observation_time_ = now;
    step_with_observation(observation, dt, true);
}

void HardwarePlannerRunner::step_with_observation(const RealRobotObservation& observation,
                                                  double dt,
                                                  bool send_pwm) {
    const double bounded_dt = clamp_value(dt, 0.01, 0.25);
    if (observation.have_controller_telemetry) {
        telemetry_ready_ = true;
    }

    const auto step_start = std::chrono::steady_clock::now();

    world_.update_gate_layout(sim_time_);
    sync_gate_specs_from_world(false);

    const auto estimator_start = std::chrono::steady_clock::now();
    update_estimate_from_observation(observation, bounded_dt);
    if (observation.have_lidar_scan) {
        correct_pose_with_lidar(observation.lidar_scan);
        update_lidar_hits_world(observation.lidar_scan);
        lidar_compute_ms_ = 0.0;
    } else {
        estimate_.min_lidar_distance = config_.localization.max_range_m;
        estimate_.front_lidar_distance = config_.localization.max_range_m;
        lidar_hits_.clear();
        lidar_compute_ms_ = 0.0;
    }
    estimator_compute_ms_ = elapsed_ms(estimator_start, std::chrono::steady_clock::now());

    sync_planner_from_estimate(false);
    refresh_gate_diagnostics();

    const auto planning_start = std::chrono::steady_clock::now();
    plan_if_needed();
    update_planner_references(bounded_dt);
    update_selected_trajectory();
    planning_compute_ms_ = elapsed_ms(planning_start, std::chrono::steady_clock::now());

    const auto tracking_start = std::chrono::steady_clock::now();
    compute_control_command(bounded_dt);
    tracking_compute_ms_ = elapsed_ms(tracking_start, std::chrono::steady_clock::now());

    sim_time_ += bounded_dt;
    ++step_count_;
    if (structured_road_is_closed_loop(world_)) {
        const double goal_position_distance =
            distance(estimate_.position, structured_goal_ready_ ? structured_goal_position_ : world_.start());
        distance_to_goal_ = structured_goal_ready_
                                ? std::max(structured_goal_progress_target_ - structured_progress_s_, 0.0)
                                : cl_.end_point_s;
        const double progress_margin = std::max(0.8, 0.03 * std::max(cl_.end_point_s, 1.0));
        goal_reached_ = structured_goal_ready_ &&
                        structured_progress_s_ + progress_margin >= structured_goal_progress_target_ &&
                        goal_position_distance < 0.75;
    } else {
        distance_to_goal_ = distance(estimate_.position, world_.goal());
        goal_reached_ = distance_to_goal_ < config_.goal_tolerance_m &&
                        std::abs(estimate_.speed) < config_.goal_stop_speed_mps;
    }
    step_compute_ms_ = elapsed_ms(step_start, std::chrono::steady_clock::now());
    push_history();

    if (send_pwm && connected_ && bridge_.controller_connected()) {
        bridge_.send_pwm(
            static_cast<std::int16_t>(last_command_.pwm_left),
            static_cast<std::int16_t>(last_command_.pwm_right),
            false,
            MotorControlMode::DirectPwm);
    }
}

HardwarePlannerReport HardwarePlannerRunner::run(int max_steps) {
    const int limit = max_steps > 0 ? max_steps : 2000;
    while (step_count_ < limit && !goal_reached_) {
        step();
    }

    return current_report();
}

HardwarePlannerReport HardwarePlannerRunner::current_report() const {
    const RealRobotObservation& observation = bridge_.observation();
    const bool controller_front_alert =
        observation.have_controller_telemetry &&
        ((observation.controller.safety_flags &
          static_cast<std::uint16_t>(SafetyFlag::FrontAlert)) != 0U);
    const bool lidar_front_blocked =
        estimate_.front_lidar_distance > 0.0 &&
        estimate_.front_lidar_distance < config_.localization.obstacle_stop_distance_m;

    const std::uint16_t controller_safety_flags = observation.have_controller_telemetry
                                                      ? observation.controller.safety_flags
                                                      : static_cast<std::uint16_t>(0);
    const std::uint16_t controller_motor_flags = observation.have_controller_telemetry
                                                     ? observation.controller.motor_flags
                                                     : static_cast<std::uint16_t>(0);
    const std::uint16_t controller_status_flags = observation.have_controller_telemetry
                                                      ? observation.controller.status_flags
                                                      : static_cast<std::uint16_t>(0);
    const std::uint16_t controller_error_code = observation.have_controller_telemetry
                                                    ? observation.controller.error_code
                                                    : static_cast<std::uint16_t>(0);
    const std::int16_t controller_pwm_left = observation.have_controller_telemetry
                                                 ? observation.controller.pwm_l
                                                 : static_cast<std::int16_t>(0);
    const std::int16_t controller_pwm_right = observation.have_controller_telemetry
                                                  ? observation.controller.pwm_r
                                                  : static_cast<std::int16_t>(0);
    const std::int16_t controller_target_pwm_left = observation.have_controller_telemetry
                                                        ? observation.controller.target_pwm_l
                                                        : static_cast<std::int16_t>(0);
    const std::int16_t controller_target_pwm_right = observation.have_controller_telemetry
                                                         ? observation.controller.target_pwm_r
                                                         : static_cast<std::int16_t>(0);

    return {
        goal_reached_,
        telemetry_ready_,
        safety_stop_active_,
        controller_front_alert,
        lidar_front_blocked,
        observation.have_lidar_scan,
        step_count_,
        sim_time_,
        estimate_.position,
        distance_to_goal_,
        estimate_.min_lidar_distance,
        estimate_.front_lidar_distance,
        count_passed_gates(),
        controller_safety_flags,
        controller_motor_flags,
        controller_status_flags,
        controller_error_code,
        controller_pwm_left,
        controller_pwm_right,
        controller_target_pwm_left,
        controller_target_pwm_right,
        last_command_.pwm_left,
        last_command_.pwm_right,
    };
}

int HardwarePlannerRunner::count_passed_gates() const {
    int total = 0;
    for (const gate& g : gates_) {
        if (g.passed) {
            ++total;
        }
    }
    return total;
}

}  // namespace thesis_sim
