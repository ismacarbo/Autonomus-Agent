#include "hardware_planner_runner.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <thread>
#include <utility>

namespace thesis_sim {

namespace {

constexpr double kPi = 3.14159265358979323846;

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

double deg_to_rad(double angle_deg) {
    return angle_deg * kPi / 180.0;
}

double monotonic_seconds() {
    using clock = std::chrono::steady_clock;
    return std::chrono::duration<double>(clock::now().time_since_epoch()).count();
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

}  // namespace

HardwarePlannerRunner::HardwarePlannerRunner(WorldMap world, RealRobotBridge::Options bridge_options, HardwarePlannerConfig config)
    : world_(std::move(world)),
      config_(config),
      bridge_(std::move(bridge_options)),
      null_stream_("/dev/null") {
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
    chosen_gate_index_ = -1;
    goal_reached_ = false;
    safety_stop_active_ = false;
    history_.clear();
    trail_.clear();
    visible_gate_indices_.clear();
    lidar_hits_.clear();
    last_command_ = {};
    last_observation_time_ = 0.0;

    initialize_planner_state();
    reset_pose(world_.start(), world_.start_heading());
}

void HardwarePlannerRunner::reset_pose(const Vec2& position, double heading) {
    estimate_ = {};
    estimate_.position = position;
    estimate_.yaw = heading;
    estimate_.min_lidar_distance = config_.localization.max_range_m;
    estimate_.front_lidar_distance = config_.localization.max_range_m;
    estimate_.localized = true;

    virtual_speed_ref_ = 0.0;
    virtual_accel_ref_ = 0.0;
    virtual_curvature_ref_ = 0.0;
    yaw_offset_initialized_ = false;

    distance_to_goal_ = distance(position, world_.goal());
    sync_planner_from_estimate(true);
    initialize_gates();
}

void HardwarePlannerRunner::initialize_planner_state() {
    sim_ = {};
    sim_.W = 3.0;
    sim_.T_max = 20.0;
    sim_.la = 8.0;
    sim_.la_stop = 18.0;
    sim_.z_coord = 0.1;
    sim_.veh_W = config_.drive.body_width;
    sim_.veh_L = config_.drive.body_length;
    sim_.end_sim = 200.0;
    sim_.tol_obst = 0.25;
    sim_.lat_tol = 0.2;
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
    constexpr double kLowCurvature = 1e-4;
    constexpr double kMinCruise = 0.35;

    if (std::abs(cl_.kappa) < kLowCurvature) {
        sim_.V_max = config_.cruise_speed_limit;
        return;
    }

    const double curvature_limit = 1.5 * std::pow(std::abs(cl_.kappa), -1.0 / 3.0);
    sim_.V_max = clamp_value(curvature_limit, kMinCruise, config_.cruise_speed_limit);
}

std::vector<int> HardwarePlannerRunner::select_gate_candidates() const {
    std::vector<int> candidates;

    int nearest_idx = -1;
    double nearest_dist = std::numeric_limits<double>::infinity();

    for (size_t i = 0; i < gates_.size(); ++i) {
        if (gates_[i].passed) {
            continue;
        }

        const Vec2 gate_pos{gates_[i].x_pos, gates_[i].y_pos};
        const double dist = distance(estimate_.position, gate_pos);
        if (dist < nearest_dist) {
            nearest_dist = dist;
            nearest_idx = static_cast<int>(i);
        }

        const bool visible = dist <= config_.localization.max_range_m + 2.0 &&
                             world_.line_of_sight(estimate_.position, gate_pos);
        const bool close_final = gates_[i].final && dist <= 10.0;
        if (visible || close_final) {
            candidates.push_back(static_cast<int>(i));
        }
    }

    if (candidates.empty() && nearest_idx >= 0) {
        candidates.push_back(nearest_idx);
    }

    if (distance_to_goal_ < 7.0) {
        const int final_idx = static_cast<int>(gates_.size()) - 1;
        if (std::find(candidates.begin(), candidates.end(), final_idx) == candidates.end()) {
            candidates.push_back(final_idx);
        }
    }

    return candidates;
}

void HardwarePlannerRunner::sync_gate_selection(const std::vector<int>& candidate_indices,
                                                const std::vector<gate>& local_gates,
                                                int chosen_local_index) {
    chosen_gate_index_ = -1;
    for (size_t i = 0; i < candidate_indices.size(); ++i) {
        gate& target = gates_[candidate_indices[i]];
        target = local_gates[i];
        if (static_cast<int>(i) == chosen_local_index) {
            chosen_gate_index_ = candidate_indices[i];
        }
    }

    for (size_t i = 0; i < gates_.size(); ++i) {
        if (std::find(candidate_indices.begin(), candidate_indices.end(), static_cast<int>(i)) == candidate_indices.end()) {
            gates_[i].choose = false;
        }
    }
}

void HardwarePlannerRunner::plan_if_needed() {
    if (step_count_ % std::max(config_.control_interval_steps, 1) != 0) {
        return;
    }

    visible_gate_indices_ = select_gate_candidates();
    if (visible_gate_indices_.empty()) {
        last_j_ = 0.0;
        last_r_ = 0.0;
        chosen_gate_index_ = -1;
        return;
    }

    std::vector<gate> local_gates;
    local_gates.reserve(visible_gate_indices_.size());
    for (int idx : visible_gate_indices_) {
        local_gates.push_back(gates_[static_cast<size_t>(idx)]);
    }

    std::vector<double> commands = sel_jr(
        false,
        step_count_,
        false,
        nullptr,
        true,
        &local_gates,
        sim_,
        x0_,
        g_x0_,
        cl_,
        null_stream_,
        null_stream_,
        null_stream_);

    double next_j = commands.size() > 0 ? commands[0] : 0.0;
    double next_r = commands.size() > 1 ? commands[1] : 0.0;
    int chosen_local = commands.size() > 2 ? static_cast<int>(commands[2]) : -1;

    if (!std::isfinite(next_j) || !std::isfinite(next_r)) {
        next_j = 0.0;
        next_r = 0.0;
        chosen_local = -1;
    }

    last_j_ = clamp_value(next_j, -3.5, 2.5);
    last_r_ = clamp_value(next_r, -1.2, 1.2);
    sync_gate_selection(visible_gate_indices_, local_gates, chosen_local);
}

void HardwarePlannerRunner::update_estimate_from_observation(double dt) {
    const RealRobotObservation& observation = bridge_.observation();
    if (!observation.have_controller_telemetry) {
        return;
    }

    const ControllerTelemetry& telemetry = observation.controller;
    const double imu_yaw = static_cast<double>(telemetry.yaw_mrad) / 1000.0;
    if (!yaw_offset_initialized_) {
        yaw_offset_ = estimate_.yaw - imu_yaw;
        yaw_offset_initialized_ = true;
    }

    estimate_.yaw = wrap_angle(imu_yaw + yaw_offset_);
    estimate_.yaw_rate = static_cast<double>(telemetry.yaw_rate_mrad_s) / 1000.0;

    const double prev_speed = estimate_.speed;
    const double pwm_avg = 0.5 * static_cast<double>(telemetry.pwm_l + telemetry.pwm_r);
    const double speed_model = pwm_to_speed(static_cast<int>(std::lround(pwm_avg)));
    estimate_.speed = prev_speed + config_.pwm.speed_filter_alpha * (speed_model - prev_speed);

    const double raw_accel = dt > 1e-5 ? (estimate_.speed - prev_speed) / dt : 0.0;
    estimate_.accel = estimate_.accel + config_.pwm.accel_filter_alpha * (raw_accel - estimate_.accel);
    estimate_.curvature = std::abs(estimate_.speed) > 0.05
                              ? clamp_value(estimate_.yaw_rate / estimate_.speed,
                                            -config_.drive.max_curvature,
                                            config_.drive.max_curvature)
                              : 0.0;

    estimate_.position.x += estimate_.speed * std::cos(estimate_.yaw) * dt;
    estimate_.position.y += estimate_.speed * std::sin(estimate_.yaw) * dt;
}

void HardwarePlannerRunner::correct_pose_with_lidar(const std::vector<RPLidarA1::ScanPoint>& scan) {
    if (static_cast<int>(scan.size()) < config_.localization.min_scan_points) {
        return;
    }

    const double base_score = score_candidate_pose(estimate_.position, estimate_.yaw, scan);
    Vec2 best_position = estimate_.position;
    double best_yaw = estimate_.yaw;
    double best_score = base_score;

    for (double dx = -config_.localization.xy_search_window_m;
         dx <= config_.localization.xy_search_window_m + 1e-9;
         dx += config_.localization.xy_search_step_m) {
        for (double dy = -config_.localization.xy_search_window_m;
             dy <= config_.localization.xy_search_window_m + 1e-9;
             dy += config_.localization.xy_search_step_m) {
            for (double dyaw = -config_.localization.yaw_search_window_rad;
                 dyaw <= config_.localization.yaw_search_window_rad + 1e-9;
                 dyaw += config_.localization.yaw_search_step_rad) {
                const Vec2 candidate{estimate_.position.x + dx, estimate_.position.y + dy};
                const double candidate_yaw = wrap_angle(estimate_.yaw + dyaw);
                const double score = score_candidate_pose(candidate, candidate_yaw, scan);
                if (score < best_score) {
                    best_score = score;
                    best_position = candidate;
                    best_yaw = candidate_yaw;
                }
            }
        }
    }

    if (std::isfinite(best_score)) {
        estimate_.position = best_position;
        estimate_.yaw = best_yaw;
        estimate_.localized = true;
        if (bridge_.observation().have_controller_telemetry) {
            const double imu_yaw = static_cast<double>(bridge_.observation().controller.yaw_mrad) / 1000.0;
            yaw_offset_ = wrap_angle(estimate_.yaw - imu_yaw);
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
        if (point.distance_m <= 0.0 || point.distance_m > config_.localization.max_range_m) {
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
        const double range_error = std::min(std::abs(predicted_range - point.distance_m),
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
        if (point.distance_m <= 0.0) {
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

void HardwarePlannerRunner::update_virtual_reference(double dt) {
    virtual_accel_ref_ = clamp_value(virtual_accel_ref_ + last_j_ * dt, -2.0, 1.5);
    virtual_speed_ref_ = clamp_value(
        virtual_speed_ref_ + virtual_accel_ref_ * dt,
        0.0,
        std::min(config_.cruise_speed_limit, config_.drive.max_linear_speed));

    const double curvature_speed = std::max(virtual_speed_ref_, 0.05);
    virtual_curvature_ref_ = clamp_value(
        virtual_curvature_ref_ + curvature_speed * last_r_ * dt,
        -config_.drive.max_curvature,
        config_.drive.max_curvature);
}

void HardwarePlannerRunner::compute_control_command() {
    last_command_ = {};

    const bool controller_front_alert =
        bridge_.observation().have_controller_telemetry &&
        ((bridge_.observation().controller.safety_flags &
          static_cast<std::uint16_t>(SafetyFlag::FrontAlert)) != 0U);

    safety_stop_active_ =
        controller_front_alert ||
        (estimate_.front_lidar_distance > 0.0 &&
         estimate_.front_lidar_distance < config_.localization.obstacle_stop_distance_m);

    if (goal_reached_ && config_.stop_on_goal) {
        last_command_.safety_stop = true;
        return;
    }

    if (safety_stop_active_) {
        last_command_.safety_stop = true;
        return;
    }

    last_command_.target_speed = clamp_value(
        virtual_speed_ref_,
        0.0,
        config_.drive.max_linear_speed);
    last_command_.target_curvature = clamp_value(
        virtual_curvature_ref_,
        -config_.drive.max_curvature,
        config_.drive.max_curvature);
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
        if (point.distance_m > 0.0) {
            min_range = std::min(min_range, point.distance_m);
        }
    }
    return min_range;
}

double HardwarePlannerRunner::compute_front_lidar_distance(const std::vector<RPLidarA1::ScanPoint>& scan) const {
    double min_range = config_.localization.max_range_m;
    for (const RPLidarA1::ScanPoint& point : scan) {
        if (point.distance_m <= 0.0) {
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
    pwm = clamp_value(pwm,
                      static_cast<double>(config_.pwm.min_effective_pwm),
                      static_cast<double>(config_.pwm.max_pwm));
    return static_cast<int>(std::lround(sign * pwm));
}

double HardwarePlannerRunner::pwm_to_speed(int pwm) const {
    if (std::abs(pwm) < config_.pwm.min_effective_pwm) {
        return 0.0;
    }

    const double sign = pwm >= 0 ? 1.0 : -1.0;
    const double magnitude = std::max(0.0, static_cast<double>(std::abs(pwm) - config_.pwm.min_effective_pwm));
    return sign * magnitude * config_.pwm.speed_estimate_per_pwm;
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

    update_estimate_from_observation(dt);
    if (observation.have_lidar_scan) {
        correct_pose_with_lidar(observation.lidar_scan);
        update_lidar_hits_world(observation.lidar_scan);
    } else {
        estimate_.min_lidar_distance = config_.localization.max_range_m;
        estimate_.front_lidar_distance = config_.localization.max_range_m;
        lidar_hits_.clear();
    }

    sync_planner_from_estimate(false);
    update_speed_limit();
    plan_if_needed();
    update_virtual_reference(dt);

    sim_time_ += dt;
    ++step_count_;
    distance_to_goal_ = distance(estimate_.position, world_.goal());
    goal_reached_ = distance_to_goal_ < 1.75 && std::abs(estimate_.speed) < 0.15;
    compute_control_command();
    push_history();

    if (goal_reached_ && config_.stop_on_goal) {
        bridge_.send_pwm(0, 0, true);
        bridge_.stop(StopReason::UserRequest, false);
    } else if (last_command_.safety_stop) {
        bridge_.send_pwm(0, 0, true);
        bridge_.stop(StopReason::SafetyOverride, false);
    } else {
        bridge_.send_pwm(
            static_cast<std::int16_t>(last_command_.pwm_left),
            static_cast<std::int16_t>(last_command_.pwm_right));
    }
}

HardwarePlannerReport HardwarePlannerRunner::run(int max_steps) {
    const int limit = max_steps > 0 ? max_steps : 2000;
    while (step_count_ < limit && !goal_reached_) {
        step();
    }

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
