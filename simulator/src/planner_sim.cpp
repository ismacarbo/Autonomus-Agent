#include "planner_sim.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace thesis_sim {

namespace {

double clamp_value(double value, double lo, double hi) {
    return std::max(lo, std::min(value, hi));
}

bool is_finite_pair(double a, double b) {
    return std::isfinite(a) && std::isfinite(b);
}

}  // namespace

PlannerDrivenVehicleSim::PlannerDrivenVehicleSim(WorldMap world, SimConfig config)
    : world_(std::move(world)),
      config_(config),
      vehicle_model_(make_four_wheel_car_model(geometry_)),
      null_stream_("/dev/null") {
    reset();
}

void PlannerDrivenVehicleSim::reset() {
    step_count_ = 0;
    sim_time_ = 0.0;
    last_j_ = 0.0;
    last_r_ = 0.0;
    chosen_gate_index_ = -1;
    goal_reached_ = false;
    collision_ = false;
    distance_to_goal_ = distance(world_.start(), world_.goal());
    min_lidar_distance_ = config_.lidar_range;

    history_.clear();
    trail_.clear();
    visible_gate_indices_.clear();
    lidar_hits_.clear();

    sim_ = {};
    sim_.W = 3.0;
    sim_.T_max = 20.0;
    sim_.la = 8.0;
    sim_.la_stop = 18.0;
    sim_.z_coord = 0.1;
    sim_.veh_W = geometry_.body_width;
    sim_.veh_L = geometry_.body_length;
    sim_.end_sim = 200.0;
    sim_.tol_obst = 0.25;
    sim_.lat_tol = 0.2;
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
    sync_planner_from_vehicle(true);

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

    update_vehicle_snapshot();
    update_lidar();
    update_telemetry();
}

void PlannerDrivenVehicleSim::sync_planner_from_vehicle(bool reset_relative_state) {
    const VehicleModelState& state = vehicle_model_->state();
    g_x0_.x_fix = state.position.x;
    g_x0_.y_fix = state.position.y;
    g_x0_.theta = state.yaw;
    g_x0_.kappa_veh = state.curvature;
    g_x0_.v_fix = state.speed;
    g_x0_.a_fix = state.accel;

    if (reset_relative_state) {
        x0_.v = state.speed;
        x0_.a = state.accel;
        x0_.b = 0.0;
        x0_.c = state.curvature;
    }
}

void PlannerDrivenVehicleSim::update_speed_limit() {
    constexpr double kLowCurvature = 1e-4;
    constexpr double kMinCruise = 2.5;

    if (std::abs(cl_.kappa) < kLowCurvature) {
        sim_.V_max = config_.cruise_speed_limit;
        return;
    }

    const double curvature_limit = 1.5 * std::pow(std::abs(cl_.kappa), -1.0 / 3.0);
    sim_.V_max = clamp_value(curvature_limit, kMinCruise, config_.cruise_speed_limit);
}

std::vector<int> PlannerDrivenVehicleSim::select_gate_candidates() const {
    std::vector<int> candidates;

    const Vec2 pose{g_x0_.x_fix, g_x0_.y_fix};
    int nearest_idx = -1;
    double nearest_dist = std::numeric_limits<double>::infinity();

    for (size_t i = 0; i < gates_.size(); ++i) {
        if (gates_[i].passed) {
            continue;
        }

        const Vec2 gate_pos{gates_[i].x_pos, gates_[i].y_pos};
        const double dist = distance(pose, gate_pos);
        if (dist < nearest_dist) {
            nearest_dist = dist;
            nearest_idx = static_cast<int>(i);
        }

        const bool visible = dist <= config_.lidar_range + 2.0 && world_.line_of_sight(pose, gate_pos);
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

void PlannerDrivenVehicleSim::sync_gate_selection(const std::vector<int>& candidate_indices,
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

void PlannerDrivenVehicleSim::plan_if_needed() {
    if (step_count_ % config_.control_interval_steps != 0) {
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

    if (!is_finite_pair(next_j, next_r)) {
        next_j = 0.0;
        next_r = 0.0;
        chosen_local = -1;
    }

    last_j_ = clamp_value(next_j, -3.5, 2.5);
    last_r_ = clamp_value(next_r, -0.9, 0.9);

    sync_gate_selection(visible_gate_indices_, local_gates, chosen_local);
}

void PlannerDrivenVehicleSim::update_lidar() {
    const Vec2 origin{g_x0_.x_fix, g_x0_.y_fix};
    lidar_hits_ = world_.raycast(origin, g_x0_.theta, config_.lidar_beams, config_.lidar_fov_rad, config_.lidar_range);
    min_lidar_distance_ = compute_min_lidar();
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
    vehicle_.model_name = vehicle_model_->name();
    vehicle_.body_corners = make_box_corners(vehicle_.position, vehicle_.yaw, geometry_.body_length, geometry_.body_width);

    const std::array<Vec2, 4> wheel_local{{
        {geometry_.wheelbase * 0.5, geometry_.track * 0.5},
        {geometry_.wheelbase * 0.5, -geometry_.track * 0.5},
        {-geometry_.wheelbase * 0.5, geometry_.track * 0.5},
        {-geometry_.wheelbase * 0.5, -geometry_.track * 0.5},
    }};

    const double curvature = g_x0_.kappa_veh;
    const double radius = std::abs(curvature) > 1e-5 ? 1.0 / curvature : std::numeric_limits<double>::infinity();

    for (size_t i = 0; i < wheel_local.size(); ++i) {
        const Vec2 global_offset = rotate(wheel_local[i], vehicle_.yaw);
        vehicle_.wheels[i].center = {vehicle_.position.x + global_offset.x, vehicle_.position.y + global_offset.y};
        vehicle_.wheels[i].steering = i < 2;
        vehicle_.wheels[i].yaw = vehicle_.yaw + (i < 2 ? vehicle_.steer_angle : 0.0);

        if (std::isfinite(radius)) {
            const double lateral_sign = wheel_local[i].y >= 0.0 ? 1.0 : -1.0;
            const double wheel_radius = std::max(0.1, std::abs(radius - lateral_sign * geometry_.track * 0.5));
            vehicle_.wheels[i].speed = vehicle_.speed * wheel_radius / std::abs(radius);
        } else {
            vehicle_.wheels[i].speed = vehicle_.speed;
        }
    }
}

double PlannerDrivenVehicleSim::compute_min_lidar() const {
    if (lidar_hits_.empty()) {
        return config_.lidar_range;
    }
    double min_value = config_.lidar_range;
    for (const LidarHit& hit : lidar_hits_) {
        min_value = std::min(min_value, hit.distance);
    }
    return min_value;
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
        vehicle_.sideslip,
        distance_to_goal_,
        min_lidar_distance_,
    });
    if (static_cast<int>(history_.size()) > config_.max_history) {
        history_.erase(history_.begin());
    }

    trail_.push_back(vehicle_.position);
    if (static_cast<int>(trail_.size()) > config_.max_history) {
        trail_.erase(trail_.begin());
    }
}

void PlannerDrivenVehicleSim::step() {
    if (goal_reached_ || collision_) {
        return;
    }

    update_speed_limit();
    plan_if_needed();

    vehicle_model_->step(config_.dt, last_j_, last_r_);
    sync_planner_from_vehicle(false);

    sim_time_ += config_.dt;
    ++step_count_;

    update_vehicle_snapshot();
    update_lidar();
    collision_ = world_.collides(vehicle_.body_corners);
    distance_to_goal_ = distance(vehicle_.position, world_.goal());
    goal_reached_ = distance_to_goal_ < 1.75 && vehicle_.speed < 1.0;
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
