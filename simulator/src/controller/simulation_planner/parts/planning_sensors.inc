void PlannerDrivenVehicleSim::plan_if_needed() {
    if (step_count_ % planning_interval_steps() != 0) {
        return;
    }

    sim_.V_max = planner_speed_limit(config_.cruise_speed_limit, cl_.kappa);
    const double planner_r_limit =
        config_.vehicle_model == VehicleModelKind::TrackedVehicle
            ? tracked_planner_yaw_accel_limit(geometry_)
            : 1.4;
    auto commit_planner_command = [&](double next_j, double next_r) {
        last_j_ = clamp_value(next_j, -3.5, 2.5);
        last_r_ = clamp_value(next_r, -planner_r_limit, planner_r_limit);
        planner_initial_yaw_rate_ = navigation_yaw_rate_;
        last_planner_command_step_ = step_count_;
    };

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
            planner_traj_);

        double next_j = commands.size() > 0 ? commands[0] : 0.0;
        double next_r = commands.size() > 1 ? commands[1] : 0.0;
        if (!is_finite_pair(next_j, next_r)) {
            next_j = 0.0;
            next_r = 0.0;
        }

        visible_gate_indices_.clear();
        chosen_gate_index_ = -1;
        commit_planner_command(next_j, next_r);
        update_selected_trajectory();
        return;
    }

    if (mixed_mode_enabled() && road_ != nullptr && mixed_gate_mode_active_) {
        const bool mixed_closed_sequence_world = mixed_closed_gate_sequence(world_);
        const bool mixed_semantic_sequence_world = mixed_closed_semantic_gate_sequence(world_);
        std::vector<int> active_indices = active_gate_indices();
        if (mixed_semantic_sequence_world && active_indices.size() > 1) {
            active_indices.resize(1);
        }
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
            planner_traj_);

        double next_j = commands.size() > 0 ? commands[0] : 0.0;
        double next_r = commands.size() > 1 ? commands[1] : 0.0;
        int chosen_gate = commands.size() > 2 ? static_cast<int>(commands[2]) : -1;

        if (!is_finite_pair(next_j, next_r)) {
            next_j = 0.0;
            next_r = 0.0;
            chosen_gate = -1;
        }

        for (size_t i = 0; i < active_indices.size(); ++i) {
            gate& global_gate = gates_[static_cast<size_t>(active_indices[i])];
            if (mixed_semantic_sequence_world) {
                global_gate.choose = active_gates[i].choose;
                global_gate.too_far = false;
            } else {
                global_gate = active_gates[i];
            }
        }

        commit_planner_command(next_j, next_r);

        const bool compact_mixed_open_world =
            world_span_m(world_) <= 1.20 &&
            mixed_mode_enabled() &&
            !structured_road_is_closed_loop(world_);
        if (chosen_gate == 100 || chosen_gate < 0) {
            if ((compact_mixed_open_world || mixed_closed_sequence_world) && !active_indices.empty()) {
                refresh_gate_diagnostics();
                chosen_gate_index_ = active_indices.front();
                mixed_gate_mode_active_ = true;
                last_j_ = std::max(last_j_, 0.0);
                last_r_ = 0.0;
                planner_initial_yaw_rate_ = navigation_yaw_rate_;
                last_planner_command_step_ = step_count_;
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
        planner_initial_yaw_rate_ = navigation_yaw_rate_;
        last_planner_command_step_ = step_count_;
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
        planner_traj_);

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

    commit_planner_command(next_j, next_r);
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

    const VehicleModelState& state = vehicle_model_->state();
    lidar_hits_ = mvc::model::acquire_simulated_lidar_scan(
        &world_,
        state.position,
        state.yaw,
        sim_time_,
        structured_progress_s_,
        make_lidar_model_config(config_),
        &simulation_rng_);
    min_lidar_distance_ = compute_min_lidar();
    lidar_scan_fresh_ = true;
}

void PlannerDrivenVehicleSim::update_slam_map() {
    if (!config_.lidar_enabled || !lidar_scan_fresh_ || lidar_hits_.empty()) {
        return;
    }

    const VehicleModelState& physical_state = vehicle_model_->state();
    mvc::model::accumulate_slam_endpoints(
        lidar_hits_,
        physical_state.yaw,
        navigation_position_,
        navigation_yaw_,
        world_.bounds(),
        &slam_points_,
        &slam_voxels_);
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
    return mvc::model::minimum_lidar_distance(lidar_hits_, active_lidar_range());
}

double PlannerDrivenVehicleSim::score_lidar_pose_candidate(const Vec2& position, double yaw) const {
    if (!config_.lidar_enabled || lidar_hits_.empty()) {
        return std::numeric_limits<double>::infinity();
    }
    if (!point_in_bounds(world_.bounds(), position)) {
        return std::numeric_limits<double>::infinity();
    }

    const RangeSensorSpec sensor = mvc::model::range_sensor_spec(make_lidar_model_config(config_));
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
        const std::vector<LidarHit> expected = world_.raycast(position, beam_heading, 1, 0.0, sensor.range_m);
        if (expected.empty()) {
            continue;
        }

        const LidarHit& measured = lidar_hits_[static_cast<size_t>(i)];
        const LidarHit& predicted = expected.front();
        const double measured_range = clamp_value(measured.distance, 0.0, sensor.range_m);
        if (measured_range <= 0.0) {
            continue;
        }

        const double predicted_range = clamp_value(predicted.distance, 0.0, sensor.range_m);
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
    return mvc::model::range_sensor_spec(make_lidar_model_config(config_)).beams;
}

double PlannerDrivenVehicleSim::active_lidar_fov_rad() const {
    return mvc::model::range_sensor_spec(make_lidar_model_config(config_)).fov_rad;
}

double PlannerDrivenVehicleSim::active_lidar_range() const {
    return mvc::model::range_sensor_spec(make_lidar_model_config(config_)).range_m;
}

void PlannerDrivenVehicleSim::set_sensor_suite(bool imu_enabled,
                                               bool lidar_enabled,
                                               RangeSensorProfile profile) {
    config_.ideal_conditions = false;
    config_.calibrated_conditions = false;
    config_.imu_enabled = imu_enabled;
    config_.lidar_enabled = lidar_enabled;
    config_.range_sensor_profile = profile;
    update_lidar();
    update_navigation_state(config_.dt);
    update_slam_map();
    sync_planner_from_vehicle(false);
    update_selected_trajectory();
}

void PlannerDrivenVehicleSim::set_lidar_range(double range_m) {
    config_.lidar_range = std::max(range_m, 0.10);
    update_lidar();
    update_navigation_state(config_.dt);
    sync_planner_from_vehicle(false);
    update_selected_trajectory();
}

void PlannerDrivenVehicleSim::set_ideal_conditions(bool enabled) {
    constexpr double kPi = 3.14159265358979323846;
    config_.ideal_conditions = enabled;
    config_.calibrated_conditions = false;
    if (enabled) {
        config_.imu_enabled = true;
        config_.lidar_enabled = true;
        config_.range_sensor_profile = RangeSensorProfile::IdealLidar2D;
        config_.lidar_beams = 360;
        config_.lidar_fov_rad = 2.0 * kPi;
        config_.lidar_range = 12.0;
        config_.control_interval_steps = 1;
    } else {
        const SimConfig defaults{};
        config_.imu_enabled = defaults.imu_enabled;
        config_.lidar_enabled = defaults.lidar_enabled;
        config_.range_sensor_profile = defaults.range_sensor_profile;
        config_.lidar_beams = defaults.lidar_beams;
        config_.lidar_fov_rad = defaults.lidar_fov_rad;
        config_.lidar_range = defaults.lidar_range;
        config_.control_interval_steps = defaults.control_interval_steps;
    }
    reset();
}

void PlannerDrivenVehicleSim::set_calibrated_conditions(bool enabled) {
    config_.calibrated_conditions = enabled;
    if (enabled) {
        config_.ideal_conditions = false;
        config_.imu_enabled = true;
        config_.lidar_enabled = true;
        config_.range_sensor_profile = RangeSensorProfile::RplidarA1;
        config_.lidar_beams = 360;
        config_.lidar_fov_rad = 2.0 * 3.14159265358979323846;
        config_.lidar_range = std::max(config_.lidar_range, 1.20);
        config_.control_interval_steps = std::max(config_.control_interval_steps, 2);
    } else {
        const SimConfig defaults{};
        config_.imu_enabled = defaults.imu_enabled;
        config_.lidar_enabled = defaults.lidar_enabled;
        config_.range_sensor_profile = defaults.range_sensor_profile;
        config_.lidar_beams = defaults.lidar_beams;
        config_.lidar_fov_rad = defaults.lidar_fov_rad;
        config_.lidar_range = defaults.lidar_range;
        config_.control_interval_steps = defaults.control_interval_steps;
    }
    reset();
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

