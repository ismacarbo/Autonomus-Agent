#include "support.inc"

namespace thesis_sim {

HardwarePlannerRunner::HardwarePlannerRunner(WorldMap world,
                                             RealRobotBridge::Options bridge_options,
                                             HardwarePlannerConfig config)
    : world_(std::move(world)),
      config_(config),
      geometry_(make_vehicle_geometry(config_)),
      bridge_(std::move(bridge_options)) {
    estimator_.set_geometry(geometry_);
    estimator_.set_tracked_vehicle(config_.vehicle_model == VehicleModelKind::TrackedVehicle);
    initialize_planner_state();
    reset();
}

void HardwarePlannerRunner::connect() {
    if (connected_) {
        return;
    }

    const auto reconnect_bridge = [&]() {
        bridge_.disconnect();
        connected_ = false;
        telemetry_ready_ = false;
        std::this_thread::sleep_for(std::chrono::milliseconds(350));
        bridge_.connect(lidar_enabled_for_current_mode());
        connected_ = true;
    };

    bridge_.connect(lidar_enabled_for_current_mode());
    connected_ = true;

    if (bridge_.controller_connected()) {
        // Prime the binary link first so the controller can start streaming after serial open.
        for (int i = 0; i < 4; ++i) {
            try {
                bridge_.poll_controller(0.12);
                break;
            } catch (const std::exception& e) {
                std::cerr << "hardware_runner_warning=controller bootstrap poll failed: "
                          << e.what() << '\n';
                if (i + 1 < 4) {
                    reconnect_bridge();
                }
            }
        }

        if (config_.auto_gyro_zero) {
            bool gyro_zero_ok = false;
            for (int attempt = 0; attempt < 3 && !gyro_zero_ok; ++attempt) {
                try {
                    bridge_.gyro_zero();
                    gyro_zero_ok = true;
                } catch (const std::exception& e) {
                    std::cerr << "hardware_runner_warning=GYRO_ZERO retry "
                              << (attempt + 1) << "/3 failed: " << e.what() << '\n';
                    if (attempt + 1 < 3) {
                        reconnect_bridge();
                        try {
                            bridge_.poll_controller(0.18);
                        } catch (const std::exception&) {
                        }
                    }
                }
            }
            if (!gyro_zero_ok) {
                std::cerr << "hardware_runner_warning=continuing without startup GYRO_ZERO; "
                             "the controller boot calibration will be used\n";
            }
        }
        if (config_.auto_set_autonomous_mode) {
            bool autonomous_mode_ok = false;
            for (int attempt = 0; attempt < 3 && !autonomous_mode_ok; ++attempt) {
                try {
                    bridge_.set_mode(ControllerMode::Autonomous);
                    autonomous_mode_ok = true;
                } catch (const std::exception& e) {
                    std::cerr << "hardware_runner_warning=AUTONOMOUS mode retry "
                              << (attempt + 1) << "/3 failed: " << e.what() << '\n';
                    if (attempt + 1 < 3) {
                        reconnect_bridge();
                        try {
                            bridge_.poll_controller(0.18);
                        } catch (const std::exception&) {
                        }
                    }
                }
            }
            if (!autonomous_mode_ok) {
                std::cerr << "hardware_runner_warning=controller AUTONOMOUS mode was not acked on connect; "
                             "continuing and relying on the current controller mode\n";
            }
        }

        const int desired_cmd_timeout_ms = desired_controller_cmd_timeout_ms(config_.nominal_dt);
        try {
            bridge_.config_set(
                static_cast<std::uint8_t>(ConfigParamId::CmdTimeoutMs),
                ConfigValueType::Uint16,
                desired_cmd_timeout_ms);
        } catch (const std::exception& e) {
            std::cerr << "hardware_runner_warning=controller CMD_TIMEOUT config failed: "
                      << e.what() << '\n';
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

    if (config_.use_mcu_velocity_closed_loop) {
        const ControllerTelemetry& telemetry = bridge_.observation().controller;
        const std::uint8_t required_minor =
            config_.vehicle_model == VehicleModelKind::TrackedVehicle ? 11U : 3U;
        if (telemetry.fw_major != 1U || telemetry.fw_minor < required_minor) {
            throw ProtocolResponseError(
                "--mcu-wheel-pid requires firmware >= 1." +
                std::to_string(required_minor) +
                "; detected " + std::to_string(telemetry.fw_major) + "." +
                std::to_string(telemetry.fw_minor));
        }
        bridge_.config_set(
            static_cast<std::uint8_t>(ConfigParamId::EncoderTicksPerRevolution),
            ConfigValueType::Uint16,
            config_.drive.encoder_ticks_per_revolution);
        bridge_.config_set(
            static_cast<std::uint8_t>(ConfigParamId::WheelRadiusMicrometers),
            ConfigValueType::Uint32,
            static_cast<std::int64_t>(std::llround(config_.drive.wheel_radius * 1.0e6)));
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
            bridge_.stop(StopReason::UserRequest, true, 0.25);
            std::this_thread::sleep_for(std::chrono::milliseconds(40));
            bridge_.send_pwm(0, 0, true);
        }
    } catch (const std::exception&) {
        try {
            if (bridge_.controller_connected()) {
                bridge_.send_pwm(0, 0, true);
                bridge_.stop(StopReason::UserRequest, false);
            }
        } catch (const std::exception&) {
        }
    }

    bridge_.disconnect();
    connected_ = false;
}

void HardwarePlannerRunner::apply_world(WorldMap world) {
    if (connected_ && bridge_.controller_connected()) {
        bridge_.send_pwm(0, 0, true);
    }

    world_ = std::move(world);
    reset();
}

void HardwarePlannerRunner::apply_config(HardwarePlannerConfig config) {
    if (connected_ && bridge_.controller_connected()) {
        bridge_.send_pwm(0, 0, true);
    }

    config_ = config;
    reset();
}

void HardwarePlannerRunner::reset() {
    geometry_ = make_vehicle_geometry(config_);
    const bool tiny_indoor_loop = tiny_indoor_structured_loop(world_);
    const bool compact_manual_unstructured_car =
        manual_unstructured_exploration(world_) &&
        config_.vehicle_model == VehicleModelKind::CarLikeBicycle;
    if (compact_manual_unstructured_car) {
        // The physical car is actuated by independent left/right motor groups.
        // A 1.8 1/m bicycle limit gives a 0.56 m minimum radius and cannot
        // follow the few-decimetre gate clothoids used in the 1.20 m arena.
        // Keep the MPC bicycle prediction, but expose the tighter forward-only
        // differential authority that the drivetrain can actually execute.
        geometry_.max_steer_angle = std::max(geometry_.max_steer_angle, 1.10);
        geometry_.max_curvature = std::max(geometry_.max_curvature, 6.0);
    } else if (micro_structured_world(world_)) {
        geometry_.max_steer_angle = std::max(geometry_.max_steer_angle, 1.35);
        geometry_.max_curvature = std::max(geometry_.max_curvature, 14.0);
    } else if (compact_structured_world(world_)) {
        geometry_.max_steer_angle = std::max(geometry_.max_steer_angle, 1.10);
        geometry_.max_curvature = std::max(geometry_.max_curvature, 5.50);
    }
    estimator_.set_geometry(geometry_);
    estimator_.set_tracked_vehicle(config_.vehicle_model == VehicleModelKind::TrackedVehicle);
    MpcFollowerConfig mpc_config{};
    if (compact_manual_unstructured_car) {
        // Dynamic gate clothoids are only a few decimetres long. The generic
        // 0.6/1.4 m MPC lookahead jumps directly to their endpoint and turns
        // path following back into point pursuit in the 1.20 m editor arena.
        mpc_config.horizon_steps = 8;
        mpc_config.horizon_dt = 0.08;
        mpc_config.preview_distance = 0.10;
        mpc_config.min_lookahead_distance = 0.055;
        mpc_config.preview_min_index = 1;
        mpc_config.min_accel = -1.4;
        mpc_config.max_accel = 1.0;
        mpc_config.max_steer_rate = 3.20;
        mpc_config.w_cross_track = 32.0;
        mpc_config.w_heading = 22.0;
        mpc_config.w_speed = 0.65;
        mpc_config.w_steer = 0.30;
        mpc_config.w_accel = 0.14;
        mpc_config.w_steer_rate = 0.06;
        mpc_config.w_terminal = 12.0;
    } else if (tiny_indoor_loop) {
        mpc_config.horizon_steps = 5;
        mpc_config.horizon_dt = 0.06;
        mpc_config.preview_distance = 0.045;
        mpc_config.min_lookahead_distance = 0.035;
        mpc_config.preview_min_index = 1;
        mpc_config.min_accel = -1.4;
        mpc_config.max_accel = 1.0;
        mpc_config.max_steer_rate = 3.40;
        mpc_config.w_cross_track = 40.0;
        mpc_config.w_heading = 28.0;
        mpc_config.w_speed = 0.50;
        mpc_config.w_steer = 0.28;
        mpc_config.w_accel = 0.14;
        mpc_config.w_steer_rate = 0.07;
        mpc_config.w_terminal = 14.0;
    } else if (micro_structured_world(world_)) {
        mpc_config.horizon_steps = 6;
        mpc_config.horizon_dt = 0.07;
        mpc_config.preview_distance = 0.05;
        mpc_config.min_lookahead_distance = 0.04;
        mpc_config.max_steer_rate = 4.50;
        mpc_config.w_cross_track = 30.0;
        mpc_config.w_heading = 22.0;
        mpc_config.w_speed = 1.0;
        mpc_config.w_steer_rate = 0.050;
        mpc_config.w_terminal = 10.0;
    } else if (compact_structured_world(world_)) {
        mpc_config.horizon_steps = 8;
        mpc_config.horizon_dt = 0.08;
        mpc_config.preview_distance = 0.12;
        mpc_config.min_lookahead_distance = 0.08;
        mpc_config.max_steer_rate = 3.20;
        mpc_config.w_cross_track = 28.0;
        mpc_config.w_heading = 18.0;
        mpc_config.w_speed = 0.9;
        mpc_config.w_steer_rate = 0.040;
        mpc_config.w_terminal = 10.0;
    }
    mpc_follower_ = ModelPredictivePathFollower(mpc_config);

    step_count_ = 0;
    sim_time_ = 0.0;
    last_j_ = 0.0;
    last_r_ = 0.0;
    planner_speed_ref_ = 0.0;
    planner_accel_ref_ = 0.0;
    planner_yaw_rate_ref_ = 0.0;
    planner_yaw_accel_ref_ = 0.0;
    planner_initial_yaw_rate_ = 0.0;
    last_planner_command_step_ = 0;
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
    clear_locked_gap_goal();
    startup_scan_elapsed_s_ = 0.0;
    unstructured_no_candidate_scan_elapsed_s_ = 0.0;
    startup_scan_direction_ = 1.0;
    structured_goal_ready_ = false;
    startup_scan_complete_ = false;
    chosen_gate_index_ = -1;
    goal_reached_ = false;
    passed_unstructured_gap_count_ = 0;
    safety_stop_active_ = false;
    yaw_offset_ = 0.0;
    last_raw_imu_yaw_ = 0.0;
    last_accepted_imu_yaw_ = 0.0;
    last_observation_time_ = 0.0;
    commanded_speed_ = 0.0;
    commanded_steer_angle_ = 0.0;
    measured_left_wheel_speed_ = 0.0;
    measured_right_wheel_speed_ = 0.0;
    wheel_speed_error_integral_left_ = 0.0;
    wheel_speed_error_integral_right_ = 0.0;
    last_left_encoder_ticks_ = 0;
    last_right_encoder_ticks_ = 0;
    latest_controller_left_encoder_ticks_ = 0;
    latest_controller_right_encoder_ticks_ = 0;
    latest_controller_left_encoder_delta_ = 0;
    latest_controller_right_encoder_delta_ = 0;
    latest_controller_encoder_dt_ms_ = 0.0;
    yaw_offset_initialized_ = false;
    have_raw_imu_yaw_ = false;
    have_accepted_imu_yaw_ = false;
    last_accepted_imu_ms_ = 0;
    encoder_ticks_initialized_ = false;
    encoder_ready_streak_ = 0;
    measured_wheel_speeds_valid_ = false;
    no_motion_command_cycles_ = 0;
    left_wheel_stall_cycles_ = 0;
    right_wheel_stall_cycles_ = 0;
    left_wheel_breakaway_hold_cycles_ = 0;
    right_wheel_breakaway_hold_cycles_ = 0;
    stuck_motion_elapsed_s_ = 0.0;
    stuck_recovery_until_s_ = -1.0;
    stuck_recovery_direction_ = 1.0;
    lidar_reverse_reposition_until_s_ = -1.0;
    lidar_post_reverse_search_until_s_ = -1.0;
    last_controller_rearm_time_s_ = -1.0;
    active_lidar_scan_duration_s_ = 0.0;
    slam_pose_valid_ = false;
    last_slam_correction_accepted_ = false;
    last_slam_position_ = {};
    last_slam_yaw_ = 0.0;
    last_slam_position_innovation_m_ = 0.0;
    last_slam_yaw_innovation_rad_ = 0.0;
    use_dynamic_gap_gates_ = dynamic_gap_mode_enabled();
    gap_recovery_turn_active_ = false;
    stall_boost_active_ = false;
    encoder_slip_guard_active_ = false;
    history_.clear();
    trail_.clear();
    planned_trajectory_.clear();
    reference_trajectory_.clear();
    visible_gate_indices_.clear();
    passed_unstructured_gap_positions_.clear();
    dynamic_gap_tracks_.clear();
    next_dynamic_gap_track_id_ = 1;
    lidar_hits_.clear();
    lidar_map_points_.clear();
    exploration_cells_.clear();
    global_free_points_.clear();
    global_occupied_points_.clear();
    exploration_path_.clear();
    visited_frontiers_.clear();
    selected_frontier_.reset();
    last_frontier_plan_step_ = -1;
    exploration_map_source_ = ExplorationMapSource::LocalPersistentGrid;
    navigation_target_kind_ = NavigationTargetKind::None;
    exploration_state_ = UnstructuredExplorationState::Disabled;
    control_source_ = HardwareControlSource::Hold;
    slam_toolbox_snapshot_ = {};
    slam_snapshot_received_sim_time_s_ = -1.0;
    external_pose_reference_ = {};
    external_pose_received_sim_time_s_ = -1.0;
    lidar_occupancy_cells_.clear();
    lidar_map_keys_.clear();
    gate_specs_.clear();
    diagnostics_ = {};
    diagnostics_.dynamic_gap_gates = use_dynamic_gap_gates_;
    diagnostics_.slam_enabled = config_.slam_toolbox_enabled;
    last_command_ = {};
    last_mpc_command_.reset();
    structured_anchor_hint_index_ = 0;

    initialize_planner_state();
    sync_road_from_world();
    estimator_.set_geometry(geometry_);
    estimator_.set_tracked_vehicle(config_.vehicle_model == VehicleModelKind::TrackedVehicle);
    estimator_.reset(world_.start(), world_.start_heading());
    reset_pose(world_.start(), world_.start_heading());
    reset_start_matching();
}

void HardwarePlannerRunner::set_external_pose_reference(
    const ExternalPoseReference& reference,
    double timeout_s) {
    external_pose_reference_ = reference;
    external_pose_reference_.valid =
        reference.valid &&
        std::isfinite(reference.source_timestamp_s) &&
        std::isfinite(reference.position.x) &&
        std::isfinite(reference.position.y) &&
        std::isfinite(reference.yaw) &&
        std::isfinite(reference.quality) &&
        reference.quality >= 0.0;
    external_pose_reference_.quality =
        std::clamp(reference.quality, 0.0, 1.0);
    external_pose_reference_.yaw = wrap_angle(reference.yaw);
    external_pose_received_sim_time_s_ = sim_time_;
    external_pose_timeout_s_ = std::max(timeout_s, 0.01);
}

bool HardwarePlannerRunner::external_pose_reference_fresh() const {
    return external_pose_reference_.valid &&
           external_pose_received_sim_time_s_ >= 0.0 &&
           sim_time_ - external_pose_received_sim_time_s_ <= external_pose_timeout_s_;
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
    commanded_speed_ = 0.0;
    commanded_steer_angle_ = 0.0;
    clear_locked_gap_goal();
    startup_scan_elapsed_s_ = 0.0;
    unstructured_no_candidate_scan_elapsed_s_ = 0.0;
    startup_scan_direction_ = 1.0;
    startup_scan_complete_ = false;
    measured_left_wheel_speed_ = 0.0;
    measured_right_wheel_speed_ = 0.0;
    wheel_speed_error_integral_left_ = 0.0;
    wheel_speed_error_integral_right_ = 0.0;
    encoder_ready_streak_ = 0;
    measured_wheel_speeds_valid_ = false;
    no_motion_command_cycles_ = 0;
    left_wheel_stall_cycles_ = 0;
    right_wheel_stall_cycles_ = 0;
    left_wheel_breakaway_hold_cycles_ = 0;
    right_wheel_breakaway_hold_cycles_ = 0;
    lidar_reverse_reposition_until_s_ = -1.0;
    lidar_post_reverse_search_until_s_ = -1.0;
    stall_boost_active_ = false;
    encoder_slip_guard_active_ = false;
    tracker_cross_track_error_ = 0.0;
    tracker_heading_error_deg_ = 0.0;
    diagnostics_.chosen_gate_distance = std::numeric_limits<double>::infinity();
    diagnostics_.planner_has_reference = false;
    estimator_.reset(position, heading);
    sync_planner_from_estimate(true);
    if (dynamic_gap_mode_enabled()) {
        gates_.clear();
        gate_specs_.clear();
        dynamic_gap_tracks_.clear();
        next_dynamic_gap_track_id_ = 1;
    } else {
        sync_gate_specs_from_world(true);
    }
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
    const double world_span = world_has_structured_reference(world_)
                                  ? structured_course_span_m(world_)
                                  : std::max(world_width, world_height);
    const bool compact_world = world_span <= 5.0;
    const bool compact_structured_world =
        compact_world && world_has_structured_reference(world_);
    const bool mixed_world = world_is_mixed(world_);

    const bool micro_structured_world =
        !mixed_world && compact_structured_world && world_span <= 0.35;
    const bool tiny_indoor_loop =
        micro_structured_world &&
        structured_road_is_closed_loop(world_) &&
        structured_road_length_m(world_) <= 0.95;
    const bool compact_mixed_loop =
        mixed_world &&
        compact_structured_world &&
        structured_road_is_closed_loop(world_);
    const bool compact_mixed_open =
        mixed_world &&
        compact_structured_world &&
        compact_mixed_open_road(world_);
    const bool compact_mixed_road =
        compact_mixed_loop || compact_mixed_open;
    const bool hardware_mixed_lab =
        compact_hardware_mixed_lab_world(world_);
    const double compact_mixed_road_length =
        compact_mixed_road ? structured_road_length_m(world_) : 0.0;

    sim_ = {};
    sim_.W = tiny_indoor_loop ? clamp_value(world_span * 0.36, 0.09, 0.13)
             : micro_structured_world ? clamp_value(world_span * 0.42, 0.10, 0.16)
             : hardware_mixed_lab ? clamp_value(0.38 * world_span_m(world_), 0.16, 0.22)
             : compact_mixed_road ? clamp_value(geometry_.body_width + 0.06, 0.28, 0.34)
             : (compact_structured_world ? clamp_value(geometry_.body_width + 0.08, 0.28, 0.38)
                                         : (compact_world ? 0.90 : 3.0));
    sim_.T_max = tiny_indoor_loop ? 2.6 : (micro_structured_world ? 4.0 : (hardware_mixed_lab ? 3.4 : (compact_mixed_road ? 4.5 : (compact_world ? 8.0 : 20.0))));
    sim_.la = tiny_indoor_loop ? 0.16 : (micro_structured_world ? 0.35 : (hardware_mixed_lab ? 0.22 : (compact_mixed_road ? 0.38 : (compact_world ? 1.20 : 8.0))));
    sim_.la_stop = tiny_indoor_loop ? 0.26 : (micro_structured_world ? 0.70 : (hardware_mixed_lab ? 0.44 : (compact_mixed_road ? 0.80 : (compact_world ? 2.40 : 18.0))));
    sim_.z_coord = 0.1;
    sim_.veh_W = geometry_.body_width;
    sim_.veh_L = geometry_.body_length;
    sim_.end_sim = tiny_indoor_loop ? std::max(world_span * 4.0, 1.4)
                   : micro_structured_world ? std::max(world_span * 8.0, 3.0)
                   : compact_mixed_road ? std::max(compact_mixed_road_length, 0.50)
                   : (compact_world ? std::max(world_span * 4.0, 6.0) : 200.0);
    sim_.tol_obst = tiny_indoor_loop ? 0.04 : (micro_structured_world ? 0.06 : (hardware_mixed_lab ? 0.075 : (compact_mixed_road ? 0.12 : (compact_world ? 0.18 : 0.25))));
    sim_.lat_tol = tiny_indoor_loop ? 0.025 : (micro_structured_world ? 0.04 : (hardware_mixed_lab ? 0.045 : (compact_mixed_road ? 0.08 : (compact_world ? 0.12 : 0.2))));
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
    gate_specs_ = world_.gates();
    for (const GateSpec& spec : gate_specs_) {
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
    if (!world_has_structured_reference(world_) || world_.road_centerline().size() < 2) {
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
    if (!world_has_dynamic_gap_layer(world_)) {
        gates_.clear();
        gate_specs_.clear();
        return;
    }
    if (dynamic_gap_mode_enabled()) {
        if (reset_flags) {
            gates_.clear();
            gate_specs_.clear();
        }
        return;
    }
    if (reset_flags || gates_.size() != specs.size()) {
        initialize_gates();
        return;
    }

    gate_specs_ = specs;
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
    g_x0_.omega = estimate_.yaw_rate;
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
    if (!world_has_dynamic_gap_layer(world_) || gates_.empty()) {
        return;
    }

    if (use_dynamic_gap_gates_) {
        for (gate& candidate : gates_) {
            candidate.too_far = false;
            if (candidate.passed) {
                candidate.choose = false;
            }
        }
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
    if (!world_has_dynamic_gap_layer(world_)) {
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
    if (locked_gap_goal_.has_value() &&
        chosen_gate_index_ < 0 &&
        !gates_.empty()) {
        size_t locked_index = 0;
        double best_distance_sq = std::numeric_limits<double>::infinity();
        for (size_t i = 0; i < gates_.size(); ++i) {
            if (gates_[i].passed || gates_[i].too_far) {
                continue;
            }
            const Vec2 gate_position{gates_[i].x_pos, gates_[i].y_pos};
            const double candidate_distance_sq = distance_sq(gate_position, *locked_gap_goal_);
            if (candidate_distance_sq < best_distance_sq) {
                best_distance_sq = candidate_distance_sq;
                locked_index = i;
            }
        }
        gates_[locked_index].choose = true;
        gates_[locked_index].too_far = false;
        chosen_gate_index_ = static_cast<int>(locked_index);
        if (std::find(visible_gate_indices_.begin(),
                      visible_gate_indices_.end(),
                      chosen_gate_index_) == visible_gate_indices_.end()) {
            visible_gate_indices_.push_back(chosen_gate_index_);
        }
    }
}

int HardwarePlannerRunner::planning_interval_steps() const {
    return std::max(config_.control_interval_steps, 1);
}

int HardwarePlannerRunner::required_unstructured_gap_pass_count() const {
    // A perception-only compact hardware scene has no authoritative number of
    // physical LiDAR apertures. Keep exploring until the operator stops the
    // run instead of turning an arbitrary local count into a false mission end.
    if (manual_unstructured_exploration(world_)) {
        return std::numeric_limits<int>::max();
    }
    return std::max(config_.gap_extraction.min_passed_gates_to_complete, 1);
}

bool HardwarePlannerRunner::dynamic_gap_mode_enabled() const {
    return world_has_dynamic_gap_layer(world_) &&
           config_.gap_extraction.enabled;
}

bool HardwarePlannerRunner::strict_locked_gate_motion_enabled() const {
    return dynamic_gap_mode_enabled() &&
           world_.environment_mode() == EnvironmentMode::UnstructuredGates &&
           config_.gap_extraction.strict_locked_gate_motion;
}

double HardwarePlannerRunner::mixed_road_gate_lookahead_m() const {
    if (!world_is_mixed(world_)) {
        return 1.35;
    }

    const double span = world_span_m(world_);
    const double course_span = structured_course_span_m(world_);
    if (span <= 1.25) {
        return clamp_value(0.46 * course_span, 0.20, 0.34);
    }
    if (span <= 2.50) {
        return clamp_value(0.45 * course_span, 0.36, 0.70);
    }
    return 1.35;
}

double HardwarePlannerRunner::compute_mixed_road_forward_clearance(double lookahead_m) const {
    if (!world_is_mixed(world_) ||
        world_.road_centerline().size() < 2 ||
        !(lookahead_m > 1e-3)) {
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
        const Vec2 segment{road[i + 1].x - road[i].x, road[i + 1].y - road[i].y};
        const double segment_len_sq = dot_product(segment, segment);
        if (!(segment_len_sq > 1e-12)) {
            continue;
        }
        const Vec2 from_start{estimate_.position.x - road[i].x, estimate_.position.y - road[i].y};
        const double t = clamp_value(dot_product(from_start, segment) / segment_len_sq, 0.0, 1.0);
        const Vec2 projection{
            road[i].x + segment.x * t,
            road[i].y + segment.y * t,
        };
        const double d_sq = distance_sq(estimate_.position, projection);
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

    const double corridor_padding = 0.5 * geometry_.body_width + 0.08;
    const double corridor_padding_sq = corridor_padding * corridor_padding;
    const double current_world_span = world_span_m(world_);
    const double sample_step = 0.06;
    const bool closed_loop = structured_road_is_closed_loop(world_);
    const bool compact_closed_mixed_loop =
        world_is_mixed(world_) &&
        closed_loop &&
        current_world_span <= 2.50;
    for (double ahead = sample_step; ahead <= lookahead_m + 1e-9; ahead += sample_step) {
        double s_probe = best_s + ahead;
        if (closed_loop) {
            s_probe = wrap_arc_length(s_probe, road_length);
        } else if (s_probe > road_length) {
            break;
        }
        const Vec2 probe = point_at_s(s_probe);
        if (!is_inside_bounds(world_, probe, 0.0)) {
            return ahead;
        }
        for (const Rect& obstacle : world_.obstacles()) {
            if (compact_closed_mixed_loop) {
                const Vec2 obstacle_center{
                    0.5 * (obstacle.min_x + obstacle.max_x),
                    0.5 * (obstacle.min_y + obstacle.max_y),
                };
                if (distance_sq(probe, obstacle_center) > 0.24 * 0.24) {
                    continue;
                }
            }
            if (point_in_expanded_rect(probe, obstacle, corridor_padding)) {
                return ahead;
            }
        }
        for (const Vec2& point : lidar_map_points_) {
            if (distance_sq(probe, point) <= corridor_padding_sq) {
                return ahead;
            }
        }
        for (const LidarHit& hit : lidar_hits_) {
            if (hit.hit && distance_sq(probe, hit.point) <= corridor_padding_sq) {
                return ahead;
            }
        }
    }

    return lookahead_m;
}

double HardwarePlannerRunner::compute_mixed_road_block_score(double lookahead_m) const {
    if (!world_is_mixed(world_)) {
        return 0.0;
    }
    const double clearance = compute_mixed_road_forward_clearance(lookahead_m);
    if (clearance >= lookahead_m - 1e-3) {
        return 0.0;
    }

    const double span = world_span_m(world_);
    if (span <= 1.25) {
        const double enter_distance = clamp_value(0.28 * span, 0.09, 0.18);
        const double release_band = clamp_value(0.26 * span, 0.09, 0.18);
        return 1.0 - clamp_value((clearance - enter_distance) / release_band, 0.0, 1.0);
    }
    if (span <= 2.50) {
        return 1.0 - clamp_value((clearance - 0.20) / 0.24, 0.0, 1.0);
    }
    return 1.0 - clamp_value((clearance - 0.18) / 0.70, 0.0, 1.0);
}

bool HardwarePlannerRunner::perception_map_ready() const {
    return static_cast<int>(lidar_map_points_.size()) >= std::max(config_.localization.min_scan_points / 2, 24);
}

bool HardwarePlannerRunner::unstructured_perception_only_mode() const {
    return world_.environment_mode() == EnvironmentMode::UnstructuredGates;
}

bool HardwarePlannerRunner::dynamic_gap_point_allowed(const Vec2& position) const {
    if (!dynamic_gap_mode_enabled()) {
        return is_inside_bounds(world_, position);
    }
    if (world_is_mixed(world_) &&
        world_has_structured_reference(world_) &&
        world_span_m(world_) <= 2.50 &&
        !world_.road_centerline().empty()) {
        if (!is_inside_bounds(world_, position, 0.0)) {
            return false;
        }
        for (const Rect& obstacle : world_.obstacles()) {
            if (point_in_expanded_rect(position, obstacle, 0.025)) {
                return false;
            }
        }
        const bool micro_hardware_lab =
            compact_hardware_mixed_lab_world(world_);
        const double corridor_half_width = micro_hardware_lab
            ? clamp_value(0.24 * world_span_m(world_), 0.12, 0.18)
            : clamp_value(
                  std::max(geometry_.body_width + 0.10, 0.30 * structured_course_span_m(world_)),
                  0.26,
                  0.38);
        return distance_to_polyline_m(world_.road_centerline(), position) <= corridor_half_width;
    }
    return is_inside_bounds(
        world_,
        position,
        std::max(config_.gap_extraction.dynamic_bounds_margin_m, 0.0));
}

Vec2 HardwarePlannerRunner::clamp_dynamic_gap_point(const Vec2& position) const {
    if (!dynamic_gap_mode_enabled()) {
        return clamp_point_to_bounds(world_, position);
    }
    return clamp_point_to_bounds(
        world_,
        position,
        -std::max(config_.gap_extraction.dynamic_bounds_margin_m, 0.0));
}

bool HardwarePlannerRunner::lidar_enabled_for_current_mode() const {
    return config_.mapping_lidar_enabled ||
           world_.environment_mode() != EnvironmentMode::StructuredRoad;
}

bool HardwarePlannerRunner::lidar_pose_correction_enabled_for_current_mode() const {
    if (!lidar_enabled_for_current_mode() ||
        world_.environment_mode() == EnvironmentMode::StructuredRoad) {
        return false;
    }
    switch (config_.localization_policy) {
        case HardwareLocalizationPolicy::EncoderImuPrimary:
            return false;
        case HardwareLocalizationPolicy::LidarPoseCorrection:
            return true;
        case HardwareLocalizationPolicy::Auto:
        default:
            // The legacy local matcher aligns a scan against points accumulated
            // in the same drifting odometry frame. It has neither a pose graph
            // nor loop closure, so enabling it implicitly can create confident
            // map-scale jumps in symmetric gate/loop environments. In Auto,
            // LiDAR remains a perception source and global pose corrections are
            // accepted only from the SLAM Toolbox bridge. Keep this matcher as
            // an explicit diagnostic option through `--pose-fusion lidar`.
            return false;
    }
}

}  // namespace thesis_sim
