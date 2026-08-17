#include "support.inc"

namespace thesis_sim {

const char* PlannerDrivenVehicleSim::localization_mode_name() const {
    if (config_.calibrated_conditions && gate_environment(world_)) {
        return "Encoders + IMU (LiDAR perception only)";
    }
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
      config_(config) {
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
            (world_.structured_preset() == StructuredMapPreset::ValidationRoad ||
             world_.structured_preset() == StructuredMapPreset::IdealCircle);
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
    if (config_.calibrated_conditions) {
        geometry_.left_pwm_scale *= config_.actuator_left_scale;
        geometry_.right_pwm_scale *= config_.actuator_right_scale;
        geometry_.motor_time_constant *= std::max(config_.actuator_time_constant_scale, 0.10);
        config_.control_interval_steps = std::max(config_.control_interval_steps, 2);
        config_.range_sensor_profile = RangeSensorProfile::RplidarA1;
    }
    rebuild_vehicle_model();
    MpcFollowerConfig mpc_config{};
    const bool normalized_structured_validation =
        world_.environment_mode() == EnvironmentMode::StructuredRoad &&
        world_span_m(world_) >= 0.90 &&
        world_span_m(world_) <= 1.25;
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
    } else if (normalized_structured_validation) {
        mpc_config.preview_distance = 0.24;
        mpc_config.min_lookahead_distance = 0.14;
        mpc_config.max_steer_rate = 2.80;
        mpc_config.w_cross_track = 32.0;
        mpc_config.w_heading = 20.0;
        mpc_config.w_steer_rate = 0.025;
    } else if (world_.environment_mode() == EnvironmentMode::UnstructuredGates &&
               world_span_m(world_) <= 5.0) {
        mpc_config.preview_distance = 0.24;
        mpc_config.min_lookahead_distance = 0.12;
        mpc_config.max_steer_rate = 3.20;
        mpc_config.w_cross_track = 32.0;
        mpc_config.w_heading = 18.0;
        mpc_config.w_steer_rate = 0.025;
    } else if (world_.environment_mode() == EnvironmentMode::MixedRoadGates) {
        mpc_config.preview_distance = 0.65;
        mpc_config.min_lookahead_distance = 0.32;
        mpc_config.max_steer_rate = 3.20;
        mpc_config.w_heading = 13.0;
        mpc_config.w_steer_rate = 0.030;
    }
    mpc_follower_ = ModelPredictivePathFollower(mpc_config);

    step_count_ = 0;
    sim_time_ = 0.0;
    simulation_rng_.seed(config_.simulation_seed);
    delayed_controls_.clear();
    simulated_imu_bias_rad_ = 0.0;
    last_j_ = 0.0;
    last_r_ = 0.0;
    planner_yaw_rate_ref_ = 0.0;
    planner_yaw_accel_ref_ = 0.0;
    planner_initial_yaw_rate_ = 0.0;
    last_planner_command_step_ = 0;
    chosen_gate_index_ = -1;
    dynamic_lidar_passed_gates_ = 0;
    dynamic_lidar_candidate_count_ = 0;
    dynamic_lidar_gate_hold_steps_ = 0;
    dynamic_lidar_stable_steps_ = 0;
    dynamic_lidar_active_gate_width_ = 0.0;
    dynamic_lidar_active_gate_score_ = 0.0;
    dynamic_lidar_active_gate_confidence_ = 0.0;
    active_dynamic_lidar_gate_track_id_ = -1;
    next_dynamic_lidar_gate_track_id_ = 1;
    dynamic_lidar_gate_target_switches_ = 0;
    dynamic_lidar_gate_tracks_.clear();
    dynamic_lidar_passed_gate_positions_.clear();
    mixed_gate_mode_active_ = false;
    mixed_gate_rejoin_cooldown_steps_ = 0;
    mixed_switch_count_ = 0;
    mixed_abort_count_ = 0;
    mixed_gate_score_ = 0.0;
    mixed_structured_score_ = 1.0;
    mixed_gate_confidence_ = 0.0;
    mixed_rejoin_active_ = false;
    mixed_rejoin_started_time_s_ = -1.0;
    mixed_rejoin_progress_start_s_ = 0.0;
    mixed_rejoin_road_lateral_error_ = std::numeric_limits<double>::infinity();
    mixed_rejoin_road_heading_error_deg_ = std::numeric_limits<double>::infinity();
    goal_reached_ = false;
    goal_stop_active_ = false;
    collision_ = false;
    collision_cause_.clear();
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
    structured_route_last_s_ = std::numeric_limits<double>::quiet_NaN();
    structured_goal_position_ = world_.start();
    structured_goal_ready_ = false;
    lidar_scan_fresh_ = false;
    last_mpc_command_.reset();

    history_.clear();
    trail_.clear();
    estimated_trail_.clear();
    planned_trajectory_.clear();
    slam_points_.clear();
    slam_voxels_.clear();
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
    g_x0_.omega = 0.0;
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
    world_.reset_perception_obstacles();
    sync_gate_specs_from_world(true);
    refresh_gate_diagnostics();
    sync_planner_from_vehicle(true);

    update_vehicle_snapshot();
    update_lidar();
    update_slam_map();
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
    g_x0_.omega = navigation_yaw_rate_;
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
    auto load_gate_specs = [&]() {
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
    };
    if (use_dynamic_lidar_gates()) {
        if (reset_flags) {
            gates_.clear();
            chosen_gate_index_ = -1;
            dynamic_lidar_gate_hold_steps_ = 0;
            dynamic_lidar_stable_steps_ = 0;
            dynamic_lidar_active_gate_width_ = 0.0;
            dynamic_lidar_active_gate_score_ = 0.0;
            dynamic_lidar_active_gate_confidence_ = 0.0;
            active_dynamic_lidar_gate_track_id_ = -1;
            dynamic_lidar_gate_tracks_.clear();
            dynamic_lidar_candidate_count_ = 0;
        }
        if (mixed_closed_semantic_gate_sequence(world_) &&
            (reset_flags || gates_.size() != specs.size())) {
            load_gate_specs();
        }
        return;
    }
    if (reset_flags || gates_.size() != specs.size()) {
        load_gate_specs();
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
    double left_dist = static_cast<double>(state.left_encoder_delta) * ticks_to_distance;
    double right_dist = static_cast<double>(state.right_encoder_delta) * ticks_to_distance;
    if (config_.calibrated_conditions) {
        std::normal_distribution<double> encoder_noise(
            0.0,
            std::max(config_.encoder_distance_noise_std_m, 0.0));
        left_dist = left_dist * config_.encoder_left_scale + encoder_noise(simulation_rng_);
        right_dist = right_dist * config_.encoder_right_scale + encoder_noise(simulation_rng_);
    }
    const double yaw_response_scale = clamp_value(geometry_.yaw_response_scale, 0.05, 2.0);
    const double odom_delta_yaw =
        (std::abs(geometry_.track) > 1e-6 ? (right_dist - left_dist) / geometry_.track : 0.0) *
        yaw_response_scale;
    const double odom_speed = dt > 1e-6 ? 0.5 * (left_dist + right_dist) / dt : 0.0;
    const double odom_yaw_rate = dt > 1e-6 ? odom_delta_yaw / dt : 0.0;
    estimator_.predict(dt, odom_speed, odom_yaw_rate);

    if (config_.imu_enabled) {
        double measured_yaw = state.yaw;
        double measured_yaw_rate = state.yaw_rate;
        if (config_.calibrated_conditions) {
            std::normal_distribution<double> yaw_noise(
                0.0,
                std::max(config_.imu_yaw_noise_std_rad, 0.0));
            std::normal_distribution<double> yaw_rate_noise(
                0.0,
                std::max(config_.imu_yaw_rate_noise_std_rad_s, 0.0));
            std::normal_distribution<double> bias_walk(
                0.0,
                std::max(config_.imu_yaw_bias_walk_std_rad, 0.0));
            simulated_imu_bias_rad_ += bias_walk(simulation_rng_);
            measured_yaw = wrap_angle(
                state.yaw + simulated_imu_bias_rad_ + yaw_noise(simulation_rng_));
            measured_yaw_rate = state.yaw_rate + yaw_rate_noise(simulation_rng_);
        } else if (!config_.ideal_conditions) {
            measured_yaw = wrap_angle(
                state.yaw +
                0.004 * std::sin(sim_time_ * 0.31 + 0.45) +
                0.002 * std::cos(sim_time_ * 0.57 + 0.12));
            measured_yaw_rate =
                state.yaw_rate +
                0.010 * std::sin(sim_time_ * 0.43 + 0.60) +
                0.006 * std::cos(sim_time_ * 0.71 + 0.20);
        }
        estimator_.update_imu(measured_yaw, measured_yaw_rate);
    }

    // The lightweight local matcher is intentionally limited to unstructured
    // maps. The closed mixed loop contains repeated/symmetric geometry that can
    // produce a locally better but globally wrong association; mixed hardware
    // correction is supplied by the gated SLAM Toolbox bridge instead.
    const bool lidar_pose_correction_enabled =
        world_.environment_mode() == EnvironmentMode::UnstructuredGates &&
        !config_.calibrated_conditions;
    if (config_.lidar_enabled &&
        lidar_pose_correction_enabled &&
        lidar_scan_fresh_ &&
        !lidar_hits_.empty()) {
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

    if (mixed_mode_enabled() && (mixed_gate_mode_active_ || mixed_rejoin_active_)) {
        const double usable_clearance =
            min_lidar_distance_ > 0.0 ? min_lidar_distance_ : active_lidar_range();
        const double clearance_scale = clamp_value(
            (usable_clearance - 0.50 * geometry_.body_width) /
                std::max(0.45, 2.5 * geometry_.body_width),
            0.25,
            1.0);
        const double curvature_scale = 1.0 / (
            1.0 + 0.45 * std::abs(navigation_curvature_));
        const double mode_cap =
            config_.vehicle_model == VehicleModelKind::TrackedVehicle
                ? (mixed_gate_mode_active_ ? 0.060 : 0.070)
                : (mixed_gate_mode_active_ ? 0.095 : 0.110);
        planner_speed_ref_ = std::min(
            planner_speed_ref_,
            mode_cap * clearance_scale * curvature_scale);
    }

    if (use_dynamic_lidar_gates() &&
        !mixed_mode_enabled() &&
        active_gate_indices().empty()) {
        planner_speed_ref_ = 0.0;
        planner_accel_ref_ = clamp_value(
            -navigation_speed_ / std::max(dt, 1e-3),
            -geometry_.max_decel,
            0.0);
    }

    if (config_.vehicle_model == VehicleModelKind::TrackedVehicle) {
        const double elapsed =
            std::max(1, step_count_ - last_planner_command_step_ + 1) *
            std::max(dt, 1e-3);
        const double t = primitive_sample_time(planner_traj_.lat_traj, elapsed);
        const double yaw_accel_limit = tracked_planner_yaw_accel_limit(geometry_);
        planner_yaw_accel_ref_ = clamp_value(
            primitive_control_at(planner_traj_.lat_traj, t, last_r_),
            -yaw_accel_limit,
            yaw_accel_limit);
        planner_yaw_rate_ref_ = clamp_value(
            planner_initial_yaw_rate_ +
                primitive_integral(planner_traj_.lat_traj, t, last_r_),
            -geometry_.max_yaw_rate,
            geometry_.max_yaw_rate);
    } else {
        planner_yaw_accel_ref_ = 0.0;
        planner_yaw_rate_ref_ = 0.0;
        planner_initial_yaw_rate_ = navigation_yaw_rate_;
    }
}

void PlannerDrivenVehicleSim::refresh_gate_diagnostics() {
    update_unstructured_gate_progress();
    update_gate_activation_window();
    visible_gate_indices_.clear();
    chosen_gate_index_ = -1;
    if (mixed_closed_gate_sequence(world_)) {
        for (gate& candidate : gates_) {
            candidate.choose = false;
        }
        for (size_t i = 0; i < gates_.size(); ++i) {
            if (gates_[i].passed || gates_[i].final) {
                continue;
            }
            if (!gates_[i].too_far) {
                visible_gate_indices_.push_back(static_cast<int>(i));
                if (mixed_gate_mode_active_) {
                    chosen_gate_index_ = static_cast<int>(i);
                    gates_[i].choose = true;
                }
            }
            break;
        }
        return;
    }
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

}  // namespace thesis_sim
