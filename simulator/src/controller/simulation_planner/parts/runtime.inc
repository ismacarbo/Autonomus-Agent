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

    // With the measured car calibration the plant can still be moving at
    // 0.2 m/s when it first enters the final acceptance region.  Requiring a
    // low speed for completion while continuing to chase an intermittently
    // visible LiDAR goal makes it overshoot and re-accelerate.  Once the
    // unstructured robot physically enters the goal region, latch a controlled
    // stop until it settles; intermediate gates never activate this state.
    if (world_.environment_mode() == EnvironmentMode::UnstructuredGates) {
        const double span = world_span_m(world_);
        const double final_acceptance = span <= 5.0
            ? std::clamp(0.05 * span, 0.08, 0.16)
            : 0.75;
        const bool was_goal_stop_active = goal_stop_active_;
        goal_stop_active_ = goal_stop_active_ ||
                            distance(vehicle_.position, world_.goal()) < final_acceptance;
        if (!was_goal_stop_active && goal_stop_active_ &&
            std::getenv("THESIS_TRACE_GOAL_STOP") != nullptr) {
            std::cerr << "goal_stop_latched t=" << sim_time_
                      << " position=(" << vehicle_.position.x << ',' << vehicle_.position.y << ')'
                      << " distance=" << distance(vehicle_.position, world_.goal())
                      << " speed=" << vehicle_.speed << '\n';
        }
    }

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
    const bool mixed_closed_loop_step =
        mixed_mode_enabled() &&
        (structured_road_is_closed_loop(world_) || mixed_closed_gate_sequence(world_));
    const bool obstacle_closed_mixed_step =
        mixed_closed_loop_step &&
        !mixed_closed_semantic_gate_sequence(world_);
    const bool pure_structured_closed_step =
        world_.environment_mode() == EnvironmentMode::StructuredRoad &&
        structured_road_is_closed_loop(world_);
    const bool has_selected_mixed_gate =
        mixed_gate_mode_active_ &&
        chosen_gate_index_ >= 0 &&
        chosen_gate_index_ < static_cast<int>(gates_.size());
    const bool compact_mixed_gate_step =
        (compact_mixed_open_step || compact_mixed_closed_loop(world_)) &&
        has_selected_mixed_gate;
    const bool mixed_gate_direct_step =
        has_selected_mixed_gate &&
        (compact_mixed_gate_step || mixed_closed_loop_step);
    const bool tracked_mixed_structured_rejoin =
        tracked_vehicle && mixed_mode_enabled() && !mixed_gate_mode_active_;
    double tracking_desired_speed = planner_speed_ref_;
    if (pure_structured_closed_step && reference_trajectory_.size() >= 2) {
        // A zero longitudinal primitive may be a transient planner result at a
        // high-curvature sample. Keep the path follower alive at a conservative
        // crawl speed; the MPC still owns steering, yaw and acceleration limits.
        const double structured_crawl_speed =
            tracked_vehicle ? 0.050 : 0.040;
        tracking_desired_speed = std::max(
            tracking_desired_speed,
            std::min(structured_crawl_speed, config_.cruise_speed_limit));
    }
    if ((world_.environment_mode() == EnvironmentMode::UnstructuredGates ||
         (mixed_mode_enabled() && mixed_gate_mode_active_)) &&
        !compact_unstructured_world(world_) &&
        reference_trajectory_.size() >= 2 &&
        distance_to_goal_ > 1.50) {
        tracking_desired_speed = std::max(
            tracking_desired_speed,
            std::min(0.30, config_.cruise_speed_limit));
    }
    if (world_.environment_mode() == EnvironmentMode::UnstructuredGates &&
        use_dynamic_lidar_gates() &&
        reference_trajectory_.size() >= 2) {
        int speed_gate_index = chosen_gate_index_;
        if (speed_gate_index < 0 ||
            speed_gate_index >= static_cast<int>(gates_.size())) {
            const std::vector<int> active_indices = active_gate_indices();
            speed_gate_index = active_indices.empty() ? -1 : active_indices.front();
        }
        if (speed_gate_index >= 0 &&
            speed_gate_index < static_cast<int>(gates_.size())) {
            const Vec2 target = gate_position(gates_[static_cast<size_t>(speed_gate_index)]);
            const double target_distance = distance(navigation_position_, target);
            const double heading_error = std::abs(wrap_angle(
                angle_to(navigation_position_, target) - navigation_yaw_));
            const double base_gate_cap = compact_unstructured_world(world_) ? 0.12 : 0.16;
            const double distance_scale = clamp_value(
                target_distance / std::max(4.0 * geometry_.body_length, 0.45),
                0.55,
                1.0);
            const double heading_scale = 1.0 / (1.0 + 1.30 * heading_error);
            const double clearance_scale = clamp_value(
                (min_lidar_distance_ - 0.5 * geometry_.body_width) /
                    std::max(3.0 * geometry_.body_width, 0.30),
                0.55,
                1.0);
            const bool tight_gate_turn =
                heading_error >= 0.50 ||
                (target_distance <= 0.45 && heading_error >= 0.30);
            if (tight_gate_turn) {
                const double safe_gate_speed = clamp_value(
                    base_gate_cap * distance_scale * heading_scale * clearance_scale,
                    0.055,
                    base_gate_cap);
                tracking_desired_speed = std::min(
                    tracking_desired_speed,
                    std::min(safe_gate_speed, config_.cruise_speed_limit));
            }
        }
    }
    if (compact_mixed_gate_step && reference_trajectory_.size() >= 2) {
        tracking_desired_speed = clamp_value(
            tracking_desired_speed,
            std::min(0.045, config_.cruise_speed_limit),
            std::min(0.070, config_.cruise_speed_limit));
    } else if (mixed_mode_enabled() && reference_trajectory_.size() >= 2) {
        tracking_desired_speed = std::max(
            tracking_desired_speed,
            std::min(tracked_mixed_structured_rejoin ? 0.085 : 0.12,
                     config_.cruise_speed_limit));
    }
    if (tracked_vehicle && reference_trajectory_.size() >= 2) {
        const double speed_floor =
            tracked_mixed_structured_rejoin
                ? std::min(geometry_.max_linear_speed, 0.085)
                : tracked_vehicle_speed_floor(world_, geometry_);
        tracking_desired_speed = std::max(
            tracking_desired_speed,
            speed_floor);
    }
    if (obstacle_closed_mixed_step && reference_trajectory_.size() >= 2) {
        const double obstacle_speed_cap =
            mixed_gate_mode_active_ ? 0.075 : 0.055;
        tracking_desired_speed = std::min(
            tracking_desired_speed,
            std::min(obstacle_speed_cap, config_.cruise_speed_limit));
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
                0,
                config_.vehicle_model);
            if (mpc_command.valid) {
                control.accel_cmd = mpc_command.accel_cmd;
                control.target_speed = mpc_command.target_speed;
                const double mpc_target_curvature = clamp_value(
                    std::tan(mpc_command.target_steer_angle) / std::max(geometry_.wheelbase, 1e-6),
                    -geometry_.max_curvature,
                    geometry_.max_curvature);
                if (tracked_vehicle) {
                    control.target_yaw_rate = clamp_value(
                        mpc_command.target_yaw_rate,
                        -geometry_.max_yaw_rate,
                        geometry_.max_yaw_rate);
                    control.target_curvature =
                        std::abs(control.target_speed) > 1e-4
                            ? clamp_value(control.target_yaw_rate / control.target_speed,
                                          -geometry_.max_curvature,
                                          geometry_.max_curvature)
                            : 0.0;
                    control.target_steer_angle = 0.0;
                    control.steer_rate_cmd = 0.0;
                    if (std::abs(mpc_command.heading_error) >= 0.72) {
                        control.target_speed = 0.0;
                        control.accel_cmd = clamp_value(
                            -navigation_speed_ / std::max(config_.dt, 1e-3),
                            -geometry_.max_decel,
                            geometry_.max_accel);
                        const double pivot_direction =
                            std::abs(control.target_yaw_rate) > 1e-4
                                ? control.target_yaw_rate
                                : -mpc_command.heading_error;
                        control.target_yaw_rate = std::copysign(
                            std::max(std::abs(control.target_yaw_rate), 0.32),
                            pivot_direction);
                        control.target_curvature = 0.0;
                    }
                } else {
                    control.target_curvature = mpc_target_curvature;
                    control.steer_rate_cmd = mpc_command.steer_rate_cmd;
                    control.target_steer_angle = mpc_command.target_steer_angle;
                    control.target_yaw_rate = control.target_speed * control.target_curvature;
                    if (mixed_gate_direct_step) {
                        const Vec2 target = gate_position(gates_[static_cast<size_t>(chosen_gate_index_)]);
                        const double target_distance = distance(navigation_position_, target);
                        const bool allow_direct_gate_pull =
                            !obstacle_closed_mixed_step || target_distance >= 0.55;
                        if (allow_direct_gate_pull) {
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
                    if (obstacle_closed_mixed_step &&
                        !mixed_gate_mode_active_ &&
                        reference_trajectory_.size() >= 2) {
                        const size_t road_heading_index =
                            mixed_rejoin_active_
                                ? reference_trajectory_.size() - 1
                                : std::min<size_t>(
                                      reference_trajectory_.size() - 1,
                                      std::max<size_t>(
                                          2,
                                          reference_trajectory_.size() / 3));
                        const double rejoin_heading_error = wrap_angle(
                            reference_trajectory_[road_heading_index].yaw -
                            navigation_yaw_);
                        const double rejoin_steer = clamp_value(
                            (mixed_rejoin_active_ ? 1.25 : 0.90) *
                                rejoin_heading_error,
                            -geometry_.max_steer_angle,
                            geometry_.max_steer_angle);
                        if (std::abs(rejoin_steer) >
                            std::abs(control.target_steer_angle)) {
                            control.target_steer_angle = rejoin_steer;
                            control.steer_rate_cmd = clamp_value(
                                (rejoin_steer - tracking_state.steer_angle) /
                                    std::max(config_.dt, 1e-3),
                                -geometry_.max_steer_rate,
                                geometry_.max_steer_rate);
                            control.target_curvature = clamp_value(
                                std::tan(rejoin_steer) /
                                    std::max(geometry_.wheelbase, 1e-6),
                                -geometry_.max_curvature,
                                geometry_.max_curvature);
                            control.target_yaw_rate =
                                control.target_speed * control.target_curvature;
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
                    planner_yaw_rate_ref_,
                    -geometry_.max_yaw_rate,
                    geometry_.max_yaw_rate);
                control.target_curvature =
                    std::abs(control.target_speed) > 1e-4
                        ? clamp_value(control.target_yaw_rate / control.target_speed,
                                      -geometry_.max_curvature,
                                      geometry_.max_curvature)
                        : 0.0;
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

    const bool tracked_gate_target_available =
        tracked_vehicle &&
        chosen_gate_index_ >= 0 &&
        chosen_gate_index_ < static_cast<int>(gates_.size()) &&
        (world_.environment_mode() == EnvironmentMode::UnstructuredGates ||
         (mixed_mode_enabled() && mixed_gate_mode_active_));
    if (tracked_gate_target_available) {
        const Vec2 target = gate_position(gates_[static_cast<size_t>(chosen_gate_index_)]);
        const double gate_heading_error = wrap_angle(
            angle_to(navigation_position_, target) - navigation_yaw_);
        const double yaw_rate_limit =
            (mixed_gate_mode_active_ ? 0.72 : 0.85) * geometry_.max_yaw_rate;
        const double direct_gate_yaw_rate = clamp_value(
            1.60 * gate_heading_error,
            -yaw_rate_limit,
            yaw_rate_limit);

        // The planner trajectory can initially preserve the current tangent
        // even when a newly selected gate lies on the opposite side.  Keep
        // the tracked MPC output tied to the physical gate bearing so the
        // differential drive cannot turn away from the selected opening.
        if (std::abs(gate_heading_error) >= 0.10 &&
            (control.target_yaw_rate * gate_heading_error <= 0.0 ||
             std::abs(control.target_yaw_rate) < 0.70 * std::abs(direct_gate_yaw_rate))) {
            control.target_yaw_rate = direct_gate_yaw_rate;
            control.target_curvature =
                std::abs(control.target_speed) > 1e-4
                    ? clamp_value(control.target_yaw_rate / control.target_speed,
                                  -geometry_.max_curvature,
                                  geometry_.max_curvature)
                    : 0.0;
        }

        const double direct_pivot_threshold = mixed_gate_mode_active_ ? 0.48 : 0.42;
        if (std::abs(gate_heading_error) >= direct_pivot_threshold) {
            control.target_speed = 0.0;
            control.accel_cmd = clamp_value(
                -navigation_speed_ / std::max(config_.dt, 1e-3),
                -geometry_.max_decel,
                geometry_.max_accel);
            control.target_yaw_rate = std::copysign(
                std::max(std::abs(direct_gate_yaw_rate), 0.32),
                direct_gate_yaw_rate);
            control.target_curvature = 0.0;
            control.target_steer_angle = 0.0;
            control.steer_rate_cmd = 0.0;
        }
    }

    if (tracked_vehicle && reference_trajectory_.size() >= 2) {
        const size_t heading_index =
            obstacle_closed_mixed_step && mixed_rejoin_active_
                ? reference_trajectory_.size() - 1
                : std::min<size_t>(2, reference_trajectory_.size() - 1);
        const double heading_error = wrap_angle(
            reference_trajectory_[heading_index].yaw - navigation_yaw_);
        const double pivot_threshold = mixed_gate_mode_active_ ? 0.48 : 0.72;
        if (std::abs(heading_error) >= pivot_threshold) {
            const double pivot_yaw_rate = clamp_value(
                1.45 * heading_error,
                -geometry_.max_yaw_rate,
                geometry_.max_yaw_rate);
            control.target_speed = 0.0;
            control.accel_cmd = clamp_value(
                -navigation_speed_ / std::max(config_.dt, 1e-3),
                -geometry_.max_decel,
                geometry_.max_accel);
            control.target_yaw_rate = std::copysign(
                std::max(std::abs(pivot_yaw_rate), 0.32),
                pivot_yaw_rate);
            control.target_curvature = 0.0;
            control.target_steer_angle = 0.0;
            control.steer_rate_cmd = 0.0;
        }
    }

    if (!tracked_vehicle &&
        !goal_stop_active_ &&
        world_.environment_mode() == EnvironmentMode::UnstructuredGates &&
        use_dynamic_lidar_gates() &&
        reference_trajectory_.size() >= 2 &&
        !active_gate_indices().empty()) {
        // Apply the gate-motion floor after MPC. A pre-MPC desired-speed
        // floor was insufficient because the finite-horizon solution could
        // still alternate zero and low commands, repeatedly dropping the
        // calibrated motor below break-away speed.
        const double gate_motion_floor =
            compact_unstructured_world(world_) ? 0.055 : 0.080;
        control.target_speed = std::max(
            control.target_speed,
            std::min(gate_motion_floor, config_.cruise_speed_limit));
        control.target_yaw_rate = control.target_speed * control.target_curvature;
    }

    if (goal_stop_active_) {
        control.target_speed = 0.0;
        control.accel_cmd = clamp_value(
            -navigation_speed_ / std::max(config_.dt, 1e-3),
            -geometry_.max_decel,
            0.0);
        control.target_yaw_rate = 0.0;
        control.target_curvature = 0.0;
        control.target_steer_angle = 0.0;
        control.steer_rate_cmd = clamp_value(
            -vehicle_.steer_angle / std::max(config_.dt, 1e-3),
            -geometry_.max_steer_rate,
            geometry_.max_steer_rate);
    }

    VehicleControlInput applied_control = control;
    if (config_.calibrated_conditions && config_.command_delay_steps > 0) {
        delayed_controls_.push_back(control);
        if (delayed_controls_.size() <= static_cast<size_t>(config_.command_delay_steps)) {
            applied_control = VehicleControlInput{};
            applied_control.target_speed = 0.0;
            applied_control.target_yaw_rate = 0.0;
        } else {
            applied_control = delayed_controls_.front();
            delayed_controls_.pop_front();
        }
    }

    vehicle_model_->step(config_.dt, applied_control);
    sim_time_ += config_.dt;
    ++step_count_;

    const auto lidar_start = std::chrono::steady_clock::now();
    update_lidar();
    lidar_compute_ms_ = elapsed_ms(lidar_start, std::chrono::steady_clock::now());

    const auto estimator_start = std::chrono::steady_clock::now();
    update_navigation_state(config_.dt);
    update_slam_map();
    estimator_compute_ms_ = elapsed_ms(estimator_start, std::chrono::steady_clock::now());
    sync_planner_from_vehicle(false);
    update_vehicle_snapshot();
    const bool compact_mixed_world =
        world_.environment_mode() == EnvironmentMode::MixedRoadGates &&
        (world_span_m(world_) <= 2.50 || world_.structured_preset() == StructuredMapPreset::IdealCircle);
    const bool compact_unstructured_world =
        world_.environment_mode() == EnvironmentMode::UnstructuredGates &&
        world_span_m(world_) <= 5.0;
    const bool aruco_reference_mixed_world =
        world_.environment_mode() == EnvironmentMode::MixedRoadGates &&
        world_.structured_preset() == StructuredMapPreset::TankCircuit &&
        !world_.obstacles().empty();
    const bool normalized_structured_arena =
        world_.environment_mode() == EnvironmentMode::StructuredRoad &&
        world_.obstacles().empty() &&
        world_.perception_obstacles().empty() &&
        world_span_m(world_) >= 0.90 &&
        world_span_m(world_) <= 1.25;
    // The normalized indoor structured baseline uses bounds as chart extents; there are
    // no simulated walls or obstacles on that validation loop.
    collision_cause_.clear();
    if (micro_structured_world(world_) || normalized_structured_arena) {
        collision_ = false;
    } else if (aruco_reference_mixed_world) {
        // These bounds delimit the registered ArUco coordinate chart; they are
        // not physical walls. The map already carries a 0.20 m chart margin,
        // so keep the robot reference point inside it without interpreting a
        // rotated body corner as a collision with an imaginary wall.
        const Rect bounds = world_.bounds();
        constexpr double kRegistrationChartToleranceM = 0.03;
        collision_ = !point_in_expanded_rect(
            vehicle_.position,
            bounds,
            kRegistrationChartToleranceM);
        if (collision_) {
            collision_cause_ = "bounds";
        }
    } else {
        const double collision_padding =
            compact_mixed_world
                ? -0.025
                : (compact_unstructured_world
                       ? -0.025
                       : (compact_structured_world(world_) ? 0.0 : 0.05));
        collision_ = world_.collides(vehicle_.body_corners, collision_padding);
        if (collision_) {
            collision_cause_ = "obstacle_or_bounds";
            for (const Vec2& corner : vehicle_.body_corners) {
                if (!point_in_bounds(world_.bounds(), corner)) {
                    collision_cause_ = "bounds";
                    break;
                }
            }
        }
    }
    if (structured_road_is_closed_loop(world_) || mixed_closed_gate_sequence(world_)) {
        const bool tiny_indoor_loop = micro_structured_world(world_);
        const bool pure_structured_closed_finish =
            world_.environment_mode() == EnvironmentMode::StructuredRoad &&
            structured_road_is_closed_loop(world_);
        const double structured_route_length =
            structured_goal_ready_ && structured_goal_progress_target_ > 0.10
                ? structured_goal_progress_target_
                : cl_.end_point_s;
        const double wrapped_track_s =
            pure_structured_closed_finish && std::isfinite(structured_route_last_s_)
                ? wrap_arc_length(structured_route_last_s_, structured_route_length)
                : (std::isfinite(x0_.x)
                       ? wrap_arc_length(x0_.x, structured_route_length)
                       : 0.0);
        const double goal_position_distance =
            distance(vehicle_.position, structured_goal_ready_ ? structured_goal_position_ : world_.start());
        const bool mixed_closed_sequence = mixed_closed_gate_sequence(world_);
        const bool semantic_closed_sequence = mixed_closed_semantic_gate_sequence(world_);
        const bool dynamic_obstacle_closed_sequence =
            mixed_closed_sequence && !semantic_closed_sequence;
        const double route_remaining =
            structured_goal_ready_
                ? std::max(structured_goal_progress_target_ - structured_progress_s_, 0.0)
                : cl_.end_point_s;
        distance_to_goal_ =
            dynamic_obstacle_closed_sequence
                ? std::max(route_remaining, goal_position_distance)
                : route_remaining;
        const double progress_margin =
            tiny_indoor_loop ? std::clamp(0.11 * structured_route_length, 0.065, 0.10)
                             : std::clamp(0.03 * std::max(structured_route_length, 1.0), 0.05, 0.20);
        const double start_window =
            tiny_indoor_loop ? std::clamp(0.10 * structured_route_length, 0.065, 0.10)
                             : std::clamp(0.04 * std::max(structured_route_length, 1.0), 0.10, 0.35);
        const double goal_position_acceptance =
            tiny_indoor_loop
                ? std::clamp(0.28 * world_span_m(world_), 0.09, 0.11)
                : (tracked_vehicle
                       ? std::clamp(0.20 * world_span_m(world_), 0.18, 0.24)
                       : std::clamp(0.15 * world_span_m(world_), 0.12, 0.18));
        const bool returned_to_start =
            wrapped_track_s <= start_window ||
            wrapped_track_s >= std::max(structured_route_length - start_window, 0.0);
        const bool physically_near_start = goal_position_distance < goal_position_acceptance;
        const bool progress_complete =
            structured_goal_ready_ &&
            structured_progress_s_ + progress_margin >= structured_goal_progress_target_;
        const bool closed_loop_finished =
            progress_complete && physically_near_start && returned_to_start;
        const double finish_cross_track_limit =
            dynamic_obstacle_closed_sequence ? 1.20 : 0.50;
        const bool finish_inside_road =
            std::abs(x0_.n) <= finish_cross_track_limit &&
            tracker_cross_track_error_ <= finish_cross_track_limit + 0.08;
        if (dynamic_obstacle_closed_sequence) {
            // Obstacles that do not block the physical footprint are handled on
            // the structured road and must not create fictitious mandatory
            // gates. Completion requires a full lap plus evidence that mixed
            // arbitration crossed at least one real LiDAR-derived bypass.
            constexpr int kRequiredMixedBypasses = 1;
            goal_reached_ =
                progress_complete &&
                physically_near_start &&
                finish_inside_road &&
                !mixed_gate_mode_active_ &&
                dynamic_lidar_passed_gates_ >= kRequiredMixedBypasses;
        } else if (semantic_closed_sequence) {
            goal_reached_ =
                closed_loop_finished &&
                count_passed_gates() >= 2;
        } else {
            goal_reached_ = closed_loop_finished;
        }
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
        const bool road_gate_validation_mixed =
            mixed &&
            world_.structured_preset() == StructuredMapPreset::HardwareTrack &&
            world_span_m(world_) > 1.20 &&
            !structured_road_is_closed_loop(world_);
        const double goal_acceptance =
            lab_scale_mixed_open
                // The reference point may reach the hardware goal while the
                // definitive 25 cm car footprint is already close to the map
                // chart edge.  Use one common physical acceptance region for
                // car and tank; 18 cm is still smaller than one car length.
                ? 0.18
                : road_gate_validation_mixed
                ? 0.35
                : compact_mixed
                ? std::clamp(0.08 * world_span_m(world_), 0.06, 0.12)
                : (compact_unstructured
                       ? std::clamp(0.05 * world_span_m(world_), 0.08, 0.16)
                       : 0.75);
        // Intermediate gate crossings are evidence of navigation progress,
        // never a substitute for reaching the configured physical goal.
        goal_reached_ =
            distance_to_goal_ < goal_acceptance &&
            std::abs(vehicle_.speed) < 0.15;
    }
    step_compute_ms_ = elapsed_ms(step_start, std::chrono::steady_clock::now());
    update_telemetry();
}

}  // namespace thesis_sim
