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
        !mixed_rejoin_active_ &&
        (!has_planner_clothoid || world_.road_centerline().size() < 2)) {
        planned_trajectory_ = world_.road_centerline();
        reference_trajectory_ = build_reference_waypoints(planned_trajectory_, planner_speed_ref_);
        return;
    }

    if ((world_.environment_mode() == EnvironmentMode::UnstructuredGates || mixed_gate_reference) &&
        chosen_gate_index_ < 0) {
        return;
    }
    const bool unstructured_goal_reference =
        world_.environment_mode() == EnvironmentMode::UnstructuredGates &&
        chosen_gate_index_ >= 0 &&
        chosen_gate_index_ < static_cast<int>(gates_.size()) &&
        distance(gate_position(gates_[static_cast<size_t>(chosen_gate_index_)]),
                 world_.goal()) <= 0.02;
    if (unstructured_goal_reference) {
        const Vec2 start = navigation_position_;
        const Vec2 target = world_.goal();
        const double target_distance = distance(start, target);
        if (target_distance >= 0.02) {
            const Vec2 target_axis{
                (target.x - start.x) / target_distance,
                (target.y - start.y) / target_distance,
            };
            const double start_handle = std::clamp(
                0.30 * target_distance, 0.05, 0.18);
            const double target_handle = std::clamp(
                0.36 * target_distance, 0.06, 0.20);
            const Vec2 p1{
                start.x + std::cos(navigation_yaw_) * start_handle,
                start.y + std::sin(navigation_yaw_) * start_handle,
            };
            const Vec2 p2{
                target.x - target_axis.x * target_handle,
                target.y - target_axis.y * target_handle,
            };
            constexpr int kGoalSamples = 28;
            planned_trajectory_.reserve(kGoalSamples);
            for (int i = 0; i < kGoalSamples; ++i) {
                const double t =
                    static_cast<double>(i) /
                    static_cast<double>(kGoalSamples - 1);
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
            reference_trajectory_ = build_reference_waypoints(
                planned_trajectory_, planner_speed_ref_);
        }
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
            const bool obstacle_closed_gate_reference =
                mixed_closed_gate_sequence(world_) &&
                !mixed_closed_semantic_gate_sequence(world_);
            Vec2 exit_axis = gate_axis;
            if (obstacle_closed_gate_reference) {
                exit_axis = {
                    std::cos(selected_gate.road.PSI_end),
                    std::sin(selected_gate.road.PSI_end),
                };
                if (exit_axis.x * gate_axis.x + exit_axis.y * gate_axis.y < -0.35) {
                    exit_axis = {-exit_axis.x, -exit_axis.y};
                }
            }
            const double entry_scale = obstacle_closed_gate_reference ? 0.32 : 0.45;
            const double exit_scale = obstacle_closed_gate_reference ? 0.42 : 0.15;
            const Vec2 p1{
                start.x + std::cos(navigation_yaw_) * entry_scale * gate_distance,
                start.y + std::sin(navigation_yaw_) * entry_scale * gate_distance,
            };
            const Vec2 p2{
                target.x - exit_axis.x * exit_scale * gate_distance,
                target.y - exit_axis.y * exit_scale * gate_distance,
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

    const bool compact_open_mixed_rejoin =
        mixed_mode_enabled() &&
        mixed_rejoin_active_ &&
        world_span_m(world_) <= 1.20 &&
        !structured_road_is_closed_loop(world_) &&
        !mixed_closed_gate_sequence(world_) &&
        world_.road_centerline().size() >= 2;
    if (compact_open_mixed_rejoin) {
        // Hardware Lab has an open road, so it cannot use the closed-loop
        // projection/rejoin branch below.  Previously it fell through to a
        // stale generic clothoid: the car kept driving near the upper bound,
        // while the rejoin diagnostics remained infinite and arbitration
        // could never return to ROAD.  Rejoin the actual road at an anchor
        // that is strictly ahead of the current longitudinal progress.
        const Vec2 road_start = world_.road_centerline().front();
        const Vec2 road_end = world_.road_centerline().back();
        Vec2 road_axis = subtract(road_end, road_start);
        const double road_length = std::hypot(road_axis.x, road_axis.y);
        if (road_length > 1e-6) {
            road_axis.x /= road_length;
            road_axis.y /= road_length;
            const Vec2 road_normal{-road_axis.y, road_axis.x};
            const Vec2 relative = subtract(navigation_position_, road_start);
            const double current_s = clamp_value(
                dot_product(relative, road_axis), 0.0, road_length);
            const double road_lateral = dot_product(relative, road_normal);
            const double road_heading = std::atan2(road_axis.y, road_axis.x);
            mixed_rejoin_road_lateral_error_ = std::abs(road_lateral);
            mixed_rejoin_road_heading_error_deg_ =
                std::abs(wrap_angle(road_heading - navigation_yaw_)) *
                180.0 / 3.14159265358979323846;

            double target_s = std::min(current_s + 0.28, road_length);
            for (const GateSpec& spec : world_.gates()) {
                if (spec.final) {
                    continue;
                }
                const Vec2 gate_relative = subtract(spec.position, road_start);
                const double gate_lateral = std::abs(
                    dot_product(gate_relative, road_normal));
                const double gate_s = dot_product(gate_relative, road_axis);
                if (gate_lateral <= 0.06 && gate_s > current_s + 0.08) {
                    target_s = std::max(target_s, std::min(gate_s, road_length));
                }
            }
            const Vec2 start = navigation_position_;
            const Vec2 target{
                road_start.x + road_axis.x * target_s,
                road_start.y + road_axis.y * target_s,
            };
            const double target_distance = std::max(distance(start, target), 1e-3);
            const double start_handle = std::clamp(
                0.30 * target_distance, 0.055, 0.15);
            const double target_handle = std::clamp(
                0.38 * target_distance, 0.070, 0.18);
            const Vec2 p1{
                start.x + std::cos(navigation_yaw_) * start_handle,
                start.y + std::sin(navigation_yaw_) * start_handle,
            };
            const Vec2 p2{
                target.x - road_axis.x * target_handle,
                target.y - road_axis.y * target_handle,
            };
            constexpr int kOpenRejoinSamples = 32;
            planned_trajectory_.reserve(kOpenRejoinSamples);
            for (int i = 0; i < kOpenRejoinSamples; ++i) {
                const double t =
                    static_cast<double>(i) /
                    static_cast<double>(kOpenRejoinSamples - 1);
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
            reference_trajectory_ = build_reference_waypoints(
                planned_trajectory_, planner_speed_ref_);
        }
        return;
    }

    const bool pure_structured_closed_world =
        world_.environment_mode() == EnvironmentMode::StructuredRoad &&
        structured_road_is_closed_loop(world_);
    const bool obstacle_closed_mixed_world =
        mixed_mode_enabled() &&
        mixed_closed_gate_sequence(world_) &&
        !mixed_closed_semantic_gate_sequence(world_);
    if (pure_structured_closed_world && world_.road_centerline().size() >= 2) {
        const ClosedPathProjection route_projection = project_closed_path(
            world_.road_centerline(),
            navigation_position_,
            std::isfinite(structured_route_last_s_)
                ? structured_route_last_s_
                : 0.0);
        if (route_projection.valid && route_projection.length > 0.10) {
            if (!structured_goal_ready_) {
                structured_goal_progress_target_ = route_projection.length;
                structured_goal_position_ = world_.start();
                structured_goal_ready_ = true;
            }
            if (std::isfinite(structured_route_last_s_)) {
                structured_progress_s_ = std::max(
                    0.0,
                    structured_progress_s_ + closed_loop_forward_delta(
                                                   structured_route_last_s_,
                                                   route_projection.s,
                                                   route_projection.length));
            } else {
                structured_progress_s_ = 0.0;
            }
            structured_route_last_s_ = route_projection.s;
        }
    }
    if (obstacle_closed_mixed_world && world_.road_centerline().size() >= 2) {
        const ClosedPathProjection physical_projection = project_closed_path(
            world_.road_centerline(),
            navigation_position_,
            std::numeric_limits<double>::quiet_NaN());
        ClosedPathProjection projection =
            project_closed_path(
                world_.road_centerline(),
                navigation_position_,
                pure_structured_closed_world && !std::isfinite(structured_last_s_)
                    ? 0.0
                    : structured_last_s_);
        if (projection.valid && projection.length > 0.10) {
            if (mixed_rejoin_active_ && std::isfinite(structured_last_s_)) {
                const double maximum_forward_rejoin_step = std::max(
                    0.08,
                    navigation_speed_ * std::max(config_.dt, 1e-3) * 4.0 + 0.05);
                const double projected_forward_delta = closed_loop_forward_delta(
                    structured_last_s_, projection.s, projection.length);
                if (projected_forward_delta > maximum_forward_rejoin_step) {
                    projection.s = wrap_arc_length(
                        structured_last_s_ + maximum_forward_rejoin_step,
                        projection.length);
                    const ClosedPathProjection constrained = project_closed_path(
                        world_.road_centerline(),
                        sample_closed_path_span(
                            world_.road_centerline(),
                            projection.s,
                            0.02,
                            2,
                            projection.length).front(),
                        projection.s);
                    if (constrained.valid) {
                        projection = constrained;
                    }
                }
            }
            if (!structured_goal_ready_) {
                structured_goal_progress_target_ = projection.length;
                structured_goal_position_ = world_.goal();
                structured_goal_ready_ = true;
            }

            if (std::isfinite(structured_last_s_)) {
                structured_progress_s_ = std::max(
                    0.0,
                    structured_progress_s_ +
                        closed_loop_forward_delta(structured_last_s_, projection.s, projection.length));
            } else if (!pure_structured_closed_world) {
                structured_progress_s_ = std::max(structured_progress_s_, projection.s);
            } else {
                structured_progress_s_ = 0.0;
            }
            structured_last_s_ = projection.s;
            x0_.x = projection.s;
            x0_.n = projection.lateral;
            x0_.v = navigation_speed_;
            x0_.a = navigation_accel_;
            x0_.c = navigation_curvature_;

            if (mixed_rejoin_active_) {
                const ClosedPathProjection& rejoin_projection =
                    physical_projection.valid ? physical_projection : projection;
                const bool tracked_vehicle =
                    config_.vehicle_model == VehicleModelKind::TrackedVehicle;
                const double tangent_step = 0.035;
                const Vec2 tangent_before = sample_closed_path(
                    world_.road_centerline(),
                    rejoin_projection.s - tangent_step,
                    rejoin_projection.length);
                const Vec2 tangent_after = sample_closed_path(
                    world_.road_centerline(),
                    rejoin_projection.s + tangent_step,
                    rejoin_projection.length);
                const double road_heading = angle_to(tangent_before, tangent_after);
                mixed_rejoin_road_lateral_error_ =
                    std::abs(rejoin_projection.lateral);
                mixed_rejoin_road_heading_error_deg_ =
                    std::abs(wrap_angle(road_heading - navigation_yaw_)) *
                    180.0 / 3.14159265358979323846;

                // Rejoin through a local cubic whose endpoint is always ahead
                // of current structured progress. Sampling the global loop
                // directly from the nearest projection creates a discontinuous
                // reference after a bypass and was the source of the long
                // oscillating rejoin seen in Closed Obstacle Road.
                const double heading_mismatch =
                    std::abs(wrap_angle(road_heading - navigation_yaw_));
                const double rejoin_ahead = std::clamp(
                    (tracked_vehicle ? 0.25 : 0.31) +
                        0.75 * std::abs(rejoin_projection.lateral) +
                        0.08 * heading_mismatch,
                    tracked_vehicle ? 0.25 : 0.31,
                    tracked_vehicle ? 0.48 : 0.62);
                const double target_s = wrap_arc_length(
                    rejoin_projection.s + rejoin_ahead,
                    rejoin_projection.length);
                const Vec2 target = sample_closed_path(
                    world_.road_centerline(), target_s, rejoin_projection.length);
                const Vec2 target_before = sample_closed_path(
                    world_.road_centerline(),
                    target_s - tangent_step,
                    rejoin_projection.length);
                const Vec2 target_after = sample_closed_path(
                    world_.road_centerline(),
                    target_s + tangent_step,
                    rejoin_projection.length);
                const double target_heading = angle_to(target_before, target_after);
                const Vec2 start = navigation_position_;
                const double target_distance = std::max(distance(start, target), 1e-3);
                const double start_handle = std::clamp(
                    0.26 * target_distance,
                    tracked_vehicle ? 0.045 : 0.060,
                    tracked_vehicle ? 0.13 : 0.17);
                const double target_handle = std::clamp(
                    0.30 * target_distance,
                    tracked_vehicle ? 0.060 : 0.075,
                    tracked_vehicle ? 0.15 : 0.20);
                const Vec2 p1{
                    start.x + std::cos(navigation_yaw_) * start_handle,
                    start.y + std::sin(navigation_yaw_) * start_handle,
                };
                const Vec2 p2{
                    target.x - std::cos(target_heading) * target_handle,
                    target.y - std::sin(target_heading) * target_handle,
                };
                constexpr int kRejoinSamples = 34;
                planned_trajectory_.reserve(kRejoinSamples);
                for (int i = 0; i < kRejoinSamples; ++i) {
                    const double t =
                        static_cast<double>(i) /
                        static_cast<double>(kRejoinSamples - 1);
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
                reference_trajectory_ = build_reference_waypoints(
                    planned_trajectory_, planner_speed_ref_);
                return;
            }

            const bool lab_scale_mixed_loop =
                obstacle_closed_mixed_world && world_span_m(world_) <= 2.25;
            const double reference_span =
                pure_structured_closed_world
                    ? std::min(
                          projection.length,
                          std::clamp(
                              0.22 * projection.length,
                              0.42,
                              0.62))
                    : (lab_scale_mixed_loop
                           ? std::min(
                                 projection.length,
                                 std::clamp(
                                     0.16 * projection.length,
                                     0.48,
                                     0.72))
                           : std::min(
                                 projection.length,
                                 std::clamp(
                                     0.24 * projection.length,
                                     1.25,
                                     1.70)));
            planned_trajectory_ = sample_closed_path_span(
                world_.road_centerline(),
                projection.s,
                reference_span,
                pure_structured_closed_world ? 28 : 36,
                projection.length);
            reference_trajectory_ = build_reference_waypoints(planned_trajectory_, planner_speed_ref_);
            return;
        }
    }
    if (!has_planner_clothoid) {
        return;
    }

    const bool closed_structured_loop =
        structured_road_is_closed_loop(world_) || mixed_closed_gate_sequence(world_);
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
        if (!pure_structured_closed_world && !structured_goal_ready_) {
            if (obstacle_closed_mixed_world) {
                double goal_s = cl_.end_point_s;
                double goal_lateral = 0.0;
                if (!project_curvilinear_state(
                        cl_,
                        world_.goal(),
                        0.0,
                        true,
                        &goal_s,
                        &goal_lateral) ||
                    goal_s < 0.35) {
                    goal_s = cl_.end_point_s;
                }
                structured_goal_progress_target_ =
                    clamp_value(goal_s, 0.35, cl_.end_point_s);
                structured_goal_position_ = world_.goal();
            } else {
                structured_goal_progress_target_ = std::max(0.0, cl_.end_point_s);
                structured_goal_position_ = world_.start();
            }
            structured_goal_ready_ = structured_goal_progress_target_ > 0.0;
        }

        if (!pure_structured_closed_world) {
            if (!std::isfinite(structured_last_s_)) {
                structured_progress_s_ = 0.0;
            } else {
                structured_progress_s_ = std::max(
                    0.0,
                    structured_progress_s_ + structured_progress_delta);
            }
        }
        structured_last_s_ = s_current;
    }

    const bool unstructured =
        world_.environment_mode() == EnvironmentMode::UnstructuredGates || mixed_gate_reference;
    const bool obstacle_closed_mixed_reference =
        obstacle_closed_mixed_world &&
        !mixed_gate_reference;
    const double lookahead_distance =
        unstructured ? 22.0 : (obstacle_closed_mixed_reference ? 1.25 : (closed_structured_loop ? 14.0 : 18.0));
    const int sample_count =
        unstructured ? 48 : (obstacle_closed_mixed_reference ? 32 : (closed_structured_loop ? 64 : 36));
    double s_start = closed_structured_loop ? wrap_arc_length(s_current, cl_.end_point_s) : std::max(0.0, s_current);
    double s_end = 0.0;
    if (closed_structured_loop) {
        const double max_span =
            obstacle_closed_mixed_reference
                ? std::min(cl_.end_point_s, std::clamp(0.22 * cl_.end_point_s, 1.05, 1.35))
                : closed_loop_reference_span(cl_.end_point_s, lookahead_distance);
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
