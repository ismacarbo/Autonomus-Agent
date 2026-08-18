#include "support.inc"

namespace thesis_sim {

void HardwarePlannerRunner::update_planner_references(double dt) {
    planner_accel_ref_ = clamp_value(
        estimate_.accel + last_j_ * dt,
        -geometry_.max_decel,
        geometry_.max_accel);
    planner_speed_ref_ = clamp_value(
        estimate_.speed + planner_accel_ref_ * dt,
        0.0,
        config_.cruise_speed_limit);
    const auto update_tracked_yaw_reference = [&]() {
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
            planner_initial_yaw_rate_ = estimate_.yaw_rate;
        }
    };

    const bool mixed_gate_active = world_is_mixed(world_) && locked_gap_goal_.has_value();
    if (world_has_structured_reference(world_) && !mixed_gate_active) {
        const bool tiny_indoor_loop = tiny_indoor_structured_loop(world_);
        const double slowdown_radius = structured_slowdown_radius(world_, config_.goal_slowdown_radius_m);
        const double active_distance = structured_goal_active_distance(world_, config_.goal_tolerance_m);
        const double base_speed =
            tiny_indoor_loop ? 0.024 : (micro_structured_world(world_) ? 0.035 : 0.060);
        const double alpha = clamp_value(distance_to_goal_ / slowdown_radius, 0.0, 1.0);
        double goal_speed_cap = base_speed + alpha * (config_.cruise_speed_limit - base_speed);
        if (distance_to_goal_ < active_distance) {
            goal_speed_cap = std::min(
                goal_speed_cap,
                tiny_indoor_loop ? 0.035 : (micro_structured_world(world_) ? 0.045 : 0.10));
        }
        const bool keep_tracking =
            !goal_reached_ &&
            distance_to_goal_ > active_distance;
        if (keep_tracking) {
            planner_speed_ref_ = std::max(
                planner_speed_ref_,
                structured_tracking_speed_floor(world_, config_.cruise_speed_limit));
        }
        planner_speed_ref_ = clamp_value(planner_speed_ref_, 0.0, goal_speed_cap);
        planner_accel_ref_ = clamp_value(
            (planner_speed_ref_ - estimate_.speed) / std::max(dt, 1e-3),
            -geometry_.max_decel,
            geometry_.max_accel);
        update_tracked_yaw_reference();
        return;
    }

    if (world_.environment_mode() == EnvironmentMode::UnstructuredGates || mixed_gate_active) {
        if (startup_scan_active() ||
            (strict_locked_gate_motion_enabled() && !locked_gap_goal_.has_value()) ||
            chosen_gate_index_ < 0 ||
            chosen_gate_index_ >= static_cast<int>(gate_specs_.size())) {
            planner_accel_ref_ = 0.0;
            planner_speed_ref_ = 0.0;
            update_tracked_yaw_reference();
            return;
        }

        const Vec2 target = gate_specs_[static_cast<size_t>(chosen_gate_index_)].position;
        const double gap_distance = distance(estimate_.position, target);
        const double cruise_speed = clamp_value(
            config_.gap_extraction.gap_goal_cruise_speed_mps,
            0.05,
            config_.cruise_speed_limit);
        const double distance_scale = clamp_value(
            gap_distance / std::max(config_.gap_extraction.max_target_distance_m, 0.35),
            0.0,
            1.0);
        double speed_ref = 0.05 + (cruise_speed - 0.05) * distance_scale;
        if (estimate_.front_lidar_distance > 0.0) {
            const double clearance_scale = clamp_value(
                (estimate_.front_lidar_distance - config_.localization.obstacle_stop_distance_m) / 0.45,
                0.0,
                1.0);
            speed_ref *= 0.35 + 0.65 * clearance_scale;
        }
        if (gap_distance < std::max(config_.gap_extraction.gap_goal_tolerance_m * 2.0, 0.22)) {
            speed_ref = std::min(speed_ref, 0.08);
        }

        planner_speed_ref_ = clamp_value(speed_ref, 0.0, cruise_speed);
        planner_accel_ref_ = clamp_value(
            (planner_speed_ref_ - estimate_.speed) / std::max(dt, 1e-3),
            -geometry_.max_decel,
            geometry_.max_accel);
        update_tracked_yaw_reference();
        return;
    }

    const double slowdown_radius = std::max(config_.goal_slowdown_radius_m, 0.25);
    const double alpha = clamp_value(distance_to_goal_ / slowdown_radius, 0.0, 1.0);
    double goal_speed_cap = 0.06 + alpha * (config_.cruise_speed_limit - 0.06);
    if (distance_to_goal_ < std::max(config_.goal_tolerance_m * 2.0, 0.35)) {
        goal_speed_cap = std::min(goal_speed_cap, 0.10);
    }
    planner_speed_ref_ = std::min(planner_speed_ref_, goal_speed_cap);
    update_tracked_yaw_reference();
}

void HardwarePlannerRunner::update_selected_trajectory() {
    planned_trajectory_.clear();
    reference_trajectory_.clear();
    diagnostics_.planner_has_reference = false;
    const bool mixed_gate_active = world_is_mixed(world_) && locked_gap_goal_.has_value();
    const auto assign_structured_centerline_fallback = [&]() {
        if (!world_has_structured_reference(world_) ||
            mixed_gate_active ||
            world_.road_centerline().size() < 2) {
            return false;
        }
        planned_trajectory_ = world_.road_centerline();
        reference_trajectory_ = build_reference_waypoints(planned_trajectory_, planner_speed_ref_);
        diagnostics_.planner_has_reference = reference_trajectory_.size() >= 2;
        return diagnostics_.planner_has_reference;
    };
    const auto assign_compact_mixed_gate_fallback = [&]() {
        if (!mixed_gate_active ||
            !locked_gap_goal_.has_value() ||
            world_span_m(world_) > 1.25) {
            return false;
        }
        const Vec2 target = *locked_gap_goal_;
        const double gate_distance = distance(estimate_.position, target);
        if (!(gate_distance > 0.025)) {
            return false;
        }

        const int sample_count = std::max(
            3,
            static_cast<int>(std::ceil(gate_distance / 0.045)) + 1);
        planned_trajectory_.clear();
        planned_trajectory_.reserve(static_cast<size_t>(sample_count));
        for (int i = 0; i < sample_count; ++i) {
            const double alpha = sample_count > 1
                ? static_cast<double>(i) / static_cast<double>(sample_count - 1)
                : 1.0;
            planned_trajectory_.push_back({
                estimate_.position.x + alpha * (target.x - estimate_.position.x),
                estimate_.position.y + alpha * (target.y - estimate_.position.y),
            });
        }
        const double local_speed_ref = clamp_value(
            std::max(planner_speed_ref_, 0.045),
            0.0,
            std::min(config_.cruise_speed_limit, config_.gap_extraction.gap_goal_cruise_speed_mps));
        reference_trajectory_ = build_reference_waypoints(planned_trajectory_, local_speed_ref);
        diagnostics_.planner_has_reference = reference_trajectory_.size() >= 2;
        return diagnostics_.planner_has_reference;
    };
    diagnostics_.chosen_gate_distance =
        chosen_gate_index_ >= 0 && chosen_gate_index_ < static_cast<int>(gates_.size())
            ? distance_to_gate_point(gates_[static_cast<size_t>(chosen_gate_index_)], estimate_.position)
            : std::numeric_limits<double>::infinity();
    const bool has_planner_clothoid =
        cl_.prev_road.num_segments() > 0 &&
        std::isfinite(cl_.end_point_s) &&
        cl_.end_point_s > 0.10;

    if (world_has_structured_reference(world_) &&
        !mixed_gate_active &&
        (!has_planner_clothoid || world_.road_centerline().size() < 2)) {
        assign_structured_centerline_fallback();
        return;
    }

    if (world_.environment_mode() == EnvironmentMode::UnstructuredGates &&
        (chosen_gate_index_ < 0 ||
         (strict_locked_gate_motion_enabled() && !locked_gap_goal_.has_value()))) {
        return;
    }
    if (world_is_mixed(world_) &&
        !mixed_gate_active &&
        road_ == nullptr) {
        return;
    }
    if (assign_compact_mixed_gate_fallback()) {
        return;
    }
    if (!has_planner_clothoid) {
        return;
    }

    const bool closed_structured_loop = structured_road_is_closed_loop(world_);
    const bool prefer_hinted_loop_projection =
        world_is_mixed(world_) &&
        closed_structured_loop &&
        world_span_m(world_) <= 2.50;
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
    if (world_has_structured_reference(world_) && !mixed_gate_active) {
        if (std::isfinite(structured_last_s_)) {
            s_current = structured_last_s_;
        }
        if (closed_structured_loop) {
            s_current = wrap_arc_length(s_current, cl_.end_point_s);
        } else {
            s_current = clamp_value(std::max(0.0, s_current), 0.0, cl_.end_point_s);
        }
    } else if (!project_curvilinear_state(
                   cl_,
                   estimate_.position,
                   s_current,
                   closed_structured_loop,
                   prefer_hinted_loop_projection,
                   &s_current,
                   &lateral_offset)) {
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
            structured_goal_progress_target_ = std::max(0.0, cl_.end_point_s);
            structured_goal_position_ = world_.start();
            structured_goal_ready_ = structured_goal_progress_target_ > 0.0;
        }
        structured_last_s_ = s_current;
    }

    const bool unstructured =
        world_.environment_mode() == EnvironmentMode::UnstructuredGates ||
        mixed_gate_active;
    const bool tiny_indoor_loop = tiny_indoor_structured_loop(world_);
    const bool micro_loop = micro_structured_world(world_);
    const bool compact_mixed_loop = compact_mixed_structured_loop(world_);
    const double lookahead_distance =
        unstructured ? 22.0
        : (closed_structured_loop
               ? (tiny_indoor_loop ? 0.22 : (micro_loop ? 0.40 : (compact_mixed_loop ? 0.55 : 14.0)))
               : 18.0);
    const int sample_count =
        unstructured ? 48
        : (closed_structured_loop
               ? (tiny_indoor_loop ? 22 : (micro_loop ? 28 : (compact_mixed_loop ? 32 : 64)))
               : 36);
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
    const double min_reference_span =
        closed_structured_loop
            ? (tiny_indoor_loop ? 0.04 : (micro_loop ? 0.06 : (compact_mixed_loop ? 0.08 : 0.10)))
            : 0.10;
    if (!(s_end > s_start + min_reference_span)) {
        assign_structured_centerline_fallback();
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
    if (manual_unstructured_exploration(world_)) {
        // A clear straight chord to the detected aperture does not guarantee
        // that the car-like clothoid is safe. Validate the complete oriented
        // footprint against both the editor bounds and the current LiDAR
        // endpoints before the MPC is allowed to follow it. This catches the
        // outward loop produced by lateral/rear gates and curved approaches
        // that graze one of the two aperture posts.
        const double footprint_margin_m = 0.010;
        const double half_length = 0.5 * geometry_.body_length + footprint_margin_m;
        const double half_width = 0.5 * geometry_.body_width + footprint_margin_m;
        const double global_map_resolution =
            exploration_map_source_ == ExplorationMapSource::SlamToolbox
                ? std::max(slam_toolbox_snapshot_.map_resolution_m, 0.02)
                : std::max(config_.frontier_exploration.grid_resolution_m, 0.02);
        const double known_free_tolerance =
            std::max(1.8 * global_map_resolution, 0.055);
        bool reference_is_safe = true;
        for (const ReferenceWaypoint& waypoint : reference_trajectory_) {
            const auto corners = make_box_corners(
                waypoint.position,
                waypoint.yaw,
                geometry_.body_length + 2.0 * footprint_margin_m,
                geometry_.body_width + 2.0 * footprint_margin_m);
            if (std::any_of(
                    corners.begin(),
                    corners.end(),
                    [&](const Vec2& corner) {
                        return !is_inside_bounds(world_, corner);
                    })) {
                reference_is_safe = false;
                break;
            }

            if (!global_free_points_.empty()) {
                const bool waypoint_is_known_free = std::any_of(
                    global_free_points_.begin(),
                    global_free_points_.end(),
                    [&](const Vec2& free_cell) {
                        return distance(free_cell, waypoint.position) <=
                               known_free_tolerance;
                    });
                if (!waypoint_is_known_free) {
                    reference_is_safe = false;
                    break;
                }
            }

            const double cos_yaw = std::cos(waypoint.yaw);
            const double sin_yaw = std::sin(waypoint.yaw);
            for (const Vec2& occupied : global_occupied_points_) {
                const double dx = occupied.x - waypoint.position.x;
                const double dy = occupied.y - waypoint.position.y;
                const double longitudinal = cos_yaw * dx + sin_yaw * dy;
                const double lateral = -sin_yaw * dx + cos_yaw * dy;
                if (std::abs(longitudinal) <= half_length &&
                    std::abs(lateral) <= half_width) {
                    reference_is_safe = false;
                    break;
                }
            }
            if (!reference_is_safe) {
                break;
            }
            for (const LidarHit& hit : lidar_hits_) {
                if (!hit.hit ||
                    !(hit.distance > 0.0) ||
                    hit.distance >= config_.localization.max_range_m - 0.03) {
                    continue;
                }
                const double dx = hit.point.x - waypoint.position.x;
                const double dy = hit.point.y - waypoint.position.y;
                const double longitudinal = cos_yaw * dx + sin_yaw * dy;
                const double lateral = -sin_yaw * dx + cos_yaw * dy;
                if (std::abs(longitudinal) <= half_length &&
                    std::abs(lateral) <= half_width) {
                    reference_is_safe = false;
                    break;
                }
            }
            if (!reference_is_safe) {
                break;
            }
        }

        if (!reference_is_safe) {
            // Return to perception/search without executing even the first
            // sample of an unsafe reference. Frontier targets are blacklisted
            // and replanned; physical gates return to stationary perception
            // before they can be reconsidered.
            if (navigation_target_kind_ == NavigationTargetKind::ExplorationFrontier) {
                if (selected_frontier_.has_value()) {
                    visited_frontiers_.push_back(*selected_frontier_);
                }
                clear_exploration_target("frontier_clothoid_failed_footprint_validation");
                last_frontier_plan_step_ = -1;
            } else {
                restart_unstructured_scan();
            }
            diagnostics_.planner_has_reference = false;
            return;
        }

        // This local reference is rebuilt from the robot's current projection
        // every cycle. An anchor index inherited from the previous reference
        // points near the old endpoint and makes the MPC skip the new clothoid.
        structured_anchor_hint_index_ = 0;
    }
    diagnostics_.planner_has_reference = reference_trajectory_.size() >= 2;
    if (!diagnostics_.planner_has_reference) {
        assign_structured_centerline_fallback();
    }
}

}  // namespace thesis_sim
