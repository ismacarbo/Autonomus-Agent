bool HardwarePlannerRunner::startup_scan_active() const {
    return use_dynamic_gap_gates_ &&
           world_.environment_mode() == EnvironmentMode::UnstructuredGates &&
           config_.gap_extraction.startup_scan_duration_s > 1e-3 &&
           !goal_reached_ &&
           !locked_gap_goal_.has_value() &&
           !startup_scan_complete_;
}

void HardwarePlannerRunner::clear_locked_gap_goal() {
    locked_gap_goal_.reset();
    locked_gap_start_position_ = {};
    locked_gap_approach_direction_ = {1.0, 0.0};
    locked_gap_corridor_half_width_m_ = 0.0;
    locked_gap_set_time_s_ = 0.0;
    locked_gap_crossed_ = false;
    locked_gap_invalid_streak_ = 0;
}

void HardwarePlannerRunner::restart_unstructured_scan() {
    clear_locked_gap_goal();
    startup_scan_elapsed_s_ = 0.0;
    unstructured_no_candidate_scan_elapsed_s_ = 0.0;
    update_unstructured_scan_direction(false);
    startup_scan_complete_ = false;
    gap_recovery_turn_active_ = false;
    gates_.clear();
    gate_specs_.clear();
    visible_gate_indices_.clear();
    chosen_gate_index_ = -1;
    reference_trajectory_.clear();
    planned_trajectory_.clear();
    diagnostics_.candidate_gates = 0;
    diagnostics_.chosen_gate_distance = std::numeric_limits<double>::infinity();
    diagnostics_.planner_has_reference = false;
}

void HardwarePlannerRunner::update_unstructured_scan_direction(bool flip_when_aligned) {
    const Vec2 goal = world_.goal();
    if (distance(estimate_.position, goal) > 0.05) {
        const double heading_error = wrap_angle(angle_to(estimate_.position, goal) - estimate_.yaw);
        if (std::abs(heading_error) > 0.12) {
            startup_scan_direction_ = heading_error >= 0.0 ? 1.0 : -1.0;
            return;
        }
    }

    if (flip_when_aligned) {
        startup_scan_direction_ = startup_scan_direction_ >= 0.0 ? -1.0 : 1.0;
    } else if (std::abs(startup_scan_direction_) < 0.5) {
        startup_scan_direction_ = 1.0;
    }
}

void HardwarePlannerRunner::set_locked_gap_goal(const Vec2& target) {
    if (compact_hardware_mixed_lab_world(world_)) {
        const double progress_target =
            structured_goal_progress_target_ > 0.0
                ? structured_goal_progress_target_
                : structured_road_length_m(world_);
        const double finish_suppression =
            clamp_value(0.16 * std::max(progress_target, 0.0), 0.065, 0.120);
        const bool near_structured_finish =
            progress_target > 0.0 &&
            progress_target - structured_progress_s_ <= finish_suppression;
        const double goal_position_finish_radius =
            clamp_value(0.18 * world_span_m(world_), 0.075, 0.105);
        const bool near_goal_position =
            distance(estimate_.position, world_.goal()) <= goal_position_finish_radius;
        const bool front_dynamic_pressure =
            lidar_pressure_from_hits(
                world_,
                lidar_hits_,
                estimate_.yaw,
                config_.localization.obstacle_stop_distance_m + 0.07,
                config_.localization.front_sector_half_angle_rad,
                true);
        if (near_goal_position ||
            (near_structured_finish && !front_dynamic_pressure)) {
            clear_locked_gap_goal();
            return;
        }
    }

    locked_gap_goal_ = target;
    locked_gap_start_position_ = estimate_.position;
    locked_gap_set_time_s_ = sim_time_;

    const Vec2 fallback_direction{std::cos(estimate_.yaw), std::sin(estimate_.yaw)};
    const Vec2 raw_direction{
        target.x - estimate_.position.x,
        target.y - estimate_.position.y,
    };
    locked_gap_approach_direction_ = normalize_with_fallback(raw_direction, fallback_direction);

    const double usable_gap_width = std::max(
        config_.gap_extraction.min_gap_width_m - geometry_.body_width,
        config_.gap_extraction.target_clearance_radius_m);
    locked_gap_corridor_half_width_m_ = clamp_value(
        0.5 * usable_gap_width,
        0.05,
        std::max(0.5 * config_.gap_extraction.min_gap_width_m, 0.05));
    locked_gap_crossed_ = false;
    locked_gap_invalid_streak_ = 0;
}

double HardwarePlannerRunner::locked_gap_longitudinal_progress(const Vec2& position) const {
    if (!locked_gap_goal_.has_value()) {
        return 0.0;
    }

    const Vec2 relative{
        position.x - locked_gap_goal_->x,
        position.y - locked_gap_goal_->y,
    };
    return dot_product(relative, locked_gap_approach_direction_);
}

double HardwarePlannerRunner::locked_gap_lateral_offset(const Vec2& position) const {
    if (!locked_gap_goal_.has_value()) {
        return 0.0;
    }

    const Vec2 relative{
        position.x - locked_gap_goal_->x,
        position.y - locked_gap_goal_->y,
    };
    const Vec2 lateral_axis{
        -locked_gap_approach_direction_.y,
        locked_gap_approach_direction_.x,
    };
    return dot_product(relative, lateral_axis);
}

void HardwarePlannerRunner::publish_locked_gap_goal() {
    if (!locked_gap_goal_.has_value()) {
        gates_.clear();
        gate_specs_.clear();
        chosen_gate_index_ = -1;
        diagnostics_.candidate_gates = 0;
        diagnostics_.chosen_gate_distance = std::numeric_limits<double>::infinity();
        return;
    }

    const Vec2 target = *locked_gap_goal_;
    GateSpec spec{};
    spec.name = "locked_gap_goal";
    spec.position = target;
    spec.anchor_position = target;
    spec.motion_amplitude = {0.0, 0.0};
    spec.motion_frequency_hz = 0.0;
    spec.motion_phase_rad = 0.0;
    spec.heading_hint = std::atan2(locked_gap_approach_direction_.y, locked_gap_approach_direction_.x);
    spec.final = world_.environment_mode() == EnvironmentMode::UnstructuredGates;

    gate_specs_.assign(1, spec);

    gate g{};
    g.x_pos = target.x;
    g.y_pos = target.y;
    g.road = cl_;
    g.road.PSI_end = spec.heading_hint;
    g.passed = locked_gap_crossed_;
    g.choose = !locked_gap_crossed_;
    g.too_far = false;
    g.final = spec.final;
    gates_.assign(1, g);

    chosen_gate_index_ = 0;
    diagnostics_.candidate_gates = 1;
    diagnostics_.chosen_gate_distance = distance(target, estimate_.position);
}

void HardwarePlannerRunner::count_mixed_gate_crossing_if_needed() {
    if (!world_is_mixed(world_) || !locked_gap_goal_.has_value()) {
        return;
    }
    if (compact_mixed_open_road(world_) &&
        passed_unstructured_gap_count_ >= required_unstructured_gap_pass_count()) {
        return;
    }
    const bool compact_mixed_world =
        std::max(world_.bounds().max_x - world_.bounds().min_x,
                 world_.bounds().max_y - world_.bounds().min_y) <= 2.50;
    const bool compact_rejoin_phase =
        compact_mixed_world &&
        passed_unstructured_gap_count_ > 0 &&
        (passed_unstructured_gap_count_ % 2) == 1;

    const double longitudinal_progress = locked_gap_longitudinal_progress(estimate_.position);
    const double crossing_progress =
        longitudinal_progress + std::max(geometry_.cg_to_front, 0.0);
    const double lateral_offset = std::abs(locked_gap_lateral_offset(estimate_.position));
    const double crossing_margin =
        std::max(config_.gap_extraction.gap_crossing_margin_m, 0.0);
    const double pass_lateral_limit =
        locked_gap_corridor_half_width_m_ +
        std::max(config_.gap_extraction.gap_goal_acceptance_lateral_slack_m, 0.0);
    const double crossing_distance = std::max(crossing_margin - crossing_progress, 0.0);
    const double near_pass_longitudinal_slack = clamp_value(
        0.46 * std::max(config_.gap_extraction.gap_goal_tolerance_m, 0.0),
        0.045,
        0.085);
    const double gate_distance = distance(estimate_.position, *locked_gap_goal_);
    const bool micro_hardware_lab = compact_hardware_mixed_lab_world(world_);
    const double near_pass_gate_radius = micro_hardware_lab
        ? clamp_value(
              1.25 * std::max(config_.gap_extraction.gap_goal_tolerance_m, 0.0),
              0.045,
              std::max(config_.gap_extraction.gap_goal_acceptance_radius_m, 0.055))
        : clamp_value(
              1.25 * std::max(config_.gap_extraction.gap_goal_tolerance_m, 0.0),
              0.18,
              std::max(std::min(config_.gap_extraction.gap_goal_acceptance_radius_m, 0.24), 0.18));
    const double compact_gate_radius = micro_hardware_lab
        ? clamp_value(compact_mixed_gate_validation_radius_m(world_), 0.045, 0.070)
        : clamp_value(
              compact_mixed_gate_validation_radius_m(world_),
              0.10,
              0.14);
    const bool compact_gate_reached =
        compact_mixed_world &&
        gate_distance <= compact_gate_radius;
    const bool gate_crossed =
        compact_gate_reached ||
        (lateral_offset <= pass_lateral_limit &&
         (crossing_progress >= crossing_margin ||
          (crossing_distance <= near_pass_longitudinal_slack &&
           gate_distance <= near_pass_gate_radius)));
    if (!gate_crossed) {
        return;
    }
    if (compact_mixed_world) {
        const double active_time = std::max(0.0, sim_time_ - locked_gap_set_time_s_);
        const double travel_since_lock = distance(estimate_.position, locked_gap_start_position_);
        const bool lab_scale_mixed_world = world_span_m(world_) <= 1.25;
        const double min_active_time = lab_scale_mixed_world ? 0.45 : 0.25;
        const double min_lock_travel = lab_scale_mixed_world ? 0.035 : 0.07;
        if (active_time < min_active_time || travel_since_lock < min_lock_travel) {
            return;
        }
    }

    // On the compact open-road validation map, reaching the visual gate is not
    // enough: the rear of the robot must have cleared the obstacle before road
    // tracking is allowed to resume. This prevents an early switch back to the
    // centerline while the footprint is still alongside the blocking object.
    if (compact_mixed_open_road(world_) &&
        passed_unstructured_gap_count_ == 0 &&
        !world_.obstacles().empty()) {
        Vec2 road_axis{
            world_.goal().x - world_.start().x,
            world_.goal().y - world_.start().y,
        };
        const double axis_norm = vector_norm(road_axis);
        if (axis_norm > 1e-6) {
            road_axis.x /= axis_norm;
            road_axis.y /= axis_norm;
            double required_center_progress =
                -std::numeric_limits<double>::infinity();
            bool cleared_obstacles = true;
            for (const Rect& obstacle : world_.obstacles()) {
                const std::array<Vec2, 4> corners{{
                    {obstacle.min_x, obstacle.min_y},
                    {obstacle.max_x, obstacle.min_y},
                    {obstacle.max_x, obstacle.max_y},
                    {obstacle.min_x, obstacle.max_y},
                }};
                double farthest_obstacle_progress =
                    -std::numeric_limits<double>::infinity();
                for (const Vec2& corner : corners) {
                    const double corner_progress = dot_product(
                        Vec2{corner.x - world_.start().x,
                             corner.y - world_.start().y},
                        road_axis);
                    required_center_progress = std::max(
                        required_center_progress,
                        corner_progress + 0.5 * geometry_.body_length + 0.02);
                    farthest_obstacle_progress = std::max(
                        farthest_obstacle_progress,
                        dot_product(
                            Vec2{corner.x - estimate_.position.x,
                                 corner.y - estimate_.position.y},
                            road_axis));
                }
                if (farthest_obstacle_progress > -0.5 * geometry_.body_length) {
                    cleared_obstacles = false;
                }
            }
            if (!cleared_obstacles) {
                const double locked_progress = dot_product(
                    Vec2{locked_gap_goal_->x - world_.start().x,
                         locked_gap_goal_->y - world_.start().y},
                    road_axis);
                const double advance = required_center_progress - locked_progress;
                if (advance > 0.015) {
                    Vec2 staged_target{
                        locked_gap_goal_->x + advance * road_axis.x,
                        locked_gap_goal_->y + advance * road_axis.y,
                    };
                    staged_target = clamp_point_to_bounds(world_, staged_target, 0.10);
                    if (dynamic_gap_point_allowed(staged_target)) {
                        set_locked_gap_goal(staged_target);
                        publish_locked_gap_goal();
                    }
                }
                return;
            }
        }
    }

    double road_lateral_offset = std::numeric_limits<double>::infinity();
    double road_s = 0.0;
    double road_length = 0.0;
    Vec2 road_tangent{1.0, 0.0};
    bool have_road_projection = false;
    const std::vector<Vec2>& road = world_.road_centerline();
    std::vector<double> road_cumulative(road.size(), 0.0);
    for (size_t i = 1; i < road.size(); ++i) {
        road_cumulative[i] = road_cumulative[i - 1] + distance(road[i - 1], road[i]);
    }
    const bool closed_road_loop = structured_road_is_closed_loop(world_);
    if (!road_cumulative.empty()) {
        road_length = road_cumulative.back();
    }
    auto point_at_road_s = [&](double s) {
        if (road.empty()) {
            return estimate_.position;
        }
        if (road.size() == 1 || road_cumulative.empty()) {
            return road.front();
        }
        s = closed_road_loop ? wrap_arc_length(s, road_cumulative.back())
                             : clamp_value(s, 0.0, road_cumulative.back());
        auto upper = std::upper_bound(road_cumulative.begin(), road_cumulative.end(), s);
        if (upper == road_cumulative.begin()) {
            return road.front();
        }
        if (upper == road_cumulative.end()) {
            return road.back();
        }
        const size_t index = static_cast<size_t>(std::distance(road_cumulative.begin(), upper));
        const double s0 = road_cumulative[index - 1];
        const double s1 = road_cumulative[index];
        const double alpha = (s1 > s0) ? clamp_value((s - s0) / (s1 - s0), 0.0, 1.0) : 0.0;
        return Vec2{
            road[index - 1].x + (road[index].x - road[index - 1].x) * alpha,
            road[index - 1].y + (road[index].y - road[index - 1].y) * alpha,
        };
    };
    double best_road_projection_cost = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i + 1 < road.size(); ++i) {
        const Vec2 segment{
            road[i + 1].x - road[i].x,
            road[i + 1].y - road[i].y,
        };
        const double segment_len_sq = dot_product(segment, segment);
        if (!(segment_len_sq > 1e-12)) {
            continue;
        }
        const Vec2 from_start{
            estimate_.position.x - road[i].x,
            estimate_.position.y - road[i].y,
        };
        const double t = clamp_value(dot_product(from_start, segment) / segment_len_sq, 0.0, 1.0);
        const Vec2 projection{
            road[i].x + segment.x * t,
            road[i].y + segment.y * t,
        };
        const double segment_len = std::sqrt(segment_len_sq);
        const Vec2 tangent{segment.x / segment_len, segment.y / segment_len};
        const Vec2 normal{-tangent.y, tangent.x};
        const double lateral_offset = dot_product(
            Vec2{estimate_.position.x - projection.x, estimate_.position.y - projection.y},
            normal);
        double projection_cost = lateral_offset * lateral_offset;
        if (compact_mixed_world &&
            closed_road_loop &&
            road_length > 1e-6 &&
            (std::isfinite(structured_last_s_) || structured_progress_s_ > 0.0)) {
            const double hint_s =
                std::isfinite(structured_last_s_)
                    ? structured_last_s_
                    : wrap_arc_length(structured_progress_s_, road_length);
            const double candidate_s = road_cumulative[i] + segment_len * t;
            const double arc_delta =
                std::abs(signed_wrapped_arc_delta(hint_s, candidate_s, road_length));
            const double continuity_weight = 0.18;
            projection_cost += continuity_weight * continuity_weight * arc_delta * arc_delta;
        }
        if (projection_cost < best_road_projection_cost) {
            road_lateral_offset = lateral_offset;
            road_s = road_cumulative[i] + segment_len * t;
            road_tangent = tangent;
            have_road_projection = true;
            best_road_projection_cost = projection_cost;
        }
    }
    if (std::isfinite(road_lateral_offset)) {
        const double road_exit_offset =
            compact_mixed_world ? 0.055 : std::max(0.20, 0.5 * geometry_.body_width + 0.08);
        const double road_rejoin_offset =
            compact_mixed_world ? 0.10 : std::max(0.13, 0.5 * geometry_.body_width + 0.02);
        if (!compact_gate_reached &&
            !compact_rejoin_phase &&
            std::abs(road_lateral_offset) < road_exit_offset) {
            return;
        }
        if (!compact_gate_reached &&
            compact_rejoin_phase &&
            std::abs(road_lateral_offset) > road_rejoin_offset) {
            return;
        }
    }
    if (!compact_mixed_world &&
        passed_unstructured_gap_count_ == 0 &&
        have_road_projection &&
        road_length > 1e-6 &&
        compute_mixed_road_block_score(0.75) > 0.35) {
        const Vec2 normal{-road_tangent.y, road_tangent.x};
        const Vec2 locked_target_delta{
            locked_gap_goal_->x - point_at_road_s(road_s).x,
            locked_gap_goal_->y - point_at_road_s(road_s).y,
        };
        const double locked_target_side = dot_product(locked_target_delta, normal);
        const double side = locked_target_side < 0.0 ? -1.0 : 1.0;
        const Vec2 road_target = point_at_road_s(road_s + 0.56);
        Vec2 next_bypass_target{
            road_target.x + side * 0.46 * normal.x,
            road_target.y + side * 0.46 * normal.y,
        };
        next_bypass_target = clamp_point_to_bounds(world_, next_bypass_target, 0.18);
        if (dynamic_gap_point_allowed(next_bypass_target)) {
            set_locked_gap_goal(next_bypass_target);
            publish_locked_gap_goal();
        }
        return;
    }

    passed_unstructured_gap_positions_.push_back(*locked_gap_goal_);
    ++passed_unstructured_gap_count_;
    clear_locked_gap_goal();
    gates_.clear();
    gate_specs_.clear();
    visible_gate_indices_.clear();
    chosen_gate_index_ = -1;
    diagnostics_.candidate_gates = 0;
    diagnostics_.chosen_gate_distance = std::numeric_limits<double>::infinity();
    startup_scan_complete_ = true;
    startup_scan_elapsed_s_ = config_.gap_extraction.startup_scan_duration_s;
}

void HardwarePlannerRunner::update_unstructured_gap_workflow(double dt) {
    if (!use_dynamic_gap_gates_) {
        clear_locked_gap_goal();
        startup_scan_elapsed_s_ = 0.0;
        unstructured_no_candidate_scan_elapsed_s_ = 0.0;
        startup_scan_direction_ = 1.0;
        startup_scan_complete_ = false;
        return;
    }
    if (world_.environment_mode() == EnvironmentMode::UnstructuredGates &&
        passed_unstructured_gap_count_ >= required_unstructured_gap_pass_count()) {
        clear_locked_gap_goal();
        gates_.clear();
        gate_specs_.clear();
        visible_gate_indices_.clear();
        chosen_gate_index_ = -1;
        diagnostics_.candidate_gates = 0;
        diagnostics_.chosen_gate_distance = std::numeric_limits<double>::infinity();
        startup_scan_complete_ = true;
        unstructured_no_candidate_scan_elapsed_s_ = 0.0;
        return;
    }
    const bool compact_mixed_world =
        world_is_mixed(world_) &&
        std::max(world_.bounds().max_x - world_.bounds().min_x,
                 world_.bounds().max_y - world_.bounds().min_y) <= 2.50;
    const bool mixed_mode = world_is_mixed(world_);
    if (mixed_mode) {
        startup_scan_complete_ = true;
        startup_scan_elapsed_s_ = config_.gap_extraction.startup_scan_duration_s;
        if (compact_mixed_open_road(world_) &&
            passed_unstructured_gap_count_ >= required_unstructured_gap_pass_count()) {
            clear_locked_gap_goal();
            gates_.clear();
            gate_specs_.clear();
            dynamic_gap_tracks_.clear();
            next_dynamic_gap_track_id_ = 1;
            visible_gate_indices_.clear();
            chosen_gate_index_ = -1;
            diagnostics_.candidate_gates = 0;
            diagnostics_.chosen_gate_distance = std::numeric_limits<double>::infinity();
            unstructured_no_candidate_scan_elapsed_s_ = 0.0;
            return;
        }
        if (!locked_gap_goal_.has_value()) {
            const double road_block_lookahead = mixed_road_gate_lookahead_m();
            const double road_block_score = compute_mixed_road_block_score(road_block_lookahead);

            struct RoadProjection {
                bool valid = false;
                double s = 0.0;
                double length = 0.0;
                Vec2 point{};
                Vec2 tangent{1.0, 0.0};
                double lateral_offset = 0.0;
            };
            std::vector<double> road_cumulative;
            const std::vector<Vec2>& road = world_.road_centerline();
            road_cumulative.resize(road.size(), 0.0);
            for (size_t i = 1; i < road.size(); ++i) {
                road_cumulative[i] = road_cumulative[i - 1] + distance(road[i - 1], road[i]);
            }
            const bool closed_road_loop = structured_road_is_closed_loop(world_);
            auto point_at_road_s = [&](double s) {
                if (road.empty()) {
                    return estimate_.position;
                }
                if (road.size() == 1 || road_cumulative.empty()) {
                    return road.front();
                }
                const double road_length = road_cumulative.back();
                s = closed_road_loop ? wrap_arc_length(s, road_length)
                                     : clamp_value(s, 0.0, road_length);
                auto upper = std::upper_bound(road_cumulative.begin(), road_cumulative.end(), s);
                if (upper == road_cumulative.begin()) {
                    return road.front();
                }
                if (upper == road_cumulative.end()) {
                    return road.back();
                }
                const size_t index = static_cast<size_t>(std::distance(road_cumulative.begin(), upper));
                const double s0 = road_cumulative[index - 1];
                const double s1 = road_cumulative[index];
                const double alpha = (s1 > s0) ? clamp_value((s - s0) / (s1 - s0), 0.0, 1.0) : 0.0;
                return Vec2{
                    road[index - 1].x + (road[index].x - road[index - 1].x) * alpha,
                    road[index - 1].y + (road[index].y - road[index - 1].y) * alpha,
                };
            };
            auto project_on_road = [&]() {
                RoadProjection projection{};
                if (road.size() < 2 || road_cumulative.empty() || !(road_cumulative.back() > 1e-6)) {
                    return projection;
                }

                double best_projection_cost = std::numeric_limits<double>::infinity();
                for (size_t i = 0; i + 1 < road.size(); ++i) {
                    const Vec2 segment{
                        road[i + 1].x - road[i].x,
                        road[i + 1].y - road[i].y,
                    };
                    const double segment_len_sq = dot_product(segment, segment);
                    if (!(segment_len_sq > 1e-12)) {
                        continue;
                    }
                    const Vec2 from_start{
                        estimate_.position.x - road[i].x,
                        estimate_.position.y - road[i].y,
                    };
                    const double t = clamp_value(dot_product(from_start, segment) / segment_len_sq, 0.0, 1.0);
                    const Vec2 point{
                        road[i].x + segment.x * t,
                        road[i].y + segment.y * t,
                    };
                    const double d_sq = distance_sq(estimate_.position, point);
                    const double segment_len = std::sqrt(segment_len_sq);
                    const double candidate_s = road_cumulative[i] + segment_len * t;
                    double projection_cost = d_sq;
                    if (compact_mixed_world &&
                        closed_road_loop &&
                        road_cumulative.back() > 1e-6 &&
                        (std::isfinite(structured_last_s_) || structured_progress_s_ > 0.0)) {
                        const double hint_s =
                            std::isfinite(structured_last_s_)
                                ? structured_last_s_
                                : wrap_arc_length(structured_progress_s_, road_cumulative.back());
                        const double arc_delta = std::abs(
                            signed_wrapped_arc_delta(hint_s, candidate_s, road_cumulative.back()));
                        const double continuity_weight = 0.18;
                        projection_cost += continuity_weight * continuity_weight * arc_delta * arc_delta;
                    }
                    if (projection_cost < best_projection_cost) {
                        projection.valid = true;
                        projection.s = candidate_s;
                        projection.length = road_cumulative.back();
                        projection.point = point;
                        projection.tangent = {segment.x / segment_len, segment.y / segment_len};
                        const Vec2 normal{-projection.tangent.y, projection.tangent.x};
                        projection.lateral_offset = dot_product(
                            Vec2{estimate_.position.x - point.x, estimate_.position.y - point.y},
                            normal);
                        best_projection_cost = projection_cost;
                    }
                }
                return projection;
            };

            const RoadProjection road_projection = project_on_road();
            const bool compact_mixed_world =
                std::max(world_.bounds().max_x - world_.bounds().min_x,
                         world_.bounds().max_y - world_.bounds().min_y) <= 2.50;
            const bool rejoin_stage =
                compact_mixed_world &&
                passed_unstructured_gap_count_ > 0 &&
                (passed_unstructured_gap_count_ % 2) == 1 &&
                road_projection.valid &&
                std::abs(road_projection.lateral_offset) > 0.09;
            const bool lab_scale_mixed_world =
                compact_mixed_world &&
                std::max(world_.bounds().max_x - world_.bounds().min_x,
                         world_.bounds().max_y - world_.bounds().min_y) <= 1.25;
            const bool micro_hardware_lab =
                compact_hardware_mixed_lab_world(world_);
            const double compact_min_road_progress =
                compact_mixed_world
                    ? clamp_value(0.08 * std::max(structured_goal_progress_target_, 0.0),
                                  lab_scale_mixed_world ? 0.045 : 0.14,
                                  lab_scale_mixed_world ? 0.080 : 0.22)
                    : 0.0;
            const bool compact_road_has_started =
                !compact_mixed_world ||
                structured_progress_s_ >= compact_min_road_progress;
            const double micro_goal_progress_target =
                structured_goal_progress_target_ > 0.0
                    ? structured_goal_progress_target_
                    : structured_road_length_m(world_);
            const double micro_finish_gate_suppression_m =
                micro_hardware_lab
                    ? clamp_value(0.16 * std::max(micro_goal_progress_target, 0.0),
                                  0.065,
                                  0.120)
                    : 0.0;
            const bool micro_near_structured_finish =
                micro_hardware_lab &&
                micro_goal_progress_target > 0.0 &&
                micro_goal_progress_target - structured_progress_s_ <=
                    micro_finish_gate_suppression_m;
            if (micro_hardware_lab &&
                !rejoin_stage &&
                (!compact_road_has_started || micro_near_structured_finish)) {
                gates_.clear();
                gate_specs_.clear();
                clear_locked_gap_goal();
                dynamic_gap_tracks_.clear();
                next_dynamic_gap_track_id_ = 1;
                visible_gate_indices_.clear();
                chosen_gate_index_ = -1;
                diagnostics_.candidate_gates = 0;
                diagnostics_.chosen_gate_distance = std::numeric_limits<double>::infinity();
                return;
            }
            const bool compact_dynamic_obstacle_required =
                compact_mixed_world && (world_.obstacles().empty() || lab_scale_mixed_world);
            const bool have_close_dynamic_obstacle = micro_hardware_lab
                ? lidar_pressure_from_hits(
                      world_,
                      lidar_hits_,
                      estimate_.yaw,
                      config_.localization.obstacle_stop_distance_m + 0.06,
                      config_.localization.front_sector_half_angle_rad,
                      false)
                : (diagnostics_.front_close_lidar_points > 0 ||
                   diagnostics_.close_lidar_points > 0 ||
                   (estimate_.front_lidar_distance > 0.0 &&
                    estimate_.front_lidar_distance < config_.localization.obstacle_stop_distance_m) ||
                   (estimate_.min_lidar_distance > 0.0 &&
                    estimate_.min_lidar_distance < config_.localization.obstacle_stop_distance_m));
            bool compact_front_obstacle = false;
            if (compact_mixed_world) {
                const double front_pressure_distance =
                    config_.localization.obstacle_stop_distance_m +
                    (micro_hardware_lab ? 0.065 : 0.015);
                if (micro_hardware_lab) {
                    compact_front_obstacle =
                        lidar_pressure_from_hits(
                            world_,
                            lidar_hits_,
                            estimate_.yaw,
                            front_pressure_distance,
                            config_.localization.front_sector_half_angle_rad,
                            true);
                } else {
                    for (const LidarHit& hit : lidar_hits_) {
                        if (!hit.hit ||
                            !(hit.distance > 0.0) ||
                            hit.distance > front_pressure_distance) {
                            continue;
                        }
                        const double local_angle = std::abs(wrap_angle(hit.angle - estimate_.yaw));
                        if (local_angle <= config_.localization.front_sector_half_angle_rad) {
                            compact_front_obstacle = true;
                            break;
                        }
                    }
                }
            }
            const bool front_obstacle_pressure =
                compact_mixed_world
                    ? compact_front_obstacle
                    : (diagnostics_.front_close_lidar_points > 0 ||
                       (diagnostics_.close_lidar_points > 0 &&
                        estimate_.front_lidar_distance > 0.0 &&
                        estimate_.front_lidar_distance <
                            config_.localization.obstacle_stop_distance_m + 0.015));
            const bool tracked_lab_scale_mixed =
                config_.vehicle_model == VehicleModelKind::TrackedVehicle &&
                lab_scale_mixed_world;
            const bool tracked_close_obstacle_pressure =
                tracked_lab_scale_mixed &&
                estimate_.min_lidar_distance > 0.0 &&
                estimate_.min_lidar_distance <
                    config_.localization.obstacle_stop_distance_m + 0.060;
            const bool compact_close_obstacle_recovery =
                compact_mixed_world &&
                have_close_dynamic_obstacle &&
                (micro_hardware_lab
                     ? (front_obstacle_pressure &&
                        ((estimate_.front_lidar_distance > 0.0 &&
                          estimate_.front_lidar_distance <
                              config_.localization.obstacle_stop_distance_m + 0.060) ||
                         no_motion_command_cycles_ >= 4 ||
                         tracker_heading_error_deg_ >= 70.0))
                     : (tracked_close_obstacle_pressure ||
                        (estimate_.front_lidar_distance > 0.0 &&
                         estimate_.front_lidar_distance <
                             config_.localization.obstacle_stop_distance_m + 0.025) ||
                        no_motion_command_cycles_ >= 4 ||
                        tracker_heading_error_deg_ >= 70.0));
            const bool road_needs_bypass =
                road_block_score >= 0.12 ||
                (compact_mixed_world &&
                 (front_obstacle_pressure || compact_close_obstacle_recovery));
            auto lock_existing_dynamic_candidate = [&]() {
                if (gate_specs_.empty()) {
                    return false;
                }
                int best_index = -1;
                double best_score = -std::numeric_limits<double>::infinity();
                for (size_t i = 0; i < gate_specs_.size(); ++i) {
                    const Vec2& target = gate_specs_[i].position;
                    if (!dynamic_gap_point_allowed(target)) {
                        continue;
                    }
                    const double target_distance = distance(estimate_.position, target);
                    const double max_lock_distance =
                        std::max(config_.gap_extraction.max_target_distance_m * 1.25, 0.55);
                    if (target_distance < 0.14 || target_distance > max_lock_distance) {
                        continue;
                    }
                    const double heading_error =
                        std::abs(wrap_angle(angle_to(estimate_.position, target) - estimate_.yaw));
                    if (heading_error > 92.0 * kPi / 180.0) {
                        continue;
                    }
                    double road_proximity_score = 0.0;
                    if (road_projection.valid) {
                        const double road_target_distance = distance(target, road_projection.point);
                        if (road_target_distance > 0.78) {
                            continue;
                        }
                        road_proximity_score =
                            0.45 * (1.0 - clamp_value(road_target_distance / 0.78, 0.0, 1.0));
                    }
                    const double heading_score =
                        clamp_value(std::cos(heading_error), 0.0, 1.0);
                    const double distance_score =
                        1.0 - clamp_value(target_distance / max_lock_distance, 0.0, 1.0);
                    const double score =
                        1.30 * heading_score +
                        0.70 * distance_score +
                        road_proximity_score -
                        0.05 * static_cast<double>(i);
                    if (score > best_score) {
                        best_score = score;
                        best_index = static_cast<int>(i);
                    }
                }
                if (best_index < 0) {
                    return false;
                }
                set_locked_gap_goal(gate_specs_[static_cast<size_t>(best_index)].position);
                publish_locked_gap_goal();
                return true;
            };
            if (compact_mixed_world &&
                passed_unstructured_gap_count_ > 0 &&
                (passed_unstructured_gap_count_ % 2) == 1 &&
                road_projection.valid &&
                std::abs(road_projection.lateral_offset) <= 0.065) {
                const double rejoin_clear_lookahead =
                    lab_scale_mixed_world ? mixed_road_gate_lookahead_m() : 0.45;
                const bool road_front_clear =
                    estimate_.front_lidar_distance <= 0.0 ||
                    estimate_.front_lidar_distance >=
                        config_.localization.obstacle_stop_distance_m + 0.12 ||
                    compute_mixed_road_block_score(rejoin_clear_lookahead) < 0.35;
                if (road_front_clear) {
                    ++passed_unstructured_gap_count_;
                    clear_locked_gap_goal();
                    gates_.clear();
                    gate_specs_.clear();
                    dynamic_gap_tracks_.clear();
                    next_dynamic_gap_track_id_ = 1;
                    visible_gate_indices_.clear();
                    chosen_gate_index_ = -1;
                    diagnostics_.candidate_gates = 0;
                    diagnostics_.chosen_gate_distance = std::numeric_limits<double>::infinity();
                    startup_scan_complete_ = true;
                    unstructured_no_candidate_scan_elapsed_s_ = 0.0;
                    return;
                }
            }

            const bool compact_bypass_sensor_ready =
                lab_scale_mixed_world
                    ? (front_obstacle_pressure || compact_close_obstacle_recovery)
                    : (have_close_dynamic_obstacle || front_obstacle_pressure);
            const bool lab_scale_gate_priority_ready =
                !compact_mixed_world ||
                rejoin_stage ||
                front_obstacle_pressure ||
                compact_close_obstacle_recovery ||
                (compact_road_has_started && road_block_score >= 0.45);
            if (!lab_scale_gate_priority_ready) {
                gates_.clear();
                gate_specs_.clear();
                dynamic_gap_tracks_.clear();
                next_dynamic_gap_track_id_ = 1;
                visible_gate_indices_.clear();
                chosen_gate_index_ = -1;
                diagnostics_.candidate_gates = 0;
                diagnostics_.chosen_gate_distance = std::numeric_limits<double>::infinity();
                return;
            }
            if (!rejoin_stage && !road_needs_bypass) {
                gates_.clear();
                gate_specs_.clear();
                dynamic_gap_tracks_.clear();
                next_dynamic_gap_track_id_ = 1;
                visible_gate_indices_.clear();
                chosen_gate_index_ = -1;
                diagnostics_.candidate_gates = 0;
                diagnostics_.chosen_gate_distance = std::numeric_limits<double>::infinity();
                return;
            }
            if (!rejoin_stage &&
                compact_dynamic_obstacle_required &&
                !compact_bypass_sensor_ready) {
                gates_.clear();
                gate_specs_.clear();
                dynamic_gap_tracks_.clear();
                next_dynamic_gap_track_id_ = 1;
                visible_gate_indices_.clear();
                chosen_gate_index_ = -1;
                diagnostics_.candidate_gates = 0;
                diagnostics_.chosen_gate_distance = std::numeric_limits<double>::infinity();
                return;
            }
            if (!rejoin_stage &&
                (front_obstacle_pressure || compact_close_obstacle_recovery) &&
                lock_existing_dynamic_candidate()) {
                return;
            }
            {
                const double left_clearance = sector_min_clearance(
                    lidar_hits_,
                    estimate_.yaw,
                    42.0 * kPi / 180.0,
                    24.0 * kPi / 180.0,
                    config_.localization.max_range_m);
                const double right_clearance = sector_min_clearance(
                    lidar_hits_,
                    estimate_.yaw,
                    -42.0 * kPi / 180.0,
                    24.0 * kPi / 180.0,
                    config_.localization.max_range_m);
                double side = left_clearance >= right_clearance ? 1.0 : -1.0;
                if (rejoin_stage && std::abs(road_projection.lateral_offset) > 0.04) {
                    side = road_projection.lateral_offset >= 0.0 ? 1.0 : -1.0;
                }

                const double target_ahead =
                    micro_hardware_lab
                        ? (rejoin_stage ? 0.10 : 0.14)
                        : (rejoin_stage ? (lab_scale_mixed_world ? 0.18 : (compact_mixed_world ? 0.30 : 0.46))
                                        : (lab_scale_mixed_world ? 0.20 : (compact_mixed_world ? 0.38 : 0.62)));
                const double road_offset =
                    micro_hardware_lab
                        ? (rejoin_stage ? 0.045 : 0.075)
                        : (rejoin_stage ? (lab_scale_mixed_world ? 0.04 : (compact_mixed_world ? 0.08 : 0.10))
                                        : (compact_mixed_world
                                               ? clamp_value(0.5 * geometry_.body_width + (lab_scale_mixed_world ? 0.115 : 0.19),
                                                             lab_scale_mixed_world ? 0.18 : 0.26,
                                                             lab_scale_mixed_world ? 0.22 : 0.32)
                                               : 0.46));
                const double target_bounds_margin =
                    micro_hardware_lab ? 0.035 : (lab_scale_mixed_world ? 0.10 : 0.18);
                if (road_projection.valid && !rejoin_stage) {
                    const auto side_score = [&](double candidate_side) {
                        const Vec2 road_target = point_at_road_s(road_projection.s + target_ahead);
                        const Vec2 normal{-road_projection.tangent.y, road_projection.tangent.x};
                        const Vec2 candidate_target{
                            road_target.x + candidate_side * road_offset * normal.x,
                            road_target.y + candidate_side * road_offset * normal.y,
                        };
                        double score = 0.0;
                        if (!is_inside_bounds(world_, candidate_target, -target_bounds_margin)) {
                            score -= 4.0;
                        }
                        const double padding = 0.5 * geometry_.body_width + 0.08;
                        for (const Rect& obstacle : world_.obstacles()) {
                            if (point_in_expanded_rect(candidate_target, obstacle, padding)) {
                                score -= 12.0;
                            }
                        }
                        score += candidate_side > 0.0 ? 0.05 * left_clearance : 0.05 * right_clearance;
                        return score;
                    };
                    const double upper_score = side_score(1.0);
                    const double lower_score = side_score(-1.0);
                    side = upper_score >= lower_score ? 1.0 : -1.0;
                }
                const auto build_mixed_target = [&](double candidate_side,
                                                    double candidate_ahead,
                                                    double candidate_offset) {
                    Vec2 target{};
                    if (road_projection.valid) {
                        const Vec2 road_target = point_at_road_s(road_projection.s + candidate_ahead);
                        const Vec2 normal{-road_projection.tangent.y, road_projection.tangent.x};
                        target = {
                            road_target.x + candidate_side * candidate_offset * normal.x,
                            road_target.y + candidate_side * candidate_offset * normal.y,
                        };
                    } else {
                        const Vec2 forward{std::cos(estimate_.yaw), std::sin(estimate_.yaw)};
                        const Vec2 lateral{-forward.y, forward.x};
                        target = {
                            estimate_.position.x + candidate_ahead * forward.x + candidate_side * candidate_offset * lateral.x,
                            estimate_.position.y + candidate_ahead * forward.y + candidate_side * candidate_offset * lateral.y,
                        };
                    }
                    return clamp_point_to_bounds(world_, target, target_bounds_margin);
                };
                const auto repeats_recent_compact_target = [&](const Vec2& target) {
                    if (!compact_mixed_world) {
                        return false;
                    }
                    for (const Vec2& passed_target : passed_unstructured_gap_positions_) {
                        if (distance(passed_target, target) <= 0.16) {
                            return true;
                        }
                    }
                    return false;
                };

                std::vector<Vec2> target_options;
                target_options.reserve(8);
                const std::array<double, 2> candidate_sides{{side, -side}};
                const std::array<double, 2> candidate_aheads{{
                    target_ahead,
                    std::max(0.08, 0.65 * target_ahead),
                }};
                const std::array<double, 2> candidate_offsets{{
                    road_offset,
                    micro_hardware_lab
                        ? std::max(rejoin_stage ? 0.030 : 0.055, 0.72 * road_offset)
                        : std::max(rejoin_stage ? 0.035 : 0.08, 0.72 * road_offset),
                }};
                for (double candidate_side : candidate_sides) {
                    if (rejoin_stage && candidate_side != side) {
                        continue;
                    }
                    for (double candidate_ahead : candidate_aheads) {
                        for (double candidate_offset : candidate_offsets) {
                            target_options.push_back(
                                build_mixed_target(candidate_side, candidate_ahead, candidate_offset));
                        }
                    }
                }

                std::optional<Vec2> fallback_repeated_target;
                for (const Vec2& mixed_target : target_options) {
                    if (!dynamic_gap_point_allowed(mixed_target)) {
                        continue;
                    }
                    if (repeats_recent_compact_target(mixed_target)) {
                        if (!fallback_repeated_target.has_value()) {
                            fallback_repeated_target = mixed_target;
                        }
                        continue;
                    }
                    set_locked_gap_goal(mixed_target);
                    publish_locked_gap_goal();
                    return;
                }
                if ((front_obstacle_pressure || compact_close_obstacle_recovery) &&
                    fallback_repeated_target.has_value()) {
                    set_locked_gap_goal(*fallback_repeated_target);
                    publish_locked_gap_goal();
                    return;
                }
            }
        }
    }

    if (locked_gap_goal_.has_value()) {
        if (!dynamic_gap_point_allowed(*locked_gap_goal_)) {
            restart_unstructured_scan();
        } else {
            startup_scan_complete_ = true;
            unstructured_no_candidate_scan_elapsed_s_ = 0.0;
            publish_locked_gap_goal();
            return;
        }
    }

    const bool strict_locked_motion = strict_locked_gate_motion_enabled();
    if (strict_locked_motion && gate_specs_.empty()) {
        unstructured_no_candidate_scan_elapsed_s_ += std::max(dt, 0.0);
    } else {
        unstructured_no_candidate_scan_elapsed_s_ = 0.0;
    }
    const bool startup_scan_enabled = config_.gap_extraction.startup_scan_duration_s > 1e-3;
    if (!startup_scan_enabled) {
        startup_scan_complete_ = true;
    } else {
        startup_scan_elapsed_s_ += std::max(dt, 0.0);
        const double startup_scan_duration =
            std::max(config_.gap_extraction.startup_scan_duration_s, 0.0);
        startup_scan_complete_ = startup_scan_elapsed_s_ >= startup_scan_duration;

        if (!startup_scan_complete_ &&
            perception_map_ready() &&
            !gate_specs_.empty()) {
            bool have_forward_candidate = false;
            for (const GateSpec& spec : gate_specs_) {
                const double heading_error = std::abs(
                    wrap_angle(angle_to(estimate_.position, spec.position) - estimate_.yaw));
                const double forward_lock_angle = strict_locked_motion
                    ? 72.0 * kPi / 180.0
                    : 28.0 * kPi / 180.0;
                if (heading_error <= forward_lock_angle) {
                    have_forward_candidate = true;
                    break;
                }
            }

            const double min_scan_before_lock_s = strict_locked_motion
                ? clamp_value(0.40 * startup_scan_duration, 0.55, 0.85)
                : std::min(0.35, 0.5 * startup_scan_duration);
            if (have_forward_candidate &&
                startup_scan_elapsed_s_ >= min_scan_before_lock_s) {
                startup_scan_complete_ = true;
            }
        }
        if (strict_locked_motion &&
            startup_scan_complete_ &&
            (!perception_map_ready() || gate_specs_.empty())) {
            startup_scan_complete_ = false;
            startup_scan_elapsed_s_ = 0.0;
            update_unstructured_scan_direction(true);
        }
    }

    if (!startup_scan_complete_) {
        if (!strict_locked_motion) {
            gates_.clear();
            gate_specs_.clear();
            diagnostics_.candidate_gates = 0;
        } else {
            for (gate& candidate : gates_) {
                candidate.choose = false;
                candidate.too_far = false;
            }
            diagnostics_.candidate_gates = static_cast<int>(gate_specs_.size());
        }
        visible_gate_indices_.clear();
        chosen_gate_index_ = -1;
        diagnostics_.chosen_gate_distance = std::numeric_limits<double>::infinity();
        return;
    }

    for (gate& candidate : gates_) {
        candidate.choose = false;
        candidate.too_far = false;
    }

    if (!startup_scan_complete_ || !perception_map_ready() || gate_specs_.empty()) {
        chosen_gate_index_ = -1;
        diagnostics_.candidate_gates = static_cast<int>(gate_specs_.size());
        if (strict_locked_motion) {
            startup_scan_complete_ = false;
        }
        return;
    }

    size_t preferred_gate_index = 0;
    double best_gate_score = -std::numeric_limits<double>::infinity();
    bool have_reachable_gate = false;
    for (size_t i = 0; i < gate_specs_.size(); ++i) {
        const double heading_error = std::abs(
            wrap_angle(angle_to(estimate_.position, gate_specs_[i].position) - estimate_.yaw));
        if (heading_error <= 85.0 * kPi / 180.0) {
            have_reachable_gate = true;
            break;
        }
    }
    for (size_t i = 0; i < gate_specs_.size(); ++i) {
        const Vec2& target = gate_specs_[i].position;
        const double heading_error = std::abs(
            wrap_angle(angle_to(estimate_.position, target) - estimate_.yaw));
        const double gate_distance = distance(estimate_.position, target);
        const double forward_progress =
            std::max(std::cos(heading_error), 0.0) * gate_distance;
        const double goal_distance = distance(target, world_.goal());
        const double rank_bonus = std::max(0.0, 1.6 - 0.30 * static_cast<double>(i));
        const double goal_progress = clamp_value(
            (distance(estimate_.position, world_.goal()) - goal_distance) /
                std::max(config_.gap_extraction.max_target_distance_m, 0.10),
            -0.60,
            1.20);
        const double heading_penalty = clamp_value(
            (heading_error - 40.0 * kPi / 180.0) /
                (50.0 * kPi / 180.0),
            0.0,
            1.6);
        const double reverse_penalty = clamp_value(
            (heading_error - 90.0 * kPi / 180.0) /
                (35.0 * kPi / 180.0),
            0.0,
            2.0);
        double gate_score =
            rank_bonus +
            1.15 * goal_progress +
            0.95 * forward_progress +
            0.40 * clamp_value(std::cos(heading_error), -0.35, 1.0) -
            1.55 * heading_penalty -
            (have_reachable_gate ? 2.10 : 0.90) * reverse_penalty -
            0.12 * gate_distance;
        if (i == 0) {
            gate_score += 0.18;
        }

        if (gate_score > best_gate_score + 1e-6 ||
            (std::abs(gate_score - best_gate_score) <= 1e-6 &&
             goal_distance + 1e-6 < distance(gate_specs_[preferred_gate_index].position, world_.goal()))) {
            preferred_gate_index = i;
            best_gate_score = gate_score;
        }
    }

    set_locked_gap_goal(gate_specs_[preferred_gate_index].position);
    publish_locked_gap_goal();
}

