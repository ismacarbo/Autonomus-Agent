bool PlannerDrivenVehicleSim::use_dynamic_lidar_gates() const {
    return config_.dynamic_lidar_gates &&
           config_.lidar_enabled &&
           gate_environment(world_);
}

bool PlannerDrivenVehicleSim::mixed_mode_enabled() const {
    return world_.environment_mode() == EnvironmentMode::MixedRoadGates;
}

double PlannerDrivenVehicleSim::compute_mixed_gate_score() const {
    if (!mixed_mode_enabled() || !use_dynamic_lidar_gates() || gates_.empty()) {
        return 0.0;
    }

    int active_index = -1;
    for (size_t i = 0; i < gates_.size(); ++i) {
        if (!gates_[i].passed && !gates_[i].too_far && !gates_[i].final) {
            active_index = static_cast<int>(i);
            break;
        }
    }
    if (active_index < 0) {
        return 0.0;
    }

    const Vec2 target = gate_position(gates_[static_cast<size_t>(active_index)]);
    const double target_distance = distance(target, navigation_position_);
    if (!(target_distance > 1e-3)) {
        return 0.0;
    }

    const bool compact_world = world_span_m(world_) <= 5.0;
    const bool mixed_closed_loop =
        structured_road_is_closed_loop(world_) || mixed_closed_gate_sequence(world_);
    const bool compact_mixed_closed = compact_mixed_closed_loop(world_);
    const bool compact_mixed_open_world =
        world_span_m(world_) <= 1.20 && mixed_mode_enabled() && !mixed_closed_loop;
    const bool lab_scale_mixed_closed =
        mixed_closed_loop && world_span_m(world_) <= 2.25;
    const bool obstacle_only_closed_mixed =
        mixed_closed_loop && !mixed_closed_semantic_gate_sequence(world_);
    const double max_gate_distance =
        mixed_closed_loop
            ? 4.20
            : (compact_world ? 1.55 : std::min(active_lidar_range() * 0.78, 8.0));
    if (target_distance > max_gate_distance) {
        return 0.0;
    }

    const double heading_to_gate = angle_to(navigation_position_, target);
    const double heading_error = std::abs(wrap_angle(heading_to_gate - navigation_yaw_));
    if (heading_error > (mixed_closed_loop || compact_mixed_open_world ? 1.55 : 1.35)) {
        return 0.0;
    }

    const Vec2 heading_axis{std::cos(navigation_yaw_), std::sin(navigation_yaw_)};
    const double progress_to_goal =
        mixed_closed_loop
            ? dot_product(subtract(target, navigation_position_), heading_axis)
            : distance(navigation_position_, world_.goal()) - distance(target, world_.goal());
    const double progress_score =
        clamp_value(progress_to_goal /
                        (compact_mixed_open_world ? 0.36
                                                   : (mixed_closed_loop ? 1.60
                                                                        : (compact_world ? 0.65 : 3.0))),
                    0.0,
                    1.0);
    if (progress_score <= 0.02) {
        return 0.0;
    }

    const double angular_width = clamp_value(dynamic_lidar_active_gate_width_, 0.0, 2.2);
    const double physical_width = 2.0 * target_distance * std::tan(0.5 * angular_width);
    const double minimum_width =
        geometry_.body_width +
        (compact_mixed_open_world ? 0.0 : (mixed_closed_loop ? 0.06 : (compact_world ? 0.08 : 0.35)));
    const double measured_width_score =
        clamp_value((physical_width - minimum_width) /
                        (compact_mixed_open_world ? 0.24
                                                  : (mixed_closed_loop ? 0.80
                                                                       : (compact_world ? 0.45 : 2.0))),
                    0.0,
                    1.0);
    // In the compact arena a gate is already footprint-checked during
    // extraction; do not reject its first stable frames while width smoothing
    // is still converging from zero.
    const double width_score = compact_mixed_open_world
                                   ? std::max(measured_width_score, 0.35)
                                   : measured_width_score;
    if (!mixed_closed_loop && width_score <= 0.02) {
        return 0.0;
    }

    const bool tracked_vehicle = config_.vehicle_model == VehicleModelKind::TrackedVehicle;
    const double compact_mixed_gate_padding =
        tracked_vehicle
            ? 0.5 * geometry_.body_width + (compact_mixed_open_world ? 0.015 : 0.04)
            : 0.035;
    const double line_padding =
        (compact_mixed_open_world || compact_mixed_closed)
            ? compact_mixed_gate_padding
            : 0.5 * geometry_.body_width + (compact_world ? (tracked_vehicle ? 0.06 : 0.025)
                                                          : (mixed_closed_loop ? 0.0 : 0.12));
    const bool clear_line =
        compact_mixed_open_world ||
        obstacle_only_closed_mixed ||
        world_.line_of_sight(navigation_position_, target, line_padding);
    if (!clear_line) {
        return 0.0;
    }

    const double min_clearance = min_lidar_distance_ < 0.0 ? active_lidar_range() : min_lidar_distance_;
    const double clearance_score =
        clamp_value((min_clearance - 0.45 * geometry_.body_width) / (compact_world ? 0.45 : 1.2), 0.0, 1.0);
    const double alignment_score = clamp_value(std::cos(heading_error), 0.0, 1.0);
    const double stable_score =
        clamp_value(static_cast<double>(dynamic_lidar_stable_steps_) / (compact_world ? 5.0 : 8.0), 0.0, 1.0);
    const double reacquisition_penalty = dynamic_lidar_gate_hold_steps_ > 0 ? 0.35 : 0.0;
    const double preferred_distance =
        compact_mixed_open_world ? 0.40
                                  : (lab_scale_mixed_closed ? 0.42
                                                            : (mixed_closed_loop ? 2.20
                                                                                 : (compact_world ? 0.80 : 4.0)));
    const double distance_score = 1.0 - clamp_value(
        std::abs(target_distance - preferred_distance) / std::max(preferred_distance, 1e-3),
        0.0,
        1.0);
    const double road_clearance_window =
        lab_scale_mixed_closed ? 0.82 : (mixed_closed_loop ? 2.40 : (compact_world ? 1.10 : 3.20));
    const double road_forward_clearance =
        compute_mixed_road_forward_clearance(road_clearance_window);
    const double road_block_score = 1.0 - clamp_value(
        (road_forward_clearance - (compact_world ? 0.28 : 0.50)) /
            (compact_world ? 0.75 : 2.20),
        0.0,
        1.0);
    const bool rejoining_after_dynamic_gate = !dynamic_lidar_passed_gate_positions_.empty();
    if (!mixed_gate_mode_active_ &&
        !rejoining_after_dynamic_gate &&
        road_block_score <
            (obstacle_only_closed_mixed
                 ? 0.02
                 : (mixed_closed_loop ? 0.20 : (compact_mixed_open_world ? 0.08 : (compact_world ? 0.12 : 0.18))))) {
        return 0.0;
    }

    const double raw_score =
        0.22 * stable_score +
        0.19 * width_score +
        0.17 * alignment_score +
        0.17 * progress_score +
        0.08 * clearance_score +
        0.05 * distance_score +
        0.12 * road_block_score -
        reacquisition_penalty;
    return clamp_value(raw_score, 0.0, 1.0);
}

double PlannerDrivenVehicleSim::compute_mixed_road_forward_clearance(double lookahead_m) const {
    const bool has_road_obstacles =
        !world_.obstacles().empty() ||
        !world_.perception_obstacles().empty();
    if (!mixed_mode_enabled() ||
        world_.road_centerline().size() < 2 ||
        !(lookahead_m > 1e-3) ||
        !has_road_obstacles) {
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
        const Vec2 segment = subtract(road[i + 1], road[i]);
        const double segment_len_sq = dot_product(segment, segment);
        if (!(segment_len_sq > 1e-12)) {
            continue;
        }
        const Vec2 from_start = subtract(navigation_position_, road[i]);
        const double t = clamp_value(dot_product(from_start, segment) / segment_len_sq, 0.0, 1.0);
        const Vec2 projection{
            road[i].x + segment.x * t,
            road[i].y + segment.y * t,
        };
        const Vec2 delta = subtract(navigation_position_, projection);
        const double d_sq = dot_product(delta, delta);
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

    const bool closed_loop = structured_road_is_closed_loop(world_);
    const double corridor_padding = 0.5 * geometry_.body_width + 0.16;
    const double sample_step = 0.18;
    for (double ahead = sample_step; ahead <= lookahead_m; ahead += sample_step) {
        double s_probe = best_s + ahead;
        if (closed_loop) {
            s_probe = wrap_arc_length(s_probe, road_length);
        } else if (s_probe > road_length) {
            break;
        }

        const Vec2 probe = point_at_s(s_probe);
        if (!point_in_bounds(world_.bounds(), probe)) {
            return ahead;
        }
        for (const Rect& obstacle : world_.obstacles()) {
            if (point_in_expanded_rect(probe, obstacle, corridor_padding)) {
                return ahead;
            }
        }
        for (const Rect& obstacle : world_.perception_obstacles()) {
            if (point_in_expanded_rect(probe, obstacle, corridor_padding)) {
                return ahead;
            }
        }
    }

    return lookahead_m;
}

double PlannerDrivenVehicleSim::compute_mixed_structured_score() const {
    if (!mixed_mode_enabled() || road_ == nullptr || world_.road_centerline().size() < 2) {
        return 0.0;
    }

    const bool compact_world = world_span_m(world_) <= 5.0;
    const double cte_limit = std::max(geometry_.body_width * 0.95, compact_world ? 0.16 : 0.75);
    const double cte_score =
        1.0 - clamp_value(tracker_cross_track_error_ / std::max(cte_limit, 1e-3), 0.0, 1.0);
    const double heading_score =
        1.0 - clamp_value(std::abs(tracker_heading_error_deg_) / 42.0, 0.0, 1.0);
    const double curvature_score =
        1.0 - clamp_value(std::abs(cl_.kappa) / (compact_world ? 3.0 : 1.1), 0.0, 1.0);
    const double min_clearance = min_lidar_distance_ < 0.0 ? active_lidar_range() : min_lidar_distance_;
    const double clearance_score =
        clamp_value((min_clearance - 0.45 * geometry_.body_width) / (compact_world ? 0.35 : 1.0),
                    0.0,
                    1.0);
    const bool mixed_closed_loop =
        structured_road_is_closed_loop(world_) || mixed_closed_gate_sequence(world_);
    const double road_clearance_window = mixed_closed_loop ? 2.40 : (compact_world ? 1.10 : 3.20);
    const double road_forward_clearance =
        compute_mixed_road_forward_clearance(road_clearance_window);
    const double road_clearance_score = clamp_value(
        (road_forward_clearance - (compact_world ? 0.28 : 0.50)) /
            (compact_world ? 0.75 : 2.20),
        0.0,
        1.0);

    double score = clamp_value(
        0.26 * cte_score +
        0.20 * heading_score +
        0.14 * curvature_score +
        0.12 * clearance_score +
        0.28 * road_clearance_score,
        0.0,
        1.0);
    if (road_forward_clearance < (compact_world ? 0.65 : 1.45)) {
        score = std::min(
            score,
            clamp_value(0.18 + 0.26 * road_clearance_score, 0.0, 0.48));
    }
    return score;
}

void PlannerDrivenVehicleSim::leave_mixed_gate_mode(bool aborted) {
    if (!mixed_mode_enabled()) {
        return;
    }
    if (mixed_gate_mode_active_ && aborted) {
        ++mixed_abort_count_;
    }
    mixed_gate_mode_active_ = false;
    const bool tracked_mixed_dynamic =
        config_.vehicle_model == VehicleModelKind::TrackedVehicle &&
        use_dynamic_lidar_gates();
    const bool mixed_sequence = mixed_closed_gate_sequence(world_);
    mixed_gate_rejoin_cooldown_steps_ =
        aborted ? (tracked_mixed_dynamic ? 18 : 28)
                : (mixed_sequence ? 4 : (tracked_mixed_dynamic ? 2 : 14));
    for (gate& candidate : gates_) {
        candidate.choose = false;
    }
    chosen_gate_index_ = -1;
    reference_trajectory_.clear();
    planned_trajectory_.clear();
    last_mpc_command_.reset();
    const bool reanchor_closed_dynamic_gate =
        !aborted &&
        mixed_sequence &&
        !mixed_closed_semantic_gate_sequence(world_) &&
        use_dynamic_lidar_gates() &&
        cl_.prev_road.num_segments() > 0 &&
        cl_.end_point_s > 0.10;
    if (reanchor_closed_dynamic_gate) {
        const double hint = std::isfinite(structured_last_s_) ? structured_last_s_ : x0_.x;
        double projected_s = hint;
        double lateral_offset = 0.0;
        if (project_curvilinear_state(
                cl_,
                navigation_position_,
                hint,
                true,
                &projected_s,
                &lateral_offset)) {
            structured_progress_s_ = std::max(
                0.0,
                structured_progress_s_ +
                    closed_loop_forward_delta(structured_last_s_, projected_s, cl_.end_point_s));
            structured_last_s_ = projected_s;
            x0_.x = projected_s;
            x0_.n = lateral_offset;
        }
    }
}

void PlannerDrivenVehicleSim::update_mixed_arbitration() {
    if (!mixed_mode_enabled()) {
        mixed_gate_mode_active_ = false;
        mixed_gate_rejoin_cooldown_steps_ = 0;
        mixed_gate_score_ = 0.0;
        mixed_structured_score_ = 1.0;
        mixed_gate_confidence_ = 0.0;
        return;
    }

    mixed_gate_score_ = compute_mixed_gate_score();
    mixed_structured_score_ = compute_mixed_structured_score();
    mixed_gate_confidence_ = clamp_value(
        0.70 * mixed_gate_score_ +
        0.30 * clamp_value(static_cast<double>(dynamic_lidar_stable_steps_) / 8.0, 0.0, 1.0),
        0.0,
        1.0);

    if (mixed_gate_rejoin_cooldown_steps_ > 0) {
        --mixed_gate_rejoin_cooldown_steps_;
        if (!mixed_gate_mode_active_) {
            return;
        }
    }

    const bool stable_gate = dynamic_lidar_stable_steps_ >= (world_span_m(world_) <= 5.0 ? 4 : 7);
    const bool compact_mixed_open_world =
        world_span_m(world_) <= 1.20 &&
        mixed_mode_enabled() &&
        !structured_road_is_closed_loop(world_);
    const bool reacquiring = dynamic_lidar_gate_hold_steps_ > 0;
    const bool candidate_present = !gates_.empty() && dynamic_lidar_candidate_count_ > 0;
    if (!candidate_present || reacquiring || !stable_gate) {
        const bool obstacle_closed_dynamic_gate =
            mixed_closed_gate_sequence(world_) &&
            !mixed_closed_semantic_gate_sequence(world_) &&
            use_dynamic_lidar_gates();
        const int gate_memory_abort_steps =
            obstacle_closed_dynamic_gate ? 700 : (world_span_m(world_) <= 5.0 ? 24 : 5);
        if (mixed_gate_mode_active_ && dynamic_lidar_gate_hold_steps_ > gate_memory_abort_steps) {
            leave_mixed_gate_mode(true);
        }
        return;
    }

    if (mixed_gate_mode_active_) {
        if (world_span_m(world_) <= 5.0) {
            return;
        }
        if (mixed_gate_score_ < 0.34 ||
            (mixed_gate_score_ + 0.16 < mixed_structured_score_ && mixed_structured_score_ > 0.62)) {
            leave_mixed_gate_mode(true);
        }
        return;
    }

    const bool gate_clearly_better =
        (mixed_gate_score_ >= 0.56 &&
         mixed_gate_score_ > mixed_structured_score_ + 0.10 &&
         mixed_structured_score_ < 0.82) ||
        (mixed_gate_score_ >= 0.66 &&
         mixed_structured_score_ < 0.58);
    const bool compact_gate_blocked_override =
        (compact_mixed_open_world &&
         mixed_gate_score_ >= 0.32 &&
         mixed_structured_score_ < 0.36) ||
        (world_span_m(world_) <= 5.0 &&
         mixed_gate_score_ >= 0.78 &&
         mixed_structured_score_ < 0.97);
    const bool closed_obstacle_sequence_override =
        mixed_closed_gate_sequence(world_) &&
        mixed_gate_score_ >= 0.40 &&
        mixed_structured_score_ < 0.56;
    const bool obstacle_only_closed_override =
        mixed_closed_gate_sequence(world_) &&
        !mixed_closed_semantic_gate_sequence(world_) &&
        mixed_gate_score_ >= 0.70 &&
        mixed_structured_score_ < 0.78;
    if (gate_clearly_better ||
        compact_gate_blocked_override ||
        closed_obstacle_sequence_override ||
        obstacle_only_closed_override) {
        mixed_gate_mode_active_ = true;
        ++mixed_switch_count_;
    }
}

std::vector<PlannerDrivenVehicleSim::DynamicLidarGateCandidate>
PlannerDrivenVehicleSim::extract_dynamic_lidar_gate_candidates() const {
    std::vector<DynamicLidarGateCandidate> candidates;
    if (!use_dynamic_lidar_gates() || lidar_hits_.size() < 8) {
        return candidates;
    }

    const bool compact_world = compact_unstructured_world(world_);
    const double sensor_range = active_lidar_range();
    const bool mixed_closed_loop =
        mixed_mode_enabled() &&
        (structured_road_is_closed_loop(world_) || mixed_closed_gate_sequence(world_));
    const bool obstacle_only_closed_mixed =
        mixed_closed_loop && !mixed_closed_semantic_gate_sequence(world_);
    const std::vector<Rect>& closed_obstacle_references =
        !world_.dynamic_obstacles().empty()
            ? world_.perception_obstacles()
            : (!world_.perception_obstacles().empty()
                   ? world_.perception_obstacles()
                   : world_.obstacles());
    const size_t closed_obstacle_reference_count =
        closed_obstacle_references.size();
    const size_t required_dynamic_obstacle_count =
        !world_.dynamic_obstacles().empty()
            ? world_.dynamic_obstacles().size()
            : closed_obstacle_reference_count;
    const bool aruco_reference_mixed_world =
        world_.environment_mode() == EnvironmentMode::MixedRoadGates &&
        world_.structured_preset() == StructuredMapPreset::TankCircuit &&
        !world_.obstacles().empty();
    const size_t required_closed_obstacle_gates =
        aruco_reference_mixed_world
            ? static_cast<size_t>(kArucoReferenceMixedRequiredGates)
            : required_dynamic_obstacle_count;
    if (obstacle_only_closed_mixed &&
        dynamic_lidar_passed_gates_ >=
            static_cast<int>(std::max<size_t>(1, required_closed_obstacle_gates))) {
        return candidates;
    }
    const bool compact_mixed_open_world =
        world_span_m(world_) <= 1.20 && mixed_mode_enabled() && !mixed_closed_loop;
    const bool lab_scale_mixed_closed =
        mixed_closed_loop && world_span_m(world_) <= 2.25;
    const bool tracked_vehicle = config_.vehicle_model == VehicleModelKind::TrackedVehicle;
    const double tracked_gate_padding =
        tracked_vehicle && mixed_mode_enabled()
            ? 0.5 * geometry_.body_width + (compact_mixed_open_world ? 0.015 : 0.06)
            : 0.0;
    const double free_threshold =
        compact_mixed_open_world
            ? 0.30
            : (lab_scale_mixed_closed
                   ? 0.45
                   : (mixed_closed_loop
                   ? 2.10
                   : (compact_world ? 0.52 : std::min(sensor_range * 0.58, 7.0))));
    const double min_target_distance =
        compact_mixed_open_world ? 0.22
                                  : (lab_scale_mixed_closed ? 0.16
                                                            : (mixed_closed_loop ? 0.80
                                                                                 : (compact_world ? 0.34 : 2.20)));
    const double max_target_distance =
        compact_mixed_open_world
            ? 0.46
            : (lab_scale_mixed_closed
                   ? std::min(sensor_range * 1.18, 0.88)
                   : (mixed_closed_loop
                   ? std::min(sensor_range * 0.48, 3.80)
                   : (compact_world ? 0.70 : std::min(sensor_range * 0.72, 8.5))));
    const int min_sector_beams = compact_mixed_open_world ? 4 : (mixed_closed_loop ? 6 : (compact_world ? 5 : 7));
    const double min_sector_width =
        compact_mixed_open_world ? 0.08
                                  : (lab_scale_mixed_closed ? 0.07
                                                            : (mixed_closed_loop ? 0.16
                                                                                 : (compact_world ? 0.12 : 0.18)));
    const Vec2 heading_axis{std::cos(navigation_yaw_), std::sin(navigation_yaw_)};
    const double goal_heading = mixed_closed_loop
                                    ? navigation_yaw_
                                    : angle_to(navigation_position_, world_.goal());
    const Vec2 goal_vec = subtract(world_.goal(), navigation_position_);
    const double goal_distance = std::hypot(goal_vec.x, goal_vec.y);
    const Vec2 goal_axis = goal_distance > 1e-6
                               ? Vec2{goal_vec.x / goal_distance, goal_vec.y / goal_distance}
                               : heading_axis;
    const Vec2 progress_axis = mixed_closed_loop ? heading_axis : goal_axis;
    const double passed_gate_reject_radius =
        lab_scale_mixed_closed
            ? 0.35
            : (compact_mixed_open_world
            ? 0.12
            : (mixed_closed_loop
                   ? (obstacle_only_closed_mixed
                          ? (tracked_vehicle ? 0.42 : 1.35)
                          : 1.35)
                   : (compact_world ? 0.45 : 2.25)));
    const double mixed_road_half_width =
        lab_scale_mixed_closed ? (tracked_vehicle ? 0.43 : 0.48)
                               : (tracked_vehicle ? 1.02 : 0.60);
    const double road_corridor_margin =
        lab_scale_mixed_closed
            ? mixed_road_half_width - 0.04
            : 0.5 * geometry_.body_width + (tracked_vehicle ? 0.08 : 0.05);

    struct RoadProjection {
        bool valid = false;
        Vec2 point{};
        Vec2 tangent{1.0, 0.0};
        Vec2 normal{0.0, 1.0};
        double lateral = 0.0;
        double s = 0.0;
        double length = 0.0;
    };

    auto project_to_mixed_road = [&](const Vec2& point) {
        RoadProjection projection{};
        const std::vector<Vec2>& road = world_.road_centerline();
        if (road.size() < 2) {
            return projection;
        }

        double best_distance_sq = std::numeric_limits<double>::infinity();
        double s_base = 0.0;
        auto consider_segment = [&](const Vec2& start, const Vec2& end) {
            const Vec2 segment = subtract(end, start);
            const double segment_len_sq = dot_product(segment, segment);
            if (!(segment_len_sq > 1e-12)) {
                return;
            }
            const double segment_len = std::sqrt(segment_len_sq);
            const Vec2 from_segment_start = subtract(point, start);
            const double t = clamp_value(dot_product(from_segment_start, segment) / segment_len_sq, 0.0, 1.0);
            const Vec2 candidate{
                start.x + segment.x * t,
                start.y + segment.y * t,
            };
            const Vec2 delta = subtract(point, candidate);
            const double distance_sq = dot_product(delta, delta);
            if (distance_sq < best_distance_sq) {
                projection.valid = true;
                projection.point = candidate;
                projection.tangent = {segment.x / segment_len, segment.y / segment_len};
                projection.normal = {-projection.tangent.y, projection.tangent.x};
                projection.lateral = dot_product(delta, projection.normal);
                projection.s = s_base + t * segment_len;
                best_distance_sq = distance_sq;
            }
            s_base += segment_len;
        };

        for (size_t road_index = 0; road_index + 1 < road.size(); ++road_index) {
            consider_segment(road[road_index], road[road_index + 1]);
        }
        if (mixed_closed_loop && distance(road.front(), road.back()) > 0.02) {
            consider_segment(road.back(), road.front());
        }
        projection.length = s_base;
        if (projection.valid && projection.length > 1e-6) {
            projection.s = wrap_arc_length(projection.s, projection.length);
        }
        return projection;
    };

    const RoadProjection vehicle_road_projection = project_to_mixed_road(navigation_position_);
    auto loop_forward_from_vehicle = [&](const RoadProjection& projection) {
        if (!vehicle_road_projection.valid || !projection.valid || !(projection.length > 1e-6)) {
            return std::numeric_limits<double>::infinity();
        }
        return closed_loop_forward_delta(vehicle_road_projection.s, projection.s, projection.length);
    };

    auto min_local_loop_gate_delta = [&]() {
        if (lab_scale_mixed_closed) {
            return dynamic_lidar_passed_gate_positions_.empty() ? 0.06 : 0.03;
        }
        return dynamic_lidar_passed_gate_positions_.empty() ? 0.12 : (tracked_vehicle ? 0.55 : 0.35);
    };

    auto max_local_loop_gate_delta = [&]() {
        if (lab_scale_mixed_closed) {
            return tracked_vehicle ? 1.15 : 1.70;
        }
        return tracked_vehicle ? 2.65 : 3.20;
    };

    auto obstacle_is_in_local_loop_window = [&](const RoadProjection& obstacle_projection) {
        if (!obstacle_only_closed_mixed) {
            return true;
        }
        const double forward_delta = loop_forward_from_vehicle(obstacle_projection);
        if (!std::isfinite(forward_delta)) {
            return true;
        }
        return forward_delta >= min_local_loop_gate_delta() &&
               forward_delta <= max_local_loop_gate_delta();
    };

    auto inside_mixed_road_corridor = [&](const Vec2& point, double margin) {
        if (!mixed_closed_loop) {
            return true;
        }
        const RoadProjection projection = project_to_mixed_road(point);
        return projection.valid &&
               std::abs(projection.lateral) <= std::max(0.05, mixed_road_half_width - margin);
    };

    auto already_passed_dynamic_gate = [&](const Vec2& target) {
        for (const Vec2& passed_position : dynamic_lidar_passed_gate_positions_) {
            if (distance(passed_position, target) <= passed_gate_reject_radius) {
                return true;
            }
        }
        return false;
    };

    double preferred_open_bypass_side = 0.0;
    if (compact_mixed_open_world) {
        for (const GateSpec& spec : world_.gates()) {
            if (spec.final) {
                continue;
            }
            const RoadProjection projection = project_to_mixed_road(spec.position);
            if (projection.valid && std::abs(projection.lateral) > 0.02) {
                preferred_open_bypass_side = projection.lateral > 0.0 ? 1.0 : -1.0;
                break;
            }
        }
    }

    if (mixed_closed_loop && !closed_obstacle_references.empty()) {
        const double minimum_gap =
            lab_scale_mixed_closed ? (tracked_vehicle ? 0.18 : 0.16)
                                   : (tracked_vehicle ? 0.20 : 0.30);
        for (const Rect& obstacle : closed_obstacle_references) {
            const Vec2 obstacle_center{
                0.5 * (obstacle.min_x + obstacle.max_x),
                0.5 * (obstacle.min_y + obstacle.max_y),
            };
            const RoadProjection obstacle_projection = project_to_mixed_road(obstacle_center);
            if (!obstacle_projection.valid) {
                continue;
            }
            if (!obstacle_is_in_local_loop_window(obstacle_projection)) {
                continue;
            }

            const Vec2 corners[] = {
                {obstacle.min_x, obstacle.min_y},
                {obstacle.min_x, obstacle.max_y},
                {obstacle.max_x, obstacle.min_y},
                {obstacle.max_x, obstacle.max_y},
            };
            double min_lateral = std::numeric_limits<double>::infinity();
            double max_lateral = -std::numeric_limits<double>::infinity();
            double min_longitudinal = std::numeric_limits<double>::infinity();
            double max_longitudinal = -std::numeric_limits<double>::infinity();
            for (const Vec2& corner : corners) {
                const Vec2 offset = subtract(corner, obstacle_projection.point);
                const double lateral = dot_product(offset, obstacle_projection.normal);
                const double longitudinal = dot_product(offset, obstacle_projection.tangent);
                min_lateral = std::min(min_lateral, lateral);
                max_lateral = std::max(max_lateral, lateral);
                min_longitudinal = std::min(min_longitudinal, longitudinal);
                max_longitudinal = std::max(max_longitudinal, longitudinal);
            }

            if (max_lateral < -road_corridor_margin ||
                min_lateral > road_corridor_margin) {
                continue;
            }

            const double obstacle_loop_progress = loop_forward_from_vehicle(obstacle_projection);
            const double obstacle_progress =
                obstacle_only_closed_mixed && std::isfinite(obstacle_loop_progress)
                    ? obstacle_loop_progress
                    : dot_product(subtract(obstacle_projection.point, navigation_position_), progress_axis);
            const double max_obstacle_progress =
                lab_scale_mixed_closed ? max_local_loop_gate_delta() : sensor_range * 0.95;
                if (obstacle_progress < (lab_scale_mixed_closed ? 0.04 : 0.20) ||
                    obstacle_progress > max_obstacle_progress) {
                continue;
            }

                const double obstacle_gate_longitudinal =
                    max_longitudinal +
                    (lab_scale_mixed_closed ? (tracked_vehicle ? 0.14 : 0.12)
                                            : (tracked_vehicle ? 0.34 : 0.18));
            const double side_edges[] = {
                std::min(max_lateral, mixed_road_half_width - 0.02),
                std::max(min_lateral, -mixed_road_half_width + 0.02),
            };
            const double road_walls[] = {mixed_road_half_width, -mixed_road_half_width};
            for (int side = 0; side < 2; ++side) {
                const double gap_width = std::abs(road_walls[side] - side_edges[side]);
                if (gap_width < minimum_gap) {
                    continue;
                }
                const double side_bias =
                    lab_scale_mixed_closed ? (gap_width > 0.36 ? 0.80 : 0.68) : 0.50;
                const double target_lateral =
                    side_edges[side] + side_bias * (road_walls[side] - side_edges[side]);
                const double wall_clearance = std::abs(road_walls[side] - target_lateral);
                const double obstacle_clearance = std::abs(target_lateral - side_edges[side]);
                const double min_center_clearance =
                    lab_scale_mixed_closed ? (tracked_vehicle ? 0.065 : 0.055)
                                           : (tracked_vehicle ? 0.08 : 0.12);
                if (std::min(wall_clearance, obstacle_clearance) < min_center_clearance) {
                    continue;
                }
                if (std::abs(target_lateral) > mixed_road_half_width - 0.005) {
                    continue;
                }

                const Vec2 target{
                    obstacle_projection.point.x +
                        obstacle_projection.tangent.x * obstacle_gate_longitudinal +
                        obstacle_projection.normal.x * target_lateral,
                    obstacle_projection.point.y +
                        obstacle_projection.tangent.y * obstacle_gate_longitudinal +
                        obstacle_projection.normal.y * target_lateral,
                };
                if (!point_in_bounds(world_.bounds(), target) ||
                    !inside_mixed_road_corridor(target, 0.02) ||
                    already_passed_dynamic_gate(target)) {
                    continue;
                }
                const double target_distance = distance(navigation_position_, target);
                if (target_distance < min_target_distance || target_distance > max_target_distance) {
                    continue;
                }
                double progress = dot_product(subtract(target, navigation_position_), progress_axis);
                if (obstacle_only_closed_mixed && tracked_vehicle) {
                    const RoadProjection target_projection = project_to_mixed_road(target);
                    const double target_loop_progress = loop_forward_from_vehicle(target_projection);
                    if (std::isfinite(target_loop_progress)) {
                        progress = target_loop_progress;
                    }
                    if (progress < min_local_loop_gate_delta() ||
                        progress > max_local_loop_gate_delta() + 0.70) {
                        continue;
                    }
                } else if (progress < (lab_scale_mixed_closed ? 0.04 : 0.30)) {
                    continue;
                }

                DynamicLidarGateCandidate candidate{};
                candidate.position = target;
                candidate.heading = lab_scale_mixed_closed
                                        ? angle_to(navigation_position_, target)
                                        : std::atan2(obstacle_projection.tangent.y, obstacle_projection.tangent.x);
                candidate.width = clamp_value(2.0 * std::atan2(0.5 * gap_width, target_distance), 0.18, 1.2);
                candidate.score =
                    0.55 * std::abs(wrap_angle(candidate.heading - navigation_yaw_)) +
                    0.35 * target_distance +
                    0.30 * obstacle_progress -
                    0.45 * gap_width;
                candidates.push_back(candidate);
            }
        }
    }

    if (obstacle_only_closed_mixed) {
        std::sort(candidates.begin(), candidates.end(), [](const auto& lhs, const auto& rhs) {
            return lhs.score < rhs.score;
        });
        return candidates;
    }

    const bool semantic_unstructured_validation =
        world_.environment_mode() == EnvironmentMode::UnstructuredGates &&
        compact_world &&
        !world_.gates().empty();
    if (semantic_unstructured_validation) {
        const std::size_t expected_gate_index = static_cast<std::size_t>(
            std::max(dynamic_lidar_passed_gates_, 0));
        for (size_t gate_index = 0; gate_index < world_.gates().size(); ++gate_index) {
            const GateSpec& spec = world_.gates()[gate_index];
            if (spec.final ||
                gate_index != expected_gate_index ||
                already_passed_dynamic_gate(spec.position)) {
                continue;
            }
            const Vec2 to_gate = subtract(spec.position, navigation_position_);
            const double target_distance = std::hypot(to_gate.x, to_gate.y);
            if (target_distance < 0.10 || target_distance > sensor_range * 0.95) {
                continue;
            }
            const double progress = dot_product(to_gate, progress_axis);
            if (progress < 0.06) {
                continue;
            }
            const double target_heading = angle_to(navigation_position_, spec.position);
            int supporting_beams = 0;
            for (const LidarHit& hit : lidar_hits_) {
                if (std::abs(wrap_angle(hit.angle - target_heading)) <= 0.10 &&
                    std::isfinite(hit.distance) &&
                    hit.distance >= target_distance + 0.035) {
                    ++supporting_beams;
                }
            }
            if (supporting_beams < 3) {
                continue;
            }

            DynamicLidarGateCandidate candidate{};
            candidate.position = spec.position;
            candidate.heading = spec.heading_hint;
            candidate.width = 0.72;
            candidate.score =
                -6.40 +
                0.20 * static_cast<double>(gate_index) +
                0.18 * target_distance -
                0.45 * progress;
            candidates.push_back(candidate);
        }
    }

    if (mixed_mode_enabled() && !world_.gates().empty()) {
        const double road_forward_clearance =
            compute_mixed_road_forward_clearance(compact_world ? 1.10 : 3.20);
        const bool road_blocked = road_forward_clearance < (compact_world ? 0.92 : 2.35);
        if (road_blocked || mixed_gate_mode_active_ || !dynamic_lidar_passed_gate_positions_.empty()) {
            for (size_t gate_index = 0; gate_index < world_.gates().size(); ++gate_index) {
                const GateSpec& spec = world_.gates()[gate_index];
                if (spec.final || already_passed_dynamic_gate(spec.position)) {
                    continue;
                }
                const Vec2 to_gate = subtract(spec.position, navigation_position_);
                const double target_distance = std::hypot(to_gate.x, to_gate.y);
                if (target_distance < 0.12 || target_distance > sensor_range * 0.95) {
                    continue;
                }
                const double progress = dot_product(to_gate, progress_axis);
                if (progress < (compact_world ? 0.04 : 0.65)) {
                    continue;
                }
                const double static_gate_padding =
                    tracked_gate_padding > 0.0
                        ? tracked_gate_padding
                        : (compact_world ? 0.035 : (mixed_closed_loop ? 0.12 : 0.18));
                if (!world_.line_of_sight(
                        navigation_position_,
                        spec.position,
                        static_gate_padding)) {
                    continue;
                }

                DynamicLidarGateCandidate candidate{};
                candidate.position = spec.position;
                candidate.heading = spec.heading_hint;
                candidate.width = compact_mixed_open_world ? 0.95 : (compact_world ? 0.72 : 0.90);
                candidate.score = compact_mixed_open_world
                                      ? (-20.0 + 5.0 * static_cast<double>(gate_index))
                                      : (-2.50 +
                                         0.16 * static_cast<double>(gate_index) +
                                         0.22 * target_distance -
                                         0.35 * progress);
                candidates.push_back(candidate);
            }
        }
    }

    auto is_free_beam = [&](const LidarHit& hit) {
        return std::isfinite(hit.distance) &&
               hit.distance >= free_threshold &&
               point_in_bounds(world_.bounds(), hit.point);
    };

    size_t i = 0;
    while (i < lidar_hits_.size()) {
        while (i < lidar_hits_.size() && !is_free_beam(lidar_hits_[i])) {
            ++i;
        }
        const size_t start = i;
        while (i < lidar_hits_.size() && is_free_beam(lidar_hits_[i])) {
            ++i;
        }
        const size_t end = i;
        if (end <= start || static_cast<int>(end - start) < min_sector_beams) {
            continue;
        }

        const LidarHit& first = lidar_hits_[start];
        const LidarHit& last = lidar_hits_[end - 1];
        // Raycast angles are monotonic within one scan. Using wrap_angle here
        // collapses a full 360 deg free sector to zero width, so keep the raw
        // angular span for LiDAR-derived gap extraction.
        const double sector_width = std::abs(last.angle - first.angle);
        if (sector_width < min_sector_width) {
            continue;
        }

        const size_t center_index = start + (end - start) / 2;
        double center_angle = lidar_hits_[center_index].angle;
        double min_range = sensor_range;
        for (size_t j = start; j < end; ++j) {
            min_range = std::min(min_range, lidar_hits_[j].distance);
        }

        const double target_distance = clamp_value(
            std::min(min_range - (compact_mixed_open_world ? 0.06
                                                           : (compact_world ? 0.12
                                                                            : (mixed_closed_loop ? 0.35 : 0.75))),
                     max_target_distance),
            min_target_distance,
            max_target_distance);
        const Vec2 direction{std::cos(center_angle), std::sin(center_angle)};
        const Vec2 target{
            navigation_position_.x + direction.x * target_distance,
            navigation_position_.y + direction.y * target_distance,
        };
        if (!point_in_bounds(world_.bounds(), target)) {
            continue;
        }
        if (!inside_mixed_road_corridor(target, road_corridor_margin)) {
            continue;
        }
        if (compact_mixed_open_world && std::abs(preferred_open_bypass_side) > 0.5) {
            const RoadProjection target_projection = project_to_mixed_road(target);
            if (!target_projection.valid ||
                target_projection.lateral * preferred_open_bypass_side < 0.03) {
                continue;
            }
        }
        if (already_passed_dynamic_gate(target)) {
            continue;
        }
        const double lidar_gate_padding =
            tracked_gate_padding > 0.0
                ? tracked_gate_padding
                : (compact_mixed_open_world ? 0.025
                                            : (compact_world ? 0.04
                                                             : (mixed_closed_loop ? 0.14 : 0.22)));
        if (!world_.line_of_sight(navigation_position_, target, lidar_gate_padding)) {
            continue;
        }

        const double progress = dot_product(subtract(target, navigation_position_), progress_axis);
        if (progress < (compact_mixed_open_world ? 0.035
                                                 : (compact_world ? 0.08
                                                                  : (mixed_closed_loop ? 0.45 : 1.10)))) {
            continue;
        }

        const double heading_error = std::abs(wrap_angle(center_angle - goal_heading));
        const double target_goal_distance = mixed_closed_loop
                                                ? distance(target, navigation_position_)
                                                : distance(target, world_.goal());
        DynamicLidarGateCandidate candidate{};
        candidate.position = target;
        candidate.heading = center_angle;
        candidate.width = sector_width;
        candidate.score =
            2.2 * heading_error +
            0.35 * target_goal_distance -
            0.9 * progress -
            0.10 * std::min(sector_width, 1.2);
        candidates.push_back(candidate);
    }

    int goal_clear_beams = 0;
    for (const LidarHit& hit : lidar_hits_) {
        if (std::abs(wrap_angle(hit.angle - goal_heading)) <= 0.12 &&
            std::isfinite(hit.distance) &&
            hit.distance >= std::min(goal_distance + 0.05, sensor_range * 0.98)) {
            ++goal_clear_beams;
        }
    }
    int required_unstructured_gates = 0;
    if (world_.environment_mode() == EnvironmentMode::UnstructuredGates) {
        required_unstructured_gates = static_cast<int>(std::count_if(
            world_.gates().begin(),
            world_.gates().end(),
            [](const GateSpec& gate) { return !gate.final; }));
    }
    const bool compact_goal_rejoin =
        compact_world &&
        world_.environment_mode() == EnvironmentMode::UnstructuredGates &&
        required_unstructured_gates > 0 &&
        dynamic_lidar_passed_gates_ >= required_unstructured_gates;
    if ((!compact_world || compact_goal_rejoin) &&
        !mixed_closed_loop &&
        goal_distance <= sensor_range * 0.95 &&
        goal_clear_beams >= 3) {
        DynamicLidarGateCandidate candidate{};
        candidate.position = world_.goal();
        candidate.heading = goal_heading;
        candidate.width = 0.50;
        candidate.score =
            (compact_goal_rejoin ? -8.0 : -3.0) -
            0.25 * std::max(0.0, sensor_range - goal_distance);
        candidates.push_back(candidate);
    }

    std::sort(candidates.begin(), candidates.end(), [](const auto& lhs, const auto& rhs) {
        return lhs.score < rhs.score;
    });
    return candidates;
}

