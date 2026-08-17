#include "support.inc"

namespace thesis_sim {

void PlannerDrivenVehicleSim::update_dynamic_lidar_gates() {
    if (!use_dynamic_lidar_gates()) {
        dynamic_lidar_candidate_count_ = 0;
        dynamic_lidar_gate_hold_steps_ = 0;
        dynamic_lidar_stable_steps_ = 0;
        dynamic_lidar_active_gate_width_ = 0.0;
        dynamic_lidar_active_gate_score_ = 0.0;
        dynamic_lidar_active_gate_confidence_ = 0.0;
        active_dynamic_lidar_gate_track_id_ = -1;
        dynamic_lidar_gate_tracks_.clear();
        return;
    }

    const bool compact_mixed_open_world =
        mixed_mode_enabled() &&
        world_span_m(world_) <= 1.25 &&
        !structured_road_is_closed_loop(world_) &&
        !mixed_closed_gate_sequence(world_);
    if (compact_mixed_open_world && dynamic_lidar_passed_gates_ >= 1) {
        if (mixed_gate_mode_active_) {
            leave_mixed_gate_mode(false);
        }
        gates_.clear();
        chosen_gate_index_ = -1;
        dynamic_lidar_candidate_count_ = 0;
        dynamic_lidar_gate_hold_steps_ = 0;
        dynamic_lidar_stable_steps_ = 0;
        dynamic_lidar_active_gate_width_ = 0.0;
        dynamic_lidar_active_gate_score_ = 0.0;
        dynamic_lidar_active_gate_confidence_ = 0.0;
        active_dynamic_lidar_gate_track_id_ = -1;
        dynamic_lidar_gate_tracks_.clear();
        return;
    }

    const std::vector<DynamicLidarGateCandidate> raw_candidates =
        extract_dynamic_lidar_gate_candidates();
    const double span = world_span_m(world_);
    const double track_match_radius =
        span <= 2.25 ? 0.20 : (span <= 5.0 ? 0.45 : 1.00);
    const int track_max_misses = span <= 5.0 ? 10 : 7;
    const int track_min_hits = mixed_mode_enabled() ? 3 : 2;
    std::vector<bool> track_matched(dynamic_lidar_gate_tracks_.size(), false);

    for (const DynamicLidarGateCandidate& raw : raw_candidates) {
        int best_track = -1;
        double best_distance = track_match_radius;
        for (size_t i = 0; i < dynamic_lidar_gate_tracks_.size(); ++i) {
            if (track_matched[i]) {
                continue;
            }
            const double candidate_distance =
                distance(dynamic_lidar_gate_tracks_[i].position, raw.position);
            if (candidate_distance <= best_distance) {
                best_distance = candidate_distance;
                best_track = static_cast<int>(i);
            }
        }

        if (best_track < 0) {
            DynamicLidarGateTrack track;
            track.id = next_dynamic_lidar_gate_track_id_++;
            track.position = raw.position;
            track.heading = raw.heading;
            track.score = raw.score;
            track.width = raw.width;
            track.hits = 1;
            track.last_seen_step = step_count_;
            dynamic_lidar_gate_tracks_.push_back(track);
            track_matched.push_back(true);
            continue;
        }

        DynamicLidarGateTrack& track =
            dynamic_lidar_gate_tracks_[static_cast<size_t>(best_track)];
        const bool locked_track =
            track.id == active_dynamic_lidar_gate_track_id_ &&
            (!mixed_mode_enabled() || mixed_gate_mode_active_);
        const double alpha = locked_track ? 0.12 : 0.24;
        track.position = {
            (1.0 - alpha) * track.position.x + alpha * raw.position.x,
            (1.0 - alpha) * track.position.y + alpha * raw.position.y,
        };
        track.heading = wrap_angle(
            track.heading + 0.18 * wrap_angle(raw.heading - track.heading));
        track.width = 0.75 * track.width + 0.25 * raw.width;
        track.score = 0.75 * track.score + 0.25 * raw.score;
        track.hits = std::min(track.hits + 1, 1000000);
        track.misses = 0;
        track.last_seen_step = step_count_;
        track_matched[static_cast<size_t>(best_track)] = true;
    }

    for (size_t i = 0; i < dynamic_lidar_gate_tracks_.size(); ++i) {
        if (i >= track_matched.size() || !track_matched[i]) {
            ++dynamic_lidar_gate_tracks_[i].misses;
        }
    }
    dynamic_lidar_gate_tracks_.erase(
        std::remove_if(
            dynamic_lidar_gate_tracks_.begin(),
            dynamic_lidar_gate_tracks_.end(),
            [&](const DynamicLidarGateTrack& track) {
                return track.misses > track_max_misses &&
                       track.id != active_dynamic_lidar_gate_track_id_;
            }),
        dynamic_lidar_gate_tracks_.end());

    std::vector<DynamicLidarGateCandidate> candidates;
    candidates.reserve(dynamic_lidar_gate_tracks_.size());
    for (const DynamicLidarGateTrack& track : dynamic_lidar_gate_tracks_) {
        if (track.hits < track_min_hits || track.misses > 2) {
            continue;
        }
        DynamicLidarGateCandidate filtered;
        filtered.track_id = track.id;
        filtered.position = track.position;
        filtered.heading = track.heading;
        filtered.score = track.score;
        filtered.width = track.width;
        filtered.confidence = clamp_value(
            static_cast<double>(track.hits - track_min_hits + 1) / 6.0,
            0.0,
            1.0) *
            clamp_value(1.0 - static_cast<double>(track.misses) / 3.0, 0.0, 1.0);
        candidates.push_back(filtered);
    }
    std::stable_sort(
        candidates.begin(),
        candidates.end(),
        [&](const DynamicLidarGateCandidate& lhs,
            const DynamicLidarGateCandidate& rhs) {
            const bool selection_locked =
                !mixed_mode_enabled() || mixed_gate_mode_active_;
            const bool lhs_locked = selection_locked &&
                                    lhs.track_id == active_dynamic_lidar_gate_track_id_;
            const bool rhs_locked = selection_locked &&
                                    rhs.track_id == active_dynamic_lidar_gate_track_id_;
            if (lhs_locked != rhs_locked) {
                return lhs_locked;
            }
            // Candidate extraction defines lower scores as better (all raw
            // candidate lists are sorted ascending). Reversing that order here
            // selected the farthest/worst bypass in Closed Obstacle Road.
            return lhs.score < rhs.score;
        });
    dynamic_lidar_candidate_count_ = static_cast<int>(candidates.size());
    if (std::getenv("THESIS_TRACE_GATES") != nullptr &&
        step_count_ % 20 == 0) {
        std::cerr << "dynamic_gate t=" << sim_time_
                  << " pose=(" << navigation_position_.x << ',' << navigation_position_.y << ')'
                  << " yaw=" << navigation_yaw_
                  << " passed=" << dynamic_lidar_passed_gates_
                  << " candidates=" << candidates.size();
        if (!candidates.empty()) {
            std::cerr << " first=(" << candidates.front().position.x
                      << ',' << candidates.front().position.y << ')'
                      << " heading=" << candidates.front().heading
                      << " score=" << candidates.front().score;
        }
        std::cerr << '\n';
    }

    auto publish_gate = [&](const DynamicLidarGateCandidate& candidate) {
        if (active_dynamic_lidar_gate_track_id_ >= 0 &&
            active_dynamic_lidar_gate_track_id_ != candidate.track_id) {
            ++dynamic_lidar_gate_target_switches_;
        }
        active_dynamic_lidar_gate_track_id_ = candidate.track_id;
        gate g{};
        g.x_pos = candidate.position.x;
        g.y_pos = candidate.position.y;
        g.road = cl_;
        g.road.PSI_end = candidate.heading;
        g.passed = false;
        g.choose = false;
        g.too_far = false;
        // Keep a LiDAR-confirmed goal selectable by the legacy gate planner.
        // That planner intentionally excludes GateSpec::final from primitive
        // selection. update_unstructured_gate_progress() recognises this
        // target by position and never counts it as an intermediate gate.
        g.final = false;
        gates_.clear();
        gates_.push_back(g);
        dynamic_lidar_gate_hold_steps_ = 0;
        dynamic_lidar_stable_steps_ = 1;
        dynamic_lidar_active_gate_width_ = candidate.width;
        dynamic_lidar_active_gate_score_ = candidate.score;
        dynamic_lidar_active_gate_confidence_ = candidate.confidence;
    };

    auto first_unpassed_gate_index = [&]() {
        for (size_t i = 0; i < gates_.size(); ++i) {
            if (!gates_[i].passed && !gates_[i].final) {
                return static_cast<int>(i);
            }
        }
        return -1;
    };

    if (mixed_mode_enabled() && !mixed_gate_mode_active_ && !candidates.empty()) {
        const DynamicLidarGateCandidate& candidate = candidates.front();
        const int mixed_active_index = first_unpassed_gate_index();
        if (mixed_closed_semantic_gate_sequence(world_) && mixed_active_index < 0) {
            return;
        }
        if (mixed_active_index >= 0) {
            gate& active_gate = gates_[static_cast<size_t>(mixed_active_index)];
            const double best_distance = distance(gate_position(active_gate), candidate.position);
            if (best_distance <= 1.20) {
                if (!mixed_closed_semantic_gate_sequence(world_)) {
                    const double kMixedAcquireAlpha =
                        mixed_closed_gate_sequence(world_) ? 0.55 : 0.60;
                    active_gate.x_pos =
                        (1.0 - kMixedAcquireAlpha) * active_gate.x_pos + kMixedAcquireAlpha * candidate.position.x;
                    active_gate.y_pos =
                        (1.0 - kMixedAcquireAlpha) * active_gate.y_pos + kMixedAcquireAlpha * candidate.position.y;
                    active_gate.road.PSI_end =
                        wrap_angle(0.75 * active_gate.road.PSI_end + 0.25 * candidate.heading);
                    // This gate has not been selected yet: follow the best
                    // stable track during acquisition. Once arbitration enters
                    // gate mode the branch below freezes both ID and position
                    // until the physical crossing.
                    active_dynamic_lidar_gate_track_id_ = candidate.track_id;
                }
                active_gate.passed = false;
                active_gate.final = false;
                active_gate.too_far = false;
                dynamic_lidar_gate_hold_steps_ = 0;
                dynamic_lidar_stable_steps_ = std::min(dynamic_lidar_stable_steps_ + 1, 1000000);
                dynamic_lidar_active_gate_width_ =
                    0.70 * dynamic_lidar_active_gate_width_ + 0.30 * candidate.width;
                dynamic_lidar_active_gate_score_ =
                    0.70 * dynamic_lidar_active_gate_score_ + 0.30 * candidate.score;
                dynamic_lidar_active_gate_confidence_ =
                    0.70 * dynamic_lidar_active_gate_confidence_ + 0.30 * candidate.confidence;
                return;
            }
        }
        if (mixed_closed_semantic_gate_sequence(world_) && mixed_active_index >= 0) {
            gate& active_gate = gates_[static_cast<size_t>(mixed_active_index)];
            active_gate.too_far = false;
            dynamic_lidar_gate_hold_steps_ = 0;
            dynamic_lidar_stable_steps_ = std::max(dynamic_lidar_stable_steps_, 1);
            dynamic_lidar_active_gate_width_ =
                dynamic_lidar_active_gate_width_ > 0.0 ? dynamic_lidar_active_gate_width_ : 0.72;
            return;
        }
        publish_gate(candidate);
        return;
    }

    int active_index = -1;
    for (size_t i = 0; i < gates_.size(); ++i) {
        if (!gates_[i].passed && !gates_[i].final) {
            active_index = static_cast<int>(i);
            break;
        }
    }

    if (active_index >= 0) {
        gate& active_gate = gates_[static_cast<size_t>(active_index)];
        int best_match = -1;
        double best_match_distance = std::numeric_limits<double>::infinity();
        const bool ordered_unstructured_validation =
            world_.environment_mode() == EnvironmentMode::UnstructuredGates &&
            compact_unstructured_world(world_) &&
            !world_.gates().empty();
        if (ordered_unstructured_validation) {
            if (!candidates.empty() && candidates.front().score < -5.0) {
                best_match = 0;
                best_match_distance = distance(
                    gate_position(active_gate),
                    candidates.front().position);
            }
        } else {
            for (size_t i = 0; i < candidates.size(); ++i) {
                if (candidates[i].track_id == active_dynamic_lidar_gate_track_id_) {
                    best_match = static_cast<int>(i);
                    best_match_distance = distance(
                        gate_position(active_gate),
                        candidates[i].position);
                    break;
                }
                const double d = distance(gate_position(active_gate), candidates[i].position);
                if (d < best_match_distance) {
                    best_match_distance = d;
                    best_match = static_cast<int>(i);
                }
            }
        }

        const bool compact_dynamic_world = compact_unstructured_world(world_);
        const bool mixed_dynamic_world = mixed_mode_enabled();
        const double match_radius =
            compact_dynamic_world ? 0.42 : (mixed_dynamic_world ? 1.65 : 2.40);
        if (best_match >= 0 && best_match_distance <= match_radius) {
            const DynamicLidarGateCandidate& candidate = candidates[static_cast<size_t>(best_match)];
            if ((compact_dynamic_world || mixed_dynamic_world) &&
                !mixed_closed_gate_sequence(world_)) {
                const double update_alpha = mixed_dynamic_world ? 0.45 : 0.30;
                active_gate.x_pos = (1.0 - update_alpha) * active_gate.x_pos + update_alpha * candidate.position.x;
                active_gate.y_pos = (1.0 - update_alpha) * active_gate.y_pos + update_alpha * candidate.position.y;
            }
            if (!mixed_closed_gate_sequence(world_)) {
                active_gate.road.PSI_end =
                    wrap_angle(0.85 * active_gate.road.PSI_end + 0.15 * candidate.heading);
            }
            active_gate.too_far = false;
            dynamic_lidar_gate_hold_steps_ = 0;
            dynamic_lidar_stable_steps_ = std::min(dynamic_lidar_stable_steps_ + 1, 1000000);
            dynamic_lidar_active_gate_width_ =
                0.75 * dynamic_lidar_active_gate_width_ + 0.25 * candidate.width;
            dynamic_lidar_active_gate_score_ =
                0.75 * dynamic_lidar_active_gate_score_ + 0.25 * candidate.score;
            dynamic_lidar_active_gate_confidence_ =
                0.75 * dynamic_lidar_active_gate_confidence_ + 0.25 * candidate.confidence;
            return;
        }

        ++dynamic_lidar_gate_hold_steps_;
        const bool pure_closed_dynamic_gate =
            mixed_mode_enabled() &&
            mixed_closed_gate_sequence(world_) &&
            !mixed_closed_semantic_gate_sequence(world_);
        const bool compact_open_mixed_gate =
            mixed_mode_enabled() &&
            !mixed_closed_gate_sequence(world_) &&
            world_span_m(world_) <= 1.25;
        const int hold_limit =
            pure_closed_dynamic_gate
                ? 700
                : (compact_open_mixed_gate
                       ? 700
                       // A semantic anchor has already been validated by the
                       // LiDAR before it is published.  Keep that identity
                       // locked while the robot physically crosses it: close
                       // to the anchor there may no longer be three beams
                       // extending beyond the target, which is not evidence
                       // that a different gate should be selected.
                       : (compact_unstructured_world(world_) ? 700
                                                             : (mixed_mode_enabled() ? 12 : 36)));
        if (dynamic_lidar_gate_hold_steps_ <= hold_limit) {
            active_gate.too_far = false;
            return;
        }

        if (mixed_closed_semantic_gate_sequence(world_)) {
            active_gate.too_far = false;
            dynamic_lidar_gate_hold_steps_ = 0;
            dynamic_lidar_stable_steps_ = std::max(dynamic_lidar_stable_steps_, 1);
            dynamic_lidar_active_gate_width_ =
                dynamic_lidar_active_gate_width_ > 0.0 ? dynamic_lidar_active_gate_width_ : 0.72;
            dynamic_lidar_active_gate_score_ = std::min(dynamic_lidar_active_gate_score_, 0.0);
            return;
        }

        gates_.clear();
        chosen_gate_index_ = -1;
        reference_trajectory_.clear();
        planned_trajectory_.clear();
        last_mpc_command_.reset();
        dynamic_lidar_gate_hold_steps_ = 0;
        dynamic_lidar_stable_steps_ = 0;
        dynamic_lidar_active_gate_width_ = 0.0;
        dynamic_lidar_active_gate_score_ = 0.0;
        dynamic_lidar_active_gate_confidence_ = 0.0;
        active_dynamic_lidar_gate_track_id_ = -1;
    }

    if (!candidates.empty()) {
        if (mixed_closed_semantic_gate_sequence(world_)) {
            return;
        }
        publish_gate(candidates.front());
    }
}

void PlannerDrivenVehicleSim::update_unstructured_gate_progress() {
    if (!gate_environment(world_) || gates_.empty()) {
        return;
    }

    const Vec2 vehicle_position = navigation_position_;
    const double world_span = world_span_m(world_);
    const bool compact_world = world_span <= 5.0;
    const double lane_width = std::max(sim_.W, compact_world ? 0.40 : 1.0);

    for (size_t i = 0; i < gates_.size(); ++i) {
        gate& candidate = gates_[i];
        if (candidate.passed || candidate.final) {
            continue;
        }
        if (use_dynamic_lidar_gates() && mixed_closed_gate_sequence(world_) &&
            (!mixed_gate_mode_active_ || chosen_gate_index_ != static_cast<int>(i))) {
            continue;
        }

        const Vec2 current_gate = gate_position(candidate);
        const bool dynamic_goal_target =
            use_dynamic_lidar_gates() &&
            world_.environment_mode() == EnvironmentMode::UnstructuredGates &&
            distance(current_gate, world_.goal()) <= 0.02;
        if (dynamic_goal_target) {
            continue;
        }
        const Vec2 previous_anchor = (i == 0) ? world_.start() : gate_position(gates_[i - 1]);
        const Vec2 next_anchor =
            (i + 1 < gates_.size()) ? gate_position(gates_[i + 1]) : world_.goal();

        Vec2 route_axis = subtract(next_anchor, previous_anchor);
        double route_length = std::hypot(route_axis.x, route_axis.y);
        if (!(route_length > 1e-6)) {
            route_axis = subtract(world_.goal(), previous_anchor);
            route_length = std::hypot(route_axis.x, route_axis.y);
        }
        if (!(route_length > 1e-6)) {
            if (use_dynamic_lidar_gates() && mixed_mode_enabled()) {
                route_axis = {
                    std::cos(candidate.road.PSI_end),
                    std::sin(candidate.road.PSI_end),
                };
                route_length = 1.0;
            } else {
                continue;
            }
        }
        route_axis.x /= route_length;
        route_axis.y /= route_length;

        const Vec2 gate_to_vehicle = subtract(vehicle_position, current_gate);
        const double gate_distance = std::hypot(gate_to_vehicle.x, gate_to_vehicle.y);
        const double signed_forward = dot_product(gate_to_vehicle, route_axis);
        const double lateral_from_gate_corridor = std::abs(cross_product(route_axis, gate_to_vehicle));
        const double previous_to_gate = distance(previous_anchor, current_gate);
        const double gate_to_next = distance(current_gate, next_anchor);
        const double local_spacing = std::min(previous_to_gate, gate_to_next);

        const bool dynamic_gates = use_dynamic_lidar_gates();
        const bool compact_unstructured_gate_world =
            world_.environment_mode() == EnvironmentMode::UnstructuredGates &&
            compact_world;
        const double pass_radius =
            compact_unstructured_gate_world
                ? clamp_value(0.055 + 0.10 * local_spacing, 0.085, 0.13)
                : compact_world
                ? clamp_value(0.16 + 0.12 * local_spacing, 0.22, 0.60)
                : (dynamic_gates
                       ? clamp_value(0.28 + 0.035 * local_spacing, 0.55, 1.05)
                       : clamp_value(0.55 + 0.08 * local_spacing, 0.85, 2.40));
        const double pass_lateral =
            compact_unstructured_gate_world
                ? clamp_value(0.055 + 0.12 * local_spacing, 0.085, 0.15)
                : compact_world
                ? clamp_value(0.65 * lane_width + 0.06 * local_spacing, 0.28, 0.75)
                : (dynamic_gates
                       ? clamp_value(0.45 * lane_width + 0.025 * local_spacing, 0.70, 1.40)
                       : clamp_value(0.75 * lane_width + 0.06 * local_spacing, 1.10, 3.00));
        const double forward_margin =
            compact_world
                ? clamp_value(0.06 + 0.04 * local_spacing, 0.08, 0.24)
                : (dynamic_gates
                       ? clamp_value(0.18 + 0.025 * local_spacing, 0.35, 0.80)
                       : clamp_value(0.25 + 0.035 * local_spacing, 0.40, 0.95));

        const bool reached_gate_region =
            !dynamic_gates && gate_distance <= pass_radius;
        const bool dynamic_unstructured_reached_gate =
            dynamic_gates &&
            world_.environment_mode() == EnvironmentMode::UnstructuredGates &&
            gate_distance <= (compact_world ? std::max(pass_radius, 0.20) : pass_radius);
        const bool compact_mixed_dynamic_gate =
            dynamic_gates && mixed_mode_enabled() && compact_world;
        const bool compact_mixed_closed =
            compact_mixed_closed_loop(world_);
        const bool closed_mixed_dynamic_gate =
            dynamic_gates && mixed_mode_enabled() &&
            (structured_road_is_closed_loop(world_) || mixed_closed_gate_sequence(world_));
        const bool semantic_closed_gate_sequence =
            mixed_closed_semantic_gate_sequence(world_);
        const bool obstacle_closed_dynamic_gate =
            closed_mixed_dynamic_gate && !semantic_closed_gate_sequence;
        const bool lab_scale_closed_dynamic_gate =
            obstacle_closed_dynamic_gate && world_span_m(world_) <= 2.25;
        const bool aruco_reference_mixed_world =
            obstacle_closed_dynamic_gate &&
            world_.structured_preset() == StructuredMapPreset::TankCircuit &&
            !world_.obstacles().empty();
        const double closed_dynamic_gate_reach_radius =
            semantic_closed_gate_sequence
                ? 0.46
                : (config_.vehicle_model == VehicleModelKind::TrackedVehicle ? 0.40 : 0.46);
        const bool lab_scale_open_dynamic_gate =
            compact_mixed_dynamic_gate &&
            !closed_mixed_dynamic_gate &&
            world_span <= 1.25;
        const double mixed_dynamic_reach_radius =
            aruco_reference_mixed_world
                ? (dynamic_lidar_passed_gates_ == 0 ? 0.36
                                                    : (dynamic_lidar_passed_gates_ == 1 ? 0.30 : 0.34))
                : (lab_scale_closed_dynamic_gate
                ? (dynamic_lidar_passed_gates_ == 0 ? 0.36 : 0.24)
                : (compact_mixed_closed ? 0.22
                                        : (closed_mixed_dynamic_gate
                                               ? closed_dynamic_gate_reach_radius
                                               : (lab_scale_open_dynamic_gate ? 0.16 : 0.14))));
        const bool compact_mixed_reached_gate =
            compact_mixed_dynamic_gate && gate_distance <= mixed_dynamic_reach_radius;
        const bool closed_mixed_reached_gate =
            closed_mixed_dynamic_gate && gate_distance <= mixed_dynamic_reach_radius;
        const double compact_mixed_pass_radius =
            compact_mixed_dynamic_gate
                ? (closed_mixed_dynamic_gate
                       ? mixed_dynamic_reach_radius
                       : (lab_scale_open_dynamic_gate ? 0.20 : 0.22))
                : pass_radius;
        const bool gate_corridor_distance_ok =
            obstacle_closed_dynamic_gate
                ? gate_distance <= mixed_dynamic_reach_radius
                : (compact_unstructured_gate_world
                       ? gate_distance <= std::max(pass_radius, 0.14)
                       : (!compact_mixed_dynamic_gate || gate_distance <= compact_mixed_pass_radius));
        const double effective_pass_lateral =
            dynamic_gates &&
                    world_.environment_mode() == EnvironmentMode::UnstructuredGates &&
                    compact_world
                ? std::max(pass_lateral, 0.38)
                : pass_lateral;
        const bool crossed_gate_corridor =
            signed_forward >= forward_margin &&
            lateral_from_gate_corridor <= effective_pass_lateral &&
            gate_corridor_distance_ok;

        bool car_body_cleared_compact_obstacle = true;
        if (lab_scale_open_dynamic_gate &&
            config_.vehicle_model == VehicleModelKind::CarLikeBicycle &&
            !world_.obstacles().empty()) {
            // A bypass may clear the block laterally while its centre is
            // still behind the obstacle's far longitudinal edge.  Testing
            // only road-axis progress therefore kept the car gate locked
            // after the actual opening had been crossed.  Use the same
            // oriented footprint contract as collision detection instead.
            car_body_cleared_compact_obstacle =
                !world_.collides(vehicle_.body_corners, -0.025);
        }

        if (car_body_cleared_compact_obstacle &&
            (reached_gate_region || dynamic_unstructured_reached_gate ||
             compact_mixed_reached_gate ||
             closed_mixed_reached_gate || crossed_gate_corridor)) {
            candidate.passed = true;
            candidate.choose = false;
            candidate.too_far = false;
            if (use_dynamic_lidar_gates()) {
                const int passed_track_id = active_dynamic_lidar_gate_track_id_;
                ++dynamic_lidar_passed_gates_;
                dynamic_lidar_passed_gate_positions_.push_back(current_gate);
                chosen_gate_index_ = -1;
                reference_trajectory_.clear();
                planned_trajectory_.clear();
                last_mpc_command_.reset();
                dynamic_lidar_gate_hold_steps_ = 0;
                dynamic_lidar_stable_steps_ = 0;
                dynamic_lidar_active_gate_width_ = 0.0;
                dynamic_lidar_active_gate_score_ = 0.0;
                dynamic_lidar_active_gate_confidence_ = 0.0;
                active_dynamic_lidar_gate_track_id_ = -1;
                dynamic_lidar_gate_tracks_.erase(
                    std::remove_if(
                        dynamic_lidar_gate_tracks_.begin(),
                        dynamic_lidar_gate_tracks_.end(),
                        [&](const DynamicLidarGateTrack& track) {
                            return track.id == passed_track_id;
                        }),
                    dynamic_lidar_gate_tracks_.end());
                if (mixed_mode_enabled()) {
                    leave_mixed_gate_mode(false);
                    if (mixed_closed_semantic_gate_sequence(world_)) {
                        return;
                    }
                }
                gates_.clear();
                return;
            }
        }
    }
}

void PlannerDrivenVehicleSim::update_gate_activation_window() {
    if (!gate_environment(world_) || gates_.empty()) {
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

    const bool compact_world = world_span_m(world_) <= 5.0;
    const double activation_range =
        std::max(active_lidar_range() * 1.20, compact_world ? 1.0 : 6.0);
    for (size_t i = 0; i < gates_.size(); ++i) {
        if (gates_[i].passed) {
            gates_[i].too_far = false;
            gates_[i].choose = false;
            continue;
        }

        const int lookahead_index = static_cast<int>(i) - primary_index;
        const bool is_primary = lookahead_index == 0;
        const bool in_lookahead_window =
            lookahead_index > 0 &&
            lookahead_index <= 2 &&
            distance_to_gate_point(gates_[i], navigation_position_) <= activation_range;
        gates_[i].too_far = !(is_primary || in_lookahead_window);
        if (gates_[i].too_far) {
            gates_[i].choose = false;
        }
    }
}

std::vector<int> PlannerDrivenVehicleSim::active_gate_indices() const {
    std::vector<int> indices;
    if (!gate_environment(world_)) {
        return indices;
    }

    if (mixed_closed_gate_sequence(world_)) {
        for (size_t i = 0; i < gates_.size(); ++i) {
            if (gates_[i].final) {
                continue;
            }
            if (!gates_[i].passed) {
                if (!gates_[i].too_far) {
                    indices.push_back(static_cast<int>(i));
                }
                return indices;
            }
        }
        return indices;
    }

    for (size_t i = 0; i < gates_.size(); ++i) {
        if (!gates_[i].passed && !gates_[i].too_far) {
            indices.push_back(static_cast<int>(i));
        }
    }
    return indices;
}

}  // namespace thesis_sim
