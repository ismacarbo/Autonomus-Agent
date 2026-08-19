#include "support.inc"

namespace thesis_sim {

void HardwarePlannerRunner::rebuild_dynamic_gap_gates(const std::vector<RPLidarA1::ScanPoint>& scan) {
    use_dynamic_gap_gates_ = dynamic_gap_mode_enabled();
    diagnostics_.dynamic_gap_gates = use_dynamic_gap_gates_;
    diagnostics_.candidate_gates = 0;
    diagnostics_.chosen_gate_distance = std::numeric_limits<double>::infinity();
    if (!use_dynamic_gap_gates_) {
        dynamic_gap_tracks_.clear();
        return;
    }
    if (compact_mixed_open_road(world_) &&
        passed_unstructured_gap_count_ >= required_unstructured_gap_pass_count()) {
        clear_locked_gap_goal();
        dynamic_gap_tracks_.clear();
        gates_.clear();
        gate_specs_.clear();
        visible_gate_indices_.clear();
        chosen_gate_index_ = -1;
        diagnostics_.candidate_gates = 0;
        diagnostics_.chosen_gate_distance = std::numeric_limits<double>::infinity();
        return;
    }
    if (world_.environment_mode() == EnvironmentMode::UnstructuredGates &&
        passed_unstructured_gap_count_ >= required_unstructured_gap_pass_count()) {
        dynamic_gap_tracks_.clear();
        gates_.clear();
        gate_specs_.clear();
        visible_gate_indices_.clear();
        chosen_gate_index_ = -1;
        diagnostics_.candidate_gates = 0;
        diagnostics_.chosen_gate_distance = std::numeric_limits<double>::infinity();
        return;
    }

    // An exploration waypoint is not a perceived gate. Remove its temporary
    // planner representation before extracting physical apertures so it can
    // never be persisted or scored as a gate candidate. The frontier itself
    // remains stored separately and is restored by the workflow when no gate
    // preempts it.
    if (navigation_target_kind_ == NavigationTargetKind::ExplorationFrontier) {
        clear_locked_gap_goal();
        gates_.clear();
        gate_specs_.clear();
        chosen_gate_index_ = -1;
    }

    const std::vector<gate> previous_gates = gates_;
    const std::vector<GateSpec> previous_gate_specs = gate_specs_;
    const int previous_chosen_gate_index = chosen_gate_index_;

    struct ScanBeam {
        double local_angle = 0.0;
        double world_angle = 0.0;
        double raw_distance = 0.0;
        double distance = 0.0;
        Vec2 hit;
    };

    struct TargetCandidate {
        double score = 0.0;
        double target_distance = 0.0;
        double center_local_angle = 0.0;
        double center_world_angle = 0.0;
        double progress_score = 0.0;
        double gap_width = 0.0;
        double centerline_clearance = 0.0;
        int track_id = -1;
        int track_hits = 0;
        double track_score = 0.0;
        bool temporally_confirmed = false;
        Vec2 target;
        Vec2 crossing_point;
        bool crossing_point_valid = false;
    };

    struct FreeSpaceSector {
        int start_index = -1;
        int end_index = -1;
        int left_boundary_index = -1;
        int right_boundary_index = -1;
        bool require_depth_contrast = false;
        bool target_near_aperture = false;
        double score_bonus = 0.0;
    };

    if (scan.empty()) {
        return;
    }

    const bool micro_hardware_lab = compact_hardware_mixed_lab_world(world_);
    const bool compact_unstructured_hardware =
        hardware_unstructured_exploration_world(world_);
    const bool manual_exploration =
        open_ended_unstructured_exploration(world_);
    const double current_world_span = world_span_m(world_);
    const bool compact_mixed_reference =
        world_is_mixed(world_) &&
        world_has_structured_reference(world_) &&
        current_world_span <= 2.50 &&
        !world_.road_centerline().empty();
    if (compact_mixed_reference) {
        const double road_block_score =
            compute_mixed_road_block_score(mixed_road_gate_lookahead_m());
        const double micro_road_start_distance =
            micro_hardware_lab
                ? clamp_value(0.10 * std::max(structured_road_length_m(world_), world_span_m(world_)),
                              0.045,
                              0.085)
                : 0.0;
        const bool micro_road_has_started =
            !micro_hardware_lab ||
            structured_progress_s_ >= micro_road_start_distance ||
            (std::isfinite(structured_last_s_) &&
             structured_last_s_ >= micro_road_start_distance);
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
        const bool tracked_lab_scale_mixed =
            config_.vehicle_model == VehicleModelKind::TrackedVehicle &&
            current_world_span <= 1.35;
        const bool tracked_close_obstacle_pressure =
            tracked_lab_scale_mixed &&
            estimate_.min_lidar_distance > 0.0 &&
            estimate_.min_lidar_distance <
                config_.localization.obstacle_stop_distance_m + 0.060;
        const bool front_obstacle_pressure = micro_hardware_lab
            ? lidar_pressure_from_hits(
                  world_,
                  lidar_hits_,
                  estimate_.yaw,
                  config_.localization.obstacle_stop_distance_m + 0.07,
                  config_.localization.front_sector_half_angle_rad,
                  true)
            : ((estimate_.front_lidar_distance > 0.0 &&
                estimate_.front_lidar_distance <
                    config_.localization.obstacle_stop_distance_m + 0.05) ||
               tracked_close_obstacle_pressure);
        const bool square_lab_annulus = current_world_span <= 1.95;
        const double required_block_score = square_lab_annulus ? 0.42 : 0.18;
        if ((micro_hardware_lab && (!micro_road_has_started || micro_near_structured_finish)) ||
            (square_lab_annulus &&
             !locked_gap_goal_.has_value() &&
             !front_obstacle_pressure) ||
            (!square_lab_annulus &&
             !locked_gap_goal_.has_value() &&
             road_block_score < required_block_score &&
             !front_obstacle_pressure)) {
            clear_locked_gap_goal();
            dynamic_gap_tracks_.clear();
            gates_.clear();
            gate_specs_.clear();
            visible_gate_indices_.clear();
            chosen_gate_index_ = -1;
            diagnostics_.candidate_gates = 0;
            diagnostics_.chosen_gate_distance = std::numeric_limits<double>::infinity();
            return;
        }
    }

    const Vec2 lidar_origin = lidar_origin_world(estimate_.position, estimate_.yaw, config_.localization);
    const double planning_range = planning_lidar_range(config_);
    std::vector<ScanBeam> beams;
    beams.reserve(scan.size());
    for (const RPLidarA1::ScanPoint& point : scan) {
        if (point.distance_m <= 0.0 ||
            point.distance_m < config_.localization.min_valid_range_m ||
            point.distance_m > planning_range ||
            scan_point_is_self_hit(point)) {
            continue;
        }
        const double local_angle = lidar_body_local_angle_rad(config_.localization, point.angle_deg);
        const double world_angle = wrap_angle(estimate_.yaw + local_angle);
        beams.push_back({
            local_angle,
            world_angle,
            point.distance_m,
            point.distance_m,
            {
                lidar_origin.x + std::cos(world_angle) * point.distance_m,
                lidar_origin.y + std::sin(world_angle) * point.distance_m,
            },
        });
    }

    if (beams.size() < 6) {
        return;
    }

    std::sort(beams.begin(), beams.end(), [](const ScanBeam& a, const ScanBeam& b) {
        return a.local_angle < b.local_angle;
    });

    const int median_window = std::max(1, config_.gap_extraction.median_filter_window | 1);
    if (median_window > 1 && beams.size() >= static_cast<size_t>(median_window)) {
        const int half_window = median_window / 2;
        std::vector<double> filtered_ranges(beams.size(), 0.0);
        std::vector<double> samples;
        samples.reserve(static_cast<size_t>(median_window));
        for (size_t i = 0; i < beams.size(); ++i) {
            samples.clear();
            const int start = std::max<int>(0, static_cast<int>(i) - half_window);
            const int end = std::min<int>(
                static_cast<int>(beams.size()) - 1,
                static_cast<int>(i) + half_window);
            for (int j = start; j <= end; ++j) {
                samples.push_back(beams[static_cast<size_t>(j)].raw_distance);
            }
            const size_t median_index = samples.size() / 2U;
            std::nth_element(
                samples.begin(),
                samples.begin() + static_cast<std::ptrdiff_t>(median_index),
                samples.end());
            filtered_ranges[i] = samples[median_index];
        }
        for (size_t i = 0; i < beams.size(); ++i) {
            ScanBeam& beam = beams[i];
            beam.distance = filtered_ranges[i];
            beam.hit = {
                lidar_origin.x + std::cos(beam.world_angle) * beam.distance,
                lidar_origin.y + std::sin(beam.world_angle) * beam.distance,
            };
        }
    }

    const double outlier_far_spike_delta =
        std::max(config_.gap_extraction.outlier_filter_far_spike_delta_m, 0.0);
    if (outlier_far_spike_delta > 1e-6 && beams.size() >= 3U) {
        const double neighbor_delta =
            std::max(config_.gap_extraction.outlier_filter_neighbor_delta_m, 0.0);
        const double max_angle_step =
            std::max(config_.gap_extraction.outlier_filter_max_angle_step_rad, 1e-3);
        std::vector<double> corrected_ranges(beams.size(), 0.0);
        for (size_t i = 0; i < beams.size(); ++i) {
            corrected_ranges[i] = beams[i].distance;
        }
        for (size_t i = 1; i + 1 < beams.size(); ++i) {
            const ScanBeam& left = beams[i - 1U];
            const ScanBeam& center = beams[i];
            const ScanBeam& right = beams[i + 1U];
            const double left_angle_step = std::abs(center.local_angle - left.local_angle);
            const double right_angle_step = std::abs(right.local_angle - center.local_angle);
            if (left_angle_step > max_angle_step || right_angle_step > max_angle_step) {
                continue;
            }
            if (std::abs(left.distance - right.distance) > neighbor_delta) {
                continue;
            }
            const double neighbor_range = std::max(left.distance, right.distance);
            if (center.distance - neighbor_range <= outlier_far_spike_delta) {
                continue;
            }
            corrected_ranges[i] = neighbor_range;
        }
        for (size_t i = 0; i < beams.size(); ++i) {
            ScanBeam& beam = beams[i];
            beam.distance = corrected_ranges[i];
            beam.hit = {
                lidar_origin.x + std::cos(beam.world_angle) * beam.distance,
                lidar_origin.y + std::sin(beam.world_angle) * beam.distance,
            };
        }
    }

    const double min_target_distance = micro_hardware_lab
        ? clamp_value(
              0.26 * current_world_span,
              config_.localization.min_valid_range_m + 0.005,
              config_.localization.min_valid_range_m + 0.080)
        : clamp_value(
              config_.gap_extraction.min_target_distance_m,
              std::max(
                  config_.localization.min_valid_range_m + 0.08,
                  std::max(geometry_.cg_to_front + 0.04, 0.20)),
              planning_range * 0.68);
    const double max_target_distance = micro_hardware_lab
        ? clamp_value(
              0.62 * current_world_span,
              min_target_distance + 0.06,
              std::min(planning_range * 0.92, 0.38))
        : clamp_value(
              config_.gap_extraction.max_target_distance_m,
              min_target_distance + 0.05,
              planning_range * 0.92);
    const double free_threshold = micro_hardware_lab
        ? clamp_value(
              0.36 * current_world_span,
              config_.localization.min_valid_range_m + 0.04,
              config_.localization.obstacle_stop_distance_m + 0.02)
        : std::max(
              config_.gap_extraction.free_distance_threshold_m,
              config_.localization.obstacle_stop_distance_m + 0.03);
    const Vec2 global_goal = world_.goal();
    const bool have_global_goal =
        !manual_exploration &&
        is_inside_bounds(world_, global_goal) &&
        distance(lidar_origin, global_goal) > 0.05;
    const double goal_heading = have_global_goal ? angle_to(lidar_origin, global_goal) : estimate_.yaw;
    const double goal_distance_from_origin =
        have_global_goal ? distance(lidar_origin, global_goal) : 0.0;
    std::vector<TargetCandidate> candidates;
    std::optional<GateSpec> persisted_spec;
    std::optional<gate> persisted_gate;
    const auto dynamic_gap_target_allowed = [&](const Vec2& target) {
        if (!dynamic_gap_point_allowed(target)) {
            return false;
        }
        if (!manual_exploration) {
            return true;
        }

        // Open-ended hardware exploration has no authored wall/geofence. Gate
        // safety comes from live LiDAR, occupancy and swept-footprint checks,
        // not from the finite GUI/map canvas.
        return true;
    };

    auto try_persist_previous_gate = [&](int index) {
        if (index < 0 ||
            index >= static_cast<int>(previous_gates.size()) ||
            index >= static_cast<int>(previous_gate_specs.size())) {
            return false;
        }

        const GateSpec& previous_spec = previous_gate_specs[static_cast<size_t>(index)];
        const gate& previous_gate = previous_gates[static_cast<size_t>(index)];
        if (previous_gate.passed || previous_spec.final) {
            return false;
        }

        const double gate_distance = distance(previous_spec.position, estimate_.position);
        if (gate_distance < 0.035 ||
            gate_distance > std::max(config_.gap_extraction.max_target_distance_m * 1.8, 2.0)) {
            return false;
        }

        const double gate_heading_local =
            wrap_angle(angle_to(lidar_origin, previous_spec.position) - estimate_.yaw);
        const double persistence_heading_limit =
            manual_exploration ? 0.92 * kPi : 0.70 * kPi;
        if (std::abs(gate_heading_local) > persistence_heading_limit) {
            return false;
        }

        const bool committed_close_approach =
            manual_exploration &&
            index == previous_chosen_gate_index &&
            gate_distance <=
                std::max(2.0 * config_.gap_extraction.gap_goal_acceptance_radius_m,
                         0.24);
        if (!perception_map_supports_target(lidar_origin, previous_spec.position) &&
            !committed_close_approach) {
            return false;
        }

        persisted_spec = previous_spec;
        persisted_gate = previous_gate;
        return true;
    };

    if (!try_persist_previous_gate(previous_chosen_gate_index)) {
        for (size_t i = 0; i < previous_gates.size(); ++i) {
            if (try_persist_previous_gate(static_cast<int>(i))) {
                break;
            }
        }
    }

    auto continuity_bonus_for_target = [&](const Vec2& target) {
        if (!persisted_spec.has_value()) {
            return 0.0;
        }
        const double target_delta = distance(target, persisted_spec->position);
        if (target_delta > 0.36) {
            return 0.0;
        }
        return 0.85 * (1.0 - clamp_value(target_delta / 0.36, 0.0, 1.0));
    };

    const double grid_resolution = micro_hardware_lab
        ? clamp_value(0.055 * current_world_span, 0.018, 0.032)
        : (compact_unstructured_hardware
               ? clamp_value(
                     config_.gap_extraction.map_point_resolution_m,
                     0.018,
                     0.030)
        : clamp_value(
              config_.gap_extraction.map_point_resolution_m * 1.5,
              0.04,
              0.07));
    const double inflate_radius = micro_hardware_lab
        ? clamp_value(0.09 * current_world_span, 0.035, 0.055)
        : std::max(
              config_.gap_extraction.target_clearance_radius_m,
              geometry_.body_width * 0.55);
    const int half_cells = std::max(
        6,
        static_cast<int>(std::ceil((planning_range + inflate_radius + grid_resolution) / grid_resolution)));
    LocalOccupancyGrid local_grid;
    local_grid.resolution_m = grid_resolution;
    local_grid.width = 2 * half_cells + 1;
    local_grid.height = 2 * half_cells + 1;
    local_grid.origin = {
        lidar_origin.x - static_cast<double>(half_cells) * grid_resolution,
        lidar_origin.y - static_cast<double>(half_cells) * grid_resolution,
    };
    local_grid.free_votes.assign(
        static_cast<size_t>(local_grid.width * local_grid.height),
        0U);
    local_grid.occupied_votes.assign(
        static_cast<size_t>(local_grid.width * local_grid.height),
        0U);

    int free_beam_count = 0;
    for (const ScanBeam& beam : beams) {
        if (beam.distance >= free_threshold) {
            ++free_beam_count;
        }

        const bool obstacle_endpoint =
            beam.distance < planning_range - 0.03 &&
            beam.distance < config_.localization.max_range_m - 0.03;
        const double free_distance = obstacle_endpoint
            ? std::max(0.04, beam.distance - inflate_radius * 0.60)
            : beam.distance;
        const Vec2 free_endpoint{
            lidar_origin.x + std::cos(beam.world_angle) * free_distance,
            lidar_origin.y + std::sin(beam.world_angle) * free_distance,
        };
        sample_segment_points(
            lidar_origin,
            free_endpoint,
            grid_resolution * 0.65,
            true,
            [&](const Vec2& sample) {
                local_grid.add_free_sample(sample);
            });

        if (obstacle_endpoint) {
            local_grid.add_occupied_sample(beam.hit);
        }
    }

    sample_segment_points(
        {lidar_origin.x - 0.04, lidar_origin.y},
        {lidar_origin.x + 0.04, lidar_origin.y},
        grid_resolution * 0.5,
        true,
        [&](const Vec2& sample) {
            local_grid.add_free_sample(sample);
        });
    sample_segment_points(
        {lidar_origin.x, lidar_origin.y - 0.04},
        {lidar_origin.x, lidar_origin.y + 0.04},
        grid_resolution * 0.5,
        true,
        [&](const Vec2& sample) {
            local_grid.add_free_sample(sample);
        });

    const double map_overlay_radius_sq =
        (planning_range + inflate_radius) * (planning_range + inflate_radius);
    for (const Vec2& map_point : lidar_map_points_) {
        if (distance_sq(map_point, lidar_origin) <= map_overlay_radius_sq) {
            local_grid.add_occupied_sample(map_point);
        }
    }
    local_grid.finalize(inflate_radius);

    if (locked_gap_goal_.has_value()) {
        const double gap_distance = distance(estimate_.position, *locked_gap_goal_);
        const double longitudinal_progress = locked_gap_longitudinal_progress(estimate_.position);
        const double lateral_offset = std::abs(locked_gap_lateral_offset(estimate_.position));
        const double crossing_margin =
            std::max(config_.gap_extraction.gap_crossing_margin_m, 0.0);
        const double gate_heading_local =
            std::abs(wrap_angle(angle_to(lidar_origin, *locked_gap_goal_) - estimate_.yaw));
        const double missed_gap_lateral_limit =
            locked_gap_corridor_half_width_m_ +
            std::max(config_.gap_extraction.target_clearance_radius_m, 0.04);
        const bool missed_locked_gap =
            gap_distance <= std::max(config_.gap_extraction.gap_goal_tolerance_m, 0.10) &&
            longitudinal_progress < crossing_margin &&
            lateral_offset > missed_gap_lateral_limit;
        const bool committed_compact_open_gate =
            compact_mixed_open_road(world_) &&
            !missed_locked_gap &&
            gap_distance <= std::max(config_.gap_extraction.max_target_distance_m * 1.15, 0.75) &&
            sim_time_ - locked_gap_set_time_s_ <= 25.0;
        if (committed_compact_open_gate) {
            locked_gap_invalid_streak_ = 0;
            publish_locked_gap_goal();
            return;
        }
        const bool grid_supports_locked_gap =
            grid_segment_is_clear(local_grid, lidar_origin, *locked_gap_goal_);
        const bool perception_supports_locked_gap =
            perception_map_supports_target(lidar_origin, *locked_gap_goal_);
        const bool scan_supports_locked_gap =
            scan_supports_target(*locked_gap_goal_, scan);
        const bool soft_hold_front_open =
            estimate_.front_lidar_distance <= 0.0 ||
            estimate_.front_lidar_distance >=
                config_.localization.obstacle_stop_distance_m + 0.03;
        const bool soft_hold_gap_close =
            gap_distance <=
            std::max(config_.gap_extraction.max_target_distance_m * 1.25, 0.70);
        const bool soft_hold_gap_near =
            gap_distance <=
            std::max(config_.gap_extraction.max_target_distance_m * 0.95, 0.60);
        const bool soft_hold_heading_ok =
            gate_heading_local <= config_.gap_extraction.locked_gap_soft_hold_heading_rad;
        const bool strong_support_locked_gap =
            (scan_supports_locked_gap &&
             (grid_supports_locked_gap || perception_supports_locked_gap)) ||
            (grid_supports_locked_gap && perception_supports_locked_gap);
        const bool soft_hold_support_locked_gap =
            scan_supports_locked_gap ||
            grid_supports_locked_gap ||
            perception_supports_locked_gap;
        const bool soft_hold_locked_gap =
            !missed_locked_gap &&
            soft_hold_gap_close &&
            soft_hold_heading_ok &&
            (soft_hold_support_locked_gap || soft_hold_gap_near) &&
            (soft_hold_front_open || gate_heading_local <= 0.75 || soft_hold_gap_near);
        const bool committed_lab_mixed_gate =
            world_is_mixed(world_) &&
            world_span_m(world_) <= 1.25 &&
            !missed_locked_gap &&
            dynamic_gap_target_allowed(*locked_gap_goal_) &&
            gap_distance <= std::max(config_.gap_extraction.max_target_distance_m * 1.10, 0.70) &&
            gate_heading_local <= 2.35;
        const bool committed_manual_gate =
            manual_exploration &&
            dynamic_gap_target_allowed(*locked_gap_goal_) &&
            gap_distance <= std::max(
                config_.gap_extraction.max_target_distance_m * 1.40,
                0.80) &&
            sim_time_ - locked_gap_set_time_s_ <= 30.0;

        if (!missed_locked_gap && strong_support_locked_gap) {
            locked_gap_invalid_streak_ = 0;
            publish_locked_gap_goal();
            return;
        }

        if (committed_manual_gate) {
            // Manual Editor apertures are static. Close-range occlusion and a
            // temporary loss of both boundary returns must not destroy the
            // selected clothoid before the centre of the robot crosses the
            // gate plane. Crossing is evaluated before every rebuild.
            locked_gap_invalid_streak_ = 0;
            publish_locked_gap_goal();
            return;
        }

        if ((soft_hold_locked_gap || committed_lab_mixed_gate) &&
            locked_gap_invalid_streak_ < std::max(1, config_.gap_extraction.locked_gap_grace_frames)) {
            const bool blocked_mixed_target =
                world_is_mixed(world_) &&
                !strong_support_locked_gap &&
                estimate_.front_lidar_distance > 0.0 &&
                estimate_.front_lidar_distance <
                    config_.localization.obstacle_stop_distance_m + 0.03;
            locked_gap_invalid_streak_ += blocked_mixed_target ? 8 : 1;
            publish_locked_gap_goal();
            return;
        }

        clear_locked_gap_goal();
        if (strict_locked_gate_motion_enabled()) {
            restart_unstructured_scan();
            return;
        }
    }

    double beam_angle_resolution = 2.0 * kPi / static_cast<double>(std::max<size_t>(beams.size(), 360U));
    if (beams.size() > 1) {
        double angular_sum = 0.0;
        int angular_count = 0;
        for (size_t i = 1; i < beams.size(); ++i) {
            const double delta = beams[i].local_angle - beams[i - 1].local_angle;
            if (delta > 1e-4) {
                angular_sum += delta;
                ++angular_count;
            }
        }
        if (angular_count > 0) {
            beam_angle_resolution = std::max(
                angular_sum / static_cast<double>(angular_count),
                1.0 * kPi / 180.0);
        }
    }
    const double required_gap_width = micro_hardware_lab
        ? clamp_value(
              0.26 * current_world_span,
              0.105,
              0.160)
        : (compact_unstructured_hardware
               ? std::max(
                     config_.gap_extraction.min_gap_width_m,
                     geometry_.body_width + 0.04)
        : std::max(
              config_.gap_extraction.min_gap_width_m,
              geometry_.body_width + 2.0 * config_.gap_extraction.path_clearance_radius_m));
    const double passed_gap_reject_radius = std::max(
        config_.gap_extraction.gap_goal_acceptance_radius_m,
        std::max(config_.gap_extraction.gap_goal_tolerance_m * 1.15, 0.24));

    auto try_add_candidate = [&](const Vec2& target,
                                 const Vec2& crossing_point,
                                 double base_score,
                                 double observed_gap_width) {
        if (!dynamic_gap_target_allowed(target)) {
            return;
        }

        for (const Vec2& passed_target : passed_unstructured_gap_positions_) {
            if (distance(passed_target, target) <= passed_gap_reject_radius) {
                return;
            }
        }

        const double target_distance = distance(lidar_origin, target);
        if (target_distance < min_target_distance || target_distance > max_target_distance) {
            return;
        }

        const double candidate_world_angle = angle_to(lidar_origin, target);
        const double candidate_local_angle = wrap_angle(candidate_world_angle - estimate_.yaw);
        const double centerline_half_angle = clamp_value(
            std::atan2(0.5 * geometry_.body_width, std::max(target_distance, 0.10)),
            5.0 * kPi / 180.0,
            14.0 * kPi / 180.0);
        const double centerline_clearance = sector_min_clearance(
            lidar_hits_,
            estimate_.yaw,
            candidate_local_angle,
            centerline_half_angle,
            planning_range);
        const bool object_before_target =
            centerline_clearance + 0.04 < target_distance &&
            centerline_clearance <
                std::max(config_.localization.obstacle_stop_distance_m + 0.10,
                         config_.localization.min_valid_range_m + 0.14);
        if (object_before_target) {
            return;
        }

        if (!grid_segment_is_clear(local_grid, lidar_origin, target)) {
            return;
        }

        if (!perception_map_supports_target(lidar_origin, target)) {
            return;
        }

        if (!scan_supports_target(target, scan)) {
            return;
        }

        int cell_x = 0;
        int cell_y = 0;
        if (!local_grid.world_to_cell(target, &cell_x, &cell_y)) {
            return;
        }
        if (local_grid.free_mask[static_cast<size_t>(local_grid.index(cell_x, cell_y))] == 0U) {
            return;
        }

        const double abs_candidate_local_angle = std::abs(candidate_local_angle);
        const double distance_score = clamp_value(
            (target_distance - min_target_distance) /
                std::max(max_target_distance - min_target_distance, 0.05),
            0.0,
            1.0);
        const double forward_alignment = 0.5 * (1.0 + std::cos(candidate_local_angle));
        const double progress_score =
            clamp_value(std::cos(candidate_local_angle), 0.0, 1.0) * distance_score;
        const double goal_alignment = have_global_goal
            ? 0.5 * (1.0 + std::cos(wrap_angle(candidate_world_angle - goal_heading)))
            : forward_alignment;
        const double goal_progress = have_global_goal
            ? clamp_value(
                  (goal_distance_from_origin - distance(target, global_goal)) /
                      std::max(max_target_distance, 0.05),
                  -0.35,
                  1.0)
            : progress_score;
        const double clearance_score = grid_local_clearance_score(
            local_grid,
            cell_x,
            cell_y,
            std::max(1, static_cast<int>(std::ceil(inflate_radius / grid_resolution))));
        const double lateral_penalty_start =
            (manual_exploration ? 95.0 : 55.0) * kPi / 180.0;
        const double lateral_penalty_span =
            (manual_exploration ? 60.0 : 45.0) * kPi / 180.0;
        const double lateral_heading_penalty = clamp_value(
            (abs_candidate_local_angle - lateral_penalty_start) /
                lateral_penalty_span,
            0.0,
            1.6);
        const double reverse_penalty_start =
            (manual_exploration ? 155.0 : 95.0) * kPi / 180.0;
        const double reverse_heading_penalty = clamp_value(
            (abs_candidate_local_angle - reverse_penalty_start) /
                (35.0 * kPi / 180.0),
            0.0,
            1.8);
        const double rear_search_score =
            strict_locked_gate_motion_enabled()
                ? clamp_value(
                      (abs_candidate_local_angle - 95.0 * kPi / 180.0) /
                          (70.0 * kPi / 180.0),
                      0.0,
                      1.0) *
                      clamp_value(
                          unstructured_no_candidate_scan_elapsed_s_ /
                              std::max(config_.gap_extraction.strict_scan_escape_after_s, 1.0),
                          0.0,
                          1.0)
                : 0.0;

        const double coverage_bonus = manual_exploration
            ? 0.70 * clamp_value(
                  (abs_candidate_local_angle - 55.0 * kPi / 180.0) /
                      (90.0 * kPi / 180.0),
                  0.0,
                  1.0)
            : 0.0;
        TargetCandidate candidate{
            base_score +
                (manual_exploration ? 0.75 : 1.95) * progress_score +
                1.55 * distance_score +
                (manual_exploration ? 0.45 : 1.35) * forward_alignment +
                (manual_exploration ? 0.40 : 1.20) * goal_alignment +
                (manual_exploration ? 0.30 : 1.25) * goal_progress +
                1.10 * clearance_score + coverage_bonus +
                continuity_bonus_for_target(target) + 1.25 * rear_search_score -
                1.45 * lateral_heading_penalty -
                1.70 * reverse_heading_penalty,
            target_distance,
            candidate_local_angle,
            candidate_world_angle,
            progress_score,
            observed_gap_width,
            centerline_clearance,
            -1,
            0,
            0.0,
            false,
            target,
        };
        candidate.crossing_point = crossing_point;
        candidate.crossing_point_valid = true;
        candidates.push_back(candidate);
    };

    if (persisted_spec.has_value()) {
        const Vec2 target = persisted_spec->position;
        const double persisted_distance = distance(lidar_origin, target);
        if (persisted_distance >= 0.10 &&
            persisted_distance <= std::max(max_target_distance * 1.2, 1.1) &&
            grid_segment_is_clear(local_grid, lidar_origin, target) &&
            scan_supports_target(target, scan)) {
            try_add_candidate(
                target,
                persisted_spec->anchor_position,
                1.40,
                config_.gap_extraction.min_gap_width_m);
        }
    }

    if (!manual_exploration &&
        world_.environment_mode() == EnvironmentMode::UnstructuredGates &&
        passed_unstructured_gap_count_ > 0 &&
        have_global_goal) {
        // Once at least one real opening has been crossed, admit the physical
        // mission goal as the next target only through the same LiDAR,
        // occupancy-grid and persistent-map checks used for every dynamic
        // candidate.  This prevents an open Hardware Lab from demanding a
        // second arbitrary free-sector gate (or succeeding on a false gap
        // caused by localization drift) while still forbidding a shortcut
        // through an obstacle.
        const Vec2 goal_direction = normalize_with_fallback(
            Vec2{global_goal.x - lidar_origin.x, global_goal.y - lidar_origin.y},
            Vec2{std::cos(estimate_.yaw), std::sin(estimate_.yaw)});
        const double goal_entry_radius = clamp_value(
            config_.gap_extraction.gap_goal_acceptance_radius_m,
            0.07,
            0.12);
        const Vec2 goal_entry_plane{
            global_goal.x - goal_entry_radius * goal_direction.x,
            global_goal.y - goal_entry_radius * goal_direction.y,
        };
        try_add_candidate(
            global_goal,
            goal_entry_plane,
            4.50,
            std::max(required_gap_width, geometry_.body_width + 0.08));
    }

    const double min_depth_contrast = micro_hardware_lab
        ? clamp_value(0.16 * current_world_span, 0.045, 0.095)
        : std::max(
              config_.gap_extraction.min_gap_depth_contrast_m,
              0.35 * required_gap_width);
    double nearest_beam_distance = planning_range;
    std::vector<double> beam_distances;
    beam_distances.reserve(beams.size());
    for (const ScanBeam& beam : beams) {
        nearest_beam_distance = std::min(nearest_beam_distance, beam.distance);
        beam_distances.push_back(beam.distance);
    }
    double range_quantile_threshold = free_threshold;
    if (!beam_distances.empty()) {
        const size_t quantile_index = std::min(
            beam_distances.size() - 1U,
            static_cast<size_t>(std::floor(0.40 * static_cast<double>(beam_distances.size() - 1U))));
        std::nth_element(
            beam_distances.begin(),
            beam_distances.begin() + static_cast<std::ptrdiff_t>(quantile_index),
            beam_distances.end());
        range_quantile_threshold = beam_distances[quantile_index];
    }
    const double close_scene_margin = clamp_value(0.28 * required_gap_width, 0.09, 0.14);
    const double regular_scene_margin = clamp_value(0.48 * required_gap_width, 0.16, 0.26);
    const double free_sector_threshold =
        nearest_beam_distance < free_threshold + 0.10
            ? clamp_value(
                  nearest_beam_distance + close_scene_margin,
                  free_threshold,
                  std::max(free_threshold, planning_range - 0.05))
            : clamp_value(
                  std::max(
                      std::min(
                          range_quantile_threshold,
                          nearest_beam_distance + 0.70 * required_gap_width),
                      nearest_beam_distance + regular_scene_margin),
                  free_threshold,
                  std::max(free_threshold, planning_range - 0.05));

    const auto aperture_geometry_is_plausible =
        [&](double angular_span,
            const ScanBeam& left_boundary,
            const ScanBeam& right_boundary,
            const Vec2& aperture_center) {
            if (!compact_unstructured_hardware) {
                return true;
            }

            // A very wide free sector can have obstacle endpoints on opposite
            // sides of the LiDAR. Their Euclidean midpoint then collapses onto
            // the sensor and looks like a gate plane underneath the robot. In
            // run 211728 this produced a 158 degree "gate" whose midpoint was
            // only 2.6 cm from the LiDAR and was subsequently counted as
            // crossed. Real apertures observed before commitment retain a
            // meaningful midpoint distance relative to both boundary ranges.
            const double aperture_distance = distance(lidar_origin, aperture_center);
            const double boundary_reference_distance = std::max(
                std::min(left_boundary.distance, right_boundary.distance),
                config_.localization.min_valid_range_m);
            const double midpoint_range_ratio =
                aperture_distance / boundary_reference_distance;
            const double minimum_aperture_distance = std::max(
                config_.localization.min_valid_range_m,
                0.12);
            const double maximum_aperture_span = 120.0 * kPi / 180.0;
            return aperture_distance >= minimum_aperture_distance &&
                   midpoint_range_ratio >= 0.35 &&
                   angular_span <= maximum_aperture_span;
        };

    auto add_gap_sector_candidate = [&](int start_index,
                                        int end_index,
                                        int left_boundary_index,
                                        int right_boundary_index,
                                        bool require_depth_contrast,
                                        bool target_near_aperture,
                                        double range_jump_bonus) {
        if (start_index < 0 || end_index < start_index ||
            end_index >= static_cast<int>(beams.size()) ||
            left_boundary_index < 0 ||
            right_boundary_index >= static_cast<int>(beams.size()) ||
            left_boundary_index >= start_index ||
            right_boundary_index <= end_index) {
            return;
        }

        double min_sector_distance = std::numeric_limits<double>::infinity();
        double sum_sector_distance = 0.0;
        for (int i = start_index; i <= end_index; ++i) {
            min_sector_distance = std::min(min_sector_distance, beams[static_cast<size_t>(i)].distance);
            sum_sector_distance += beams[static_cast<size_t>(i)].distance;
        }
        const int sector_beam_count = end_index - start_index + 1;
        if (sector_beam_count <= 0 || !std::isfinite(min_sector_distance)) {
            return;
        }

        const double start_angle = beams[static_cast<size_t>(start_index)].local_angle;
        const double end_angle = beams[static_cast<size_t>(end_index)].local_angle;
        const double angular_span = std::max(
            end_angle - start_angle + beam_angle_resolution,
            beam_angle_resolution);
        if (angular_span < std::max(config_.gap_extraction.min_gap_angle_rad, beam_angle_resolution)) {
            return;
        }

        const ScanBeam& left_boundary = beams[static_cast<size_t>(left_boundary_index)];
        const ScanBeam& right_boundary = beams[static_cast<size_t>(right_boundary_index)];
        const bool have_left_obstacle =
            left_boundary.distance < planning_range - 0.03 &&
            left_boundary.distance < config_.localization.max_range_m - 0.03;
        const bool have_right_obstacle =
            right_boundary.distance < planning_range - 0.03 &&
            right_boundary.distance < config_.localization.max_range_m - 0.03;
        if (!have_left_obstacle || !have_right_obstacle) {
            return;
        }

        const double depth_contrast =
            min_sector_distance - std::max(left_boundary.distance, right_boundary.distance);
        const bool near_obstacle_boundaries =
            left_boundary.distance < free_sector_threshold &&
            right_boundary.distance < free_sector_threshold;
        if (require_depth_contrast &&
            !near_obstacle_boundaries &&
            depth_contrast < min_depth_contrast) {
            return;
        }

        const double gap_width = distance(left_boundary.hit, right_boundary.hit);
        if (!(gap_width >= required_gap_width)) {
            return;
        }

        // A range discontinuity often leaves the two aperture posts at
        // different distances.  Averaging their angles biases the target
        // toward the closer post.  Use the actual Euclidean midpoint instead.
        const Vec2 aperture_center{
            0.5 * (left_boundary.hit.x + right_boundary.hit.x),
            0.5 * (left_boundary.hit.y + right_boundary.hit.y),
        };
        const double aperture_distance = distance(lidar_origin, aperture_center);
        if (!(aperture_distance > 1e-6) ||
            !aperture_geometry_is_plausible(
                angular_span,
                left_boundary,
                right_boundary,
                aperture_center)) {
            return;
        }
        const double center_world_angle = angle_to(lidar_origin, aperture_center);
        const double center_local_angle = wrap_angle(center_world_angle - estimate_.yaw);

        const double mean_sector_distance = sum_sector_distance / static_cast<double>(sector_beam_count);
        const double support_distance = 0.65 * min_sector_distance + 0.35 * mean_sector_distance;
        const double target_distance_cap = std::min(
            max_target_distance,
            support_distance - std::max(inflate_radius * 0.55, 0.035));
        if (!(target_distance_cap >= min_target_distance)) {
            return;
        }

        double nominal_target_distance = config_.gap_extraction.target_distance_scale * support_distance;
        if (target_near_aperture) {
            const double aperture_margin = micro_hardware_lab
                ? clamp_value(0.07 * current_world_span, 0.025, 0.050)
                : clamp_value(
                      config_.gap_extraction.gap_aperture_target_margin_m,
                      0.03,
                      0.18);
            nominal_target_distance = aperture_distance + aperture_margin;
        }
        const double target_distance = clamp_value(
            nominal_target_distance,
            min_target_distance,
            target_distance_cap);
        const Vec2 target{
            lidar_origin.x + std::cos(center_world_angle) * target_distance,
            lidar_origin.y + std::sin(center_world_angle) * target_distance,
        };
        const double width_score = clamp_value(
            (gap_width - required_gap_width) / std::max(required_gap_width, 0.05),
            0.0,
            1.0);
        const double depth_score = clamp_value(
            depth_contrast / std::max(config_.gap_extraction.max_target_distance_m, 0.10),
            0.0,
            1.4);
        const double forward_alignment = 0.5 * (1.0 + std::cos(center_local_angle));
        const double goal_alignment = have_global_goal
            ? 0.5 * (1.0 + std::cos(wrap_angle(center_world_angle - goal_heading)))
            : forward_alignment;
        const double near_obstacle_aperture_score = clamp_value(
            (0.95 - std::max(left_boundary.distance, right_boundary.distance)) / 0.55,
            0.0,
            1.0);
        try_add_candidate(
            target,
            aperture_center,
            range_jump_bonus + 1.80 * width_score + 0.85 * depth_score +
                1.30 * goal_alignment + 0.90 * forward_alignment +
                0.65 * near_obstacle_aperture_score,
            gap_width);
    };

    auto add_wrapped_gap_sector_candidate = [&](int tail_start_index,
                                                int tail_end_index,
                                                int head_start_index,
                                                int head_end_index,
                                                int left_boundary_index,
                                                int right_boundary_index,
                                                bool require_depth_contrast,
                                                bool target_near_aperture,
                                                double range_jump_bonus) {
        if (tail_start_index < 0 ||
            tail_end_index < tail_start_index ||
            head_start_index < 0 ||
            head_end_index < head_start_index ||
            tail_end_index >= static_cast<int>(beams.size()) ||
            head_end_index >= static_cast<int>(beams.size()) ||
            left_boundary_index < 0 ||
            right_boundary_index >= static_cast<int>(beams.size()) ||
            left_boundary_index >= tail_start_index ||
            right_boundary_index <= head_end_index) {
            return;
        }

        double min_sector_distance = std::numeric_limits<double>::infinity();
        double sum_sector_distance = 0.0;
        int sector_beam_count = 0;
        auto accumulate_sector = [&](int start_index, int end_index) {
            for (int i = start_index; i <= end_index; ++i) {
                min_sector_distance = std::min(min_sector_distance, beams[static_cast<size_t>(i)].distance);
                sum_sector_distance += beams[static_cast<size_t>(i)].distance;
                ++sector_beam_count;
            }
        };
        accumulate_sector(tail_start_index, tail_end_index);
        accumulate_sector(head_start_index, head_end_index);
        if (sector_beam_count <= 0 || !std::isfinite(min_sector_distance)) {
            return;
        }

        const double start_angle = beams[static_cast<size_t>(tail_start_index)].local_angle;
        const double end_angle = beams[static_cast<size_t>(head_end_index)].local_angle + 2.0 * kPi;
        const double angular_span = std::max(
            end_angle - start_angle + beam_angle_resolution,
            beam_angle_resolution);
        if (angular_span < std::max(config_.gap_extraction.min_gap_angle_rad, beam_angle_resolution)) {
            return;
        }

        const ScanBeam& left_boundary = beams[static_cast<size_t>(left_boundary_index)];
        const ScanBeam& right_boundary = beams[static_cast<size_t>(right_boundary_index)];
        const bool have_left_obstacle =
            left_boundary.distance < planning_range - 0.03 &&
            left_boundary.distance < config_.localization.max_range_m - 0.03;
        const bool have_right_obstacle =
            right_boundary.distance < planning_range - 0.03 &&
            right_boundary.distance < config_.localization.max_range_m - 0.03;
        if (!have_left_obstacle || !have_right_obstacle) {
            return;
        }

        const double depth_contrast =
            min_sector_distance - std::max(left_boundary.distance, right_boundary.distance);
        const bool near_obstacle_boundaries =
            left_boundary.distance < free_sector_threshold &&
            right_boundary.distance < free_sector_threshold;
        if (require_depth_contrast &&
            !near_obstacle_boundaries &&
            depth_contrast < min_depth_contrast) {
            return;
        }

        const double gap_width = distance(left_boundary.hit, right_boundary.hit);
        if (!(gap_width >= required_gap_width)) {
            return;
        }

        const Vec2 aperture_center{
            0.5 * (left_boundary.hit.x + right_boundary.hit.x),
            0.5 * (left_boundary.hit.y + right_boundary.hit.y),
        };
        const double aperture_distance = distance(lidar_origin, aperture_center);
        if (!(aperture_distance > 1e-6) ||
            !aperture_geometry_is_plausible(
                angular_span,
                left_boundary,
                right_boundary,
                aperture_center)) {
            return;
        }
        const double center_world_angle = angle_to(lidar_origin, aperture_center);
        const double center_local_angle = wrap_angle(center_world_angle - estimate_.yaw);
        const double mean_sector_distance = sum_sector_distance / static_cast<double>(sector_beam_count);
        const double support_distance = 0.65 * min_sector_distance + 0.35 * mean_sector_distance;
        const double target_distance_cap = std::min(
            max_target_distance,
            support_distance - std::max(inflate_radius * 0.55, 0.035));
        if (!(target_distance_cap >= min_target_distance)) {
            return;
        }

        double nominal_target_distance = config_.gap_extraction.target_distance_scale * support_distance;
        if (target_near_aperture) {
            const double aperture_margin = micro_hardware_lab
                ? clamp_value(0.07 * current_world_span, 0.025, 0.050)
                : clamp_value(
                      config_.gap_extraction.gap_aperture_target_margin_m,
                      0.03,
                      0.18);
            nominal_target_distance = aperture_distance + aperture_margin;
        }

        const double target_distance = clamp_value(
            nominal_target_distance,
            min_target_distance,
            target_distance_cap);
        const Vec2 target{
            lidar_origin.x + std::cos(center_world_angle) * target_distance,
            lidar_origin.y + std::sin(center_world_angle) * target_distance,
        };
        const double width_score = clamp_value(
            (gap_width - required_gap_width) / std::max(required_gap_width, 0.05),
            0.0,
            1.0);
        const double depth_score = clamp_value(
            depth_contrast / std::max(config_.gap_extraction.max_target_distance_m, 0.10),
            0.0,
            1.4);
        const double forward_alignment = 0.5 * (1.0 + std::cos(center_local_angle));
        const double goal_alignment = have_global_goal
            ? 0.5 * (1.0 + std::cos(wrap_angle(center_world_angle - goal_heading)))
            : forward_alignment;
        const double near_obstacle_aperture_score = clamp_value(
            (0.95 - std::max(left_boundary.distance, right_boundary.distance)) / 0.55,
            0.0,
            1.0);
        const double rear_aperture_score = clamp_value(
            (std::abs(center_local_angle) - 110.0 * kPi / 180.0) /
                (55.0 * kPi / 180.0),
            0.0,
            1.0);
        try_add_candidate(
            target,
            aperture_center,
            range_jump_bonus + 1.80 * width_score + 0.85 * depth_score +
                1.30 * goal_alignment + 0.90 * forward_alignment +
                0.65 * near_obstacle_aperture_score + 0.75 * rear_aperture_score,
            gap_width);
    };

    std::vector<FreeSpaceSector> free_space_sectors;
    int free_sector_start = -1;
    for (int i = 0; i < static_cast<int>(beams.size()); ++i) {
        const bool beam_is_free = beams[static_cast<size_t>(i)].distance >= free_sector_threshold;
        if (beam_is_free) {
            if (free_sector_start < 0) {
                free_sector_start = i;
            }
            continue;
        }
        if (free_sector_start >= 0) {
            free_space_sectors.push_back({
                free_sector_start,
                i - 1,
                free_sector_start - 1,
                i,
                false,
                true,
                0.0,
            });
            free_sector_start = -1;
        }
    }
    if (free_sector_start >= 0) {
        free_space_sectors.push_back({
            free_sector_start,
            static_cast<int>(beams.size()) - 1,
            free_sector_start - 1,
            static_cast<int>(beams.size()),
            false,
            true,
            0.0,
        });
    }

    int range_jump_sector_start = -1;
    int range_jump_left_boundary = -1;
    for (int i = 1; i < static_cast<int>(beams.size()); ++i) {
        const double opening_jump =
            beams[static_cast<size_t>(i)].distance -
            beams[static_cast<size_t>(i - 1)].distance;
        const double closing_jump =
            beams[static_cast<size_t>(i - 1)].distance -
            beams[static_cast<size_t>(i)].distance;

        if (range_jump_sector_start < 0) {
            if (opening_jump >= min_depth_contrast &&
                beams[static_cast<size_t>(i - 1)].distance < planning_range - 0.03) {
                range_jump_sector_start = i;
                range_jump_left_boundary = i - 1;
            }
            continue;
        }

        if (closing_jump >= min_depth_contrast &&
            beams[static_cast<size_t>(i)].distance < planning_range - 0.03) {
            free_space_sectors.push_back({
                range_jump_sector_start,
                i - 1,
                range_jump_left_boundary,
                i,
                true,
                true,
                0.45,
            });
            range_jump_sector_start = -1;
            range_jump_left_boundary = -1;
        }
    }

    if (!beams.empty() &&
        beams.front().distance >= free_sector_threshold &&
        beams.back().distance >= free_sector_threshold) {
        int head_end_index = 0;
        while (head_end_index + 1 < static_cast<int>(beams.size()) &&
               beams[static_cast<size_t>(head_end_index + 1)].distance >= free_sector_threshold) {
            ++head_end_index;
        }

        int tail_start_index = static_cast<int>(beams.size()) - 1;
        while (tail_start_index - 1 >= 0 &&
               beams[static_cast<size_t>(tail_start_index - 1)].distance >= free_sector_threshold) {
            --tail_start_index;
        }

        if (head_end_index + 1 < tail_start_index) {
            add_wrapped_gap_sector_candidate(
                tail_start_index,
                static_cast<int>(beams.size()) - 1,
                0,
                head_end_index,
                tail_start_index - 1,
                head_end_index + 1,
                false,
                true,
                0.25);
        }
    }

    for (const FreeSpaceSector& sector : free_space_sectors) {
        add_gap_sector_candidate(
            sector.start_index,
            sector.end_index,
            sector.left_boundary_index,
            sector.right_boundary_index,
            sector.require_depth_contrast,
            sector.target_near_aperture,
            sector.score_bonus);
    }

    const bool lab_scale_mixed_world_for_candidates =
        world_is_mixed(world_) && world_span_m(world_) <= 1.25;
    const bool lab_scale_front_pressure_for_candidates =
        micro_hardware_lab
            ? lidar_pressure_from_hits(
                  world_,
                  lidar_hits_,
                  estimate_.yaw,
                  config_.localization.obstacle_stop_distance_m + 0.07,
                  config_.localization.front_sector_half_angle_rad,
                  true)
            : (estimate_.front_lidar_distance > 0.0 &&
               estimate_.front_lidar_distance <
                   config_.localization.obstacle_stop_distance_m + 0.04);
    if (world_is_mixed(world_) &&
        compute_mixed_road_block_score(mixed_road_gate_lookahead_m()) >= 0.12 &&
        (!lab_scale_mixed_world_for_candidates || lab_scale_front_pressure_for_candidates)) {
        const Vec2 forward{std::cos(estimate_.yaw), std::sin(estimate_.yaw)};
        const Vec2 lateral{-forward.y, forward.x};
        const std::array<double, 2> lateral_signs{{1.0, -1.0}};
        const std::array<double, 2> lateral_offsets{{
            micro_hardware_lab ? 0.11 : 0.42,
            micro_hardware_lab ? 0.08 : 0.34,
        }};
        const std::array<double, 2> forward_offsets{{
            micro_hardware_lab ? 0.14 : 0.52,
            micro_hardware_lab ? 0.20 : 0.68,
        }};
        for (double side : lateral_signs) {
            for (double ahead : forward_offsets) {
                for (double lateral_offset : lateral_offsets) {
                    const Vec2 target{
                        estimate_.position.x + forward.x * ahead + lateral.x * side * lateral_offset,
                        estimate_.position.y + forward.y * ahead + lateral.y * side * lateral_offset,
                    };
                    if (!dynamic_gap_target_allowed(target)) {
                        continue;
                    }
                    const double target_distance = distance(lidar_origin, target);
                    if (target_distance < min_target_distance || target_distance > max_target_distance) {
                        continue;
                    }
                    const double target_heading = angle_to(lidar_origin, target);
                    const double target_local_heading = wrap_angle(target_heading - estimate_.yaw);
                    if (std::abs(target_local_heading) > 82.0 * kPi / 180.0) {
                        continue;
                    }
                    if (!world_.line_of_sight(lidar_origin, target, micro_hardware_lab ? 0.025 : 0.08)) {
                        continue;
                    }
                    const double heading_score =
                        clamp_value(std::cos(target_local_heading), 0.0, 1.0);
                    const double goal_progress = have_global_goal
                        ? clamp_value(
                              (goal_distance_from_origin - distance(target, global_goal)) /
                                  std::max(max_target_distance, 0.05),
                              -0.20,
                              1.0)
                        : 0.0;
                    candidates.push_back({
                        6.0 + 1.2 * heading_score + 0.9 * goal_progress,
                        target_distance,
                        target_local_heading,
                        target_heading,
                        std::max(0.0, goal_progress),
                        required_gap_width,
                        sector_min_clearance(
                            lidar_hits_,
                            estimate_.yaw,
                            target_local_heading,
                            0.20,
                            planning_range),
                        -1,
                        0,
                        0.0,
                        false,
                        target,
                    });
                }
            }
        }
    }

    if (candidates.empty()) {
        for (DynamicGapTrack& track : dynamic_gap_tracks_) {
            ++track.misses;
            track.score = std::max(0.0, track.score - config_.gap_extraction.gap_track_miss_decay);
        }
        dynamic_gap_tracks_.erase(
            std::remove_if(
                dynamic_gap_tracks_.begin(),
                dynamic_gap_tracks_.end(),
                [&](const DynamicGapTrack& track) {
                    return track.misses > std::max(config_.gap_extraction.gap_track_max_misses, 1) ||
                           track.score <= 0.0;
                }),
            dynamic_gap_tracks_.end());

        gates_.clear();
        gate_specs_.clear();
        bool persisted_track_held = false;
        if (persisted_spec.has_value()) {
            for (const DynamicGapTrack& track : dynamic_gap_tracks_) {
                if (distance(track.position, persisted_spec->position) <=
                        std::max(config_.gap_extraction.gap_track_match_radius_m, 0.08) &&
                    track.score >= std::max(config_.gap_extraction.gap_track_hold_score, 0.1)) {
                    persisted_track_held = true;
                    break;
                }
            }
        }
        if (persisted_spec.has_value() &&
            persisted_gate.has_value() &&
            persisted_track_held &&
            grid_segment_is_clear(local_grid, lidar_origin, persisted_spec->position) &&
            scan_supports_target(persisted_spec->position, scan)) {
            GateSpec spec = *persisted_spec;
            spec.heading_hint = angle_to(lidar_origin, spec.position);
            gate_specs_.push_back(spec);

            gate g = *persisted_gate;
            g.x_pos = spec.position.x;
            g.y_pos = spec.position.y;
            g.road = cl_;
            g.road.PSI_end = spec.heading_hint;
            g.choose = false;
            g.too_far = false;
            gates_.push_back(g);
        }
        diagnostics_.candidate_gates = static_cast<int>(gate_specs_.size());
        return;
    }

    if (persisted_spec.has_value()) {
        for (TargetCandidate& candidate : candidates) {
            const double target_delta = distance(candidate.target, persisted_spec->position);
            if (target_delta > 0.22) {
                continue;
            }

            candidate.target.x = 0.35 * persisted_spec->position.x + 0.65 * candidate.target.x;
            candidate.target.y = 0.35 * persisted_spec->position.y + 0.65 * candidate.target.y;
            candidate.center_world_angle = angle_to(lidar_origin, candidate.target);
            candidate.center_local_angle = wrap_angle(candidate.center_world_angle - estimate_.yaw);
            candidate.target_distance = distance(lidar_origin, candidate.target);
            candidate.score += 0.12;
        }
    }

    std::sort(candidates.begin(), candidates.end(), [](const TargetCandidate& a, const TargetCandidate& b) {
        if (std::abs(a.score - b.score) > 1e-6) {
            return a.score > b.score;
        }
        return a.target_distance < b.target_distance;
    });

    std::vector<bool> track_matched(dynamic_gap_tracks_.size(), false);
    const double track_match_radius = micro_hardware_lab
        ? clamp_value(0.30 * current_world_span, 0.12, 0.18)
        : std::max(config_.gap_extraction.gap_track_match_radius_m, 0.08);
    const double track_confirm_score = micro_hardware_lab
        ? 1.20
        : std::max(config_.gap_extraction.gap_track_confirm_score, 0.1);
    const double track_hold_score = micro_hardware_lab
        ? 0.70
        : std::max(config_.gap_extraction.gap_track_hold_score, 0.1);
    const int track_min_hits = micro_hardware_lab
        ? 1
        : std::max(config_.gap_extraction.gap_track_min_hits, 1);
    for (TargetCandidate& candidate : candidates) {
        int best_track_index = -1;
        double best_track_cost = std::numeric_limits<double>::infinity();
        for (size_t i = 0; i < dynamic_gap_tracks_.size(); ++i) {
            if (track_matched[i]) {
                continue;
            }
            const DynamicGapTrack& track = dynamic_gap_tracks_[i];
            const double target_delta = distance(candidate.target, track.position);
            if (target_delta > track_match_radius) {
                continue;
            }
            const double heading_delta =
                std::abs(wrap_angle(candidate.center_world_angle - track.heading));
            if (heading_delta > 0.75) {
                continue;
            }
            const double track_cost = target_delta + 0.10 * heading_delta;
            if (track_cost < best_track_cost) {
                best_track_cost = track_cost;
                best_track_index = static_cast<int>(i);
            }
        }

        DynamicGapTrack* track = nullptr;
        if (best_track_index >= 0) {
            track = &dynamic_gap_tracks_[static_cast<size_t>(best_track_index)];
            track_matched[static_cast<size_t>(best_track_index)] = true;
            const double alpha = clamp_value(
                0.35 + 0.25 * clamp_value(candidate.score / 8.0, 0.0, 1.0),
                0.30,
                0.60);
            track->position.x = (1.0 - alpha) * track->position.x + alpha * candidate.target.x;
            track->position.y = (1.0 - alpha) * track->position.y + alpha * candidate.target.y;
            if (candidate.crossing_point_valid) {
                if (track->crossing_point_valid) {
                    track->crossing_point.x =
                        (1.0 - alpha) * track->crossing_point.x + alpha * candidate.crossing_point.x;
                    track->crossing_point.y =
                        (1.0 - alpha) * track->crossing_point.y + alpha * candidate.crossing_point.y;
                } else {
                    track->crossing_point = candidate.crossing_point;
                    track->crossing_point_valid = true;
                }
            }
            track->heading = angle_to(lidar_origin, track->position);
            track->score = clamp_value(
                track->score + config_.gap_extraction.gap_track_hit_gain +
                    0.18 * clamp_value(candidate.score, 0.0, 8.0),
                0.0,
                8.0);
            track->hits += 1;
            track->misses = 0;
            track->last_seen_step = step_count_;
            track->gap_width = track->gap_width > 0.0
                                   ? (0.65 * track->gap_width + 0.35 * candidate.gap_width)
                                   : candidate.gap_width;
        } else {
            DynamicGapTrack new_track{};
            new_track.id = next_dynamic_gap_track_id_++;
            new_track.position = candidate.target;
            new_track.crossing_point = candidate.crossing_point;
            new_track.crossing_point_valid = candidate.crossing_point_valid;
            new_track.heading = candidate.center_world_angle;
            new_track.score = clamp_value(
                config_.gap_extraction.gap_track_hit_gain +
                    0.18 * clamp_value(candidate.score, 0.0, 8.0),
                0.0,
                8.0);
            new_track.hits = 1;
            new_track.misses = 0;
            new_track.last_seen_step = step_count_;
            new_track.gap_width = candidate.gap_width;
            dynamic_gap_tracks_.push_back(new_track);
            track_matched.push_back(true);
            track = &dynamic_gap_tracks_.back();
        }

        track->candidate_score = candidate.score;
        track->target_distance = distance(lidar_origin, track->position);

        candidate.target = track->position;
        candidate.crossing_point = track->crossing_point;
        candidate.crossing_point_valid = track->crossing_point_valid;
        candidate.center_world_angle = angle_to(lidar_origin, candidate.target);
        candidate.center_local_angle = wrap_angle(candidate.center_world_angle - estimate_.yaw);
        candidate.target_distance = distance(lidar_origin, candidate.target);
        candidate.track_id = track->id;
        candidate.track_hits = track->hits;
        candidate.track_score = track->score;
        const bool held_by_previous_gate =
            persisted_spec.has_value() &&
            distance(candidate.target, persisted_spec->position) <= 0.18 &&
            track->score >= track_hold_score;
        candidate.temporally_confirmed =
            (track->hits >= track_min_hits && track->score >= track_confirm_score) ||
            held_by_previous_gate;
        candidate.score += 0.35 * track->score + 0.08 * static_cast<double>(track->hits);
    }

    for (size_t i = 0; i < dynamic_gap_tracks_.size(); ++i) {
        if (i < track_matched.size() && track_matched[i]) {
            continue;
        }
        DynamicGapTrack& track = dynamic_gap_tracks_[i];
        ++track.misses;
        track.score = std::max(0.0, track.score - config_.gap_extraction.gap_track_miss_decay);
    }
    dynamic_gap_tracks_.erase(
        std::remove_if(
            dynamic_gap_tracks_.begin(),
            dynamic_gap_tracks_.end(),
            [&](const DynamicGapTrack& track) {
                return track.misses > std::max(config_.gap_extraction.gap_track_max_misses, 1) ||
                       track.score <= 0.0;
            }),
        dynamic_gap_tracks_.end());

    std::sort(candidates.begin(), candidates.end(), [](const TargetCandidate& a, const TargetCandidate& b) {
        if (std::abs(a.score - b.score) > 1e-6) {
            return a.score > b.score;
        }
        if (std::abs(a.progress_score - b.progress_score) > 1e-6) {
            return a.progress_score > b.progress_score;
        }
        return a.target_distance < b.target_distance;
    });

    gates_.clear();
    gate_specs_.clear();
    const int gate_limit = std::max(1, config_.gap_extraction.max_candidate_gates);
    for (size_t i = 0; i < candidates.size() && static_cast<int>(gate_specs_.size()) < gate_limit; ++i) {
        const TargetCandidate& candidate = candidates[i];
        if (std::getenv("THESIS_TRACE_GATES") != nullptr && step_count_ < 25) {
            std::cerr << "gate_candidate step=" << step_count_
                      << " rank=" << i
                      << " score=" << candidate.score
                      << " target=(" << candidate.target.x << ',' << candidate.target.y << ')'
                      << " aperture=(" << candidate.crossing_point.x << ','
                      << candidate.crossing_point.y << ')'
                      << " width=" << candidate.gap_width
                      << " hits=" << candidate.track_hits
                      << " confirmed=" << (candidate.temporally_confirmed ? 1 : 0)
                      << '\n';
        }
        if (!candidate.temporally_confirmed) {
            continue;
        }
        if (!world_is_mixed(world_) && !scan_supports_target(candidate.target, scan)) {
            continue;
        }
        bool duplicate_target = false;
        for (const GateSpec& existing : gate_specs_) {
            if (distance(existing.position, candidate.target) < 0.14) {
                duplicate_target = true;
                break;
            }
        }
        if (duplicate_target) {
            continue;
        }

        GateSpec spec{};
        spec.name = candidate.track_id >= 0
                        ? "gap_track_" + std::to_string(candidate.track_id)
                        : "gap_" + std::to_string(gate_specs_.size() + 1);
        spec.position = candidate.target;
        spec.anchor_position = candidate.crossing_point_valid
                                   ? candidate.crossing_point
                                   : candidate.target;
        spec.motion_amplitude = {0.0, 0.0};
        spec.motion_frequency_hz = 0.0;
        spec.motion_phase_rad = 0.0;
        spec.heading_hint = candidate.center_world_angle;
        spec.final = false;
        gate_specs_.push_back(spec);

        gate g{};
        g.x_pos = spec.position.x;
        g.y_pos = spec.position.y;
        g.road = cl_;
        g.road.PSI_end = spec.heading_hint;
        g.passed = false;
        g.choose = false;
        g.too_far = false;
        g.final = false;
        gates_.push_back(g);
    }

    diagnostics_.candidate_gates = static_cast<int>(gate_specs_.size());
}

}  // namespace thesis_sim
