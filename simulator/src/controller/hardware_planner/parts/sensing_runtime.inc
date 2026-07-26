double HardwarePlannerRunner::compute_min_lidar_distance(const std::vector<RPLidarA1::ScanPoint>& scan) const {
    double min_range = config_.localization.max_range_m;
    for (const RPLidarA1::ScanPoint& point : scan) {
        if (point.distance_m >= config_.localization.min_valid_range_m &&
            point.distance_m <= config_.localization.max_range_m &&
            !scan_point_is_self_hit(point)) {
            min_range = std::min(min_range, point.distance_m);
        }
    }
    return min_range;
}

double HardwarePlannerRunner::compute_front_lidar_distance(const std::vector<RPLidarA1::ScanPoint>& scan) const {
    double min_range = config_.localization.max_range_m;
    for (const RPLidarA1::ScanPoint& point : scan) {
        if (point.distance_m <= 0.0 ||
            point.distance_m < config_.localization.min_valid_range_m ||
            point.distance_m > config_.localization.max_range_m ||
            scan_point_is_self_hit(point)) {
            continue;
        }
        const double local_angle = lidar_body_local_angle_rad(config_.localization, point.angle_deg);
        if (std::abs(local_angle) <= config_.localization.front_sector_half_angle_rad) {
            min_range = std::min(min_range, point.distance_m);
        }
    }
    return min_range;
}

bool HardwarePlannerRunner::perception_map_supports_target(const Vec2& origin, const Vec2& target) const {
    if (lidar_map_points_.empty()) {
        return true;
    }

    const bool micro_hardware_lab = compact_hardware_mixed_lab_world(world_);
    const double span = world_span_m(world_);
    const double target_clearance_radius = micro_hardware_lab
        ? clamp_value(0.10 * span, 0.035, 0.055)
        : std::max(
              config_.gap_extraction.target_clearance_radius_m,
              geometry_.body_width * 0.55);
    const double path_clearance_radius = micro_hardware_lab
        ? clamp_value(0.065 * span, 0.025, 0.040)
        : std::max(
              config_.gap_extraction.path_clearance_radius_m,
              geometry_.body_width * 0.42);
    const double target_clearance_sq = target_clearance_radius * target_clearance_radius;
    const double path_clearance_sq = path_clearance_radius * path_clearance_radius;

    for (const Vec2& point : lidar_map_points_) {
        if (distance_sq(point, target) <= target_clearance_sq) {
            return false;
        }
        if (point_segment_distance_sq(point, origin, target) <= path_clearance_sq) {
            return false;
        }
    }

    return true;
}

bool HardwarePlannerRunner::scan_supports_target(const Vec2& target,
                                                 const std::vector<RPLidarA1::ScanPoint>& scan) const {
    if (scan.empty()) {
        return false;
    }

    const Vec2 lidar_origin = lidar_origin_world(estimate_.position, estimate_.yaw, config_.localization);
    const double planning_range = planning_lidar_range(config_);
    const double target_distance = distance(lidar_origin, target);
    if (!(target_distance > 0.05) || target_distance > planning_range) {
        return false;
    }

    const double target_angle_world = angle_to(lidar_origin, target);
    const double target_angle_local = wrap_angle(target_angle_world - estimate_.yaw - config_.localization.lidar_yaw_offset);
    const bool micro_hardware_lab = compact_hardware_mixed_lab_world(world_);
    const double span = world_span_m(world_);
    const double target_clearance_radius = micro_hardware_lab
        ? clamp_value(0.10 * span, 0.035, 0.055)
        : std::max(
              config_.gap_extraction.target_clearance_radius_m,
              geometry_.body_width * 0.55);
    const double target_clearance_sq = target_clearance_radius * target_clearance_radius;
    const double corridor_half_width = micro_hardware_lab
        ? clamp_value(0.12 * span, 0.045, 0.065)
        : std::max(
              0.5 * geometry_.body_width + config_.gap_extraction.path_clearance_radius_m,
              config_.gap_extraction.target_clearance_radius_m);
    const double corridor_margin = micro_hardware_lab
        ? clamp_value(0.07 * span, 0.025, 0.040)
        : std::max(0.04, target_clearance_radius * 0.55);
    const Vec2 target_delta{
        target.x - lidar_origin.x,
        target.y - lidar_origin.y,
    };
    const Vec2 target_axis{
        target_delta.x / target_distance,
        target_delta.y / target_distance,
    };
    const double corridor_half_width_sq = corridor_half_width * corridor_half_width;
    double best_angle_delta = std::numeric_limits<double>::infinity();
    double best_range = 0.0;
    int support_beams = 0;
    int free_support_beams = 0;

    for (const RPLidarA1::ScanPoint& point : scan) {
        if (point.distance_m <= 0.0 ||
            point.distance_m < config_.localization.min_valid_range_m ||
            point.distance_m > planning_range ||
            scan_point_is_self_hit(point)) {
            continue;
        }

        const bool obstacle_endpoint =
            point.distance_m < planning_range - 0.03 &&
            point.distance_m < config_.localization.max_range_m - 0.03;
        if (obstacle_endpoint) {
            const Vec2 hit = scan_point_world_hit(point, estimate_.position, estimate_.yaw);
            if (distance_sq(hit, target) <= target_clearance_sq) {
                return false;
            }
            const Vec2 relative_hit{
                hit.x - lidar_origin.x,
                hit.y - lidar_origin.y,
            };
            const double along = dot_product(relative_hit, target_axis);
            const double lateral_sq = std::max(
                0.0,
                dot_product(relative_hit, relative_hit) - along * along);
            if (along > config_.localization.min_valid_range_m &&
                along + corridor_margin < target_distance &&
                lateral_sq <= corridor_half_width_sq) {
                return false;
            }
        }

        const double beam_angle_local = lidar_sensor_angle_rad(config_.localization, point.angle_deg);
        const double angle_delta = std::abs(wrap_angle(beam_angle_local - target_angle_local));
        if (angle_delta < best_angle_delta) {
            best_angle_delta = angle_delta;
            best_range = point.distance_m;
        }

        const double support_tolerance = std::max(
            config_.gap_extraction.min_gap_angle_rad * 0.35,
            6.0 * kPi / 180.0);
        if (angle_delta <= support_tolerance) {
            ++support_beams;
            if (point.distance_m + 0.08 >= target_distance) {
                ++free_support_beams;
            }
        }
    }

    const double angular_tolerance = std::max(config_.gap_extraction.min_gap_angle_rad * 0.5, 10.0 * kPi / 180.0);
    if (!(best_angle_delta <= angular_tolerance)) {
        return false;
    }

    if (!(best_range + 0.08 >= target_distance)) {
        return false;
    }

    if (support_beams >= 2 && free_support_beams < support_beams) {
        return false;
    }

    return perception_map_supports_target(lidar_origin, target);
}

Vec2 HardwarePlannerRunner::scan_point_world_hit(const RPLidarA1::ScanPoint& point,
                                                 const Vec2& position,
                                                 double yaw) const {
    const Vec2 origin = lidar_origin_world(position, yaw, config_.localization);
    const double beam_heading =
        wrap_angle(yaw + lidar_body_local_angle_rad(config_.localization, point.angle_deg));
    return {
        origin.x + std::cos(beam_heading) * point.distance_m,
        origin.y + std::sin(beam_heading) * point.distance_m,
    };
}

bool HardwarePlannerRunner::scan_point_is_self_hit(const RPLidarA1::ScanPoint& point,
                                                   const Vec2& position,
                                                   double yaw) const {
    if (point.distance_m <= 0.0 ||
        point.distance_m < config_.localization.min_valid_range_m ||
        point.distance_m > config_.localization.max_range_m) {
        return false;
    }

    const Vec2 hit = scan_point_world_hit(point, position, yaw);
    const Vec2 relative{hit.x - position.x, hit.y - position.y};
    const Vec2 local = rotate(relative, -yaw);
    const double half_length = std::max(
        geometry_.body_length * 0.5,
        std::max(config_.drive.cg_to_front, config_.drive.cg_to_rear));
    const double half_width = geometry_.body_width * 0.5;
    const double front_extent = half_length + 0.03;
    const double rear_extent = half_length + 0.01;
    const double lateral_extent = half_width + 0.02;

    return local.x >= -rear_extent &&
           local.x <= front_extent &&
           std::abs(local.y) <= lateral_extent;
}

bool HardwarePlannerRunner::scan_point_is_self_hit(const RPLidarA1::ScanPoint& point) const {
    return scan_point_is_self_hit(point, estimate_.position, estimate_.yaw);
}

int HardwarePlannerRunner::wheel_speed_to_pwm(double wheel_speed_mps, double scale) const {
    return mvc::model::wheel_speed_to_pwm(
        wheel_speed_mps,
        scale,
        config_.pwm.min_effective_pwm,
        config_.pwm.max_pwm,
        config_.pwm.wheel_speed_to_pwm_gain,
        config_.pwm.wheel_speed_to_pwm_bias);
}

void HardwarePlannerRunner::step() {
    if (!connected_) {
        throw ProtocolResponseError("HardwarePlannerRunner not connected");
    }

    if (bridge_.controller_connected()) {
        const RealRobotObservation& previous_observation = bridge_.observation();
        if (previous_observation.have_controller_telemetry) {
            rearm_controller_if_needed(previous_observation.controller);
        }
        bridge_.send_pwm(
            static_cast<std::int16_t>(last_command_.pwm_left),
            static_cast<std::int16_t>(last_command_.pwm_right),
            true,
            MotorControlMode::SafeDirectPwm);
    }

    const int lidar_acquisition_min_points =
        std::max(18, std::max(config_.localization.min_scan_points, 1) / 4);
    bridge_.pump(
        std::min(0.05, config_.nominal_dt * 0.5),
        lidar_acquisition_min_points,
        lidar_enabled_for_current_mode());
    telemetry_ready_ = bridge_.observation().have_controller_telemetry;
    if (!telemetry_ready_) {
        return;
    }

    const RealRobotObservation& observation = bridge_.observation();
    rearm_controller_if_needed(observation.controller);
    const double now = observation.host_timestamp_s;
    const double dt = last_observation_time_ > 0.0
                          ? clamp_value(now - last_observation_time_, 0.02, 0.25)
                          : config_.nominal_dt;
    last_observation_time_ = now;
    step_with_observation(observation, dt, true);
}

void HardwarePlannerRunner::step_with_observation(const RealRobotObservation& observation,
                                                  double dt,
                                                  bool send_pwm) {
    const double bounded_dt = clamp_value(dt, 0.01, 0.25);
    const bool lidar_enabled = lidar_enabled_for_current_mode();
    if (observation.have_controller_telemetry) {
        telemetry_ready_ = true;
    }

    const auto step_start = std::chrono::steady_clock::now();

    world_.update_gate_layout(sim_time_);
    if (!dynamic_gap_mode_enabled()) {
        sync_gate_specs_from_world(false);
    }

    const auto estimator_start = std::chrono::steady_clock::now();
    update_estimate_from_observation(observation, bounded_dt);
    estimator_compute_ms_ = elapsed_ms(estimator_start, std::chrono::steady_clock::now());

    const auto lidar_start = std::chrono::steady_clock::now();
    if (lidar_enabled && observation.have_lidar_scan) {
        correct_pose_with_lidar(observation.lidar_scan);
        update_lidar_hits_world(observation.lidar_scan);
        if (dynamic_gap_mode_enabled()) {
            // Validate the currently locked gate before rebuilding candidates.
            // Once the aperture is behind the robot it may legitimately lose
            // scan support; rebuilding first used to discard that gate before
            // its physical crossing could be counted.
            count_mixed_gate_crossing_if_needed();
            rebuild_dynamic_gap_gates(observation.lidar_scan);
            update_unstructured_gap_workflow(bounded_dt);
        }
    } else {
        estimate_.min_lidar_distance = lidar_enabled ? config_.localization.max_range_m : -1.0;
        estimate_.front_lidar_distance = lidar_enabled ? config_.localization.max_range_m : -1.0;
        lidar_hits_.clear();
        if (dynamic_gap_mode_enabled()) {
            restart_unstructured_scan();
        }
        if (!lidar_enabled) {
            lidar_map_points_.clear();
            lidar_occupancy_cells_.clear();
            lidar_map_keys_.clear();
        }
        diagnostics_.valid_lidar_points = 0;
        diagnostics_.close_lidar_points = 0;
        diagnostics_.front_close_lidar_points = 0;
        diagnostics_.lidar_front_blocked = false;
        diagnostics_.accumulated_lidar_points = static_cast<int>(lidar_map_points_.size());
        diagnostics_.candidate_gates = static_cast<int>(gate_specs_.size());
    }
    lidar_compute_ms_ = elapsed_ms(lidar_start, std::chrono::steady_clock::now());

    sync_planner_from_estimate(false);
    refresh_gate_diagnostics();

    const auto planning_start = std::chrono::steady_clock::now();
    plan_if_needed();
    update_planner_references(bounded_dt);
    update_selected_trajectory();
    planning_compute_ms_ = elapsed_ms(planning_start, std::chrono::steady_clock::now());

    const auto tracking_start = std::chrono::steady_clock::now();
    compute_control_command(bounded_dt);
    tracking_compute_ms_ = elapsed_ms(tracking_start, std::chrono::steady_clock::now());

    sim_time_ += bounded_dt;
    ++step_count_;
    count_mixed_gate_crossing_if_needed();
    if (structured_road_is_closed_loop(world_)) {
        const bool tiny_indoor_loop = tiny_indoor_structured_loop(world_);
        const bool compact_mixed_loop = compact_mixed_structured_loop(world_);
        const bool free_hardware_mixed_loop =
            compact_hardware_mixed_lab_world(world_) && world_.obstacles().empty();
        const bool tracked_closed_loop =
            config_.vehicle_model == VehicleModelKind::TrackedVehicle;
        const double wrapped_track_s =
            std::isfinite(x0_.x) ? wrap_arc_length(x0_.x, cl_.end_point_s) : 0.0;
        const double goal_position_distance =
            distance(estimate_.position, structured_goal_ready_ ? structured_goal_position_ : world_.start());
        const double progress_margin =
            tracked_closed_loop ? std::clamp(0.025 * cl_.end_point_s, 0.018, 0.040)
                                : tiny_indoor_loop ? std::clamp(0.11 * cl_.end_point_s, 0.065, 0.10)
                                                   : compact_mixed_loop ? std::clamp(0.10 * std::max(cl_.end_point_s, 1.0), 0.14, 0.22)
                                                                        : std::clamp(0.04 * std::max(cl_.end_point_s, 1.0), 0.05, 0.20);
        const double start_window =
            tracked_closed_loop ? std::clamp(0.030 * cl_.end_point_s, 0.022, 0.045)
                                : tiny_indoor_loop ? std::clamp(0.10 * cl_.end_point_s, 0.065, 0.10)
                                                   : compact_mixed_loop ? std::clamp(0.08 * std::max(cl_.end_point_s, 1.0), 0.12, 0.24)
                                                                        : std::clamp(0.04 * std::max(cl_.end_point_s, 1.0), 0.10, 0.35);
        const double goal_position_acceptance =
            tracked_closed_loop
                ? (tiny_indoor_loop ? std::clamp(0.16 * structured_course_span_m(world_), 0.035, 0.055)
                                    : std::clamp(0.20 * structured_course_span_m(world_), 0.060, 0.100))
                                : free_hardware_mixed_loop ? std::clamp(0.32 * structured_course_span_m(world_), 0.13, 0.17)
                                  : tiny_indoor_loop ? std::clamp(0.28 * structured_course_span_m(world_), 0.09, 0.11)
                                                  : compact_mixed_loop ? std::clamp(0.24 * structured_course_span_m(world_), 0.11, 0.16)
                                                                        : std::max(config_.goal_tolerance_m * 2.0, 0.35);
        const double tracked_goal_cross_track_acceptance =
            tiny_indoor_loop ? std::clamp(0.16 * structured_course_span_m(world_), 0.030, 0.045)
                             : std::clamp(0.18 * structured_course_span_m(world_), 0.040, 0.070);
        const double tracked_goal_heading_acceptance_deg =
            tiny_indoor_loop ? 42.0 : 55.0;
        const bool tracked_goal_pose_aligned =
            !tracked_closed_loop ||
            (tracker_cross_track_error_ <= tracked_goal_cross_track_acceptance &&
             tracker_heading_error_deg_ <= tracked_goal_heading_acceptance_deg);
        const bool tiny_loop_neighborhood_complete =
            tiny_indoor_loop &&
            !tracked_closed_loop &&
            structured_goal_ready_ &&
            structured_progress_s_ >= 0.50 * structured_goal_progress_target_ &&
            goal_position_distance < goal_position_acceptance;
        const bool returned_to_start =
            wrapped_track_s <= start_window || wrapped_track_s >= std::max(cl_.end_point_s - start_window, 0.0);
        distance_to_goal_ = structured_goal_ready_
                                ? std::max(structured_goal_progress_target_ - structured_progress_s_, 0.0)
                                : cl_.end_point_s;
        const bool full_loop_progress_complete =
            structured_goal_ready_ &&
            structured_progress_s_ + progress_margin >= structured_goal_progress_target_;
        const bool precise_tracked_goal =
            tracked_closed_loop &&
            full_loop_progress_complete &&
            goal_position_distance < goal_position_acceptance &&
            tracked_goal_pose_aligned;
        const bool large_tank_structured_loop =
            tracked_closed_loop &&
            simple_single_turn_structured_loop(world_) &&
            world_span_m(world_) >= 0.90;
        const double large_tank_goal_position_acceptance =
            std::clamp(0.13 * structured_course_span_m(world_), 0.060, 0.085);
        const bool legacy_large_tank_loop_goal =
            tracked_closed_loop &&
            simple_single_turn_structured_loop(world_) &&
            world_span_m(world_) >= 0.45 &&
            full_loop_progress_complete &&
            returned_to_start &&
            (!large_tank_structured_loop ||
             goal_position_distance < large_tank_goal_position_acceptance) &&
            tracker_cross_track_error_ <= std::clamp(0.35 * structured_course_span_m(world_), 0.070, 0.110);
        if (compact_mixed_loop) {
            const double minimum_mixed_progress =
                0.65 * structured_goal_progress_target_;
            const bool active_dynamic_gate = locked_gap_goal_.has_value();
            if (tracked_closed_loop) {
                goal_reached_ = !active_dynamic_gate && (precise_tracked_goal || legacy_large_tank_loop_goal);
            } else {
                goal_reached_ =
                    !active_dynamic_gate &&
                    structured_goal_ready_ &&
                    structured_progress_s_ >= minimum_mixed_progress &&
                    returned_to_start &&
                    goal_position_distance < goal_position_acceptance;
            }
            if (!goal_reached_ &&
                distance_to_goal_ <= structured_goal_active_distance(world_, config_.goal_tolerance_m) &&
                (!returned_to_start || goal_position_distance >= goal_position_acceptance)) {
                distance_to_goal_ = std::max(
                    goal_position_distance,
                    structured_goal_active_distance(world_, config_.goal_tolerance_m));
            }
        } else {
            if (tracked_closed_loop) {
                goal_reached_ = precise_tracked_goal || legacy_large_tank_loop_goal;
            } else {
                goal_reached_ =
                    tiny_loop_neighborhood_complete ||
                    (full_loop_progress_complete &&
                     (returned_to_start || goal_position_distance < goal_position_acceptance));
            }
        }
        if (!goal_reached_ &&
            tracked_closed_loop &&
            (full_loop_progress_complete || returned_to_start)) {
            distance_to_goal_ = std::max(goal_position_distance, goal_position_acceptance);
        }
        if (goal_reached_) {
            distance_to_goal_ = 0.0;
        }
    } else if (world_.environment_mode() == EnvironmentMode::UnstructuredGates) {
        if (passed_unstructured_gap_count_ >= required_unstructured_gap_pass_count()) {
            distance_to_goal_ = 0.0;
            goal_reached_ = true;
        } else if (locked_gap_goal_.has_value()) {
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
            const double lateral_distance =
                std::max(lateral_offset - pass_lateral_limit, 0.0);
            distance_to_goal_ = std::max(crossing_distance, lateral_distance);
            const double near_pass_longitudinal_slack = clamp_value(
                0.46 * std::max(config_.gap_extraction.gap_goal_tolerance_m, 0.0),
                0.045,
                0.085);
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
            const double gate_distance = distance(estimate_.position, *locked_gap_goal_);
            const bool gate_nearly_crossed =
                lateral_offset <= pass_lateral_limit &&
                crossing_distance <= near_pass_longitudinal_slack &&
                gate_distance <= near_pass_gate_radius;
            const bool gate_crossed =
                lateral_offset <= pass_lateral_limit &&
                (crossing_progress >= crossing_margin || gate_nearly_crossed);
            if (gate_crossed) {
                const Vec2 completed_gap = *locked_gap_goal_;
                locked_gap_crossed_ = true;
                passed_unstructured_gap_positions_.push_back(completed_gap);
                ++passed_unstructured_gap_count_;
                goal_reached_ =
                    passed_unstructured_gap_count_ >= required_unstructured_gap_pass_count();
                if (goal_reached_) {
                    clear_locked_gap_goal();
                    gates_.clear();
                    gate_specs_.clear();
                    visible_gate_indices_.clear();
                    chosen_gate_index_ = -1;
                    reference_trajectory_.clear();
                    planned_trajectory_.clear();
                    diagnostics_.candidate_gates = 0;
                    diagnostics_.chosen_gate_distance = std::numeric_limits<double>::infinity();
                    diagnostics_.planner_has_reference = false;
                } else {
                    restart_unstructured_scan();
                    commanded_speed_ = 0.0;
                    commanded_steer_angle_ = 0.0;
                    last_command_ = {};
                    last_mpc_command_.reset();
                }
                distance_to_goal_ = goal_reached_ ? 0.0 : -1.0;
            }
        } else if (chosen_gate_index_ >= 0 && chosen_gate_index_ < static_cast<int>(gate_specs_.size())) {
            distance_to_goal_ =
                distance(estimate_.position, gate_specs_[static_cast<size_t>(chosen_gate_index_)].position);
            goal_reached_ = false;
        } else {
            distance_to_goal_ = -1.0;
            goal_reached_ = false;
        }
    } else {
        distance_to_goal_ = distance(estimate_.position, world_.goal());
        double goal_tolerance = config_.goal_tolerance_m;
        if (world_has_structured_reference(world_)) {
            const double min_goal_tolerance = micro_structured_world(world_) ? 0.045 : 0.035;
            goal_tolerance = std::clamp(0.14 * structured_course_span_m(world_), min_goal_tolerance, 0.10);
        }
        const bool compact_mixed_open_world =
            compact_mixed_open_road(world_);
        if (compact_mixed_open_world) {
            goal_tolerance = std::max(goal_tolerance, 0.09);
        }
        double goal_stop_speed = config_.goal_stop_speed_mps;
        if (compact_mixed_open_world && distance_to_goal_ < goal_tolerance) {
            goal_stop_speed = std::max(goal_stop_speed, 0.12);
        }
        const bool active_dynamic_gate =
            compact_mixed_open_world && locked_gap_goal_.has_value();
        const double open_progress_target =
            compact_mixed_open_world ? std::max(cl_.end_point_s, 0.0) : 0.0;
        const double open_progress_margin =
            compact_mixed_open_world
                ? std::clamp(0.08 * std::max(open_progress_target, 0.10), 0.035, 0.080)
                : 0.0;
        const bool open_progress_ready =
            !compact_mixed_open_world ||
            (open_progress_target > 0.0 &&
             structured_progress_s_ + open_progress_margin >= open_progress_target);
        const bool obstacle_free_dynamic_open_world =
            compact_mixed_open_world && world_.obstacles().empty();
        const bool mixed_gate_sequence_armed =
            !obstacle_free_dynamic_open_world ||
            passed_unstructured_gap_count_ > 0 ||
            locked_gap_goal_.has_value() ||
            !gate_specs_.empty();
        const int required_mixed_pass_count =
            compact_mixed_open_world && mixed_gate_sequence_armed
                ? std::max(config_.gap_extraction.min_passed_gates_to_complete, 1)
                : 0;
        const bool mixed_gate_sequence_complete =
            !compact_mixed_open_world ||
            passed_unstructured_gap_count_ >= required_mixed_pass_count;
        if (compact_mixed_open_world && !open_progress_ready) {
            distance_to_goal_ = std::max(
                distance_to_goal_,
                std::max(open_progress_target - structured_progress_s_, 0.0));
        }
        if (compact_mixed_open_world && !mixed_gate_sequence_complete) {
            distance_to_goal_ = std::max(
                distance_to_goal_,
                std::max(config_.gap_extraction.gap_goal_tolerance_m, goal_tolerance));
        }
        goal_reached_ = !active_dynamic_gate &&
                        mixed_gate_sequence_complete &&
                        open_progress_ready &&
                        distance_to_goal_ < goal_tolerance &&
                        std::abs(estimate_.speed) < goal_stop_speed;
        if (goal_reached_) {
            distance_to_goal_ = 0.0;
        }
    }

    if (goal_reached_) {
        safety_stop_active_ = false;
        commanded_speed_ = 0.0;
        commanded_steer_angle_ = 0.0;
        last_command_ = {};
    }
    step_compute_ms_ = elapsed_ms(step_start, std::chrono::steady_clock::now());
    push_history();

    if (send_pwm && connected_ && bridge_.controller_connected()) {
        bridge_.send_pwm(
            static_cast<std::int16_t>(last_command_.pwm_left),
            static_cast<std::int16_t>(last_command_.pwm_right),
            false,
            MotorControlMode::SafeDirectPwm);
    }
}

HardwarePlannerReport HardwarePlannerRunner::run(int max_steps) {
    const int limit = max_steps > 0 ? max_steps : 2000;
    while (step_count_ < limit && !goal_reached_) {
        step();
    }

    return current_report();
}

}  // namespace thesis_sim
