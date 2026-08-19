#include "support.inc"

namespace thesis_sim {

void HardwarePlannerRunner::compute_control_command(double dt) {
    const int previous_pwm_left = last_command_.pwm_left;
    const int previous_pwm_right = last_command_.pwm_right;
    last_command_ = {};
    safety_stop_active_ = false;
    stall_boost_active_ = false;
    tracker_cross_track_error_ = 0.0;
    tracker_heading_error_deg_ = 0.0;
    control_source_ = HardwareControlSource::Hold;

    if (!motion_permitted_by_start_match()) {
        commanded_speed_ = 0.0;
        commanded_steer_angle_ = 0.0;
        last_command_ = {};
        last_mpc_command_.reset();
        exploration_state_ = UnstructuredExplorationState::StartMatching;
        return;
    }

    if (goal_reached_) {
        commanded_speed_ = 0.0;
        commanded_steer_angle_ = 0.0;
        last_command_.target_speed = 0.0;
        last_command_.target_curvature = 0.0;
        last_command_.target_yaw_rate = 0.0;
        return;
    }

    const double safe_dt = std::max(dt, 1e-3);
    const double speed_limit =
        std::min(config_.cruise_speed_limit, geometry_.max_linear_speed);
    const double measured_equivalent_steer =
        steer_from_curvature(geometry_, estimate_.curvature);
    if (std::abs(estimate_.speed) > 0.02 || std::abs(estimate_.yaw_rate) > 0.04) {
        // On hardware we do not sense steering angle directly, so keep a
        // bicycle-equivalent steering state lightly anchored to the observed motion.
        commanded_steer_angle_ = clamp_value(
            0.85 * commanded_steer_angle_ + 0.15 * measured_equivalent_steer,
            -geometry_.max_steer_angle,
            geometry_.max_steer_angle);
    }

    last_command_.target_speed = commanded_speed_;
    last_command_.target_curvature =
        curvature_from_steer(geometry_, commanded_steer_angle_);
    const bool have_reference_trajectory = reference_trajectory_.size() >= 2;
    diagnostics_.planner_has_reference = have_reference_trajectory;
    bool use_direct_yaw_rate_command = false;
    double direct_yaw_rate_command = 0.0;
    const bool scanning_startup = startup_scan_active();
    const bool strict_locked_motion = strict_locked_gate_motion_enabled();
    const bool mixed_gate_active = world_is_mixed(world_) && locked_gap_goal_.has_value();
    const bool compact_mixed_gate_active =
        mixed_gate_active && world_span_m(world_) <= 2.50;
    const bool following_dynamic_gate =
        world_.environment_mode() == EnvironmentMode::UnstructuredGates ||
        mixed_gate_active;
    const bool tracked_vehicle = config_.vehicle_model == VehicleModelKind::TrackedVehicle;
    const bool compact_hardware_car =
        compact_hardware_mixed_lab_world(world_) && !tracked_vehicle;
    const bool compact_unstructured_car =
        !tracked_vehicle &&
        world_.environment_mode() == EnvironmentMode::UnstructuredGates &&
        world_span_m(world_) <= 1.25;
    const bool forward_creep_gate_car = compact_hardware_car || compact_unstructured_car;

    const bool bounded_hardware_car =
        !tracked_vehicle &&
        (compact_unstructured_car ||
         (world_is_mixed(world_) && world_span_m(world_) <= 2.50));
    const auto hardware_car_footprint = make_box_corners(
        estimate_.position,
        estimate_.yaw,
        geometry_.body_length,
        geometry_.body_width);
    const bool hardware_car_footprint_outside_bounds =
        bounded_hardware_car &&
        std::any_of(
            hardware_car_footprint.begin(),
            hardware_car_footprint.end(),
            [&](const Vec2& corner) {
                return !is_inside_bounds(world_, corner);
            });
    if (hardware_car_footprint_outside_bounds) {
        // Encoder slip or an invalid recovery target must never let a compact
        // hardware run keep driving after its estimated footprint leaves the
        // configured physical arena.
        safety_stop_active_ = true;
        commanded_speed_ = 0.0;
        commanded_steer_angle_ = 0.0;
        last_command_ = {};
        last_command_.safety_stop = true;
        wheel_speed_error_integral_left_ = 0.0;
        wheel_speed_error_integral_right_ = 0.0;
        no_motion_command_cycles_ = 0;
        left_wheel_stall_cycles_ = 0;
        right_wheel_stall_cycles_ = 0;
        left_wheel_breakaway_hold_cycles_ = 0;
        right_wheel_breakaway_hold_cycles_ = 0;
        diagnostics_.no_motion_command_cycles = 0;
        diagnostics_.stall_boost_active = false;
        control_source_ = HardwareControlSource::SafetyStop;
        return;
    }
    const bool forward_only_tracked_tracks =
        tracked_vehicle && world_has_structured_reference(world_) && !mixed_gate_active;
    const bool simple_tiny_tracked_loop =
        forward_only_tracked_tracks &&
        tiny_indoor_structured_loop(world_) &&
        simple_single_turn_structured_loop(world_);
    const bool large_single_turn_tracked_loop =
        forward_only_tracked_tracks &&
        !simple_tiny_tracked_loop &&
        structured_road_is_closed_loop(world_) &&
        simple_single_turn_structured_loop(world_) &&
        world_span_m(world_) >= 0.90 &&
        structured_course_span_m(world_) >= 0.50;
    bool allow_unlocked_recovery_motion = false;
    bool allow_reverse_recovery_motion = false;

    auto apply_forward_gate_search = [&]() {
        double search_speed = 0.0;
        double search_yaw_rate = 0.0;
        if (!compute_forward_gate_search_command(
                speed_limit,
                &search_speed,
                &search_yaw_rate)) {
            return false;
        }
        if (search_speed <= 1e-4) {
            return false;
        }

        use_direct_yaw_rate_command = true;
        commanded_steer_angle_ = 0.0;
        last_command_.target_curvature = 0.0;
        last_mpc_command_.reset();
        gap_recovery_turn_active_ = true;
        direct_yaw_rate_command = search_yaw_rate;
        commanded_speed_ = search_speed;
        last_command_.target_speed = search_speed;
        allow_unlocked_recovery_motion = search_speed > 1e-4;
        const bool straight_search_command =
            open_ended_unstructured_exploration(world_) &&
            std::abs(search_yaw_rate) <= 0.015;
        gap_recovery_turn_active_ = !straight_search_command;
        exploration_state_ = straight_search_command
            ? UnstructuredExplorationState::AdvancingStraight
            : UnstructuredExplorationState::SearchingFreeSpace;
        control_source_ = straight_search_command
            ? HardwareControlSource::StraightExploration
            : HardwareControlSource::ForwardSearch;
        return true;
    };

    auto apply_strict_scan_escape = [&]() {
        if (!strict_locked_motion ||
            !use_dynamic_gap_gates_ ||
            locked_gap_goal_.has_value() ||
            have_reference_trajectory ||
            !gate_specs_.empty() ||
            goal_reached_) {
            return false;
        }

        const double escape_after_s =
            std::max(config_.gap_extraction.strict_scan_escape_after_s, 0.0);
        if (unstructured_no_candidate_scan_elapsed_s_ < escape_after_s) {
            return false;
        }

        const double escape_period_s =
            std::max(config_.gap_extraction.strict_scan_escape_period_s, 0.5);
        const double escape_pulse_s = clamp_value(
            config_.gap_extraction.strict_scan_escape_pulse_s,
            0.10,
            escape_period_s);
        const double escape_phase =
            std::fmod(unstructured_no_candidate_scan_elapsed_s_ - escape_after_s, escape_period_s);
        if (escape_phase > escape_pulse_s) {
            return false;
        }

        const double fallback_front_clearance =
            estimate_.front_lidar_distance > 0.0
                ? estimate_.front_lidar_distance
                : config_.localization.max_range_m;
        const double forward_sector_clearance = sector_min_clearance(
            lidar_hits_,
            estimate_.yaw,
            0.0,
            std::max(config_.gap_extraction.recovery_sector_half_angle_rad, 0.20),
            fallback_front_clearance);
        const double escape_clearance = std::max(
            config_.localization.obstacle_stop_distance_m + 0.09,
            config_.localization.min_valid_range_m + 0.22);
        if (!(forward_sector_clearance > escape_clearance)) {
            return false;
        }

        const Vec2 goal = world_.goal();
        double escape_yaw_rate = 0.0;
        if (is_inside_bounds(world_, goal) && distance(estimate_.position, goal) > 0.05) {
            const double heading_error = wrap_angle(angle_to(estimate_.position, goal) - estimate_.yaw);
            escape_yaw_rate = clamp_value(
                0.48 * heading_error,
                -0.24,
                0.24);
            if (std::abs(escape_yaw_rate) < 0.055 && std::abs(heading_error) > 0.10) {
                escape_yaw_rate = signum(heading_error > 0.0 ? 1 : -1) * 0.055;
            }
        } else {
            escape_yaw_rate = 0.10 * (startup_scan_direction_ >= 0.0 ? 1.0 : -1.0);
        }

        const double clearance_scale = clamp_value(
            (forward_sector_clearance - escape_clearance) / 0.35,
            0.0,
            1.0);
        const double escape_speed = clamp_value(
            config_.gap_extraction.strict_scan_escape_speed_mps *
                (0.55 + 0.45 * clearance_scale),
            0.0,
            std::max(config_.gap_extraction.strict_scan_escape_speed_mps, 0.0));

        allow_unlocked_recovery_motion = true;
        gap_recovery_turn_active_ = true;
        use_direct_yaw_rate_command = true;
        direct_yaw_rate_command = escape_yaw_rate;
        commanded_speed_ = escape_speed;
        commanded_steer_angle_ = 0.0;
        last_command_.target_speed = escape_speed;
        last_command_.target_curvature = 0.0;
        last_mpc_command_.reset();
        return true;
    };

    auto apply_reverse_lidar_reposition = [&]() {
        if (!open_ended_unstructured_exploration(world_) ||
            tracked_vehicle ||
            goal_reached_ ||
            locked_gap_goal_.has_value() ||
            have_reference_trajectory) {
            return false;
        }

        constexpr double kReverseCompletionDistanceM = 0.075;
        constexpr double kReverseMaximumDurationS = 0.60;
        // Do not insert a stationary cooldown while every forward arc remains
        // invalid. Measured displacement still terminates each short segment;
        // the next cycle tries forward first and only extends the reverse when
        // the rear swept footprint is still clear.
        constexpr double kReverseRetryCooldownS = 0.0;
        const bool reverse_reposition_active =
            sim_time_ < lidar_reverse_reposition_until_s_;
        if (reverse_reposition_active &&
            distance(estimate_.position, lidar_reverse_reposition_start_position_) >=
                kReverseCompletionDistanceM) {
            // The legacy direct-PWM firmware cannot realise -3 cm/s: its
            // minimum effective PWM produced 20--60 cm/s in the reports. End
            // the recovery from measured displacement, not only elapsed time.
            lidar_reverse_reposition_until_s_ = sim_time_;
            lidar_reverse_reposition_cooldown_until_s_ = std::max(
                lidar_reverse_reposition_cooldown_until_s_,
                sim_time_ + kReverseRetryCooldownS);
            return false;
        }

        // A bounded forward-only car can reach a corner from which every
        // forward arc intersects the footprint margin. Do not hold forever:
        // after live LiDAR has rejected both straight motion and all forward
        // sectors, back up a short distance only if the complete rear swept
        // footprint is clear. This is a geometric escape, never gate pursuit.
        const double rear_clearance = sector_min_clearance(
            lidar_hits_,
            estimate_.yaw,
            kPi,
            0.30,
            config_.localization.max_range_m);
        const double required_rear_clearance = std::max(
            config_.localization.obstacle_stop_distance_m + 0.055,
            config_.localization.min_valid_range_m + 0.16);
        if (!(rear_clearance > required_rear_clearance)) {
            return false;
        }

        if (sim_time_ >= lidar_reverse_reposition_until_s_) {
            if (sim_time_ < lidar_reverse_reposition_cooldown_until_s_) {
                return false;
            }
            const double left_clearance = sector_min_clearance(
                lidar_hits_, estimate_.yaw, 0.78, 0.36,
                config_.localization.max_range_m);
            const double right_clearance = sector_min_clearance(
                lidar_hits_, estimate_.yaw, -0.78, 0.36,
                config_.localization.max_range_m);
            // This sign is the desired subsequent forward turn. Reverse
            // steering uses the opposite yaw sign so the body translates away
            // from the crowded side before that turn.
            startup_scan_direction_ =
                left_clearance >= right_clearance ? 1.0 : -1.0;
            lidar_reverse_reposition_start_position_ = estimate_.position;
            lidar_reverse_reposition_until_s_ =
                sim_time_ + kReverseMaximumDurationS;
            lidar_reverse_reposition_cooldown_until_s_ =
                lidar_reverse_reposition_until_s_ + kReverseRetryCooldownS;
            lidar_post_reverse_search_until_s_ = sim_time_ + 3.00;
        }

        constexpr double kReverseProbeDistanceM = 0.080;
        constexpr double kReverseSpeedMps = -0.030;
        const double footprint_half_length =
            0.5 * geometry_.body_length + 0.030;
        const double footprint_half_width =
            0.5 * geometry_.body_width + 0.030;
        const auto reverse_arc_is_clear = [&](double yaw_rate) {
            const double reverse_probe_time_s =
                kReverseProbeDistanceM / std::abs(kReverseSpeedMps);
            const double reverse_yaw_delta =
                yaw_rate * reverse_probe_time_s;
            const double reverse_probe_heading =
                estimate_.yaw + 0.5 * reverse_yaw_delta;
            const Vec2 reverse_probe{
                estimate_.position.x -
                    kReverseProbeDistanceM * std::cos(reverse_probe_heading),
                estimate_.position.y -
                    kReverseProbeDistanceM * std::sin(reverse_probe_heading),
            };
            const double reverse_probe_yaw =
                estimate_.yaw + reverse_yaw_delta;
            const auto reverse_footprint = make_box_corners(
                reverse_probe,
                reverse_probe_yaw,
                geometry_.body_length,
                geometry_.body_width);
            if (std::any_of(
                    reverse_footprint.begin(),
                    reverse_footprint.end(),
                    [&](const Vec2& corner) {
                        return !is_inside_bounds(world_, corner, -0.015);
                    })) {
                return false;
            }

            const double cos_yaw = std::cos(reverse_probe_yaw);
            const double sin_yaw = std::sin(reverse_probe_yaw);
            return std::none_of(
                lidar_hits_.begin(),
                lidar_hits_.end(),
                [&](const LidarHit& hit) {
                    if (!hit.hit ||
                        !(hit.distance > 0.0) ||
                        hit.distance >= config_.localization.max_range_m - 0.03) {
                        return false;
                    }
                    const double dx = hit.point.x - reverse_probe.x;
                    const double dy = hit.point.y - reverse_probe.y;
                    const double longitudinal = cos_yaw * dx + sin_yaw * dy;
                    const double lateral = -sin_yaw * dx + cos_yaw * dy;
                    return std::abs(longitudinal) <= footprint_half_length &&
                           std::abs(lateral) <= footprint_half_width;
                });
        };
        double reverse_yaw_rate = -startup_scan_direction_ * 0.15;
        if (!reverse_arc_is_clear(reverse_yaw_rate)) {
            reverse_yaw_rate = startup_scan_direction_ * 0.15;
        }
        if (!reverse_arc_is_clear(reverse_yaw_rate)) {
            reverse_yaw_rate = 0.0;
        }
        if (!reverse_arc_is_clear(reverse_yaw_rate)) {
            return false;
        }

        allow_unlocked_recovery_motion = true;
        allow_reverse_recovery_motion = true;
        gap_recovery_turn_active_ = true;
        use_direct_yaw_rate_command = true;
        direct_yaw_rate_command = reverse_yaw_rate;
        commanded_speed_ = kReverseSpeedMps;
        commanded_steer_angle_ = 0.0;
        last_command_.target_speed = commanded_speed_;
        last_command_.target_curvature = 0.0;
        last_command_.target_yaw_rate = reverse_yaw_rate;
        last_mpc_command_.reset();
        exploration_state_ = UnstructuredExplorationState::SearchingFreeSpace;
        control_source_ = HardwareControlSource::Recovery;
        return true;
    };

    VehicleModelState tracking_state = build_tracking_state(
        estimate_.position,
        estimate_.yaw,
        estimate_.speed,
        estimate_.accel,
        estimate_.curvature,
        estimate_.yaw_rate,
        commanded_steer_angle_);
    tracking_state.target_speed = commanded_speed_;
    tracking_state.target_steer_angle = commanded_steer_angle_;

    if (scanning_startup) {
        if (open_ended_unstructured_exploration(world_)) {
            // A reference invalidation may reset startup_scan_complete_ after
            // the workflow has already run.  Never turn that bookkeeping
            // cycle into a stationary scan: continue with live-LiDAR forward
            // search or the independently validated short reverse escape.
            if (!apply_forward_gate_search() &&
                !apply_reverse_lidar_reposition()) {
                commanded_speed_ = 0.0;
                commanded_steer_angle_ = 0.0;
                last_command_.target_speed = 0.0;
                last_command_.target_curvature = 0.0;
                use_direct_yaw_rate_command = false;
                last_mpc_command_.reset();
                exploration_state_ = UnstructuredExplorationState::Hold;
            }
        } else if (!apply_forward_gate_search()) {
            commanded_speed_ = 0.0;
            commanded_steer_angle_ = 0.0;
            last_command_.target_speed = 0.0;
            last_command_.target_curvature = 0.0;
            use_direct_yaw_rate_command = true;
            direct_yaw_rate_command = startup_scan_direction_ * clamp_value(
                std::abs(config_.gap_extraction.startup_scan_yaw_rate),
                0.0,
                0.85 * config_.drive.max_yaw_rate);
            last_mpc_command_.reset();
            apply_strict_scan_escape();
        }
    } else if (have_reference_trajectory) {
        const MpcCommand mpc_command = mpc_follower_.solve(
            geometry_,
            tracking_state,
            reference_trajectory_,
            planner_speed_ref_,
            structured_anchor_hint_index_,
            config_.vehicle_model);
        if (mpc_command.valid) {
            const double predicted_speed = clamp_value(
                estimate_.speed + mpc_command.accel_cmd * safe_dt,
                0.0,
                speed_limit);
            const double mpc_target_speed = clamp_value(
                mpc_command.target_speed,
                0.0,
                speed_limit);
            const double integrated_steer = clamp_value(
                commanded_steer_angle_ + mpc_command.steer_rate_cmd * safe_dt,
                -geometry_.max_steer_angle,
                geometry_.max_steer_angle);
            commanded_speed_ = following_dynamic_gate
                ? mpc_target_speed
                : clamp_value(0.45 * predicted_speed + 0.55 * mpc_target_speed, 0.0, speed_limit);
            last_command_.target_speed = commanded_speed_;
            if (tracked_vehicle) {
                direct_yaw_rate_command = clamp_value(
                    mpc_command.target_yaw_rate,
                    -geometry_.max_yaw_rate,
                    geometry_.max_yaw_rate);
                const double pivot_heading_threshold =
                    following_dynamic_gate ? 0.48 : 0.72;
                if (std::abs(mpc_command.heading_error) >= pivot_heading_threshold) {
                    const double pivot_direction =
                        std::abs(direct_yaw_rate_command) > 1e-4
                            ? direct_yaw_rate_command
                            : -mpc_command.heading_error;
                    commanded_speed_ = 0.0;
                    last_command_.target_speed = 0.0;
                    direct_yaw_rate_command = std::copysign(
                        std::max(std::abs(direct_yaw_rate_command), 0.32),
                        pivot_direction);
                }
                use_direct_yaw_rate_command = true;
                commanded_steer_angle_ = 0.0;
                last_command_.target_curvature =
                    std::abs(last_command_.target_speed) > 1e-4
                        ? clamp_value(
                              direct_yaw_rate_command / last_command_.target_speed,
                              -geometry_.max_curvature,
                              geometry_.max_curvature)
                        : 0.0;
            } else {
                commanded_steer_angle_ = clamp_value(
                    0.65 * integrated_steer + 0.35 * mpc_command.target_steer_angle,
                    -geometry_.max_steer_angle,
                    geometry_.max_steer_angle);
                last_command_.target_curvature =
                    curvature_from_steer(geometry_, commanded_steer_angle_);
            }
            tracker_cross_track_error_ = std::abs(mpc_command.cross_track_error);
            tracker_heading_error_deg_ =
                std::abs(mpc_command.heading_error) * 180.0 / kPi;
            last_mpc_command_ = mpc_command;
            control_source_ =
                navigation_target_kind_ == NavigationTargetKind::ExplorationFrontier
                    ? HardwareControlSource::FrontierMpc
                    : (following_dynamic_gate
                           ? HardwareControlSource::GateMpc
                           : HardwareControlSource::StructuredMpc);
            if (navigation_target_kind_ == NavigationTargetKind::PhysicalGate) {
                exploration_state_ = UnstructuredExplorationState::TrackingGateClothoid;
            }
            structured_anchor_hint_index_ = std::max(mpc_command.anchor_index, 0);
        } else {
            commanded_speed_ = 0.0;
            commanded_steer_angle_ = clamp_value(
                0.60 * commanded_steer_angle_ + 0.40 * measured_equivalent_steer,
                -geometry_.max_steer_angle,
                geometry_.max_steer_angle);
            last_command_.target_speed = 0.0;
            last_command_.target_curvature = 0.0;
            last_mpc_command_.reset();
        }
    } else {
        commanded_speed_ = 0.0;
        commanded_steer_angle_ = 0.0;
        last_command_.target_speed = 0.0;
        last_command_.target_curvature = 0.0;
        last_mpc_command_.reset();
    }

    const bool lidar_front_blocked =
        estimate_.front_lidar_distance > 0.0 &&
        estimate_.front_lidar_distance < config_.localization.obstacle_stop_distance_m;
    const bool static_map_front_block_only =
        !mixed_gate_active &&
        world_has_structured_reference(world_) &&
        compact_hardware_mixed_lab_world(world_) &&
        !world_.obstacles().empty() &&
        !lidar_pressure_from_hits(
            world_,
            lidar_hits_,
            estimate_.yaw,
            config_.localization.obstacle_stop_distance_m + 0.03,
            config_.localization.front_sector_half_angle_rad,
            true);
    const bool lidar_front_blocked_for_recovery =
        estimate_.front_lidar_distance > 0.0 &&
        estimate_.front_lidar_distance <
            (gap_recovery_turn_active_
                 ? config_.localization.obstacle_stop_distance_m + 0.03
                 : config_.localization.obstacle_stop_distance_m);
    const double lateral_stop_distance = std::max(
        config_.localization.min_valid_range_m + 0.02,
        config_.localization.obstacle_stop_distance_m - 0.07);
    const bool authoritative_clothoid_mpc_active =
        !tracked_vehicle &&
        have_reference_trajectory &&
        last_mpc_command_.has_value() &&
        (open_ended_unstructured_exploration(world_) ||
         navigation_target_kind_ == NavigationTargetKind::PhysicalGate);
    double motion_heading_local = 0.0;
    bool have_motion_heading = false;
    if (authoritative_clothoid_mpc_active) {
        // The gate/frontier centre is only the terminal condition. Once a valid
        // clothoid exists, obstacle-sector checks and steering decisions must
        // use its local tangent; using the robot-to-target chord made a curved
        // reference look blocked and handed control back to point pursuit.
        const int tangent_index = std::clamp(
            last_mpc_command_->anchor_index + 1,
            0,
            static_cast<int>(reference_trajectory_.size()) - 1);
        motion_heading_local = wrap_angle(
            reference_trajectory_[static_cast<size_t>(tangent_index)].yaw -
            estimate_.yaw);
        have_motion_heading = true;
    } else if (locked_gap_goal_.has_value()) {
        motion_heading_local =
            wrap_angle(angle_to(estimate_.position, *locked_gap_goal_) - estimate_.yaw);
        have_motion_heading = true;
    } else if (chosen_gate_index_ >= 0 &&
               chosen_gate_index_ < static_cast<int>(gate_specs_.size())) {
        motion_heading_local = wrap_angle(
            angle_to(estimate_.position, gate_specs_[static_cast<size_t>(chosen_gate_index_)].position) -
            estimate_.yaw);
        have_motion_heading = true;
    }
    const double motion_sector_clearance =
        have_motion_heading
            ? sector_min_clearance(
                  lidar_hits_,
                  estimate_.yaw,
                  motion_heading_local,
                  mixed_gate_active ? 0.30 : 0.50,
                  config_.localization.max_range_m)
            : estimate_.front_lidar_distance;
    const bool lidar_side_clearance_blocked =
        use_dynamic_gap_gates_ &&
        have_reference_trajectory &&
        last_command_.target_speed > 0.02 &&
        motion_sector_clearance > 0.0 &&
        motion_sector_clearance < lateral_stop_distance;
    bool use_gap_recovery_turn = false;
    bool handled_gap_priority_tracking = false;
    const double recovery_turn_clearance =
        std::max(config_.localization.min_valid_range_m + 0.005, 0.10);
    const double recovery_turn_enter_heading = 0.12;
    const double recovery_turn_hold_heading = 0.05;
    const double gap_acquire_clearance = std::max(
        config_.localization.min_valid_range_m + 0.04,
        config_.localization.obstacle_stop_distance_m - 0.01);
    const double gap_acquire_forward_clearance = forward_creep_gate_car
        ? std::max(config_.localization.obstacle_stop_distance_m - 0.010,
                   config_.localization.min_valid_range_m + 0.035)
        : std::max(config_.localization.obstacle_stop_distance_m + 0.07,
                   config_.localization.min_valid_range_m + 0.14);
    const bool manual_clothoid_tangent_acquisition =
        authoritative_clothoid_mpc_active &&
        compact_unstructured_car &&
        navigation_target_kind_ == NavigationTargetKind::PhysicalGate &&
        have_motion_heading &&
        std::abs(motion_heading_local) >
            (gap_recovery_turn_active_ ? 0.35 : 0.70);
    if ((!authoritative_clothoid_mpc_active || manual_clothoid_tangent_acquisition) &&
        !scanning_startup &&
        use_dynamic_gap_gates_ &&
        have_reference_trajectory &&
        (!strict_locked_motion || locked_gap_goal_.has_value()) &&
        have_motion_heading) {
        // Normally this is a fallback for modes without an authoritative
        // clothoid. In compact open-ended exploration it is also a hysteretic tangent
        // acquisition stage for large errors: it aims at the clothoid tangent,
        // never at the gate-centre chord, and remains a forward-only arc.
        const double heading_error = motion_heading_local;
        const double heading_threshold = manual_clothoid_tangent_acquisition
            ? (gap_recovery_turn_active_ ? 0.35 : 0.70)
            : (gap_recovery_turn_active_
                   ? config_.gap_extraction.gap_acquire_hold_heading_rad
                   : config_.gap_extraction.gap_acquire_enter_heading_rad);
        const bool motion_heading_supported =
            motion_sector_clearance <= 0.0 || motion_sector_clearance > gap_acquire_clearance;
        if (std::abs(heading_error) > heading_threshold && motion_heading_supported) {
            handled_gap_priority_tracking = true;
            gap_recovery_turn_active_ = true;
            use_gap_recovery_turn = true;
            use_direct_yaw_rate_command = true;
            const double yaw_rate_limit = strict_locked_motion
                ? 0.65 * config_.drive.max_yaw_rate
                : (compact_mixed_gate_active ? 0.42 : 0.90) * config_.drive.max_yaw_rate;
            direct_yaw_rate_command = clamp_value(
                config_.gap_extraction.gap_acquire_yaw_gain * heading_error,
                -yaw_rate_limit,
                yaw_rate_limit);

            double acquire_speed = 0.0;
            const double turn_in_place_heading = std::max(
                compact_hardware_car
                    ? std::max(config_.gap_extraction.gap_acquire_turn_in_place_heading_rad, 1.55)
                    : config_.gap_extraction.gap_acquire_turn_in_place_heading_rad,
                heading_threshold + 0.02);
            double drive_heading_limit = strict_locked_motion
                ? std::max(config_.gap_extraction.strict_locked_gate_drive_heading_rad, turn_in_place_heading)
                : turn_in_place_heading;
            if (compact_unstructured_car) {
                drive_heading_limit = std::max(drive_heading_limit, 1.50);
            }
            const bool locked_gate_drive_allowed =
                !strict_locked_motion || locked_gap_goal_.has_value();
            if (locked_gate_drive_allowed &&
                motion_sector_clearance > gap_acquire_forward_clearance &&
                std::abs(heading_error) < drive_heading_limit) {
                const double heading_scale = clamp_value(
                    (drive_heading_limit - std::abs(heading_error)) /
                        std::max(
                            drive_heading_limit -
                                config_.gap_extraction.gap_acquire_hold_heading_rad,
                            1e-3),
                    0.0,
                    1.0);
                const double clearance_scale = clamp_value(
                    (motion_sector_clearance - gap_acquire_forward_clearance) /
                        (compact_hardware_car ? 0.12 : 0.40),
                    0.0,
                    1.0);
                const double creep_speed_cap = strict_locked_motion
                    ? std::min(config_.gap_extraction.gap_acquire_creep_speed_mps,
                               config_.gap_extraction.strict_locked_gate_creep_speed_mps)
                    : config_.gap_extraction.gap_acquire_creep_speed_mps;
                acquire_speed = clamp_value(
                    creep_speed_cap *
                        (0.30 + 0.70 * heading_scale) *
                        (0.45 + 0.55 * clearance_scale),
                    0.0,
                    creep_speed_cap);
            }

            commanded_speed_ = acquire_speed;
            commanded_steer_angle_ = 0.0;
            last_command_.target_speed = acquire_speed;
            last_command_.target_curvature = 0.0;
            last_mpc_command_.reset();
        }
    }
    if (!authoritative_clothoid_mpc_active &&
        !handled_gap_priority_tracking &&
        !scanning_startup &&
        lidar_front_blocked_for_recovery &&
        use_dynamic_gap_gates_ &&
        (!strict_locked_motion || locked_gap_goal_.has_value()) &&
        chosen_gate_index_ >= 0 &&
        chosen_gate_index_ < static_cast<int>(gate_specs_.size()) &&
        estimate_.min_lidar_distance > recovery_turn_clearance) {
        const Vec2 gate_target = gate_specs_[static_cast<size_t>(chosen_gate_index_)].position;
        const double heading_error = wrap_angle(angle_to(estimate_.position, gate_target) - estimate_.yaw);
        const double heading_threshold =
            gap_recovery_turn_active_ ? recovery_turn_hold_heading : recovery_turn_enter_heading;
        const double gate_distance = distance(estimate_.position, gate_target);
        const bool gate_is_reachable =
            gate_distance <= std::max(config_.gap_extraction.max_target_distance_m * 1.35, 0.45);
        if (std::abs(heading_error) > heading_threshold && gate_is_reachable) {
            gap_recovery_turn_active_ = true;
            use_gap_recovery_turn = true;
            use_direct_yaw_rate_command = true;
            const double yaw_rate_limit =
                (compact_mixed_gate_active ? 0.42 : 0.85) * config_.drive.max_yaw_rate;
            direct_yaw_rate_command = clamp_value(
                1.6 * heading_error,
                -yaw_rate_limit,
                yaw_rate_limit);
            commanded_speed_ = 0.0;
            commanded_steer_angle_ = 0.0;
            last_command_.target_speed = 0.0;
            last_command_.target_curvature = 0.0;
        } else {
            gap_recovery_turn_active_ = false;
        }
    } else if (!handled_gap_priority_tracking &&
               !scanning_startup &&
               use_dynamic_gap_gates_ &&
               !have_reference_trajectory) {
        if (open_ended_unstructured_exploration(world_)) {
            // One LiDAR-sector selector owns both cases: it returns zero yaw in
            // an open corridor and a bounded forward arc near obstacles or
            // bounds. This prevents a barely-valid straight fallback from
            // undoing a boundary-avoidance turn on the following cycle.
            bool exploration_motion = false;
            if (sim_time_ < lidar_reverse_reposition_until_s_) {
                exploration_motion = apply_reverse_lidar_reposition();
            }
            if (!exploration_motion) {
                exploration_motion = apply_forward_gate_search();
            }
            if (!exploration_motion) {
                exploration_motion = apply_reverse_lidar_reposition();
            }
            if (!exploration_motion) {
                gap_recovery_turn_active_ = false;
                use_gap_recovery_turn = false;
                use_direct_yaw_rate_command = false;
                commanded_speed_ = 0.0;
                commanded_steer_angle_ = 0.0;
                last_command_.target_speed = 0.0;
                last_command_.target_curvature = 0.0;
                last_mpc_command_.reset();
                exploration_state_ = UnstructuredExplorationState::Hold;
            }
        } else if (strict_locked_motion) {
            gap_recovery_turn_active_ = true;
            use_gap_recovery_turn = true;
            use_direct_yaw_rate_command = true;
            if (!apply_forward_gate_search() && !apply_strict_scan_escape()) {
                direct_yaw_rate_command = startup_scan_direction_ * clamp_value(
                    std::abs(config_.gap_extraction.startup_scan_yaw_rate),
                    0.0,
                    0.65 * config_.drive.max_yaw_rate);
                commanded_speed_ = 0.0;
                commanded_steer_angle_ = 0.0;
                last_command_.target_speed = 0.0;
                last_command_.target_curvature = 0.0;
                last_mpc_command_.reset();
            }
        } else {
        const double heading_limit = clamp_value(
            config_.gap_extraction.recovery_heading_search_half_angle_rad,
            0.45,
            0.95 * kPi);
        const double sector_half_width = clamp_value(
            config_.gap_extraction.recovery_sector_half_angle_rad,
            0.10,
            0.45);
        const double recovery_search_range = clamp_value(
            config_.gap_extraction.planning_max_range_m,
            config_.localization.min_valid_range_m + 0.10,
            config_.localization.max_range_m);
        const double heading_step = 10.0 * kPi / 180.0;
        const double forward_sector_clearance = sector_min_clearance(
            lidar_hits_,
            estimate_.yaw,
            0.0,
            sector_half_width,
            recovery_search_range);
        double best_heading = 0.0;
        double best_sector_clearance = -std::numeric_limits<double>::infinity();
        double best_heading_score = -std::numeric_limits<double>::infinity();
        for (double heading = -heading_limit; heading <= heading_limit + 1e-6; heading += heading_step) {
            const double sector_clearance = sector_min_clearance(
                lidar_hits_,
                estimate_.yaw,
                heading,
                sector_half_width,
                recovery_search_range);
            const double heading_score = sector_clearance - 0.18 * std::abs(heading);
            if (heading_score > best_heading_score) {
                best_heading_score = heading_score;
                best_sector_clearance = sector_clearance;
                best_heading = heading;
            }
        }

        const double recovery_clearance_margin = 0.05;
        const double forward_creep_clearance = std::max(
            config_.localization.obstacle_stop_distance_m + recovery_clearance_margin,
            config_.localization.min_valid_range_m + 0.10);
        const double turn_only_clearance = std::max(
            config_.localization.obstacle_stop_distance_m - 0.02,
            config_.localization.min_valid_range_m + 0.04);
        const bool forward_sector_is_open =
            forward_sector_clearance > forward_creep_clearance;
        const bool forward_heading_competitive =
            best_sector_clearance <= forward_sector_clearance + 0.12;
        if (forward_sector_is_open && forward_heading_competitive) {
            best_heading = 0.0;
            best_sector_clearance = forward_sector_clearance;
        }
        const double no_reference_close_clearance = std::max(
            config_.localization.obstacle_stop_distance_m + 0.04,
            config_.localization.min_valid_range_m + 0.12);
        const bool no_reference_close_obstacle =
            estimate_.min_lidar_distance > 0.0 &&
            estimate_.min_lidar_distance < no_reference_close_clearance &&
            diagnostics_.close_lidar_points > 0;
        if (passed_unstructured_gap_count_ > 0 || no_reference_close_obstacle) {
            gap_recovery_turn_active_ = true;
            use_gap_recovery_turn = true;
            use_direct_yaw_rate_command = true;
            const double search_heading =
                startup_scan_direction_ *
                (no_reference_close_obstacle ? 0.52 : 0.42);
            direct_yaw_rate_command = clamp_value(
                1.2 * search_heading,
                -0.55 * config_.drive.max_yaw_rate,
                0.55 * config_.drive.max_yaw_rate);
            commanded_speed_ = 0.0;
            commanded_steer_angle_ = 0.0;
            last_command_.target_speed = 0.0;
            last_command_.target_curvature = 0.0;
            last_mpc_command_.reset();
        } else if (best_sector_clearance > forward_creep_clearance) {
            gap_recovery_turn_active_ = true;
            use_gap_recovery_turn = true;
            use_direct_yaw_rate_command = true;
            const double yaw_rate_limit =
                (compact_mixed_gate_active ? 0.42 : 0.65) * config_.drive.max_yaw_rate;
            direct_yaw_rate_command = clamp_value(
                1.4 * best_heading,
                -yaw_rate_limit,
                yaw_rate_limit);
            const double creep_alignment = clamp_value(
                1.0 - std::abs(best_heading) / std::max(heading_limit, 1e-3),
                0.0,
                1.0);
            const double creep_clearance_scale = clamp_value(
                (best_sector_clearance - forward_creep_clearance) / 0.45,
                0.0,
                1.0);
            commanded_speed_ = clamp_value(
                config_.gap_extraction.recovery_creep_speed_mps *
                    (0.45 + 0.55 * creep_alignment) *
                    (0.40 + 0.60 * creep_clearance_scale),
                0.0,
                config_.gap_extraction.recovery_creep_speed_mps);
            commanded_steer_angle_ = 0.0;
            last_command_.target_speed = commanded_speed_;
            last_command_.target_curvature = 0.0;
        } else if (best_sector_clearance > turn_only_clearance && std::abs(best_heading) > 0.08) {
            gap_recovery_turn_active_ = true;
            use_gap_recovery_turn = true;
            use_direct_yaw_rate_command = true;
            const double yaw_rate_limit =
                (compact_mixed_gate_active ? 0.42 : 0.55) * config_.drive.max_yaw_rate;
            direct_yaw_rate_command = clamp_value(
                1.2 * best_heading,
                -yaw_rate_limit,
                yaw_rate_limit);
            commanded_speed_ = 0.0;
            commanded_steer_angle_ = 0.0;
            last_command_.target_speed = 0.0;
            last_command_.target_curvature = 0.0;
        } else if (lidar_front_blocked_for_recovery &&
                   std::abs(best_heading) > config_.gap_extraction.recovery_escape_turn_min_heading_rad) {
            gap_recovery_turn_active_ = true;
            use_gap_recovery_turn = true;
            use_direct_yaw_rate_command = true;
            direct_yaw_rate_command = clamp_value(
                config_.gap_extraction.recovery_escape_turn_yaw_gain * best_heading,
                -0.75 * config_.drive.max_yaw_rate,
                0.75 * config_.drive.max_yaw_rate);
            commanded_speed_ = 0.0;
            commanded_steer_angle_ = 0.0;
            last_command_.target_speed = 0.0;
            last_command_.target_curvature = 0.0;
        } else {
            gap_recovery_turn_active_ = false;
        }
        }
    } else if (!handled_gap_priority_tracking) {
        gap_recovery_turn_active_ = false;
    }
    if (!use_gap_recovery_turn &&
        mixed_gate_active &&
        locked_gap_goal_.has_value() &&
        lidar_front_blocked &&
        have_motion_heading &&
        std::abs(motion_heading_local) > 0.05 &&
        estimate_.min_lidar_distance > config_.localization.min_valid_range_m + 0.04) {
        gap_recovery_turn_active_ = true;
        use_gap_recovery_turn = true;
        use_direct_yaw_rate_command = true;
        const double yaw_rate_limit =
            (compact_mixed_gate_active ? 0.42 : 0.65) * config_.drive.max_yaw_rate;
        direct_yaw_rate_command = clamp_value(
            1.25 * motion_heading_local,
            -yaw_rate_limit,
            yaw_rate_limit);
        commanded_speed_ = 0.0;
        commanded_steer_angle_ = 0.0;
        last_command_.target_speed = 0.0;
        last_command_.target_curvature = 0.0;
        last_mpc_command_.reset();
    }
    const bool dynamic_gap_safety_stop_enabled =
        use_dynamic_gap_gates_ && following_dynamic_gate && have_reference_trajectory;
    const bool compact_unstructured_search_safety_enabled =
        compact_unstructured_car &&
        use_dynamic_gap_gates_ &&
        !have_reference_trajectory;
    bool locked_gap_about_to_cross = false;
    if (dynamic_gap_safety_stop_enabled && locked_gap_goal_.has_value()) {
        const double crossing_progress =
            locked_gap_longitudinal_progress(estimate_.position) + std::max(geometry_.cg_to_front, 0.0);
        const double lateral_offset = std::abs(locked_gap_lateral_offset(estimate_.position));
        const double gate_distance = distance(estimate_.position, *locked_gap_goal_);
        const double crossing_margin =
            std::max(config_.gap_extraction.gap_crossing_margin_m, 0.0);
        const double pass_lateral_limit = locked_gap_pass_lateral_limit();
        const double near_crossing_margin = clamp_value(
            0.46 * std::max(config_.gap_extraction.gap_goal_tolerance_m, 0.0),
            0.045,
            0.085);
        const bool micro_hardware_lab = compact_hardware_mixed_lab_world(world_);
        const double near_crossing_gate_radius = micro_hardware_lab
            ? clamp_value(
                  1.25 * std::max(config_.gap_extraction.gap_goal_tolerance_m, 0.0),
                  0.045,
                  std::max(config_.gap_extraction.gap_goal_acceptance_radius_m, 0.055))
            : clamp_value(
                  1.25 * std::max(config_.gap_extraction.gap_goal_tolerance_m, 0.0),
                  0.18,
                  std::max(std::min(config_.gap_extraction.gap_goal_acceptance_radius_m, 0.24), 0.18));
        locked_gap_about_to_cross =
            lateral_offset <= pass_lateral_limit &&
            crossing_progress + near_crossing_margin >= crossing_margin &&
            gate_distance <= near_crossing_gate_radius;
    }
    const bool mixed_gate_side_motion_clear =
        mixed_gate_active &&
        have_motion_heading &&
        std::abs(motion_heading_local) > 16.0 * kPi / 180.0 &&
        motion_sector_clearance > std::max(
            config_.localization.obstacle_stop_distance_m + 0.05,
            config_.localization.min_valid_range_m + 0.16);
    const bool effective_lidar_front_blocked =
        lidar_front_blocked && !mixed_gate_side_motion_clear && !static_map_front_block_only;
    const bool validated_reverse_reposition =
        allow_reverse_recovery_motion &&
        last_command_.target_speed < -1e-4;
    if ((config_.planner_safety_stop_enabled ||
         dynamic_gap_safety_stop_enabled ||
         compact_unstructured_search_safety_enabled) &&
        (!scanning_startup || compact_unstructured_car) &&
        (!use_gap_recovery_turn || compact_unstructured_car) &&
        ((!validated_reverse_reposition && effective_lidar_front_blocked) ||
         lidar_side_clearance_blocked)) {
        safety_stop_active_ = true;
        last_command_.safety_stop = true;
        commanded_speed_ = 0.0;
        commanded_steer_angle_ = 0.0;
        last_command_.target_speed = 0.0;
        last_command_.target_curvature = 0.0;
        last_mpc_command_.reset();
        if (dynamic_gap_safety_stop_enabled &&
            locked_gap_goal_.has_value() &&
            !locked_gap_about_to_cross) {
            if (!mixed_gate_active) {
                dynamic_gap_tracks_.clear();
                next_dynamic_gap_track_id_ = 1;
                restart_unstructured_scan();
            }
        }
    }

    if (world_has_structured_reference(world_) && !mixed_gate_active && !safety_stop_active_) {
        const bool tiny_indoor_loop = tiny_indoor_structured_loop(world_);
        const bool compact_mixed_loop = compact_mixed_structured_loop(world_);
        const bool compact_mixed_open = compact_mixed_open_road(world_);
        const bool compact_mixed_road = compact_mixed_loop || compact_mixed_open;
        const bool closed_structured_course = structured_road_is_closed_loop(world_);
        const double active_distance = structured_goal_active_distance(world_, config_.goal_tolerance_m);
        const bool keep_tracking = !goal_reached_ && distance_to_goal_ > active_distance;
        if (compact_structured_world(world_) &&
            !(tracked_vehicle && tiny_indoor_loop) &&
            keep_tracking &&
            have_reference_trajectory &&
            last_mpc_command_.has_value()) {
            const bool closed_structured_loop = structured_road_is_closed_loop(world_);
            const double heading_error = last_mpc_command_->heading_error;
            const double heading_error_abs = std::abs(heading_error);
            const bool early_progress = structured_progress_s_ < std::max(0.05, 0.10 * structured_course_span_m(world_));
            const double enter_heading_rad =
                tiny_indoor_loop ? deg_to_rad(42.0)
                : (micro_structured_world(world_) ? deg_to_rad(48.0)
                                                  : (compact_mixed_road ? deg_to_rad(50.0) : deg_to_rad(62.0)));
            const double min_turn_yaw_rate =
                tiny_indoor_loop ? 0.14
                : (micro_structured_world(world_) ? 0.35
                                                  : (compact_mixed_road ? 0.16 : 0.25));
            const double yaw_gain =
                tiny_indoor_loop ? 0.70
                : (micro_structured_world(world_) ? 1.55
                                                  : (compact_mixed_road ? 1.00 : 1.20));
            bool allow_direct_yaw_acquire =
                !closed_structured_loop ||
                (tiny_indoor_loop ? early_progress : (early_progress || std::abs(estimate_.speed) < 0.02));
            if (compact_mixed_road && !tracked_vehicle) {
                allow_direct_yaw_acquire = false;
            }
            if (!closed_structured_loop && micro_structured_world(world_)) {
                allow_direct_yaw_acquire = early_progress && std::abs(estimate_.speed) < 0.012;
            }
            const bool suppress_car_like_mixed_direct_yaw =
                compact_mixed_road && !tracked_vehicle;
            const bool severe_indoor_heading =
                !suppress_car_like_mixed_direct_yaw &&
                tiny_indoor_loop &&
                closed_structured_loop &&
                std::abs(estimate_.speed) < 0.012 &&
                heading_error_abs > deg_to_rad(100.0);
            const bool stalled_indoor_heading =
                !suppress_car_like_mixed_direct_yaw &&
                tiny_indoor_loop &&
                closed_structured_loop &&
                !early_progress &&
                std::abs(estimate_.speed) < 0.020 &&
                tracker_cross_track_error_ < 0.030 &&
                heading_error_abs > deg_to_rad(72.0);
            if ((allow_direct_yaw_acquire &&
                 (heading_error_abs > enter_heading_rad ||
                  (early_progress &&
                   heading_error_abs > deg_to_rad(tiny_indoor_loop ? 32.0 : 40.0)))) ||
                severe_indoor_heading ||
                stalled_indoor_heading) {
                use_direct_yaw_rate_command = true;
                const double yaw_rate_limit =
                    (compact_mixed_road ? 0.55 : 0.85) *
                    config_.drive.max_yaw_rate;
                direct_yaw_rate_command = clamp_value(
                    -yaw_gain * heading_error,
                    -yaw_rate_limit,
                    yaw_rate_limit);
                if (std::abs(direct_yaw_rate_command) < min_turn_yaw_rate) {
                    direct_yaw_rate_command =
                        std::copysign(
                            min_turn_yaw_rate,
                            direct_yaw_rate_command == 0.0 ? -heading_error : direct_yaw_rate_command);
                }
                const double tracked_acquire_speed =
                    tracked_vehicle
                        ? clamp_value(
                              structured_tracking_speed_floor(world_, speed_limit) *
                                  (heading_error_abs > deg_to_rad(105.0) ? 0.35 : 0.70),
                              0.0,
                              tiny_indoor_loop ? 0.018 : 0.035)
                        : 0.0;
                commanded_speed_ = tracked_vehicle ? tracked_acquire_speed : 0.0;
                commanded_steer_angle_ = 0.0;
                last_command_.target_speed = commanded_speed_;
                last_command_.target_curvature = 0.0;
            }
        }

        const double slowdown_radius = structured_slowdown_radius(world_, config_.goal_slowdown_radius_m);
        const double speed_scale = structured_tracking_speed_scale(
            distance_to_goal_,
            slowdown_radius,
            tracker_heading_error_deg_,
            tracker_cross_track_error_,
            last_command_.target_curvature,
            geometry_);
        last_command_.target_speed *= speed_scale;
        if (tracked_vehicle && use_direct_yaw_rate_command && keep_tracking) {
            const double direct_crawl_floor =
                tiny_indoor_loop ? 0.012 : (micro_structured_world(world_) ? 0.018 : 0.020);
            const double direct_crawl_ceiling =
                tiny_indoor_loop ? 0.022 : (micro_structured_world(world_) ? 0.035 : 0.045);
            last_command_.target_speed = std::min(
                std::max(last_command_.target_speed, direct_crawl_floor),
                direct_crawl_ceiling);
        }
        if (compact_structured_world(world_) &&
            have_reference_trajectory &&
            keep_tracking &&
            !use_direct_yaw_rate_command) {
            const double heading_stop_deg =
                tiny_indoor_loop ? 55.0 : (micro_structured_world(world_) ? 55.0 : (compact_mixed_road ? 70.0 : 38.0));
            const double heading_slow_deg =
                tiny_indoor_loop ? 24.0 : (micro_structured_world(world_) ? 24.0 : (compact_mixed_road ? 32.0 : 20.0));
            const double cross_stop_m =
                tiny_indoor_loop ? 0.040 : (micro_structured_world(world_) ? 0.05 : (compact_mixed_open ? 0.22 : (compact_mixed_loop ? 0.18 : 0.07)));
            const double cross_slow_m =
                tiny_indoor_loop ? 0.018 : (micro_structured_world(world_) ? 0.025 : (compact_mixed_open ? 0.12 : (compact_mixed_loop ? 0.09 : 0.04)));
            if (tracker_cross_track_error_ > cross_stop_m) {
                last_command_.target_speed =
                    (compact_mixed_road || closed_structured_course)
                        ? std::min(std::max(last_command_.target_speed, 0.012), 0.018)
                        : 0.0;
            } else if (tracker_heading_error_deg_ > heading_stop_deg) {
                last_command_.target_speed =
                    tiny_indoor_loop ? std::min(std::max(last_command_.target_speed, 0.012), 0.018)
                                     : (compact_mixed_road ? std::min(std::max(last_command_.target_speed, 0.012), 0.020)
                                                           : (tracked_vehicle && micro_structured_world(world_)
                                                                  ? std::min(std::max(last_command_.target_speed, 0.018), 0.032)
                                                                  : (closed_structured_course
                                                                         ? std::min(std::max(last_command_.target_speed, 0.012), 0.018)
                                                                         : 0.0)));
            } else if (tracker_heading_error_deg_ > heading_slow_deg ||
                       tracker_cross_track_error_ > cross_slow_m) {
                const double slow_floor =
                    tiny_indoor_loop ? 0.016 : (micro_structured_world(world_) ? 0.014 : (compact_mixed_road ? 0.018 : 0.0));
                last_command_.target_speed =
                    std::min(std::max(last_command_.target_speed, slow_floor),
                             tiny_indoor_loop ? 0.030 : (compact_mixed_road ? 0.045 : 0.04));
            }
        }
        if (have_reference_trajectory &&
            keep_tracking &&
            !use_direct_yaw_rate_command) {
            if (tiny_indoor_loop) {
                last_command_.target_speed = std::min(last_command_.target_speed, 0.040);
            }
            const double heading_floor_deg =
                tiny_indoor_loop ? 18.0 : (micro_structured_world(world_) ? 10.0 : (compact_mixed_road ? 20.0 : 14.0));
            const double cross_floor_m =
                tiny_indoor_loop ? 0.015 : (micro_structured_world(world_) ? 0.02 : (compact_mixed_open ? 0.12 : (compact_mixed_loop ? 0.08 : 0.03)));
            if ((tiny_indoor_loop || micro_structured_world(world_) || compact_mixed_road) &&
                tracker_heading_error_deg_ < (compact_mixed_road ? 50.0 : 42.0) &&
                tracker_cross_track_error_ < (compact_mixed_open ? 0.18 : (compact_mixed_loop ? 0.12 : 0.025))) {
                last_command_.target_speed = std::max(last_command_.target_speed, 0.018);
            }
            if (tracked_vehicle &&
                micro_structured_world(world_) &&
                tracker_heading_error_deg_ < 140.0 &&
                tracker_cross_track_error_ < 0.10) {
                last_command_.target_speed = std::max(last_command_.target_speed, 0.020);
            }
            if (tracker_heading_error_deg_ < heading_floor_deg &&
                tracker_cross_track_error_ < cross_floor_m) {
                last_command_.target_speed = std::max(
                    last_command_.target_speed,
                    structured_tracking_speed_floor(world_, speed_limit));
            }
        }
        commanded_speed_ = last_command_.target_speed;
    }

    if (strict_locked_motion && !locked_gap_goal_.has_value() && !allow_unlocked_recovery_motion) {
        commanded_speed_ = 0.0;
        last_command_.target_speed = 0.0;
        if (!use_direct_yaw_rate_command) {
            commanded_steer_angle_ = 0.0;
            last_command_.target_curvature = 0.0;
        }
        last_mpc_command_.reset();
    }

    const bool micro_hardware_recovery_world =
        config_.vehicle_model == VehicleModelKind::CarLikeBicycle &&
        (compact_hardware_mixed_lab_world(world_) || compact_unstructured_car);
    if (micro_hardware_recovery_world &&
        !goal_reached_ &&
        sim_time_ < stuck_recovery_until_s_) {
        clear_locked_gap_goal();
        gates_.clear();
        gate_specs_.clear();
        dynamic_gap_tracks_.clear();
        visible_gate_indices_.clear();
        chosen_gate_index_ = -1;
        diagnostics_.candidate_gates = 0;
        diagnostics_.chosen_gate_distance = std::numeric_limits<double>::infinity();

        safety_stop_active_ = false;
        gap_recovery_turn_active_ = true;
        const bool reverse_unstructured_recovery = compact_unstructured_car;
        allow_reverse_recovery_motion = reverse_unstructured_recovery;
        use_direct_yaw_rate_command = true;
        direct_yaw_rate_command = clamp_value(
            stuck_recovery_direction_ *
                (reverse_unstructured_recovery ? 0.20 : 0.58),
            -0.70 * config_.drive.max_yaw_rate,
            0.70 * config_.drive.max_yaw_rate);
        last_command_.target_speed =
            reverse_unstructured_recovery ? -0.035 : 0.0;
        last_command_.target_curvature = 0.0;
        commanded_speed_ = last_command_.target_speed;
        commanded_steer_angle_ = 0.0;
        last_mpc_command_.reset();
    }

    if (forward_creep_gate_car &&
        !goal_reached_ &&
        !safety_stop_active_ &&
        !allow_reverse_recovery_motion &&
        !scanning_startup &&
        !(sim_time_ < stuck_recovery_until_s_) &&
        use_direct_yaw_rate_command &&
        (have_reference_trajectory || mixed_gate_active || chosen_gate_index_ >= 0 ||
         locked_gap_goal_.has_value())) {
        const bool gate_target_active = mixed_gate_active || chosen_gate_index_ >= 0 ||
                                        locked_gap_goal_.has_value();
        const double relevant_clearance =
            have_motion_heading ? motion_sector_clearance : estimate_.front_lidar_distance;
        const double creep_clearance = gate_target_active
            ? std::max(config_.localization.obstacle_stop_distance_m - 0.020,
                       config_.localization.min_valid_range_m + 0.025)
            : std::max(config_.localization.obstacle_stop_distance_m - 0.010,
                       config_.localization.min_valid_range_m + 0.035);
        if ((relevant_clearance <= 0.0 || relevant_clearance > creep_clearance) &&
            std::abs(direct_yaw_rate_command) > 0.015) {
            const double creep_floor = compact_unstructured_car
                ? (gate_target_active ? 0.045 : 0.040)
                : (gate_target_active ? 0.012 : 0.010);
            const double creep_ceiling = compact_unstructured_car
                ? std::min(0.060, speed_limit)
                : (gate_target_active ? 0.026 : 0.022);
            last_command_.target_speed = std::min(
                std::max(last_command_.target_speed, creep_floor),
                creep_ceiling);
            commanded_speed_ = last_command_.target_speed;
        }
    }

    if (!tracked_vehicle &&
        world_is_mixed(world_) &&
        use_direct_yaw_rate_command) {
        // Mixed road mode owns an explicit structured/clothoid reference and
        // must never replace it with point-acquisition differential control.
        use_direct_yaw_rate_command = false;
        direct_yaw_rate_command = 0.0;
        commanded_steer_angle_ = 0.0;
        last_command_.target_curvature = 0.0;
    }

    const double min_target_speed =
        allow_reverse_recovery_motion
            ? -std::min(0.040, 0.35 * geometry_.max_linear_speed)
            : 0.0;
    last_command_.target_speed = clamp_value(
        last_command_.target_speed,
        min_target_speed,
        geometry_.max_linear_speed);
    commanded_speed_ = last_command_.target_speed;
    last_command_.target_curvature = clamp_value(
        last_command_.target_curvature,
        -geometry_.max_curvature,
        geometry_.max_curvature);
    if (!use_direct_yaw_rate_command) {
        commanded_steer_angle_ = steer_from_curvature(geometry_, last_command_.target_curvature);
        last_command_.target_curvature = curvature_from_steer(geometry_, commanded_steer_angle_);
    } else {
        commanded_steer_angle_ = 0.0;
    }
    const double half_track = config_.drive.track_width * 0.5;
    if (use_direct_yaw_rate_command) {
        last_command_.target_yaw_rate = clamp_value(
            direct_yaw_rate_command,
            -config_.drive.max_yaw_rate,
            config_.drive.max_yaw_rate);
    } else {
        last_command_.target_yaw_rate = clamp_value(
            last_command_.target_speed * last_command_.target_curvature,
            -config_.drive.max_yaw_rate,
            config_.drive.max_yaw_rate);
        if (world_has_structured_reference(world_) &&
            !mixed_gate_active &&
            tiny_indoor_structured_loop(world_) &&
            last_command_.target_speed > 1e-4) {
            double forward_curvature_limit =
                0.82 / std::max(half_track, 1e-3);
            if (world_.structured_preset() == StructuredMapPreset::FigureEight) {
                // Keep the figure-eight below the point where one wheel needs
                // to reverse continuously. On real hardware that tends to
                // collapse into an in-place pivot instead of a clean lobe.
                forward_curvature_limit = std::max(forward_curvature_limit, 7.0);
            }
            forward_curvature_limit = std::min(forward_curvature_limit, geometry_.max_curvature);
            const double forward_yaw_limit =
                forward_curvature_limit * last_command_.target_speed;
            last_command_.target_yaw_rate = clamp_value(
                last_command_.target_yaw_rate,
                -forward_yaw_limit,
                forward_yaw_limit);
        }
    }
    if (forward_only_tracked_tracks && last_command_.target_speed > 1e-4) {
        const double forward_yaw_limit =
            0.98 * last_command_.target_speed / std::max(half_track, 1e-3);
        last_command_.target_yaw_rate = clamp_value(
            last_command_.target_yaw_rate,
            -forward_yaw_limit,
            forward_yaw_limit);
        if (simple_tiny_tracked_loop) {
            const double loop_turn_sign = structured_loop_winding_sign(world_);
            const bool loop_not_settled =
                !structured_goal_ready_ ||
                structured_progress_s_ < 0.99 * structured_goal_progress_target_ ||
                tracker_cross_track_error_ > 0.040 ||
                tracker_heading_error_deg_ > 38.0;
            if (loop_not_settled && std::abs(loop_turn_sign) > 0.5) {
                const double yaw_floor =
                    std::min(forward_yaw_limit, std::max(0.10, 0.55 * forward_yaw_limit));
                last_command_.target_yaw_rate =
                    std::copysign(
                        std::max(std::abs(last_command_.target_yaw_rate), yaw_floor),
                        loop_turn_sign);
                last_command_.target_curvature = clamp_value(
                    last_command_.target_yaw_rate / std::max(last_command_.target_speed, 1e-4),
                    -geometry_.max_curvature,
                    geometry_.max_curvature);
            }
        }
    }
    if (compact_unstructured_car && use_direct_yaw_rate_command) {
        if (std::abs(last_command_.target_speed) <= 1e-4) {
            // Never convert a search or recovery request into opposing wheel
            // commands: with asymmetric traction that produced false encoder
            // translation and an indefinitely moving gate estimate.
            last_command_.target_yaw_rate = 0.0;
        } else {
            const double forward_arc_yaw_limit =
                0.68 * std::abs(last_command_.target_speed) /
                std::max(half_track, 1e-3);
            last_command_.target_yaw_rate = clamp_value(
                last_command_.target_yaw_rate,
                -forward_arc_yaw_limit,
                forward_arc_yaw_limit);
            last_command_.target_curvature = clamp_value(
                last_command_.target_yaw_rate / last_command_.target_speed,
                -geometry_.max_curvature,
                geometry_.max_curvature);
        }
    }

    const auto [left_wheel_speed, right_wheel_speed] =
        wheel_speeds_from_body(last_command_.target_speed, last_command_.target_yaw_rate, half_track);
    last_command_.target_left_wheel_speed_mps = left_wheel_speed;
    last_command_.target_right_wheel_speed_mps = right_wheel_speed;

    const int ff_left = wheel_speed_to_pwm(left_wheel_speed, config_.pwm.left_scale);
    const int ff_right = wheel_speed_to_pwm(right_wheel_speed, config_.pwm.right_scale);
    const bool commanding_motion =
        !safety_stop_active_ &&
        (std::abs(last_command_.target_speed) > 1e-4 || std::abs(last_command_.target_yaw_rate) > 1e-4);

    if (!commanding_motion) {
        wheel_speed_error_integral_left_ = 0.0;
        wheel_speed_error_integral_right_ = 0.0;
    }

    double measured_left_wheel_speed = measured_left_wheel_speed_;
    double measured_right_wheel_speed = measured_right_wheel_speed_;
    if (!measured_wheel_speeds_valid_) {
        const auto fallback_wheel_speeds =
            wheel_speeds_from_body(estimate_.speed, estimate_.yaw_rate, half_track);
        measured_left_wheel_speed = fallback_wheel_speeds.first;
        measured_right_wheel_speed = fallback_wheel_speeds.second;
    }

    if (measured_wheel_speeds_valid_ && commanding_motion) {
        wheel_speed_error_integral_left_ += (left_wheel_speed - measured_left_wheel_speed) * safe_dt;
        wheel_speed_error_integral_right_ += (right_wheel_speed - measured_right_wheel_speed) * safe_dt;
        wheel_speed_error_integral_left_ = clamp_value(
            wheel_speed_error_integral_left_,
            -config_.pwm.wheel_speed_integral_limit,
            config_.pwm.wheel_speed_integral_limit);
        wheel_speed_error_integral_right_ = clamp_value(
            wheel_speed_error_integral_right_,
            -config_.pwm.wheel_speed_integral_limit,
            config_.pwm.wheel_speed_integral_limit);

        const int fb_left = static_cast<int>(std::lround(
            config_.pwm.wheel_speed_kp * (left_wheel_speed - measured_left_wheel_speed) +
            config_.pwm.wheel_speed_ki * wheel_speed_error_integral_left_));
        const int fb_right = static_cast<int>(std::lround(
            config_.pwm.wheel_speed_kp * (right_wheel_speed - measured_right_wheel_speed) +
            config_.pwm.wheel_speed_ki * wheel_speed_error_integral_right_));

        last_command_.pwm_left = static_cast<int>(clamp_value(
            static_cast<double>(ff_left + fb_left),
            -config_.pwm.max_pwm,
            config_.pwm.max_pwm));
        last_command_.pwm_right = static_cast<int>(clamp_value(
            static_cast<double>(ff_right + fb_right),
            -config_.pwm.max_pwm,
            config_.pwm.max_pwm));
    } else {
        if (!measured_wheel_speeds_valid_) {
            wheel_speed_error_integral_left_ = 0.0;
            wheel_speed_error_integral_right_ = 0.0;
        }

        const double measured_speed_for_feedback = clamp_value(
            estimate_.speed,
            0.0,
            std::max(config_.cruise_speed_limit * 1.35, 0.28));
        const double measured_yaw_rate_for_feedback = clamp_value(
            estimate_.yaw_rate,
            -config_.drive.max_yaw_rate,
            config_.drive.max_yaw_rate);
        const int fb_linear = static_cast<int>(std::lround(
            config_.pwm.linear_feedback_gain * (last_command_.target_speed - measured_speed_for_feedback)));
        const int fb_yaw = static_cast<int>(std::lround(
            config_.pwm.yaw_feedback_gain * (last_command_.target_yaw_rate - measured_yaw_rate_for_feedback)));

        last_command_.pwm_left = static_cast<int>(clamp_value(
            static_cast<double>(ff_left + fb_linear - fb_yaw),
            -config_.pwm.max_pwm,
            config_.pwm.max_pwm));
        last_command_.pwm_right = static_cast<int>(clamp_value(
            static_cast<double>(ff_right + fb_linear + fb_yaw),
            -config_.pwm.max_pwm,
            config_.pwm.max_pwm));
    }

    const double max_target_wheel_speed = std::max(std::abs(left_wheel_speed), std::abs(right_wheel_speed));
    const double max_measured_wheel_speed =
        std::max(std::abs(measured_left_wheel_speed), std::abs(measured_right_wheel_speed));
    const bool demanding_motion =
        !safety_stop_active_ &&
        (max_target_wheel_speed >= std::max(config_.pwm.stall_target_speed_threshold_mps * 0.45, 0.03) ||
         (forward_only_tracked_tracks && last_command_.target_speed > 1e-4));
    const bool robot_is_still =
        max_measured_wheel_speed <= config_.pwm.stall_speed_threshold_mps;
    if (demanding_motion && robot_is_still) {
        ++no_motion_command_cycles_;
    } else {
        no_motion_command_cycles_ = 0;
    }

    const bool timed_recovery_active =
        micro_hardware_recovery_world &&
        sim_time_ < stuck_recovery_until_s_;
    const bool safety_stuck =
        safety_stop_active_ &&
        std::abs(estimate_.speed) < 0.012 &&
        ((estimate_.front_lidar_distance > 0.0 &&
          estimate_.front_lidar_distance <
              config_.localization.obstacle_stop_distance_m + 0.04) ||
         diagnostics_.close_lidar_points > 0 ||
         lidar_side_clearance_blocked);
    const bool motion_stuck =
        demanding_motion && robot_is_still;
    const bool timed_stuck_candidate =
        micro_hardware_recovery_world &&
        !goal_reached_ &&
        !timed_recovery_active &&
        (motion_stuck || safety_stuck);
    if (timed_stuck_candidate) {
        stuck_motion_elapsed_s_ += safe_dt;
    } else {
        stuck_motion_elapsed_s_ = 0.0;
    }

    const double stuck_recovery_threshold_s = safety_stuck ? 1.20 : 5.0;
    if (stuck_motion_elapsed_s_ >= stuck_recovery_threshold_s) {
        const double left_clearance = sector_min_clearance(
            lidar_hits_,
            estimate_.yaw,
            0.78,
            0.36,
            config_.localization.max_range_m);
        const double right_clearance = sector_min_clearance(
            lidar_hits_,
            estimate_.yaw,
            -0.78,
            0.36,
            config_.localization.max_range_m);
        stuck_recovery_direction_ = left_clearance >= right_clearance ? 1.0 : -1.0;
        stuck_recovery_until_s_ = sim_time_ + (safety_stuck ? 1.70 : 1.45);
        stuck_motion_elapsed_s_ = 0.0;
        no_motion_command_cycles_ = 0;
        clear_locked_gap_goal();
        gates_.clear();
        gate_specs_.clear();
        dynamic_gap_tracks_.clear();
        visible_gate_indices_.clear();
        chosen_gate_index_ = -1;
        diagnostics_.candidate_gates = 0;
        diagnostics_.chosen_gate_distance = std::numeric_limits<double>::infinity();
    }

    const int stall_boost_cycles_required =
        use_dynamic_gap_gates_ ? std::max(1, config_.pwm.stall_boost_after_cycles - 1)
                               : config_.pwm.stall_boost_after_cycles;
    const int compact_breakaway_retry_cycles = 10;
    const bool compact_breakaway_pulse =
        compact_unstructured_car &&
        no_motion_command_cycles_ >= stall_boost_cycles_required &&
        ((no_motion_command_cycles_ - stall_boost_cycles_required) %
             compact_breakaway_retry_cycles <
         2);
    const bool continuous_breakaway_boost =
        !compact_unstructured_car &&
        no_motion_command_cycles_ >= stall_boost_cycles_required;
    if (compact_breakaway_pulse ||
        continuous_breakaway_boost ||
        (forward_only_tracked_tracks && commanding_motion && robot_is_still)) {
        apply_start_motion_boost(
            std::max(config_.pwm.start_motion_pwm, config_.pwm.min_effective_pwm),
            &last_command_.pwm_left,
            &last_command_.pwm_right);
        stall_boost_active_ = true;
    }

    const bool forward_arc_command =
        compact_unstructured_car && last_command_.target_speed > 1e-4;
    if (forward_arc_command) {
        const double wheel_target_threshold = 0.010;
        const double wheel_motion_threshold = 0.008;
        const bool left_requested = left_wheel_speed > wheel_target_threshold;
        const bool right_requested = right_wheel_speed > wheel_target_threshold;

        // The old direct-PWM firmware can expose usable raw encoder deltas in
        // a cycle where the higher-level odometry readiness streak is not yet
        // valid. Use those deltas for per-wheel break-away detection instead
        // of falling back to the aggregate robot speed, where one spinning
        // wheel masks the stalled side.
        double left_speed_for_stall = std::abs(measured_left_wheel_speed);
        double right_speed_for_stall = std::abs(measured_right_wheel_speed);
        const double raw_encoder_dt_s = latest_controller_encoder_dt_ms_ * 0.001;
        const bool raw_encoder_motion_available =
            encoder_ticks_initialized_ && raw_encoder_dt_s > 1e-4;
        if (raw_encoder_motion_available) {
            const double meters_per_tick =
                (2.0 * kPi * geometry_.wheel_radius) /
                static_cast<double>(
                    std::max<std::int32_t>(geometry_.encoder_ticks_per_revolution, 1));
            left_speed_for_stall =
                std::abs(static_cast<double>(latest_controller_left_encoder_delta_)) *
                meters_per_tick / raw_encoder_dt_s;
            right_speed_for_stall =
                std::abs(static_cast<double>(latest_controller_right_encoder_delta_)) *
                meters_per_tick / raw_encoder_dt_s;
        }
        const bool have_individual_wheel_motion =
            measured_wheel_speeds_valid_ || raw_encoder_motion_available;
        left_wheel_stall_cycles_ =
            have_individual_wheel_motion &&
                    left_requested && left_speed_for_stall < wheel_motion_threshold
                ? left_wheel_stall_cycles_ + 1
                : 0;
        right_wheel_stall_cycles_ =
            have_individual_wheel_motion &&
                    right_requested && right_speed_for_stall < wheel_motion_threshold
                ? right_wheel_stall_cycles_ + 1
                : 0;

        const int per_wheel_breakaway_pwm = std::clamp(
            std::max(config_.pwm.start_motion_pwm, config_.pwm.min_effective_pwm),
            0,
            config_.pwm.max_pwm);
        // Two 10 Hz control cycles are enough to cross static friction without
        // turning the break-away level into the sustained cruise command.
        constexpr int kBreakawayHoldCycles = 2;
        const auto wheel_breakaway_pulse_due = [&](int stall_cycles) {
            return stall_cycles >= stall_boost_cycles_required &&
                   ((stall_cycles - stall_boost_cycles_required) %
                        compact_breakaway_retry_cycles ==
                    0);
        };
        if (wheel_breakaway_pulse_due(left_wheel_stall_cycles_) && left_requested) {
            left_wheel_breakaway_hold_cycles_ = kBreakawayHoldCycles;
        }
        if (wheel_breakaway_pulse_due(right_wheel_stall_cycles_) && right_requested) {
            right_wheel_breakaway_hold_cycles_ = kBreakawayHoldCycles;
        }
        if (left_wheel_breakaway_hold_cycles_ > 0 && left_requested) {
            last_command_.pwm_left = std::max(last_command_.pwm_left, per_wheel_breakaway_pwm);
            wheel_speed_error_integral_left_ = 0.0;
            stall_boost_active_ = true;
            --left_wheel_breakaway_hold_cycles_;
        } else {
            left_wheel_breakaway_hold_cycles_ = 0;
        }
        if (right_wheel_breakaway_hold_cycles_ > 0 && right_requested) {
            last_command_.pwm_right = std::max(last_command_.pwm_right, per_wheel_breakaway_pwm);
            wheel_speed_error_integral_right_ = 0.0;
            stall_boost_active_ = true;
            --right_wheel_breakaway_hold_cycles_;
        } else {
            right_wheel_breakaway_hold_cycles_ = 0;
        }
    } else {
        left_wheel_stall_cycles_ = 0;
        right_wheel_stall_cycles_ = 0;
        left_wheel_breakaway_hold_cycles_ = 0;
        right_wheel_breakaway_hold_cycles_ = 0;
    }

    if (commanding_motion) {
        last_command_.pwm_left = clamp_motion_pwm_band(
            last_command_.pwm_left,
            config_.pwm.min_effective_pwm,
            config_.pwm.max_pwm);
        last_command_.pwm_right = clamp_motion_pwm_band(
            last_command_.pwm_right,
            config_.pwm.min_effective_pwm,
            config_.pwm.max_pwm);
    }
    if (forward_only_tracked_tracks && last_command_.target_speed > 1e-4) {
        last_command_.pwm_left = std::max(last_command_.pwm_left, 0);
        last_command_.pwm_right = std::max(last_command_.pwm_right, 0);
        const int turn_start_pwm =
            std::max(config_.pwm.start_motion_pwm, config_.pwm.min_effective_pwm);
        const int arc_outer_pwm =
            simple_tiny_tracked_loop
                ? std::clamp(
                      turn_start_pwm + 8,
                      config_.pwm.min_effective_pwm + 70,
                      config_.pwm.max_pwm)
                : turn_start_pwm;
        const bool positive_turn_command = last_command_.target_yaw_rate > 0.0;
        const double measured_inner_track_speed =
            positive_turn_command ? std::abs(measured_left_wheel_speed)
                                  : std::abs(measured_right_wheel_speed);
        const double measured_outer_track_speed =
            positive_turn_command ? std::abs(measured_right_wheel_speed)
                                  : std::abs(measured_left_wheel_speed);
        const bool inner_track_lagging =
            simple_tiny_tracked_loop &&
            measured_wheel_speeds_valid_ &&
            measured_outer_track_speed > 0.045 &&
            measured_inner_track_speed < 0.006;
        const bool large_inner_track_lagging =
            large_single_turn_tracked_loop &&
            measured_wheel_speeds_valid_ &&
            measured_outer_track_speed > 0.045 &&
            measured_inner_track_speed < 0.012;
        const bool inner_needs_loaded_pwm =
            inner_track_lagging || large_inner_track_lagging || robot_is_still;
        const int arc_inner_pwm =
            simple_tiny_tracked_loop
                ? std::clamp(
                      config_.pwm.min_effective_pwm + (inner_needs_loaded_pwm ? 48 : 30),
                      config_.pwm.min_effective_pwm,
                      std::max(config_.pwm.min_effective_pwm, arc_outer_pwm - 1))
                : 0;
        const int large_loop_loaded_inner_pwm =
            large_single_turn_tracked_loop
                ? std::clamp(
                      config_.pwm.min_effective_pwm + 28,
                      config_.pwm.min_effective_pwm,
                      std::max(config_.pwm.min_effective_pwm, turn_start_pwm - 1))
                : 0;
        const int large_loop_loaded_inner_cap =
            large_single_turn_tracked_loop
                ? std::clamp(
                      large_loop_loaded_inner_pwm + 10,
                      large_loop_loaded_inner_pwm,
                      std::max(large_loop_loaded_inner_pwm, turn_start_pwm - 1))
                : 0;
        const bool stalled_turn_needed =
            robot_is_still ||
            no_motion_command_cycles_ >= stall_boost_cycles_required;
        const bool hard_turn_needed =
            stalled_turn_needed || (!simple_tiny_tracked_loop && tracker_heading_error_deg_ > 35.0);
        const int moving_turn_inner_cap =
            simple_tiny_tracked_loop
                ? arc_inner_pwm
                : (large_single_turn_tracked_loop && inner_needs_loaded_pwm
                       ? large_loop_loaded_inner_cap
                       : std::max(config_.pwm.min_effective_pwm, turn_start_pwm - 70));
        const int turn_inner_cap = stalled_turn_needed
            ? (simple_tiny_tracked_loop
                   ? arc_inner_pwm
                   : (large_single_turn_tracked_loop ? large_loop_loaded_inner_cap : 0))
            : moving_turn_inner_cap;
        const int turn_inner_floor =
            simple_tiny_tracked_loop
                ? arc_inner_pwm
                : (large_single_turn_tracked_loop && inner_needs_loaded_pwm
                       ? large_loop_loaded_inner_pwm
                       : 0);
        const int turn_outer_floor =
            stalled_turn_needed
                ? std::min(255, turn_start_pwm + 8)
                : (simple_tiny_tracked_loop && tracker_heading_error_deg_ > 35.0
                       ? std::min(config_.pwm.max_pwm, arc_outer_pwm + 8)
                       : (hard_turn_needed ? std::min(255, turn_start_pwm + 8) : arc_outer_pwm));
        enforce_forward_tracked_turn_authority(
            last_command_.target_yaw_rate,
            turn_outer_floor,
            turn_inner_floor,
            turn_inner_cap,
            &last_command_.pwm_left,
            &last_command_.pwm_right);
        if (world_has_structured_reference(world_) &&
            !mixed_gate_active &&
            !goal_reached_ &&
            !safety_stop_active_) {
            const int crawl_pwm = std::clamp(config_.pwm.min_effective_pwm, 0, config_.pwm.max_pwm);
            const bool tight_turn_command =
                simple_tiny_tracked_loop &&
                std::abs(last_command_.target_yaw_rate) > 0.04;
            if (!tight_turn_command) {
                last_command_.pwm_left = std::max(last_command_.pwm_left, crawl_pwm);
                last_command_.pwm_right = std::max(last_command_.pwm_right, crawl_pwm);
            } else if (last_command_.target_yaw_rate > 0.0) {
                last_command_.pwm_right = std::max(last_command_.pwm_right, crawl_pwm);
                last_command_.pwm_left = std::max(last_command_.pwm_left, arc_inner_pwm);
            } else {
                last_command_.pwm_left = std::max(last_command_.pwm_left, crawl_pwm);
                last_command_.pwm_right = std::max(last_command_.pwm_right, arc_inner_pwm);
            }
        }
    }
    if (forward_arc_command) {
        last_command_.pwm_left = std::max(last_command_.pwm_left, config_.pwm.min_effective_pwm);
        last_command_.pwm_right = std::max(last_command_.pwm_right, config_.pwm.min_effective_pwm);
    }

    const int pwm_slew_limit =
        commanding_motion
            ? (stall_boost_active_ ? 70 : 42)
            : 110;
    last_command_.pwm_left = slew_limit_pwm(previous_pwm_left, last_command_.pwm_left, pwm_slew_limit);
    last_command_.pwm_right = slew_limit_pwm(previous_pwm_right, last_command_.pwm_right, pwm_slew_limit);
    // Slew limiting used to turn a valid break-away command (for example 60)
    // back into an ineffective 42 PWM first step.  Preserve the sign but
    // re-apply the calibrated motion band after the ramp whenever motion is
    // genuinely requested.
    if (commanding_motion) {
        last_command_.pwm_left = clamp_motion_pwm_band(
            last_command_.pwm_left,
            config_.pwm.min_effective_pwm,
            config_.pwm.max_pwm);
        last_command_.pwm_right = clamp_motion_pwm_band(
            last_command_.pwm_right,
            config_.pwm.min_effective_pwm,
            config_.pwm.max_pwm);
    }
    if (forward_only_tracked_tracks && last_command_.target_speed > 1e-4) {
        last_command_.pwm_left = std::max(last_command_.pwm_left, 0);
        last_command_.pwm_right = std::max(last_command_.pwm_right, 0);
    }
    const bool directional_pwm_car =
        !tracked_vehicle &&
        (world_is_mixed(world_) || compact_unstructured_car);
    if (directional_pwm_car) {
        if (last_command_.target_speed > 1e-4) {
            last_command_.pwm_left = std::max(
                last_command_.pwm_left, config_.pwm.min_effective_pwm);
            last_command_.pwm_right = std::max(
                last_command_.pwm_right, config_.pwm.min_effective_pwm);
        } else if (last_command_.target_speed < -1e-4) {
            last_command_.pwm_left = std::min(
                last_command_.pwm_left, -config_.pwm.min_effective_pwm);
            last_command_.pwm_right = std::min(
                last_command_.pwm_right, -config_.pwm.min_effective_pwm);
        } else {
            // A stop/hold must be immediate.  Slewing two unequal previous PWM
            // values toward zero can otherwise leave one positive and one
            // negative for a cycle, which is still a physical pivot command.
            last_command_.pwm_left = 0;
            last_command_.pwm_right = 0;
        }
    }
    if (forward_arc_command) {
        last_command_.pwm_left = std::max(last_command_.pwm_left, config_.pwm.min_effective_pwm);
        last_command_.pwm_right = std::max(last_command_.pwm_right, config_.pwm.min_effective_pwm);
        if (std::abs(last_command_.target_yaw_rate) > 0.015) {
            const bool gate_tracking_turn =
                control_source_ == HardwareControlSource::GateMpc;
            const int minimum_turn_delta = gate_tracking_turn
                ? std::clamp(
                      static_cast<int>(std::lround(
                          12.0 + 100.0 * std::abs(last_command_.target_yaw_rate))),
                      14,
                      38)
                : std::clamp(
                      static_cast<int>(std::lround(
                          6.0 + 55.0 * std::abs(last_command_.target_yaw_rate))),
                      8,
                      18);
            enforce_forward_differential_pwm(
                last_command_.target_yaw_rate,
                minimum_turn_delta,
                config_.pwm.max_pwm,
                &last_command_.pwm_left,
                &last_command_.pwm_right);
        }
    }
    if (!commanding_motion && std::abs(last_command_.pwm_left) < 4) {
        last_command_.pwm_left = 0;
    }
    if (!commanding_motion && std::abs(last_command_.pwm_right) < 4) {
        last_command_.pwm_right = 0;
    }

    diagnostics_.no_motion_command_cycles = no_motion_command_cycles_;
    diagnostics_.stall_boost_active = stall_boost_active_;
    if (control_source_ == HardwareControlSource::Hold &&
        (use_gap_recovery_turn || allow_unlocked_recovery_motion)) {
        control_source_ = HardwareControlSource::Recovery;
    }
    if (safety_stop_active_) {
        control_source_ = HardwareControlSource::SafetyStop;
    }
}

}  // namespace thesis_sim
