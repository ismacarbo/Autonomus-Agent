void HardwarePlannerRunner::update_estimate_from_observation(const RealRobotObservation& observation, double dt) {
    if (!observation.have_controller_telemetry) {
        return;
    }

    const ControllerTelemetry& telemetry = observation.controller;
    const double imu_yaw = static_cast<double>(telemetry.yaw_mrad) / 1000.0;
    last_raw_imu_yaw_ = imu_yaw;
    have_raw_imu_yaw_ = true;

    if (!yaw_offset_initialized_) {
        yaw_offset_ = wrap_angle(estimator_.state().yaw - imu_yaw);
        yaw_offset_initialized_ = true;
    }

    double measured_yaw = wrap_angle(imu_yaw + yaw_offset_);
    const double measured_yaw_rate_raw = static_cast<double>(telemetry.yaw_rate_mrad_s) / 1000.0;
    // Hardware IMU spikes above the platform's physical yaw-rate envelope are
    // usually transients; keep them from destabilizing the EKF and controller.
    const double measured_yaw_rate = clamp_value(
        measured_yaw_rate_raw,
        -1.5 * config_.drive.max_yaw_rate,
        1.5 * config_.drive.max_yaw_rate);

    if (have_accepted_imu_yaw_ && telemetry.imu_ms != last_accepted_imu_ms_) {
        const double imu_dt =
            telemetry.imu_ms > last_accepted_imu_ms_
                ? clamp_value(static_cast<double>(telemetry.imu_ms - last_accepted_imu_ms_) / 1000.0,
                              0.005,
                              0.25)
                : std::max(dt, 0.005);
        const double yaw_delta = std::abs(wrap_angle(measured_yaw - last_accepted_imu_yaw_));
        const double expected_delta =
            std::abs(measured_yaw_rate) * imu_dt +
            2.5 * config_.drive.max_yaw_rate * imu_dt;
        const double max_plausible_yaw_step = std::max(0.70, expected_delta + 0.20);
        if (yaw_delta > max_plausible_yaw_step) {
            // BNO080 rotation-vector yaw can occasionally jump when magnetic
            // calibration changes. Keep the estimator continuous and absorb
            // the jump into the IMU-to-world offset.
            yaw_offset_ = wrap_angle(last_accepted_imu_yaw_ - imu_yaw);
            measured_yaw = last_accepted_imu_yaw_;
        }
    }
    last_accepted_imu_yaw_ = measured_yaw;
    last_accepted_imu_ms_ = telemetry.imu_ms;
    have_accepted_imu_yaw_ = true;

    const bool mixed_gate_active = world_is_mixed(world_) && locked_gap_goal_.has_value();
    if (world_has_structured_reference(world_) && !mixed_gate_active) {
        update_estimate_from_structured_motion_fallback(telemetry, dt, measured_yaw, measured_yaw_rate);
        return;
    }

    std::int32_t left_delta_ticks = 0;
    std::int32_t right_delta_ticks = 0;
    update_controller_encoder_snapshot(telemetry, &left_delta_ticks, &right_delta_ticks);

    const double encoder_dt =
        telemetry.enc_dt_ms > 0
            ? clamp_value(static_cast<double>(telemetry.enc_dt_ms) / 1000.0, 0.01, 0.25)
            : std::max(dt, 0.01);
    const bool encoder_ready =
        controller_encoder_odometry_usable(telemetry, left_delta_ticks, right_delta_ticks, encoder_dt);

    double odom_speed = 0.0;
    double odom_yaw_rate = 0.0;
    const double half_track = config_.drive.track_width * 0.5;
    if (encoder_ready) {
        const double ticks_to_distance =
            (2.0 * kPi * geometry_.wheel_radius) /
            static_cast<double>(std::max<std::int32_t>(geometry_.encoder_ticks_per_revolution, 1));
        const double left_dist = signed_encoder_distance(
            left_delta_ticks,
            encoder_flag_set(telemetry.enc_flags, EncoderFlag::LeftDirNeg),
            ticks_to_distance);
        const double right_dist = signed_encoder_distance(
            right_delta_ticks,
            encoder_flag_set(telemetry.enc_flags, EncoderFlag::RightDirNeg),
            ticks_to_distance);
        const double odom_delta_yaw =
            std::abs(geometry_.track) > 1e-6 ? (right_dist - left_dist) / geometry_.track : 0.0;
        measured_left_wheel_speed_ = encoder_dt > 1e-6 ? left_dist / encoder_dt : 0.0;
        measured_right_wheel_speed_ = encoder_dt > 1e-6 ? right_dist / encoder_dt : 0.0;
        measured_wheel_speeds_valid_ = true;
        odom_speed = encoder_dt > 1e-6 ? 0.5 * (left_dist + right_dist) / encoder_dt : 0.0;
        odom_yaw_rate = encoder_dt > 1e-6 ? odom_delta_yaw / encoder_dt : 0.0;
        odom_speed = clamp_value(odom_speed, -config_.drive.max_linear_speed, config_.drive.max_linear_speed);
        odom_yaw_rate = clamp_value(odom_yaw_rate, -config_.drive.max_yaw_rate, config_.drive.max_yaw_rate);
    } else {
        const double pwm_speed_estimate = clamp_value(
            std::abs(forward_speed_from_pwm_estimate(telemetry, geometry_)),
            0.0,
            geometry_.max_linear_speed);
        const bool controller_driving =
            std::abs(telemetry.pwm_l) >= geometry_.min_effective_pwm ||
            std::abs(telemetry.pwm_r) >= geometry_.min_effective_pwm ||
            std::abs(telemetry.target_pwm_l) >= geometry_.min_effective_pwm ||
            std::abs(telemetry.target_pwm_r) >= geometry_.min_effective_pwm;
        const bool commanded_linear_motion = std::abs(last_command_.target_speed) > 0.02;

        odom_speed = clamp_value(last_command_.target_speed, 0.0, geometry_.max_linear_speed);
        if (odom_speed <= 1e-4 && controller_driving && commanded_linear_motion) {
            odom_speed = pwm_speed_estimate;
        } else if (!controller_driving) {
            odom_speed = 0.0;
        }

        odom_yaw_rate = clamp_value(
            std::abs(measured_yaw_rate) > 1e-4 ? measured_yaw_rate : last_command_.target_yaw_rate,
            -config_.drive.max_yaw_rate,
            config_.drive.max_yaw_rate);
        const auto [fallback_left_wheel_speed, fallback_right_wheel_speed] =
            wheel_speeds_from_body(odom_speed, odom_yaw_rate, half_track);
        measured_left_wheel_speed_ = fallback_left_wheel_speed;
        measured_right_wheel_speed_ = fallback_right_wheel_speed;
        measured_wheel_speeds_valid_ = false;
    }
    estimator_.predict(encoder_dt, odom_speed, odom_yaw_rate);
    estimator_.update_imu(measured_yaw, measured_yaw_rate);
    sync_estimate_from_ekf_state();
    if (encoder_ready) {
        stabilize_compact_open_gate_odometry(odom_speed, encoder_dt);
    }
}

void HardwarePlannerRunner::update_controller_encoder_snapshot(const ControllerTelemetry& telemetry,
                                                               std::int32_t* left_delta_ticks,
                                                               std::int32_t* right_delta_ticks) {
    if (!encoder_ticks_initialized_) {
        last_left_encoder_ticks_ = telemetry.ticks_left;
        last_right_encoder_ticks_ = telemetry.ticks_right;
        encoder_ticks_initialized_ = true;
    }

    std::int32_t left_delta = telemetry.ticks_left - last_left_encoder_ticks_;
    std::int32_t right_delta = telemetry.ticks_right - last_right_encoder_ticks_;
    last_left_encoder_ticks_ = telemetry.ticks_left;
    last_right_encoder_ticks_ = telemetry.ticks_right;

    const double encoder_dt =
        telemetry.enc_dt_ms > 0U
            ? clamp_value(static_cast<double>(telemetry.enc_dt_ms) / 1000.0, 0.01, 0.25)
            : std::max(config_.nominal_dt, 0.01);
    if (!controller_encoder_delta_plausible(left_delta, right_delta, encoder_dt)) {
        left_delta = 0;
        right_delta = 0;
        encoder_ready_streak_ = 0;
    }

    latest_controller_left_encoder_ticks_ = telemetry.ticks_left;
    latest_controller_right_encoder_ticks_ = telemetry.ticks_right;
    latest_controller_left_encoder_delta_ = left_delta;
    latest_controller_right_encoder_delta_ = right_delta;
    latest_controller_encoder_dt_ms_ = static_cast<double>(telemetry.enc_dt_ms);

    if (left_delta_ticks != nullptr) {
        *left_delta_ticks = left_delta;
    }
    if (right_delta_ticks != nullptr) {
        *right_delta_ticks = right_delta;
    }
}

bool HardwarePlannerRunner::controller_encoder_delta_plausible(std::int32_t left_delta_ticks,
                                                               std::int32_t right_delta_ticks,
                                                               double encoder_dt) const {
    if (encoder_dt <= 1e-6) {
        return false;
    }

    const double ticks_to_distance =
        (2.0 * kPi * geometry_.wheel_radius) /
        static_cast<double>(std::max<std::int32_t>(geometry_.encoder_ticks_per_revolution, 1));
    if (!(ticks_to_distance > 1e-9)) {
        return false;
    }

    const double max_wheel_speed =
        geometry_.max_linear_speed + 0.5 * std::abs(geometry_.track) * geometry_.max_yaw_rate;
    const double allowed_wheel_speed =
        std::max(max_wheel_speed * 2.5, max_wheel_speed + 0.20);
    const double allowed_ticks =
        std::max(120.0, allowed_wheel_speed * encoder_dt / ticks_to_distance);
    return std::abs(static_cast<double>(left_delta_ticks)) <= allowed_ticks &&
           std::abs(static_cast<double>(right_delta_ticks)) <= allowed_ticks;
}

bool HardwarePlannerRunner::controller_encoder_odometry_usable(const ControllerTelemetry& telemetry,
                                                               std::int32_t left_delta_ticks,
                                                               std::int32_t right_delta_ticks,
                                                               double encoder_dt) {
    const bool raw_encoder_ready = config_.use_encoder_odometry && controller_encoders_ready(telemetry);
    const bool overflow_warn = encoder_flag_set(telemetry.enc_flags, EncoderFlag::OverflowWarn);
    if (!raw_encoder_ready || overflow_warn || encoder_dt <= 1e-6) {
        encoder_ready_streak_ = 0;
        return false;
    }

    const double ticks_to_distance =
        (2.0 * kPi * geometry_.wheel_radius) /
        static_cast<double>(std::max<std::int32_t>(geometry_.encoder_ticks_per_revolution, 1));
    const double left_wheel_speed =
        std::abs(static_cast<double>(left_delta_ticks)) * ticks_to_distance / encoder_dt;
    const double right_wheel_speed =
        std::abs(static_cast<double>(right_delta_ticks)) * ticks_to_distance / encoder_dt;
    const double max_wheel_speed =
        geometry_.max_linear_speed + 0.5 * std::abs(geometry_.track) * geometry_.max_yaw_rate;
    const double allowed_wheel_speed =
        std::max(max_wheel_speed * kEncoderWheelSpeedMargin, max_wheel_speed + 0.08);
    if (left_wheel_speed > allowed_wheel_speed || right_wheel_speed > allowed_wheel_speed) {
        encoder_ready_streak_ = 0;
        return false;
    }

    ++encoder_ready_streak_;
    return encoder_ready_streak_ >= kStableEncoderReadyFrames;
}

bool HardwarePlannerRunner::rearm_controller_if_needed(const ControllerTelemetry& telemetry) {
    if (!config_.auto_set_autonomous_mode ||
        !connected_ ||
        !bridge_.controller_connected() ||
        !telemetry.have_motor) {
        return false;
    }

    const bool command_active =
        std::abs(last_command_.pwm_left) >= 4 ||
        std::abs(last_command_.pwm_right) >= 4 ||
        std::abs(last_command_.target_speed) > 1e-4 ||
        std::abs(last_command_.target_yaw_rate) > 1e-4;
    if (!command_active) {
        return false;
    }

    const bool fault_latched =
        (telemetry.status_flags & static_cast<std::uint16_t>(StatusFlag::FaultLatched)) != 0U ||
        (telemetry.safety_flags & static_cast<std::uint16_t>(SafetyFlag::EmergencyStop)) != 0U;
    if (fault_latched) {
        return false;
    }

    const bool motor_enabled =
        (telemetry.motor_flags & static_cast<std::uint16_t>(MotorFlag::Enabled)) != 0U;
    const bool command_timeout =
        (telemetry.motor_flags & static_cast<std::uint16_t>(MotorFlag::CmdTimeout)) != 0U ||
        (telemetry.safety_flags & static_cast<std::uint16_t>(SafetyFlag::CmdTimeout)) != 0U;
    if (motor_enabled && !command_timeout) {
        return false;
    }

    const double now_s = monotonic_seconds();
    if (last_controller_rearm_time_s_ > 0.0 &&
        (now_s - last_controller_rearm_time_s_) < 0.35) {
        return false;
    }
    last_controller_rearm_time_s_ = now_s;

    try {
        bridge_.set_mode(ControllerMode::Autonomous, 0.25);
        try {
            bridge_.config_set(
                static_cast<std::uint8_t>(ConfigParamId::CmdTimeoutMs),
                ConfigValueType::Uint16,
                desired_controller_cmd_timeout_ms(config_.nominal_dt),
                0.20);
        } catch (const std::exception&) {
        }
        bridge_.send_pwm(
            static_cast<std::int16_t>(last_command_.pwm_left),
            static_cast<std::int16_t>(last_command_.pwm_right),
            true,
            MotorControlMode::SafeDirectPwm);
        return true;
    } catch (const std::exception& e) {
        std::cerr << "hardware_runner_warning=controller autonomous rearm failed: "
                  << e.what() << '\n';
        return false;
    }
}

void HardwarePlannerRunner::update_estimate_from_structured_motion_fallback(const ControllerTelemetry& telemetry,
                                                                            double dt,
                                                                            double measured_yaw,
                                                                            double measured_yaw_rate) {
    std::int32_t left_delta_ticks = 0;
    std::int32_t right_delta_ticks = 0;
    update_controller_encoder_snapshot(telemetry, &left_delta_ticks, &right_delta_ticks);

    const double effective_dt =
        telemetry.enc_dt_ms > 0U
            ? clamp_value(static_cast<double>(telemetry.enc_dt_ms) / 1000.0, 0.01, 0.25)
            : std::max(dt, 0.01);
    const bool encoder_ready =
        controller_encoder_odometry_usable(telemetry, left_delta_ticks, right_delta_ticks, effective_dt);

    double odom_speed = 0.0;
    double odom_yaw_rate = 0.0;
    const double half_track = config_.drive.track_width * 0.5;
    if (encoder_ready) {
        const double ticks_to_distance =
            (2.0 * kPi * geometry_.wheel_radius) /
            static_cast<double>(std::max<std::int32_t>(geometry_.encoder_ticks_per_revolution, 1));
        const double left_dist = signed_encoder_distance(
            left_delta_ticks,
            encoder_flag_set(telemetry.enc_flags, EncoderFlag::LeftDirNeg),
            ticks_to_distance);
        const double right_dist = signed_encoder_distance(
            right_delta_ticks,
            encoder_flag_set(telemetry.enc_flags, EncoderFlag::RightDirNeg),
            ticks_to_distance);
        const double odom_delta_yaw =
            std::abs(geometry_.track) > 1e-6 ? (right_dist - left_dist) / geometry_.track : 0.0;
        measured_left_wheel_speed_ = effective_dt > 1e-6 ? left_dist / effective_dt : 0.0;
        measured_right_wheel_speed_ = effective_dt > 1e-6 ? right_dist / effective_dt : 0.0;
        measured_wheel_speeds_valid_ = true;
        odom_speed = effective_dt > 1e-6 ? 0.5 * (left_dist + right_dist) / effective_dt : 0.0;
        odom_yaw_rate = effective_dt > 1e-6 ? odom_delta_yaw / effective_dt : 0.0;
        odom_speed = clamp_value(odom_speed, -config_.drive.max_linear_speed, config_.drive.max_linear_speed);
        odom_yaw_rate = clamp_value(odom_yaw_rate, -config_.drive.max_yaw_rate, config_.drive.max_yaw_rate);
    } else {
        const double pwm_speed_estimate = clamp_value(
            std::abs(forward_speed_from_pwm_estimate(telemetry, geometry_)),
            0.0,
            geometry_.max_linear_speed);
        const bool controller_driving =
            std::abs(telemetry.pwm_l) >= geometry_.min_effective_pwm ||
            std::abs(telemetry.pwm_r) >= geometry_.min_effective_pwm ||
            std::abs(telemetry.target_pwm_l) >= geometry_.min_effective_pwm ||
            std::abs(telemetry.target_pwm_r) >= geometry_.min_effective_pwm;
        const bool commanded_linear_motion = std::abs(last_command_.target_speed) > 0.02;

        odom_speed = clamp_value(last_command_.target_speed, 0.0, geometry_.max_linear_speed);
        if (odom_speed <= 1e-4 && controller_driving && commanded_linear_motion) {
            odom_speed = pwm_speed_estimate;
        } else if (!controller_driving) {
            odom_speed = 0.0;
        }

        odom_yaw_rate = clamp_value(
            std::abs(measured_yaw_rate) > 1e-4 ? measured_yaw_rate : last_command_.target_yaw_rate,
            -config_.drive.max_yaw_rate,
            config_.drive.max_yaw_rate);
        const auto [fallback_left_wheel_speed, fallback_right_wheel_speed] =
            wheel_speeds_from_body(odom_speed, odom_yaw_rate, half_track);
        measured_left_wheel_speed_ = fallback_left_wheel_speed;
        measured_right_wheel_speed_ = fallback_right_wheel_speed;
        measured_wheel_speeds_valid_ = false;
    }

    estimator_.predict(effective_dt, odom_speed, odom_yaw_rate);
    estimator_.update_imu(measured_yaw, measured_yaw_rate);
    sync_estimate_from_ekf_state();

    if (road_ == nullptr ||
        cl_.prev_road.num_segments() <= 0 ||
        !std::isfinite(cl_.end_point_s) ||
        cl_.end_point_s <= 1e-6) {
        const bool mixed_gate_active = world_is_mixed(world_) && locked_gap_goal_.has_value();
        const bool closed_reference = structured_road_is_closed_loop(world_);
        const double fallback_road_length = structured_road_length_m(world_);
        if (world_is_mixed(world_) &&
            !mixed_gate_active &&
            closed_reference &&
            fallback_road_length > 1e-6) {
            const double progress_delta = std::max(odom_speed, 0.0) * effective_dt;
            const double base_s =
                std::isfinite(structured_last_s_)
                    ? structured_last_s_
                    : (std::isfinite(x0_.x) ? x0_.x : structured_progress_s_);
            structured_last_s_ = wrap_arc_length(base_s + progress_delta, fallback_road_length);
            structured_progress_s_ = std::max(0.0, structured_progress_s_ + progress_delta);
        }
        x0_.x = std::isfinite(structured_last_s_) ? structured_last_s_ : 0.0;
        x0_.n = 0.0;
        x0_.b = 0.0;
        return;
    }

    const bool closed_structured_loop = structured_road_is_closed_loop(world_);
    const bool prefer_hinted_loop_projection =
        world_is_mixed(world_) &&
        closed_structured_loop &&
        world_span_m(world_) <= 2.50;
    const double s_hint = std::isfinite(structured_last_s_)
                              ? structured_last_s_
                              : (std::isfinite(x0_.x) ? x0_.x : 0.0);
    double projected_s = s_hint;
    double projected_n = 0.0;
    if (!project_curvilinear_state(
            cl_,
            estimate_.position,
            s_hint,
            closed_structured_loop,
            prefer_hinted_loop_projection,
            &projected_s,
            &projected_n)) {
        x0_.x = s_hint;
        x0_.n = 0.0;
        x0_.b = 0.0;
        structured_progress_s_ = std::max(0.0, structured_progress_s_ + odom_speed * effective_dt);
        return;
    }

    const double max_forward_step =
        std::max(odom_speed * effective_dt + 0.25, closed_structured_loop ? 0.08 : 0.05);
    double progress_delta = 0.0;
    const double constrained_s =
        stabilize_structured_track_s(projected_s, max_forward_step, closed_structured_loop, &progress_delta);
    const bool tracked_structured_loop =
        config_.vehicle_model == VehicleModelKind::TrackedVehicle && closed_structured_loop;
    const double odom_progress_delta = clamp_value(
        std::max(odom_speed, 0.0) * effective_dt,
        0.0,
        max_forward_step);
    double fused_progress_delta = progress_delta;
    double fused_constrained_s = constrained_s;
    if (tracked_structured_loop && odom_progress_delta > 1e-4) {
        const double odom_floor_delta = 0.65 * odom_progress_delta;
        if (fused_progress_delta < odom_floor_delta) {
            const double previous_s =
                std::isfinite(structured_last_s_)
                    ? wrap_arc_length(structured_last_s_, cl_.end_point_s)
                    : wrap_arc_length(constrained_s, cl_.end_point_s);
            fused_progress_delta = std::min(odom_floor_delta, max_forward_step);
            fused_constrained_s = wrap_arc_length(previous_s + fused_progress_delta, cl_.end_point_s);
        }
    }

    // A road projection is a useful weak constraint in structured-only runs,
    // but it is not a pose measurement. Applying it in mixed mode pulled the
    // EKF back onto the centerline as soon as a temporary gate was released,
    // hiding the physical bypass seen by encoder/IMU odometry.
    const bool allow_centerline_pose_constraint = !world_is_mixed(world_);
    if (allow_centerline_pose_constraint) {
        double x_on_path = 0.0;
        double y_on_path = 0.0;
        cl_.prev_road.eval(fused_constrained_s, x_on_path, y_on_path);
        estimator_.update_lidar_pose({x_on_path, y_on_path}, measured_yaw, false);
        sync_estimate_from_ekf_state();
    }

    double constrained_n = 0.0;
    double confirmed_s = fused_constrained_s;
    if (project_curvilinear_state(
            cl_,
            estimate_.position,
            fused_constrained_s,
            closed_structured_loop,
            prefer_hinted_loop_projection,
            &confirmed_s,
            &constrained_n)) {
        x0_.n = constrained_n;
    } else {
        x0_.n = 0.0;
    }
    estimate_.localized = true;

    structured_last_s_ = fused_constrained_s;
    structured_progress_s_ = std::max(0.0, structured_progress_s_ + fused_progress_delta);
    x0_.x = fused_constrained_s;
    x0_.b = 0.0;
}

double HardwarePlannerRunner::stabilize_structured_track_s(double candidate_s,
                                                           double max_forward_step,
                                                           bool closed_loop,
                                                           double* progress_delta) const {
    const double road_length = std::max(cl_.end_point_s, 1e-6);
    const double safe_step = std::max(max_forward_step, 0.02);
    if (!std::isfinite(structured_last_s_)) {
        if (progress_delta != nullptr) {
            *progress_delta = 0.0;
        }
        return closed_loop ? wrap_arc_length(candidate_s, road_length)
                           : clamp_value(candidate_s, 0.0, road_length);
    }

    const double previous_s =
        closed_loop ? wrap_arc_length(structured_last_s_, road_length)
                    : clamp_value(structured_last_s_, 0.0, road_length);
    double delta_s = candidate_s - previous_s;
    if (closed_loop) {
        if (delta_s > 0.5 * road_length) {
            delta_s -= road_length;
        } else if (delta_s < -0.5 * road_length) {
            delta_s += road_length;
        }
    }

    if (delta_s < -0.15) {
        delta_s = 0.0;
    }
    delta_s = clamp_value(delta_s, 0.0, safe_step);
    if (progress_delta != nullptr) {
        *progress_delta = delta_s;
    }

    const double stabilized_s = previous_s + delta_s;
    return closed_loop ? wrap_arc_length(stabilized_s, road_length)
                       : clamp_value(stabilized_s, 0.0, road_length);
}

void HardwarePlannerRunner::stabilize_compact_open_gate_odometry(double odom_speed, double dt) {
    if (!compact_mixed_open_road(world_) ||
        !locked_gap_goal_.has_value() ||
        !(odom_speed > 1e-5) ||
        !(dt > 1e-6) ||
        cl_.prev_road.num_segments() <= 0 ||
        !std::isfinite(cl_.end_point_s) ||
        cl_.end_point_s <= 1e-6) {
        return;
    }

    const double road_length = cl_.end_point_s;
    const double progress_scale = 0.86;
    const double progress_delta = clamp_value(progress_scale * odom_speed * dt, 0.0, 0.08);
    if (!(progress_delta > 1e-6)) {
        return;
    }

    const double previous_progress = clamp_value(
        std::max(structured_progress_s_,
                 std::isfinite(structured_last_s_) ? structured_last_s_ : 0.0),
        0.0,
        road_length);
    structured_progress_s_ = clamp_value(previous_progress + progress_delta, 0.0, road_length);

    const double s_hint =
        std::isfinite(structured_last_s_) ? structured_last_s_ : structured_progress_s_;
    double projected_s = structured_progress_s_;
    double projected_n = 0.0;
    const bool have_projection = project_curvilinear_state(
        cl_,
        estimate_.position,
        s_hint,
        false,
        false,
        &projected_s,
        &projected_n);
    if (!have_projection) {
        projected_s = structured_progress_s_;
        projected_n = 0.0;
    }

    const double odom_floor_s = structured_progress_s_;
    const double fused_s = clamp_value(std::max(projected_s, odom_floor_s), 0.0, road_length);
    const bool rejoin_phase =
        passed_unstructured_gap_count_ > 0 &&
        (passed_unstructured_gap_count_ % 2) == 1;
    double fused_n = projected_n;
    if (rejoin_phase) {
        const double rejoin_band =
            odom_floor_s >= 0.70 * road_length ? 0.035 : 0.065;
        fused_n = clamp_value(fused_n, -rejoin_band, rejoin_band);
    } else {
        fused_n = clamp_value(fused_n, -0.22, 0.22);
    }

    structured_last_s_ = fused_s;
    x0_.x = fused_s;
    x0_.n = fused_n;
    x0_.b = 0.0;
    // Keep this gate helper progress-only. During a gate bypass the robot may
    // legitimately leave the structured centerline; overriding the EKF pose here
    // creates artificial jumps in hardware exactly when the first gate is locked.
}

void HardwarePlannerRunner::sync_estimate_from_ekf_state() {
    const EkfState& state = estimator_.state();
    estimate_.position = state.position;
    estimate_.yaw = state.yaw;
    estimate_.speed = state.speed;
    estimate_.accel = state.accel;
    estimate_.yaw_rate = state.yaw_rate;
    estimate_.curvature = std::abs(state.speed) > 0.05
                              ? clamp_value(
                                    state.yaw_rate / state.speed,
                                    -geometry_.max_curvature,
                                    geometry_.max_curvature)
                              : 0.0;
    estimate_.localized = true;
}

void HardwarePlannerRunner::correct_pose_with_lidar(const std::vector<RPLidarA1::ScanPoint>& scan) {
    if (static_cast<int>(scan.size()) < config_.localization.min_scan_points) {
        return;
    }
    if (!lidar_pose_correction_enabled_for_current_mode()) {
        return;
    }

    const bool unstructured_perception_mode = unstructured_perception_only_mode();
    const bool compact_mixed_open_world = compact_mixed_open_road(world_);
    const double motion_speed =
        std::max(std::abs(estimate_.speed), std::abs(last_command_.target_speed));
    const double motion_yaw_rate =
        std::max(std::abs(estimate_.yaw_rate), std::abs(last_command_.target_yaw_rate));

    if (unstructured_perception_mode) {
        // In unstructured mode the "map" is a local LiDAR accumulation built from the
        // robot's own estimated frame. Allowing broad scan-matching updates here can
        // make the estimate slide or jump during near in-place turns. Keep LiDAR as a
        // perception source for gap extraction, and only accept small translation
        // corrections while the robot is actually making forward progress.
        if (motion_speed < 0.05 || motion_yaw_rate > 0.45) {
            return;
        }
    }
    if (compact_mixed_open_world) {
        if (sim_time_ < 0.65) {
            return;
        }
        if (motion_speed < 0.018 && motion_yaw_rate > 0.45) {
            return;
        }
    }

    const double base_score = score_candidate_pose(estimate_.position, estimate_.yaw, scan);
    if (!std::isfinite(base_score)) {
        return;
    }
    Vec2 best_position = estimate_.position;
    double best_yaw = estimate_.yaw;
    double best_score = base_score;

    const std::array<LidarSearchWindow, 2> search_windows{{
        {compact_mixed_open_world ? std::min(config_.localization.xy_search_window_m, 0.06)
                                  : config_.localization.xy_search_window_m,
         compact_mixed_open_world ? std::min(config_.localization.xy_search_step_m, 0.03)
                                  : config_.localization.xy_search_step_m,
         compact_mixed_open_world ? std::min(config_.localization.yaw_search_window_rad, 0.045)
                                  : config_.localization.yaw_search_window_rad,
         compact_mixed_open_world ? std::min(config_.localization.yaw_search_step_rad, 0.018)
                                  : config_.localization.yaw_search_step_rad},
        {compact_mixed_open_world ? 0.025 : std::max(config_.localization.xy_search_step_m, 0.08),
         compact_mixed_open_world ? 0.0125 : std::max(config_.localization.xy_search_step_m * 0.5, 0.04),
         compact_mixed_open_world ? 0.020 : std::max(config_.localization.yaw_search_step_rad, 0.03),
         compact_mixed_open_world ? 0.010 : std::max(config_.localization.yaw_search_step_rad * 0.5, 0.015)},
    }};

    for (const LidarSearchWindow& window : search_windows) {
        const Vec2 center = best_position;
        const double center_yaw = best_yaw;
        for (double dx = -window.xy_window; dx <= window.xy_window + 1e-9; dx += window.xy_step) {
            for (double dy = -window.xy_window; dy <= window.xy_window + 1e-9; dy += window.xy_step) {
                for (double dyaw = -window.yaw_window; dyaw <= window.yaw_window + 1e-9; dyaw += window.yaw_step) {
                    const Vec2 candidate{center.x + dx, center.y + dy};
                    const double candidate_yaw = wrap_angle(center_yaw + dyaw);
                    const double score = score_candidate_pose(candidate, candidate_yaw, scan);
                    if (score < best_score) {
                        best_score = score;
                        best_position = candidate;
                        best_yaw = candidate_yaw;
                    }
                }
            }
        }
    }

    if (std::isfinite(best_score)) {
        if (compact_mixed_open_world &&
            base_score - best_score < 0.012) {
            return;
        }
        Vec2 corrected_position = best_position;
        if (compact_mixed_open_world) {
            Vec2 road_tangent{1.0, 0.0};
            double best_segment_distance_sq = std::numeric_limits<double>::infinity();
            const std::vector<Vec2>& road = world_.road_centerline();
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
                const double d_sq = distance_sq(estimate_.position, projection);
                if (d_sq < best_segment_distance_sq) {
                    const double segment_len = std::sqrt(segment_len_sq);
                    road_tangent = {segment.x / segment_len, segment.y / segment_len};
                    best_segment_distance_sq = d_sq;
                }
            }
            const Vec2 road_normal{-road_tangent.y, road_tangent.x};
            const Vec2 correction_delta{
                best_position.x - estimate_.position.x,
                best_position.y - estimate_.position.y,
            };
            const double dt_hint = std::max(config_.nominal_dt, 0.01);
            const double max_longitudinal =
                clamp_value(0.004 + 0.55 * motion_speed * dt_hint, 0.004, 0.018);
            const double max_lateral =
                locked_gap_goal_.has_value()
                    ? clamp_value(0.008 + 0.40 * motion_speed * dt_hint, 0.008, 0.018)
                    : clamp_value(0.006 + 0.30 * motion_speed * dt_hint, 0.006, 0.014);
            const double longitudinal =
                clamp_value(dot_product(correction_delta, road_tangent),
                            -max_longitudinal,
                            max_longitudinal);
            const double lateral =
                clamp_value(dot_product(correction_delta, road_normal),
                            -max_lateral,
                            max_lateral);
            corrected_position = {
                estimate_.position.x + longitudinal * road_tangent.x + lateral * road_normal.x,
                estimate_.position.y + longitudinal * road_tangent.y + lateral * road_normal.y,
            };
            corrected_position = clamp_dynamic_gap_point(corrected_position);
        } else if (unstructured_perception_mode) {
            const Vec2 correction_delta{
                best_position.x - estimate_.position.x,
                best_position.y - estimate_.position.y,
            };
            const double correction_norm = vector_norm(correction_delta);
            double max_correction = clamp_value(0.03 + 0.45 * motion_speed, 0.03, 0.08);
            if (motion_yaw_rate > 0.25) {
                max_correction = std::min(max_correction, 0.04);
            }
            if (correction_norm > max_correction && correction_norm > 1e-6) {
                const double scale = max_correction / correction_norm;
                corrected_position = {
                    estimate_.position.x + correction_delta.x * scale,
                    estimate_.position.y + correction_delta.y * scale,
                };
            }
            corrected_position = clamp_dynamic_gap_point(corrected_position);
        }

        estimator_.update_lidar_pose(corrected_position, best_yaw, !yaw_offset_initialized_);
        sync_estimate_from_ekf_state();
        if (unstructured_perception_mode && !dynamic_gap_point_allowed(estimate_.position)) {
            estimator_.update_lidar_pose(clamp_dynamic_gap_point(estimate_.position),
                                         estimate_.yaw,
                                         false);
            sync_estimate_from_ekf_state();
        }
        if (have_raw_imu_yaw_) {
            yaw_offset_ = wrap_angle(estimate_.yaw - last_raw_imu_yaw_);
            yaw_offset_initialized_ = true;
        }
    }
}

double HardwarePlannerRunner::score_candidate_pose(const Vec2& position,
                                                   double yaw,
                                                   const std::vector<RPLidarA1::ScanPoint>& scan) const {
    if (unstructured_perception_only_mode()) {
        return score_candidate_pose_against_perception_map(position, yaw, scan);
    }

    if (!dynamic_gap_point_allowed(position)) {
        return std::numeric_limits<double>::infinity();
    }

    const Vec2 origin = lidar_origin_world(position, yaw, config_.localization);
    if (!dynamic_gap_point_allowed(origin)) {
        return std::numeric_limits<double>::infinity();
    }

    double score = 0.0;
    int used = 0;
    const int downsample = std::max(config_.localization.scan_downsample, 1);

    for (size_t i = 0; i < scan.size(); i += static_cast<size_t>(downsample)) {
        const RPLidarA1::ScanPoint& point = scan[i];
        if (point.distance_m <= 0.0 ||
            point.distance_m < config_.localization.min_valid_range_m ||
            point.distance_m > config_.localization.max_range_m ||
            scan_point_is_self_hit(point, position, yaw)) {
            continue;
        }

        const double beam_heading =
            wrap_angle(yaw + lidar_body_local_angle_rad(config_.localization, point.angle_deg));
        const std::vector<LidarHit> expected = world_.raycast(
            origin,
            beam_heading,
            1,
            0.0,
            config_.localization.max_range_m);

        const double predicted_range = expected.empty()
                                           ? config_.localization.max_range_m
                                           : expected.front().distance;
        const double range_error = std::min(
            std::abs(predicted_range - point.distance_m),
            config_.localization.max_range_m);
        const double weight = 1.0 / (0.25 + point.distance_m);
        score += range_error * weight;
        ++used;
    }

    if (used < 6) {
        return std::numeric_limits<double>::infinity();
    }

    return score / static_cast<double>(used);
}

double HardwarePlannerRunner::score_candidate_pose_against_perception_map(
    const Vec2& position,
    double yaw,
    const std::vector<RPLidarA1::ScanPoint>& scan) const {
    if (!perception_map_ready()) {
        return std::numeric_limits<double>::infinity();
    }

    if (!dynamic_gap_point_allowed(position)) {
        return std::numeric_limits<double>::infinity();
    }

    const Vec2 origin = lidar_origin_world(position, yaw, config_.localization);
    if (!dynamic_gap_point_allowed(origin)) {
        return std::numeric_limits<double>::infinity();
    }
    const double local_radius = config_.localization.max_range_m + 0.40;
    const double local_radius_sq = local_radius * local_radius;

    std::vector<Vec2> local_map_points;
    local_map_points.reserve(lidar_map_points_.size());
    for (const Vec2& point : lidar_map_points_) {
        if (distance_sq(point, origin) <= local_radius_sq) {
            local_map_points.push_back(point);
        }
    }
    if (local_map_points.size() < 18) {
        return std::numeric_limits<double>::infinity();
    }

    double score = 0.0;
    int used = 0;
    int matched = 0;
    const int downsample = std::max(config_.localization.scan_downsample, 1);
    const double max_match_distance = 0.35;
    const double max_match_distance_sq = max_match_distance * max_match_distance;

    for (size_t i = 0; i < scan.size(); i += static_cast<size_t>(downsample)) {
        const RPLidarA1::ScanPoint& point = scan[i];
        if (point.distance_m <= 0.0 ||
            point.distance_m < config_.localization.min_valid_range_m ||
            point.distance_m > config_.localization.max_range_m ||
            scan_point_is_self_hit(point, position, yaw)) {
            continue;
        }

        const double beam_heading =
            wrap_angle(yaw + lidar_body_local_angle_rad(config_.localization, point.angle_deg));
        const Vec2 hit{
            origin.x + std::cos(beam_heading) * point.distance_m,
            origin.y + std::sin(beam_heading) * point.distance_m,
        };

        double best_distance_sq = std::numeric_limits<double>::infinity();
        for (const Vec2& map_point : local_map_points) {
            const double candidate_distance_sq = distance_sq(hit, map_point);
            if (candidate_distance_sq < best_distance_sq) {
                best_distance_sq = candidate_distance_sq;
            }
        }

        const double weight = 1.0 / (0.20 + point.distance_m);
        if (best_distance_sq <= max_match_distance_sq) {
            score += std::sqrt(best_distance_sq) * weight;
            ++matched;
        } else {
            score += max_match_distance * 1.6 * weight;
        }
        ++used;
    }

    if (used < 6 || matched < std::max(6, used / 4)) {
        return std::numeric_limits<double>::infinity();
    }

    return score / static_cast<double>(used);
}

void HardwarePlannerRunner::update_lidar_hits_world(const std::vector<RPLidarA1::ScanPoint>& scan) {
    lidar_hits_.clear();
    lidar_hits_.reserve(scan.size());
    lidar_map_points_.clear();
    lidar_map_keys_.clear();

    diagnostics_.valid_lidar_points = 0;
    diagnostics_.close_lidar_points = 0;
    diagnostics_.front_close_lidar_points = 0;
    diagnostics_.lidar_front_blocked = false;

    const Vec2 origin = lidar_origin_world(estimate_.position, estimate_.yaw, config_.localization);
    const double map_resolution = std::max(config_.gap_extraction.map_point_resolution_m, 1e-3);
    const int confirm_hits = std::max(config_.gap_extraction.occupancy_confirm_hits, 1);
    const int decay_steps = std::max(config_.gap_extraction.occupancy_decay_steps, 1);
    const int decay_stride = std::max(decay_steps / confirm_hits, 1);
    const int max_cells = std::max(config_.gap_extraction.max_persistent_points, 1);
    std::unordered_set<std::uint64_t> scan_keys;
    std::unordered_map<std::uint64_t, int> free_space_clear_votes;
    scan_keys.reserve(scan.size());
    free_space_clear_votes.reserve(scan.size() * 8);

    for (const RPLidarA1::ScanPoint& point : scan) {
        if (point.distance_m <= 0.0 ||
            point.distance_m < config_.localization.min_valid_range_m ||
            point.distance_m > config_.localization.max_range_m ||
            scan_point_is_self_hit(point)) {
            continue;
        }
        ++diagnostics_.valid_lidar_points;
        const double angle_local = lidar_body_local_angle_rad(config_.localization, point.angle_deg);
        const double angle_world = wrap_angle(estimate_.yaw + angle_local);
        if (point.distance_m < config_.localization.obstacle_stop_distance_m) {
            ++diagnostics_.close_lidar_points;
            if (std::abs(angle_local) <= config_.localization.front_sector_half_angle_rad) {
                ++diagnostics_.front_close_lidar_points;
            }
        }
        const Vec2 hit{
            origin.x + std::cos(angle_world) * point.distance_m,
            origin.y + std::sin(angle_world) * point.distance_m,
        };
        lidar_hits_.push_back({angle_world, point.distance_m, hit, true});

        const bool obstacle_endpoint =
            point.distance_m < config_.localization.max_range_m - 0.03;
        const double free_distance = obstacle_endpoint
            ? std::max(config_.localization.min_valid_range_m, point.distance_m - map_resolution * 1.5)
            : point.distance_m;
        const Vec2 free_endpoint{
            origin.x + std::cos(angle_world) * free_distance,
            origin.y + std::sin(angle_world) * free_distance,
        };
        sample_segment_points(
            origin,
            free_endpoint,
            map_resolution * 0.85,
            false,
            [&](const Vec2& sample) {
                const std::uint64_t free_key = quantized_point_key(sample, map_resolution);
                ++free_space_clear_votes[free_key];
            });

        const std::uint64_t key = quantized_point_key(hit, map_resolution);
        if (!scan_keys.insert(key).second) {
            continue;
        }

        auto occupancy_it = lidar_occupancy_cells_.find(key);
        if (occupancy_it == lidar_occupancy_cells_.end()) {
            if (static_cast<int>(lidar_occupancy_cells_.size()) >= max_cells) {
                continue;
            }
            PerceptionOccupancyCell cell{};
            cell.center = quantized_point_center(hit, map_resolution);
            cell.hit_count = 1;
            cell.last_seen_step = step_count_;
            lidar_occupancy_cells_.emplace(key, cell);
        } else {
            const int age_steps = std::max(0, step_count_ - occupancy_it->second.last_seen_step);
            const int effective_hits =
                std::max(0, occupancy_it->second.hit_count - age_steps / decay_stride);
            occupancy_it->second.center = quantized_point_center(hit, map_resolution);
            occupancy_it->second.hit_count = std::min(effective_hits + 1, confirm_hits + 6);
            occupancy_it->second.last_seen_step = step_count_;
        }
    }

    for (const auto& [key, clear_votes] : free_space_clear_votes) {
        if (scan_keys.find(key) != scan_keys.end()) {
            continue;
        }
        auto occupancy_it = lidar_occupancy_cells_.find(key);
        if (occupancy_it == lidar_occupancy_cells_.end()) {
            continue;
        }

        occupancy_it->second.hit_count = std::max(0, occupancy_it->second.hit_count - clear_votes);
        occupancy_it->second.last_seen_step = step_count_;
    }

    for (auto it = lidar_occupancy_cells_.begin(); it != lidar_occupancy_cells_.end();) {
        const int age_steps = std::max(0, step_count_ - it->second.last_seen_step);
        const int effective_hits = std::max(0, it->second.hit_count - age_steps / decay_stride);
        if (age_steps > decay_steps || effective_hits <= 0) {
            it = lidar_occupancy_cells_.erase(it);
            continue;
        }

        if (effective_hits >= confirm_hits) {
            lidar_map_keys_.insert(it->first);
            lidar_map_points_.push_back(it->second.center);
        }
        ++it;
    }

    estimate_.min_lidar_distance = compute_min_lidar_distance(scan);
    estimate_.front_lidar_distance = compute_front_lidar_distance(scan);
    diagnostics_.accumulated_lidar_points = static_cast<int>(lidar_map_points_.size());
    diagnostics_.lidar_front_blocked =
        estimate_.front_lidar_distance > 0.0 &&
        estimate_.front_lidar_distance < config_.localization.obstacle_stop_distance_m;
}

