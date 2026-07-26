bool stream_enabled(const AppOptions& options) {
    return !options.stream_host.empty() && options.stream_port > 0;
}

bool setup_external_pose_receiver(const AppOptions& options,
                                  ExternalPoseUdpReceiver* receiver) {
    if (options.external_pose_udp_port <= 0) {
        return true;
    }
    if (receiver == nullptr ||
        !receiver->open(
            static_cast<std::uint16_t>(options.external_pose_udp_port),
            options.external_pose_transform)) {
        std::cerr << "mocap_receiver_error="
                  << (receiver != nullptr ? receiver->last_error() : "receiver unavailable")
                  << '\n';
        return false;
    }
    std::cout << "mocap_receiver_status=listening_udp:"
              << receiver->port()
              << " reference_only=1 controller_feedback=0\n";
    return true;
}

void poll_external_pose(const AppOptions& options,
                        ExternalPoseUdpReceiver* receiver,
                        HardwarePlannerRunner* runner) {
    if (receiver == nullptr || runner == nullptr || !receiver->active()) {
        return;
    }
    ExternalPoseSample sample;
    if (!receiver->poll(&sample)) {
        return;
    }
    ExternalPoseReference reference;
    reference.valid = sample.valid;
    reference.source_timestamp_s = sample.source_timestamp_s;
    reference.position = {sample.position_x_m, sample.position_y_m};
    reference.yaw = sample.yaw_rad;
    reference.quality = sample.quality;
    runner->set_external_pose_reference(reference, options.external_pose_timeout_s);
}

void apply_vehicle_profile(const AppOptions& options, HardwarePlannerConfig* config) {
    if (config == nullptr) {
        return;
    }

    config->vehicle_model = vehicle_model_from_profile(options.vehicle_profile);

    if (options.vehicle_profile == VehicleProfile::Tank) {
        // Conservative tracked/skid-steer defaults. The tank encoders produce
        // far more ticks per wheel revolution than the small car profile; using
        // the car default makes odometry jump and collapses structured tracking
        // into an in-place yaw correction.
        config->drive.wheelbase = 0.18;
        config->drive.cg_to_front = 0.09;
        config->drive.cg_to_rear = 0.09;
        config->drive.track_width = 0.13;
        config->drive.body_length = 0.25;
        config->drive.body_width = 0.15;
        config->drive.wheel_radius = 0.032;
        config->drive.encoder_ticks_per_revolution = 2400;
        config->drive.max_linear_speed = 0.22;
        config->drive.max_yaw_rate = 1.80;
        config->drive.max_curvature = 3.60;
        config->pwm.start_motion_pwm = 138;
        config->pwm.min_effective_pwm = 48;
        config->pwm.linear_feedback_gain = 42.0;
        config->pwm.yaw_feedback_gain = 18.0;
        config->pwm.stall_boost_after_cycles = 2;
        config->pwm.stall_target_speed_threshold_mps = 0.010;
        config->cruise_speed_limit = std::min(config->cruise_speed_limit, 0.09);
    }

    if (options.encoder_ticks_per_revolution > 0) {
        config->drive.encoder_ticks_per_revolution = options.encoder_ticks_per_revolution;
    }
    if (options.wheel_radius_m > 0.0) {
        config->drive.wheel_radius = options.wheel_radius_m;
    }
    if (options.track_width_m > 0.0) {
        config->drive.track_width = options.track_width_m;
    }
    if (options.max_linear_speed_mps > 0.0) {
        config->drive.max_linear_speed = options.max_linear_speed_mps;
    }
    if (options.max_yaw_rate_rad_s > 0.0) {
        config->drive.max_yaw_rate = options.max_yaw_rate_rad_s;
    }
    if (options.cruise_speed_limit_mps > 0.0) {
        config->cruise_speed_limit = options.cruise_speed_limit_mps;
    }
}

bool setup_stream_client(const AppOptions& options,
                         LiveViewStreamClient* streamer) {
    if (!stream_enabled(options) || streamer == nullptr) {
        return true;
    }

    if (!streamer->connect_to(options.stream_host, static_cast<std::uint16_t>(options.stream_port))) {
        std::cerr << "live_stream_warning=" << streamer->last_error()
                  << " (continuing without GUI stream; will retry automatically)\n";
        return true;
    }
    std::cout << "live_stream_status=connected " << options.stream_host << ':' << options.stream_port << '\n';
    return true;
}

bool send_stream_scene(const AppOptions& options,
                       const HardwarePlannerRunner& runner,
                       LiveViewStreamClient* streamer) {
    if (!stream_enabled(options) || streamer == nullptr || !streamer->connected()) {
        return true;
    }
    if (!streamer->send_scene(make_live_scene_snapshot(runner))) {
        std::cerr << "live_stream_error=" << streamer->last_error() << '\n';
        streamer->disconnect();
        return true;
    }
    return true;
}

bool process_stream_control(const AppOptions& options,
                            const HardwarePlannerConfig& base_config,
                            HardwarePlannerConfig* active_config,
                            HardwarePlannerRunner* runner,
                            LiveViewStreamClient* streamer,
                            bool* world_applied) {
    if (world_applied != nullptr) {
        *world_applied = false;
    }
    if (!stream_enabled(options) || runner == nullptr || streamer == nullptr || !streamer->connected()) {
        return true;
    }

    const LiveViewStreamClient::PollResult poll_result = streamer->poll();
    if (!streamer->connected()) {
        if (!streamer->last_error().empty()) {
            std::cerr << "live_stream_error=" << streamer->last_error() << '\n';
        }
        return true;
    }
    if (poll_result.robot_profile_received && poll_result.robot_profile.has_value()) {
        try {
            AppOptions profile_options = options;
            profile_options.vehicle_profile = vehicle_profile_from_model(*poll_result.robot_profile);
            HardwarePlannerConfig next_config = base_config;
            apply_vehicle_profile(profile_options, &next_config);
            apply_structured_hardware_config(runner->world(), &next_config);
            apply_mixed_hardware_config(runner->world(), &next_config);
            apply_tracked_mixed_hardware_config(runner->world(), &next_config);
            runner->apply_config(next_config);
            if (active_config != nullptr) {
                *active_config = next_config;
            }
            if (!streamer->send_control_ack(
                    true,
                    std::string("robot profile applied: ") +
                        thesis_sim::vehicle_model_kind_name(next_config.vehicle_model) +
                        " (planner reset)")) {
                std::cerr << "live_stream_error=" << streamer->last_error() << '\n';
                streamer->disconnect();
                return true;
            }
            if (world_applied != nullptr) {
                *world_applied = true;
            }
        } catch (const std::exception& e) {
            if (!streamer->send_control_ack(false, std::string("robot profile rejected: ") + e.what())) {
                std::cerr << "live_stream_error=" << streamer->last_error() << '\n';
                streamer->disconnect();
                return true;
            }
        }
    }

    if (poll_result.world_received && poll_result.world.has_value()) {
        const bool sensor_mode_changed =
            runner->lidar_enabled_for_current_mode() !=
            (poll_result.world->environment_mode() != EnvironmentMode::StructuredRoad);
        try {
            runner->apply_world(*poll_result.world);
            HardwarePlannerConfig next_config = active_config != nullptr ? *active_config : base_config;
            apply_structured_hardware_config(*poll_result.world, &next_config);
            apply_mixed_hardware_config(*poll_result.world, &next_config);
            apply_tracked_mixed_hardware_config(*poll_result.world, &next_config);
            runner->apply_config(next_config);
            if (active_config != nullptr) {
                *active_config = next_config;
            }
            std::string message = "custom map applied from GUI";
            if (sensor_mode_changed) {
                message += " (sensor mode changed; restart the runner if the new scenario needs different hardware ports)";
            }
            if (!streamer->send_control_ack(true, message)) {
                std::cerr << "live_stream_error=" << streamer->last_error() << '\n';
                streamer->disconnect();
                return true;
            }
            if (world_applied != nullptr) {
                *world_applied = true;
            }
        } catch (const std::exception& e) {
            if (!streamer->send_control_ack(false, std::string("custom map rejected: ") + e.what())) {
                std::cerr << "live_stream_error=" << streamer->last_error() << '\n';
                streamer->disconnect();
                return true;
            }
        }
    }
    return true;
}

bool wait_for_initial_stream_world(const AppOptions& options,
                                   const HardwarePlannerConfig& base_config,
                                   HardwarePlannerConfig* active_config,
                                   HardwarePlannerRunner* runner,
                                   LiveViewStreamClient* streamer,
                                   bool* world_applied) {
    if (world_applied != nullptr) {
        *world_applied = false;
    }
    if (!stream_enabled(options) || runner == nullptr || streamer == nullptr || !streamer->connected()) {
        return true;
    }

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(900);
    while (std::chrono::steady_clock::now() < deadline && streamer->connected()) {
        bool applied = false;
        if (!process_stream_control(options, base_config, active_config, runner, streamer, &applied)) {
            return false;
        }
        if (applied) {
            if (world_applied != nullptr) {
                *world_applied = true;
            }
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }
    return true;
}

void stream_frame_if_due(const AppOptions& options,
                         const HardwarePlannerRunner& runner,
                         LiveViewStreamClient* streamer,
                         bool force = false) {
    if (streamer == nullptr || !streamer->connected() || !stream_enabled(options)) {
        return;
    }

    const bool interval_hit = (runner.step_count() % std::max(options.stream_every_n_steps, 1)) == 0;
    if (!force && !interval_hit) {
        return;
    }

    if (!streamer->send_frame(make_live_frame_snapshot(runner))) {
        std::cerr << "live_stream_error=" << streamer->last_error() << '\n';
        streamer->disconnect();
    }
}

ControllerTelemetry make_controller_telemetry(const thesis_sim::VehicleModelState& state,
                                              const thesis_sim::HardwareControlCommand& command) {
    ControllerTelemetry telemetry{};
    telemetry.yaw_mrad = static_cast<std::int32_t>(std::lround(state.yaw * 1000.0));
    telemetry.yaw_rate_mrad_s = static_cast<std::int32_t>(std::lround(state.yaw_rate * 1000.0));
    telemetry.yaw_deg = state.yaw * 180.0 / kPi;
    telemetry.yaw_rate_dps = state.yaw_rate * 180.0 / kPi;
    telemetry.pwm_l = static_cast<std::int16_t>(command.pwm_left);
    telemetry.pwm_r = static_cast<std::int16_t>(command.pwm_right);
    telemetry.target_pwm_l = static_cast<std::int16_t>(command.pwm_left);
    telemetry.target_pwm_r = static_cast<std::int16_t>(command.pwm_right);
    telemetry.ticks_left = state.left_encoder_ticks;
    telemetry.ticks_right = state.right_encoder_ticks;
    telemetry.enc_dt_ms = static_cast<std::uint16_t>(std::lround(state.encoder_dt_ms));
    telemetry.motor_flags =
        static_cast<std::uint16_t>(thesis_sim::MotorFlag::Enabled) |
        static_cast<std::uint16_t>(thesis_sim::MotorFlag::StbyHigh);
    telemetry.status_flags =
        static_cast<std::uint16_t>(thesis_sim::StatusFlag::ImuReady) |
        static_cast<std::uint16_t>(thesis_sim::StatusFlag::EncodersReady) |
        static_cast<std::uint16_t>(thesis_sim::StatusFlag::MotorsReady) |
        static_cast<std::uint16_t>(thesis_sim::StatusFlag::HostLinkOk);
    telemetry.enc_flags =
        static_cast<std::uint16_t>(thesis_sim::EncoderFlag::LeftValid) |
        static_cast<std::uint16_t>(thesis_sim::EncoderFlag::RightValid);
    telemetry.have_imu = true;
    telemetry.have_motor = true;
    telemetry.have_encoder = true;
    telemetry.have_heartbeat = true;
    return telemetry;
}

std::vector<RPLidarA1::ScanPoint> make_lidar_scan(const WorldMap& world,
                                                  const thesis_sim::VehicleModelState& state,
                                                  const thesis_sim::HardwarePlannerConfig& config) {
    constexpr int kBeams = 360;
    const Vec2 origin = lidar_origin_world(state.position, state.yaw, config.localization);
    const std::vector<LidarHit> hits = world.raycast(
        origin,
        state.yaw + config.localization.lidar_yaw_offset,
        kBeams,
        2.0 * kPi,
        config.localization.max_range_m,
        true);

    std::vector<RPLidarA1::ScanPoint> scan;
    scan.reserve(hits.size());
    for (size_t i = 0; i < hits.size(); ++i) {
        const double alpha =
            hits.size() > 1 ? static_cast<double>(i) / static_cast<double>(hits.size() - 1) : 0.0;
        const double angle_deg = -180.0 + alpha * 360.0;
        const double angle_rad = deg_to_rad(angle_deg);
        const double range_m = hits[i].distance;
        scan.push_back({
            static_cast<std::uint8_t>(range_m < config.localization.max_range_m ? 15 : 0),
            angle_deg,
            range_m * 1000.0,
            range_m,
            std::cos(angle_rad) * range_m,
            std::sin(angle_rad) * range_m,
        });
    }
    return scan;
}

VehicleControlInput make_vehicle_control(const thesis_sim::VehicleGeometry& geometry,
                                         const thesis_sim::VehicleModelState& state,
                                         const thesis_sim::HardwareControlCommand& command,
                                         double dt) {
    VehicleControlInput input{};
    input.target_speed = command.target_speed;
    input.target_curvature = command.target_curvature;
    input.accel_cmd = clamp_value(
        (command.target_speed - state.speed) / std::max(dt, 1e-3),
        -geometry.max_decel,
        geometry.max_accel);
    const double target_steer_angle = clamp_value(
        std::atan(geometry.wheelbase * command.target_curvature),
        -geometry.max_steer_angle,
        geometry.max_steer_angle);
    input.target_steer_angle = target_steer_angle;
    input.steer_rate_cmd = clamp_value(
        (target_steer_angle - state.steer_angle) / std::max(dt, 1e-3),
        -geometry.max_steer_rate,
        geometry.max_steer_rate);
    input.target_yaw_rate = command.target_yaw_rate;
    return input;
}

