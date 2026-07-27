int main(int argc, char** argv) {
    install_shutdown_signal_handlers();
    const AppOptions options = parse_args(argc, argv);
    std::unique_ptr<HardwarePlannerRunner> runner;
    try {
        const WorldMap world = make_world_from_options(options);
        const bool structured_mode = world.environment_mode() == EnvironmentMode::StructuredRoad;
        if (!options.simulate && options.controller_port.empty()) {
            print_usage(argv[0]);
            return 2;
        }
        if (!options.simulate && !structured_mode && options.lidar_port.empty()) {
            print_usage(argv[0]);
            return 2;
        }

        RealRobotBridge::Options bridge_options;
        bridge_options.controller.port = options.controller_port;
        bridge_options.controller.baudrate = options.controller_baudrate;
        bridge_options.controller.timeout_s = 0.05;
        bridge_options.controller.startup_delay_s = 4.0;
        bridge_options.controller.heartbeat_interval_s = 0.75;
        bridge_options.controller.reset_on_connect = options.controller_reset_on_connect;
        bridge_options.lidar_port = options.lidar_port;
        bridge_options.lidar_baudrate = options.lidar_baudrate;
        bridge_options.lidar_timeout_s = 0.10;

        HardwarePlannerConfig planner_config;
        planner_config.nominal_dt = options.dt;
        planner_config.auto_set_autonomous_mode = options.auto_mode;
        planner_config.auto_gyro_zero = options.gyro_zero;
        planner_config.use_encoder_odometry = true;
        planner_config.planner_safety_stop_enabled = options.planner_safety_stop_enabled;
        planner_config.mapping_lidar_enabled = options.simulate || !options.lidar_port.empty();
        planner_config.use_mcu_velocity_closed_loop =
            options.mcu_velocity_closed_loop && !options.simulate;
        planner_config.localization_policy = options.localization_policy;
        apply_unstructured_hardware_config(world, &planner_config);
        apply_structured_hardware_config(world, &planner_config);
        apply_mixed_hardware_config(world, &planner_config);
        const HardwarePlannerConfig base_planner_config = planner_config;
        apply_vehicle_profile(options, &planner_config);
        apply_robot_calibration_profile(options, &planner_config);
        apply_unstructured_hardware_config(world, &planner_config);
        apply_structured_hardware_config(world, &planner_config);
        apply_mixed_hardware_config(world, &planner_config);
        apply_tracked_mixed_hardware_config(world, &planner_config);

        if (options.simulate) {
            // The synthetic controller already follows canonical physical
            // left/right ordering; wiring compensation is real-hardware only.
            planner_config.controller_motor_channels_swapped = false;
            return run_simulated(options, world, std::move(bridge_options), base_planner_config, planner_config);
        }

        std::cout << "startup_calibration_profile_version="
                  << planner_config.calibration_profile_version << '\n';
        std::cout << "startup_calibration_profile_hash="
                  << planner_config.calibration_profile_hash << '\n';
        std::cout << "startup_controller_motor_channels_swapped="
                  << (planner_config.controller_motor_channels_swapped ? 1 : 0) << '\n';

        runner = std::make_unique<HardwarePlannerRunner>(
            world,
            bridge_options,
            planner_config);
        ExternalPoseUdpReceiver external_pose_receiver;
        if (!setup_external_pose_receiver(options, &external_pose_receiver)) {
            return 2;
        }
        LiveViewStreamClient streamer;
        SlamToolboxBridgeClient slam_bridge;
        std::uint64_t slam_sequence = 0;
        if (options.slam_bridge_port > 0 &&
            !slam_bridge.open(
                options.slam_bridge_host,
                static_cast<std::uint16_t>(options.slam_bridge_port))) {
            std::cerr << "hardware_runner_warning=SLAM bridge disabled: "
                      << slam_bridge.last_error() << '\n';
        }
        auto next_stream_connect_attempt = std::chrono::steady_clock::now();
        auto ensure_stream_client = [&](bool force_frame_push) {
            if (!stream_enabled(options) || streamer.connected()) {
                return true;
            }

            const auto now = std::chrono::steady_clock::now();
            if (now < next_stream_connect_attempt) {
                return true;
            }
            next_stream_connect_attempt = now + std::chrono::milliseconds(kStreamReconnectRetryMs);

            if (!streamer.connect_to(options.stream_host, static_cast<std::uint16_t>(options.stream_port))) {
                std::cerr << "live_stream_warning=" << streamer.last_error()
                          << " (GUI unreachable, retrying automatically)\n";
                return true;
            }

            std::cout << "live_stream_status=connected " << options.stream_host << ':' << options.stream_port << '\n';
            if (!send_stream_scene(options, *runner, &streamer)) {
                return true;
            }
            stream_frame_if_due(options, *runner, &streamer, force_frame_push);
            return true;
        };
        runner->connect();
        if (!setup_stream_client(options, &streamer)) {
            runner->disconnect();
            return 2;
        }
        bool world_applied = false;
        if (!wait_for_initial_stream_world(options, base_planner_config, &planner_config, runner.get(), &streamer, &world_applied)) {
            runner->disconnect();
            return 2;
        }
        if (!send_stream_scene(options, *runner, &streamer)) {
            runner->disconnect();
            return 2;
        }
        stream_frame_if_due(options, *runner, &streamer, true);
        bool stream_connected_once = streamer.connected();
        auto stop_if_stream_lost = [&]() {
            if (streamer.connected()) {
                stream_connected_once = true;
                return false;
            }
            if (options.stop_on_stream_loss && stream_connected_once) {
                std::cerr << "hardware_runner_error=live stream disconnected; stopping robot\n";
                runner->disconnect();
                return true;
            }
            return false;
        };
        if (stop_if_stream_lost()) {
            return 1;
        }

        const int limit = options.max_steps > 0 ? options.max_steps : 1500;
        while (runner->step_count() < limit && !runner->goal_reached() && !g_shutdown_requested.load()) {
            if (!ensure_stream_client(true)) {
                runner->disconnect();
                return 2;
            }
            if (stop_if_stream_lost()) {
                return 1;
            }
            world_applied = false;
            if (!process_stream_control(options, base_planner_config, &planner_config, runner.get(), &streamer, &world_applied)) {
                runner->disconnect();
                return 2;
            }
            if (stop_if_stream_lost()) {
                return 1;
            }
            if (world_applied) {
                if (!send_stream_scene(options, *runner, &streamer)) {
                    runner->disconnect();
                    return 2;
                }
                stream_frame_if_due(options, *runner, &streamer, true);
                if (stop_if_stream_lost()) {
                    return 1;
                }
                continue;
            }
            poll_external_pose(options, &external_pose_receiver, runner.get());
            runner->step();
            if (slam_bridge.active() && !runner->lidar_hits().empty()) {
                const RealRobotObservation& slam_observation =
                    runner->bridge().observation();
                const double scan_timestamp =
                    slam_observation.lidar_scan_mid_timestamp_s > 0.0
                        ? slam_observation.lidar_scan_mid_timestamp_s
                        : slam_observation.host_timestamp_s;
                slam_bridge.submit_scan(
                    options.vehicle_profile == VehicleProfile::Tank
                        ? "hardware_tank"
                        : "hardware_car",
                    ++slam_sequence,
                    scan_timestamp,
                    runner->estimate().position,
                    runner->estimate().yaw,
                    runner->lidar_hits(),
                    planner_config.localization.min_valid_range_m,
                    planner_config.localization.max_range_m);
                if (slam_bridge.poll() &&
                    options.slam_pose_feedback &&
                    slam_bridge.snapshot().connected &&
                    slam_bridge.snapshot().pose_valid) {
                    runner->apply_slam_pose_correction(
                        slam_bridge.snapshot().corrected_position,
                        slam_bridge.snapshot().corrected_yaw);
                }
            }
            stream_frame_if_due(options, *runner, &streamer);
            if (stop_if_stream_lost()) {
                return 1;
            }
        }
        stream_frame_if_due(options, *runner, &streamer, true);

        const HardwarePlannerReport report = runner->current_report();
        runner->disconnect();

        std::cout << "status=" << (report.goal_reached ? "goal_reached" : "stopped") << '\n';
        std::cout << "telemetry_ready=" << (report.telemetry_ready ? 1 : 0) << '\n';
        std::cout << "safety_stop_active=" << (report.safety_stop_active ? 1 : 0) << '\n';
        std::cout << "controller_front_alert=" << (report.controller_front_alert ? 1 : 0) << '\n';
        std::cout << "lidar_front_blocked=" << (report.lidar_front_blocked ? 1 : 0) << '\n';
        std::cout << "have_lidar_scan=" << (report.have_lidar_scan ? 1 : 0) << '\n';
        std::cout << "steps=" << report.steps << '\n';
        std::cout << "time=" << report.runtime_s << '\n';
        std::cout << "final_x=" << report.final_position.x << '\n';
        std::cout << "final_y=" << report.final_position.y << '\n';
        std::cout << "distance_to_goal=" << report.distance_to_goal << '\n';
        if (!runner->history().empty()) {
            const thesis_sim::HardwareTelemetrySample& latest = runner->history().back();
            std::cout << "structured_track_s=" << latest.structured_track_s << '\n';
            std::cout << "structured_progress_s=" << latest.structured_progress_s << '\n';
            std::cout << "target_speed=" << latest.target_speed << '\n';
            std::cout << "target_yaw_rate=" << latest.target_yaw_rate << '\n';
        }
        std::cout << "min_lidar_distance=" << report.min_lidar_distance << '\n';
        std::cout << "front_lidar_distance=" << report.front_lidar_distance << '\n';
        std::cout << "dynamic_gap_gates=" << (report.dynamic_gap_gates ? 1 : 0) << '\n';
        std::cout << "planner_has_reference=" << (report.planner_has_reference ? 1 : 0) << '\n';
        std::cout << "stall_boost_active=" << (report.stall_boost_active ? 1 : 0) << '\n';
        std::cout << "valid_lidar_points=" << report.valid_lidar_points << '\n';
        std::cout << "close_lidar_points=" << report.close_lidar_points << '\n';
        std::cout << "front_close_lidar_points=" << report.front_close_lidar_points << '\n';
        std::cout << "candidate_gates=" << report.candidate_gates << '\n';
        std::cout << "chosen_gate_distance=" << report.chosen_gate_distance << '\n';
        std::cout << "accumulated_lidar_points=" << report.accumulated_lidar_points << '\n';
        std::cout << "no_motion_command_cycles=" << report.no_motion_command_cycles << '\n';
        std::cout << "passed_gates=" << report.passed_gates << '\n';
        std::cout << "controller_safety_flags=0x" << std::hex << report.controller_safety_flags << std::dec << '\n';
        std::cout << "controller_motor_flags=0x" << std::hex << report.controller_motor_flags << std::dec << '\n';
        std::cout << "controller_status_flags=0x" << std::hex << report.controller_status_flags << std::dec << '\n';
        std::cout << "controller_error_code=0x" << std::hex << report.controller_error_code << std::dec << '\n';
        std::cout << "controller_pwm_left=" << report.controller_pwm_left << '\n';
        std::cout << "controller_pwm_right=" << report.controller_pwm_right << '\n';
        std::cout << "controller_target_pwm_left=" << report.controller_target_pwm_left << '\n';
        std::cout << "controller_target_pwm_right=" << report.controller_target_pwm_right << '\n';
        std::cout << "planned_pwm_left=" << report.planned_pwm_left << '\n';
        std::cout << "planned_pwm_right=" << report.planned_pwm_right << '\n';
        std::cout << "calibration_profile_name="
                  << planner_config.calibration_profile_name << '\n';
        std::cout << "calibration_profile_version="
                  << planner_config.calibration_profile_version << '\n';
        std::cout << "calibration_profile_hash="
                  << planner_config.calibration_profile_hash << '\n';
        std::cout << "controller_motor_channels_swapped="
                  << (planner_config.controller_motor_channels_swapped ? 1 : 0) << '\n';
        std::cout << "wheel_radius_calibrated="
                  << (planner_config.wheel_radius_calibrated ? 1 : 0) << '\n';
        std::cout << "mcu_velocity_closed_loop="
                  << (planner_config.use_mcu_velocity_closed_loop ? 1 : 0) << '\n';
        return report.goal_reached ? 0 : 1;
    } catch (const std::exception& e) {
        try {
            if (runner) {
                runner->disconnect();
            }
        } catch (const std::exception&) {
        }
        std::cerr << "hardware_runner_error=" << e.what() << '\n';
        return 2;
    }
}
