int run_test(const TestOptions& options) {
    if (!options.simulate && options.controller_port.empty()) {
        std::cerr << "controller port is required unless --simulate is used\n";
        return 2;
    }

    const std::vector<MotionSegment> script = build_motion_script(options);
    const double total_duration_s = total_script_duration(script);
    const WorldMap local_reference_world = make_local_mapping_world(options);
    const Pose2D local_origin_pose{0.0, 0.0, local_reference_world.start_heading()};
    OccupancyGrid grid(options.map.size_m, options.map.resolution_m);
    EkfSlamLocalizer localizer(options.drive);
    RunStats stats;
    std::vector<Pose2D> trail;
    std::vector<LidarHit> current_scan_hits;
    std::vector<Vec2> slam_points;

    const std::string telemetry_path = options.output_prefix + "_telemetry.csv";
    const std::string trail_path = options.output_prefix + "_trail.csv";
    const std::string map_path = options.output_prefix + "_map.pgm";
    const std::string summary_path = options.output_prefix + "_summary.txt";

    std::ofstream telemetry_log(telemetry_path);
    if (!telemetry_log.is_open()) {
        std::cerr << "cannot write telemetry log: " << telemetry_path << '\n';
        return 3;
    }
    telemetry_log
        << "time_s,phase,cmd_left,cmd_right,controller_ready,pwm_left,pwm_right,target_pwm_left,target_pwm_right,"
        << "ticks_left,ticks_right,delta_left,delta_right,ekf_x,ekf_y,ekf_yaw_deg,ekf_speed,ekf_yaw_rate_dps,"
        << "front_distance_m,min_scan_range_m,have_lidar,matched_pose,map_points\n";

    const VehicleGeometry geometry = localizer.geometry();
    const WorldMap stream_world = make_stream_world(options);
    const LiveSceneSnapshot stream_scene = make_smoke_scene(options, geometry, stream_world);
    const Vec2 stream_shift = stream_offset(stream_world);
    LiveViewStreamClient streamer;
    if (!setup_stream_client(options, stream_scene, &streamer)) {
        return 4;
    }
    std::cout << "stream_profile=slam"
              << " scenario=" << (options.environment_mode == EnvironmentMode::StructuredRoad ? "structured" : "unstructured")
              << " preset="
              << (options.environment_mode == EnvironmentMode::StructuredRoad
                      ? structured_map_cli_name(options.structured_preset)
                      : unstructured_map_cli_name(options.unstructured_preset))
              << " infinite=" << (options.infinite ? 1 : 0)
              << '\n';

    WorldMap sim_world;
    SimRobotState sim_state{};
    RealRobotBridge bridge;
    bool bridge_connected = false;
    ControllerTelemetry last_telemetry{};
    bool have_last_telemetry = false;

    try {
        if (options.simulate) {
            sim_world = local_reference_world;
            localizer.reset(local_origin_pose);
            sim_state.pose = local_origin_pose;
        } else {
            RealRobotBridge::Options bridge_options;
            bridge_options.controller.port = options.controller_port;
            bridge_options.controller.baudrate = options.controller_baudrate;
            bridge_options.lidar_port = options.lidar_port;
            bridge_options.lidar_baudrate = options.lidar_baudrate;
            bridge = RealRobotBridge(bridge_options);
            bridge.connect(!options.lidar_port.empty());
            bridge_connected = true;
            localizer.reset(local_origin_pose);

            if (bridge.controller_connected()) {
                const auto ping = bridge.ping();
                std::cout << "controller_ping_ack=" << static_cast<int>(ping.status) << '\n';
                if (options.auto_mode) {
                    const auto mode_ack = bridge.set_mode(ControllerMode::Autonomous);
                    std::cout << "controller_mode_ack=" << static_cast<int>(mode_ack.status) << '\n';
                }
                if (options.gyro_zero) {
                    const auto gyro_ack = bridge.gyro_zero();
                    std::cout << "controller_gyro_zero_ack=" << static_cast<int>(gyro_ack.status) << '\n';
                }
                bridge.send_pwm(0, 0, true, MotorControlMode::SafeDirectPwm);
            }
        }

        double last_front_distance = 0.0;
        double next_report_time = 0.0;
        const double start_time_s = monotonic_seconds();

        while (true) {
            const double now_s = monotonic_seconds();
            const double elapsed_s = now_s - start_time_s;
            if (!options.infinite && elapsed_s >= total_duration_s) {
                break;
            }

            const double scripted_elapsed_s =
                options.infinite && total_duration_s > 1e-6
                    ? std::fmod(elapsed_s, total_duration_s)
                    : elapsed_s;
            const std::size_t phase_index = phase_index_at_time(script, scripted_elapsed_s);
            const MotionSegment& segment = script[phase_index];
            int cmd_left = segment.pwm_left;
            int cmd_right = segment.pwm_right;
            bool safety_stop_active = false;

            if (cmd_left > 0 && cmd_right > 0 &&
                last_front_distance > 0.0 &&
                last_front_distance < options.obstacle_stop_distance_m) {
                cmd_left = 0;
                cmd_right = 0;
                safety_stop_active = true;
                ++stats.blocked_cycles;
            }

            if (!options.simulate) {
                bridge.send_pwm(cmd_left, cmd_right, true, MotorControlMode::SafeDirectPwm);
            }
            if (cmd_left != 0 || cmd_right != 0) {
                ++stats.nonzero_command_cycles;
            }

            std::this_thread::sleep_for(std::chrono::duration<double>(std::max(options.dt, 0.01)));

            const double sample_time_s = monotonic_seconds();
            const double lidar_begin_s = monotonic_seconds();
            RealRobotObservation observation{};
            if (options.simulate) {
                observation = make_sim_observation(
                    sim_world,
                    &sim_state,
                    cmd_left,
                    cmd_right,
                    options.dt,
                    options,
                    sample_time_s);
                last_telemetry = observation.controller;
                have_last_telemetry = true;
            } else {
                bridge.pump(options.dt * 0.25, options.map.lidar_min_points, !options.lidar_port.empty());
                observation = bridge.observation();
                last_telemetry = bridge.controller().telemetry_snapshot();
                have_last_telemetry = true;
            }
            const double lidar_ms = (monotonic_seconds() - lidar_begin_s) * 1000.0;

            const bool controller_ready = observation.have_controller_telemetry;
            if (controller_ready) {
                stats.saw_controller_ready = true;
            }

            const double estimator_begin_s = monotonic_seconds();
            OdomUpdate odom{};
            if (controller_ready) {
                odom = localizer.fuse_controller(observation.controller, options.dt);
                stats.total_abs_left_ticks += std::abs(odom.left_delta_ticks);
                stats.total_abs_right_ticks += std::abs(odom.right_delta_ticks);
                stats.total_distance_m += std::abs(odom.distance_m);
                if ((cmd_left != 0 || cmd_right != 0) &&
                    (std::abs(observation.controller.target_pwm_l) > 0 ||
                     std::abs(observation.controller.target_pwm_r) > 0 ||
                     std::abs(observation.controller.pwm_l) > 0 ||
                     std::abs(observation.controller.pwm_r) > 0)) {
                    ++stats.pwm_echo_cycles;
                    stats.saw_pwm_echo = true;
                }
                if ((cmd_left != 0 || cmd_right != 0) &&
                    (std::abs(odom.left_delta_ticks) > 0 || std::abs(odom.right_delta_ticks) > 0)) {
                    ++stats.encoder_motion_cycles;
                    stats.saw_encoder_motion = true;
                }
            }
            const double estimator_ms_before_lidar = (monotonic_seconds() - estimator_begin_s) * 1000.0;

            bool matched_pose = false;
            double min_scan_range = observation.have_lidar_scan
                                        ? (observation.min_lidar_range_m > 0.0
                                               ? observation.min_lidar_range_m
                                               : compute_min_scan_range(observation.lidar_scan, options.map))
                                        : 0.0;
            double front_distance = observation.have_lidar_scan
                                        ? compute_front_distance(observation.lidar_scan, options.map)
                                        : 0.0;
            last_front_distance = front_distance;
            if (front_distance > 0.0) {
                stats.min_front_distance_m = std::min(stats.min_front_distance_m, front_distance);
            }

            current_scan_hits.clear();
            if (observation.have_lidar_scan) {
                stats.saw_lidar = true;
                ++stats.lidar_frames;

                const Pose2D predicted_pose = localizer.estimate().pose;
                const Pose2D matched_pose_candidate =
                    match_scan_to_map(grid, predicted_pose, observation.lidar_scan, options.map);
                if (std::hypot(matched_pose_candidate.x - predicted_pose.x,
                               matched_pose_candidate.y - predicted_pose.y) > 1e-4 ||
                    std::abs(wrap_angle(matched_pose_candidate.yaw - predicted_pose.yaw)) > 1e-4) {
                    matched_pose = true;
                    ++stats.scan_match_updates;
                }
                localizer.fuse_lidar_pose(matched_pose_candidate, true);
                current_scan_hits = build_lidar_hits_world(localizer.estimate().pose, observation.lidar_scan, options.map);
                grid.integrate_scan(localizer.estimate().pose, observation.lidar_scan, options.map);
                if ((stats.cycles % 2) == 0 || slam_points.empty()) {
                    slam_points = grid.extract_occupied_points(options.map.max_stream_points);
                }
            }
            const double estimator_ms =
                estimator_ms_before_lidar + (monotonic_seconds() - estimator_begin_s) * 1000.0 - estimator_ms_before_lidar;

            const SlamEstimate& estimate = localizer.estimate();
            trail.push_back(estimate.pose);

            HardwareTelemetrySample sample{};
            sample.time = elapsed_s;
            sample.speed = estimate.speed;
            sample.accel = estimate.accel;
            sample.yaw_rate = estimate.yaw_rate;
            sample.jerk = 0.0;
            sample.command_r = estimate_target_yaw_rate(cmd_left, cmd_right, options.drive);
            sample.target_speed = estimate_target_speed(cmd_left, cmd_right, options.drive);
            sample.target_yaw_rate = sample.command_r;
            sample.curvature = estimate.curvature;
            sample.distance_to_goal = stats.total_distance_m;
            sample.min_lidar = min_scan_range;
            sample.front_lidar = front_distance;
            sample.planner_speed_ref = sample.target_speed;
            sample.tracker_cross_track = 0.0;
            sample.tracker_heading_error_deg = 0.0;
            sample.planning_ms = 0.0;
            sample.tracking_ms = 0.0;
            sample.lidar_ms = lidar_ms;
            sample.estimator_ms = estimator_ms;
            sample.step_ms = options.dt * 1000.0;
            sample.visible_gates = static_cast<double>(current_scan_hits.size());
            sample.pwm_left = controller_ready ? observation.controller.target_pwm_l : cmd_left;
            sample.pwm_right = controller_ready ? observation.controller.target_pwm_r : cmd_right;

            const ControllerTelemetry* telemetry_ptr = controller_ready ? &observation.controller : nullptr;
            const LiveFrameSnapshot frame = make_smoke_frame(
                elapsed_s,
                stats.cycles,
                options.simulate || (bridge_connected && bridge.controller_connected()),
                controller_ready,
                safety_stop_active,
                stats.total_distance_m,
                min_scan_range,
                front_distance,
                estimate,
                stream_world,
                geometry,
                options.drive,
                odom,
                telemetry_ptr,
                cmd_left,
                cmd_right,
                trail,
                current_scan_hits,
                slam_points,
                sample,
                stream_shift);
            stream_frame_if_due(options, stats.cycles, frame, &streamer);

            telemetry_log
                << elapsed_s << ','
                << segment.label << ','
                << cmd_left << ','
                << cmd_right << ','
                << (controller_ready ? 1 : 0) << ','
                << (controller_ready ? observation.controller.pwm_l : last_telemetry.pwm_l) << ','
                << (controller_ready ? observation.controller.pwm_r : last_telemetry.pwm_r) << ','
                << (controller_ready ? observation.controller.target_pwm_l : last_telemetry.target_pwm_l) << ','
                << (controller_ready ? observation.controller.target_pwm_r : last_telemetry.target_pwm_r) << ','
                << (controller_ready ? observation.controller.ticks_left : last_telemetry.ticks_left) << ','
                << (controller_ready ? observation.controller.ticks_right : last_telemetry.ticks_right) << ','
                << odom.left_delta_ticks << ','
                << odom.right_delta_ticks << ','
                << estimate.pose.x << ','
                << estimate.pose.y << ','
                << rad_to_deg(estimate.pose.yaw) << ','
                << estimate.speed << ','
                << rad_to_deg(estimate.yaw_rate) << ','
                << front_distance << ','
                << min_scan_range << ','
                << (observation.have_lidar_scan ? 1 : 0) << ','
                << (matched_pose ? 1 : 0) << ','
                << slam_points.size() << '\n';

            ++stats.cycles;
            if (elapsed_s >= next_report_time) {
                std::cout << std::fixed << std::setprecision(2)
                          << "t=" << elapsed_s
                          << "s phase=" << segment.label
                          << " cmd=(" << cmd_left << "," << cmd_right << ")"
                          << " ticks=(" << odom.left_delta_ticks << "," << odom.right_delta_ticks << ")"
                          << " pose=(" << estimate.pose.x << "," << estimate.pose.y << "," << rad_to_deg(estimate.pose.yaw) << "deg)"
                          << " front=" << front_distance
                          << " map_pts=" << slam_points.size()
                          << " matched=" << (matched_pose ? 1 : 0)
                          << '\n';
                next_report_time += 0.50;
            }
        }

        if (!options.simulate && bridge_connected && bridge.controller_connected()) {
            bridge.stop(thesis_sim::StopReason::UserRequest, false);
            bridge.send_pwm(0, 0, true, MotorControlMode::SafeDirectPwm);
        }
    } catch (...) {
        if (!options.simulate && bridge_connected) {
            try {
                if (bridge.controller_connected()) {
                    bridge.stop(thesis_sim::StopReason::FaultRecovery, false);
                    bridge.send_pwm(0, 0, true, MotorControlMode::SafeDirectPwm);
                }
            } catch (const std::exception&) {
            }
            try {
                bridge.disconnect();
            } catch (const std::exception&) {
            }
        }
        throw;
    }

    if (!options.simulate && bridge_connected) {
        bridge.disconnect();
    }

    telemetry_log.flush();
    write_trail_csv(trail_path, trail);
    const bool map_saved = grid.save_pgm(map_path, trail);
    write_summary(summary_path, stats, localizer.estimate().pose, last_telemetry, have_last_telemetry, slam_points.size());

    std::cout << "result=" << classify_result(stats) << '\n';
    std::cout << "telemetry_csv=" << telemetry_path << '\n';
    std::cout << "trail_csv=" << trail_path << '\n';
    std::cout << "map_pgm=" << (map_saved ? map_path : std::string("save_failed")) << '\n';
    std::cout << "summary_txt=" << summary_path << '\n';
    std::cout << "controller_ready=" << (stats.saw_controller_ready ? 1 : 0)
              << " pwm_echo=" << (stats.saw_pwm_echo ? 1 : 0)
              << " encoder_motion=" << (stats.saw_encoder_motion ? 1 : 0)
              << " lidar_frames=" << stats.lidar_frames
              << " total_distance_m=" << stats.total_distance_m
              << " map_points=" << slam_points.size()
              << '\n';
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const TestOptions options = parse_args(argc, argv);
        return run_test(options);
    } catch (const std::exception& e) {
        std::cerr << "hardware_smoke_test_error=" << e.what() << '\n';
        return 1;
    }
}
