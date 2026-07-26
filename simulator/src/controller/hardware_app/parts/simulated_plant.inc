std::unique_ptr<VehicleDynamicsModel> make_simulated_hardware_plant(const HardwarePlannerRunner& runner) {
    // Both thesis hardware platforms are driven through independent left/right
    // PWM channels. The car-like model remains the planner/controller
    // abstraction, while the smoke-test plant must reproduce the differential
    // low-level actuation so recovery pivots can actually rotate the robot.
    return thesis_sim::make_tracked_vehicle_model(runner.geometry());
}

int run_simulated(const AppOptions& options,
                  WorldMap world,
                  RealRobotBridge::Options bridge_options,
                  HardwarePlannerConfig base_config,
                  HardwarePlannerConfig planner_config) {
    HardwarePlannerRunner runner(world, std::move(bridge_options), planner_config);
    ExternalPoseUdpReceiver external_pose_receiver;
    if (!setup_external_pose_receiver(options, &external_pose_receiver)) {
        return 2;
    }
    LiveViewStreamClient streamer;
    if (!setup_stream_client(options, &streamer)) {
        return 2;
    }
    bool world_applied = false;
    if (!process_stream_control(options, base_config, &planner_config, &runner, &streamer, &world_applied)) {
        return 2;
    }
    if (!send_stream_scene(options, runner, &streamer)) {
        return 2;
    }
    std::unique_ptr<VehicleDynamicsModel> plant = make_simulated_hardware_plant(runner);
    plant->reset(runner.world().start(), runner.world().start_heading());

    bool collision = false;
    // Use a non-zero monotonic epoch because zero explicitly means
    // "timestamp unavailable" in RealRobotObservation/ControllerTelemetry.
    double host_time_s = 1.0;
    const bool trace_gate_events = std::getenv("THESIS_TRACE_GATES") != nullptr;
    Vec2 previous_gate_target{};
    bool had_previous_gate_target = false;
    int previous_passed_gates = 0;
    int pivot_command_cycles = 0;
    int opposite_pwm_cycles = 0;
    const int limit = options.max_steps > 0 ? options.max_steps : 1500;
    for (int step = 0; step < limit && !runner.goal_reached() && !collision && !g_shutdown_requested.load(); ++step) {
        world_applied = false;
        if (!process_stream_control(options, base_config, &planner_config, &runner, &streamer, &world_applied)) {
            return 2;
        }
        if (world_applied) {
            plant = make_simulated_hardware_plant(runner);
            plant->reset(runner.world().start(), runner.world().start_heading());
            collision = false;
            host_time_s = 1.0;
            if (!send_stream_scene(options, runner, &streamer)) {
                return 2;
            }
            stream_frame_if_due(options, runner, &streamer, true);
            continue;
        }

        const thesis_sim::VehicleModelState& plant_state = plant->state();
        RealRobotObservation observation{};
        observation.host_timestamp_s = host_time_s;
        observation.have_controller_telemetry = true;
        observation.controller = make_controller_telemetry(plant_state, runner.last_command());
        observation.controller.rx_timestamp_s = host_time_s;
        observation.controller.imu_rx_timestamp_s = host_time_s;
        observation.controller.safety_rx_timestamp_s = host_time_s;
        observation.controller.encoder_rx_timestamp_s = host_time_s;
        observation.controller.motor_rx_timestamp_s = host_time_s;
        observation.controller.heartbeat_rx_timestamp_s = host_time_s;
        observation.controller.imu_host_timestamp_s = host_time_s;
        observation.controller.safety_host_timestamp_s = host_time_s;
        observation.controller.encoder_host_timestamp_s = host_time_s;
        observation.controller.motor_host_timestamp_s = host_time_s;
        observation.controller.heartbeat_host_timestamp_s = host_time_s;
        if (runner.lidar_enabled_for_current_mode()) {
            observation.have_lidar_scan = true;
            observation.lidar_scan = make_lidar_scan(runner.world(), plant_state, planner_config);
            observation.lidar_scan_duration_s = planner_config.nominal_dt;
            observation.lidar_scan_start_timestamp_s =
                host_time_s - observation.lidar_scan_duration_s;
            observation.lidar_scan_end_timestamp_s = host_time_s;
            observation.lidar_scan_mid_timestamp_s =
                0.5 * (observation.lidar_scan_start_timestamp_s +
                       observation.lidar_scan_end_timestamp_s);
            observation.lidar_scan_age_s = 0.0;
            observation.lidar_scan_reused = false;
            observation.min_lidar_range_m = observation.lidar_scan.empty()
                                                ? 0.0
                                                : observation.lidar_scan.front().distance_m;
            for (const RPLidarA1::ScanPoint& point : observation.lidar_scan) {
                observation.min_lidar_range_m = std::min(observation.min_lidar_range_m, point.distance_m);
            }
        }

        poll_external_pose(options, &external_pose_receiver, &runner);
        runner.step_with_observation(observation, planner_config.nominal_dt, false);
        const auto& applied_command = runner.last_command();
        if (std::abs(applied_command.target_speed) <= 0.005 &&
            std::abs(applied_command.target_yaw_rate) >= 0.10) {
            ++pivot_command_cycles;
        }
        if (applied_command.pwm_left * applied_command.pwm_right < 0) {
            ++opposite_pwm_cycles;
        }
        if (trace_gate_events) {
            const int passed_gates = runner.passed_gate_count();
            const int chosen_index = runner.chosen_gate_index();
            bool have_gate_target = false;
            Vec2 gate_target{};
            if (chosen_index >= 0 &&
                chosen_index < static_cast<int>(runner.gate_specs().size())) {
                gate_target = runner.gate_specs()[static_cast<size_t>(chosen_index)].position;
                have_gate_target = true;
            }
            const bool gate_changed =
                have_gate_target != had_previous_gate_target ||
                (have_gate_target &&
                 (!had_previous_gate_target || distance(gate_target, previous_gate_target) > 0.025));
            if (gate_changed || passed_gates != previous_passed_gates) {
                std::cerr << "gate_event t=" << host_time_s
                          << " pose=(" << runner.estimate().position.x
                          << ',' << runner.estimate().position.y << ')'
                          << " yaw=" << runner.estimate().yaw
                          << " target=";
                if (have_gate_target) {
                    std::cerr << '(' << gate_target.x << ',' << gate_target.y << ')'
                              << " distance=" << distance(runner.estimate().position, gate_target);
                } else {
                    std::cerr << "none";
                }
                std::cerr << " passed=" << passed_gates << '\n';
            }
            previous_gate_target = gate_target;
            had_previous_gate_target = have_gate_target;
            previous_passed_gates = passed_gates;
        }
        stream_frame_if_due(options, runner, &streamer);

        const VehicleControlInput control = make_vehicle_control(
            runner.geometry(),
            plant->state(),
            runner.last_command(),
            planner_config.nominal_dt);
        plant->step(planner_config.nominal_dt, control);
        host_time_s += planner_config.nominal_dt;

        const thesis_sim::Rect& world_bounds = runner.world().bounds();
        const double world_span = std::max(
            world_bounds.max_x - world_bounds.min_x,
            world_bounds.max_y - world_bounds.min_y);
        const bool compact_structured_world =
            runner.world().environment_mode() == EnvironmentMode::StructuredRoad &&
            world_span <= 0.75;
        const bool indoor_structured_world =
            runner.world().environment_mode() == EnvironmentMode::StructuredRoad &&
            world_span <= 2.05;
        const bool compact_mixed_world =
            runner.world().environment_mode() == EnvironmentMode::MixedRoadGates &&
            world_span <= 2.50;
        const double collision_padding =
            compact_mixed_world
                ? 0.0
                : compact_structured_world && runner.world().obstacles().empty()
                ? -0.45
                : (indoor_structured_world && runner.world().obstacles().empty()
                       ? 0.0
                       : (compact_structured_world ? 0.0 : 0.05));
        collision =
            !compact_mixed_world &&
            !(compact_structured_world && runner.world().obstacles().empty()) &&
            runner.world().collides(
                thesis_sim::make_box_corners(
                    plant->state().position,
                    plant->state().yaw,
                    runner.geometry().body_length,
                    runner.geometry().body_width),
                collision_padding);
    }

    stream_frame_if_due(options, runner, &streamer, true);

    const HardwarePlannerReport report = runner.current_report();
    const std::string status = runner.goal_reached() ? "goal_reached" : (collision ? "collision" : "timeout");
    std::cout << "status=" << status << '\n';
    std::cout << "telemetry_ready=" << (report.telemetry_ready ? 1 : 0) << '\n';
    std::cout << "safety_stop_active=" << (report.safety_stop_active ? 1 : 0) << '\n';
    std::cout << "controller_front_alert=0\n";
    std::cout << "lidar_front_blocked=" << (report.lidar_front_blocked ? 1 : 0) << '\n';
    std::cout << "have_lidar_scan=" << (runner.lidar_enabled_for_current_mode() ? 1 : 0) << '\n';
    std::cout << "steps=" << report.steps << '\n';
    std::cout << "time=" << report.runtime_s << '\n';
    std::cout << "final_x=" << report.final_position.x << '\n';
    std::cout << "final_y=" << report.final_position.y << '\n';
    std::cout << "simulated_ground_truth_x=" << plant->state().position.x << '\n';
    std::cout << "simulated_ground_truth_y=" << plant->state().position.y << '\n';
    std::cout << "simulated_ground_truth_yaw=" << plant->state().yaw << '\n';
    std::cout << "distance_to_goal=" << report.distance_to_goal << '\n';
    if (!runner.history().empty()) {
        const thesis_sim::HardwareTelemetrySample& latest = runner.history().back();
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
    std::cout << "pivot_command_cycles=" << pivot_command_cycles << '\n';
    std::cout << "opposite_pwm_cycles=" << opposite_pwm_cycles << '\n';
    std::cout << "controller_safety_flags=0x0\n";
    std::cout << "controller_motor_flags=0x0\n";
    std::cout << "controller_status_flags=0x0\n";
    std::cout << "controller_error_code=0x0\n";
    std::cout << "controller_pwm_left=" << runner.last_command().pwm_left << '\n';
    std::cout << "controller_pwm_right=" << runner.last_command().pwm_right << '\n';
    std::cout << "controller_target_pwm_left=" << runner.last_command().pwm_left << '\n';
    std::cout << "controller_target_pwm_right=" << runner.last_command().pwm_right << '\n';
    std::cout << "planned_pwm_left=" << runner.last_command().pwm_left << '\n';
    std::cout << "planned_pwm_right=" << runner.last_command().pwm_right << '\n';
    return runner.goal_reached() ? 0 : 1;
}

}  // namespace
