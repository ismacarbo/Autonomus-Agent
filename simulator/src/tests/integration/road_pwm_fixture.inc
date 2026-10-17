// End-to-end hardware control fixture. The planner never sees the obstacle
// rectangles used by the physical plant and the synthetic LiDAR.
void check_road_pwm_plant(bool mixed, double side = 1.0, double period = .1) {
    AppOptions options;
    options.environment_mode = mixed ? EnvironmentMode::MixedRoadGates : EnvironmentMode::StructuredRoad;
    auto world = straight_road_world(options.environment_mode);
    world.set_start({.15, .6});
    world.editable_road_centerline() = {{.15, .6}, {.6, .6}, {1., .6}};
    world.finalize_editor_changes();
    auto physical = world;
    if (mixed) physical.editable_obstacles() = side > 0.0
        ? std::vector<thesis_sim::Rect>{{.57, .08, .70, .63}}
        : std::vector<thesis_sim::Rect>{{.57, .57, .70, 1.12}};
    HardwarePlannerConfig config;
    apply_vehicle_profile(options, &config);
    apply_robot_calibration_profile(options, &config);
    apply_structured_hardware_config(world, &config);
    apply_mixed_hardware_config(world, &config);
    config.start_matching.enabled = config.slam_toolbox_enabled = false;
    config.mapping_lidar_enabled = true;
    HardwarePlannerRunner runner(world, RealRobotBridge::Options{}, config);
    std::cout << "road_fixture mixed=" << mixed << " side=" << side << " dt=" << period
        << " goal=" << world.goal().x << ',' << world.goal().y
        << " tolerance=" << config.goal_tolerance_m << " stop_speed=" << config.goal_stop_speed_mps << '\n';
    thesis_sim::VehicleModelState plant{};
    plant.position = world.start();
    // Structured tracking must recover a measured lateral/heading error.
    if (!mixed) { plant.position.y += .025; plant.yaw = .12; }
    runner.reset_pose(plant.position, plant.yaw);
    double left_speed = 0.0, right_speed = 0.0, left_distance = 0.0, right_distance = 0.0;
    const double tick_distance = 2.0*kPi*.0327/38.0;
    double max_speed = 0.0;
    int gate_steps = 0;
    for (int step = 0; step < 1400 && !runner.goal_reached(); ++step) {
        auto observation = stationary_observation(100.0+period*step);
        observation.controller.status_flags = 0x99;
        observation.controller.ticks_left = static_cast<int>(std::lround(left_distance/tick_distance));
        observation.controller.ticks_right = static_cast<int>(std::lround(right_distance/tick_distance));
        observation.controller.yaw_mrad = static_cast<int>(std::lround(plant.yaw*1000));
        observation.controller.yaw_rate_mrad_s = static_cast<int>(std::lround(plant.yaw_rate*1000));
        observation.controller.pwm_l = observation.controller.target_pwm_l = runner.last_command().pwm_left;
        observation.controller.pwm_r = observation.controller.target_pwm_r = runner.last_command().pwm_right;
        auto sensor_config = config;
        sensor_config.localization.lidar_yaw_offset = 0.0;
        sensor_config.localization.lidar_flip_left_right = true;
        observation.lidar_scan = make_lidar_scan(physical, plant, sensor_config);
        observation.lidar_scan_duration_s = 0.0;
        observation.lidar_scan_start_timestamp_s = observation.host_timestamp_s;
        observation.lidar_scan_mid_timestamp_s = observation.host_timestamp_s;
        observation.lidar_scan_end_timestamp_s = observation.host_timestamp_s;
        runner.step_with_observation(observation, period, false);
        const auto& command = runner.last_command();
        gate_steps += runner.diagnostics().control_source == thesis_sim::HardwareControlSource::GateMpc;
        if (std::getenv("THESIS_TEST_GATE_TRACE")) {
            const auto& h = runner.history().back();
            std::cout << "road," << mixed << ',' << step << ',' << plant.position.x << ',' << plant.position.y
                << ',' << plant.yaw << ',' << command.target_speed << ',' << command.planner_target_yaw_rate
                << ',' << command.pwm_left << ',' << command.pwm_right << ',' << runner.passed_gate_count()
                << ',' << h.locked_gate_crossing_x << ',' << h.locked_gate_crossing_y
                << ',' << runner.estimate().position.x << ',' << runner.estimate().position.y
                << ',' << runner.distance_to_goal() << ',' << runner.estimate().speed
                << ',' << runner.diagnostics().reference_invalidation_reason << '\n';
        }
        const auto motor = [&](int pwm, bool right) {
            if (pwm == 0) return 0.0;
            const double canonical = right ? (std::abs(pwm)-26.021505)/.849462 : std::abs(pwm);
            return std::copysign(std::max(0.0, canonical-45.0)*.006209*(right ? .90 : 1.10),
                                 static_cast<double>(pwm));
        };
        const double requested_left = motor(command.pwm_left, false), requested_right = motor(command.pwm_right, true);
        for (int sub = 0; sub < static_cast<int>(std::lround(period/.01)); ++sub) {
            left_speed += (requested_left-left_speed)*(1.0-std::exp(-.01/.24));
            right_speed += (requested_right-right_speed)*(1.0-std::exp(-.01/.24));
            left_distance += left_speed*.01;
            right_distance += right_speed*.01;
            plant.speed = .5*(left_speed+right_speed);
            plant.yaw_rate = .65*(right_speed-left_speed)/.13;
            plant.position.x += plant.speed*std::cos(plant.yaw+.005*plant.yaw_rate)*.01;
            plant.position.y += plant.speed*std::sin(plant.yaw+.005*plant.yaw_rate)*.01;
            plant.yaw += plant.yaw_rate*.01;
            max_speed = std::max(max_speed, std::abs(plant.speed));
            require(!physical.collides(thesis_sim::make_box_corners(plant.position, plant.yaw, .25, .15), 0.0),
                    "road PWM plant collided with the arena or an unknown obstacle");
        }
    }
    std::cout << "road_pwm mixed=" << mixed << " side=" << side << " dt=" << period << " goal=" << runner.goal_reached()
        << " gates=" << runner.passed_gate_count() << " pose=" << plant.position.x << ',' << plant.position.y
        << " max_speed=" << max_speed << " time=" << runner.history().back().time
        << " reason=" << runner.diagnostics().reference_invalidation_reason << '\n';
    require(runner.goal_reached() && distance(plant.position, world.goal()) < .10,
            "road PWM plant did not reach the physical goal");
    require(!mixed || (gate_steps > 20 && runner.passed_gate_count() == 2),
            "unknown obstacle did not produce a complete bypass/rejoin sequence");
}

void check_mixed_blocked_reference() {
    AppOptions options;
    options.environment_mode = EnvironmentMode::MixedRoadGates;
    auto world = straight_road_world(options.environment_mode);
    auto physical = world;
    physical.editable_obstacles() = {{.54, 0.0, .65, 1.2}};
    HardwarePlannerConfig config;
    apply_vehicle_profile(options, &config);
    apply_robot_calibration_profile(options, &config);
    apply_mixed_hardware_config(world, &config);
    config.start_matching.enabled = config.slam_toolbox_enabled = false;
    HardwarePlannerRunner runner(world, RealRobotBridge::Options{}, config);
    thesis_sim::VehicleModelState pose{};
    pose.position = {.30, .60};
    runner.reset_pose(pose.position, 0.0);
    for (int step = 0; step < 12; ++step) {
        auto observation = stationary_observation(100.0+.1*step);
        observation.controller.status_flags = 0x99;
        observation.lidar_scan = make_lidar_scan(physical, pose, config);
        observation.lidar_scan_duration_s = 0.0;
        runner.step_with_observation(observation, .1, false);
        require(runner.last_command().pwm_left == 0 && runner.last_command().pwm_right == 0,
                "blocked mixed road authorized motion through the swept-footprint guard");
        require(!runner.goal_reached() && runner.passed_gate_count() == 0,
                "blocked mixed road counted a gate or completed without motion");
    }
}
