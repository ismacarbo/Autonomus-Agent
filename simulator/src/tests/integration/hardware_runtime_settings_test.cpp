#include <future>
#include <stdexcept>
#include <unistd.h>

// Exercise the runner's actual CLI and TCP control path, without serial devices.
#include "../../controller/hardware_app/parts/options.inc"
#include "../../controller/hardware_app/parts/stream_config.inc"
}  // namespace

namespace {

void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

class ViewerConnection {
  public:
    thesis_sim::LiveViewStreamServer server;
    LiveViewStreamClient client;

    ViewerConnection() {
        const int first_port = 42000 + (getpid() % 8000);
        for (int offset = 0; offset < 50 && !server.listening(); ++offset) {
            server.start(static_cast<std::uint16_t>(first_port + offset));
        }
        require(server.listening(), "could not start test viewer");
        auto connection = std::async(std::launch::async, [&]() {
            return client.connect_to("127.0.0.1", server.port());
        });
        while (connection.wait_for(std::chrono::milliseconds(1)) != std::future_status::ready) {
            server.poll();
        }
        require(connection.get(), "runner/viewer handshake failed");
        server.poll();
    }

    std::string exchange(const AppOptions& options,
                         HardwarePlannerConfig* config,
                         HardwarePlannerRunner* runner,
                         bool* reset) {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
        *reset = false;
        while (std::chrono::steady_clock::now() < deadline) {
            bool changed = false;
            require(process_stream_control(options, *config, config, runner, &client, &changed),
                    "runtime control processing failed");
            *reset = *reset || changed;
            auto result = server.poll();
            if (result.control_ack_received) {
                require(result.control_ack->ok, "viewer setting rejected unexpectedly");
                return result.control_ack->message;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        throw std::runtime_error("timed out waiting for runtime settings acknowledgement");
    }
};

RealRobotObservation stationary_observation(double timestamp) {
    RealRobotObservation observation;
    observation.host_timestamp_s = timestamp;
    observation.have_controller_telemetry = true;
    auto& controller = observation.controller;
    controller.have_imu = controller.have_encoder = controller.have_motor = true;
    controller.have_heartbeat = true;
    controller.fw_major = 1;
    controller.fw_minor = 5;
    controller.ms = controller.imu_ms = controller.encoder_ms =
        static_cast<std::uint32_t>(std::lround(timestamp * 1000.0));
    controller.rx_timestamp_s = controller.imu_host_timestamp_s =
        controller.encoder_host_timestamp_s = controller.heartbeat_host_timestamp_s = timestamp;
    controller.imu_rx_timestamp_s = controller.encoder_rx_timestamp_s =
        controller.motor_rx_timestamp_s = controller.heartbeat_rx_timestamp_s = timestamp;
    controller.enc_dt_ms = 100;
    // Match the failed physical run: valid analog inputs but no encoder arming
    // yet. This must not deadlock the first motion command after a firmware flash.
    controller.status_flags = 0x91;
    controller.enc_flags = 0x3;
    observation.have_lidar_scan = true;
    observation.lidar_scan_start_timestamp_s = timestamp - 0.10;
    observation.lidar_scan_mid_timestamp_s = timestamp - 0.05;
    observation.lidar_scan_end_timestamp_s = timestamp;
    observation.lidar_scan_duration_s = 0.10;
    for (int i = 0; i < 360; ++i) {
        const double angle = -180.0 + i;
        const double radians = angle * kPi / 180.0;
        observation.lidar_scan.push_back({
            15, angle, 1200.0, 1.2, 1.2 * std::cos(radians), 1.2 * std::sin(radians)});
    }
    return observation;
}

bool run_stationary_samples(HardwarePlannerRunner* runner) {
    bool commanded_forward = false;
    for (int step = 0; step < 20; ++step) {
        runner->step_with_observation(stationary_observation(10.0 + step * 0.1), 0.1, false);
        const auto& command = runner->last_command();
        commanded_forward = commanded_forward ||
            (command.target_speed > 0.0 && command.pwm_left > 0 && command.pwm_right > 0);
    }
    return commanded_forward;
}

void check_settings(int disabled_mask, bool simulate = false) {
    std::vector<std::string> arguments{
        "runner", "--scenario", "unstructured", "--unstructured-map", "hardware_lab"};
    if ((disabled_mask & 1) != 0) arguments.push_back("--no-start-matching");
    if ((disabled_mask & 2) != 0) arguments.push_back("--no-slam-toolbox");
    if (simulate) arguments.push_back("--simulate");
    std::vector<char*> argv;
    for (auto& argument : arguments) argv.push_back(argument.data());
    AppOptions options = parse_args(static_cast<int>(argv.size()), argv.data());
    ViewerConnection viewer;
    options.stream_host = "127.0.0.1";
    options.stream_port = viewer.server.port();
    const WorldMap world = make_world_from_options(options);
    HardwarePlannerConfig config;
    apply_unstructured_hardware_config(world, &config);
    config.start_matching.enabled = options.start_matching_enabled && !simulate;
    config.start_matching.reference_path = "/nonexistent/runtime-settings-test.csv";
    config.slam_toolbox_enabled = options.slam_toolbox_enabled;
    HardwarePlannerRunner runner(world, RealRobotBridge::Options{}, config);

    LiveRuntimeSettings gui_defaults;
    gui_defaults.start_matching_enabled = true;
    gui_defaults.slam_toolbox_enabled = true;
    require(viewer.server.queue_runtime_settings(gui_defaults), "could not queue GUI defaults");
    bool reset = false;
    const std::string acknowledgement = viewer.exchange(options, &config, &runner, &reset);
    const bool expect_matching = (disabled_mask & 1) == 0 && !simulate;
    const bool expect_slam = (disabled_mask & 2) == 0;
    require(runner.config().start_matching.enabled == expect_matching,
            "GUI defaults overrode --no-start-matching");
    require(runner.config().slam_toolbox_enabled == expect_slam,
            "GUI defaults overrode --no-slam-toolbox");
    require(!reset, "unchanged effective settings reset the planner");
    if (disabled_mask == 3) {
        require(acknowledgement.find("--no-start-matching") != std::string::npos &&
                acknowledgement.find("--no-slam-toolbox") != std::string::npos,
                "acknowledgement omitted the CLI override explanation");
    }
    const bool commanded_forward = run_stationary_samples(&runner);
    if (expect_matching) {
        require(!commanded_forward && runner.last_command().pwm_left == 0 &&
                runner.last_command().pwm_right == 0 &&
                !runner.start_matching_status().accepted,
                "enabled start matching failed to inhibit motion with a missing reference");
    } else {
        require(commanded_forward, "debug bypass did not produce forward PWM in open space");
    }

    const int previous_step = runner.step_count();
    require(viewer.server.queue_runtime_settings(gui_defaults), "could not resend GUI defaults");
    viewer.exchange(options, &config, &runner, &reset);
    require(!reset && runner.step_count() == previous_step,
            "repeated GUI settings discarded the running mission");

    // Map and robot-profile messages must retain the effective runtime choices.
    require(viewer.server.queue_robot_profile(VehicleModelKind::CarLikeBicycle),
            "could not queue robot profile");
    viewer.exchange(options, &config, &runner, &reset);
    require(viewer.server.queue_world(world), "could not queue world");
    viewer.exchange(options, &config, &runner, &reset);
    require(runner.config().start_matching.enabled == expect_matching &&
            runner.config().slam_toolbox_enabled == expect_slam,
            "map/profile reset lost the CLI overrides");

    // Without CLI restrictions an explicit GUI toggle still enables the guard.
    if (disabled_mask == 0 && !simulate) {
        LiveRuntimeSettings disabled;
        disabled.start_matching_enabled = disabled.slam_toolbox_enabled = false;
        viewer.server.queue_runtime_settings(disabled);
        viewer.exchange(options, &config, &runner, &reset);
        require(reset && !runner.config().slam_toolbox_enabled &&
                run_stationary_samples(&runner), "GUI debug toggle did not release the hold");
        viewer.server.queue_runtime_settings(gui_defaults);
        viewer.exchange(options, &config, &runner, &reset);
        require(reset && runner.config().slam_toolbox_enabled &&
                !run_stationary_samples(&runner), "GUI could not restore start matching");
    }
}

void check_actuation_guards() {
    AppOptions options;
    options.environment_mode = EnvironmentMode::UnstructuredGates;
    const auto world = make_world_from_options(options);
    HardwarePlannerConfig config;
    apply_vehicle_profile(options, &config);
    apply_robot_calibration_profile(options, &config);
    apply_unstructured_hardware_config(world, &config);
    config.start_matching.enabled = config.slam_toolbox_enabled = false;
    require(config.pwm.min_effective_pwm == 45, "hardware setup overwrote the measured running deadband");
    HardwarePlannerRunner runner(world, RealRobotBridge::Options{}, config);
    bool braked = false;
    for (int step = 0; step < 12; ++step) {
        auto observation = stationary_observation(20.0 + step * 0.1);
        observation.controller.status_flags = 0x99;
        observation.controller.ticks_left = observation.controller.ticks_right = 4 * step;
        runner.step_with_observation(observation, 0.1, false);
        if (step == 1) {
            require(runner.history().back().overspeed_braking_active > 0.5 &&
                    !runner.diagnostics().stall_boost_active,
                    "fresh motion was ignored while waiting for odometry readiness");
        }
        if (runner.history().back().overspeed_braking_active > 0.5) {
            braked = true;
            require(runner.last_command().pwm_left == 0 && runner.last_command().pwm_right == 0,
                    "minimum-PWM or steering clamp overrode overspeed braking");
        }
    }
    require(braked, "0.21 m/s encoder speed did not brake a 0.06 m/s request");
    bool resumed = false;
    for (int step = 0; step < 12; ++step) {
        auto observation = stationary_observation(21.2 + step * 0.1);
        observation.controller.status_flags = 0x99;
        observation.controller.ticks_left = observation.controller.ticks_right = 45 + step;
        runner.step_with_observation(observation, 0.1, false);
        resumed = resumed || (runner.last_command().pwm_left > 0 && runner.last_command().pwm_right > 0);
    }
    require(resumed, "speed regulation did not release braking after slowing down");

    HardwarePlannerRunner feedback_guard(world, RealRobotBridge::Options{}, config);
    auto stale = stationary_observation(22.0);
    stale.controller.imu_host_timestamp_s -= .65;
    stale.controller.encoder_host_timestamp_s -= .65;
    feedback_guard.step_with_observation(stale, .1, false);
    require(feedback_guard.last_command().pwm_left == 0 &&
            feedback_guard.last_command().pwm_right == 0 &&
            feedback_guard.history().back().reference_invalidation_reason == "motion_feedback_stale",
            "fresh heartbeat allowed motion with stale IMU/encoder feedback");
    feedback_guard.step_with_observation(stationary_observation(22.1), .1, false);
    require(!feedback_guard.last_command().safety_stop &&
            feedback_guard.last_command().target_speed > 0.0,
            "fresh motion telemetry did not release the temporary hold");

    HardwarePlannerRunner pulse_guard(world, RealRobotBridge::Options{}, config);
    bool saw_pulse = false;
    for (int step = 0; step < 14; ++step) {
        auto observation = stationary_observation(24.0 + step * .1);
        observation.controller.status_flags = 0x99;
        observation.controller.pwm_l = pulse_guard.last_command().pwm_left;
        observation.controller.pwm_r = pulse_guard.last_command().pwm_right;
        pulse_guard.step_with_observation(observation, .1, false);
        if (!pulse_guard.diagnostics().stall_boost_active) continue;
        saw_pulse = true;
        const auto stalled_cycles = pulse_guard.history().back().no_motion_cycles;
        // A cached zero-delta observation cannot authorize another spurt.
        observation.host_timestamp_s += .02;
        pulse_guard.step_with_observation(observation, .02, false);
        require(!pulse_guard.diagnostics().stall_boost_active &&
                pulse_guard.history().back().no_motion_cycles == stalled_cycles,
                "cached feedback extended the breakaway pulse or counted another stall");
        break;
    }
    require(saw_pulse, "fresh stationary encoder intervals never authorized a breakaway pulse");

    HardwarePlannerRunner startup(world, RealRobotBridge::Options{}, config);
    bool startup_pulsed = false;
    for (int step = 0; step < 25; ++step) {
        auto observation = stationary_observation(26.0 + step * .1);
        // 0x91: analog inputs valid; encoders await first motion to arm.
        observation.controller.pwm_l = startup.last_command().pwm_left;
        observation.controller.pwm_r = startup.last_command().pwm_right;
        startup.step_with_observation(observation, .1, false);
        startup_pulsed = startup_pulsed || startup.diagnostics().stall_boost_active;
    }
    require(startup_pulsed, "encoder readiness created a startup breakaway deadlock");
    require(startup.history().back().drivetrain_stall_stop_active > .5 &&
            startup.last_command().pwm_left == 0 && startup.last_command().pwm_right == 0,
            "unarmed encoders permitted unbounded startup pulses");

    HardwarePlannerRunner scrub(world, RealRobotBridge::Options{}, config);
    for (int step = 0; step < 8; ++step) {
        auto observation = stationary_observation(25.0 + step * .1);
        observation.controller.status_flags = 0x99;
        observation.controller.ticks_left = 3 * step;
        scrub.step_with_observation(observation, .1, false);
    }
    const double expected_linear_speed = .5 * 3 * (2 * kPi * .0327 / 38) / .1;
    require(std::abs(scrub.estimate().speed - expected_linear_speed) < .001,
            "rotational scrub silently discarded valid encoder translation");

    HardwarePlannerRunner stalled(world, RealRobotBridge::Options{}, config);
    int left_ticks = 0;
    bool stopped = false;
    bool isolated_pulse = false;
    for (int step = 0; step < 45; ++step) {
        // The left wheel only moves when actually commanded. It must not be
        // diagnosed as stalled during the right wheel's isolated recovery pulse.
        const bool healthy_side_held = stalled.last_command().pwm_left == 0;
        if (!healthy_side_held) ++left_ticks;
        auto observation = stationary_observation(30.0 + step * 0.1);
        observation.controller.status_flags = 0x99;
        observation.controller.ticks_left = left_ticks;
        observation.controller.ticks_right = 0;
        stalled.step_with_observation(observation, 0.1, false);
        const auto& command = stalled.last_command();
        isolated_pulse = isolated_pulse || (command.pwm_left == 0 && command.pwm_right >= 110);
        stopped = stopped || stalled.history().back().drivetrain_stall_stop_active > 0.5;
        if (healthy_side_held && !stopped) {
            require(stalled.history().back().left_wheel_stall_cycles == 0,
                    "intentional healthy-wheel hold triggered an opposite-side boost");
        }
        if (stopped) {
            require(command.pwm_left == 0 && command.pwm_right == 0 && command.safety_stop,
                    "persistent single-wheel stall resumed motion without a reset");
        }
    }
    require(isolated_pulse && stopped, "single-wheel recovery did not terminate in a bounded stop");
    require(stalled.history().back().reference_invalidation_reason == "drivetrain_stall_stop",
            "persistent stall was not identified in telemetry");
    stalled.reset();
    stalled.step_with_observation(stationary_observation(40.0), 0.1, false);
    require(stalled.history().back().drivetrain_stall_stop_active == 0.0,
            "explicit mission reset did not clear the stall latch");
}

void check_stream_backpressure() {
    ViewerConnection viewer;
    thesis_sim::LiveFrameSnapshot frame;
    frame.global_free_points.resize(60000, {0.25, 0.50});
    for (int step = 0; step < 40; ++step) {
        frame.step_count = step;
        const auto started = std::chrono::steady_clock::now();
        require(viewer.client.send_frame(frame), "paused GUI disconnected the nonblocking sender");
        require(std::chrono::steady_clock::now() - started < std::chrono::milliseconds(250),
                "telemetry send blocked the motor control loop");
    }
    // Reproduce a GUI pause longer than the previous blocking send timeout.
    const auto pause_until = std::chrono::steady_clock::now() + std::chrono::milliseconds(1200);
    while (std::chrono::steady_clock::now() < pause_until) {
        viewer.client.poll();
        require(viewer.client.connected(), "brief receiver pause was treated as link loss");
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    require(viewer.client.send_control_ack(true, "after frames"), "could not queue control acknowledgement");
    bool latest_received = false;
    bool ack_received = false;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while ((!latest_received || !ack_received) && std::chrono::steady_clock::now() < deadline) {
        viewer.client.poll();
        const auto received = viewer.server.poll();
        if (received.frame_received) {
            require(received.frame->global_free_points.size() == 60000,
                    "partial TCP frame corrupted during coalescing");
            latest_received = received.frame->step_count == 39;
        }
        ack_received = ack_received || (received.control_ack_received && received.control_ack->ok);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    require(latest_received && ack_received, "latest telemetry or reliable control packet was lost");
    viewer.server.stop();
    for (int i = 0; i < 50 && viewer.client.connected(); ++i) {
        viewer.client.send_frame(frame);
        viewer.client.poll();
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    require(!viewer.client.connected(), "real GUI disconnect was hidden by the send queue");

    ViewerConnection blocked_viewer;
    for (int i = 0; i < 40; ++i) {
        require(blocked_viewer.client.send_frame(frame), "persistent-block fixture failed to enqueue");
    }
    const auto stall_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(8);
    while (blocked_viewer.client.connected() && std::chrono::steady_clock::now() < stall_deadline) {
        blocked_viewer.client.poll();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    require(!blocked_viewer.client.connected() &&
            blocked_viewer.client.last_error().find("no send progress") != std::string::npos,
            "permanently blocked telemetry did not expose stream loss to the runner");
}

void check_car_lidar_sides() {
    AppOptions options;
    options.environment_mode = EnvironmentMode::UnstructuredGates;
    HardwarePlannerConfig config;
    apply_vehicle_profile(options, &config);
    apply_robot_calibration_profile(options, &config);
    config.start_matching.enabled = config.slam_toolbox_enabled = false;
    HardwarePlannerRunner runner(make_world_from_options(options), RealRobotBridge::Options{}, config);
    auto observation = stationary_observation(10.0);
    observation.lidar_scan.clear();
    // Independent mounting fixture: raw 162 deg is forward, 132 is right,
    // 192 is left. Do not synthesize these angles from the config under test.
    for (const auto [angle, range] : {std::pair{132.0, 0.6}, {162.0, 0.8}, {192.0, 0.7}}) {
        observation.lidar_scan.push_back({15, angle, range * 1000, range, 0, 0});
    }
    runner.step_with_observation(observation, 0.1, false);
    const Vec2 origin = lidar_origin_world(runner.world().start(), 0.0, config.localization);
    require(runner.lidar_hits().size() == 3, "missing independent LiDAR fixture returns");
    for (const auto& hit : runner.lidar_hits()) {
        const double angle = std::atan2(hit.point.y - origin.y, hit.point.x - origin.x);
        const double expected = hit.distance < .65 ? -kPi / 6 : hit.distance < .75 ? kPi / 6 : 0.0;
        require(std::abs(angle - expected) < 1e-6,
                "physical right/left LiDAR returns are mirrored or forward was rotated");
    }
    // A reference captured with the old mirror must not silently override the
    // corrected sensor frame during start matching.
    thesis_sim::mvc::model::InitialLidarReferenceCloud reference;
    reference.metadata.stability_valid = true;
    reference.points = {{.4, -.2}, {.4, .2}, {.8, .2}};
    const auto path = std::filesystem::temp_directory_path() /
        ("thesis_sensor_frame_" + std::to_string(getpid()) + ".csv");
    std::string error;
    require(thesis_sim::mvc::model::write_initial_lidar_reference(path.string(), reference, &error),
            "could not write sensor-frame reference fixture");
    config.start_matching.enabled = true;
    config.start_matching.reference_path = path.string();
    HardwarePlannerRunner old_reference(make_world_from_options(options), RealRobotBridge::Options{}, config);
    require(old_reference.start_matching_status().status == "reference_sensor_extrinsics_mismatch" &&
            !old_reference.start_matching_status().accepted,
            "old mirrored start reference silently accepted");
    reference.metadata.lidar_yaw_offset_rad = config.localization.lidar_yaw_offset;
    reference.metadata.lidar_flip_left_right = config.localization.lidar_flip_left_right;
    require(thesis_sim::mvc::model::write_initial_lidar_reference(path.string(), reference, &error),
            "could not write corrected sensor-frame fixture");
    HardwarePlannerRunner new_reference(make_world_from_options(options), RealRobotBridge::Options{}, config);
    require(new_reference.start_matching_status().reference_loaded,
            "matching sensor-frame reference incorrectly rejected");
    std::filesystem::remove(path);
}

void check_lateral_gate_pwm_plant(double lateral_offset, double period = 0.1,
                                  double initial_heading = 0.0, bool loaded = false) {
    AppOptions options;
    options.environment_mode = EnvironmentMode::UnstructuredGates;
    auto world = make_world_from_options(options);
    world.set_start_heading(initial_heading);
    HardwarePlannerConfig config;
    apply_vehicle_profile(options, &config);
    apply_robot_calibration_profile(options, &config);
    apply_unstructured_hardware_config(world, &config);
    if (loaded && std::getenv("THESIS_TEST_LEGACY_ACTUATION")) config.pwm.body_yaw_pwm_kp = 0.0;
    config.start_matching.enabled = config.slam_toolbox_enabled = false;
    auto physical = world;
    const double plane_x = world.start().x + 0.50;
    const double gate_y = world.start().y + lateral_offset;
    physical.set_bounds({-3.0, -3.0, 3.0, 3.0});
    physical.editable_obstacles() = {
        {plane_x, -3.0, plane_x + 0.04, gate_y - 0.18},
        {plane_x, gate_y + 0.18, plane_x + 0.04, 3.0}};
    physical.finalize_editor_changes();
    HardwarePlannerRunner runner(world, RealRobotBridge::Options{}, config);
    thesis_sim::VehicleModelState plant{};
    plant.position = world.start();
    plant.yaw = initial_heading;
    double left_speed = 0.0, right_speed = 0.0;
    double left_distance = 0.0, right_distance = 0.0;
    const double tick_distance = 2.0 * kPi * 0.0327 / 38.0;
    int gate_steps = 0;
    double max_speed = 0.0;
    double gate_speed_sum = 0.0;
    // The plant consumes emitted physical PWM, not the requested v/omega.
    // Include side-gain error, actuator lag, quantized encoders and yaw slip.
    for (int step = 0; step < 350 && plant.position.x < plane_x + 0.13; ++step) {
        auto observation = stationary_observation(50.0 + period * step);
        observation.controller.status_flags = 0x99;
        observation.controller.ticks_left = static_cast<int>(std::lround(left_distance / tick_distance));
        observation.controller.ticks_right = static_cast<int>(std::lround(right_distance / tick_distance));
        observation.controller.yaw_mrad = static_cast<int>(std::lround(plant.yaw * 1000.0));
        observation.controller.yaw_rate_mrad_s = static_cast<int>(std::lround(plant.yaw_rate * 1000.0));
        observation.controller.pwm_l = runner.last_command().pwm_left;
        observation.controller.pwm_r = runner.last_command().pwm_right;
        observation.controller.target_pwm_l = observation.controller.pwm_l;
        observation.controller.target_pwm_r = observation.controller.pwm_r;
        auto sensor_config = config;
        // The physical sensor does not change handedness when runner settings
        // change. Keep the observation generator independent of those settings.
        sensor_config.localization.lidar_yaw_offset = -162.0 * kPi / 180.0;
        sensor_config.localization.lidar_flip_left_right = false;
        observation.lidar_scan = make_lidar_scan(physical, plant, sensor_config);
        runner.step_with_observation(observation, period, false);
        const auto& command = runner.last_command();
        const bool tracking_gate = runner.diagnostics().control_source == thesis_sim::HardwareControlSource::GateMpc;
        if (std::getenv("THESIS_TEST_GATE_TRACE")) {
            std::cout << "trace," << step << ',' << plant.position.x << ',' << plant.position.y
                      << ',' << plant.yaw << ',' << plant.speed << ',' << plant.yaw_rate
                      << ',' << runner.estimate().position.x << ',' << runner.estimate().position.y
                      << ',' << command.target_speed << ',' << command.planner_target_yaw_rate
                      << ',' << command.pwm_left << ',' << command.pwm_right
                      << ',' << static_cast<int>(runner.diagnostics().control_source)
                      << ',' << runner.diagnostics().reference_invalidation_reason << '\n';
        }
        gate_steps += tracking_gate;
        if (tracking_gate) gate_speed_sum += std::abs(plant.speed);
        const auto motor_speed = [&](int pwm, bool right) {
            if (pwm == 0) return 0.0;
            const double canonical = right
                ? (std::abs(pwm) - 26.021505) / 0.849462 : std::abs(pwm);
            const double measured = right ? right_speed : left_speed;
            if (loaded && canonical < (std::abs(measured) < .015 ? 90.0 : 62.0)) return 0.0;
            return std::copysign(std::max(0.0, canonical - 45.0) * 0.006209 *
                                (right ? 0.90 : 1.10), static_cast<double>(pwm));
        };
        const double requested_left = motor_speed(command.pwm_left, false);
        const double requested_right = motor_speed(command.pwm_right, true);
        for (int substep = 0; substep < static_cast<int>(std::lround(period / 0.01)); ++substep) {
            left_speed += (requested_left - left_speed) * (1.0 - std::exp(-0.01 / 0.24));
            right_speed += (requested_right - right_speed) * (1.0 - std::exp(-0.01 / 0.24));
            left_distance += left_speed * 0.01;
            right_distance += right_speed * 0.01;
            plant.speed = 0.5 * (left_speed + right_speed);
            const double difference = right_speed - left_speed;
            const double turning_difference = loaded
                ? std::copysign(std::max(0.0, std::abs(difference) - .08), difference) : difference;
            plant.yaw_rate = 0.65 * turning_difference / 0.13;
            plant.position.x += plant.speed * std::cos(plant.yaw + 0.005 * plant.yaw_rate) * 0.01;
            plant.position.y += plant.speed * std::sin(plant.yaw + 0.005 * plant.yaw_rate) * 0.01;
            plant.yaw += plant.yaw_rate * 0.01;
            max_speed = std::max(max_speed, std::abs(plant.speed));
            require(!physical.collides(thesis_sim::make_box_corners(
                        plant.position, plant.yaw, runner.geometry().body_length,
                        runner.geometry().body_width), 0.0),
                    "PWM-driven footprint hit a gate post");
        }
    }
    std::cout << "pwm_gate offset=" << lateral_offset << " period=" << period
              << " loaded=" << loaded
              << " initial_heading=" << initial_heading << " gate_steps=" << gate_steps
              << " passed=" << runner.passed_gate_count() << " pose=" << plant.position.x << ',' << plant.position.y
              << " yaw=" << plant.yaw << " max_speed=" << max_speed
              << " gate_mean_speed=" << gate_speed_sum / std::max(gate_steps, 1)
              << " reason=" << runner.diagnostics().reference_invalidation_reason << '\n';
    require(gate_steps >= 5, "PWM-driven lateral approach never tracked a clothoid");
    require(runner.passed_gate_count() == 1 && plant.position.x > plane_x,
            "PWM-driven plant did not physically cross the lateral gate");
    require(std::abs(plant.position.y - gate_y) < 0.10 &&
            lateral_offset * (plant.position.y - world.start().y) > 0.0,
            "PWM-driven plant passed outside the lateral aperture");
}

}  // namespace

int main(int argc, char** argv) {
    try {
        if (argc == 2 && std::string(argv[1]) == "--loaded-gates") {
            check_lateral_gate_pwm_plant(0.18, 0.15, 0.0, true);
            check_lateral_gate_pwm_plant(-0.18, 0.15, 0.0, true);
            return 0;
        }
        check_car_lidar_sides();
        {
            const char* args[] = {"runner", "--slam-observe-only"};
            auto options = parse_args(2, const_cast<char**>(args));
            require(options.slam_observe_only && !options.slam_pose_feedback,
                    "SLAM observation mode did not preserve disabled EKF feedback");
            const char* conflicting[] = {"runner", "--slam-observe-only", "--slam-pose-feedback"};
            bool rejected = false;
            try { parse_args(3, const_cast<char**>(conflicting)); }
            catch (const std::runtime_error&) { rejected = true; }
            require(rejected, "contradictory SLAM observation/feedback flags were accepted");
        }
        for (int mask = 0; mask < 4; ++mask) check_settings(mask);
        check_settings(0, true);
        check_actuation_guards();
        check_stream_backpressure();
        check_lateral_gate_pwm_plant(0.18);
        check_lateral_gate_pwm_plant(-0.18);
        check_lateral_gate_pwm_plant(0.18, 0.15, -0.35);
        check_lateral_gate_pwm_plant(-0.18, 0.15, 0.35);
        std::cout << "hardware_runtime_settings: ok\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "hardware_runtime_settings: " << error.what() << '\n';
        return 1;
    }
}
