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

}  // namespace

int main() {
    try {
        for (int mask = 0; mask < 4; ++mask) check_settings(mask);
        check_settings(0, true);
        std::cout << "hardware_runtime_settings: ok\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "hardware_runtime_settings: " << error.what() << '\n';
        return 1;
    }
}
