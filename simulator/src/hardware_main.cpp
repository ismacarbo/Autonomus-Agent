#include <cstdlib>
#include <cstring>
#include <exception>
#include <iostream>
#include <string>

#include "hardware_planner_runner.h"

namespace {

using thesis_sim::HardwarePlannerConfig;
using thesis_sim::HardwarePlannerReport;
using thesis_sim::HardwarePlannerRunner;
using thesis_sim::RealRobotBridge;
using thesis_sim::WorldMap;

struct AppOptions {
    std::string controller_port;
    std::string lidar_port;
    int controller_baudrate = 115200;
    int lidar_baudrate = 115200;
    int max_steps = 1500;
    double dt = 0.10;
    bool auto_mode = true;
    bool gyro_zero = true;
};

void print_usage(const char* argv0) {
    std::cout
        << "Usage: " << argv0
        << " --controller-port /dev/ttyACM0 --lidar-port /dev/ttyUSB0 [options]\n"
        << "Options:\n"
        << "  --controller-baudrate N   default 115200\n"
        << "  --lidar-baudrate N        default 115200\n"
        << "  --max-steps N             default 1500\n"
        << "  --dt SEC                  default 0.10\n"
        << "  --no-auto-mode            do not force AUTONOMOUS mode on connect\n"
        << "  --no-gyro-zero            do not send GYRO_ZERO on connect\n";
}

AppOptions parse_args(int argc, char** argv) {
    AppOptions options;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--controller-port" && i + 1 < argc) {
            options.controller_port = argv[++i];
        } else if (arg == "--lidar-port" && i + 1 < argc) {
            options.lidar_port = argv[++i];
        } else if (arg == "--controller-baudrate" && i + 1 < argc) {
            options.controller_baudrate = std::atoi(argv[++i]);
        } else if (arg == "--lidar-baudrate" && i + 1 < argc) {
            options.lidar_baudrate = std::atoi(argv[++i]);
        } else if (arg == "--max-steps" && i + 1 < argc) {
            options.max_steps = std::atoi(argv[++i]);
        } else if (arg == "--dt" && i + 1 < argc) {
            options.dt = std::atof(argv[++i]);
        } else if (arg == "--no-auto-mode") {
            options.auto_mode = false;
        } else if (arg == "--no-gyro-zero") {
            options.gyro_zero = false;
        } else if (arg == "--help" || arg == "-h") {
            print_usage(argv[0]);
            std::exit(0);
        }
    }
    return options;
}

}  // namespace

int main(int argc, char** argv) {
    const AppOptions options = parse_args(argc, argv);
    if (options.controller_port.empty() || options.lidar_port.empty()) {
        print_usage(argv[0]);
        return 2;
    }

    RealRobotBridge::Options bridge_options;
    bridge_options.controller.port = options.controller_port;
    bridge_options.controller.baudrate = options.controller_baudrate;
    bridge_options.controller.timeout_s = 0.05;
    bridge_options.controller.startup_delay_s = 4.0;
    bridge_options.controller.heartbeat_interval_s = 0.75;
    bridge_options.controller.reset_on_connect = true;
    bridge_options.lidar_port = options.lidar_port;
    bridge_options.lidar_baudrate = options.lidar_baudrate;
    bridge_options.lidar_timeout_s = 0.10;

    HardwarePlannerConfig planner_config;
    planner_config.nominal_dt = options.dt;
    planner_config.auto_set_autonomous_mode = options.auto_mode;
    planner_config.auto_gyro_zero = options.gyro_zero;

    HardwarePlannerRunner runner(WorldMap::thesis_demo(), bridge_options, planner_config);

    try {
        runner.connect();
        const HardwarePlannerReport report = runner.run(options.max_steps);
        runner.disconnect();

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
        std::cout << "min_lidar_distance=" << report.min_lidar_distance << '\n';
        std::cout << "front_lidar_distance=" << report.front_lidar_distance << '\n';
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
        return report.goal_reached ? 0 : 1;
    } catch (const std::exception& e) {
        try {
            runner.disconnect();
        } catch (const std::exception&) {
        }
        std::cerr << "hardware_runner_error=" << e.what() << '\n';
        return 2;
    }
}
