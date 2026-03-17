#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <iostream>
#include <memory>
#include <string>

#include "hardware_planner_runner.h"

namespace {

using thesis_sim::ControllerTelemetry;
using thesis_sim::GateBehaviorMode;
using thesis_sim::HardwarePlannerConfig;
using thesis_sim::HardwarePlannerReport;
using thesis_sim::HardwarePlannerRunner;
using thesis_sim::LidarHit;
using thesis_sim::MotorControlMode;
using thesis_sim::RealRobotBridge;
using thesis_sim::RealRobotObservation;
using thesis_sim::RPLidarA1;
using thesis_sim::UnstructuredMapPreset;
using thesis_sim::Vec2;
using thesis_sim::VehicleControlInput;
using thesis_sim::VehicleDynamicsModel;
using thesis_sim::WorldMap;

constexpr double kPi = 3.14159265358979323846;

struct AppOptions {
    std::string controller_port;
    std::string lidar_port;
    int controller_baudrate = 115200;
    int lidar_baudrate = 115200;
    int max_steps = 1500;
    double dt = 0.10;
    bool auto_mode = true;
    bool gyro_zero = true;
    bool simulate = false;
    UnstructuredMapPreset unstructured_preset = UnstructuredMapPreset::RobotValidation;
};

double clamp_value(double value, double lo, double hi) {
    return std::max(lo, std::min(value, hi));
}

double deg_to_rad(double angle_deg) {
    return angle_deg * kPi / 180.0;
}

Vec2 lidar_origin_world(const Vec2& base_position,
                        double base_yaw,
                        const thesis_sim::LidarLocalizationConfig& cfg) {
    const Vec2 local_offset{cfg.lidar_x_offset, cfg.lidar_y_offset};
    const Vec2 rotated = thesis_sim::rotate(local_offset, base_yaw);
    return {base_position.x + rotated.x, base_position.y + rotated.y};
}

UnstructuredMapPreset parse_unstructured_preset(const std::string& value) {
    if (value == "robot_validation" || value == "robot" || value == "validation") {
        return UnstructuredMapPreset::RobotValidation;
    }
    if (value == "tight" || value == "tight_corridor") {
        return UnstructuredMapPreset::TightCorridor;
    }
    if (value == "slalom" || value == "wide_slalom" || value == "wide") {
        return UnstructuredMapPreset::WideSlalom;
    }
    if (value == "lower" || value == "lower_bypass") {
        return UnstructuredMapPreset::LowerBypass;
    }
    return UnstructuredMapPreset::RobotValidation;
}

void print_usage(const char* argv0) {
    std::cout
        << "Usage: " << argv0
        << " --controller-port /dev/ttyACM0 --lidar-port /dev/ttyUSB0 [options]\n"
        << "Or:    " << argv0
        << " --simulate [--unstructured-map robot_validation|tight|slalom|lower] [options]\n"
        << "Options:\n"
        << "  --controller-baudrate N   default 115200\n"
        << "  --lidar-baudrate N        default 115200\n"
        << "  --max-steps N             default 1500\n"
        << "  --dt SEC                  default 0.10\n"
        << "  --simulate                run the hardware planner against synthetic sensors\n"
        << "  --unstructured-map NAME   robot_validation | tight | slalom | lower\n"
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
        } else if (arg == "--simulate") {
            options.simulate = true;
        } else if (arg == "--unstructured-map" && i + 1 < argc) {
            options.unstructured_preset = parse_unstructured_preset(argv[++i]);
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
    telemetry.have_imu = true;
    telemetry.have_motor = true;
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
        config.localization.max_range_m);

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
    return input;
}

int count_passed_gates(const HardwarePlannerRunner& runner) {
    int total = 0;
    for (const gate& gate : runner.gates()) {
        if (gate.passed) {
            ++total;
        }
    }
    return total;
}

int run_simulated(const AppOptions& options,
                  RealRobotBridge::Options bridge_options,
                  HardwarePlannerConfig planner_config) {
    WorldMap world = WorldMap::thesis_demo(options.unstructured_preset, GateBehaviorMode::Static, 7);
    HardwarePlannerRunner runner(world, std::move(bridge_options), planner_config);
    std::unique_ptr<VehicleDynamicsModel> plant = thesis_sim::make_four_wheel_car_model(runner.geometry());
    plant->reset(world.start(), world.start_heading());

    bool collision = false;
    double host_time_s = 0.0;
    const int limit = options.max_steps > 0 ? options.max_steps : 1500;
    for (int step = 0; step < limit && !runner.goal_reached() && !collision; ++step) {
        const thesis_sim::VehicleModelState& plant_state = plant->state();
        RealRobotObservation observation{};
        observation.host_timestamp_s = host_time_s;
        observation.have_controller_telemetry = true;
        observation.controller = make_controller_telemetry(plant_state, runner.last_command());
        observation.have_lidar_scan = true;
        observation.lidar_scan = make_lidar_scan(world, plant_state, planner_config);
        observation.min_lidar_range_m = observation.lidar_scan.empty()
                                            ? 0.0
                                            : observation.lidar_scan.front().distance_m;
        for (const RPLidarA1::ScanPoint& point : observation.lidar_scan) {
            observation.min_lidar_range_m = std::min(observation.min_lidar_range_m, point.distance_m);
        }

        runner.step_with_observation(observation, planner_config.nominal_dt, false);

        const VehicleControlInput control = make_vehicle_control(
            runner.geometry(),
            plant->state(),
            runner.last_command(),
            planner_config.nominal_dt);
        plant->step(planner_config.nominal_dt, control);
        host_time_s += planner_config.nominal_dt;

        collision = world.collides(
            thesis_sim::make_box_corners(
                plant->state().position,
                plant->state().yaw,
                runner.geometry().body_length,
                runner.geometry().body_width));
    }

    const std::string status = runner.goal_reached() ? "goal_reached" : (collision ? "collision" : "timeout");
    std::cout << "status=" << status << '\n';
    std::cout << "telemetry_ready=1\n";
    std::cout << "safety_stop_active=" << (runner.safety_stop_active() ? 1 : 0) << '\n';
    std::cout << "controller_front_alert=0\n";
    std::cout << "lidar_front_blocked="
              << ((runner.estimate().front_lidar_distance > 0.0 &&
                   runner.estimate().front_lidar_distance < planner_config.localization.obstacle_stop_distance_m) ? 1 : 0)
              << '\n';
    std::cout << "have_lidar_scan=1\n";
    std::cout << "steps=" << runner.step_count() << '\n';
    std::cout << "time=" << runner.sim_time() << '\n';
    std::cout << "final_x=" << runner.estimate().position.x << '\n';
    std::cout << "final_y=" << runner.estimate().position.y << '\n';
    std::cout << "distance_to_goal=" << runner.distance_to_goal() << '\n';
    std::cout << "min_lidar_distance=" << runner.estimate().min_lidar_distance << '\n';
    std::cout << "front_lidar_distance=" << runner.estimate().front_lidar_distance << '\n';
    std::cout << "passed_gates=" << count_passed_gates(runner) << '\n';
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

int main(int argc, char** argv) {
    const AppOptions options = parse_args(argc, argv);
    if (!options.simulate && (options.controller_port.empty() || options.lidar_port.empty())) {
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

    if (options.simulate) {
        return run_simulated(options, std::move(bridge_options), planner_config);
    }

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
