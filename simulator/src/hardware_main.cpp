#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <fstream>
#include <iostream>
#include <iterator>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "hardware_planner_runner.h"
#include "live_view_stream.h"

namespace {

using thesis_sim::ControllerTelemetry;
using thesis_sim::EnvironmentMode;
using thesis_sim::GateSpec;
using thesis_sim::GateBehaviorMode;
using thesis_sim::HardwarePlannerConfig;
using thesis_sim::HardwarePlannerReport;
using thesis_sim::HardwarePlannerRunner;
using thesis_sim::LidarHit;
using thesis_sim::LiveViewStreamClient;
using thesis_sim::MotorControlMode;
using thesis_sim::RealRobotBridge;
using thesis_sim::RealRobotObservation;
using thesis_sim::RPLidarA1;
using thesis_sim::Rect;
using thesis_sim::StructuredMapPreset;
using thesis_sim::UnstructuredMapPreset;
using thesis_sim::Vec2;
using thesis_sim::VehicleControlInput;
using thesis_sim::VehicleDynamicsModel;
using thesis_sim::VehicleModelKind;
using thesis_sim::WorldMap;

std::atomic<bool> g_shutdown_requested{false};

void handle_shutdown_signal(int) {
    g_shutdown_requested.store(true);
}

void install_shutdown_signal_handlers() {
    std::signal(SIGINT, handle_shutdown_signal);
    std::signal(SIGTERM, handle_shutdown_signal);
}

constexpr double kPi = 3.14159265358979323846;
constexpr int kStreamReconnectRetryMs = 1500;

enum class VehicleProfile {
    Car,
    Tank,
};

VehicleModelKind vehicle_model_from_profile(VehicleProfile profile) {
    return profile == VehicleProfile::Tank
               ? VehicleModelKind::TrackedVehicle
               : VehicleModelKind::CarLikeBicycle;
}

VehicleProfile vehicle_profile_from_model(VehicleModelKind model) {
    return model == VehicleModelKind::TrackedVehicle
               ? VehicleProfile::Tank
               : VehicleProfile::Car;
}

struct AppOptions {
    std::string controller_port;
    std::string lidar_port;
    int controller_baudrate = 115200;
    int lidar_baudrate = 115200;
    int max_steps = 1500;
    double dt = 0.10;
    bool auto_mode = true;
    bool gyro_zero = true;
    bool planner_safety_stop_enabled = false;
    bool controller_reset_on_connect = false;
    bool simulate = false;
    bool stop_on_stream_loss = false;
    VehicleProfile vehicle_profile = VehicleProfile::Car;
    int encoder_ticks_per_revolution = 0;
    double wheel_radius_m = 0.0;
    double track_width_m = 0.0;
    double max_linear_speed_mps = 0.0;
    double max_yaw_rate_rad_s = 0.0;
    double cruise_speed_limit_mps = 0.0;
    EnvironmentMode environment_mode = EnvironmentMode::StructuredRoad;
    UnstructuredMapPreset unstructured_preset = UnstructuredMapPreset::HardwareLab;
    StructuredMapPreset structured_preset = StructuredMapPreset::ValidationRoad;
    std::string world_file;
    std::string stream_host;
    int stream_port = 0;
    int stream_every_n_steps = 1;
};

double clamp_value(double value, double lo, double hi) {
    return std::max(lo, std::min(value, hi));
}

Vec2 scale_point_about(const Vec2& point, const Vec2& center, double scale) {
    return {
        center.x + (point.x - center.x) * scale,
        center.y + (point.y - center.y) * scale,
    };
}

Rect scale_rect_about(const Rect& rect, const Vec2& center, double scale) {
    const Vec2 min_point = scale_point_about({rect.min_x, rect.min_y}, center, scale);
    const Vec2 max_point = scale_point_about({rect.max_x, rect.max_y}, center, scale);
    return {
        std::min(min_point.x, max_point.x),
        std::min(min_point.y, max_point.y),
        std::max(min_point.x, max_point.x),
        std::max(min_point.y, max_point.y),
    };
}

Rect structured_content_bounds(const WorldMap& world) {
    Rect bounds{
        std::min(world.start().x, world.goal().x),
        std::min(world.start().y, world.goal().y),
        std::max(world.start().x, world.goal().x),
        std::max(world.start().y, world.goal().y),
    };
    const auto include_point = [&](const Vec2& point) {
        bounds.min_x = std::min(bounds.min_x, point.x);
        bounds.min_y = std::min(bounds.min_y, point.y);
        bounds.max_x = std::max(bounds.max_x, point.x);
        bounds.max_y = std::max(bounds.max_y, point.y);
    };
    for (const Vec2& point : world.road_centerline()) {
        include_point(point);
    }
    for (const Rect& obstacle : world.obstacles()) {
        include_point({obstacle.min_x, obstacle.min_y});
        include_point({obstacle.max_x, obstacle.max_y});
    }
    return bounds;
}

WorldMap fit_structured_hardware_world(WorldMap world) {
    constexpr double kStructuredMaxSpanM = 0.40;
    constexpr double kRoadEdgeMarginM = 0.04;
    if (world.environment_mode() != EnvironmentMode::StructuredRoad) {
        return world;
    }

    const Rect content = structured_content_bounds(world);
    const double content_span = std::max(content.max_x - content.min_x, content.max_y - content.min_y);
    const double target_content_span = std::max(0.28, kStructuredMaxSpanM - 2.0 * kRoadEdgeMarginM);
    const Vec2 center{
        (content.min_x + content.max_x) * 0.5,
        (content.min_y + content.max_y) * 0.5,
    };
    const double scale = content_span > target_content_span ? target_content_span / content_span : 1.0;
    if (std::abs(scale - 1.0) > 1e-6) {
        world.set_start(scale_point_about(world.start(), center, scale));
        world.set_goal(scale_point_about(world.goal(), center, scale));
        for (Rect& obstacle : world.editable_obstacles()) {
            obstacle = scale_rect_about(obstacle, center, scale);
        }
        for (GateSpec& gate : world.editable_gates()) {
            gate.position = scale_point_about(gate.position, center, scale);
            gate.anchor_position = scale_point_about(gate.anchor_position, center, scale);
            gate.motion_amplitude.x *= scale;
            gate.motion_amplitude.y *= scale;
        }
        for (Vec2& point : world.editable_road_centerline()) {
            point = scale_point_about(point, center, scale);
        }
    }
    const double half_span = 0.5 * kStructuredMaxSpanM;
    world.set_bounds({
        center.x - half_span,
        center.y - half_span,
        center.x + half_span,
        center.y + half_span,
    });
    return world;
}

double deg_to_rad(double angle_deg) {
    return angle_deg * kPi / 180.0;
}

EnvironmentMode parse_environment_mode(const std::string& value) {
    if (value == "unstructured") {
        return EnvironmentMode::UnstructuredGates;
    }
    if (value == "mixed" || value == "mixed_road_gates" || value == "road_gates") {
        return EnvironmentMode::MixedRoadGates;
    }
    return EnvironmentMode::StructuredRoad;
}

UnstructuredMapPreset parse_unstructured_preset(const std::string& value) {
    if (value == "tight" || value == "tight_corridor") {
        return UnstructuredMapPreset::TightCorridor;
    }
    if (value == "slalom" || value == "wide_slalom" || value == "wide") {
        return UnstructuredMapPreset::WideSlalom;
    }
    if (value == "lower" || value == "lower_bypass") {
        return UnstructuredMapPreset::LowerBypass;
    }
    if (value == "hardware_lab" || value == "lab") {
        return UnstructuredMapPreset::HardwareLab;
    }
    if (value == "custom") {
        return UnstructuredMapPreset::Custom;
    }
    return UnstructuredMapPreset::RobotValidation;
}

StructuredMapPreset parse_structured_preset(const std::string& value) {
    if (value == "circle" || value == "circle_loop") {
        return StructuredMapPreset::CircleLoop;
    }
    if (value == "zigzag" || value == "zig_zag") {
        return StructuredMapPreset::ZigZag;
    }
    if (value == "hardware" || value == "hardware_track") {
        return StructuredMapPreset::HardwareTrack;
    }
    if (value == "figure_eight" || value == "figure8" || value == "eight") {
        return StructuredMapPreset::FigureEight;
    }
    if (value == "tank_circuit" || value == "circuit" || value == "practice_circuit") {
        return StructuredMapPreset::TankCircuit;
    }
    if (value == "custom") {
        return StructuredMapPreset::Custom;
    }
    return StructuredMapPreset::ValidationRoad;
}

WorldMap make_world_from_options(const AppOptions& options) {
    if (!options.world_file.empty()) {
        std::ifstream in(options.world_file, std::ios::binary);
        if (!in.is_open()) {
            throw std::runtime_error("could not open world file: " + options.world_file);
        }
        std::vector<std::uint8_t> blob(
            (std::istreambuf_iterator<char>(in)),
            std::istreambuf_iterator<char>());
        WorldMap world;
        if (!thesis_sim::deserialize_world_blob(blob, &world)) {
            throw std::runtime_error("could not decode world file: " + options.world_file);
        }
        return world;
    }
    if (options.environment_mode == EnvironmentMode::StructuredRoad) {
        return fit_structured_hardware_world(WorldMap::structured_demo(options.structured_preset));
    }
    if (options.environment_mode == EnvironmentMode::MixedRoadGates) {
        return WorldMap::mixed_hardware_demo();
    }
    return WorldMap::unstructured_demo(options.unstructured_preset, GateBehaviorMode::Static, 0);
}

Vec2 lidar_origin_world(const Vec2& base_position,
                        double base_yaw,
                        const thesis_sim::LidarLocalizationConfig& cfg) {
    const Vec2 local_offset{cfg.lidar_x_offset, cfg.lidar_y_offset};
    const Vec2 rotated = thesis_sim::rotate(local_offset, base_yaw);
    return {base_position.x + rotated.x, base_position.y + rotated.y};
}

void print_usage(const char* argv0) {
    std::cout
        << "Usage: " << argv0
        << " --controller-port /dev/ttyACM0 [--lidar-port /dev/ttyUSB0] [options]\n"
        << "Or:    " << argv0
        << " --simulate [options]\n"
        << "Options:\n"
        << "  --controller-baudrate N   default 115200\n"
        << "  --lidar-baudrate N        default 115200\n"
        << "  --max-steps N             default 1500\n"
        << "  --dt SEC                  default 0.10\n"
        << "  --simulate                run the selected hardware scenario against synthetic sensors\n"
        << "  --scenario MODE           structured | unstructured | mixed\n"
        << "  --structured-map NAME     validation | circle | zigzag | hardware_track | figure_eight | tank_circuit\n"
        << "  --unstructured-map NAME   robot_validation | tight | slalom | lower | hardware_lab\n"
        << "  --world-file PATH         load a custom exported `.thmap` world file\n"
        << "  --stream-host HOST        send live view snapshots to HOST\n"
        << "  --stream-port N           send live view snapshots to TCP port N\n"
        << "  --stream-every N          send one frame every N planner steps (default 1)\n"
        << "  --stop-on-stream-loss     stop the robot if a previously connected GUI stream drops\n"
        << "  --vehicle-model NAME      car | tank (tracked/skid-steer profile)\n"
        << "  --ticks-per-rev N         override encoder ticks per wheel revolution\n"
        << "  --wheel-radius M          override wheel/track effective radius in meters\n"
        << "  --track-width M           override differential track width in meters\n"
        << "  --max-linear-speed MPS    override hardware max linear speed\n"
        << "  --max-yaw-rate RADS       override hardware max yaw rate\n"
        << "  --cruise-speed MPS        override planner cruise speed limit\n"
        << "  --no-auto-mode            do not force AUTONOMOUS mode on connect\n"
        << "  --no-gyro-zero            do not send GYRO_ZERO on connect\n"
        << "  --enable-planner-safety   enable planner-side LiDAR safety stop logic\n"
        << "  --controller-reset-on-connect  pulse DTR/RTS when opening the controller serial port\n";
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
        } else if (arg == "--scenario" && i + 1 < argc) {
            options.environment_mode = parse_environment_mode(argv[++i]);
        } else if (arg.rfind("--scenario=", 0) == 0) {
            options.environment_mode = parse_environment_mode(arg.substr(std::strlen("--scenario=")));
        } else if (arg == "--structured-map" && i + 1 < argc) {
            options.structured_preset = parse_structured_preset(argv[++i]);
        } else if (arg.rfind("--structured-map=", 0) == 0) {
            options.structured_preset = parse_structured_preset(arg.substr(std::strlen("--structured-map=")));
        } else if (arg == "--unstructured-map" && i + 1 < argc) {
            options.unstructured_preset = parse_unstructured_preset(argv[++i]);
        } else if (arg.rfind("--unstructured-map=", 0) == 0) {
            options.unstructured_preset = parse_unstructured_preset(arg.substr(std::strlen("--unstructured-map=")));
        } else if (arg == "--world-file" && i + 1 < argc) {
            options.world_file = argv[++i];
        } else if (arg.rfind("--world-file=", 0) == 0) {
            options.world_file = arg.substr(std::strlen("--world-file="));
        } else if (arg == "--stream-host" && i + 1 < argc) {
            options.stream_host = argv[++i];
        } else if (arg == "--stream-port" && i + 1 < argc) {
            options.stream_port = std::atoi(argv[++i]);
        } else if (arg == "--stream-every" && i + 1 < argc) {
            options.stream_every_n_steps = std::atoi(argv[++i]);
        } else if (arg == "--stop-on-stream-loss") {
            options.stop_on_stream_loss = true;
        } else if (arg == "--vehicle-model" && i + 1 < argc) {
            const std::string value = argv[++i];
            if (value == "tank" || value == "tracked" || value == "tracked_vehicle" ||
                value == "skid" || value == "skid_steer") {
                options.vehicle_profile = VehicleProfile::Tank;
            } else {
                options.vehicle_profile = VehicleProfile::Car;
            }
        } else if (arg.rfind("--vehicle-model=", 0) == 0) {
            const std::string value = arg.substr(std::strlen("--vehicle-model="));
            if (value == "tank" || value == "tracked" || value == "tracked_vehicle" ||
                value == "skid" || value == "skid_steer") {
                options.vehicle_profile = VehicleProfile::Tank;
            } else {
                options.vehicle_profile = VehicleProfile::Car;
            }
        } else if (arg == "--ticks-per-rev" && i + 1 < argc) {
            options.encoder_ticks_per_revolution = std::atoi(argv[++i]);
        } else if (arg == "--wheel-radius" && i + 1 < argc) {
            options.wheel_radius_m = std::atof(argv[++i]);
        } else if (arg == "--track-width" && i + 1 < argc) {
            options.track_width_m = std::atof(argv[++i]);
        } else if (arg == "--max-linear-speed" && i + 1 < argc) {
            options.max_linear_speed_mps = std::atof(argv[++i]);
        } else if (arg == "--max-yaw-rate" && i + 1 < argc) {
            options.max_yaw_rate_rad_s = std::atof(argv[++i]);
        } else if (arg == "--cruise-speed" && i + 1 < argc) {
            options.cruise_speed_limit_mps = std::atof(argv[++i]);
        } else if (arg == "--no-auto-mode") {
            options.auto_mode = false;
        } else if (arg == "--no-gyro-zero") {
            options.gyro_zero = false;
        } else if (arg == "--enable-planner-safety") {
            options.planner_safety_stop_enabled = true;
        } else if (arg == "--controller-reset-on-connect") {
            options.controller_reset_on_connect = true;
        } else if (arg == "--help" || arg == "-h") {
            print_usage(argv[0]);
            std::exit(0);
        }
    }
    options.stream_every_n_steps = std::max(options.stream_every_n_steps, 1);
    return options;
}

bool stream_enabled(const AppOptions& options) {
    return !options.stream_host.empty() && options.stream_port > 0;
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
        config->drive.track_width = 0.18;
        config->drive.wheel_radius = 0.032;
        config->drive.encoder_ticks_per_revolution = 2400;
        config->drive.max_linear_speed = 0.22;
        config->drive.max_yaw_rate = 1.20;
        config->drive.max_curvature = 3.60;
        config->pwm.start_motion_pwm = 120;
        config->pwm.min_effective_pwm = 38;
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
    input.target_yaw_rate = command.target_yaw_rate;
    return input;
}

std::unique_ptr<VehicleDynamicsModel> make_simulated_hardware_plant(const HardwarePlannerRunner& runner) {
    if (runner.config().vehicle_model == VehicleModelKind::TrackedVehicle) {
        return thesis_sim::make_tracked_vehicle_model(runner.geometry());
    }
    return thesis_sim::make_four_wheel_car_model(runner.geometry());
}

int run_simulated(const AppOptions& options,
                  WorldMap world,
                  RealRobotBridge::Options bridge_options,
                  HardwarePlannerConfig base_config,
                  HardwarePlannerConfig planner_config) {
    HardwarePlannerRunner runner(world, std::move(bridge_options), planner_config);
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
    double host_time_s = 0.0;
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
            host_time_s = 0.0;
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
        if (runner.lidar_enabled_for_current_mode()) {
            observation.have_lidar_scan = true;
            observation.lidar_scan = make_lidar_scan(runner.world(), plant_state, planner_config);
            observation.min_lidar_range_m = observation.lidar_scan.empty()
                                                ? 0.0
                                                : observation.lidar_scan.front().distance_m;
            for (const RPLidarA1::ScanPoint& point : observation.lidar_scan) {
                observation.min_lidar_range_m = std::min(observation.min_lidar_range_m, point.distance_m);
            }
        }

        runner.step_with_observation(observation, planner_config.nominal_dt, false);
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
        bridge_options.lidar_port = structured_mode ? std::string{} : options.lidar_port;
        bridge_options.lidar_baudrate = options.lidar_baudrate;
        bridge_options.lidar_timeout_s = 0.10;

        HardwarePlannerConfig planner_config;
        planner_config.nominal_dt = options.dt;
        planner_config.auto_set_autonomous_mode = options.auto_mode;
        planner_config.auto_gyro_zero = options.gyro_zero;
        planner_config.use_encoder_odometry = true;
        planner_config.planner_safety_stop_enabled = options.planner_safety_stop_enabled;
        if (world.environment_mode() == EnvironmentMode::MixedRoadGates) {
            planner_config.cruise_speed_limit = 0.16;
            planner_config.goal_tolerance_m = 0.08;
            planner_config.localization.max_range_m = 1.80;
            planner_config.localization.obstacle_stop_distance_m = 0.18;
            planner_config.gap_extraction.planning_max_range_m = 1.80;
            planner_config.gap_extraction.max_target_distance_m = 0.50;
            planner_config.gap_extraction.min_gap_width_m = 0.28;
            planner_config.gap_extraction.dynamic_bounds_margin_m = 0.08;
            planner_config.gap_extraction.startup_scan_duration_s = 0.0;
            planner_config.gap_extraction.strict_locked_gate_motion = false;
            planner_config.gap_extraction.min_passed_gates_to_complete = 1;
        }
        const HardwarePlannerConfig base_planner_config = planner_config;
        apply_vehicle_profile(options, &planner_config);

        if (options.simulate) {
            return run_simulated(options, world, std::move(bridge_options), base_planner_config, planner_config);
        }

        runner = std::make_unique<HardwarePlannerRunner>(
            world,
            bridge_options,
            planner_config);
        LiveViewStreamClient streamer;
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
            runner->step();
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
