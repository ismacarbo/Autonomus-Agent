#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <string>
#include <thread>
#include <vector>

#include "mvc/controller/hardware_io/real_robot_bridge.h"
#include "mvc/model/navigation/state_estimator_ekf.h"
#include "mvc/model/navigation/vehicle_dynamics.h"
#include "mvc/model/world/world.h"
#include "mvc/view/live_stream/live_view_stream.h"

namespace {

using thesis_sim::ControllerMode;
using thesis_sim::ControllerTelemetry;
using thesis_sim::EnvironmentMode;
using thesis_sim::GateBehaviorMode;
using thesis_sim::HardwareTelemetrySample;
using thesis_sim::KinematicBicycleEkf;
using thesis_sim::LidarHit;
using thesis_sim::LiveFrameSnapshot;
using thesis_sim::LiveSceneSnapshot;
using thesis_sim::LiveViewStreamClient;
using thesis_sim::LiveVehicleState;
using thesis_sim::MotorControlMode;
using thesis_sim::Rect;
using thesis_sim::RPLidarA1;
using thesis_sim::RealRobotBridge;
using thesis_sim::RealRobotObservation;
using thesis_sim::StructuredMapPreset;
using thesis_sim::UnstructuredMapPreset;
using thesis_sim::Vec2;
using thesis_sim::VehicleGeometry;
using thesis_sim::WorldMap;

constexpr double kPi = 3.14159265358979323846;

double monotonic_seconds() {
    using clock = std::chrono::steady_clock;
    return std::chrono::duration<double>(clock::now().time_since_epoch()).count();
}

double clamp_value(double value, double lo, double hi) {
    return std::max(lo, std::min(value, hi));
}

double wrap_angle(double angle) {
    while (angle > kPi) {
        angle -= 2.0 * kPi;
    }
    while (angle < -kPi) {
        angle += 2.0 * kPi;
    }
    return angle;
}

double deg_to_rad(double angle_deg) {
    return angle_deg * kPi / 180.0;
}

double rad_to_deg(double angle_rad) {
    return angle_rad * 180.0 / kPi;
}

const char* unstructured_map_cli_name(UnstructuredMapPreset preset) {
    switch (preset) {
        case UnstructuredMapPreset::TightCorridor:
            return "tight";
        case UnstructuredMapPreset::WideSlalom:
            return "slalom";
        case UnstructuredMapPreset::LowerBypass:
            return "lower";
        case UnstructuredMapPreset::HardwareLab:
            return "hardware_lab";
        case UnstructuredMapPreset::Custom:
            return "custom";
        case UnstructuredMapPreset::RobotValidation:
        default:
            return "robot_validation";
    }
}

const char* structured_map_cli_name(StructuredMapPreset preset) {
    switch (preset) {
        case StructuredMapPreset::CircleLoop:
            return "circle";
        case StructuredMapPreset::ZigZag:
            return "zigzag";
        case StructuredMapPreset::HardwareTrack:
            return "hardware_track";
        case StructuredMapPreset::FigureEight:
            return "figure_eight";
        case StructuredMapPreset::TankCircuit:
            return "tank_circuit";
        case StructuredMapPreset::Custom:
            return "custom";
        case StructuredMapPreset::ValidationRoad:
        default:
            return "validation";
    }
}

EnvironmentMode parse_environment_mode(const std::string& value) {
    return value == "unstructured" ? EnvironmentMode::UnstructuredGates
                                   : EnvironmentMode::StructuredRoad;
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

struct Pose2D {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
};

struct DriveConfig {
    double track_width = 0.28;
    double wheel_radius = 0.04;
    std::int32_t encoder_ticks_per_revolution = 360;
    double motor_time_constant_s = 0.18;
    int min_effective_pwm = 55;
    double speed_estimate_per_pwm = 0.0048;
    double body_length = 0.34;
    double body_width = 0.24;
};

struct MappingConfig {
    double resolution_m = 0.05;
    double size_m = 8.0;
    double lidar_max_range_m = 3.5;
    double lidar_min_range_m = 0.12;
    double lidar_x_offset = 0.0;
    double lidar_y_offset = 0.0;
    double lidar_yaw_offset_rad = 0.0;
    int lidar_min_points = 80;
    int lidar_downsample = 3;
    double match_xy_window_m = 0.14;
    double match_xy_step_m = 0.05;
    double match_yaw_window_rad = 0.12;
    double match_yaw_step_rad = 0.05;
    std::size_t max_stream_points = 2600;
};

struct StreamConfig {
    std::string host;
    int port = 0;
    int every_n_steps = 1;
    double world_size_m = 8.0;
};

struct TestOptions {
    std::string controller_port;
    std::string lidar_port;
    int controller_baudrate = 115200;
    int lidar_baudrate = 115200;
    double dt = 0.10;
    bool auto_mode = true;
    bool gyro_zero = true;
    bool simulate = false;
    bool infinite = false;
    EnvironmentMode environment_mode = EnvironmentMode::StructuredRoad;
    UnstructuredMapPreset unstructured_preset = UnstructuredMapPreset::RobotValidation;
    StructuredMapPreset structured_preset = StructuredMapPreset::ValidationRoad;
    std::string world_file;
    int forward_pwm = 150;
    int turn_pwm = 120;
    double forward_seconds = 2.0;
    double turn_seconds = 1.0;
    double stop_seconds = 0.35;
    int loops = 3;
    double obstacle_stop_distance_m = 0.32;
    std::string output_prefix = "hardware_smoke_test";
    DriveConfig drive{};
    MappingConfig map{};
    StreamConfig stream{};
};

struct MotionSegment {
    std::string label;
    double duration_s = 0.0;
    int pwm_left = 0;
    int pwm_right = 0;
};

struct OdomUpdate {
    bool have_encoder_delta = false;
    int left_delta_ticks = 0;
    int right_delta_ticks = 0;
    double left_distance_m = 0.0;
    double right_distance_m = 0.0;
    double distance_m = 0.0;
    double encoder_dt_s = 0.0;
    double odom_speed_mps = 0.0;
    double odom_yaw_rate_rps = 0.0;
};

struct SlamEstimate {
    Pose2D pose;
    double speed = 0.0;
    double accel = 0.0;
    double yaw_rate = 0.0;
    double curvature = 0.0;
    bool localized = false;
};

struct RunStats {
    int cycles = 0;
    int nonzero_command_cycles = 0;
    int pwm_echo_cycles = 0;
    int encoder_motion_cycles = 0;
    int blocked_cycles = 0;
    int lidar_frames = 0;
    int scan_match_updates = 0;
    double total_distance_m = 0.0;
    double total_abs_left_ticks = 0.0;
    double total_abs_right_ticks = 0.0;
    double min_front_distance_m = std::numeric_limits<double>::infinity();
    bool saw_controller_ready = false;
    bool saw_lidar = false;
    bool saw_pwm_echo = false;
    bool saw_encoder_motion = false;
};

Vec2 to_vec2(const Pose2D& pose) {
    return {pose.x, pose.y};
}

Pose2D from_vec2(const Vec2& point, double yaw) {
    return {point.x, point.y, yaw};
}

Vec2 add_vec(const Vec2& a, const Vec2& b) {
    return {a.x + b.x, a.y + b.y};
}

Vec2 lidar_origin_world(const Pose2D& pose, const MappingConfig& config) {
    return add_vec(to_vec2(pose), thesis_sim::rotate({config.lidar_x_offset, config.lidar_y_offset}, pose.yaw));
}

Vec2 scan_point_world(const Pose2D& pose,
                      const RPLidarA1::ScanPoint& point,
                      const MappingConfig& config) {
    const Vec2 point_in_base = thesis_sim::rotate({point.x_m, point.y_m}, config.lidar_yaw_offset_rad);
    const Vec2 local{config.lidar_x_offset + point_in_base.x, config.lidar_y_offset + point_in_base.y};
    return add_vec(to_vec2(pose), thesis_sim::rotate(local, pose.yaw));
}

VehicleGeometry make_vehicle_geometry(const DriveConfig& drive) {
    VehicleGeometry geometry{};
    geometry.track = drive.track_width;
    geometry.body_length = drive.body_length;
    geometry.body_width = drive.body_width;
    geometry.wheel_radius = drive.wheel_radius;
    geometry.encoder_ticks_per_revolution = drive.encoder_ticks_per_revolution;
    return geometry;
}
