#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <fstream>
#include <filesystem>
#include <iostream>
#include <iterator>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "mvc/controller/hardware_io/external_pose_receiver.h"
#include "mvc/controller/hardware_planner/runner.h"
#include "mvc/controller/slam/slam_toolbox_bridge.h"
#include "mvc/model/navigation/robot_calibration_profile.h"
#include "mvc/view/live_stream/live_view_stream.h"

namespace {

using thesis_sim::ControllerTelemetry;
using thesis_sim::EnvironmentMode;
using thesis_sim::ExternalPoseReference;
using thesis_sim::ExternalPoseSample;
using thesis_sim::ExternalPoseTransform;
using thesis_sim::ExternalPoseUdpReceiver;
using thesis_sim::GateSpec;
using thesis_sim::GateBehaviorMode;
using thesis_sim::HardwarePlannerConfig;
using thesis_sim::HardwarePlannerReport;
using thesis_sim::HardwarePlannerRunner;
using thesis_sim::HardwareLocalizationPolicy;
using thesis_sim::LidarHit;
using thesis_sim::LiveViewStreamClient;
using thesis_sim::MotorControlMode;
using thesis_sim::mvc::model::RobotCalibrationProfile;
using thesis_sim::mvc::model::load_robot_calibration_profile;
using thesis_sim::RealRobotBridge;
using thesis_sim::RealRobotObservation;
using thesis_sim::RPLidarA1;
using thesis_sim::SlamToolboxBridgeClient;
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
    bool mcu_velocity_closed_loop = false;
    bool simulate = false;
    bool stop_on_stream_loss = false;
    VehicleProfile vehicle_profile = VehicleProfile::Car;
    int encoder_ticks_per_revolution = 0;
    double wheel_radius_m = 0.0;
    double track_width_m = 0.0;
    double max_linear_speed_mps = 0.0;
    double max_yaw_rate_rad_s = 0.0;
    double cruise_speed_limit_mps = 0.0;
    HardwareLocalizationPolicy localization_policy = HardwareLocalizationPolicy::Auto;
    EnvironmentMode environment_mode = EnvironmentMode::StructuredRoad;
    UnstructuredMapPreset unstructured_preset = UnstructuredMapPreset::HardwareLab;
    StructuredMapPreset structured_preset = StructuredMapPreset::ValidationRoad;
    int mixed_preset = 1;
    std::string world_file;
    std::string stream_host;
    int stream_port = 0;
    int stream_every_n_steps = 1;
    int external_pose_udp_port = 0;
    ExternalPoseTransform external_pose_transform{};
    double external_pose_timeout_s = 0.25;
    std::string slam_bridge_host = "127.0.0.1";
    int slam_bridge_port = 0;
    bool slam_pose_feedback = false;
    std::string calibration_profile_path;
};

bool parse_external_pose_transform(const std::string& value,
                                   ExternalPoseTransform* transform) {
    if (transform == nullptr) {
        return false;
    }
    double tx = 0.0;
    double ty = 0.0;
    double yaw_deg = 0.0;
    double scale = 1.0;
    if (std::sscanf(value.c_str(), "%lf,%lf,%lf,%lf", &tx, &ty, &yaw_deg, &scale) != 4 ||
        !std::isfinite(tx) || !std::isfinite(ty) || !std::isfinite(yaw_deg) ||
        !std::isfinite(scale) || scale <= 0.0) {
        return false;
    }
    transform->translation_x_m = tx;
    transform->translation_y_m = ty;
    transform->yaw_rad = yaw_deg * kPi / 180.0;
    transform->scale = scale;
    return true;
}

double clamp_value(double value, double lo, double hi) {
    return std::max(lo, std::min(value, hi));
}

double world_span_m(const WorldMap& world) {
    const Rect& bounds = world.bounds();
    return std::max(bounds.max_x - bounds.min_x, bounds.max_y - bounds.min_y);
}

void apply_measured_hardware_geometry(VehicleModelKind model,
                                      thesis_sim::DifferentialDriveGeometry* drive) {
    if (drive == nullptr) {
        return;
    }
    if (model == VehicleModelKind::TrackedVehicle) {
        drive->wheelbase = thesis_sim::measured_robot::kTankBeltLengthM;
        drive->cg_to_front = 0.5 * thesis_sim::measured_robot::kTankBodyLengthM;
        drive->cg_to_rear = 0.5 * thesis_sim::measured_robot::kTankBodyLengthM;
        drive->track_width = thesis_sim::measured_robot::kTankTrackCenterDistanceM;
        drive->body_length = thesis_sim::measured_robot::kTankBodyLengthM;
        drive->body_width = thesis_sim::measured_robot::kTankBodyWidthM;
        drive->wheel_or_belt_length = thesis_sim::measured_robot::kTankBeltLengthM;
        drive->wheel_or_belt_width = thesis_sim::measured_robot::kTankBeltWidthM;
        drive->wheel_radius = thesis_sim::measured_robot::kTankEffectiveDriveRadiusDefaultM;
        return;
    }
    drive->wheelbase = 0.18;
    drive->cg_to_front = 0.09;
    drive->cg_to_rear = 0.09;
    drive->track_width = 0.13;
    drive->body_length = thesis_sim::measured_robot::kCarBodyLengthM;
    drive->body_width = thesis_sim::measured_robot::kCarBodyWidthM;
    drive->wheel_or_belt_length = thesis_sim::measured_robot::kCarWheelDiameterM;
    drive->wheel_or_belt_width = 0.020;
    drive->wheel_radius = thesis_sim::measured_robot::kCarWheelRadiusM;
}

bool compact_hardware_mixed_lab_world(const WorldMap& world) {
    return world.environment_mode() == EnvironmentMode::MixedRoadGates &&
           world.unstructured_preset() == UnstructuredMapPreset::HardwareLab &&
           world_span_m(world) <= 1.25 &&
           !world.road_centerline().empty();
}

void apply_structured_hardware_config(const WorldMap& world,
                                      HardwarePlannerConfig* config) {
    if (config == nullptr ||
        world.environment_mode() != EnvironmentMode::StructuredRoad ||
        world_span_m(world) > 1.25 ||
        world.road_centerline().empty()) {
        return;
    }

    // The normalized validation arena is 1.20 x 1.20 m and both physical
    // platforms occupy a 0.25 x 0.15 m footprint. Keep the LiDAR stop margin
    // relative to that footprint instead of inheriting the large-map default.
    apply_measured_hardware_geometry(config->vehicle_model, &config->drive);
    config->drive.max_curvature =
        config->vehicle_model == VehicleModelKind::TrackedVehicle ? 4.80 : 4.50;
    config->drive.max_steer_angle = 0.70;
    config->cruise_speed_limit =
        config->vehicle_model == VehicleModelKind::TrackedVehicle ? 0.075 : 0.060;
    config->goal_tolerance_m = 0.080;
    config->goal_stop_speed_mps = 0.075;
    config->localization.max_range_m = 1.80;
    // The LiDAR origin is 5 cm behind the front edge of the body; a 13 cm
    // return still leaves about 8 cm of physical clearance in this mode.
    config->localization.obstacle_stop_distance_m = 0.13;
}

void apply_mixed_hardware_config(const WorldMap& world, HardwarePlannerConfig* config) {
    if (config == nullptr ||
        world.environment_mode() != EnvironmentMode::MixedRoadGates) {
        return;
    }

    config->cruise_speed_limit = 0.16;
    config->goal_tolerance_m = 0.08;
    apply_measured_hardware_geometry(config->vehicle_model, &config->drive);
    config->localization.max_range_m = 1.80;
    config->localization.obstacle_stop_distance_m = 0.18;
    config->gap_extraction.planning_max_range_m = 1.80;
    config->gap_extraction.max_target_distance_m = 0.50;
    config->gap_extraction.min_gap_width_m = 0.28;
    config->gap_extraction.dynamic_bounds_margin_m = 0.08;
    config->gap_extraction.startup_scan_duration_s = 0.0;
    config->gap_extraction.strict_locked_gate_motion = false;
    config->gap_extraction.min_passed_gates_to_complete = 1;

    if (!compact_hardware_mixed_lab_world(world)) {
        return;
    }

    // Compact mixed missions use one target to clear the obstacle and a
    // second target to certify re-entry into the structured corridor.
    config->gap_extraction.min_passed_gates_to_complete = 2;

    const double span = world_span_m(world);
    config->goal_tolerance_m = clamp_value(0.12 * span, 0.045, 0.070);
    config->localization.max_range_m = 1.80;
    config->localization.obstacle_stop_distance_m = clamp_value(0.34 * span, 0.145, 0.18);
    config->gap_extraction.planning_max_range_m = config->localization.max_range_m;
    config->gap_extraction.free_distance_threshold_m =
        clamp_value(0.36 * span, config->localization.min_valid_range_m + 0.04, 0.20);
    config->gap_extraction.min_gap_width_m = clamp_value(
        config->drive.body_width + 0.06,
        0.19,
        0.24);
    config->gap_extraction.min_gap_depth_contrast_m = clamp_value(0.16 * span, 0.055, 0.095);
    config->gap_extraction.min_target_distance_m =
        clamp_value(0.26 * span, config->localization.min_valid_range_m + 0.005, 0.18);
    config->gap_extraction.max_target_distance_m =
        clamp_value(0.54 * span, config->gap_extraction.min_target_distance_m + 0.18, 0.65);
    config->gap_extraction.gap_aperture_target_margin_m = clamp_value(0.07 * span, 0.025, 0.050);
    config->gap_extraction.target_clearance_radius_m = clamp_value(
        0.5 * config->drive.body_width + 0.02,
        0.075,
        0.105);
    config->gap_extraction.path_clearance_radius_m = clamp_value(
        0.5 * config->drive.body_width + 0.01,
        0.070,
        0.095);
    config->gap_extraction.map_point_resolution_m = clamp_value(0.040 * span, 0.018, 0.025);
    config->gap_extraction.gap_goal_tolerance_m = clamp_value(0.08 * span, 0.035, 0.052);
    config->gap_extraction.gap_goal_acceptance_radius_m = clamp_value(0.12 * span, 0.050, 0.070);
    config->gap_extraction.gap_goal_acceptance_lateral_slack_m = clamp_value(0.08 * span, 0.030, 0.045);
    config->gap_extraction.gap_crossing_margin_m = clamp_value(0.04 * span, 0.012, 0.025);
    config->gap_extraction.locked_gap_grace_frames = 160;
    config->gap_extraction.gap_track_confirm_score = 1.25;
    config->gap_extraction.gap_track_hold_score = 0.58;
    config->gap_extraction.gap_track_hit_gain = 1.05;
    config->gap_extraction.gap_track_miss_decay = 0.24;
    config->gap_extraction.gap_track_min_hits = 1;
    config->gap_extraction.gap_track_max_misses = 18;
}

void apply_tracked_mixed_hardware_config(const WorldMap& world, HardwarePlannerConfig* config) {
    if (config == nullptr ||
        config->vehicle_model != VehicleModelKind::TrackedVehicle ||
        world.environment_mode() != EnvironmentMode::MixedRoadGates) {
        return;
    }

    const double span = world_span_m(world);
    const bool lab_scale = span <= 1.35 && !world.road_centerline().empty();

    config->cruise_speed_limit = std::min(config->cruise_speed_limit, lab_scale ? 0.065 : 0.075);
    config->goal_tolerance_m = lab_scale ? 0.055 : std::min(config->goal_tolerance_m, 0.070);
    config->goal_stop_speed_mps = std::min(config->goal_stop_speed_mps, 0.075);
    config->localization.max_range_m = lab_scale ? 1.05 : std::min(config->localization.max_range_m, 1.30);
    config->localization.obstacle_stop_distance_m = lab_scale ? 0.24 : std::max(config->localization.obstacle_stop_distance_m, 0.22);
    config->gap_extraction.planning_max_range_m = config->localization.max_range_m;
    config->gap_extraction.free_distance_threshold_m = lab_scale ? 0.27 : std::max(config->gap_extraction.free_distance_threshold_m, 0.27);
    config->gap_extraction.min_gap_width_m = lab_scale ? 0.24 : std::min(config->gap_extraction.min_gap_width_m, 0.28);
    config->gap_extraction.min_gap_depth_contrast_m = lab_scale ? 0.070 : std::min(config->gap_extraction.min_gap_depth_contrast_m, 0.095);
    config->gap_extraction.min_target_distance_m = lab_scale ? 0.18 : std::min(config->gap_extraction.min_target_distance_m, 0.22);
    config->gap_extraction.max_target_distance_m = lab_scale ? 0.65 : std::min(config->gap_extraction.max_target_distance_m, 0.65);
    config->gap_extraction.gap_aperture_target_margin_m = lab_scale ? 0.035 : std::min(config->gap_extraction.gap_aperture_target_margin_m, 0.045);
    const double footprint_target_clearance = 0.5 * config->drive.body_width + 0.020;
    const double footprint_path_clearance = 0.5 * config->drive.body_width + 0.010;
    config->gap_extraction.target_clearance_radius_m = std::max(
        config->gap_extraction.target_clearance_radius_m,
        footprint_target_clearance);
    config->gap_extraction.path_clearance_radius_m = std::max(
        config->gap_extraction.path_clearance_radius_m,
        footprint_path_clearance);
    config->gap_extraction.map_point_resolution_m = lab_scale ? 0.022 : std::min(config->gap_extraction.map_point_resolution_m, 0.026);
    config->gap_extraction.gap_goal_tolerance_m = lab_scale ? 0.070 : std::min(config->gap_extraction.gap_goal_tolerance_m, 0.090);
    config->gap_extraction.gap_goal_acceptance_radius_m = lab_scale ? 0.105 : std::min(config->gap_extraction.gap_goal_acceptance_radius_m, 0.14);
    config->gap_extraction.gap_goal_acceptance_lateral_slack_m = lab_scale ? 0.060 : std::min(config->gap_extraction.gap_goal_acceptance_lateral_slack_m, 0.080);
    config->gap_extraction.gap_crossing_margin_m = lab_scale ? 0.018 : std::min(config->gap_extraction.gap_crossing_margin_m, 0.025);
    config->gap_extraction.gap_goal_cruise_speed_mps = std::min(config->gap_extraction.gap_goal_cruise_speed_mps, 0.060);
    config->gap_extraction.locked_gap_grace_frames = std::max(config->gap_extraction.locked_gap_grace_frames, 180);
    config->gap_extraction.gap_track_confirm_score = std::min(config->gap_extraction.gap_track_confirm_score, 1.15);
    config->gap_extraction.gap_track_hold_score = std::min(config->gap_extraction.gap_track_hold_score, 0.55);
    config->gap_extraction.gap_track_hit_gain = std::max(config->gap_extraction.gap_track_hit_gain, 1.05);
    config->gap_extraction.gap_track_miss_decay = std::min(config->gap_extraction.gap_track_miss_decay, 0.24);
    config->gap_extraction.gap_track_min_hits = 1;
    config->gap_extraction.gap_track_max_misses = std::max(config->gap_extraction.gap_track_max_misses, 18);
    config->gap_extraction.min_passed_gates_to_complete = std::max(config->gap_extraction.min_passed_gates_to_complete, 2);
}

WorldMap fit_structured_hardware_world(WorldMap world, VehicleProfile vehicle_profile) {
    (void)vehicle_profile;
    return thesis_sim::normalize_structured_world(std::move(world));
}

double deg_to_rad(double angle_deg) {
    return angle_deg * kPi / 180.0;
}

EnvironmentMode parse_environment_mode(const std::string& value) {
    if (value == "unstructured") {
        return EnvironmentMode::UnstructuredGates;
    }
    if (value == "mixed" || value == "mixed_road_gates" || value == "road_gates" ||
        value == "mixed_obstacle" || value == "mixed-closed-obstacle" ||
        value == "mixed_closed_obstacle") {
        return EnvironmentMode::MixedRoadGates;
    }
    return EnvironmentMode::StructuredRoad;
}

int parse_mixed_preset(const std::string& value) {
    if (value == "dynamic" || value == "dynamic_obstacle" ||
        value == "dynamic-obstacle" || value == "dynamic_obstacle_road" ||
        value == "dynamic-obstacle-road") {
        return 4;
    }
    if (value == "obstacle" || value == "closed_obstacle" ||
        value == "closed-obstacle" || value == "obstacle_road" ||
        value == "obstacle-road" || value == "mixed_obstacle" ||
        value == "mixed_closed_obstacle" || value == "mixed-closed-obstacle") {
        return 3;
    }
    if (value == "hardware_aligned" || value == "hardware-aligned" ||
        value == "hardware_lab" || value == "hardware-lab" || value == "lab") {
        return 1;
    }
    if (value == "ideal" || value == "ideal_hardware_aligned" ||
        value == "ideal-hardware-aligned") {
        return 2;
    }
    if (value == "tank" || value == "tank_lab" || value == "tank-lab" ||
        value == "tracked" || value == "tracked_lab" || value == "tracked-lab") {
        return 5;
    }
    return 1;
}

WorldMap mixed_hardware_world_from_preset(int preset) {
    if (preset == 5) {
        return WorldMap::mixed_tank_hardware_demo();
    }
    if (preset == 4) {
        return WorldMap::mixed_dynamic_obstacle_demo();
    }
    if (preset == 3) {
        return WorldMap::mixed_closed_obstacle_hardware_demo();
    }
    if (preset == 2) {
        return WorldMap::mixed_ideal_demo();
    }
    if (preset == 1) {
        return WorldMap::mixed_hardware_aligned_demo();
    }
    return WorldMap::mixed_hardware_demo();
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

HardwareLocalizationPolicy parse_localization_policy(const std::string& value) {
    if (value == "encoder" || value == "encoder_imu" || value == "odom" ||
        value == "odometry" || value == "perception") {
        return HardwareLocalizationPolicy::EncoderImuPrimary;
    }
    if (value == "lidar" || value == "scan_matching" || value == "scan-match" ||
        value == "pose_correction") {
        return HardwareLocalizationPolicy::LidarPoseCorrection;
    }
    return HardwareLocalizationPolicy::Auto;
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
        return fit_structured_hardware_world(
            WorldMap::structured_demo(options.structured_preset),
            options.vehicle_profile);
    }
    if (options.environment_mode == EnvironmentMode::MixedRoadGates) {
        return mixed_hardware_world_from_preset(options.mixed_preset);
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
        << "  --mixed-map NAME          hardware | hardware_aligned | ideal | obstacle | dynamic | tank\n"
        << "  --world-file PATH         load a custom exported `.thmap` world file\n"
        << "  --stream-host HOST        send live view snapshots to HOST\n"
        << "  --stream-port N           send live view snapshots to TCP port N\n"
        << "  --stream-every N          send one frame every N planner steps (default 1)\n"
        << "  --stop-on-stream-loss     stop the robot if a previously connected GUI stream drops\n"
        << "  --mocap-udp-port N        receive timestamp,x,y,yaw[,quality] UDP packets\n"
        << "  --mocap-transform CSV     map transform tx_m,ty_m,yaw_deg,scale\n"
        << "  --mocap-body-yaw-offset DEG  fixed rigid-body heading offset\n"
        << "  --mocap-timeout SEC       maximum reference age (default 0.25)\n"
        << "  --vehicle-model NAME      car | tank (tracked/skid-steer profile)\n"
        << "  --ticks-per-rev N         override encoder ticks per wheel revolution\n"
        << "  --wheel-radius M          override wheel/track effective radius in meters\n"
        << "  --track-width M           override differential track width in meters\n"
        << "  --max-linear-speed MPS    override hardware max linear speed\n"
        << "  --max-yaw-rate RADS       override hardware max yaw rate\n"
        << "  --cruise-speed MPS        override planner cruise speed limit\n"
        << "  --pose-fusion MODE        auto | encoder | lidar (default auto)\n"
        << "  --no-auto-mode            do not force AUTONOMOUS mode on connect\n"
        << "  --no-gyro-zero            do not send GYRO_ZERO on connect\n"
        << "  --enable-planner-safety   enable planner-side LiDAR safety stop logic\n"
        << "  --controller-reset-on-connect  pulse DTR/RTS when opening the controller serial port\n"
        << "  --mcu-wheel-pid           send wheel velocity targets to firmware PID (requires new firmware)\n"
        << "  --slam-bridge-port N      stream odometry + LiDAR to the SLAM Toolbox UDP bridge\n"
        << "  --slam-bridge-host HOST   bridge host (default 127.0.0.1)\n"
        << "  --slam-pose-feedback      enable bounded SLAM-to-EKF corrections\n"
        << "  --calibration-profile P   override the versioned robot calibration JSON\n";
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
            const std::string value = argv[++i];
            options.environment_mode = parse_environment_mode(value);
            if (options.environment_mode == EnvironmentMode::MixedRoadGates) {
                options.mixed_preset = parse_mixed_preset(value);
            }
        } else if (arg.rfind("--scenario=", 0) == 0) {
            const std::string value = arg.substr(std::strlen("--scenario="));
            options.environment_mode = parse_environment_mode(value);
            if (options.environment_mode == EnvironmentMode::MixedRoadGates) {
                options.mixed_preset = parse_mixed_preset(value);
            }
        } else if (arg == "--structured-map" && i + 1 < argc) {
            options.structured_preset = parse_structured_preset(argv[++i]);
        } else if (arg.rfind("--structured-map=", 0) == 0) {
            options.structured_preset = parse_structured_preset(arg.substr(std::strlen("--structured-map=")));
        } else if (arg == "--unstructured-map" && i + 1 < argc) {
            options.unstructured_preset = parse_unstructured_preset(argv[++i]);
        } else if (arg.rfind("--unstructured-map=", 0) == 0) {
            options.unstructured_preset = parse_unstructured_preset(arg.substr(std::strlen("--unstructured-map=")));
        } else if (arg == "--mixed-map" && i + 1 < argc) {
            options.mixed_preset = parse_mixed_preset(argv[++i]);
        } else if (arg.rfind("--mixed-map=", 0) == 0) {
            options.mixed_preset = parse_mixed_preset(arg.substr(std::strlen("--mixed-map=")));
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
        } else if (arg == "--mocap-udp-port" && i + 1 < argc) {
            options.external_pose_udp_port = std::atoi(argv[++i]);
        } else if (arg == "--mocap-transform" && i + 1 < argc) {
            if (!parse_external_pose_transform(argv[++i], &options.external_pose_transform)) {
                throw std::runtime_error("invalid --mocap-transform; expected tx,ty,yaw_deg,scale");
            }
        } else if ((arg == "--mocap-body-yaw-offset" ||
                    arg == "--mocap-body-yaw-offset-deg") &&
                   i + 1 < argc) {
            options.external_pose_transform.body_yaw_offset_rad =
                std::atof(argv[++i]) * kPi / 180.0;
        } else if (arg == "--mocap-timeout" && i + 1 < argc) {
            options.external_pose_timeout_s = std::atof(argv[++i]);
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
        } else if (arg == "--pose-fusion" && i + 1 < argc) {
            options.localization_policy = parse_localization_policy(argv[++i]);
        } else if (arg.rfind("--pose-fusion=", 0) == 0) {
            options.localization_policy = parse_localization_policy(arg.substr(std::strlen("--pose-fusion=")));
        } else if (arg == "--no-auto-mode") {
            options.auto_mode = false;
        } else if (arg == "--no-gyro-zero") {
            options.gyro_zero = false;
        } else if (arg == "--enable-planner-safety") {
            options.planner_safety_stop_enabled = true;
        } else if (arg == "--controller-reset-on-connect") {
            options.controller_reset_on_connect = true;
        } else if (arg == "--mcu-wheel-pid") {
            options.mcu_velocity_closed_loop = true;
        } else if (arg == "--slam-bridge-port" && i + 1 < argc) {
            options.slam_bridge_port = std::atoi(argv[++i]);
        } else if (arg == "--slam-bridge-host" && i + 1 < argc) {
            options.slam_bridge_host = argv[++i];
        } else if (arg == "--slam-pose-feedback") {
            options.slam_pose_feedback = true;
        } else if (arg == "--calibration-profile" && i + 1 < argc) {
            options.calibration_profile_path = argv[++i];
        } else if (arg == "--help" || arg == "-h") {
            print_usage(argv[0]);
            std::exit(0);
        }
    }
    options.stream_every_n_steps = std::max(options.stream_every_n_steps, 1);
    options.external_pose_udp_port = std::max(options.external_pose_udp_port, 0);
    options.external_pose_timeout_s = std::max(options.external_pose_timeout_s, 0.01);
    options.slam_bridge_port = std::max(options.slam_bridge_port, 0);
    if (options.slam_pose_feedback && options.slam_bridge_port == 0) {
        throw std::runtime_error("--slam-pose-feedback requires --slam-bridge-port");
    }
    return options;
}
