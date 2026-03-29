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

#include "live_view_stream.h"
#include "real_robot_bridge.h"
#include "sim_world.h"
#include "state_estimator_ekf.h"
#include "vehicle_dynamics.h"

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

class EkfSlamLocalizer {
  public:
    explicit EkfSlamLocalizer(const DriveConfig& drive)
        : geometry_(make_vehicle_geometry(drive)) {
        ekf_.set_geometry(geometry_);
        sync_estimate();
    }

    void reset(const Pose2D& pose) {
        ekf_.reset({pose.x, pose.y}, pose.yaw);
        yaw_offset_ = 0.0;
        yaw_offset_initialized_ = false;
        encoder_ticks_initialized_ = false;
        last_left_ticks_ = 0;
        last_right_ticks_ = 0;
        have_raw_imu_yaw_ = false;
        last_raw_imu_yaw_ = 0.0;
        sync_estimate();
    }

    OdomUpdate fuse_controller(const ControllerTelemetry& telemetry, double fallback_dt_s) {
        OdomUpdate out{};
        const double imu_yaw = static_cast<double>(telemetry.yaw_mrad) / 1000.0;
        last_raw_imu_yaw_ = imu_yaw;
        have_raw_imu_yaw_ = true;

        if (!yaw_offset_initialized_) {
            yaw_offset_ = wrap_angle(estimate_.pose.yaw - imu_yaw);
            yaw_offset_initialized_ = true;
        }

        const double measured_yaw = wrap_angle(imu_yaw + yaw_offset_);
        const double measured_yaw_rate = static_cast<double>(telemetry.yaw_rate_mrad_s) / 1000.0;

        if (!encoder_ticks_initialized_) {
            last_left_ticks_ = telemetry.ticks_left;
            last_right_ticks_ = telemetry.ticks_right;
            encoder_ticks_initialized_ = true;
            ekf_.update_imu(measured_yaw, measured_yaw_rate);
            sync_estimate();
            return out;
        }

        out.left_delta_ticks = telemetry.ticks_left - last_left_ticks_;
        out.right_delta_ticks = telemetry.ticks_right - last_right_ticks_;
        last_left_ticks_ = telemetry.ticks_left;
        last_right_ticks_ = telemetry.ticks_right;
        out.encoder_dt_s = telemetry.enc_dt_ms > 0
                               ? clamp_value(static_cast<double>(telemetry.enc_dt_ms) / 1000.0, 0.01, 0.30)
                               : std::max(0.01, fallback_dt_s);

        const double ticks_to_distance =
            (2.0 * kPi * geometry_.wheel_radius) /
            static_cast<double>(std::max<std::int32_t>(geometry_.encoder_ticks_per_revolution, 1));
        out.left_distance_m = static_cast<double>(out.left_delta_ticks) * ticks_to_distance;
        out.right_distance_m = static_cast<double>(out.right_delta_ticks) * ticks_to_distance;
        out.distance_m = 0.5 * (out.left_distance_m + out.right_distance_m);
        const double odom_delta_yaw =
            std::abs(geometry_.track) > 1e-6 ? (out.right_distance_m - out.left_distance_m) / geometry_.track : 0.0;
        out.odom_speed_mps = out.encoder_dt_s > 1e-6 ? out.distance_m / out.encoder_dt_s : 0.0;
        out.odom_yaw_rate_rps = out.encoder_dt_s > 1e-6 ? odom_delta_yaw / out.encoder_dt_s : 0.0;
        out.have_encoder_delta = true;

        ekf_.predict(out.encoder_dt_s, out.odom_speed_mps, out.odom_yaw_rate_rps);
        ekf_.update_imu(measured_yaw, measured_yaw_rate);
        sync_estimate();
        return out;
    }

    void fuse_lidar_pose(const Pose2D& pose_measurement, bool include_yaw) {
        ekf_.update_lidar_pose({pose_measurement.x, pose_measurement.y}, pose_measurement.yaw, include_yaw);
        sync_estimate();
        if (have_raw_imu_yaw_) {
            yaw_offset_ = wrap_angle(estimate_.pose.yaw - last_raw_imu_yaw_);
            yaw_offset_initialized_ = true;
        }
    }

    const SlamEstimate& estimate() const {
        return estimate_;
    }

    const VehicleGeometry& geometry() const {
        return geometry_;
    }

  private:
    void sync_estimate() {
        const thesis_sim::EkfState& state = ekf_.state();
        estimate_.pose = {state.position.x, state.position.y, state.yaw};
        estimate_.speed = state.speed;
        estimate_.accel = state.accel;
        estimate_.yaw_rate = state.yaw_rate;
        estimate_.curvature = std::abs(state.speed) > 0.05
                                  ? clamp_value(state.yaw_rate / state.speed,
                                                -geometry_.max_curvature,
                                                geometry_.max_curvature)
                                  : 0.0;
        estimate_.localized = true;
    }

    VehicleGeometry geometry_{};
    KinematicBicycleEkf ekf_{};
    SlamEstimate estimate_{};
    double yaw_offset_ = 0.0;
    bool yaw_offset_initialized_ = false;
    bool encoder_ticks_initialized_ = false;
    std::int32_t last_left_ticks_ = 0;
    std::int32_t last_right_ticks_ = 0;
    bool have_raw_imu_yaw_ = false;
    double last_raw_imu_yaw_ = 0.0;
};

class OccupancyGrid {
  public:
    OccupancyGrid(double size_m, double resolution_m)
        : resolution_m_(std::max(0.02, resolution_m)) {
        width_ = std::max(80, static_cast<int>(std::ceil(std::max(size_m, 2.0) / resolution_m_)));
        height_ = width_;
        if ((width_ % 2) == 0) {
            ++width_;
            ++height_;
        }
        origin_world_ = {-0.5 * static_cast<double>(width_) * resolution_m_,
                         -0.5 * static_cast<double>(height_) * resolution_m_};
        log_odds_.assign(static_cast<std::size_t>(width_ * height_), 0.0F);
    }

    bool ready_for_matching() const {
        return observed_updates_ > 400;
    }

    int observed_updates() const {
        return observed_updates_;
    }

    void integrate_scan(const Pose2D& pose,
                        const std::vector<RPLidarA1::ScanPoint>& scan,
                        const MappingConfig& config) {
        int start_x = 0;
        int start_y = 0;
        if (!world_to_cell(lidar_origin_world(pose, config), &start_x, &start_y)) {
            return;
        }

        const int stride = std::max(config.lidar_downsample, 1);
        for (std::size_t i = 0; i < scan.size(); i += static_cast<std::size_t>(stride)) {
            const RPLidarA1::ScanPoint& point = scan[i];
            if (point.distance_m < config.lidar_min_range_m ||
                point.distance_m > config.lidar_max_range_m) {
                continue;
            }

            int end_x = 0;
            int end_y = 0;
            if (!world_to_cell(scan_point_world(pose, point, config), &end_x, &end_y)) {
                continue;
            }
            trace_ray(start_x, start_y, end_x, end_y);
        }
    }

    double score_scan(const Pose2D& pose,
                      const std::vector<RPLidarA1::ScanPoint>& scan,
                      const MappingConfig& config) const {
        if (!ready_for_matching()) {
            return 0.0;
        }

        double score = 0.0;
        int used = 0;
        const int stride = std::max(config.lidar_downsample, 1);
        for (std::size_t i = 0; i < scan.size(); i += static_cast<std::size_t>(stride)) {
            const RPLidarA1::ScanPoint& point = scan[i];
            if (point.distance_m < config.lidar_min_range_m ||
                point.distance_m > config.lidar_max_range_m) {
                continue;
            }

            int cell_x = 0;
            int cell_y = 0;
            if (!world_to_cell(scan_point_world(pose, point, config), &cell_x, &cell_y)) {
                continue;
            }
            score += static_cast<double>(log_odds_[index(cell_x, cell_y)]);
            ++used;
        }

        if (used < 10) {
            return -std::numeric_limits<double>::infinity();
        }
        return score / static_cast<double>(used);
    }

    std::vector<Vec2> extract_occupied_points(std::size_t max_points) const {
        std::vector<Vec2> occupied;
        occupied.reserve(static_cast<std::size_t>(width_ * height_ / 8));
        for (int y = 0; y < height_; ++y) {
            for (int x = 0; x < width_; ++x) {
                if (log_odds_[index(x, y)] <= 0.85F) {
                    continue;
                }
                occupied.push_back(cell_center(x, y));
            }
        }

        if (occupied.size() <= max_points || max_points == 0U) {
            return occupied;
        }

        std::vector<Vec2> sampled;
        sampled.reserve(max_points);
        const std::size_t stride = (occupied.size() / max_points) + 1U;
        for (std::size_t i = 0; i < occupied.size() && sampled.size() < max_points; i += stride) {
            sampled.push_back(occupied[i]);
        }
        return sampled;
    }

    bool save_pgm(const std::string& path, const std::vector<Pose2D>& trail) const {
        std::vector<std::uint8_t> pixels(static_cast<std::size_t>(width_ * height_), 205U);
        for (int y = 0; y < height_; ++y) {
            for (int x = 0; x < width_; ++x) {
                const float value = log_odds_[index(x, y)];
                std::uint8_t pixel = 205U;
                if (value > 0.75F) {
                    pixel = 20U;
                } else if (value < -0.30F) {
                    pixel = 248U;
                }
                pixels[static_cast<std::size_t>((height_ - 1 - y) * width_ + x)] = pixel;
            }
        }

        for (const Pose2D& pose : trail) {
            int cell_x = 0;
            int cell_y = 0;
            if (!world_to_cell(to_vec2(pose), &cell_x, &cell_y)) {
                continue;
            }
            pixels[static_cast<std::size_t>((height_ - 1 - cell_y) * width_ + cell_x)] = 94U;
        }
        if (!trail.empty()) {
            int cell_x = 0;
            int cell_y = 0;
            if (world_to_cell(to_vec2(trail.back()), &cell_x, &cell_y)) {
                pixels[static_cast<std::size_t>((height_ - 1 - cell_y) * width_ + cell_x)] = 255U;
            }
        }

        std::ofstream out(path, std::ios::binary);
        if (!out.is_open()) {
            return false;
        }
        out << "P5\n" << width_ << ' ' << height_ << "\n255\n";
        out.write(reinterpret_cast<const char*>(pixels.data()), static_cast<std::streamsize>(pixels.size()));
        return out.good();
    }

  private:
    int index(int x, int y) const {
        return y * width_ + x;
    }

    Vec2 cell_center(int x, int y) const {
        return {
            origin_world_.x + (static_cast<double>(x) + 0.5) * resolution_m_,
            origin_world_.y + (static_cast<double>(y) + 0.5) * resolution_m_,
        };
    }

    bool world_to_cell(const Vec2& point, int* x, int* y) const {
        const int cell_x = static_cast<int>(std::floor((point.x - origin_world_.x) / resolution_m_));
        const int cell_y = static_cast<int>(std::floor((point.y - origin_world_.y) / resolution_m_));
        if (cell_x < 0 || cell_x >= width_ || cell_y < 0 || cell_y >= height_) {
            return false;
        }
        if (x != nullptr) {
            *x = cell_x;
        }
        if (y != nullptr) {
            *y = cell_y;
        }
        return true;
    }

    void update_cell(int x, int y, float delta) {
        float& cell = log_odds_[index(x, y)];
        cell = std::clamp(cell + delta, -4.0F, 4.0F);
        ++observed_updates_;
    }

    void trace_ray(int x0, int y0, int x1, int y1) {
        int x = x0;
        int y = y0;
        const int dx = std::abs(x1 - x0);
        const int dy = -std::abs(y1 - y0);
        const int step_x = x0 < x1 ? 1 : -1;
        const int step_y = y0 < y1 ? 1 : -1;
        int err = dx + dy;

        while (true) {
            const bool endpoint = (x == x1 && y == y1);
            update_cell(x, y, endpoint ? 0.72F : -0.10F);
            if (endpoint) {
                break;
            }
            const int twice_err = 2 * err;
            if (twice_err >= dy) {
                err += dy;
                x += step_x;
            }
            if (twice_err <= dx) {
                err += dx;
                y += step_y;
            }
        }
    }

    double resolution_m_ = 0.05;
    int width_ = 0;
    int height_ = 0;
    Vec2 origin_world_{};
    std::vector<float> log_odds_;
    int observed_updates_ = 0;
};

struct SimRobotState {
    Pose2D pose;
    double left_speed_mps = 0.0;
    double right_speed_mps = 0.0;
    double left_ticks_continuous = 0.0;
    double right_ticks_continuous = 0.0;
};

double wheel_speed_from_pwm(int pwm, const DriveConfig& drive) {
    if (std::abs(pwm) < drive.min_effective_pwm) {
        return 0.0;
    }
    const double magnitude =
        static_cast<double>(std::abs(pwm) - drive.min_effective_pwm) * drive.speed_estimate_per_pwm;
    return std::copysign(magnitude, static_cast<double>(pwm));
}

double estimate_target_speed(int pwm_left, int pwm_right, const DriveConfig& drive) {
    return 0.5 * (wheel_speed_from_pwm(pwm_left, drive) + wheel_speed_from_pwm(pwm_right, drive));
}

double estimate_target_yaw_rate(int pwm_left, int pwm_right, const DriveConfig& drive) {
    if (std::abs(drive.track_width) <= 1e-6) {
        return 0.0;
    }
    return (wheel_speed_from_pwm(pwm_right, drive) - wheel_speed_from_pwm(pwm_left, drive)) /
           drive.track_width;
}

std::vector<RPLidarA1::ScanPoint> make_sim_lidar_scan(const WorldMap& world,
                                                      const Pose2D& pose,
                                                      const MappingConfig& config) {
    constexpr int kBeams = 360;
    const Vec2 origin = lidar_origin_world(pose, config);
    const std::vector<LidarHit> hits = world.raycast(
        origin,
        pose.yaw + config.lidar_yaw_offset_rad,
        kBeams,
        2.0 * kPi,
        config.lidar_max_range_m);

    std::vector<RPLidarA1::ScanPoint> scan;
    scan.reserve(hits.size());
    for (std::size_t i = 0; i < hits.size(); ++i) {
        const double alpha = hits.size() > 1
                                 ? static_cast<double>(i) / static_cast<double>(hits.size() - 1)
                                 : 0.0;
        const double angle_deg = -180.0 + alpha * 360.0;
        const double angle_rad = deg_to_rad(angle_deg);
        const double distance_m = hits[i].distance;
        scan.push_back({
            static_cast<std::uint8_t>(distance_m < config.lidar_max_range_m ? 15 : 0),
            angle_deg,
            distance_m * 1000.0,
            distance_m,
            std::cos(angle_rad) * distance_m,
            std::sin(angle_rad) * distance_m,
        });
    }
    return scan;
}

RealRobotObservation make_sim_observation(const WorldMap& world,
                                          SimRobotState* state,
                                          int pwm_left,
                                          int pwm_right,
                                          double dt,
                                          const TestOptions& options,
                                          double host_time_s) {
    const double alpha = 1.0 - std::exp(-dt / std::max(options.drive.motor_time_constant_s, 1e-3));
    state->left_speed_mps += alpha * (wheel_speed_from_pwm(pwm_left, options.drive) - state->left_speed_mps);
    state->right_speed_mps += alpha * (wheel_speed_from_pwm(pwm_right, options.drive) - state->right_speed_mps);

    const double left_distance = state->left_speed_mps * dt;
    const double right_distance = state->right_speed_mps * dt;
    const double delta_yaw =
        std::abs(options.drive.track_width) > 1e-6 ? (right_distance - left_distance) / options.drive.track_width : 0.0;
    const double mid_yaw = wrap_angle(state->pose.yaw + 0.5 * delta_yaw);
    const double distance = 0.5 * (left_distance + right_distance);
    state->pose.x += distance * std::cos(mid_yaw);
    state->pose.y += distance * std::sin(mid_yaw);
    state->pose.yaw = wrap_angle(state->pose.yaw + delta_yaw);

    const double ticks_per_meter =
        static_cast<double>(std::max(options.drive.encoder_ticks_per_revolution, 1)) /
        (2.0 * kPi * std::max(options.drive.wheel_radius, 1e-4));
    state->left_ticks_continuous += left_distance * ticks_per_meter;
    state->right_ticks_continuous += right_distance * ticks_per_meter;

    ControllerTelemetry telemetry{};
    telemetry.ms = static_cast<std::uint32_t>(std::llround(host_time_s * 1000.0));
    telemetry.rx_timestamp_s = host_time_s;
    telemetry.imu_ms = telemetry.ms;
    telemetry.motor_ms = telemetry.ms;
    telemetry.encoder_ms = telemetry.ms;
    telemetry.heartbeat_ms = telemetry.ms;
    telemetry.yaw_mrad = static_cast<std::int32_t>(std::llround(state->pose.yaw * 1000.0));
    telemetry.yaw_rate_mrad_s = static_cast<std::int32_t>(std::llround(delta_yaw / std::max(dt, 1e-3) * 1000.0));
    telemetry.yaw_deg = rad_to_deg(state->pose.yaw);
    telemetry.yaw_rate_dps = rad_to_deg(delta_yaw / std::max(dt, 1e-3));
    telemetry.pwm_l = static_cast<std::int16_t>(pwm_left);
    telemetry.pwm_r = static_cast<std::int16_t>(pwm_right);
    telemetry.target_pwm_l = static_cast<std::int16_t>(pwm_left);
    telemetry.target_pwm_r = static_cast<std::int16_t>(pwm_right);
    telemetry.ticks_left = static_cast<std::int32_t>(std::llround(state->left_ticks_continuous));
    telemetry.ticks_right = static_cast<std::int32_t>(std::llround(state->right_ticks_continuous));
    telemetry.enc_dt_ms = static_cast<std::uint16_t>(std::llround(dt * 1000.0));
    telemetry.have_imu = true;
    telemetry.have_motor = true;
    telemetry.have_encoder = true;
    telemetry.have_heartbeat = true;

    RealRobotObservation observation{};
    observation.host_timestamp_s = host_time_s;
    observation.have_controller_telemetry = true;
    observation.controller = telemetry;
    observation.lidar_scan = make_sim_lidar_scan(world, state->pose, options.map);
    observation.have_lidar_scan = !observation.lidar_scan.empty();
    observation.min_lidar_range_m = 0.0;
    for (const RPLidarA1::ScanPoint& point : observation.lidar_scan) {
        if (point.distance_m < options.map.lidar_min_range_m) {
            continue;
        }
        if (observation.min_lidar_range_m <= 0.0) {
            observation.min_lidar_range_m = point.distance_m;
        } else {
            observation.min_lidar_range_m = std::min(observation.min_lidar_range_m, point.distance_m);
        }
    }
    return observation;
}

double compute_min_scan_range(const std::vector<RPLidarA1::ScanPoint>& scan, const MappingConfig& config) {
    double best = std::numeric_limits<double>::infinity();
    for (const RPLidarA1::ScanPoint& point : scan) {
        if (point.distance_m < config.lidar_min_range_m || point.distance_m > config.lidar_max_range_m) {
            continue;
        }
        best = std::min(best, point.distance_m);
    }
    return std::isfinite(best) ? best : 0.0;
}

double compute_front_distance(const std::vector<RPLidarA1::ScanPoint>& scan, const MappingConfig& config) {
    double best = std::numeric_limits<double>::infinity();
    constexpr double kFrontHalfAngleRad = 0.40;
    for (const RPLidarA1::ScanPoint& point : scan) {
        if (point.distance_m < config.lidar_min_range_m || point.distance_m > config.lidar_max_range_m) {
            continue;
        }
        const double beam_yaw = wrap_angle(deg_to_rad(point.angle_deg) + config.lidar_yaw_offset_rad);
        if (std::abs(beam_yaw) <= kFrontHalfAngleRad) {
            best = std::min(best, point.distance_m);
        }
    }
    return std::isfinite(best) ? best : 0.0;
}

std::vector<LidarHit> build_lidar_hits_world(const Pose2D& pose,
                                             const std::vector<RPLidarA1::ScanPoint>& scan,
                                             const MappingConfig& config) {
    std::vector<LidarHit> hits;
    hits.reserve(scan.size());
    const Vec2 origin = lidar_origin_world(pose, config);
    for (const RPLidarA1::ScanPoint& point : scan) {
        if (point.distance_m < config.lidar_min_range_m || point.distance_m > config.lidar_max_range_m) {
            continue;
        }
        const Vec2 hit = scan_point_world(pose, point, config);
        const double angle_world = std::atan2(hit.y - origin.y, hit.x - origin.x);
        hits.push_back({angle_world, point.distance_m, hit, true});
    }
    return hits;
}

Pose2D match_scan_to_map(const OccupancyGrid& grid,
                         const Pose2D& predicted,
                         const std::vector<RPLidarA1::ScanPoint>& scan,
                         const MappingConfig& config) {
    if (!grid.ready_for_matching() || static_cast<int>(scan.size()) < config.lidar_min_points) {
        return predicted;
    }

    struct SearchWindow {
        double xy_window = 0.0;
        double xy_step = 0.0;
        double yaw_window = 0.0;
        double yaw_step = 0.0;
    };

    const std::vector<SearchWindow> windows{
        {config.match_xy_window_m, config.match_xy_step_m, config.match_yaw_window_rad, config.match_yaw_step_rad},
        {std::max(config.match_xy_step_m, 0.02), std::max(config.match_xy_step_m * 0.5, 0.02),
         std::max(config.match_yaw_step_rad, 0.02), std::max(config.match_yaw_step_rad * 0.5, 0.01)},
    };

    Pose2D best = predicted;
    double best_score = grid.score_scan(best, scan, config);
    for (const SearchWindow& window : windows) {
        const Pose2D seed = best;
        for (double dx = -window.xy_window; dx <= window.xy_window + 1e-9; dx += window.xy_step) {
            for (double dy = -window.xy_window; dy <= window.xy_window + 1e-9; dy += window.xy_step) {
                for (double dyaw = -window.yaw_window; dyaw <= window.yaw_window + 1e-9; dyaw += window.yaw_step) {
                    const Pose2D candidate{seed.x + dx, seed.y + dy, wrap_angle(seed.yaw + dyaw)};
                    double score = grid.score_scan(candidate, scan, config);
                    score -= 35.0 * (dx * dx + dy * dy);
                    score -= 0.90 * std::abs(dyaw);
                    if (score > best_score) {
                        best_score = score;
                        best = candidate;
                    }
                }
            }
        }
    }
    return best;
}

std::vector<MotionSegment> build_motion_script(const TestOptions& options) {
    std::vector<MotionSegment> script;
    script.push_back({"warmup", 1.0, 0, 0});
    for (int i = 0; i < std::max(options.loops, 1); ++i) {
        const std::string lap = std::to_string(i + 1);
        script.push_back({"forward_a_" + lap, options.forward_seconds, options.forward_pwm, options.forward_pwm});
        script.push_back({"pause_a_" + lap, options.stop_seconds, 0, 0});
        script.push_back({"spin_left_" + lap, options.turn_seconds, -options.turn_pwm, options.turn_pwm});
        script.push_back({"pause_b_" + lap, options.stop_seconds, 0, 0});
        script.push_back({"forward_b_" + lap, options.forward_seconds, options.forward_pwm, options.forward_pwm});
        script.push_back({"pause_c_" + lap, options.stop_seconds, 0, 0});
        script.push_back({"spin_right_" + lap, options.turn_seconds, options.turn_pwm, -options.turn_pwm});
        script.push_back({"pause_d_" + lap, options.stop_seconds, 0, 0});
    }
    script.push_back({"final_stop", 1.0, 0, 0});
    return script;
}

double total_script_duration(const std::vector<MotionSegment>& script) {
    double total = 0.0;
    for (const MotionSegment& segment : script) {
        total += std::max(segment.duration_s, 0.0);
    }
    return total;
}

std::size_t phase_index_at_time(const std::vector<MotionSegment>& script, double elapsed_s) {
    double cursor = 0.0;
    for (std::size_t i = 0; i < script.size(); ++i) {
        cursor += std::max(script[i].duration_s, 0.0);
        if (elapsed_s < cursor) {
            return i;
        }
    }
    return script.empty() ? 0U : (script.size() - 1U);
}

std::string classify_result(const RunStats& stats) {
    if (!stats.saw_controller_ready) {
        return "controller_not_ready";
    }
    if (stats.nonzero_command_cycles == 0) {
        return "no_motion_command_sent";
    }
    if (!stats.saw_pwm_echo) {
        return "command_not_echoed";
    }
    if (!stats.saw_encoder_motion) {
        return "pwm_echoed_but_no_encoder_motion";
    }
    if (stats.total_distance_m < 0.15 && stats.nonzero_command_cycles >= 10) {
        return "weak_motion_detected";
    }
    if (stats.saw_lidar && stats.lidar_frames <= 1) {
        return "drive_ok_but_lidar_missing";
    }
    return "low_level_path_alive";
}

bool stream_enabled(const TestOptions& options) {
    return !options.stream.host.empty() && options.stream.port > 0;
}

WorldMap make_selected_world(const TestOptions& options) {
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
        return WorldMap::structured_demo(options.structured_preset);
    }
    return WorldMap::unstructured_demo(options.unstructured_preset, GateBehaviorMode::Static, 0);
}

void translate_world(WorldMap* world, const Vec2& delta) {
    if (world == nullptr) {
        return;
    }

    Rect bounds = world->bounds();
    bounds.min_x += delta.x;
    bounds.max_x += delta.x;
    bounds.min_y += delta.y;
    bounds.max_y += delta.y;
    world->set_bounds(bounds);
    world->set_start({world->start().x + delta.x, world->start().y + delta.y});
    world->set_goal({world->goal().x + delta.x, world->goal().y + delta.y});

    for (Rect& obstacle : world->editable_obstacles()) {
        obstacle.min_x += delta.x;
        obstacle.max_x += delta.x;
        obstacle.min_y += delta.y;
        obstacle.max_y += delta.y;
    }
    for (auto& gate : world->editable_gates()) {
        gate.position = {gate.position.x + delta.x, gate.position.y + delta.y};
        gate.anchor_position = {gate.anchor_position.x + delta.x, gate.anchor_position.y + delta.y};
    }
    for (Vec2& point : world->editable_road_centerline()) {
        point = {point.x + delta.x, point.y + delta.y};
    }
}

WorldMap make_local_mapping_world(const TestOptions& options) {
    WorldMap world = make_selected_world(options);
    translate_world(&world, {-world.start().x, -world.start().y});
    return world;
}

WorldMap make_stream_world(const TestOptions& options) {
    WorldMap world = make_selected_world(options);
    world.editable_obstacles().clear();

    const double size = std::max(options.stream.world_size_m, 2.0);
    const Vec2 anchor{size * 0.5, size * 0.5};
    translate_world(&world, {anchor.x - world.start().x, anchor.y - world.start().y});

    Rect bounds = world.bounds();
    bounds.min_x = std::min(bounds.min_x, anchor.x - size * 0.5);
    bounds.max_x = std::max(bounds.max_x, anchor.x + size * 0.5);
    bounds.min_y = std::min(bounds.min_y, anchor.y - size * 0.5);
    bounds.max_y = std::max(bounds.max_y, anchor.y + size * 0.5);
    world.set_bounds(bounds);
    return world;
}

LiveSceneSnapshot make_smoke_scene(const TestOptions& options,
                                   const VehicleGeometry& geometry,
                                   const WorldMap& world) {
    LiveSceneSnapshot scene;
    scene.stream_label = options.infinite ? "Hardware Online SLAM" : "Hardware SLAM Smoke Test";
    scene.stream_profile = "slam";
    scene.world = world;
    scene.geometry = geometry;
    scene.imu_enabled = true;
    scene.lidar_enabled = options.simulate || !options.lidar_port.empty();
    scene.localization_mode = "Encoder + IMU EKF + LiDAR scan-to-map";
    scene.heading_source = "IMU fused in EKF";
    scene.range_sensor_name = scene.lidar_enabled ? "RPLidar A1" : "LiDAR disabled";
    scene.vehicle_model_name = "Differential-drive smoke test";
    scene.tracking_controller_name = "Direct PWM diagnostics";
    scene.active_lidar_beams = 360;
    scene.active_lidar_fov_rad = 2.0 * kPi;
    scene.active_lidar_range = options.map.lidar_max_range_m;
    return scene;
}

bool setup_stream_client(const TestOptions& options,
                         const LiveSceneSnapshot& scene,
                         LiveViewStreamClient* streamer) {
    if (!stream_enabled(options) || streamer == nullptr) {
        return true;
    }
    if (!streamer->connect_to(options.stream.host, static_cast<std::uint16_t>(options.stream.port))) {
        std::cerr << "live_stream_error=" << streamer->last_error() << '\n';
        return false;
    }
    if (!streamer->send_scene(scene)) {
        std::cerr << "live_stream_error=" << streamer->last_error() << '\n';
        return false;
    }
    return true;
}

void stream_frame_if_due(const TestOptions& options,
                         int step_count,
                         const LiveFrameSnapshot& frame,
                         LiveViewStreamClient* streamer,
                         bool force = false) {
    if (streamer == nullptr || !streamer->connected() || !stream_enabled(options)) {
        return;
    }
    const bool due = (step_count % std::max(options.stream.every_n_steps, 1)) == 0;
    if (!force && !due) {
        return;
    }
    if (!streamer->send_frame(frame)) {
        std::cerr << "live_stream_error=" << streamer->last_error() << '\n';
        streamer->disconnect();
    }
}

Vec2 stream_offset(const WorldMap& world) {
    return world.start();
}

std::vector<Vec2> offset_trail_for_stream(const std::vector<Pose2D>& trail,
                                          const Vec2& offset,
                                          std::size_t max_points) {
    std::vector<Vec2> out;
    if (trail.empty()) {
        return out;
    }
    const std::size_t begin = trail.size() > max_points ? trail.size() - max_points : 0U;
    out.reserve(trail.size() - begin);
    for (std::size_t i = begin; i < trail.size(); ++i) {
        out.push_back({trail[i].x + offset.x, trail[i].y + offset.y});
    }
    return out;
}

std::vector<Vec2> offset_points_for_stream(const std::vector<Vec2>& points,
                                           const Vec2& offset) {
    std::vector<Vec2> out;
    out.reserve(points.size());
    for (const Vec2& point : points) {
        out.push_back({point.x + offset.x, point.y + offset.y});
    }
    return out;
}

std::vector<LidarHit> offset_hits_for_stream(const std::vector<LidarHit>& hits,
                                             const Vec2& offset) {
    std::vector<LidarHit> out;
    out.reserve(hits.size());
    for (const LidarHit& hit : hits) {
        out.push_back({hit.angle, hit.distance, {hit.point.x + offset.x, hit.point.y + offset.y}, hit.hit});
    }
    return out;
}

LiveFrameSnapshot make_smoke_frame(double sim_time,
                                   int step_count,
                                   bool connected,
                                   bool telemetry_ready,
                                   bool safety_stop_active,
                                   double traveled_distance_m,
                                   double min_lidar_distance,
                                   double front_lidar_distance,
                                   const SlamEstimate& estimate,
                                   const WorldMap& stream_world,
                                   const VehicleGeometry& geometry,
                                   const DriveConfig& drive,
                                   const OdomUpdate& odom,
                                   const ControllerTelemetry* telemetry,
                                   int cmd_left,
                                   int cmd_right,
                                   const std::vector<Pose2D>& trail,
                                   const std::vector<LidarHit>& current_scan_hits,
                                   const std::vector<Vec2>& slam_points,
                                   const HardwareTelemetrySample& sample,
                                   const Vec2& offset) {
    LiveFrameSnapshot frame;
    frame.sim_time = sim_time;
    frame.step_count = step_count;
    frame.connected = connected;
    frame.telemetry_ready = telemetry_ready;
    frame.goal_reached = false;
    frame.safety_stop_active = safety_stop_active;
    frame.distance_to_goal = traveled_distance_m;
    frame.min_lidar_distance = min_lidar_distance;
    frame.front_lidar_distance = front_lidar_distance;
    frame.last_j = 0.0;
    frame.last_r = estimate_target_yaw_rate(cmd_left, cmd_right, drive);
    frame.planner_speed_ref = estimate_target_speed(cmd_left, cmd_right, drive);
    frame.tracker_cross_track_error = 0.0;
    frame.tracker_heading_error_deg = 0.0;
    frame.chosen_gate_index = -1;
    frame.navigation_position = {estimate.pose.x + offset.x, estimate.pose.y + offset.y};
    frame.navigation_yaw = estimate.pose.yaw;
    frame.navigation_yaw_rate = estimate.yaw_rate;
    frame.navigation_curvature = estimate.curvature;
    frame.navigation_speed = estimate.speed;
    frame.navigation_accel = estimate.accel;
    frame.trail = offset_trail_for_stream(trail, offset, 1600);
    frame.slam_points = offset_points_for_stream(slam_points, offset);
    frame.lidar_hits = offset_hits_for_stream(current_scan_hits, offset);
    frame.gates.reserve(stream_world.gates().size());
    for (const auto& gate : stream_world.gates()) {
        frame.gates.push_back({gate, false});
    }

    const double half_track = geometry.track * 0.5;
    LiveVehicleState vehicle;
    vehicle.position = frame.navigation_position;
    vehicle.yaw = estimate.pose.yaw;
    vehicle.speed = estimate.speed;
    vehicle.accel = estimate.accel;
    vehicle.curvature = estimate.curvature;
    vehicle.steer_angle = std::clamp(
        std::atan(geometry.wheelbase * estimate.curvature),
        -geometry.max_steer_angle,
        geometry.max_steer_angle);
    vehicle.yaw_rate = estimate.yaw_rate;
    vehicle.left_wheel_speed = estimate.speed - estimate.yaw_rate * half_track;
    vehicle.right_wheel_speed = estimate.speed + estimate.yaw_rate * half_track;
    vehicle.target_speed = estimate_target_speed(cmd_left, cmd_right, drive);
    vehicle.target_yaw_rate = estimate_target_yaw_rate(cmd_left, cmd_right, drive);
    vehicle.target_steer_angle = 0.0;
    vehicle.left_encoder_delta = odom.left_delta_ticks;
    vehicle.right_encoder_delta = odom.right_delta_ticks;
    vehicle.encoder_dt_ms = odom.encoder_dt_s * 1000.0;
    vehicle.left_pwm = telemetry != nullptr ? telemetry->target_pwm_l : cmd_left;
    vehicle.right_pwm = telemetry != nullptr ? telemetry->target_pwm_r : cmd_right;
    if (telemetry != nullptr) {
        vehicle.left_encoder_ticks = telemetry->ticks_left;
        vehicle.right_encoder_ticks = telemetry->ticks_right;
    }
    frame.vehicle = vehicle;
    frame.has_latest_sample = true;
    frame.latest_sample = sample;
    return frame;
}

void print_usage(const char* argv0) {
    std::cout
        << "Usage: " << argv0 << " --controller-port /dev/ttyACM0 --lidar-port /dev/ttyUSB0 [options]\n"
        << "Or:    " << argv0 << " --simulate [options]\n"
        << "Options:\n"
        << "  --controller-baudrate N   default 115200\n"
        << "  --lidar-baudrate N        default 115200\n"
        << "  --dt SEC                  default 0.10\n"
        << "  --forward-pwm N           default 150\n"
        << "  --turn-pwm N              default 120\n"
        << "  --forward-seconds SEC     default 2.0\n"
        << "  --turn-seconds SEC        default 1.0\n"
        << "  --stop-seconds SEC        default 0.35\n"
        << "  --loops N                 default 3\n"
        << "  --obstacle-stop M         default 0.32\n"
        << "  --map-resolution M        default 0.05\n"
        << "  --map-size M              default 8.0\n"
        << "  --max-range M             default 3.5\n"
        << "  --min-range M             default 0.12\n"
        << "  --track-width M           default 0.28\n"
        << "  --wheel-radius M          default 0.04\n"
        << "  --ticks-per-rev N         default 360\n"
        << "  --min-effective-pwm N     default 55\n"
        << "  --speed-per-pwm V         default 0.0048\n"
        << "  --lidar-x M               default 0.0\n"
        << "  --lidar-y M               default 0.0\n"
        << "  --lidar-yaw-deg DEG       default 0.0\n"
        << "  --stream-host HOST        send live viewer updates to HOST\n"
        << "  --stream-port N           send live viewer updates to port N\n"
        << "  --stream-every N          send one live frame every N cycles (default 1)\n"
        << "  --stream-world-size M     default 8.0\n"
        << "  --scenario MODE           structured | unstructured\n"
        << "  --structured-map NAME     validation | circle | zigzag | hardware_track\n"
        << "  --unstructured-map NAME   robot_validation | tight | slalom | lower | hardware_lab\n"
        << "  --world-file PATH         load a custom exported `.thmap` world file\n"
        << "  --infinite                keep the SLAM session online and loop the motion script forever\n"
        << "  --once                    run the script once even when streaming to the GUI\n"
        << "  --output-prefix PATH      default hardware_smoke_test\n"
        << "  --simulate                run the smoke test against a synthetic compact track\n"
        << "  --no-auto-mode            do not force AUTONOMOUS mode on connect\n"
        << "  --no-gyro-zero            do not send GYRO_ZERO on connect\n";
}

TestOptions parse_args(int argc, char** argv) {
    TestOptions options;
    bool finite_requested = false;
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
        } else if (arg == "--dt" && i + 1 < argc) {
            options.dt = std::atof(argv[++i]);
        } else if (arg == "--forward-pwm" && i + 1 < argc) {
            options.forward_pwm = std::atoi(argv[++i]);
        } else if (arg == "--turn-pwm" && i + 1 < argc) {
            options.turn_pwm = std::atoi(argv[++i]);
        } else if (arg == "--forward-seconds" && i + 1 < argc) {
            options.forward_seconds = std::atof(argv[++i]);
        } else if (arg == "--turn-seconds" && i + 1 < argc) {
            options.turn_seconds = std::atof(argv[++i]);
        } else if (arg == "--stop-seconds" && i + 1 < argc) {
            options.stop_seconds = std::atof(argv[++i]);
        } else if (arg == "--loops" && i + 1 < argc) {
            options.loops = std::atoi(argv[++i]);
        } else if (arg == "--obstacle-stop" && i + 1 < argc) {
            options.obstacle_stop_distance_m = std::atof(argv[++i]);
        } else if (arg == "--map-resolution" && i + 1 < argc) {
            options.map.resolution_m = std::atof(argv[++i]);
        } else if (arg == "--map-size" && i + 1 < argc) {
            options.map.size_m = std::atof(argv[++i]);
        } else if (arg == "--max-range" && i + 1 < argc) {
            options.map.lidar_max_range_m = std::atof(argv[++i]);
        } else if (arg == "--min-range" && i + 1 < argc) {
            options.map.lidar_min_range_m = std::atof(argv[++i]);
        } else if (arg == "--track-width" && i + 1 < argc) {
            options.drive.track_width = std::atof(argv[++i]);
        } else if (arg == "--wheel-radius" && i + 1 < argc) {
            options.drive.wheel_radius = std::atof(argv[++i]);
        } else if (arg == "--ticks-per-rev" && i + 1 < argc) {
            options.drive.encoder_ticks_per_revolution = std::atoi(argv[++i]);
        } else if (arg == "--min-effective-pwm" && i + 1 < argc) {
            options.drive.min_effective_pwm = std::atoi(argv[++i]);
        } else if (arg == "--speed-per-pwm" && i + 1 < argc) {
            options.drive.speed_estimate_per_pwm = std::atof(argv[++i]);
        } else if (arg == "--lidar-x" && i + 1 < argc) {
            options.map.lidar_x_offset = std::atof(argv[++i]);
        } else if (arg == "--lidar-y" && i + 1 < argc) {
            options.map.lidar_y_offset = std::atof(argv[++i]);
        } else if (arg == "--lidar-yaw-deg" && i + 1 < argc) {
            options.map.lidar_yaw_offset_rad = deg_to_rad(std::atof(argv[++i]));
        } else if (arg == "--stream-host" && i + 1 < argc) {
            options.stream.host = argv[++i];
        } else if (arg == "--stream-port" && i + 1 < argc) {
            options.stream.port = std::atoi(argv[++i]);
        } else if (arg == "--stream-every" && i + 1 < argc) {
            options.stream.every_n_steps = std::atoi(argv[++i]);
        } else if (arg == "--stream-world-size" && i + 1 < argc) {
            options.stream.world_size_m = std::atof(argv[++i]);
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
        } else if (arg == "--infinite") {
            options.infinite = true;
        } else if (arg == "--once" || arg == "--finite") {
            options.infinite = false;
            finite_requested = true;
        } else if (arg == "--output-prefix" && i + 1 < argc) {
            options.output_prefix = argv[++i];
        } else if (arg == "--simulate") {
            options.simulate = true;
        } else if (arg == "--no-auto-mode") {
            options.auto_mode = false;
        } else if (arg == "--no-gyro-zero") {
            options.gyro_zero = false;
        } else if (arg == "--help" || arg == "-h") {
            print_usage(argv[0]);
            std::exit(0);
        }
    }

    options.dt = std::max(options.dt, 0.03);
    options.loops = std::max(options.loops, 1);
    options.forward_pwm = std::clamp(options.forward_pwm, -255, 255);
    options.turn_pwm = std::clamp(options.turn_pwm, -255, 255);
    options.map.lidar_min_points = std::max(options.map.lidar_min_points, 20);
    options.map.lidar_downsample = std::max(options.map.lidar_downsample, 1);
    options.drive.encoder_ticks_per_revolution =
        std::max(options.drive.encoder_ticks_per_revolution, 1);
    options.stream.every_n_steps = std::max(options.stream.every_n_steps, 1);
    options.stream.world_size_m = std::max(options.stream.world_size_m, 2.0);
    if (stream_enabled(options) && !finite_requested) {
        options.infinite = true;
    }
    return options;
}

void write_trail_csv(const std::string& path, const std::vector<Pose2D>& trail) {
    std::ofstream out(path);
    if (!out.is_open()) {
        return;
    }
    out << "index,x_m,y_m,yaw_deg\n";
    for (std::size_t i = 0; i < trail.size(); ++i) {
        out << i << ',' << trail[i].x << ',' << trail[i].y << ',' << rad_to_deg(trail[i].yaw) << '\n';
    }
}

void write_summary(const std::string& path,
                   const RunStats& stats,
                   const Pose2D& final_pose,
                   const ControllerTelemetry& last_telemetry,
                   bool have_last_telemetry,
                   std::size_t streamed_map_points) {
    std::ofstream out(path);
    if (!out.is_open()) {
        return;
    }
    out << "result=" << classify_result(stats) << '\n';
    out << "cycles=" << stats.cycles << '\n';
    out << "nonzero_command_cycles=" << stats.nonzero_command_cycles << '\n';
    out << "pwm_echo_cycles=" << stats.pwm_echo_cycles << '\n';
    out << "encoder_motion_cycles=" << stats.encoder_motion_cycles << '\n';
    out << "blocked_cycles=" << stats.blocked_cycles << '\n';
    out << "lidar_frames=" << stats.lidar_frames << '\n';
    out << "scan_match_updates=" << stats.scan_match_updates << '\n';
    out << "total_distance_m=" << stats.total_distance_m << '\n';
    out << "total_abs_left_ticks=" << stats.total_abs_left_ticks << '\n';
    out << "total_abs_right_ticks=" << stats.total_abs_right_ticks << '\n';
    out << "min_front_distance_m="
        << (std::isfinite(stats.min_front_distance_m) ? stats.min_front_distance_m : 0.0) << '\n';
    out << "final_x_m=" << final_pose.x << '\n';
    out << "final_y_m=" << final_pose.y << '\n';
    out << "final_yaw_deg=" << rad_to_deg(final_pose.yaw) << '\n';
    out << "streamed_map_points=" << streamed_map_points << '\n';
    if (have_last_telemetry) {
        out << "last_status_flags=" << last_telemetry.status_flags << '\n';
        out << "last_motor_flags=" << last_telemetry.motor_flags << '\n';
        out << "last_safety_flags=" << last_telemetry.safety_flags << '\n';
        out << "last_error_code=" << last_telemetry.error_code << '\n';
        out << "last_pwm_left=" << last_telemetry.pwm_l << '\n';
        out << "last_pwm_right=" << last_telemetry.pwm_r << '\n';
    }
}

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
