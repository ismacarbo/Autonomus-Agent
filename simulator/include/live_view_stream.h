#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "hardware_planner_runner.h"

namespace thesis_sim {

struct LiveVehicleState {
    Vec2 position;
    double yaw = 0.0;
    double speed = 0.0;
    double accel = 0.0;
    double curvature = 0.0;
    double steer_angle = 0.0;
    double yaw_rate = 0.0;
    double sideslip = 0.0;
    double left_wheel_speed = 0.0;
    double right_wheel_speed = 0.0;
    double target_speed = 0.0;
    double target_yaw_rate = 0.0;
    double target_steer_angle = 0.0;
    std::int32_t left_encoder_ticks = 0;
    std::int32_t right_encoder_ticks = 0;
    std::int32_t left_encoder_delta = 0;
    std::int32_t right_encoder_delta = 0;
    int left_pwm = 0;
    int right_pwm = 0;
    double encoder_dt_ms = 0.0;
};

struct LiveMpcCommandView {
    double accel_cmd = 0.0;
    double steer_rate_cmd = 0.0;
};

struct LiveGateFrame {
    GateSpec spec;
    bool passed = false;
};

struct LiveSceneSnapshot {
    std::string stream_label;
    WorldMap world;
    VehicleGeometry geometry{};
    bool imu_enabled = true;
    bool lidar_enabled = true;
    std::string localization_mode;
    std::string heading_source;
    std::string range_sensor_name;
    std::string vehicle_model_name;
    std::string tracking_controller_name;
    int active_lidar_beams = 360;
    double active_lidar_fov_rad = 6.28318530717958647692;
    double active_lidar_range = 8.0;
};

struct LiveFrameSnapshot {
    double sim_time = 0.0;
    int step_count = 0;
    bool connected = false;
    bool telemetry_ready = false;
    bool goal_reached = false;
    bool safety_stop_active = false;
    double distance_to_goal = 0.0;
    double min_lidar_distance = 0.0;
    double front_lidar_distance = 0.0;
    double last_j = 0.0;
    double last_r = 0.0;
    double planner_speed_ref = 0.0;
    double tracker_cross_track_error = 0.0;
    double tracker_heading_error_deg = 0.0;
    int chosen_gate_index = -1;
    LiveVehicleState vehicle{};
    Vec2 navigation_position;
    double navigation_yaw = 0.0;
    double navigation_yaw_rate = 0.0;
    double navigation_curvature = 0.0;
    double navigation_speed = 0.0;
    double navigation_accel = 0.0;
    std::vector<LiveGateFrame> gates;
    std::vector<int> visible_gate_indices;
    std::vector<Vec2> trail;
    std::vector<Vec2> planned_trajectory;
    std::vector<LidarHit> lidar_hits;
    bool has_last_mpc_command = false;
    LiveMpcCommandView last_mpc_command{};
    bool has_latest_sample = false;
    HardwareTelemetrySample latest_sample{};
};

LiveSceneSnapshot make_live_scene_snapshot(const HardwarePlannerRunner& runner);
LiveFrameSnapshot make_live_frame_snapshot(const HardwarePlannerRunner& runner);

class LiveViewStreamClient {
  public:
    LiveViewStreamClient() = default;
    ~LiveViewStreamClient();

    bool connect_to(const std::string& host, std::uint16_t port);
    void disconnect();

    bool connected() const { return socket_fd_ >= 0; }
    const std::string& last_error() const { return last_error_; }

    bool send_scene(const LiveSceneSnapshot& scene);
    bool send_frame(const LiveFrameSnapshot& frame);

  private:
    bool send_packet(std::uint16_t type, const std::vector<std::uint8_t>& payload);

    int socket_fd_ = -1;
    std::string last_error_;
};

class LiveViewStreamServer {
  public:
    struct PollResult {
        bool scene_received = false;
        bool frame_received = false;
        std::optional<LiveSceneSnapshot> scene;
        std::optional<LiveFrameSnapshot> frame;
    };

    LiveViewStreamServer() = default;
    ~LiveViewStreamServer();

    bool start(std::uint16_t port);
    void stop();
    PollResult poll();

    bool listening() const { return listen_fd_ >= 0; }
    bool connected() const { return client_fd_ >= 0; }
    std::uint16_t port() const { return port_; }
    const std::string& remote_endpoint() const { return remote_endpoint_; }
    const std::string& last_error() const { return last_error_; }

  private:
    void close_client();
    bool accept_client();
    void read_client_data();
    bool parse_next_packet(PollResult* result);

    int listen_fd_ = -1;
    int client_fd_ = -1;
    std::uint16_t port_ = 0;
    std::string remote_endpoint_;
    std::string last_error_;
    std::vector<std::uint8_t> recv_buffer_;
};

}  // namespace thesis_sim
