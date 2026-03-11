#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "rplidar_a1.h"
#include "rsp_serial_bridge.h"

namespace thesis_sim {

struct RealRobotObservation {
    double host_timestamp_s = 0.0;
    bool have_controller_telemetry = false;
    ControllerTelemetry controller{};
    bool have_lidar_scan = false;
    std::vector<RPLidarA1::ScanPoint> lidar_scan;
    double min_lidar_range_m = 0.0;
};

class RealRobotBridge {
  public:
    struct Options {
        RSPSerialBridge::Options controller{};
        std::string lidar_port;
        int lidar_baudrate = 115200;
        double lidar_timeout_s = 0.1;
        int lidar_motor_pwm = RPLidarA1::kDefaultMotorPwm;
    };

    RealRobotBridge();
    explicit RealRobotBridge(Options options);

    void connect(bool start_lidar_scan = true);
    void disconnect();

    bool controller_connected() const { return controller_.is_connected(); }
    bool lidar_connected() const { return lidar_.is_connected(); }

    void poll_controller(double timeout_s = 0.0);
    std::vector<RPLidarA1::ScanPoint> read_lidar_scan(int min_points = 60);
    void pump(double controller_timeout_s = 0.0, int lidar_min_points = 60, bool fetch_lidar = true);

    const RealRobotObservation& observation() const { return observation_; }

    RSPSerialBridge& controller() { return controller_; }
    const RSPSerialBridge& controller() const { return controller_; }

    RPLidarA1& lidar() { return lidar_; }
    const RPLidarA1& lidar() const { return lidar_; }

    AckPayload ping(double timeout_s = 0.8) { return controller_.ping(timeout_s); }
    AckPayload set_mode(ControllerMode mode, double timeout_s = 1.0) { return controller_.set_mode(mode, timeout_s); }
    AckPayload gyro_zero(double timeout_s = 4.0) { return controller_.gyro_zero(timeout_s); }
    AckPayload config_set(std::uint8_t param_id, ConfigValueType value_type, std::int64_t value, double timeout_s = 1.0) {
        return controller_.config_set(param_id, value_type, value, timeout_s);
    }

    void send_heartbeat(std::uint16_t host_status = 0, std::optional<std::uint32_t> host_time_ms = std::nullopt) {
        controller_.send_heartbeat(host_status, host_time_ms);
    }
    void send_pwm(std::int16_t pwm_l,
                  std::int16_t pwm_r,
                  bool force = false,
                  MotorControlMode control_mode = MotorControlMode::SafeDirectPwm) {
        controller_.send_pwm(pwm_l, pwm_r, force, control_mode);
    }
    std::optional<AckPayload> stop(StopReason reason = StopReason::UserRequest, bool wait_ack = false, double timeout_s = 0.8) {
        return controller_.stop(reason, wait_ack, timeout_s);
    }

  private:
    void refresh_observation_timestamp();
    void refresh_controller_snapshot();
    void refresh_lidar_snapshot(std::vector<RPLidarA1::ScanPoint> scan);

    Options options_;
    RSPSerialBridge controller_;
    RPLidarA1 lidar_;
    RealRobotObservation observation_{};
};

}  // namespace thesis_sim
