#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "mvc/controller/hardware_io/rplidar_a1.h"
#include "mvc/controller/hardware_io/rsp_serial_bridge.h"

namespace thesis_sim {

struct RealRobotObservation {
    double host_timestamp_s = 0.0;
    bool have_controller_telemetry = false;
    ControllerTelemetry controller{};
    bool have_lidar_scan = false;
    std::vector<RPLidarA1::ScanPoint> lidar_scan;
    double min_lidar_range_m = 0.0;
    double lidar_scan_start_timestamp_s = 0.0;
    double lidar_scan_end_timestamp_s = 0.0;
    double lidar_scan_mid_timestamp_s = 0.0;
    double lidar_scan_duration_s = 0.0;
    double lidar_scan_age_s = 0.0;
    bool lidar_scan_reused = false;
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
    const std::string& last_lidar_error() const { return last_lidar_error_; }
    const std::optional<RPLidarA1::DeviceInfo>& lidar_device_info() const {
        return lidar_device_info_;
    }

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
    void send_wheel_velocity_mps(double left_mps, double right_mps, bool force = false) {
        controller_.send_wheel_velocity_mps(left_mps, right_mps, force);
    }
    std::optional<AckPayload> stop(StopReason reason = StopReason::UserRequest, bool wait_ack = false, double timeout_s = 0.8) {
        return controller_.stop(reason, wait_ack, timeout_s);
    }

  private:
    bool reconnect_lidar(bool log_failures);
    void refresh_observation_timestamp();
    void refresh_controller_snapshot();
    void refresh_lidar_snapshot(std::vector<RPLidarA1::ScanPoint> scan,
                                double scan_start_s = 0.0,
                                double scan_end_s = 0.0,
                                bool reused = false);
    std::vector<RPLidarA1::ScanPoint> recent_lidar_scan_or_empty(double max_age_s) const;

    Options options_;
    RSPSerialBridge controller_;
    RPLidarA1 lidar_;
    std::optional<RPLidarA1::DeviceInfo> lidar_device_info_;
    std::string last_lidar_error_;
    double next_lidar_reconnect_time_s_ = 0.0;
    std::vector<RPLidarA1::ScanPoint> last_good_lidar_scan_;
    double last_good_lidar_scan_time_s_ = 0.0;
    double last_good_lidar_scan_start_time_s_ = 0.0;
    RealRobotObservation observation_{};
};

}  // namespace thesis_sim
