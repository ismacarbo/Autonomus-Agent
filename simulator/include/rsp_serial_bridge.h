#pragma once

#include <cstdint>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#include "rsp_protocol.h"
#include "serial_port.h"

namespace thesis_sim {

class ProtocolResponseError : public std::runtime_error {
  public:
    using std::runtime_error::runtime_error;
};

struct ControllerTelemetry {
    std::uint32_t ms = 0;
    double rx_timestamp_s = 0.0;

    std::uint32_t imu_ms = 0;
    std::uint32_t safety_ms = 0;
    std::uint32_t motor_ms = 0;
    std::uint32_t heartbeat_ms = 0;

    std::int32_t yaw_mrad = 0;
    std::int32_t yaw_rate_mrad_s = 0;
    double yaw_deg = 0.0;
    double yaw_rate_dps = 0.0;
    std::int16_t acc_x_raw = 0;
    std::int16_t acc_y_raw = 0;
    std::int16_t acc_z_raw = 0;
    std::int16_t gyro_z_raw = 0;

    int dist_c_cm = -1;
    std::uint16_t ir_l_raw = 0;
    std::uint16_t ir_r_raw = 0;
    std::uint16_t safety_flags = 0;

    std::int16_t pwm_l = 0;
    std::int16_t pwm_r = 0;
    std::int16_t target_pwm_l = 0;
    std::int16_t target_pwm_r = 0;
    std::uint16_t motor_flags = 0;

    std::uint16_t uptime_s = 0;
    std::uint16_t status_flags = 0;
    std::uint8_t fw_major = 0;
    std::uint8_t fw_minor = 0;
    std::uint16_t error_code = 0;

    std::int32_t ticks_left = 0;
    std::int32_t ticks_right = 0;
    std::uint16_t enc_dt_ms = 0;
    std::uint16_t enc_flags = 0;

    bool have_imu = false;
    bool have_motor = false;
    bool have_safety = false;
    bool have_heartbeat = false;

    bool ready() const { return have_imu && have_motor; }
};

class RSPSerialBridge {
  public:
    struct Options {
        std::string port;
        int baudrate = 115200;
        double timeout_s = 0.05;
        double startup_delay_s = 4.0;
        double heartbeat_interval_s = 0.75;
        std::size_t max_payload = kRspDefaultMaxPayload;
        bool reset_on_connect = true;
        double reset_pulse_s = 0.12;
    };

    RSPSerialBridge();
    explicit RSPSerialBridge(Options options);

    void connect();
    void disconnect();

    bool is_connected() const { return serial_.is_open(); }
    const Options& options() const { return options_; }

    void poll(double timeout_s = 0.0);

    const ControllerTelemetry* telemetry() const;
    const ControllerTelemetry& telemetry_snapshot() const { return telemetry_; }

    const std::optional<ErrorPayload>& last_controller_error() const { return last_controller_error_; }
    const std::map<std::pair<std::uint8_t, std::uint8_t>, AckPayload>& pending_acks() const { return pending_acks_; }
    const std::map<std::pair<std::uint8_t, std::uint8_t>, ErrorPayload>& pending_errors() const { return pending_errors_; }

    void send_ack(std::uint8_t acked_seq,
                  std::uint8_t acked_type,
                  std::uint8_t status = static_cast<std::uint8_t>(AckStatus::AcceptedCompleted),
                  std::uint8_t detail = 0);
    void send_error(std::uint8_t error_code,
                    std::uint8_t related_type,
                    std::uint8_t related_seq,
                    std::uint8_t detail = 0);

    AckPayload ping(double timeout_s = 0.8);
    AckPayload set_mode(ControllerMode mode, double timeout_s = 1.0);
    AckPayload gyro_zero(double timeout_s = 4.0);
    AckPayload config_set(std::uint8_t param_id, ConfigValueType value_type, std::int64_t value, double timeout_s = 1.0);

    void send_heartbeat(std::uint16_t host_status = 0, std::optional<std::uint32_t> host_time_ms = std::nullopt);
    void send_pwm(std::int16_t pwm_l,
                  std::int16_t pwm_r,
                  bool force = false,
                  MotorControlMode control_mode = MotorControlMode::SafeDirectPwm);
    std::optional<AckPayload> stop(StopReason reason = StopReason::UserRequest, bool wait_ack = false, double timeout_s = 0.8);

  private:
    void require_serial() const;
    std::uint8_t next_seq();
    std::uint8_t write_frame(std::uint8_t msg_type,
                             const std::vector<std::uint8_t>& payload = {},
                             std::uint8_t flags = 0,
                             std::optional<std::uint8_t> seq = std::nullopt);
    void refresh_public_telemetry();
    void handle_frame(const Frame& frame);
    void maybe_send_heartbeat();
    AckPayload wait_for_response(std::uint8_t seq, std::uint8_t msg_type, double timeout_s);

    Options options_;
    SerialPort serial_;
    StreamDecoder parser_;
    std::uint8_t tx_seq_ = 0;
    ControllerTelemetry telemetry_{};
    std::optional<ErrorPayload> last_controller_error_;
    std::map<std::pair<std::uint8_t, std::uint8_t>, AckPayload> pending_acks_;
    std::map<std::pair<std::uint8_t, std::uint8_t>, ErrorPayload> pending_errors_;
    std::pair<std::optional<int>, std::optional<int>> last_sent_pwm_;
    double last_send_ts_ = 0.0;
    double last_heartbeat_tx_ = 0.0;
    double last_rx_ts_ = 0.0;
};

}  // namespace thesis_sim
