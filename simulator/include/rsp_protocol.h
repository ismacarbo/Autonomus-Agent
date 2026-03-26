#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <variant>
#include <vector>

namespace thesis_sim {

inline constexpr std::uint8_t kRspSof1 = 0xAA;
inline constexpr std::uint8_t kRspSof2 = 0x55;
inline constexpr std::uint8_t kRspVersion = 0x01;
inline constexpr std::size_t kRspHeaderLen = 6;
inline constexpr std::size_t kRspFrameOverhead = 2 + kRspHeaderLen + 2;
inline constexpr std::size_t kRspDefaultMaxPayload = 48;

enum class MsgType : std::uint8_t {
    Ping = 0x01,
    Ack = 0x02,
    Error = 0x03,
    MotorCmd = 0x10,
    StopCmd = 0x11,
    ModeCmd = 0x12,
    GyroZeroCmd = 0x13,
    ConfigSet = 0x14,
    HeartbeatCmd = 0x15,
    ImuTelemetry = 0x20,
    SafetyTelemetry = 0x21,
    EncoderTelemetry = 0x22,
    MotorState = 0x23,
    HeartbeatState = 0x24,
};

enum class FrameFlag : std::uint8_t {
    AckReq = 0x01,
    AckFrame = 0x02,
    ErrFrame = 0x04,
};

enum class AckStatus : std::uint8_t {
    AcceptedCompleted = 0x00,
    AcceptedPending = 0x01,
    AcceptedAlready = 0x02,
    Rejected = 0x03,
};

enum class ErrorCode : std::uint8_t {
    UnknownMsgType = 0x01,
    InvalidLength = 0x02,
    CrcMismatch = 0x03,
    InvalidValue = 0x04,
    ImuNotReady = 0x05,
    CalibrationBusy = 0x06,
    MotorsDisabled = 0x07,
    EncodersUnavailable = 0x08,
    SensorTimeout = 0x09,
    UnsupportedMode = 0x0A,
    InternalFault = 0x0B,
    Busy = 0x0C,
    NotImplemented = 0x0D,
};

enum class MotorControlMode : std::uint8_t {
    DirectPwm = 0x00,
    SafeDirectPwm = 0x01,
    VelocityReserved = 0x02,
    Reserved = 0x03,
};

enum class StopReason : std::uint8_t {
    UserRequest = 0x00,
    HostTimeout = 0x01,
    ObstacleDetected = 0x02,
    SafetyOverride = 0x03,
    FaultRecovery = 0x04,
    Shutdown = 0x05,
};

enum class ControllerMode : std::uint8_t {
    Idle = 0x00,
    Manual = 0x01,
    Autonomous = 0x02,
    Calibration = 0x03,
    EmergencyStopLatched = 0x04,
};

enum class ConfigValueType : std::uint8_t {
    Uint8 = 0x01,
    Int16 = 0x02,
    Uint16 = 0x03,
    Int32 = 0x04,
    Uint32 = 0x05,
};

enum class ConfigParamId : std::uint8_t {
    CmdTimeoutMs = 0x01,
    ImuTelemetryMs = 0x02,
    SafetyTelemetryMs = 0x03,
    MotorTelemetryMs = 0x04,
    HeartbeatMs = 0x05,
    LegacyIrAlertThreshold = 0x06,
    LegacyFrontAlertCm = 0x07,
    SlewStep = 0x08,
    SafetyBypass = 0x09,
    EncoderTelemetryMs = 0x0A,
    IrAlertThreshold = LegacyIrAlertThreshold,
    FrontAlertCm = LegacyFrontAlertCm,
};

enum class SafetyFlag : std::uint16_t {
    LegacyUltraValid = 1U << 0,
    LegacyIrLeftAlert = 1U << 1,
    LegacyIrRightAlert = 1U << 2,
    LegacyFrontAlert = 1U << 3,
    CmdTimeout = 1U << 4,
    EmergencyStop = 1U << 5,
    UltraValid = LegacyUltraValid,
    IrLeftAlert = LegacyIrLeftAlert,
    IrRightAlert = LegacyIrRightAlert,
    FrontAlert = LegacyFrontAlert,
};

enum class MotorFlag : std::uint16_t {
    Enabled = 1U << 0,
    StbyHigh = 1U << 1,
    CmdTimeout = 1U << 2,
    SlewLimiting = 1U << 3,
    StopRequested = 1U << 4,
};

enum class StatusFlag : std::uint16_t {
    ImuReady = 1U << 0,
    LegacyUltraReady = 1U << 1,
    LegacyIrReady = 1U << 2,
    EncodersReady = 1U << 3,
    MotorsReady = 1U << 4,
    Calibrating = 1U << 5,
    FaultLatched = 1U << 6,
    HostLinkOk = 1U << 7,
    UltraReady = LegacyUltraReady,
    IrReady = LegacyIrReady,
};

enum class EncoderFlag : std::uint16_t {
    LeftValid = 1U << 0,
    RightValid = 1U << 1,
    LeftDirNeg = 1U << 2,
    RightDirNeg = 1U << 3,
    OverflowWarn = 1U << 4,
};

class ProtocolError : public std::runtime_error {
  public:
    using std::runtime_error::runtime_error;
};

class DecodeError : public ProtocolError {
  public:
    using ProtocolError::ProtocolError;
};

struct Frame {
    std::uint8_t version = kRspVersion;
    std::uint8_t msg_type = 0;
    std::uint8_t flags = 0;
    std::uint8_t seq = 0;
    std::vector<std::uint8_t> payload;
    std::uint16_t crc16 = 0;

    bool ack_requested() const;
    bool is_ack() const;
    bool is_error() const;
};

struct AckPayload {
    std::uint8_t acked_seq = 0;
    std::uint8_t acked_type = 0;
    std::uint8_t status = 0;
    std::uint8_t detail = 0;
};

struct ErrorPayload {
    std::uint8_t error_code = 0;
    std::uint8_t related_type = 0;
    std::uint8_t related_seq = 0;
    std::uint8_t detail = 0;
};

struct MotorCmdPayload {
    std::int16_t pwm_left = 0;
    std::int16_t pwm_right = 0;
    std::uint8_t control_mode = 0;
    std::uint8_t reserved = 0;
};

struct StopCmdPayload {
    std::uint8_t reason = 0;
};

struct ModeCmdPayload {
    std::uint8_t mode = 0;
    std::uint8_t reserved = 0;
};

using ConfigValue = std::variant<std::uint8_t, std::int16_t, std::uint16_t, std::int32_t, std::uint32_t, std::vector<std::uint8_t>>;

struct ConfigSetPayload {
    std::uint8_t param_id = 0;
    std::uint8_t value_type = 0;
    ConfigValue value{};
    std::vector<std::uint8_t> raw_value;
};

struct HeartbeatCmdPayload {
    std::uint32_t host_time_ms = 0;
    std::uint16_t host_status = 0;
};

struct ImuTelemetryPayload {
    std::uint32_t mcu_time_ms = 0;
    std::int32_t yaw_mrad = 0;
    std::int32_t yaw_rate_mrad_s = 0;
    std::int16_t acc_x_raw = 0;
    std::int16_t acc_y_raw = 0;
    std::int16_t acc_z_raw = 0;
    std::int16_t gyro_z_raw = 0;
};

struct SafetyTelemetryPayload {
    std::uint32_t mcu_time_ms = 0;
    std::uint16_t ultra_cm = 0;
    std::uint16_t ir_left_raw = 0;
    std::uint16_t ir_right_raw = 0;
    std::uint16_t safety_flags = 0;
};

struct EncoderTelemetryPayload {
    std::uint32_t mcu_time_ms = 0;
    std::int32_t ticks_left = 0;
    std::int32_t ticks_right = 0;
    std::uint16_t dt_ms = 0;
    std::uint16_t enc_flags = 0;
};

struct MotorStatePayload {
    std::uint32_t mcu_time_ms = 0;
    std::int16_t target_pwm_left = 0;
    std::int16_t target_pwm_right = 0;
    std::int16_t current_pwm_left = 0;
    std::int16_t current_pwm_right = 0;
    std::uint16_t motor_flags = 0;
};

struct HeartbeatStatePayload {
    std::uint32_t mcu_time_ms = 0;
    std::uint16_t uptime_s = 0;
    std::uint16_t status_flags = 0;
    std::uint8_t fw_major = 0;
    std::uint8_t fw_minor = 0;
    std::uint16_t error_code = 0;
};

using DecodedPayload = std::variant<
    std::monostate,
    AckPayload,
    ErrorPayload,
    MotorCmdPayload,
    StopCmdPayload,
    ModeCmdPayload,
    ConfigSetPayload,
    HeartbeatCmdPayload,
    ImuTelemetryPayload,
    SafetyTelemetryPayload,
    EncoderTelemetryPayload,
    MotorStatePayload,
    HeartbeatStatePayload>;

struct DecodedMessage {
    Frame frame;
    std::string name;
    DecodedPayload data;
};

std::uint8_t operator|(FrameFlag lhs, FrameFlag rhs);
std::uint8_t operator|(std::uint8_t lhs, FrameFlag rhs);
bool has_flag(std::uint8_t flags, FrameFlag flag);

std::string message_name(std::uint8_t msg_type);
std::optional<std::size_t> expected_payload_length(std::uint8_t msg_type);
bool validate_payload_length(std::uint8_t msg_type, const std::vector<std::uint8_t>& payload);

std::uint16_t crc16_ccitt_false(const std::uint8_t* data, std::size_t size);
std::uint16_t crc16_ccitt_false(const std::vector<std::uint8_t>& data);
std::vector<std::uint8_t> build_frame(std::uint8_t msg_type,
                                      const std::vector<std::uint8_t>& payload = {},
                                      std::uint8_t seq = 0,
                                      std::uint8_t flags = 0);

class StreamDecoder {
  public:
    explicit StreamDecoder(std::size_t max_payload = kRspDefaultMaxPayload);

    std::vector<Frame> feed(const std::uint8_t* data, std::size_t size);
    std::vector<Frame> feed(const std::vector<std::uint8_t>& data);
    void clear();

    std::size_t frames_ok() const { return frames_ok_; }
    std::size_t frames_dropped() const { return frames_dropped_; }
    std::size_t crc_errors() const { return crc_errors_; }

  private:
    std::size_t max_payload_;
    std::vector<std::uint8_t> buffer_;
    std::size_t frames_ok_ = 0;
    std::size_t frames_dropped_ = 0;
    std::size_t crc_errors_ = 0;
};

std::vector<std::uint8_t> encode_ack(std::uint8_t acked_seq,
                                     std::uint8_t acked_type,
                                     std::uint8_t status = static_cast<std::uint8_t>(AckStatus::AcceptedCompleted),
                                     std::uint8_t detail = 0);
AckPayload decode_ack(const std::vector<std::uint8_t>& payload);

std::vector<std::uint8_t> encode_error(std::uint8_t error_code,
                                       std::uint8_t related_type,
                                       std::uint8_t related_seq,
                                       std::uint8_t detail = 0);
ErrorPayload decode_error(const std::vector<std::uint8_t>& payload);

std::vector<std::uint8_t> encode_motor_cmd(std::int16_t pwm_left,
                                           std::int16_t pwm_right,
                                           std::uint8_t control_mode = static_cast<std::uint8_t>(MotorControlMode::SafeDirectPwm),
                                           std::uint8_t reserved = 0);
MotorCmdPayload decode_motor_cmd(const std::vector<std::uint8_t>& payload);

std::vector<std::uint8_t> encode_stop_cmd(std::uint8_t reason);
StopCmdPayload decode_stop_cmd(const std::vector<std::uint8_t>& payload);

std::vector<std::uint8_t> encode_mode_cmd(std::uint8_t mode, std::uint8_t reserved = 0);
ModeCmdPayload decode_mode_cmd(const std::vector<std::uint8_t>& payload);

std::vector<std::uint8_t> encode_gyro_zero_cmd();

std::vector<std::uint8_t> pack_config_value(ConfigValueType value_type, const ConfigValue& value);
ConfigValue unpack_config_value(ConfigValueType value_type, const std::vector<std::uint8_t>& raw_value);
std::vector<std::uint8_t> encode_config_set(std::uint8_t param_id, ConfigValueType value_type, const ConfigValue& value);
ConfigSetPayload decode_config_set(const std::vector<std::uint8_t>& payload);

std::vector<std::uint8_t> encode_heartbeat_cmd(std::uint32_t host_time_ms, std::uint16_t host_status = 0);
HeartbeatCmdPayload decode_heartbeat_cmd(const std::vector<std::uint8_t>& payload);

std::vector<std::uint8_t> encode_imu_telemetry(std::uint32_t mcu_time_ms,
                                               std::int32_t yaw_mrad,
                                               std::int32_t yaw_rate_mrad_s,
                                               std::int16_t acc_x_raw,
                                               std::int16_t acc_y_raw,
                                               std::int16_t acc_z_raw,
                                               std::int16_t gyro_z_raw);
ImuTelemetryPayload decode_imu_telemetry(const std::vector<std::uint8_t>& payload);

std::vector<std::uint8_t> encode_safety_telemetry(std::uint32_t mcu_time_ms,
                                                  std::uint16_t ultra_cm,
                                                  std::uint16_t ir_left_raw,
                                                  std::uint16_t ir_right_raw,
                                                  std::uint16_t safety_flags);
SafetyTelemetryPayload decode_safety_telemetry(const std::vector<std::uint8_t>& payload);

std::vector<std::uint8_t> encode_encoder_telemetry(std::uint32_t mcu_time_ms,
                                                   std::int32_t ticks_left,
                                                   std::int32_t ticks_right,
                                                   std::uint16_t dt_ms,
                                                   std::uint16_t enc_flags);
EncoderTelemetryPayload decode_encoder_telemetry(const std::vector<std::uint8_t>& payload);

std::vector<std::uint8_t> encode_motor_state(std::uint32_t mcu_time_ms,
                                             std::int16_t target_pwm_left,
                                             std::int16_t target_pwm_right,
                                             std::int16_t current_pwm_left,
                                             std::int16_t current_pwm_right,
                                             std::uint16_t motor_flags);
MotorStatePayload decode_motor_state(const std::vector<std::uint8_t>& payload);

std::vector<std::uint8_t> encode_heartbeat_state(std::uint32_t mcu_time_ms,
                                                 std::uint16_t uptime_s,
                                                 std::uint16_t status_flags,
                                                 std::uint8_t fw_major,
                                                 std::uint8_t fw_minor,
                                                 std::uint16_t error_code);
HeartbeatStatePayload decode_heartbeat_state(const std::vector<std::uint8_t>& payload);

DecodedPayload decode_payload(std::uint8_t msg_type, const std::vector<std::uint8_t>& payload);
DecodedMessage decode_frame(const Frame& frame);

}  // namespace thesis_sim
