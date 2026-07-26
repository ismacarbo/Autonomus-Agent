#include "mvc/controller/hardware_io/rsp_protocol.h"

#include <algorithm>
#include <array>
#include <iomanip>
#include <sstream>
#include <utility>

namespace thesis_sim {

namespace {

std::uint16_t read_u16_le(const std::vector<std::uint8_t>& data, std::size_t offset) {
    return static_cast<std::uint16_t>(data.at(offset)) |
           (static_cast<std::uint16_t>(data.at(offset + 1)) << 8);
}

std::int16_t read_i16_le(const std::vector<std::uint8_t>& data, std::size_t offset) {
    return static_cast<std::int16_t>(read_u16_le(data, offset));
}

std::uint32_t read_u32_le(const std::vector<std::uint8_t>& data, std::size_t offset) {
    return static_cast<std::uint32_t>(data.at(offset)) |
           (static_cast<std::uint32_t>(data.at(offset + 1)) << 8) |
           (static_cast<std::uint32_t>(data.at(offset + 2)) << 16) |
           (static_cast<std::uint32_t>(data.at(offset + 3)) << 24);
}

std::int32_t read_i32_le(const std::vector<std::uint8_t>& data, std::size_t offset) {
    return static_cast<std::int32_t>(read_u32_le(data, offset));
}

void append_u16_le(std::vector<std::uint8_t>& out, std::uint16_t value) {
    out.push_back(static_cast<std::uint8_t>(value & 0xFFU));
    out.push_back(static_cast<std::uint8_t>((value >> 8) & 0xFFU));
}

void append_i16_le(std::vector<std::uint8_t>& out, std::int16_t value) {
    append_u16_le(out, static_cast<std::uint16_t>(value));
}

void append_u32_le(std::vector<std::uint8_t>& out, std::uint32_t value) {
    out.push_back(static_cast<std::uint8_t>(value & 0xFFU));
    out.push_back(static_cast<std::uint8_t>((value >> 8) & 0xFFU));
    out.push_back(static_cast<std::uint8_t>((value >> 16) & 0xFFU));
    out.push_back(static_cast<std::uint8_t>((value >> 24) & 0xFFU));
}

void append_i32_le(std::vector<std::uint8_t>& out, std::int32_t value) {
    append_u32_le(out, static_cast<std::uint32_t>(value));
}

[[noreturn]] void require_payload_len(std::uint8_t msg_type, const std::vector<std::uint8_t>& payload) {
    const auto expected = expected_payload_length(msg_type);
    if (expected.has_value() && payload.size() != *expected) {
        std::ostringstream oss;
        oss << message_name(msg_type) << " payload length mismatch: expected " << *expected
            << ", got " << payload.size();
        throw DecodeError(oss.str());
    }
    throw DecodeError("Unsupported payload validation request");
}

void check_payload_len(std::uint8_t msg_type, const std::vector<std::uint8_t>& payload) {
    const auto expected = expected_payload_length(msg_type);
    if (expected.has_value() && payload.size() != *expected) {
        std::ostringstream oss;
        oss << message_name(msg_type) << " payload length mismatch: expected " << *expected
            << ", got " << payload.size();
        throw DecodeError(oss.str());
    }
}

std::string unknown_msg_name(std::uint8_t msg_type) {
    std::ostringstream oss;
    oss << "UNKNOWN_0x" << std::uppercase << std::hex << std::setw(2) << std::setfill('0')
        << static_cast<int>(msg_type);
    return oss.str();
}

std::size_t find_sof(const std::vector<std::uint8_t>& buffer) {
    for (std::size_t i = 0; i + 1 < buffer.size(); ++i) {
        if (buffer[i] == kRspSof1 && buffer[i + 1] == kRspSof2) {
            return i;
        }
    }
    return std::string::npos;
}

}  // namespace

bool Frame::ack_requested() const {
    return has_flag(flags, FrameFlag::AckReq);
}

bool Frame::is_ack() const {
    return has_flag(flags, FrameFlag::AckFrame) || msg_type == static_cast<std::uint8_t>(MsgType::Ack);
}

bool Frame::is_error() const {
    return has_flag(flags, FrameFlag::ErrFrame) || msg_type == static_cast<std::uint8_t>(MsgType::Error);
}

std::uint8_t operator|(FrameFlag lhs, FrameFlag rhs) {
    return static_cast<std::uint8_t>(lhs) | static_cast<std::uint8_t>(rhs);
}

std::uint8_t operator|(std::uint8_t lhs, FrameFlag rhs) {
    return lhs | static_cast<std::uint8_t>(rhs);
}

bool has_flag(std::uint8_t flags, FrameFlag flag) {
    return (flags & static_cast<std::uint8_t>(flag)) != 0U;
}

std::string message_name(std::uint8_t msg_type) {
    switch (static_cast<MsgType>(msg_type)) {
        case MsgType::Ping:
            return "PING";
        case MsgType::Ack:
            return "ACK";
        case MsgType::Error:
            return "ERROR";
        case MsgType::MotorCmd:
            return "MOTOR_CMD";
        case MsgType::StopCmd:
            return "STOP_CMD";
        case MsgType::ModeCmd:
            return "MODE_CMD";
        case MsgType::GyroZeroCmd:
            return "GYRO_ZERO_CMD";
        case MsgType::ConfigSet:
            return "CONFIG_SET";
        case MsgType::HeartbeatCmd:
            return "HEARTBEAT_CMD";
        case MsgType::ImuTelemetry:
            return "IMU_TELEMETRY";
        case MsgType::SafetyTelemetry:
            return "SAFETY_TELEMETRY";
        case MsgType::EncoderTelemetry:
            return "ENCODER_TELEMETRY";
        case MsgType::MotorState:
            return "MOTOR_STATE";
        case MsgType::HeartbeatState:
            return "HEARTBEAT_STATE";
        default:
            return unknown_msg_name(msg_type);
    }
}

std::optional<std::size_t> expected_payload_length(std::uint8_t msg_type) {
    switch (static_cast<MsgType>(msg_type)) {
        case MsgType::Ping:
            return 0;
        case MsgType::Ack:
            return 4;
        case MsgType::Error:
            return 4;
        case MsgType::MotorCmd:
            return 6;
        case MsgType::StopCmd:
            return 1;
        case MsgType::ModeCmd:
            return 2;
        case MsgType::GyroZeroCmd:
            return 0;
        case MsgType::ConfigSet:
            return std::nullopt;
        case MsgType::HeartbeatCmd:
            return 6;
        case MsgType::ImuTelemetry:
            return 20;
        case MsgType::SafetyTelemetry:
            return 12;
        case MsgType::EncoderTelemetry:
            return 16;
        case MsgType::MotorState:
            return 14;
        case MsgType::HeartbeatState:
            return 12;
        default:
            return std::nullopt;
    }
}

bool validate_payload_length(std::uint8_t msg_type, const std::vector<std::uint8_t>& payload) {
    const auto expected = expected_payload_length(msg_type);
    return !expected.has_value() || payload.size() == *expected;
}

std::uint16_t crc16_ccitt_false(const std::uint8_t* data, std::size_t size) {
    std::uint16_t crc = 0xFFFFU;
    for (std::size_t i = 0; i < size; ++i) {
        crc ^= static_cast<std::uint16_t>(data[i]) << 8;
        for (int bit = 0; bit < 8; ++bit) {
            if ((crc & 0x8000U) != 0U) {
                crc = static_cast<std::uint16_t>((crc << 1U) ^ 0x1021U);
            } else {
                crc = static_cast<std::uint16_t>(crc << 1U);
            }
        }
    }
    return crc;
}

std::uint16_t crc16_ccitt_false(const std::vector<std::uint8_t>& data) {
    return crc16_ccitt_false(data.data(), data.size());
}

std::vector<std::uint8_t> build_frame(std::uint8_t msg_type,
                                      const std::vector<std::uint8_t>& payload,
                                      std::uint8_t seq,
                                      std::uint8_t flags) {
    std::vector<std::uint8_t> header;
    header.reserve(kRspHeaderLen + payload.size() + 4);
    header.push_back(kRspVersion);
    header.push_back(msg_type);
    header.push_back(flags);
    header.push_back(seq);
    append_u16_le(header, static_cast<std::uint16_t>(payload.size()));
    header.insert(header.end(), payload.begin(), payload.end());

    const std::uint16_t crc = crc16_ccitt_false(header);

    std::vector<std::uint8_t> frame;
    frame.reserve(kRspFrameOverhead + payload.size());
    frame.push_back(kRspSof1);
    frame.push_back(kRspSof2);
    frame.insert(frame.end(), header.begin(), header.end());
    append_u16_le(frame, crc);
    return frame;
}

StreamDecoder::StreamDecoder(std::size_t max_payload)
    : max_payload_(max_payload) {}

std::vector<Frame> StreamDecoder::feed(const std::uint8_t* data, std::size_t size) {
    if (data != nullptr && size > 0) {
        buffer_.insert(buffer_.end(), data, data + size);
    }

    std::vector<Frame> frames;

    while (true) {
        const std::size_t sof_idx = find_sof(buffer_);
        if (sof_idx == std::string::npos) {
            if (!buffer_.empty() && buffer_.back() == kRspSof1) {
                buffer_.erase(buffer_.begin(), buffer_.end() - 1);
            } else {
                buffer_.clear();
            }
            break;
        }

        if (sof_idx > 0) {
            buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(sof_idx));
        }

        if (buffer_.size() < kRspFrameOverhead) {
            break;
        }

        const std::uint8_t version = buffer_[2];
        const std::uint8_t msg_type = buffer_[3];
        const std::uint8_t flags = buffer_[4];
        const std::uint8_t seq = buffer_[5];
        const std::uint16_t length = static_cast<std::uint16_t>(buffer_[6]) |
                                     (static_cast<std::uint16_t>(buffer_[7]) << 8);

        if (version != kRspVersion || length > max_payload_) {
            ++frames_dropped_;
            buffer_.erase(buffer_.begin());
            continue;
        }

        const std::size_t frame_len = 2 + kRspHeaderLen + length + 2;
        if (buffer_.size() < frame_len) {
            break;
        }

        const std::vector<std::uint8_t> body(buffer_.begin() + 2, buffer_.begin() + 2 + kRspHeaderLen + length);
        const std::uint16_t crc_rx = static_cast<std::uint16_t>(buffer_[2 + kRspHeaderLen + length]) |
                                     (static_cast<std::uint16_t>(buffer_[2 + kRspHeaderLen + length + 1]) << 8);
        const std::uint16_t crc_calc = crc16_ccitt_false(body);
        if (crc_rx != crc_calc) {
            ++frames_dropped_;
            ++crc_errors_;
            buffer_.erase(buffer_.begin());
            continue;
        }

        Frame frame;
        frame.version = version;
        frame.msg_type = msg_type;
        frame.flags = flags;
        frame.seq = seq;
        frame.payload.assign(buffer_.begin() + 2 + kRspHeaderLen, buffer_.begin() + 2 + kRspHeaderLen + length);
        frame.crc16 = crc_rx;
        frames.push_back(std::move(frame));
        ++frames_ok_;
        buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(frame_len));
    }

    return frames;
}

std::vector<Frame> StreamDecoder::feed(const std::vector<std::uint8_t>& data) {
    return feed(data.data(), data.size());
}

void StreamDecoder::clear() {
    buffer_.clear();
}

std::vector<std::uint8_t> encode_ack(std::uint8_t acked_seq,
                                     std::uint8_t acked_type,
                                     std::uint8_t status,
                                     std::uint8_t detail) {
    return {acked_seq, acked_type, status, detail};
}

AckPayload decode_ack(const std::vector<std::uint8_t>& payload) {
    check_payload_len(static_cast<std::uint8_t>(MsgType::Ack), payload);
    return {payload[0], payload[1], payload[2], payload[3]};
}

std::vector<std::uint8_t> encode_error(std::uint8_t error_code,
                                       std::uint8_t related_type,
                                       std::uint8_t related_seq,
                                       std::uint8_t detail) {
    return {error_code, related_type, related_seq, detail};
}

ErrorPayload decode_error(const std::vector<std::uint8_t>& payload) {
    check_payload_len(static_cast<std::uint8_t>(MsgType::Error), payload);
    return {payload[0], payload[1], payload[2], payload[3]};
}

std::vector<std::uint8_t> encode_motor_cmd(std::int16_t pwm_left,
                                           std::int16_t pwm_right,
                                           std::uint8_t control_mode,
                                           std::uint8_t reserved) {
    std::vector<std::uint8_t> payload;
    payload.reserve(6);
    append_i16_le(payload, pwm_left);
    append_i16_le(payload, pwm_right);
    payload.push_back(control_mode);
    payload.push_back(reserved);
    return payload;
}

MotorCmdPayload decode_motor_cmd(const std::vector<std::uint8_t>& payload) {
    check_payload_len(static_cast<std::uint8_t>(MsgType::MotorCmd), payload);
    return {
        read_i16_le(payload, 0),
        read_i16_le(payload, 2),
        payload[4],
        payload[5],
    };
}

std::vector<std::uint8_t> encode_stop_cmd(std::uint8_t reason) {
    return {reason};
}

StopCmdPayload decode_stop_cmd(const std::vector<std::uint8_t>& payload) {
    check_payload_len(static_cast<std::uint8_t>(MsgType::StopCmd), payload);
    return {payload[0]};
}

std::vector<std::uint8_t> encode_mode_cmd(std::uint8_t mode, std::uint8_t reserved) {
    return {mode, reserved};
}

ModeCmdPayload decode_mode_cmd(const std::vector<std::uint8_t>& payload) {
    check_payload_len(static_cast<std::uint8_t>(MsgType::ModeCmd), payload);
    return {payload[0], payload[1]};
}

std::vector<std::uint8_t> encode_gyro_zero_cmd() {
    return {};
}

std::vector<std::uint8_t> pack_config_value(ConfigValueType value_type, const ConfigValue& value) {
    std::vector<std::uint8_t> payload;
    switch (value_type) {
        case ConfigValueType::Uint8:
            if (const auto* v = std::get_if<std::uint8_t>(&value)) {
                payload.push_back(*v);
                return payload;
            }
            break;
        case ConfigValueType::Int16:
            if (const auto* v = std::get_if<std::int16_t>(&value)) {
                append_i16_le(payload, *v);
                return payload;
            }
            break;
        case ConfigValueType::Uint16:
            if (const auto* v = std::get_if<std::uint16_t>(&value)) {
                append_u16_le(payload, *v);
                return payload;
            }
            break;
        case ConfigValueType::Int32:
            if (const auto* v = std::get_if<std::int32_t>(&value)) {
                append_i32_le(payload, *v);
                return payload;
            }
            break;
        case ConfigValueType::Uint32:
            if (const auto* v = std::get_if<std::uint32_t>(&value)) {
                append_u32_le(payload, *v);
                return payload;
            }
            break;
    }

    throw DecodeError("Unsupported CONFIG_SET value variant for declared type");
}

ConfigValue unpack_config_value(ConfigValueType value_type, const std::vector<std::uint8_t>& raw_value) {
    switch (value_type) {
        case ConfigValueType::Uint8:
            if (raw_value.size() != 1) {
                throw DecodeError("CONFIG_SET value length mismatch for UINT8");
            }
            return raw_value[0];
        case ConfigValueType::Int16:
            if (raw_value.size() != 2) {
                throw DecodeError("CONFIG_SET value length mismatch for INT16");
            }
            return read_i16_le(raw_value, 0);
        case ConfigValueType::Uint16:
            if (raw_value.size() != 2) {
                throw DecodeError("CONFIG_SET value length mismatch for UINT16");
            }
            return read_u16_le(raw_value, 0);
        case ConfigValueType::Int32:
            if (raw_value.size() != 4) {
                throw DecodeError("CONFIG_SET value length mismatch for INT32");
            }
            return read_i32_le(raw_value, 0);
        case ConfigValueType::Uint32:
            if (raw_value.size() != 4) {
                throw DecodeError("CONFIG_SET value length mismatch for UINT32");
            }
            return read_u32_le(raw_value, 0);
        default:
            return raw_value;
    }
}

std::vector<std::uint8_t> encode_config_set(std::uint8_t param_id, ConfigValueType value_type, const ConfigValue& value) {
    std::vector<std::uint8_t> payload;
    payload.reserve(6);
    payload.push_back(param_id);
    payload.push_back(static_cast<std::uint8_t>(value_type));
    const std::vector<std::uint8_t> encoded_value = pack_config_value(value_type, value);
    payload.insert(payload.end(), encoded_value.begin(), encoded_value.end());
    return payload;
}

ConfigSetPayload decode_config_set(const std::vector<std::uint8_t>& payload) {
    if (payload.size() < 2) {
        throw DecodeError("CONFIG_SET payload too short");
    }
    ConfigSetPayload out;
    out.param_id = payload[0];
    out.value_type = payload[1];
    out.raw_value.assign(payload.begin() + 2, payload.end());
    out.value = unpack_config_value(static_cast<ConfigValueType>(out.value_type), out.raw_value);
    return out;
}

std::vector<std::uint8_t> encode_heartbeat_cmd(std::uint32_t host_time_ms, std::uint16_t host_status) {
    std::vector<std::uint8_t> payload;
    payload.reserve(6);
    append_u32_le(payload, host_time_ms);
    append_u16_le(payload, host_status);
    return payload;
}

HeartbeatCmdPayload decode_heartbeat_cmd(const std::vector<std::uint8_t>& payload) {
    check_payload_len(static_cast<std::uint8_t>(MsgType::HeartbeatCmd), payload);
    return {
        read_u32_le(payload, 0),
        read_u16_le(payload, 4),
    };
}

std::vector<std::uint8_t> encode_imu_telemetry(std::uint32_t mcu_time_ms,
                                               std::int32_t yaw_mrad,
                                               std::int32_t yaw_rate_mrad_s,
                                               std::int16_t acc_x_raw,
                                               std::int16_t acc_y_raw,
                                               std::int16_t acc_z_raw,
                                               std::int16_t gyro_z_raw) {
    std::vector<std::uint8_t> payload;
    payload.reserve(20);
    append_u32_le(payload, mcu_time_ms);
    append_i32_le(payload, yaw_mrad);
    append_i32_le(payload, yaw_rate_mrad_s);
    append_i16_le(payload, acc_x_raw);
    append_i16_le(payload, acc_y_raw);
    append_i16_le(payload, acc_z_raw);
    append_i16_le(payload, gyro_z_raw);
    return payload;
}

ImuTelemetryPayload decode_imu_telemetry(const std::vector<std::uint8_t>& payload) {
    check_payload_len(static_cast<std::uint8_t>(MsgType::ImuTelemetry), payload);
    return {
        read_u32_le(payload, 0),
        read_i32_le(payload, 4),
        read_i32_le(payload, 8),
        read_i16_le(payload, 12),
        read_i16_le(payload, 14),
        read_i16_le(payload, 16),
        read_i16_le(payload, 18),
    };
}

std::vector<std::uint8_t> encode_safety_telemetry(std::uint32_t mcu_time_ms,
                                                  std::uint16_t ultra_cm,
                                                  std::uint16_t ir_left_raw,
                                                  std::uint16_t ir_right_raw,
                                                  std::uint16_t safety_flags) {
    std::vector<std::uint8_t> payload;
    payload.reserve(12);
    append_u32_le(payload, mcu_time_ms);
    append_u16_le(payload, ultra_cm);
    append_u16_le(payload, ir_left_raw);
    append_u16_le(payload, ir_right_raw);
    append_u16_le(payload, safety_flags);
    return payload;
}

SafetyTelemetryPayload decode_safety_telemetry(const std::vector<std::uint8_t>& payload) {
    check_payload_len(static_cast<std::uint8_t>(MsgType::SafetyTelemetry), payload);
    return {
        read_u32_le(payload, 0),
        read_u16_le(payload, 4),
        read_u16_le(payload, 6),
        read_u16_le(payload, 8),
        read_u16_le(payload, 10),
    };
}

std::vector<std::uint8_t> encode_encoder_telemetry(std::uint32_t mcu_time_ms,
                                                   std::int32_t ticks_left,
                                                   std::int32_t ticks_right,
                                                   std::uint16_t dt_ms,
                                                   std::uint16_t enc_flags) {
    std::vector<std::uint8_t> payload;
    payload.reserve(16);
    append_u32_le(payload, mcu_time_ms);
    append_i32_le(payload, ticks_left);
    append_i32_le(payload, ticks_right);
    append_u16_le(payload, dt_ms);
    append_u16_le(payload, enc_flags);
    return payload;
}

EncoderTelemetryPayload decode_encoder_telemetry(const std::vector<std::uint8_t>& payload) {
    check_payload_len(static_cast<std::uint8_t>(MsgType::EncoderTelemetry), payload);
    return {
        read_u32_le(payload, 0),
        read_i32_le(payload, 4),
        read_i32_le(payload, 8),
        read_u16_le(payload, 12),
        read_u16_le(payload, 14),
    };
}

std::vector<std::uint8_t> encode_motor_state(std::uint32_t mcu_time_ms,
                                             std::int16_t target_pwm_left,
                                             std::int16_t target_pwm_right,
                                             std::int16_t current_pwm_left,
                                             std::int16_t current_pwm_right,
                                             std::uint16_t motor_flags) {
    std::vector<std::uint8_t> payload;
    payload.reserve(14);
    append_u32_le(payload, mcu_time_ms);
    append_i16_le(payload, target_pwm_left);
    append_i16_le(payload, target_pwm_right);
    append_i16_le(payload, current_pwm_left);
    append_i16_le(payload, current_pwm_right);
    append_u16_le(payload, motor_flags);
    return payload;
}

MotorStatePayload decode_motor_state(const std::vector<std::uint8_t>& payload) {
    check_payload_len(static_cast<std::uint8_t>(MsgType::MotorState), payload);
    return {
        read_u32_le(payload, 0),
        read_i16_le(payload, 4),
        read_i16_le(payload, 6),
        read_i16_le(payload, 8),
        read_i16_le(payload, 10),
        read_u16_le(payload, 12),
    };
}

std::vector<std::uint8_t> encode_heartbeat_state(std::uint32_t mcu_time_ms,
                                                 std::uint16_t uptime_s,
                                                 std::uint16_t status_flags,
                                                 std::uint8_t fw_major,
                                                 std::uint8_t fw_minor,
                                                 std::uint16_t error_code) {
    std::vector<std::uint8_t> payload;
    payload.reserve(12);
    append_u32_le(payload, mcu_time_ms);
    append_u16_le(payload, uptime_s);
    append_u16_le(payload, status_flags);
    payload.push_back(fw_major);
    payload.push_back(fw_minor);
    append_u16_le(payload, error_code);
    return payload;
}

HeartbeatStatePayload decode_heartbeat_state(const std::vector<std::uint8_t>& payload) {
    check_payload_len(static_cast<std::uint8_t>(MsgType::HeartbeatState), payload);
    return {
        read_u32_le(payload, 0),
        read_u16_le(payload, 4),
        read_u16_le(payload, 6),
        payload[8],
        payload[9],
        read_u16_le(payload, 10),
    };
}

DecodedPayload decode_payload(std::uint8_t msg_type, const std::vector<std::uint8_t>& payload) {
    switch (static_cast<MsgType>(msg_type)) {
        case MsgType::Ping:
        case MsgType::GyroZeroCmd:
            check_payload_len(msg_type, payload);
            return std::monostate{};
        case MsgType::Ack:
            return decode_ack(payload);
        case MsgType::Error:
            return decode_error(payload);
        case MsgType::MotorCmd:
            return decode_motor_cmd(payload);
        case MsgType::StopCmd:
            return decode_stop_cmd(payload);
        case MsgType::ModeCmd:
            return decode_mode_cmd(payload);
        case MsgType::ConfigSet:
            return decode_config_set(payload);
        case MsgType::HeartbeatCmd:
            return decode_heartbeat_cmd(payload);
        case MsgType::ImuTelemetry:
            return decode_imu_telemetry(payload);
        case MsgType::SafetyTelemetry:
            return decode_safety_telemetry(payload);
        case MsgType::EncoderTelemetry:
            return decode_encoder_telemetry(payload);
        case MsgType::MotorState:
            return decode_motor_state(payload);
        case MsgType::HeartbeatState:
            return decode_heartbeat_state(payload);
        default:
            throw DecodeError("Unknown message type: " + message_name(msg_type));
    }
}

DecodedMessage decode_frame(const Frame& frame) {
    return {
        frame,
        message_name(frame.msg_type),
        decode_payload(frame.msg_type, frame.payload),
    };
}

}  // namespace thesis_sim
