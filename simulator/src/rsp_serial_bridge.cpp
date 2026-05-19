#include "rsp_serial_bridge.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <thread>

namespace thesis_sim {

namespace {

double monotonic_seconds() {
    using clock = std::chrono::steady_clock;
    return std::chrono::duration<double>(clock::now().time_since_epoch()).count();
}

ConfigValue config_value_from_int(ConfigValueType value_type, std::int64_t value) {
    switch (value_type) {
        case ConfigValueType::Uint8:
            return static_cast<std::uint8_t>(value & 0xFF);
        case ConfigValueType::Int16:
            return static_cast<std::int16_t>(value);
        case ConfigValueType::Uint16:
            return static_cast<std::uint16_t>(value & 0xFFFF);
        case ConfigValueType::Int32:
            return static_cast<std::int32_t>(value);
        case ConfigValueType::Uint32:
            return static_cast<std::uint32_t>(value & 0xFFFFFFFFULL);
        default:
            throw ProtocolResponseError("Unsupported config value type");
    }
}

}  // namespace

RSPSerialBridge::RSPSerialBridge()
    : RSPSerialBridge(Options{}) {}

RSPSerialBridge::RSPSerialBridge(Options options)
    : options_(std::move(options)),
      parser_(options_.max_payload) {}

void RSPSerialBridge::connect() {
    serial_.open(options_.port, options_.baudrate, options_.timeout_s);

    if (options_.reset_on_connect) {
        try {
            serial_.set_dtr(false);
            serial_.set_rts(false);
            std::this_thread::sleep_for(std::chrono::duration<double>(std::max(0.02, options_.reset_pulse_s)));
            serial_.reset_input_buffer();
            serial_.reset_output_buffer();
            serial_.set_dtr(true);
        } catch (const SerialError&) {
        }
    }

    std::this_thread::sleep_for(std::chrono::duration<double>(std::max(0.0, options_.startup_delay_s)));
    serial_.reset_input_buffer();
    serial_.reset_output_buffer();

    parser_.clear();
    tx_seq_ = 0;
    pending_acks_.clear();
    pending_errors_.clear();
    last_controller_error_.reset();
    telemetry_ = {};
    last_sent_pwm_ = {std::nullopt, std::nullopt};
    last_send_ts_ = 0.0;
    last_heartbeat_tx_ = 0.0;
    last_rx_ts_ = 0.0;
}

void RSPSerialBridge::disconnect() {
    if (serial_.is_open()) {
        serial_.close();
    }
}

const ControllerTelemetry* RSPSerialBridge::telemetry() const {
    return telemetry_.ready() ? &telemetry_ : nullptr;
}

void RSPSerialBridge::poll(double timeout_s) {
    require_serial();
    const double deadline = monotonic_seconds() + std::max(0.0, timeout_s);

    while (true) {
        const std::size_t waiting = serial_.bytes_available();
        if (waiting > 0) {
            const std::vector<std::uint8_t> chunk = serial_.read(waiting, 0.0);
            for (const Frame& frame : parser_.feed(chunk)) {
                handle_frame(frame);
            }
        }

        maybe_send_heartbeat();

        if (monotonic_seconds() >= deadline) {
            break;
        }
        if (waiting == 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }
}

void RSPSerialBridge::send_ack(std::uint8_t acked_seq,
                               std::uint8_t acked_type,
                               std::uint8_t status,
                               std::uint8_t detail) {
    const std::vector<std::uint8_t> payload = encode_ack(acked_seq, acked_type, status, detail);
    write_frame(static_cast<std::uint8_t>(MsgType::Ack), payload, static_cast<std::uint8_t>(FrameFlag::AckFrame));
}

void RSPSerialBridge::send_error(std::uint8_t error_code,
                                 std::uint8_t related_type,
                                 std::uint8_t related_seq,
                                 std::uint8_t detail) {
    const std::vector<std::uint8_t> payload = encode_error(error_code, related_type, related_seq, detail);
    write_frame(static_cast<std::uint8_t>(MsgType::Error), payload, static_cast<std::uint8_t>(FrameFlag::ErrFrame));
}

AckPayload RSPSerialBridge::ping(double timeout_s) {
    const std::uint8_t seq = write_frame(static_cast<std::uint8_t>(MsgType::Ping), {}, static_cast<std::uint8_t>(FrameFlag::AckReq));
    return wait_for_response(seq, static_cast<std::uint8_t>(MsgType::Ping), timeout_s);
}

AckPayload RSPSerialBridge::set_mode(ControllerMode mode, double timeout_s) {
    const std::vector<std::uint8_t> payload = encode_mode_cmd(static_cast<std::uint8_t>(mode));
    const std::uint8_t seq = write_frame(static_cast<std::uint8_t>(MsgType::ModeCmd), payload, static_cast<std::uint8_t>(FrameFlag::AckReq));
    return wait_for_response(seq, static_cast<std::uint8_t>(MsgType::ModeCmd), timeout_s);
}

AckPayload RSPSerialBridge::gyro_zero(double timeout_s) {
    const std::uint8_t seq = write_frame(static_cast<std::uint8_t>(MsgType::GyroZeroCmd),
                                         encode_gyro_zero_cmd(),
                                         static_cast<std::uint8_t>(FrameFlag::AckReq));
    return wait_for_response(seq, static_cast<std::uint8_t>(MsgType::GyroZeroCmd), timeout_s);
}

AckPayload RSPSerialBridge::config_set(std::uint8_t param_id,
                                       ConfigValueType value_type,
                                       std::int64_t value,
                                       double timeout_s) {
    const std::vector<std::uint8_t> payload = encode_config_set(param_id, value_type, config_value_from_int(value_type, value));
    const std::uint8_t seq = write_frame(static_cast<std::uint8_t>(MsgType::ConfigSet), payload, static_cast<std::uint8_t>(FrameFlag::AckReq));
    return wait_for_response(seq, static_cast<std::uint8_t>(MsgType::ConfigSet), timeout_s);
}

void RSPSerialBridge::send_heartbeat(std::uint16_t host_status, std::optional<std::uint32_t> host_time_ms) {
    if (!host_time_ms.has_value()) {
        host_time_ms = static_cast<std::uint32_t>(std::llround(monotonic_seconds() * 1000.0));
    }
    const std::vector<std::uint8_t> payload = encode_heartbeat_cmd(*host_time_ms, host_status);
    write_frame(static_cast<std::uint8_t>(MsgType::HeartbeatCmd), payload);
    last_heartbeat_tx_ = monotonic_seconds();
}

void RSPSerialBridge::send_pwm(std::int16_t pwm_l,
                               std::int16_t pwm_r,
                               bool force,
                               MotorControlMode control_mode) {
    require_serial();
    pwm_l = static_cast<std::int16_t>(std::clamp<int>(pwm_l, -255, 255));
    pwm_r = static_cast<std::int16_t>(std::clamp<int>(pwm_r, -255, 255));
    const double now = monotonic_seconds();
    if (!force &&
        last_sent_pwm_.first.has_value() &&
        last_sent_pwm_.second.has_value() &&
        pwm_l == *last_sent_pwm_.first &&
        pwm_r == *last_sent_pwm_.second &&
        (now - last_send_ts_) < 0.15) {
        return;
    }

    const std::vector<std::uint8_t> payload =
        encode_motor_cmd(pwm_l, pwm_r, static_cast<std::uint8_t>(control_mode));
    write_frame(static_cast<std::uint8_t>(MsgType::MotorCmd), payload);
    last_sent_pwm_ = {pwm_l, pwm_r};
}

std::optional<AckPayload> RSPSerialBridge::stop(StopReason reason, bool wait_ack, double timeout_s) {
    const std::vector<std::uint8_t> payload = encode_stop_cmd(static_cast<std::uint8_t>(reason));
    const std::uint8_t seq = write_frame(static_cast<std::uint8_t>(MsgType::StopCmd),
                                         payload,
                                         static_cast<std::uint8_t>(FrameFlag::AckReq));
    serial_.flush();
    last_sent_pwm_ = {0, 0};
    if (!wait_ack) {
        return std::nullopt;
    }
    return wait_for_response(seq, static_cast<std::uint8_t>(MsgType::StopCmd), timeout_s);
}

void RSPSerialBridge::require_serial() const {
    if (!serial_.is_open()) {
        throw ProtocolResponseError("Serial link not connected");
    }
}

std::uint8_t RSPSerialBridge::next_seq() {
    const std::uint8_t seq = tx_seq_;
    tx_seq_ = static_cast<std::uint8_t>((tx_seq_ + 1U) & 0xFFU);
    return seq;
}

std::uint8_t RSPSerialBridge::write_frame(std::uint8_t msg_type,
                                          const std::vector<std::uint8_t>& payload,
                                          std::uint8_t flags,
                                          std::optional<std::uint8_t> seq) {
    require_serial();
    const std::uint8_t tx_seq = seq.has_value() ? *seq : next_seq();
    const std::vector<std::uint8_t> frame = build_frame(msg_type, payload, tx_seq, flags);
    serial_.write(frame);
    last_send_ts_ = monotonic_seconds();
    return tx_seq;
}

void RSPSerialBridge::refresh_public_telemetry() {
    if (telemetry_.rx_timestamp_s <= 0.0) {
        telemetry_.rx_timestamp_s = last_rx_ts_;
    }
}

void RSPSerialBridge::handle_frame(const Frame& frame) {
    const std::pair<std::uint8_t, std::uint8_t> key{frame.seq, frame.msg_type};

    if (!validate_payload_length(frame.msg_type, frame.payload)) {
        ErrorPayload error{
            static_cast<std::uint8_t>(ErrorCode::InvalidLength),
            frame.msg_type,
            frame.seq,
            static_cast<std::uint8_t>(frame.payload.size() & 0xFFU),
        };
        pending_errors_[key] = error;
        last_controller_error_ = error;
        if (frame.ack_requested()) {
            send_error(error.error_code, error.related_type, error.related_seq, error.detail);
        }
        return;
    }

    DecodedMessage decoded;
    try {
        decoded = decode_frame(frame);
    } catch (const DecodeError&) {
        ErrorPayload error{
            static_cast<std::uint8_t>(ErrorCode::UnknownMsgType),
            frame.msg_type,
            frame.seq,
            0,
        };
        pending_errors_[key] = error;
        last_controller_error_ = error;
        if (frame.ack_requested()) {
            send_error(error.error_code, error.related_type, error.related_seq, error.detail);
        }
        return;
    }

    last_rx_ts_ = monotonic_seconds();

    if (decoded.frame.is_ack()) {
        const AckPayload* ack = std::get_if<AckPayload>(&decoded.data);
        if (ack != nullptr) {
            pending_acks_[{ack->acked_seq, ack->acked_type}] = *ack;
        }
        return;
    }

    if (decoded.frame.is_error()) {
        const ErrorPayload* error = std::get_if<ErrorPayload>(&decoded.data);
        if (error != nullptr) {
            pending_errors_[{error->related_seq, error->related_type}] = *error;
            last_controller_error_ = *error;
        }
        return;
    }

    if (decoded.frame.ack_requested()) {
        send_ack(decoded.frame.seq, decoded.frame.msg_type);
    }

    if (const auto* imu = std::get_if<ImuTelemetryPayload>(&decoded.data)) {
        telemetry_.have_imu = true;
        telemetry_.imu_ms = imu->mcu_time_ms;
        telemetry_.ms = std::max(telemetry_.ms, imu->mcu_time_ms);
        telemetry_.yaw_mrad = imu->yaw_mrad;
        telemetry_.yaw_rate_mrad_s = imu->yaw_rate_mrad_s;
        telemetry_.yaw_deg = imu->yaw_mrad * 180.0 / (1000.0 * 3.14159265358979323846);
        telemetry_.yaw_rate_dps = imu->yaw_rate_mrad_s * 180.0 / (1000.0 * 3.14159265358979323846);
        telemetry_.acc_x_raw = imu->acc_x_raw;
        telemetry_.acc_y_raw = imu->acc_y_raw;
        telemetry_.acc_z_raw = imu->acc_z_raw;
        telemetry_.gyro_z_raw = imu->gyro_z_raw;
        telemetry_.rx_timestamp_s = last_rx_ts_;
    } else if (const auto* safety = std::get_if<SafetyTelemetryPayload>(&decoded.data)) {
        telemetry_.have_safety = true;
        telemetry_.safety_ms = safety->mcu_time_ms;
        telemetry_.ms = std::max(telemetry_.ms, safety->mcu_time_ms);
        telemetry_.dist_c_cm = static_cast<int>(safety->ultra_cm);
        telemetry_.ir_l_raw = safety->ir_left_raw;
        telemetry_.ir_r_raw = safety->ir_right_raw;
        telemetry_.safety_flags = safety->safety_flags;
        telemetry_.rx_timestamp_s = last_rx_ts_;
    } else if (const auto* motor = std::get_if<MotorStatePayload>(&decoded.data)) {
        telemetry_.have_motor = true;
        telemetry_.motor_ms = motor->mcu_time_ms;
        telemetry_.ms = std::max(telemetry_.ms, motor->mcu_time_ms);
        telemetry_.target_pwm_l = motor->target_pwm_left;
        telemetry_.target_pwm_r = motor->target_pwm_right;
        telemetry_.pwm_l = motor->current_pwm_left;
        telemetry_.pwm_r = motor->current_pwm_right;
        telemetry_.motor_flags = motor->motor_flags;
        telemetry_.rx_timestamp_s = last_rx_ts_;
    } else if (const auto* heartbeat = std::get_if<HeartbeatStatePayload>(&decoded.data)) {
        telemetry_.have_heartbeat = true;
        telemetry_.heartbeat_ms = heartbeat->mcu_time_ms;
        telemetry_.ms = std::max(telemetry_.ms, heartbeat->mcu_time_ms);
        telemetry_.uptime_s = heartbeat->uptime_s;
        telemetry_.status_flags = heartbeat->status_flags;
        telemetry_.fw_major = heartbeat->fw_major;
        telemetry_.fw_minor = heartbeat->fw_minor;
        telemetry_.error_code = heartbeat->error_code;
        telemetry_.rx_timestamp_s = last_rx_ts_;
    } else if (const auto* encoder = std::get_if<EncoderTelemetryPayload>(&decoded.data)) {
        telemetry_.have_encoder = true;
        telemetry_.encoder_ms = encoder->mcu_time_ms;
        telemetry_.ms = std::max(telemetry_.ms, encoder->mcu_time_ms);
        telemetry_.ticks_left = encoder->ticks_left;
        telemetry_.ticks_right = encoder->ticks_right;
        telemetry_.enc_dt_ms = encoder->dt_ms;
        telemetry_.enc_flags = encoder->enc_flags;
        telemetry_.rx_timestamp_s = last_rx_ts_;
    }

    refresh_public_telemetry();
}

void RSPSerialBridge::maybe_send_heartbeat() {
    if (options_.heartbeat_interval_s <= 0.0) {
        return;
    }

    const double now = monotonic_seconds();
    if ((now - last_heartbeat_tx_) < options_.heartbeat_interval_s) {
        return;
    }
    if ((now - last_send_ts_) < options_.heartbeat_interval_s * 0.5) {
        return;
    }
    send_heartbeat();
}

AckPayload RSPSerialBridge::wait_for_response(std::uint8_t seq, std::uint8_t msg_type, double timeout_s) {
    const double deadline = monotonic_seconds() + std::max(0.01, timeout_s);
    const std::pair<std::uint8_t, std::uint8_t> key{seq, msg_type};

    while (monotonic_seconds() < deadline) {
        poll(0.05);

        auto err_it = pending_errors_.find(key);
        if (err_it != pending_errors_.end()) {
            const ErrorPayload error = err_it->second;
            pending_errors_.erase(err_it);
            throw ProtocolResponseError(message_name(msg_type) +
                                        " failed with ERROR 0x" +
                                        [&]() {
                                            char buf[5];
                                            std::snprintf(buf, sizeof(buf), "%02X", error.error_code);
                                            return std::string(buf);
                                        }() +
                                        " detail=" + std::to_string(error.detail));
        }

        auto ack_it = pending_acks_.find(key);
        if (ack_it != pending_acks_.end()) {
            const AckPayload ack = ack_it->second;
            pending_acks_.erase(ack_it);
            if (ack.status == static_cast<std::uint8_t>(AckStatus::Rejected)) {
                throw ProtocolResponseError(message_name(msg_type) +
                                            " rejected detail=" + std::to_string(ack.detail));
            }
            return ack;
        }
    }

    throw ProtocolResponseError("Timeout waiting ACK for " + message_name(msg_type) +
                                " seq=" + std::to_string(seq));
}

}  // namespace thesis_sim
