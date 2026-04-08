#include "rplidar_a1.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <thread>
#include <utility>

namespace thesis_sim {

namespace {

constexpr int kMotorWarmupMs = 350;
constexpr int kStartScanRetryCount = 3;

std::string bytes_to_hex(const std::vector<std::uint8_t>& data) {
    std::ostringstream oss;
    oss << std::uppercase << std::hex << std::setfill('0');
    for (std::uint8_t value : data) {
        oss << std::setw(2) << static_cast<int>(value);
    }
    return oss.str();
}

double deg_to_rad(double angle_deg) {
    return angle_deg * 3.14159265358979323846 / 180.0;
}

}  // namespace

RPLidarA1::RPLidarA1(std::string port, int baudrate, double timeout_s, int motor_pwm)
    : port_(std::move(port)),
      baudrate_(baudrate),
      timeout_s_(timeout_s),
      motor_pwm_(std::clamp(motor_pwm, 0, kMaxMotorPwm)) {}

void RPLidarA1::connect() {
    try {
        serial_.open(port_, baudrate_, timeout_s_);
        serial_.reset_input_buffer();
        serial_.reset_output_buffer();
    } catch (const SerialError& e) {
        throw LidarError("Cannot open " + port_ + ": " + e.what());
    }
}

void RPLidarA1::disconnect() {
    if (serial_.is_open()) {
        serial_.close();
    }
    motor_running_ = false;
    scanning_ = false;
}

void RPLidarA1::send_command(std::uint8_t cmd, const std::vector<std::uint8_t>& payload) {
    require_serial();

    std::vector<std::uint8_t> packet;
    if (!payload.empty()) {
        packet.reserve(4 + payload.size());
        packet.push_back(kSyncA);
        packet.push_back(cmd);
        packet.push_back(static_cast<std::uint8_t>(payload.size()));
        packet.insert(packet.end(), payload.begin(), payload.end());
        std::uint8_t checksum = 0;
        for (std::uint8_t value : packet) {
            checksum ^= value;
        }
        packet.push_back(checksum);
    } else {
        packet = {kSyncA, cmd};
    }

    try {
        serial_.write(packet);
    } catch (const SerialError& e) {
        throw LidarError(e.what());
    }
}

RPLidarA1::Descriptor RPLidarA1::read_descriptor(double timeout_s) {
    require_serial();

    const double effective_timeout = timeout_s >= 0.0 ? timeout_s : timeout_s_;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(std::max(0.01, effective_timeout));

    int state = 0;
    while (std::chrono::steady_clock::now() < deadline) {
        const std::vector<std::uint8_t> next = serial_.read(1, effective_timeout);
        if (next.empty()) {
            continue;
        }

        const std::uint8_t value = next.front();
        if (state == 0) {
            state = (value == kSyncA) ? 1 : 0;
        } else if (state == 1) {
            if (value == kSyncB) {
                const std::vector<std::uint8_t> rest = read_response(kDescriptorLen - 2, effective_timeout);
                Descriptor out;
                out.raw[0] = kSyncA;
                out.raw[1] = kSyncB;
                std::copy(rest.begin(), rest.end(), out.raw.begin() + 2);
                const std::uint32_t size_mode = static_cast<std::uint32_t>(out.raw[2]) |
                                                (static_cast<std::uint32_t>(out.raw[3]) << 8) |
                                                (static_cast<std::uint32_t>(out.raw[4]) << 16) |
                                                (static_cast<std::uint32_t>(out.raw[5]) << 24);
                out.size = size_mode & 0x3FFFFFFFU;
                out.mode = static_cast<std::uint8_t>((size_mode >> 30) & 0x03U);
                out.dtype = out.raw[6];
                return out;
            }
            state = (value == kSyncA) ? 1 : 0;
        }
    }

    throw LidarError("Timeout waiting for descriptor");
}

std::vector<std::uint8_t> RPLidarA1::read_response(std::size_t size, double timeout_s) {
    require_serial();
    try {
        return serial_.read_exact(size, timeout_s >= 0.0 ? timeout_s : timeout_s_);
    } catch (const SerialError& e) {
        throw LidarError(e.what());
    }
}

RPLidarA1::DeviceInfo RPLidarA1::get_info() {
    require_serial();
    serial_.reset_input_buffer();
    send_command(kCmdGetInfo);
    const Descriptor descriptor = read_descriptor(1.5);
    const std::vector<std::uint8_t> raw = read_response(descriptor.size, 1.5);
    if (raw.size() < 20) {
        throw LidarError("GET_INFO response too short");
    }

    DeviceInfo info;
    info.model = raw[0];
    info.firmware_major = raw[2];
    info.firmware_minor = raw[1];
    info.hardware = raw[3];
    info.serial_hex = bytes_to_hex(std::vector<std::uint8_t>(raw.begin() + 4, raw.begin() + 20));
    return info;
}

RPLidarA1::Health RPLidarA1::get_health() {
    require_serial();
    serial_.reset_input_buffer();
    send_command(kCmdGetHealth);
    const Descriptor descriptor = read_descriptor(1.5);
    const std::vector<std::uint8_t> raw = read_response(descriptor.size, 1.5);
    if (raw.size() < 3) {
        throw LidarError("GET_HEALTH response too short");
    }

    Health health;
    health.status_code = raw[0];
    health.error_code = static_cast<std::uint16_t>(raw[1]) |
                        (static_cast<std::uint16_t>(raw[2]) << 8);
    switch (health.status_code) {
        case 0:
            health.status = "Good";
            break;
        case 1:
            health.status = "Warning";
            break;
        case 2:
            health.status = "Error";
            break;
        default:
            health.status = "Unknown";
            break;
    }
    return health;
}

void RPLidarA1::set_motor_pwm(int pwm) {
    require_serial();
    motor_pwm_ = std::clamp(pwm, 0, kMaxMotorPwm);
    const std::vector<std::uint8_t> payload = {
        static_cast<std::uint8_t>(motor_pwm_ & 0xFF),
        static_cast<std::uint8_t>((motor_pwm_ >> 8) & 0xFF),
    };
    send_command(kCmdSetPwm, payload);
}

void RPLidarA1::start_motor() {
    require_serial();
    serial_.set_dtr(false);
    set_motor_pwm(motor_pwm_);
    motor_running_ = true;
    std::this_thread::sleep_for(std::chrono::milliseconds(kMotorWarmupMs));
}

void RPLidarA1::stop_motor() {
    require_serial();
    set_motor_pwm(0);
    serial_.set_dtr(true);
    motor_running_ = false;
    std::this_thread::sleep_for(std::chrono::milliseconds(80));
}

void RPLidarA1::start_scan(bool force) {
    require_serial();
    if (!motor_running_) {
        start_motor();
    }

    std::string last_error;
    for (int attempt = 0; attempt < kStartScanRetryCount; ++attempt) {
        const bool use_force_scan = force || attempt > 0;
        try {
            if (attempt > 0) {
                try {
                    send_command(kCmdStop);
                } catch (const LidarError&) {
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(60 + attempt * 40));
            }

            serial_.reset_input_buffer();
            send_command(use_force_scan ? kCmdForceScan : kCmdScan);
            const Descriptor descriptor = read_descriptor(2.5 + 0.5 * static_cast<double>(attempt));
            if (descriptor.size != 5) {
                throw LidarError("Unexpected scan descriptor size: " + std::to_string(descriptor.size));
            }
            scanning_ = true;
            return;
        } catch (const LidarError& e) {
            last_error = e.what();
            scanning_ = false;
            try {
                serial_.reset_input_buffer();
            } catch (const SerialError&) {
            }
            if (attempt + 1 < kStartScanRetryCount) {
                std::this_thread::sleep_for(std::chrono::milliseconds(120 + attempt * 80));
            }
        }
    }

    throw LidarError("Unable to start scan on " + port_ + ": " + last_error);
}

void RPLidarA1::stop_scan(bool stop_motor_after) {
    if (!serial_.is_open()) {
        return;
    }

    try {
        send_command(kCmdStop);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    } catch (const LidarError&) {
    }

    scanning_ = false;
    if (stop_motor_after && motor_running_) {
        try {
            stop_motor();
        } catch (const LidarError&) {
        }
    }
}

RPLidarA1::Measurement RPLidarA1::read_scan_node() {
    require_serial();
    if (!scanning_) {
        throw LidarError("Scan not started");
    }

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(std::max(0.01, timeout_s_));
    std::array<std::uint8_t, 5> raw{};
    std::size_t filled = 0;
    while (std::chrono::steady_clock::now() < deadline) {
        const double remaining_s = std::chrono::duration<double>(deadline - std::chrono::steady_clock::now()).count();
        if (remaining_s <= 0.0) {
            break;
        }
        const std::vector<std::uint8_t> next = serial_.read(1, std::min(timeout_s_, std::max(0.01, remaining_s)));
        if (next.empty()) {
            continue;
        }

        raw[filled] = next.front();
        if (filled < raw.size()) {
            ++filled;
        }
        if (filled < raw.size()) {
            continue;
        }

        try {
            return process_scan_packet(raw);
        } catch (const LidarError&) {
            std::copy(raw.begin() + 1, raw.end(), raw.begin());
            filled = raw.size() - 1;
        }
    }

    throw LidarError("Timeout reading scan node");
}

std::vector<RPLidarA1::ScanPoint> RPLidarA1::grab_scan(int min_points, std::size_t max_bad_packets) {
    min_points = std::max(min_points, 1);

    std::vector<ScanPoint> scan;
    std::size_t bad_packets = 0;

    while (true) {
        try {
            const Measurement measurement = read_scan_node();
            if (measurement.new_scan && !scan.empty()) {
                if (static_cast<int>(scan.size()) >= min_points) {
                    return scan;
                }
                scan.clear();
            }

            if (measurement.distance_mm > 0.0) {
                const double angle_rad = deg_to_rad(measurement.angle_deg);
                const double distance_m = measurement.distance_mm / 1000.0;
                scan.push_back({
                    measurement.quality,
                    measurement.angle_deg,
                    measurement.distance_mm,
                    distance_m,
                    std::cos(angle_rad) * distance_m,
                    std::sin(angle_rad) * distance_m,
                });
            }

            bad_packets = 0;
        } catch (const LidarError&) {
            ++bad_packets;
            if (bad_packets > max_bad_packets) {
                throw;
            }
        }
    }
}

RPLidarA1::Measurement RPLidarA1::process_scan_packet(const std::array<std::uint8_t, 5>& raw) {
    const std::uint8_t b0 = raw[0];
    const std::uint8_t b1 = raw[1];
    const std::uint8_t b2 = raw[2];
    const std::uint8_t b3 = raw[3];
    const std::uint8_t b4 = raw[4];

    const bool start_flag = (b0 & 0x01U) != 0U;
    const bool inv_start = ((b0 >> 1U) & 0x01U) != 0U;
    if (start_flag == inv_start) {
        throw LidarError("Invalid start/inverse-start flags");
    }

    if ((b1 & 0x01U) != 0x01U) {
        throw LidarError("Invalid check bit in scan packet");
    }

    Measurement measurement;
    measurement.new_scan = start_flag;
    measurement.quality = static_cast<std::uint8_t>(b0 >> 2U);
    measurement.angle_deg = std::fmod((((b1 >> 1U) | (b2 << 7U)) / 64.0), 360.0);
    if (measurement.angle_deg < 0.0) {
        measurement.angle_deg += 360.0;
    }
    measurement.distance_mm = (static_cast<double>(b3) + static_cast<double>(b4 << 8U)) / 4.0;
    return measurement;
}

void RPLidarA1::require_serial() const {
    if (!serial_.is_open()) {
        throw LidarError("LiDAR not connected");
    }
}

}  // namespace thesis_sim
