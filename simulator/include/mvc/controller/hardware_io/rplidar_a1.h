#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include "mvc/controller/hardware_io/serial_port.h"

namespace thesis_sim {

class LidarError : public std::runtime_error {
  public:
    using std::runtime_error::runtime_error;
};

class RPLidarA1 {
  public:
    static constexpr std::uint8_t kSyncA = 0xA5;
    static constexpr std::uint8_t kSyncB = 0x5A;
    static constexpr std::size_t kDescriptorLen = 7;

    static constexpr std::uint8_t kCmdStop = 0x25;
    static constexpr std::uint8_t kCmdScan = 0x20;
    static constexpr std::uint8_t kCmdForceScan = 0x21;
    static constexpr std::uint8_t kCmdReset = 0x40;
    static constexpr std::uint8_t kCmdGetInfo = 0x50;
    static constexpr std::uint8_t kCmdGetHealth = 0x52;
    static constexpr std::uint8_t kCmdSetPwm = 0xF0;

    static constexpr int kDefaultMotorPwm = 660;
    static constexpr int kMaxMotorPwm = 1023;

    struct Descriptor {
        std::uint32_t size = 0;
        std::uint8_t mode = 0;
        std::uint8_t dtype = 0;
        std::array<std::uint8_t, kDescriptorLen> raw{};
    };

    struct DeviceInfo {
        std::uint8_t model = 0;
        std::uint8_t firmware_major = 0;
        std::uint8_t firmware_minor = 0;
        std::uint8_t hardware = 0;
        std::string serial_hex;
    };

    struct Health {
        std::uint8_t status_code = 0;
        std::string status;
        std::uint16_t error_code = 0;
    };

    struct Measurement {
        bool new_scan = false;
        std::uint8_t quality = 0;
        double angle_deg = 0.0;
        double distance_mm = 0.0;
    };

    struct ScanPoint {
        std::uint8_t quality = 0;
        double angle_deg = 0.0;
        double distance_mm = 0.0;
        double distance_m = 0.0;
        double x_m = 0.0;
        double y_m = 0.0;
    };

    RPLidarA1(std::string port = {},
              int baudrate = 115200,
              double timeout_s = 1.0,
              int motor_pwm = kDefaultMotorPwm);

    void connect();
    void disconnect();

    bool is_connected() const { return serial_.is_open(); }
    bool motor_running() const { return motor_running_; }
    bool scanning() const { return scanning_; }

    void send_command(std::uint8_t cmd, const std::vector<std::uint8_t>& payload = {});
    Descriptor read_descriptor(double timeout_s = -1.0);
    std::vector<std::uint8_t> read_response(std::size_t size, double timeout_s = -1.0);

    DeviceInfo get_info();
    Health get_health();

    void set_motor_pwm(int pwm);
    void start_motor();
    void stop_motor();

    void start_scan(bool force = false);
    void stop_scan(bool stop_motor = true);

    Measurement read_scan_node();
    std::vector<ScanPoint> grab_scan(int min_points = 60, std::size_t max_bad_packets = 200);

    static Measurement process_scan_packet(const std::array<std::uint8_t, 5>& raw);

  private:
    void require_serial() const;

    SerialPort serial_;
    std::string port_;
    int baudrate_ = 115200;
    double timeout_s_ = 1.0;
    int motor_pwm_ = kDefaultMotorPwm;
    bool motor_running_ = false;
    bool scanning_ = false;
};

}  // namespace thesis_sim
