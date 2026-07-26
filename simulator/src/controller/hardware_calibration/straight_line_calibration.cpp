#include "mvc/controller/hardware_calibration/straight_line_calibration.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <poll.h>
#include <stdexcept>
#include <thread>
#include <unistd.h>

#include "mvc/controller/hardware_io/real_robot_bridge.h"

namespace thesis_sim {
namespace {

constexpr double kPi = 3.14159265358979323846;
volatile std::sig_atomic_t g_stop_requested = 0;

double monotonic_seconds() {
    using clock = std::chrono::steady_clock;
    return std::chrono::duration<double>(clock::now().time_since_epoch()).count();
}

double wrap_angle(double angle) {
    while (angle > kPi) angle -= 2.0 * kPi;
    while (angle < -kPi) angle += 2.0 * kPi;
    return angle;
}

void handle_signal(int) {
    g_stop_requested = 1;
}

bool stop_line_available() {
    pollfd input{};
    input.fd = STDIN_FILENO;
    input.events = POLLIN;
    const int rc = ::poll(&input, 1, 0);
    if (rc <= 0 || (input.revents & (POLLIN | POLLHUP)) == 0) {
        return false;
    }
    char buffer[128];
    const ssize_t count = ::read(STDIN_FILENO, buffer, sizeof(buffer));
    return count >= 0;
}

void stop_controller(RealRobotBridge* bridge) noexcept {
    if (bridge == nullptr || !bridge->controller_connected()) return;
    try {
        bridge->send_pwm(0, 0, true, MotorControlMode::SafeDirectPwm);
    } catch (const std::exception&) {
    }
    try {
        bridge->stop(StopReason::UserRequest, true, 0.45);
    } catch (const std::exception&) {
    }
    try {
        bridge->set_mode(ControllerMode::Idle, 0.45);
    } catch (const std::exception&) {
    }
}

class ControllerStopGuard {
  public:
    explicit ControllerStopGuard(RealRobotBridge* bridge) : bridge_(bridge) {}
    ~ControllerStopGuard() {
        stop_controller(bridge_);
        if (bridge_ != nullptr) bridge_->disconnect();
    }

    ControllerStopGuard(const ControllerStopGuard&) = delete;
    ControllerStopGuard& operator=(const ControllerStopGuard&) = delete;

  private:
    RealRobotBridge* bridge_;
};

const ControllerTelemetry& wait_for_calibration_telemetry(RealRobotBridge* bridge) {
    const double deadline = monotonic_seconds() + 5.0;
    while (monotonic_seconds() < deadline) {
        bridge->poll_controller(0.08);
        const ControllerTelemetry& telemetry = bridge->observation().controller;
        if (telemetry.have_motor && telemetry.have_encoder && telemetry.have_heartbeat) {
            return telemetry;
        }
    }
    throw ProtocolResponseError(
        "Controller telemetry incomplete: motor, encoder and heartbeat are required");
}

StraightLineCalibrationSample make_sample(const ControllerTelemetry& telemetry,
                                          const StraightLineCalibrationResult& result,
                                          double elapsed_s,
                                          double initial_yaw_rad) {
    StraightLineCalibrationSample sample{};
    sample.time_s = elapsed_s;
    sample.left_ticks = telemetry.ticks_left;
    sample.right_ticks = telemetry.ticks_right;
    sample.left_tick_delta =
        static_cast<std::int64_t>(telemetry.ticks_left) - result.initial_left_ticks;
    sample.right_tick_delta =
        static_cast<std::int64_t>(telemetry.ticks_right) - result.initial_right_ticks;
    const double meters_per_tick =
        2.0 * kPi * result.options.wheel_radius_m /
        static_cast<double>(result.options.encoder_ticks_per_revolution);
    sample.left_distance_m = std::abs(static_cast<double>(sample.left_tick_delta)) * meters_per_tick;
    sample.right_distance_m = std::abs(static_cast<double>(sample.right_tick_delta)) * meters_per_tick;
    sample.center_distance_m = 0.5 * (sample.left_distance_m + sample.right_distance_m);
    sample.yaw_rad = telemetry.yaw_mrad * 0.001;
    sample.yaw_delta_rad = wrap_angle(sample.yaw_rad - initial_yaw_rad);
    sample.target_pwm_left = telemetry.target_pwm_l;
    sample.target_pwm_right = telemetry.target_pwm_r;
    sample.actual_pwm_left = telemetry.pwm_l;
    sample.actual_pwm_right = telemetry.pwm_r;
    sample.encoder_flags = telemetry.enc_flags;
    sample.motor_flags = telemetry.motor_flags;
    sample.status_flags = telemetry.status_flags;
    sample.controller_error_code = telemetry.error_code;
    return sample;
}

}  // namespace

StraightLineCalibrationResult run_straight_line_calibration(
    const StraightLineCalibrationOptions& options) {
    if (options.controller_port.empty()) {
        throw std::invalid_argument("--controller-port is required");
    }
    if (options.pwm < 45 || options.pwm > 160) {
        throw std::invalid_argument("Calibration PWM must be between 45 and 160");
    }
    if (options.safety_timeout_s < 1.0 || options.safety_timeout_s > 60.0) {
        throw std::invalid_argument("Safety timeout must be between 1 and 60 seconds");
    }
    if (options.wheel_radius_m <= 0.0 || options.encoder_ticks_per_revolution <= 0) {
        throw std::invalid_argument("Wheel radius and encoder ticks/revolution must be positive");
    }

    g_stop_requested = 0;
    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    RealRobotBridge::Options bridge_options{};
    bridge_options.controller.port = options.controller_port;
    bridge_options.controller.baudrate = options.controller_baudrate;
    bridge_options.controller.reset_on_connect = options.reset_controller_on_connect;
    RealRobotBridge bridge(bridge_options);
    bridge.connect(false);
    ControllerStopGuard stop_guard(&bridge);

    try {
        bridge.ping(0.8);
    } catch (const std::exception& error) {
        std::cerr << "calibration_warning=initial ping failed: " << error.what() << '\n';
    }
    const ControllerTelemetry& ready = wait_for_calibration_telemetry(&bridge);
    std::cout << "controller_firmware=" << static_cast<int>(ready.fw_major) << "."
              << static_cast<int>(ready.fw_minor) << '\n';
    std::cout << "encoder_flags=0x" << std::hex << ready.enc_flags << std::dec << '\n';

    bridge.set_mode(ControllerMode::Manual, 1.0);
    bridge.send_pwm(0, 0, true, MotorControlMode::SafeDirectPwm);

    std::cout << "\nPosiziona la car all'inizio del tratto rettilineo.\n"
                 "Premi INVIO per iniziare; durante il moto premi di nuovo INVIO per fermarla.\n"
                 "CTRL+C attiva lo stesso arresto controllato.\n> "
              << std::flush;
    std::string start_confirmation;
    if (!std::getline(std::cin, start_confirmation)) {
        throw std::runtime_error("Standard input closed before calibration start");
    }

    for (int countdown = 3; countdown > 0; --countdown) {
        std::cout << "Avvio tra " << countdown << "...\n" << std::flush;
        bridge.send_pwm(0, 0, true, MotorControlMode::SafeDirectPwm);
        bridge.poll_controller(0.10);
        std::this_thread::sleep_for(std::chrono::milliseconds(900));
        if (g_stop_requested != 0) {
            throw std::runtime_error("Calibration cancelled before motion");
        }
    }

    const ControllerTelemetry& initial = wait_for_calibration_telemetry(&bridge);
    StraightLineCalibrationResult result{};
    result.options = options;
    result.firmware_major = initial.fw_major;
    result.firmware_minor = initial.fw_minor;
    result.initial_left_ticks = initial.ticks_left;
    result.initial_right_ticks = initial.ticks_right;
    const double initial_yaw_rad = initial.yaw_mrad * 0.001;
    double latest_encoder_rx_s = initial.encoder_rx_timestamp_s;
    const double start_s = monotonic_seconds();
    double last_print_s = -1.0;

    while (true) {
        const double elapsed_s = monotonic_seconds() - start_s;
        if (g_stop_requested != 0) {
            result.stop_reason = "signal";
            break;
        }
        if (stop_line_available()) {
            result.stop_reason = "operator_enter";
            break;
        }
        if (elapsed_s >= options.safety_timeout_s) {
            result.stop_reason = "safety_timeout";
            break;
        }

        bridge.send_pwm(
            static_cast<std::int16_t>(options.pwm),
            static_cast<std::int16_t>(options.pwm),
            false,
            MotorControlMode::SafeDirectPwm);
        bridge.poll_controller(0.02);
        const ControllerTelemetry& telemetry = bridge.observation().controller;
        if (telemetry.encoder_rx_timestamp_s > latest_encoder_rx_s) {
            latest_encoder_rx_s = telemetry.encoder_rx_timestamp_s;
            result.samples.push_back(make_sample(telemetry, result, elapsed_s, initial_yaw_rad));
        }
        if (latest_encoder_rx_s > 0.0 && monotonic_seconds() - latest_encoder_rx_s > 0.75) {
            result.stop_reason = "encoder_telemetry_timeout";
            break;
        }
        if (elapsed_s > 2.0 && !result.samples.empty() &&
            result.samples.back().left_tick_delta == 0 &&
            result.samples.back().right_tick_delta == 0) {
            result.stop_reason = "no_encoder_ticks";
            break;
        }

        if (!result.samples.empty() && elapsed_s - last_print_s >= 0.25) {
            const StraightLineCalibrationSample& sample = result.samples.back();
            std::cout << "\rt=" << std::fixed << std::setprecision(2) << elapsed_s
                      << " s  ticks L/R=" << sample.left_tick_delta << "/"
                      << sample.right_tick_delta
                      << "  odom=" << std::setprecision(3) << sample.center_distance_m
                      << " m  yaw=" << std::setprecision(1)
                      << sample.yaw_delta_rad * 180.0 / kPi << " deg   " << std::flush;
            last_print_s = elapsed_s;
        }
    }

    result.runtime_s = monotonic_seconds() - start_s;
    stop_controller(&bridge);
    std::cout << "\nMotori arrestati. Attendo l'assestamento degli encoder...\n";
    const double settle_deadline = monotonic_seconds() + 0.45;
    while (monotonic_seconds() < settle_deadline) {
        bridge.poll_controller(0.04);
    }

    const ControllerTelemetry& final = bridge.observation().controller;
    const StraightLineCalibrationSample final_sample =
        make_sample(final, result, result.runtime_s, initial_yaw_rad);
    result.samples.push_back(final_sample);
    result.final_left_ticks = final.ticks_left;
    result.final_right_ticks = final.ticks_right;
    result.left_tick_delta = final_sample.left_tick_delta;
    result.right_tick_delta = final_sample.right_tick_delta;
    result.nominal_left_distance_m = final_sample.left_distance_m;
    result.nominal_right_distance_m = final_sample.right_distance_m;
    result.nominal_center_distance_m = final_sample.center_distance_m;
    result.yaw_delta_rad = final_sample.yaw_delta_rad;
    const double mean_ticks = 0.5 * (
        std::abs(static_cast<double>(result.left_tick_delta)) +
        std::abs(static_cast<double>(result.right_tick_delta)));
    if (mean_ticks > 0.0) {
        result.tick_imbalance_percent = 100.0 *
            (std::abs(static_cast<double>(result.left_tick_delta)) -
             std::abs(static_cast<double>(result.right_tick_delta))) /
            mean_ticks;
    }
    return result;
}

void apply_straight_line_measurement(double measured_distance_m,
                                     StraightLineCalibrationResult* result) {
    if (result == nullptr) {
        throw std::invalid_argument("Straight-line result is null");
    }
    if (measured_distance_m <= 0.0) {
        throw std::invalid_argument("Measured distance must be positive");
    }
    if (result->nominal_center_distance_m <= 0.0) {
        throw std::invalid_argument("Nominal encoder distance is zero");
    }
    result->measured_distance_available = true;
    result->measured_distance_m = measured_distance_m;
    result->metric_scale_factor =
        measured_distance_m / result->nominal_center_distance_m;
    const double left_ticks = std::abs(static_cast<double>(result->left_tick_delta));
    const double right_ticks = std::abs(static_cast<double>(result->right_tick_delta));
    const double mean_ticks = 0.5 * (left_ticks + right_ticks);
    result->meters_per_tick = measured_distance_m / mean_ticks;
    result->equivalent_wheel_radius_if_ticks_fixed_m =
        result->options.wheel_radius_m * result->metric_scale_factor;
    result->left_ticks_per_meter = left_ticks / measured_distance_m;
    result->right_ticks_per_meter = right_ticks / measured_distance_m;
    result->center_ticks_per_meter = 0.5 * (left_ticks + right_ticks) / measured_distance_m;
    const double wheel_circumference_m = 2.0 * kPi * result->options.wheel_radius_m;
    result->left_ticks_per_revolution =
        result->left_ticks_per_meter * wheel_circumference_m;
    result->right_ticks_per_revolution =
        result->right_ticks_per_meter * wheel_circumference_m;
    result->calibrated_encoder_ticks_per_revolution =
        result->center_ticks_per_meter * wheel_circumference_m;
}

}  // namespace thesis_sim
