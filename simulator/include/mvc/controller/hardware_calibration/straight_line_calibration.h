#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace thesis_sim {

struct StraightLineCalibrationOptions {
    std::string controller_port;
    int controller_baudrate = 115200;
    int pwm = 90;
    double safety_timeout_s = 20.0;
    double wheel_radius_m = 0.0327;
    int encoder_ticks_per_revolution = 360;
    bool reset_controller_on_connect = true;
};

struct StraightLineCalibrationSample {
    double time_s = 0.0;
    std::int32_t left_ticks = 0;
    std::int32_t right_ticks = 0;
    std::int64_t left_tick_delta = 0;
    std::int64_t right_tick_delta = 0;
    double left_distance_m = 0.0;
    double right_distance_m = 0.0;
    double center_distance_m = 0.0;
    double yaw_rad = 0.0;
    double yaw_delta_rad = 0.0;
    std::int16_t target_pwm_left = 0;
    std::int16_t target_pwm_right = 0;
    std::int16_t actual_pwm_left = 0;
    std::int16_t actual_pwm_right = 0;
    std::uint16_t encoder_flags = 0;
    std::uint16_t motor_flags = 0;
    std::uint16_t status_flags = 0;
    std::uint16_t controller_error_code = 0;
};

struct StraightLineCalibrationResult {
    StraightLineCalibrationOptions options{};
    std::string stop_reason;
    std::uint8_t firmware_major = 0;
    std::uint8_t firmware_minor = 0;
    double runtime_s = 0.0;
    std::int32_t initial_left_ticks = 0;
    std::int32_t initial_right_ticks = 0;
    std::int32_t final_left_ticks = 0;
    std::int32_t final_right_ticks = 0;
    std::int64_t left_tick_delta = 0;
    std::int64_t right_tick_delta = 0;
    double nominal_left_distance_m = 0.0;
    double nominal_right_distance_m = 0.0;
    double nominal_center_distance_m = 0.0;
    double yaw_delta_rad = 0.0;
    double tick_imbalance_percent = 0.0;
    bool measured_distance_available = false;
    double measured_distance_m = 0.0;
    double metric_scale_factor = 0.0;
    double meters_per_tick = 0.0;
    double equivalent_wheel_radius_if_ticks_fixed_m = 0.0;
    double left_ticks_per_meter = 0.0;
    double right_ticks_per_meter = 0.0;
    double center_ticks_per_meter = 0.0;
    double left_ticks_per_revolution = 0.0;
    double right_ticks_per_revolution = 0.0;
    double calibrated_encoder_ticks_per_revolution = 0.0;
    std::vector<StraightLineCalibrationSample> samples;
};

StraightLineCalibrationResult run_straight_line_calibration(
    const StraightLineCalibrationOptions& options);

void apply_straight_line_measurement(double measured_distance_m,
                                     StraightLineCalibrationResult* result);

}  // namespace thesis_sim
