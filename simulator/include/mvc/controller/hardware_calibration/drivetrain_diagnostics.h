#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace thesis_sim {

enum class DrivetrainDiagnosticSuite {
    Encoders,
    Imu,
    All,
};

enum class DrivetrainPhaseKind {
    Stationary,
    LeftOnly,
    RightOnly,
    Straight,
    PositiveYaw,
    NegativeYaw,
};

enum class DrivetrainAssessment {
    Pass,
    Warning,
    Fail,
};

struct DrivetrainDiagnosticThresholds {
    std::int64_t minimum_moving_ticks = 2;
    double maximum_inactive_tick_ratio = 0.20;
    double maximum_stationary_yaw_rad = 0.035;
    double maximum_straight_yaw_rad = 0.14;
    double minimum_turn_yaw_rad = 0.070;
    double warning_tick_imbalance_ratio = 0.30;
    double failure_tick_imbalance_ratio = 0.65;
};

struct DrivetrainDiagnosticOptions {
    std::string controller_port;
    int controller_baudrate = 115200;
    int test_pwm = 100;
    int turn_pwm_delta = 25;
    double phase_duration_s = 1.25;
    double settle_duration_s = 0.45;
    std::string output_directory = "reports";
    bool reset_controller_on_connect = true;
    DrivetrainDiagnosticSuite suite = DrivetrainDiagnosticSuite::All;
    DrivetrainDiagnosticThresholds thresholds{};
};

struct DrivetrainDiagnosticSample {
    double elapsed_s = 0.0;
    std::int32_t left_ticks = 0;
    std::int32_t right_ticks = 0;
    std::int64_t left_tick_delta = 0;
    std::int64_t right_tick_delta = 0;
    double yaw_rad = 0.0;
    double yaw_delta_rad = 0.0;
    double yaw_rate_rad_s = 0.0;
    std::int16_t requested_pwm_left = 0;
    std::int16_t requested_pwm_right = 0;
    std::int16_t target_pwm_left = 0;
    std::int16_t target_pwm_right = 0;
    std::int16_t actual_pwm_left = 0;
    std::int16_t actual_pwm_right = 0;
    std::uint16_t encoder_flags = 0;
    std::uint16_t motor_flags = 0;
    std::uint16_t status_flags = 0;
    std::uint16_t controller_error_code = 0;
};

struct DrivetrainPhaseResult {
    DrivetrainPhaseKind kind = DrivetrainPhaseKind::Stationary;
    std::string name;
    std::string setup;
    std::int16_t requested_pwm_left = 0;
    std::int16_t requested_pwm_right = 0;
    double runtime_s = 0.0;
    std::int64_t left_tick_delta = 0;
    std::int64_t right_tick_delta = 0;
    double yaw_delta_rad = 0.0;
    double mean_abs_yaw_rate_rad_s = 0.0;
    double peak_abs_yaw_rate_rad_s = 0.0;
    std::string observed_physical_motion = "unknown";
    DrivetrainAssessment assessment = DrivetrainAssessment::Warning;
    std::vector<std::string> findings;
    std::vector<DrivetrainDiagnosticSample> samples;
};

struct DrivetrainDiagnosticResult {
    DrivetrainDiagnosticOptions options{};
    std::uint8_t firmware_major = 0;
    std::uint8_t firmware_minor = 0;
    std::string stop_reason = "completed";
    DrivetrainAssessment overall_assessment = DrivetrainAssessment::Warning;
    std::vector<DrivetrainPhaseResult> phases;
};

struct DrivetrainDiagnosticReportPaths {
    std::string csv_path;
    std::string json_path;
    std::string markdown_path;
};

const char* drivetrain_phase_kind_name(DrivetrainPhaseKind kind);
const char* drivetrain_assessment_name(DrivetrainAssessment assessment);

DrivetrainAssessment assess_drivetrain_phase(
    DrivetrainPhaseResult* phase,
    const DrivetrainDiagnosticThresholds& thresholds = {});

DrivetrainDiagnosticResult run_drivetrain_diagnostics(
    const DrivetrainDiagnosticOptions& options);

DrivetrainDiagnosticReportPaths write_drivetrain_diagnostic_report(
    const DrivetrainDiagnosticResult& result);

}  // namespace thesis_sim
