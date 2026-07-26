#include "mvc/controller/hardware_calibration/straight_line_report_writer.h"

#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace thesis_sim {
namespace {

constexpr double kRadToDeg = 57.2957795130823208768;

std::string timestamp_suffix() {
    const auto now = std::chrono::system_clock::now();
    const auto seconds = std::chrono::time_point_cast<std::chrono::seconds>(now);
    const auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(now - seconds).count();
    const std::time_t value = std::chrono::system_clock::to_time_t(now);
    std::tm local{};
#if defined(_WIN32)
    localtime_s(&local, &value);
#else
    localtime_r(&value, &local);
#endif
    std::ostringstream out;
    out << std::put_time(&local, "%Y%m%d_%H%M%S_")
        << std::setw(3) << std::setfill('0') << millis;
    return out.str();
}

std::string json_escape(const std::string& value) {
    std::ostringstream out;
    for (const char c : value) {
        switch (c) {
            case '\\': out << "\\\\"; break;
            case '"': out << "\\\""; break;
            case '\n': out << "\\n"; break;
            case '\r': out << "\\r"; break;
            case '\t': out << "\\t"; break;
            default: out << c; break;
        }
    }
    return out.str();
}

void require_stream(const std::ofstream& stream, const std::string& path) {
    if (!stream) throw std::runtime_error("Could not write calibration report: " + path);
}

void write_csv(const StraightLineCalibrationResult& result, const std::string& path) {
    std::ofstream out(path);
    require_stream(out, path);
    out << "time_s,left_ticks,right_ticks,left_tick_delta,right_tick_delta,"
           "left_distance_m,right_distance_m,center_distance_m,yaw_rad,yaw_delta_rad,"
           "target_pwm_left,target_pwm_right,actual_pwm_left,actual_pwm_right,"
           "encoder_flags,motor_flags,status_flags,controller_error_code\n";
    out << std::fixed << std::setprecision(9);
    for (const StraightLineCalibrationSample& sample : result.samples) {
        out << sample.time_s << ','
            << sample.left_ticks << ',' << sample.right_ticks << ','
            << sample.left_tick_delta << ',' << sample.right_tick_delta << ','
            << sample.left_distance_m << ',' << sample.right_distance_m << ','
            << sample.center_distance_m << ',' << sample.yaw_rad << ','
            << sample.yaw_delta_rad << ',' << sample.target_pwm_left << ','
            << sample.target_pwm_right << ',' << sample.actual_pwm_left << ','
            << sample.actual_pwm_right << ',' << sample.encoder_flags << ','
            << sample.motor_flags << ',' << sample.status_flags << ','
            << sample.controller_error_code << '\n';
    }
    require_stream(out, path);
}

void write_json(const StraightLineCalibrationResult& result, const std::string& path) {
    std::ofstream out(path);
    require_stream(out, path);
    out << std::fixed << std::setprecision(9);
    out << "{\n"
        << "  \"schema\": \"thesis_hardware_straight_calibration_v1\",\n"
        << "  \"control_mode\": \"safe_direct_pwm\",\n"
        << "  \"controller_port\": \"" << json_escape(result.options.controller_port) << "\",\n"
        << "  \"controller_baudrate\": " << result.options.controller_baudrate << ",\n"
        << "  \"firmware\": \"" << static_cast<int>(result.firmware_major) << "."
        << static_cast<int>(result.firmware_minor) << "\",\n"
        << "  \"command_pwm_left\": " << result.options.pwm << ",\n"
        << "  \"command_pwm_right\": " << result.options.pwm << ",\n"
        << "  \"safety_timeout_s\": " << result.options.safety_timeout_s << ",\n"
        << "  \"stop_reason\": \"" << json_escape(result.stop_reason) << "\",\n"
        << "  \"runtime_s\": " << result.runtime_s << ",\n"
        << "  \"assumed_wheel_radius_m\": " << result.options.wheel_radius_m << ",\n"
        << "  \"assumed_encoder_ticks_per_revolution\": "
        << result.options.encoder_ticks_per_revolution << ",\n"
        << "  \"initial_left_ticks\": " << result.initial_left_ticks << ",\n"
        << "  \"initial_right_ticks\": " << result.initial_right_ticks << ",\n"
        << "  \"final_left_ticks\": " << result.final_left_ticks << ",\n"
        << "  \"final_right_ticks\": " << result.final_right_ticks << ",\n"
        << "  \"left_tick_delta\": " << result.left_tick_delta << ",\n"
        << "  \"right_tick_delta\": " << result.right_tick_delta << ",\n"
        << "  \"nominal_left_distance_m\": " << result.nominal_left_distance_m << ",\n"
        << "  \"nominal_right_distance_m\": " << result.nominal_right_distance_m << ",\n"
        << "  \"nominal_center_distance_m\": " << result.nominal_center_distance_m << ",\n"
        << "  \"yaw_delta_rad\": " << result.yaw_delta_rad << ",\n"
        << "  \"yaw_delta_deg\": " << result.yaw_delta_rad * kRadToDeg << ",\n"
        << "  \"tick_imbalance_percent\": " << result.tick_imbalance_percent << ",\n"
        << "  \"measured_distance_available\": "
        << (result.measured_distance_available ? "true" : "false") << ",\n"
        << "  \"measured_distance_m\": " << result.measured_distance_m << ",\n"
        << "  \"metric_scale_factor_real_over_odometry\": " << result.metric_scale_factor << ",\n"
        << "  \"meters_per_tick\": " << result.meters_per_tick << ",\n"
        << "  \"equivalent_wheel_radius_if_ticks_fixed_m\": "
        << result.equivalent_wheel_radius_if_ticks_fixed_m << ",\n"
        << "  \"left_ticks_per_meter\": " << result.left_ticks_per_meter << ",\n"
        << "  \"right_ticks_per_meter\": " << result.right_ticks_per_meter << ",\n"
        << "  \"center_ticks_per_meter\": " << result.center_ticks_per_meter << ",\n"
        << "  \"left_ticks_per_revolution\": " << result.left_ticks_per_revolution << ",\n"
        << "  \"right_ticks_per_revolution\": " << result.right_ticks_per_revolution << ",\n"
        << "  \"calibrated_encoder_ticks_per_revolution\": "
        << result.calibrated_encoder_ticks_per_revolution << ",\n"
        << "  \"sample_count\": " << result.samples.size() << "\n"
        << "}\n";
    require_stream(out, path);
}

void write_markdown(const StraightLineCalibrationResult& result, const std::string& path) {
    std::ofstream out(path);
    require_stream(out, path);
    out << std::fixed << std::setprecision(6);
    out << "# Hardware straight-line calibration\n\n"
        << "This run bypasses the planner and sends equal safe-direct PWM commands to both sides.\n\n"
        << "| Metric | Value |\n| --- | ---: |\n"
        << "| Firmware | `" << static_cast<int>(result.firmware_major) << "."
        << static_cast<int>(result.firmware_minor) << "` |\n"
        << "| PWM left/right | " << result.options.pwm << " / " << result.options.pwm << " |\n"
        << "| Stop reason | `" << result.stop_reason << "` |\n"
        << "| Runtime | " << result.runtime_s << " s |\n"
        << "| Tick delta left/right | " << result.left_tick_delta << " / "
        << result.right_tick_delta << " |\n"
        << "| Tick imbalance | " << result.tick_imbalance_percent << " % |\n"
        << "| Nominal distance left/right | " << result.nominal_left_distance_m << " / "
        << result.nominal_right_distance_m << " m |\n"
        << "| Nominal center distance | " << result.nominal_center_distance_m << " m |\n"
        << "| IMU yaw change | " << result.yaw_delta_rad * kRadToDeg << " deg |\n";
    if (result.measured_distance_available) {
        out << "| Physically measured distance | " << result.measured_distance_m << " m |\n"
            << "| Metric factor real/odometry | " << result.metric_scale_factor << " |\n"
            << "| Measured meters/tick | " << result.meters_per_tick << " m |\n"
            << "| Equivalent radius if tick/giro were retained | "
            << result.equivalent_wheel_radius_if_ticks_fixed_m << " m |\n"
            << "| Tick/m left/right | " << result.left_ticks_per_meter << " / "
            << result.right_ticks_per_meter << " |\n"
            << "| Effective tick/giro left/right | " << result.left_ticks_per_revolution
            << " / " << result.right_ticks_per_revolution << " |\n"
            << "| Effective tick/giro center | "
            << result.calibrated_encoder_ticks_per_revolution << " |\n";
    }
    out << "\nThe metric factor is `physical_distance / nominal_encoder_distance`. "
           "With the measured 32.7 mm radius, the effective tick/revolution value is the "
           "preferred calibration. The equivalent radius is diagnostic only. "
           "The metric factor is not a map scaling factor. "
           "A large left/right imbalance or yaw change must be resolved before resizing the road.\n";
    require_stream(out, path);
}

}  // namespace

StraightLineReportPaths write_straight_line_calibration_report(
    const StraightLineCalibrationResult& result,
    const std::string& output_directory) {
    const std::filesystem::path directory =
        output_directory.empty() ? std::filesystem::path("reports")
                                 : std::filesystem::path(output_directory);
    std::filesystem::create_directories(directory);
    const std::string base =
        "thesis_hardware_car_straight_calibration_" + timestamp_suffix();
    StraightLineReportPaths paths{};
    paths.csv_path = (directory / (base + ".csv")).string();
    paths.json_path = (directory / (base + ".json")).string();
    paths.markdown_path = (directory / (base + ".md")).string();
    write_csv(result, paths.csv_path);
    write_json(result, paths.json_path);
    write_markdown(result, paths.markdown_path);
    return paths;
}

}  // namespace thesis_sim
