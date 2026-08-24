#include "mvc/model/navigation/actuation_model.h"
#include "mvc/model/navigation/robot_calibration_profile.h"

#include <cmath>
#include <filesystem>
#include <iostream>

namespace {

bool close(double a, double b, double tolerance = 1e-9) {
    return std::abs(a - b) <= tolerance;
}

}  // namespace

int main() {
    using namespace thesis_sim::mvc::model;

    RobotCalibrationProfile car_profile;
    std::string profile_error;
    bool profile_loaded = false;
    for (const std::filesystem::path& candidate : {
             std::filesystem::path("config/robots/car_calibrated.json"),
             std::filesystem::path("../config/robots/car_calibrated.json"),
             std::filesystem::path("../../config/robots/car_calibrated.json")}) {
        if (load_robot_calibration_profile(
                candidate.string(), &car_profile, &profile_error)) {
            profile_loaded = true;
            break;
        }
    }
    if (!profile_loaded || car_profile.version != "1.6.0" ||
        !close(car_profile.right_pwm_command_scale, 0.849462, 1e-6) ||
        !close(car_profile.right_pwm_command_offset, 26.021505, 1e-6)) {
        std::cerr << "car_calibration_profile_failed: " << profile_error << '\n';
        return 1;
    }

    const auto straight = wheel_speeds_from_body(0.20, 0.0, 0.065);
    const auto pivot = wheel_speeds_from_body(0.0, 1.0, 0.065);
    if (!close(straight.first, 0.20) || !close(straight.second, 0.20) ||
        !close(pivot.first, -0.065) || !close(pivot.second, 0.065)) {
        std::cerr << "body_to_wheels_failed\n";
        return 1;
    }

    const int left_pwm = wheel_speed_to_pwm(-0.065, 1.0, 42, 255, 500.0, 0.0);
    const int right_pwm = wheel_speed_to_pwm(0.065, 1.0, 42, 255, 500.0, 0.0);
    if (left_pwm >= 0 || right_pwm <= 0 || std::abs(left_pwm) != std::abs(right_pwm)) {
        std::cerr << "wheel_to_pwm_failed\n";
        return 1;
    }

    thesis_sim::VehicleGeometry calibrated_geometry{};
    calibrated_geometry.min_effective_pwm = 45;
    calibrated_geometry.speed_estimate_per_pwm = 0.006209;
    const double calibrated_gain = 1.0 / calibrated_geometry.speed_estimate_per_pwm;
    const int calibrated_pwm = wheel_speed_to_pwm(
        0.055,
        1.0,
        calibrated_geometry.min_effective_pwm,
        255,
        calibrated_gain,
        0.0);
    const double reconstructed_speed = wheel_speed_from_pwm_estimate(
        calibrated_pwm,
        1.0,
        calibrated_geometry);
    if (calibrated_pwm != 54 || !close(reconstructed_speed, 0.055881, 1e-6)) {
        std::cerr << "calibrated_pwm_roundtrip_failed\n";
        return 1;
    }

    const int corrected_right_70 = apply_signed_pwm_calibration(
        70, 0.8494623655913978, 26.021505376344084, 255);
    const int corrected_right_100 = apply_signed_pwm_calibration(
        100, 0.8494623655913978, 26.021505376344084, 255);
    const int corrected_reverse = apply_signed_pwm_calibration(
        -100, 0.8494623655913978, 26.021505376344084, 255);
    if (corrected_right_70 != 85 || corrected_right_100 != 111 ||
        corrected_reverse != -111 ||
        apply_signed_pwm_calibration(0, 0.85, 26.0, 255) != 0 ||
        remove_signed_pwm_calibration(
            corrected_right_100, 0.8494623655913978, 26.021505376344084, 255) != 100 ||
        remove_signed_pwm_calibration(
            corrected_reverse, 0.8494623655913978, 26.021505376344084, 255) != -100) {
        std::cerr << "signed_pwm_calibration_failed\n";
        return 1;
    }

    const double corrected_positive_yaw = yaw_rate_target_with_feedback(
        0.36, 0.03, 0.10, 1.0, 0.40, 0.85);
    const double corrected_negative_yaw = yaw_rate_target_with_feedback(
        -0.36, 0.0, -0.05, 1.0, 0.40, 0.85);
    const double saturated_yaw = yaw_rate_target_with_feedback(
        0.70, -0.30, 1.0, 1.0, 1.0, 0.85);
    if (!close(corrected_positive_yaw, 0.73) ||
        !close(corrected_negative_yaw, -0.74) ||
        !close(saturated_yaw, 0.85)) {
        std::cerr << "yaw_rate_feedback_failed\n";
        return 1;
    }

    int boosted_left = 8;
    int boosted_right = 12;
    apply_start_motion_boost(40, &boosted_left, &boosted_right);
    if (boosted_left != 40 || boosted_right != 44) {
        std::cerr << "start_boost_failed\n";
        return 1;
    }

    int left_turn_left = 70;
    int left_turn_right = 70;
    enforce_forward_differential_pwm(
        0.12, 10, 255, &left_turn_left, &left_turn_right);
    if (left_turn_right - left_turn_left < 10) {
        std::cerr << "left_turn_pwm_authority_failed\n";
        return 1;
    }

    int right_turn_left = 70;
    int right_turn_right = 70;
    enforce_forward_differential_pwm(
        -0.12, 10, 255, &right_turn_left, &right_turn_right);
    if (right_turn_left - right_turn_right < 10) {
        std::cerr << "right_turn_pwm_authority_failed\n";
        return 1;
    }

    std::cout << "actuation_model_smoke: ok\n";
    return 0;
}
