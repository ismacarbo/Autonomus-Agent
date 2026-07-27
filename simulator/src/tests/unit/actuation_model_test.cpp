#include "mvc/model/navigation/actuation_model.h"

#include <cmath>
#include <iostream>

namespace {

bool close(double a, double b, double tolerance = 1e-9) {
    return std::abs(a - b) <= tolerance;
}

}  // namespace

int main() {
    using namespace thesis_sim::mvc::model;

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
