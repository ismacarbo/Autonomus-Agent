#include "mvc/model/navigation/actuation_model.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace thesis_sim::mvc::model {
namespace {

int signum(int value) {
    return (value > 0) - (value < 0);
}

int full_scale_motion_pwm(int pwm, int max_pwm) {
    if (pwm == 0 || max_pwm <= 0) {
        return 0;
    }
    return signum(pwm) * max_pwm;
}

}  // namespace

double wheel_speed_from_pwm_estimate(int pwm,
                                     double channel_scale,
                                     const VehicleGeometry& geometry) {
    const double effective_pwm = std::abs(static_cast<double>(pwm));
    if (effective_pwm < static_cast<double>(geometry.min_effective_pwm) ||
        geometry.speed_estimate_per_pwm <= 0.0) {
        return 0.0;
    }

    const double scaled_magnitude =
        std::max(0.0, effective_pwm - static_cast<double>(geometry.min_effective_pwm));
    const double signed_speed =
        scaled_magnitude * geometry.speed_estimate_per_pwm *
        std::max(std::abs(channel_scale), 1e-3);
    return std::copysign(signed_speed, static_cast<double>(pwm));
}

std::pair<double, double> wheel_speeds_from_body(double speed_mps,
                                                 double yaw_rate_rad_s,
                                                 double half_track_m) {
    return {
        speed_mps - yaw_rate_rad_s * half_track_m,
        speed_mps + yaw_rate_rad_s * half_track_m,
    };
}

int wheel_speed_to_pwm(double wheel_speed_mps,
                       double channel_scale,
                       int min_effective_pwm,
                       int max_pwm,
                       double speed_to_pwm_gain,
                       double speed_to_pwm_bias) {
    if (std::abs(wheel_speed_mps) < 1e-4 || max_pwm <= 0) {
        return 0;
    }

    const int safe_min = std::clamp(min_effective_pwm, 0, max_pwm);
    const double scaled_speed =
        std::abs(wheel_speed_mps) * std::max(std::abs(channel_scale), 1e-3);
    const double pwm = static_cast<double>(safe_min) + speed_to_pwm_bias +
                       speed_to_pwm_gain * scaled_speed;
    const int sign = wheel_speed_mps >= 0.0 ? 1 : -1;
    const int magnitude = static_cast<int>(std::lround(std::clamp(
        pwm,
        static_cast<double>(safe_min),
        static_cast<double>(max_pwm))));
    return sign * magnitude;
}

int clamp_motion_pwm_band(int pwm, int min_pwm, int max_pwm) {
    if (pwm == 0 || max_pwm <= 0) {
        return 0;
    }
    const int safe_min = std::max(min_pwm, 0);
    const int safe_max = std::max(max_pwm, safe_min);
    return signum(pwm) * std::clamp(std::abs(pwm), safe_min, safe_max);
}

int slew_limit_pwm(int previous_pwm, int target_pwm, int max_delta) {
    if (max_delta <= 0) {
        return target_pwm;
    }
    return std::clamp(target_pwm, previous_pwm - max_delta, previous_pwm + max_delta);
}

void scale_motion_pwm_pair_to_full_scale(int max_pwm,
                                         int* pwm_left,
                                         int* pwm_right) {
    if (max_pwm <= 0 || pwm_left == nullptr || pwm_right == nullptr) {
        return;
    }

    const int dominant_magnitude = std::max(std::abs(*pwm_left), std::abs(*pwm_right));
    if (dominant_magnitude <= 0) {
        return;
    }

    const double scale = static_cast<double>(max_pwm) /
                         static_cast<double>(dominant_magnitude);
    *pwm_left = std::clamp(
        static_cast<int>(std::lround(static_cast<double>(*pwm_left) * scale)),
        -max_pwm,
        max_pwm);
    *pwm_right = std::clamp(
        static_cast<int>(std::lround(static_cast<double>(*pwm_right) * scale)),
        -max_pwm,
        max_pwm);

    if (std::abs(*pwm_left) >= std::abs(*pwm_right) && *pwm_left != 0) {
        *pwm_left = full_scale_motion_pwm(*pwm_left, max_pwm);
    } else if (*pwm_right != 0) {
        *pwm_right = full_scale_motion_pwm(*pwm_right, max_pwm);
    }
}

void apply_start_motion_boost(int min_pwm,
                              int* pwm_left,
                              int* pwm_right) {
    if (min_pwm <= 0 || pwm_left == nullptr || pwm_right == nullptr) {
        return;
    }

    constexpr int kActiveThreshold = 4;
    int active_min = std::numeric_limits<int>::max();
    if (std::abs(*pwm_left) > kActiveThreshold) {
        active_min = std::min(active_min, std::abs(*pwm_left));
    }
    if (std::abs(*pwm_right) > kActiveThreshold) {
        active_min = std::min(active_min, std::abs(*pwm_right));
    }
    if (active_min == std::numeric_limits<int>::max() || active_min >= min_pwm) {
        return;
    }

    const int delta = min_pwm - active_min;
    if (std::abs(*pwm_left) > kActiveThreshold) {
        *pwm_left = std::clamp(
            *pwm_left + signum(*pwm_left) * delta,
            -255,
            255);
    }
    if (std::abs(*pwm_right) > kActiveThreshold) {
        *pwm_right = std::clamp(
            *pwm_right + signum(*pwm_right) * delta,
            -255,
            255);
    }
}

void enforce_forward_tracked_turn_authority(double target_yaw_rate,
                                            int outer_min_pwm,
                                            int inner_min_pwm,
                                            int inner_max_pwm,
                                            int* pwm_left,
                                            int* pwm_right) {
    if (pwm_left == nullptr || pwm_right == nullptr || outer_min_pwm <= 0) {
        return;
    }
    constexpr double kTurnYawRateThreshold = 0.018;
    if (std::abs(target_yaw_rate) < kTurnYawRateThreshold) {
        return;
    }

    const int outer_floor = std::clamp(outer_min_pwm, 0, 255);
    const int inner_cap = std::clamp(inner_max_pwm, 0, std::max(0, outer_floor - 1));
    const int inner_floor = std::clamp(inner_min_pwm, 0, inner_cap);
    if (target_yaw_rate > 0.0) {
        *pwm_right = std::max(*pwm_right, outer_floor);
        *pwm_left = std::clamp(std::max(*pwm_left, 0), inner_floor, inner_cap);
    } else {
        *pwm_left = std::max(*pwm_left, outer_floor);
        *pwm_right = std::clamp(std::max(*pwm_right, 0), inner_floor, inner_cap);
    }
}

}  // namespace thesis_sim::mvc::model
