#pragma once

#include <cstdint>
#include <string>

#include "mvc/model/navigation/vehicle_dynamics.h"

namespace thesis_sim::mvc::model {

struct RobotCalibrationProfile {
    std::string name;
    std::string version;
    std::string source_path;
    std::string content_hash;
    VehicleModelKind vehicle_model = VehicleModelKind::CarLikeBicycle;

    double body_length_m = 0.0;
    double body_width_m = 0.0;
    double wheel_or_belt_length_m = 0.0;
    double wheel_or_belt_width_m = 0.0;
    double track_center_distance_m = 0.0;
    double wheel_radius_m = 0.0;
    bool wheel_radius_calibrated = false;
    std::int32_t encoder_ticks_per_revolution = 0;

    int min_effective_pwm = 0;
    double speed_estimate_per_pwm = 0.0;
    double pwm_slew_rate = 0.0;
    double motor_time_constant_s = 0.0;
    double left_actuator_scale = 1.0;
    double right_actuator_scale = 1.0;
    double max_linear_speed_mps = 0.0;
    double max_yaw_rate_rad_s = 0.0;

    int command_delay_steps = 0;
    double encoder_distance_noise_std_m = 0.0;
    double encoder_left_scale = 1.0;
    double encoder_right_scale = 1.0;
    double imu_yaw_noise_std_rad = 0.0;
    double imu_yaw_rate_noise_std_rad_s = 0.0;
    double imu_yaw_bias_walk_std_rad = 0.0;
    double lidar_range_noise_std_m = 0.0;
    double lidar_dropout_probability = 0.0;
};

bool load_robot_calibration_profile(const std::string& path,
                                    RobotCalibrationProfile* profile,
                                    std::string* error = nullptr);

}  // namespace thesis_sim::mvc::model
