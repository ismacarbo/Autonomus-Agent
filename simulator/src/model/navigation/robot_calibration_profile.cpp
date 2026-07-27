#include "mvc/model/navigation/robot_calibration_profile.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <sstream>

namespace thesis_sim::mvc::model {
namespace {

bool locate_value(const std::string& json,
                  const std::string& key,
                  std::size_t* value_start) {
    const std::string token = "\"" + key + "\"";
    const std::size_t key_pos = json.find(token);
    if (key_pos == std::string::npos) {
        return false;
    }
    const std::size_t colon = json.find(':', key_pos + token.size());
    if (colon == std::string::npos) {
        return false;
    }
    std::size_t start = colon + 1;
    while (start < json.size() &&
           std::isspace(static_cast<unsigned char>(json[start])) != 0) {
        ++start;
    }
    *value_start = start;
    return start < json.size();
}

bool read_string(const std::string& json,
                 const std::string& key,
                 std::string* value) {
    std::size_t start = 0;
    if (!locate_value(json, key, &start) || json[start] != '"') {
        return false;
    }
    const std::size_t end = json.find('"', start + 1);
    if (end == std::string::npos) {
        return false;
    }
    *value = json.substr(start + 1, end - start - 1);
    return true;
}

bool read_number(const std::string& json,
                 const std::string& key,
                 double* value) {
    std::size_t start = 0;
    if (!locate_value(json, key, &start)) {
        return false;
    }
    char* end = nullptr;
    const double parsed = std::strtod(json.c_str() + start, &end);
    if (end == json.c_str() + start || !std::isfinite(parsed)) {
        return false;
    }
    *value = parsed;
    return true;
}

bool read_bool(const std::string& json,
               const std::string& key,
               bool* value) {
    std::size_t start = 0;
    if (!locate_value(json, key, &start)) {
        return false;
    }
    if (json.compare(start, 4, "true") == 0) {
        *value = true;
        return true;
    }
    if (json.compare(start, 5, "false") == 0) {
        *value = false;
        return true;
    }
    return false;
}

std::string fnv1a_hash(const std::string& content) {
    std::uint64_t hash = 1469598103934665603ULL;
    for (const unsigned char byte : content) {
        hash ^= static_cast<std::uint64_t>(byte);
        hash *= 1099511628211ULL;
    }
    std::ostringstream out;
    out << "fnv1a64:" << std::hex << std::setw(16) << std::setfill('0') << hash;
    return out.str();
}

bool require_positive(double value,
                      const char* field,
                      std::string* error) {
    if (std::isfinite(value) && value > 0.0) {
        return true;
    }
    if (error != nullptr) {
        *error = std::string("missing or invalid positive field: ") + field;
    }
    return false;
}

}  // namespace

bool load_robot_calibration_profile(const std::string& path,
                                    RobotCalibrationProfile* profile,
                                    std::string* error) {
    if (profile == nullptr) {
        if (error != nullptr) {
            *error = "null calibration profile output";
        }
        return false;
    }
    std::ifstream input(path);
    if (!input.is_open()) {
        if (error != nullptr) {
            *error = "could not open calibration profile: " + path;
        }
        return false;
    }
    const std::string json{
        std::istreambuf_iterator<char>(input),
        std::istreambuf_iterator<char>()};

    RobotCalibrationProfile parsed;
    parsed.source_path = path;
    parsed.content_hash = fnv1a_hash(json);
    std::string model;
    if (!read_string(json, "profile_name", &parsed.name) ||
        !read_string(json, "profile_version", &parsed.version) ||
        !read_string(json, "vehicle_model", &model)) {
        if (error != nullptr) {
            *error = "profile_name, profile_version and vehicle_model are required";
        }
        return false;
    }
    parsed.vehicle_model =
        model == "tank" || model == "tracked"
            ? VehicleModelKind::TrackedVehicle
            : VehicleModelKind::CarLikeBicycle;

    read_number(json, "body_length_m", &parsed.body_length_m);
    read_number(json, "body_width_m", &parsed.body_width_m);
    read_number(json, "wheel_or_belt_length_m", &parsed.wheel_or_belt_length_m);
    read_number(json, "wheel_or_belt_width_m", &parsed.wheel_or_belt_width_m);
    read_number(json, "track_center_distance_m", &parsed.track_center_distance_m);
    read_number(json, "wheel_radius_m", &parsed.wheel_radius_m);
    read_bool(json, "wheel_radius_calibrated", &parsed.wheel_radius_calibrated);
    read_bool(json,
              "controller_motor_channels_swapped",
              &parsed.controller_motor_channels_swapped);
    double encoder_ticks = 0.0;
    read_number(json, "encoder_ticks_per_revolution", &encoder_ticks);
    parsed.encoder_ticks_per_revolution = static_cast<std::int32_t>(std::llround(encoder_ticks));
    double min_pwm = 0.0;
    read_number(json, "min_effective_pwm", &min_pwm);
    parsed.min_effective_pwm = static_cast<int>(std::lround(min_pwm));
    read_number(json, "speed_estimate_per_pwm", &parsed.speed_estimate_per_pwm);
    read_number(json, "pwm_slew_rate", &parsed.pwm_slew_rate);
    read_number(json, "motor_time_constant_s", &parsed.motor_time_constant_s);
    read_number(json, "left_actuator_scale", &parsed.left_actuator_scale);
    read_number(json, "right_actuator_scale", &parsed.right_actuator_scale);
    read_number(json, "max_linear_speed_mps", &parsed.max_linear_speed_mps);
    read_number(json, "max_yaw_rate_rad_s", &parsed.max_yaw_rate_rad_s);
    double command_delay = 0.0;
    read_number(json, "command_delay_steps", &command_delay);
    parsed.command_delay_steps = std::max(0, static_cast<int>(std::lround(command_delay)));
    read_number(json, "encoder_distance_noise_std_m", &parsed.encoder_distance_noise_std_m);
    read_number(json, "encoder_left_scale", &parsed.encoder_left_scale);
    read_number(json, "encoder_right_scale", &parsed.encoder_right_scale);
    read_number(json, "imu_yaw_noise_std_rad", &parsed.imu_yaw_noise_std_rad);
    read_number(json, "imu_yaw_rate_noise_std_rad_s", &parsed.imu_yaw_rate_noise_std_rad_s);
    read_number(json, "imu_yaw_bias_walk_std_rad", &parsed.imu_yaw_bias_walk_std_rad);
    read_number(json, "lidar_range_noise_std_m", &parsed.lidar_range_noise_std_m);
    read_number(json, "lidar_dropout_probability", &parsed.lidar_dropout_probability);

    if (!require_positive(parsed.body_length_m, "body_length_m", error) ||
        !require_positive(parsed.body_width_m, "body_width_m", error) ||
        !require_positive(parsed.wheel_radius_m, "wheel_radius_m", error) ||
        parsed.encoder_ticks_per_revolution <= 0) {
        if (error != nullptr && parsed.encoder_ticks_per_revolution <= 0) {
            *error = "missing or invalid positive field: encoder_ticks_per_revolution";
        }
        return false;
    }
    if (parsed.lidar_dropout_probability < 0.0 ||
        parsed.lidar_dropout_probability >= 1.0) {
        if (error != nullptr) {
            *error = "lidar_dropout_probability must be in [0,1)";
        }
        return false;
    }
    *profile = parsed;
    if (error != nullptr) {
        error->clear();
    }
    return true;
}

}  // namespace thesis_sim::mvc::model
