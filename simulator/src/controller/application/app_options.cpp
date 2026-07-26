#include "mvc/controller/application/app_options.h"

#include <algorithm>
#include <cstdlib>
#include <optional>
#include <string>

namespace thesis_sim::mvc::controller {
namespace {

bool read_option_value(const std::string& argument,
                       const char* name,
                       int* index,
                       int argc,
                       char** argv,
                       std::string* value) {
    const std::string flag = "--" + std::string(name);
    if (argument == flag && *index + 1 < argc) {
        *value = argv[++(*index)];
        return true;
    }
    const std::string prefix = flag + "=";
    if (argument.rfind(prefix, 0) == 0) {
        *value = argument.substr(prefix.size());
        return true;
    }
    return false;
}

bool read_double_option(const std::string& argument,
                        int* index,
                        int argc,
                        char** argv,
                        const char* name,
                        std::optional<double>* destination) {
    std::string value;
    if (!read_option_value(argument, name, index, argc, argv, &value)) {
        return false;
    }
    char* end = nullptr;
    const double parsed = std::strtod(value.c_str(), &end);
    if (end != value.c_str() && end != nullptr && *end == '\0') {
        *destination = parsed;
    }
    return true;
}

void select_scenario(const std::string& value, AppOptions* options) {
    if (value == "structured") {
        options->environment_mode = EnvironmentMode::StructuredRoad;
        return;
    }
    if (value == "mixed" || value == "mixed_road_gates") {
        options->environment_mode = EnvironmentMode::MixedRoadGates;
        options->dynamic_lidar_gates = true;
        return;
    }
    options->environment_mode = EnvironmentMode::UnstructuredGates;
    const bool hardware = value == "mixed_hardware" || value == "mixed-hardware" ||
                          value == "mixed_hardware_aligned" || value == "mixed-hardware-aligned";
    const bool tank = value == "mixed_tank" || value == "mixed-tank" ||
                      value == "mixed_tank_lab" || value == "mixed-tank-lab";
    const bool obstacle = value == "mixed_obstacle" || value == "mixed-obstacle" ||
                          value == "mixed_closed_obstacle" || value == "mixed-closed-obstacle";
    const bool dynamic = value == "mixed_dynamic_obstacle" || value == "mixed-dynamic-obstacle" ||
                         value == "mixed_dynamic_obstacle_road" || value == "mixed-dynamic-obstacle-road" ||
                         value == "dynamic_obstacle" || value == "dynamic-obstacle" ||
                         value == "dynamic_obstacle_road" || value == "dynamic-obstacle-road";
    if (hardware || tank || obstacle || dynamic) {
        options->environment_mode = EnvironmentMode::MixedRoadGates;
        options->dynamic_lidar_gates = true;
        options->mixed_preset = tank ? 5 : (dynamic ? 4 : (obstacle ? 3 : 1));
    }
}

void select_mixed_map(const std::string& value, AppOptions* options) {
    options->dynamic_lidar_gates = true;
    if (value == "ideal" || value == "perfect") {
        options->mixed_preset = 2;
        options->ideal_simulation = true;
        options->calibrated_simulation = false;
    } else if (value == "obstacle" || value == "closed_obstacle" ||
               value == "closed-obstacle" || value == "obstacle_road" || value == "obstacle-road") {
        options->mixed_preset = 3;
    } else if (value == "dynamic" || value == "dynamic_obstacle" ||
               value == "dynamic-obstacle" || value == "dynamic_obstacle_road" ||
               value == "dynamic-obstacle-road" || value == "late_obstacle" || value == "late-obstacle") {
        options->mixed_preset = 4;
    } else if (value == "tank" || value == "tank_lab" || value == "tank-lab" ||
               value == "tracked" || value == "tracked_lab" || value == "tracked-lab") {
        options->mixed_preset = 5;
    } else {
        options->mixed_preset = value == "hardware" || value == "hardware_aligned" ||
                                        value == "hardware-aligned" || value == "hardware_lab" ||
                                        value == "hardware-lab"
                                    ? 1
                                    : 0;
    }
}

void select_unstructured_map(const std::string& value, AppOptions* options) {
    if (value == "tight") {
        options->unstructured_preset = UnstructuredMapPreset::TightCorridor;
    } else if (value == "slalom") {
        options->unstructured_preset = UnstructuredMapPreset::WideSlalom;
    } else if (value == "lower") {
        options->unstructured_preset = UnstructuredMapPreset::LowerBypass;
    } else if (value == "hardware_lab" || value == "hardware" || value == "lab") {
        options->unstructured_preset = UnstructuredMapPreset::HardwareLab;
    } else if (value == "ideal" || value == "perfect" ||
               value == "ideal_validation" || value == "perfect_validation") {
        options->unstructured_preset = UnstructuredMapPreset::IdealValidation;
        options->ideal_simulation = true;
        options->calibrated_simulation = false;
        options->dynamic_lidar_gates = true;
    } else {
        options->unstructured_preset = UnstructuredMapPreset::RobotValidation;
    }
}

void select_structured_map(const std::string& value, AppOptions* options) {
    if (value == "circle") {
        options->structured_preset = StructuredMapPreset::CircleLoop;
    } else if (value == "zigzag") {
        options->structured_preset = StructuredMapPreset::ZigZag;
    } else if (value == "hardware" || value == "hardware_track") {
        options->structured_preset = StructuredMapPreset::HardwareTrack;
    } else if (value == "figure_eight" || value == "figure8" || value == "eight") {
        options->structured_preset = StructuredMapPreset::FigureEight;
    } else if (value == "tank_circuit" || value == "circuit" || value == "practice_circuit") {
        options->structured_preset = StructuredMapPreset::TankCircuit;
    } else if (value == "ideal" || value == "perfect" ||
               value == "ideal_circle" || value == "perfect_circle") {
        options->structured_preset = StructuredMapPreset::IdealCircle;
        options->ideal_simulation = true;
        options->calibrated_simulation = false;
    } else {
        options->structured_preset = StructuredMapPreset::ValidationRoad;
    }
}

void select_vehicle(const std::string& value, AppOptions* options) {
    options->vehicle_model = value == "tracked" || value == "tracked_vehicle" ||
                                     value == "skid" || value == "skid_steer" || value == "tank"
                                 ? VehicleModelKind::TrackedVehicle
                                 : VehicleModelKind::CarLikeBicycle;
}

}  // namespace

AppOptions parse_app_options(int argc, char** argv) {
    AppOptions options;
    for (int i = 1; i < argc; ++i) {
        const std::string argument = argv[i];
        std::string value;
        if (argument == "--headless") {
            options.headless = true;
        } else if (argument == "--dynamic-lidar-gates" || argument == "--lidar-dynamic-gates") {
            options.dynamic_lidar_gates = true;
        } else if (argument == "--static-gates") {
            options.dynamic_lidar_gates = false;
        } else if (argument == "--ideal-sim" || argument == "--perfect-sim") {
            options.ideal_simulation = true;
            options.calibrated_simulation = false;
        } else if (argument == "--calibrated-sim") {
            options.ideal_simulation = false;
            options.calibrated_simulation = true;
        } else if (read_option_value(argument, "sim-level", &i, argc, argv, &value)) {
            options.ideal_simulation = value == "ideal" || value == "perfect";
            options.calibrated_simulation = value == "calibrated" || value == "sim-calibrated";
        } else if (read_option_value(argument, "scenario", &i, argc, argv, &value)) {
            select_scenario(value, &options);
        } else if (read_option_value(argument, "mixed-map", &i, argc, argv, &value)) {
            select_mixed_map(value, &options);
        } else if (read_option_value(argument, "unstructured-map", &i, argc, argv, &value)) {
            select_unstructured_map(value, &options);
        } else if (read_option_value(argument, "structured-map", &i, argc, argv, &value)) {
            select_structured_map(value, &options);
        } else if (read_option_value(argument, "vehicle-model", &i, argc, argv, &value)) {
            select_vehicle(value, &options);
        } else if (read_double_option(argument, &i, argc, argv, "min-effective-pwm", &options.tuning_overrides.min_effective_pwm) ||
                   read_double_option(argument, &i, argc, argv, "speed-estimate-per-pwm", &options.tuning_overrides.speed_estimate_per_pwm) ||
                   read_double_option(argument, &i, argc, argv, "pwm-slew-rate", &options.tuning_overrides.pwm_slew_rate) ||
                   read_double_option(argument, &i, argc, argv, "motor-time-constant", &options.tuning_overrides.motor_time_constant) ||
                   read_double_option(argument, &i, argc, argv, "max-linear-speed", &options.tuning_overrides.max_linear_speed) ||
                   read_double_option(argument, &i, argc, argv, "max-curvature", &options.tuning_overrides.max_curvature) ||
                   read_double_option(argument, &i, argc, argv, "max-steer-angle", &options.tuning_overrides.max_steer_angle) ||
                   read_double_option(argument, &i, argc, argv, "max-steer-rate", &options.tuning_overrides.max_steer_rate) ||
                   read_double_option(argument, &i, argc, argv, "max-yaw-rate", &options.tuning_overrides.max_yaw_rate) ||
                   read_double_option(argument, &i, argc, argv, "linear-feedback-gain", &options.tuning_overrides.linear_feedback_gain) ||
                   read_double_option(argument, &i, argc, argv, "yaw-feedback-gain", &options.tuning_overrides.yaw_feedback_gain) ||
                   read_double_option(argument, &i, argc, argv, "left-pwm-scale", &options.tuning_overrides.left_pwm_scale) ||
                   read_double_option(argument, &i, argc, argv, "right-pwm-scale", &options.tuning_overrides.right_pwm_scale) ||
                   read_double_option(argument, &i, argc, argv, "yaw-response-scale", &options.tuning_overrides.yaw_response_scale) ||
                   read_double_option(argument, &i, argc, argv, "cruise-speed-limit", &options.tuning_overrides.cruise_speed_limit)) {
        } else if (read_option_value(argument, "max-steps", &i, argc, argv, &value)) {
            options.max_steps = std::max(std::atoi(value.c_str()), 1);
        }
    }

    options.max_steps = std::max(options.max_steps, 1);
    return options;
}

}  // namespace thesis_sim::mvc::controller
