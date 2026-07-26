#pragma once

#include <cstdint>
#include <string>

#include "mvc/controller/simulation_planner/simulator.h"

namespace thesis_sim::mvc::controller {

struct AppOptions {
    bool headless = false;
    bool dynamic_lidar_gates = false;
    int max_steps = 6000;
    EnvironmentMode environment_mode = EnvironmentMode::StructuredRoad;
    UnstructuredMapPreset unstructured_preset = UnstructuredMapPreset::IdealValidation;
    StructuredMapPreset structured_preset = StructuredMapPreset::ValidationRoad;
    int mixed_preset = 1;
    bool ideal_simulation = false;
    bool calibrated_simulation = false;
    VehicleModelKind vehicle_model = VehicleModelKind::CarLikeBicycle;
    std::uint32_t simulation_seed = 20260711U;
    std::string calibration_profile_path;
    VehicleTuningOverrides tuning_overrides;
};

AppOptions parse_app_options(int argc, char** argv);

}  // namespace thesis_sim::mvc::controller
