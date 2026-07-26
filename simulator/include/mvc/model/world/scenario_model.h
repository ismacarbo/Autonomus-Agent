#pragma once

#include <cstdint>
#include <string>

#include "mvc/model/world/world.h"

namespace thesis_sim {
enum class VehicleModelKind;
}

namespace thesis_sim::mvc::model {

constexpr double kCommonArenaSpanM = 1.20;
constexpr double kStructuredRoadSpanM = 0.90;
constexpr double kUnstructuredHardwareContentSpanM = 0.80;

WorldMap make_world_from_mode(EnvironmentMode mode,
                              UnstructuredMapPreset unstructured_preset,
                              StructuredMapPreset structured_preset,
                              GateBehaviorMode gate_behavior,
                              std::uint32_t gate_seed,
                              int mixed_preset = 0);

WorldMap hardware_mixed_world_from_preset(int preset);
int mixed_preset_from_world(const WorldMap& world);
int hardware_mixed_preset_from_world(const WorldMap& world);

WorldMap scale_world_map(const WorldMap& source, double scale);
Rect structured_content_bounds(const WorldMap& world);
WorldMap fit_hardware_structured_world(WorldMap world,
                                       VehicleModelKind vehicle_model);
WorldMap fit_hardware_unstructured_world(WorldMap world);
WorldMap fit_simulation_structured_world(WorldMap world,
                                         VehicleModelKind vehicle_model);

bool validate_hardware_world(const WorldMap& world,
                             std::string* error_message);
WorldMap sanitize_hardware_unstructured_world(WorldMap world);

}  // namespace thesis_sim::mvc::model
