#include "mvc/model/world/scenario_model.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace thesis_sim::mvc::model {
namespace {

Vec2 scale_point_about(const Vec2& point, const Vec2& center, double scale) {
    return {
        center.x + (point.x - center.x) * scale,
        center.y + (point.y - center.y) * scale,
    };
}

Rect scale_rect_about(const Rect& rect, const Vec2& center, double scale) {
    const Vec2 minimum =
        scale_point_about({rect.min_x, rect.min_y}, center, scale);
    const Vec2 maximum =
        scale_point_about({rect.max_x, rect.max_y}, center, scale);
    return {
        std::min(minimum.x, maximum.x),
        std::min(minimum.y, maximum.y),
        std::max(minimum.x, maximum.x),
        std::max(minimum.y, maximum.y),
    };
}

WorldMap scale_world_map_about(const WorldMap& source,
                               const Vec2& center,
                               double scale) {
    if (std::abs(scale - 1.0) <= 1e-6 || scale <= 0.0) {
        return source;
    }

    WorldMap scaled = source;
    scaled.set_bounds(scale_rect_about(source.bounds(), center, scale));
    scaled.set_start(scale_point_about(source.start(), center, scale));
    scaled.set_goal(scale_point_about(source.goal(), center, scale));
    for (Rect& obstacle : scaled.editable_obstacles()) {
        obstacle = scale_rect_about(obstacle, center, scale);
    }
    for (Rect& obstacle : scaled.editable_perception_obstacles()) {
        obstacle = scale_rect_about(obstacle, center, scale);
    }
    for (DynamicObstacleSpec& obstacle : scaled.editable_dynamic_obstacles()) {
        obstacle.obstacle = scale_rect_about(obstacle.obstacle, center, scale);
        obstacle.activate_after_progress_m *= scale;
    }
    for (GateSpec& gate : scaled.editable_gates()) {
        gate.position = scale_point_about(gate.position, center, scale);
        gate.anchor_position = scale_point_about(gate.anchor_position, center, scale);
        gate.motion_amplitude.x *= scale;
        gate.motion_amplitude.y *= scale;
    }
    for (Vec2& point : scaled.editable_road_centerline()) {
        point = scale_point_about(point, center, scale);
    }
    return scaled;
}

bool rect_contains_point(const Rect& rect,
                         const Vec2& point,
                         double margin = 0.0) {
    return point.x >= rect.min_x + margin &&
           point.x <= rect.max_x - margin &&
           point.y >= rect.min_y + margin &&
           point.y <= rect.max_y - margin;
}

bool validate_hardware_structured_world(const WorldMap& world,
                                        std::string* error_message) {
    if (world.environment_mode() != EnvironmentMode::StructuredRoad) {
        return true;
    }
    const auto& road = world.road_centerline();
    if (road.size() < 6) {
        if (error_message != nullptr) {
            *error_message =
                "Structured custom map rejected: the road collapsed to too few points. Reload the structured preset and re-apply the edit.";
        }
        return false;
    }

    double min_x = road.front().x;
    double max_x = road.front().x;
    double min_y = road.front().y;
    double max_y = road.front().y;
    for (const Vec2& point : road) {
        min_x = std::min(min_x, point.x);
        max_x = std::max(max_x, point.x);
        min_y = std::min(min_y, point.y);
        max_y = std::max(max_y, point.y);
    }
    const Rect bounds = world.bounds();
    const double road_span = std::max(max_x - min_x, max_y - min_y);
    const double bounds_span = std::max(
        bounds.max_x - bounds.min_x,
        bounds.max_y - bounds.min_y);
    if (road_span < 0.12 || bounds_span < 0.24) {
        if (error_message != nullptr) {
            *error_message =
                "Structured custom map rejected: the edited road is too small for the indoor hardware viewport.";
        }
        return false;
    }
    return true;
}

bool validate_hardware_unstructured_world(const WorldMap& world,
                                          std::string* error_message) {
    if (world.environment_mode() != EnvironmentMode::UnstructuredGates) {
        return true;
    }
    const Rect bounds = world.bounds();
    if (bounds.max_x - bounds.min_x < 0.60 ||
        bounds.max_y - bounds.min_y < 0.50) {
        if (error_message != nullptr) {
            *error_message =
                "Unstructured custom map rejected: the editable area is too small for LiDAR-based gate extraction on hardware.";
        }
        return false;
    }
    if (!rect_contains_point(bounds, world.start(), 0.06) ||
        !rect_contains_point(bounds, world.goal(), 0.06)) {
        if (error_message != nullptr) {
            *error_message =
                "Unstructured custom map rejected: start or goal is too close to the map boundary for safe hardware navigation.";
        }
        return false;
    }
    return true;
}

}  // namespace

WorldMap make_world_from_mode(EnvironmentMode mode,
                              UnstructuredMapPreset unstructured_preset,
                              StructuredMapPreset structured_preset,
                              GateBehaviorMode gate_behavior,
                              std::uint32_t gate_seed,
                              int mixed_preset) {
    if (mode == EnvironmentMode::StructuredRoad) {
        return WorldMap::structured_demo(structured_preset);
    }
    if (mode == EnvironmentMode::MixedRoadGates) {
        switch (mixed_preset) {
            case 5:
                return WorldMap::mixed_tank_hardware_demo();
            case 4:
                return WorldMap::mixed_dynamic_obstacle_demo();
            case 3:
                return WorldMap::mixed_closed_obstacle_demo();
            case 2:
                return WorldMap::mixed_ideal_demo();
            case 1:
                return WorldMap::mixed_hardware_aligned_demo();
            default:
                return WorldMap::mixed_demo();
        }
    }
    if (unstructured_preset == UnstructuredMapPreset::Custom) {
        return WorldMap::unstructured_demo(
            UnstructuredMapPreset::Custom,
            GateBehaviorMode::Static,
            0);
    }
    return WorldMap::unstructured_demo(
        unstructured_preset,
        gate_behavior,
        gate_seed);
}

WorldMap hardware_mixed_world_from_preset(int preset) {
    switch (preset) {
        case 5:
            return WorldMap::mixed_tank_hardware_demo();
        case 4:
            return WorldMap::mixed_dynamic_obstacle_demo();
        case 3:
            return WorldMap::mixed_closed_obstacle_hardware_demo();
        case 2:
            return WorldMap::mixed_ideal_demo();
        case 1:
            return WorldMap::mixed_hardware_aligned_demo();
        default:
            return WorldMap::mixed_hardware_demo();
    }
}

int mixed_preset_from_world(const WorldMap& world) {
    if (world.environment_mode() != EnvironmentMode::MixedRoadGates) {
        return 0;
    }
    const Rect bounds = world.bounds();
    const double span =
        std::max(bounds.max_x - bounds.min_x, bounds.max_y - bounds.min_y);
    if (!world.dynamic_obstacles().empty()) {
        return 4;
    }
    if (world.structured_preset() == StructuredMapPreset::IdealCircle &&
        !world.obstacles().empty()) {
        return 2;
    }
    if (world.structured_preset() == StructuredMapPreset::TankCircuit) {
        return span <= 1.35 ? 5 : 3;
    }
    return span <= 1.20 && !world.obstacles().empty() ? 1 : 0;
}

int hardware_mixed_preset_from_world(const WorldMap& world) {
    if (world.environment_mode() != EnvironmentMode::MixedRoadGates) {
        return 0;
    }
    if (!world.dynamic_obstacles().empty()) {
        return 4;
    }
    if (world.structured_preset() == StructuredMapPreset::IdealCircle) {
        return 2;
    }
    const Rect bounds = world.bounds();
    const double span =
        std::max(bounds.max_x - bounds.min_x, bounds.max_y - bounds.min_y);
    if (world.structured_preset() == StructuredMapPreset::TankCircuit) {
        return span <= 1.35 ? 5 : 3;
    }
    return span <= 1.20 && !world.obstacles().empty() ? 1 : 0;
}

WorldMap scale_world_map(const WorldMap& source, double scale) {
    const Rect bounds = source.bounds();
    return scale_world_map_about(
        source,
        {(bounds.min_x + bounds.max_x) * 0.5,
         (bounds.min_y + bounds.max_y) * 0.5},
        scale);
}

Rect structured_content_bounds(const WorldMap& world) {
    Rect bounds{
        std::min(world.start().x, world.goal().x),
        std::min(world.start().y, world.goal().y),
        std::max(world.start().x, world.goal().x),
        std::max(world.start().y, world.goal().y),
    };
    const auto include_point = [&bounds](const Vec2& point) {
        bounds.min_x = std::min(bounds.min_x, point.x);
        bounds.min_y = std::min(bounds.min_y, point.y);
        bounds.max_x = std::max(bounds.max_x, point.x);
        bounds.max_y = std::max(bounds.max_y, point.y);
    };
    for (const Vec2& point : world.road_centerline()) {
        include_point(point);
    }
    for (const Rect& obstacle : world.obstacles()) {
        include_point({obstacle.min_x, obstacle.min_y});
        include_point({obstacle.max_x, obstacle.max_y});
    }
    for (const Rect& obstacle : world.perception_obstacles()) {
        include_point({obstacle.min_x, obstacle.min_y});
        include_point({obstacle.max_x, obstacle.max_y});
    }
    for (const DynamicObstacleSpec& obstacle : world.dynamic_obstacles()) {
        include_point({obstacle.obstacle.min_x, obstacle.obstacle.min_y});
        include_point({obstacle.obstacle.max_x, obstacle.obstacle.max_y});
    }
    return bounds;
}

WorldMap fit_hardware_structured_world(WorldMap world,
                                       VehicleModelKind vehicle_model) {
    if (world.environment_mode() != EnvironmentMode::StructuredRoad ||
        world.structured_preset() == StructuredMapPreset::Custom) {
        return world;
    }
    (void)vehicle_model;
    return normalize_structured_world(
        std::move(world),
        kCommonArenaSpanM,
        kStructuredRoadSpanM);
}

WorldMap fit_simulation_structured_world(WorldMap world,
                                         VehicleModelKind vehicle_model) {
    return fit_hardware_structured_world(std::move(world), vehicle_model);
}

bool validate_hardware_world(const WorldMap& world,
                             std::string* error_message) {
    return validate_hardware_structured_world(world, error_message) &&
           validate_hardware_unstructured_world(world, error_message);
}

WorldMap sanitize_hardware_unstructured_world(WorldMap world) {
    if (world.environment_mode() != EnvironmentMode::UnstructuredGates) {
        return world;
    }
    world.editable_obstacles().clear();
    world.editable_perception_obstacles().clear();
    world.editable_gates().clear();
    world.finalize_editor_changes();
    world.set_gate_behavior(GateBehaviorMode::Static, 0);
    return world;
}

}  // namespace thesis_sim::mvc::model
