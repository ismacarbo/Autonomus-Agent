#include <SDL.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include "imgui.h"
#include "imgui_impl_sdl2.h"
#include "imgui_impl_sdlrenderer2.h"
#include "implot.h"

#include "live_view_stream.h"
#include "planner_sim.h"

namespace {

using thesis_sim::GateSpec;
using thesis_sim::GateBehaviorMode;
using thesis_sim::EnvironmentMode;
using thesis_sim::HardwareTelemetrySample;
using thesis_sim::LidarHit;
using thesis_sim::LiveFrameSnapshot;
using thesis_sim::LiveGateFrame;
using thesis_sim::LiveSceneSnapshot;
using thesis_sim::LiveVehicleState;
using thesis_sim::LiveViewStreamServer;
using thesis_sim::PlannerDrivenVehicleSim;
using thesis_sim::RangeSensorProfile;
using thesis_sim::Rect;
using thesis_sim::SimulationReport;
using thesis_sim::StructuredMapPreset;
using thesis_sim::TelemetrySample;
using thesis_sim::TrackingControllerMode;
using thesis_sim::UnstructuredMapPreset;
using thesis_sim::Vec2;
using thesis_sim::VehicleModelKind;
using thesis_sim::VehicleSnapshot;
using thesis_sim::VehicleTuningOverrides;
using thesis_sim::WheelPose;
using thesis_sim::WorldMap;

struct AppOptions {
    bool headless = false;
    bool dynamic_lidar_gates = false;
    int max_steps = 6000;
    EnvironmentMode environment_mode = EnvironmentMode::StructuredRoad;
    UnstructuredMapPreset unstructured_preset = UnstructuredMapPreset::RobotValidation;
    StructuredMapPreset structured_preset = StructuredMapPreset::ValidationRoad;
    int mixed_preset = 0;
    bool ideal_simulation = false;
    VehicleModelKind vehicle_model = VehicleModelKind::CarLikeBicycle;
    VehicleTuningOverrides tuning_overrides;
};

struct CanvasTransform {
    ImVec2 origin;
    float scale = 1.0f;
};

enum WorkspaceSource {
    kWorkspaceSourceSimulation = 0,
    kWorkspaceSourceHardwarePlanner = 1,
};

enum WorkspaceView {
    kWorkspaceViewMission = 0,
    kWorkspaceViewAnalytics = 1,
    kWorkspaceViewMap = 2,
    kWorkspaceViewDiagnostics = 3,
    kWorkspaceViewExport = 4,
};

constexpr double kHardwareStructuredMaxSpanM = 0.40;
constexpr double kTankHardwareStructuredMaxSpanM = 1.00;
constexpr double kTankStructuredBaselineRoadSpanM = 0.68;
constexpr float kHardwareTrackDefaultScale = 1.00f;
constexpr float kHardwareTrackMinScale = 0.70f;
constexpr float kHardwareTrackMaxScale = 1.20f;

enum class MapEditorHandleType {
    None = 0,
    Start,
    Goal,
    Obstacle,
    Gate,
    RoadPoint,
};

struct MapEditorHandle {
    MapEditorHandleType type = MapEditorHandleType::None;
    int index = -1;
};

struct UiState {
    bool paused = true;
    bool single_step = false;
    int steps_per_frame = 1;
    int workspace_source = kWorkspaceSourceHardwarePlanner;
    int workspace_tab = 0;
    int workspace_view = kWorkspaceViewMission;
    int simulation_panel_tab = 0;
    int hardware_panel_tab = 0;
    int requested_simulation_panel_tab = -1;
    int requested_hardware_panel_tab = -1;
    int hardware_listen_port = 9559;
    int hardware_environment_mode = static_cast<int>(EnvironmentMode::StructuredRoad);
    int hardware_unstructured_preset = static_cast<int>(UnstructuredMapPreset::Custom);
    int hardware_structured_preset = static_cast<int>(StructuredMapPreset::ValidationRoad);
    int hardware_vehicle_model = static_cast<int>(VehicleModelKind::CarLikeBicycle);
    float hardware_track_scale = kHardwareTrackDefaultScale;
    bool show_grid = true;
    bool show_trails = true;
    bool show_lidar_rays = true;
    bool show_lidar_hits = true;
    bool show_gate_labels = true;
    bool show_world_hud = true;
    bool dynamic_lidar_gates = false;
    int gate_seed_input = 7;
    int environment_mode = static_cast<int>(EnvironmentMode::StructuredRoad);
    int unstructured_preset = static_cast<int>(UnstructuredMapPreset::RobotValidation);
    int structured_preset = static_cast<int>(StructuredMapPreset::ValidationRoad);
    int mixed_preset = 0;
    int vehicle_model = static_cast<int>(VehicleModelKind::CarLikeBicycle);
    bool ideal_simulation = false;
    std::string last_validation_benchmark;
    WorldMap scenario_editor_world;
    bool scenario_editor_dirty = false;
    WorldMap hardware_editor_world;
    bool hardware_editor_dirty = false;
    bool map_editor_enabled = false;
    MapEditorHandle selected_editor_handle;
    MapEditorHandle active_drag_handle;
    Vec2 drag_offset;
    bool report_written = false;
    std::string last_report_path;
    std::string last_csv_report_path;
    std::string last_markdown_report_path;
    std::string last_export_error;
    bool hardware_report_written = false;
    std::string last_hardware_report_path;
    std::string last_hardware_csv_report_path;
    std::string last_hardware_markdown_report_path;
    std::string last_hardware_report_error;
    std::string last_hardware_world_path;
    std::string last_hardware_world_error;
    bool hardware_world_sync_pending = false;
    bool hardware_world_sync_ok = false;
    bool hardware_stream_connected_prev = false;
    std::string last_hardware_world_sync_status;
};

struct HardwareViewerState {
    bool has_scene = false;
    LiveSceneSnapshot scene;
    LiveFrameSnapshot frame;
    std::vector<thesis_sim::HardwareTelemetrySample> history;
};

std::string hardware_launch_hint(const UiState& ui_state);

bool simulation_uses_dynamic_gates(const PlannerDrivenVehicleSim& sim) {
    return (sim.environment_mode() == EnvironmentMode::UnstructuredGates ||
            sim.environment_mode() == EnvironmentMode::MixedRoadGates) &&
           sim.config().dynamic_lidar_gates;
}

std::string simulation_gate_label(const PlannerDrivenVehicleSim& sim, int index) {
    if (index < 0 || index >= static_cast<int>(sim.gates().size())) {
        return "none";
    }
    if (!simulation_uses_dynamic_gates(sim) &&
        index < static_cast<int>(sim.world().gates().size())) {
        return sim.world().gates()[static_cast<std::size_t>(index)].name;
    }
    return index == sim.chosen_gate_index() ? "LiDAR gap" : ("LiDAR gap " + std::to_string(index + 1));
}

Vec2 simulation_gate_position(const PlannerDrivenVehicleSim& sim, std::size_t index) {
    if (!simulation_uses_dynamic_gates(sim) && index < sim.world().gates().size()) {
        return sim.world().gates()[index].position;
    }
    if (index < sim.gates().size()) {
        return {sim.gates()[index].x_pos, sim.gates()[index].y_pos};
    }
    return {};
}

bool simulation_gate_is_final(const PlannerDrivenVehicleSim& sim, std::size_t index) {
    return !simulation_uses_dynamic_gates(sim) &&
           index < sim.world().gates().size() &&
           sim.world().gates()[index].final;
}

struct OverlayLine {
    std::string text;
    ImU32 color = IM_COL32(233, 236, 229, 255);
};

struct PlotHoverSeries {
    const std::vector<double>* values = nullptr;
    const char* label = nullptr;
    ImVec4 color = ImVec4(1.0f, 1.0f, 1.0f, 1.0f);
};

WorldMap make_world_from_mode(EnvironmentMode mode,
                              UnstructuredMapPreset preset,
                              StructuredMapPreset structured_preset,
                              GateBehaviorMode gate_behavior,
                              std::uint32_t gate_seed,
                              int mixed_preset);
bool workspace_source_is_hardware(int source);

Vec2 scale_point_about(const Vec2& point, const Vec2& center, double scale) {
    return {
        center.x + (point.x - center.x) * scale,
        center.y + (point.y - center.y) * scale,
    };
}

Vec2 transform_point_about(const Vec2& point, const Vec2& source_center, const Vec2& target_center, double scale) {
    return {
        target_center.x + (point.x - source_center.x) * scale,
        target_center.y + (point.y - source_center.y) * scale,
    };
}

Rect scale_rect_about(const Rect& rect, const Vec2& center, double scale) {
    const Vec2 min_point = scale_point_about({rect.min_x, rect.min_y}, center, scale);
    const Vec2 max_point = scale_point_about({rect.max_x, rect.max_y}, center, scale);
    return {
        std::min(min_point.x, max_point.x),
        std::min(min_point.y, max_point.y),
        std::max(min_point.x, max_point.x),
        std::max(min_point.y, max_point.y),
    };
}

Rect transform_rect_about(const Rect& rect, const Vec2& source_center, const Vec2& target_center, double scale) {
    const Vec2 min_point = transform_point_about({rect.min_x, rect.min_y}, source_center, target_center, scale);
    const Vec2 max_point = transform_point_about({rect.max_x, rect.max_y}, source_center, target_center, scale);
    return {
        std::min(min_point.x, max_point.x),
        std::min(min_point.y, max_point.y),
        std::max(min_point.x, max_point.x),
        std::max(min_point.y, max_point.y),
    };
}

Rect structured_content_bounds(const WorldMap& world) {
    Rect bounds{
        std::min(world.start().x, world.goal().x),
        std::min(world.start().y, world.goal().y),
        std::max(world.start().x, world.goal().x),
        std::max(world.start().y, world.goal().y),
    };
    const auto include_point = [&](const Vec2& point) {
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
    return bounds;
}

WorldMap scale_world_map_about(const WorldMap& source, const Vec2& center, double scale) {
    if (std::abs(scale - 1.0) <= 1e-6 || scale <= 0.0) {
        return source;
    }

    WorldMap scaled = source;
    const Rect bounds = source.bounds();

    scaled.set_bounds(scale_rect_about(bounds, center, scale));
    scaled.set_start(scale_point_about(source.start(), center, scale));
    scaled.set_goal(scale_point_about(source.goal(), center, scale));

    for (Rect& obstacle : scaled.editable_obstacles()) {
        obstacle = scale_rect_about(obstacle, center, scale);
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

WorldMap transform_world_map_about(const WorldMap& source,
                                   const Vec2& source_center,
                                   const Vec2& target_center,
                                   double scale) {
    if (scale <= 0.0) {
        return source;
    }

    WorldMap transformed = source;
    const Rect bounds = source.bounds();

    transformed.set_bounds(transform_rect_about(bounds, source_center, target_center, scale));
    transformed.set_start(transform_point_about(source.start(), source_center, target_center, scale));
    transformed.set_goal(transform_point_about(source.goal(), source_center, target_center, scale));

    for (Rect& obstacle : transformed.editable_obstacles()) {
        obstacle = transform_rect_about(obstacle, source_center, target_center, scale);
    }
    for (GateSpec& gate : transformed.editable_gates()) {
        gate.position = transform_point_about(gate.position, source_center, target_center, scale);
        gate.anchor_position = transform_point_about(gate.anchor_position, source_center, target_center, scale);
        gate.motion_amplitude.x *= scale;
        gate.motion_amplitude.y *= scale;
    }
    for (Vec2& point : transformed.editable_road_centerline()) {
        point = transform_point_about(point, source_center, target_center, scale);
    }

    return transformed;
}

WorldMap scale_world_map(const WorldMap& source, double scale) {
    const Rect bounds = source.bounds();
    const Vec2 center{
        (bounds.min_x + bounds.max_x) * 0.5,
        (bounds.min_y + bounds.max_y) * 0.5,
    };
    return scale_world_map_about(source, center, scale);
}

WorldMap fit_hardware_structured_world(WorldMap world, VehicleModelKind vehicle_model) {
    if (world.environment_mode() != EnvironmentMode::StructuredRoad) {
        return world;
    }

    const bool tank_model = vehicle_model == VehicleModelKind::TrackedVehicle;
    const double structured_max_span_m =
        tank_model ? kTankHardwareStructuredMaxSpanM : kHardwareStructuredMaxSpanM;
    const double road_edge_margin_m = tank_model ? 0.16 : 0.04;
    const Rect content = structured_content_bounds(world);
    const double content_span = std::max(content.max_x - content.min_x, content.max_y - content.min_y);
    const double target_content_span =
        std::max(tank_model ? 0.68 : 0.14, structured_max_span_m - 2.0 * road_edge_margin_m);
    const Vec2 center{
        (content.min_x + content.max_x) * 0.5,
        (content.min_y + content.max_y) * 0.5,
    };
    const Vec2 target_center =
        tank_model ? Vec2{0.5 * structured_max_span_m, 0.5 * structured_max_span_m} : center;
    double scale = 1.0;
    if (content_span > target_content_span) {
        scale = target_content_span / content_span;
    } else if (tank_model && content_span < 0.68) {
        scale = 0.68 / std::max(content_span, 1e-6);
    }
    WorldMap fitted = transform_world_map_about(world, center, target_center, scale);
    const double half_span = 0.5 * structured_max_span_m;
    fitted.set_bounds({
        target_center.x - half_span,
        target_center.y - half_span,
        target_center.x + half_span,
        target_center.y + half_span,
    });
    return fitted;
}

bool tank_structured_baseline_preset(StructuredMapPreset preset) {
    return preset == StructuredMapPreset::ValidationRoad ||
           preset == StructuredMapPreset::IdealCircle;
}

WorldMap fit_simulation_structured_world(WorldMap world, VehicleModelKind vehicle_model) {
    if (vehicle_model != VehicleModelKind::TrackedVehicle ||
        world.environment_mode() != EnvironmentMode::StructuredRoad ||
        !tank_structured_baseline_preset(world.structured_preset())) {
        return world;
    }

    const Rect content = structured_content_bounds(world);
    const double content_span = std::max(content.max_x - content.min_x, content.max_y - content.min_y);
    if (content_span <= 1e-6) {
        return world;
    }

    const Vec2 source_center{
        (content.min_x + content.max_x) * 0.5,
        (content.min_y + content.max_y) * 0.5,
    };
    const Vec2 target_center{0.5 * kTankHardwareStructuredMaxSpanM, 0.5 * kTankHardwareStructuredMaxSpanM};
    WorldMap fitted = transform_world_map_about(
        world,
        source_center,
        target_center,
        kTankStructuredBaselineRoadSpanM / content_span);
    fitted.set_bounds({0.0, 0.0, kTankHardwareStructuredMaxSpanM, kTankHardwareStructuredMaxSpanM});
    return fitted;
}

WorldMap apply_hardware_track_scale(const UiState& ui_state, WorldMap world) {
    const EnvironmentMode selected_mode = static_cast<EnvironmentMode>(ui_state.hardware_environment_mode);
    const StructuredMapPreset selected_structured_preset =
        static_cast<StructuredMapPreset>(ui_state.hardware_structured_preset);
    const VehicleModelKind selected_vehicle_model =
        static_cast<VehicleModelKind>(ui_state.hardware_vehicle_model);
    if (selected_mode == EnvironmentMode::StructuredRoad &&
        selected_structured_preset == StructuredMapPreset::HardwareTrack) {
        const double scale = std::clamp(
            static_cast<double>(ui_state.hardware_track_scale),
            static_cast<double>(kHardwareTrackMinScale),
            static_cast<double>(kHardwareTrackMaxScale));
        return fit_hardware_structured_world(scale_world_map(world, scale), selected_vehicle_model);
    }
    return fit_hardware_structured_world(std::move(world), selected_vehicle_model);
}

bool validate_hardware_structured_world(const WorldMap& world, std::string* error_message) {
    if (world.environment_mode() != EnvironmentMode::StructuredRoad) {
        return true;
    }

    const auto& road = world.road_centerline();
    if (road.size() < 6) {
        if (error_message != nullptr) {
            *error_message = "Structured custom map rejected: the road collapsed to too few points. Reload the structured preset and re-apply the edit.";
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

    const double road_span_x = max_x - min_x;
    const double road_span_y = max_y - min_y;
    const Rect bounds = world.bounds();
    const double bounds_span_x = bounds.max_x - bounds.min_x;
    const double bounds_span_y = bounds.max_y - bounds.min_y;
    if (std::max(road_span_x, road_span_y) < 0.12 || std::max(bounds_span_x, bounds_span_y) < 0.24) {
        if (error_message != nullptr) {
            *error_message = "Structured custom map rejected: the edited road is too small for the indoor hardware viewport.";
        }
        return false;
    }
    return true;
}

bool rect_contains_point(const Rect& rect, const Vec2& point, double margin = 0.0) {
    return point.x >= rect.min_x + margin && point.x <= rect.max_x - margin &&
           point.y >= rect.min_y + margin && point.y <= rect.max_y - margin;
}

bool validate_hardware_unstructured_world(const WorldMap& world, std::string* error_message) {
    if (world.environment_mode() != EnvironmentMode::UnstructuredGates) {
        return true;
    }

    const Rect bounds = world.bounds();
    const double span_x = bounds.max_x - bounds.min_x;
    const double span_y = bounds.max_y - bounds.min_y;
    if (span_x < 0.60 || span_y < 0.50) {
        if (error_message != nullptr) {
            *error_message =
                "Unstructured custom map rejected: the editable area is too small for LiDAR-based gate extraction on hardware.";
        }
        return false;
    }

    if (!rect_contains_point(bounds, world.start(), 0.06) || !rect_contains_point(bounds, world.goal(), 0.06)) {
        if (error_message != nullptr) {
            *error_message =
                "Unstructured custom map rejected: start or goal is too close to the map boundary for safe hardware navigation.";
        }
        return false;
    }

    return true;
}

bool validate_hardware_world(const WorldMap& world, std::string* error_message) {
    return validate_hardware_structured_world(world, error_message) &&
           validate_hardware_unstructured_world(world, error_message);
}

WorldMap sanitize_hardware_unstructured_world(WorldMap world) {
    if (world.environment_mode() != EnvironmentMode::UnstructuredGates) {
        return world;
    }

    world.editable_obstacles().clear();
    world.editable_gates().clear();
    world.finalize_editor_changes();
    world.set_gate_behavior(GateBehaviorMode::Static, 0);
    return world;
}

constexpr ImU32 kColorCanvas = IM_COL32(19, 24, 28, 255);
constexpr ImU32 kColorGrid = IM_COL32(39, 49, 57, 255);
constexpr ImU32 kColorBounds = IM_COL32(148, 162, 170, 255);
constexpr ImU32 kColorObstacle = IM_COL32(86, 95, 105, 255);
constexpr ImU32 kColorTrail = IM_COL32(95, 186, 255, 255);
constexpr ImU32 kColorEstimateTrail = IM_COL32(255, 183, 77, 255);
constexpr ImU32 kColorTrajectory = IM_COL32(255, 233, 118, 255);
constexpr ImU32 kColorLidar = IM_COL32(132, 232, 147, 145);
constexpr ImU32 kColorLidarMiss = IM_COL32(108, 188, 255, 95);
constexpr ImU32 kColorLidarHit = IM_COL32(214, 255, 184, 255);
constexpr ImU32 kColorGate = IM_COL32(73, 198, 236, 255);
constexpr ImU32 kColorGateVisible = IM_COL32(255, 196, 61, 255);
constexpr ImU32 kColorGatePassed = IM_COL32(130, 138, 148, 255);
constexpr ImU32 kColorGoal = IM_COL32(248, 109, 76, 255);
constexpr ImU32 kColorBody = IM_COL32(241, 239, 228, 255);
constexpr ImU32 kColorWheel = IM_COL32(52, 55, 61, 255);
constexpr ImU32 kColorHeading = IM_COL32(255, 142, 79, 255);
constexpr ImU32 kColorEditorOverlay = IM_COL32(156, 196, 255, 210);
constexpr ImU32 kColorEditorHandle = IM_COL32(255, 120, 120, 255);
constexpr ImU32 kColorEditorSelected = IM_COL32(255, 234, 120, 255);

void help_marker(const char* description) {
    ImGui::TextDisabled("(?)");
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort) && description != nullptr) {
        ImGui::BeginTooltip();
        ImGui::PushTextWrapPos(ImGui::GetFontSize() * 24.0f);
        ImGui::TextUnformatted(description);
        ImGui::PopTextWrapPos();
        ImGui::EndTooltip();
    }
}

void metric_chip(const char* label, const char* value) {
    ImGui::BeginGroup();
    ImGui::TextDisabled("%s", label);
    ImGui::TextUnformatted(value);
    ImGui::EndGroup();
}

void status_line(const char* label, const char* value, const ImVec4& color) {
    ImGui::TextDisabled("%s", label);
    ImGui::SameLine(128.0f);
    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    const ImVec2 pos = ImGui::GetCursorScreenPos();
    const float radius = 4.0f;
    const float mid_y = pos.y + ImGui::GetTextLineHeight() * 0.5f;
    draw_list->AddCircleFilled(
        ImVec2(pos.x + radius, mid_y),
        radius,
        ImGui::ColorConvertFloat4ToU32(color));
    ImGui::Dummy(ImVec2(radius * 2.0f + 6.0f, ImGui::GetTextLineHeight()));
    ImGui::SameLine(0.0f, 0.0f);
    ImGui::PushStyleColor(ImGuiCol_Text, color);
    ImGui::TextUnformatted(value);
    ImGui::PopStyleColor();
}

ImGuiTabItemFlags requested_tab_flags(int requested_tab, int tab) {
    return requested_tab == tab ? ImGuiTabItemFlags_SetSelected : 0;
}

void consume_requested_tab(int* requested_tab, int tab) {
    if (requested_tab != nullptr && *requested_tab == tab) {
        *requested_tab = -1;
    }
}

void activate_simulation_panel_tab(UiState* ui_state, int tab, int workspace_view, bool editor_mode) {
    if (ui_state == nullptr) {
        return;
    }
    const bool changed = ui_state->simulation_panel_tab != tab;
    ui_state->simulation_panel_tab = tab;
    if (changed) {
        ui_state->workspace_view = workspace_view;
        ui_state->map_editor_enabled = editor_mode;
    }
}

void activate_hardware_panel_tab(UiState* ui_state, int tab, int workspace_view, bool editor_mode) {
    if (ui_state == nullptr) {
        return;
    }
    const bool changed = ui_state->hardware_panel_tab != tab;
    ui_state->hardware_panel_tab = tab;
    if (changed) {
        ui_state->workspace_view = workspace_view;
        ui_state->map_editor_enabled = editor_mode;
    }
}

void metric_card(const char* id,
                 const char* label,
                 const char* value,
                 const char* detail,
                 const ImVec4& accent,
                 float height = 76.0f) {
    ImGui::PushID(id);
    ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 8.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(12.0f, 10.0f));
    ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.08f, 0.11f, 0.14f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0.22f, 0.28f, 0.33f, 1.0f));
    if (ImGui::BeginChild("##metric_card", ImVec2(0.0f, height), true)) {
        ImDrawList* draw_list = ImGui::GetWindowDrawList();
        const ImVec2 pos = ImGui::GetWindowPos();
        const ImVec2 size = ImGui::GetWindowSize();
        draw_list->AddRectFilled(
            pos,
            ImVec2(pos.x + size.x, pos.y + 4.0f),
            ImGui::ColorConvertFloat4ToU32(accent),
            8.0f);

        ImGui::TextDisabled("%s", label);
        ImGui::PushStyleColor(ImGuiCol_Text, accent);
        ImGui::TextUnformatted(value);
        ImGui::PopStyleColor();
        if (detail != nullptr && detail[0] != '\0') {
            ImGui::TextWrapped("%s", detail);
        }
    }
    ImGui::EndChild();
    ImGui::PopStyleColor(2);
    ImGui::PopStyleVar(2);
    ImGui::PopID();
}

void draw_overlay_panel(ImDrawList* draw_list, const ImVec2& pos, float width, const std::vector<OverlayLine>& lines) {
    if (draw_list == nullptr || lines.empty()) {
        return;
    }

    const float line_height = ImGui::GetFontSize() + 5.0f;
    const float pad_x = 12.0f;
    const float pad_y = 10.0f;
    const float height = pad_y * 2.0f + line_height * static_cast<float>(lines.size());

    draw_list->AddRectFilled(pos,
                             ImVec2(pos.x + width, pos.y + height),
                             IM_COL32(8, 12, 17, 220),
                             8.0f);
    draw_list->AddRect(pos,
                       ImVec2(pos.x + width, pos.y + height),
                       IM_COL32(86, 99, 108, 200),
                       8.0f,
                       0,
                       1.0f);

    float cursor_y = pos.y + pad_y;
    for (const OverlayLine& line : lines) {
        draw_list->AddText(ImVec2(pos.x + pad_x, cursor_y), line.color, line.text.c_str());
        cursor_y += line_height;
    }
}

void render_plot_hover_overlay(const char* title,
                               const std::vector<double>& x,
                               std::initializer_list<PlotHoverSeries> series_list);
const char* workspace_source_label(int source);

size_t plot_sample_stride(size_t point_count) {
    constexpr size_t kMaxPlotPoints = 600;
    if (point_count <= kMaxPlotPoints) {
        return 1;
    }
    return std::max<size_t>(1, point_count / kMaxPlotPoints);
}

WorldMap make_world_from_mode(EnvironmentMode mode,
                              UnstructuredMapPreset preset,
                              StructuredMapPreset structured_preset,
                              GateBehaviorMode gate_behavior,
                              std::uint32_t gate_seed,
                              int mixed_preset = 0);

std::string json_escape(const std::string& value) {
    std::string escaped;
    escaped.reserve(value.size() + 8);
    for (const char ch : value) {
        switch (ch) {
            case '\\':
                escaped += "\\\\";
                break;
            case '"':
                escaped += "\\\"";
                break;
            case '\n':
                escaped += "\\n";
                break;
            case '\r':
                escaped += "\\r";
                break;
            case '\t':
                escaped += "\\t";
                break;
            default:
                escaped.push_back(ch);
                break;
        }
    }
    return escaped;
}

std::string slugify(std::string value) {
    for (char& ch : value) {
        if ((ch >= 'A' && ch <= 'Z')) {
            ch = static_cast<char>(ch - 'A' + 'a');
        } else if (!((ch >= 'a' && ch <= 'z') || (ch >= '0' && ch <= '9'))) {
            ch = '_';
        }
    }
    return value;
}

const char* environment_mode_slug(EnvironmentMode mode) {
    switch (mode) {
        case EnvironmentMode::StructuredRoad:
            return "structured";
        case EnvironmentMode::MixedRoadGates:
            return "mixed";
        case EnvironmentMode::UnstructuredGates:
        default:
            return "unstructured";
    }
}

std::string map_preset_name(const WorldMap& world) {
    if (world.environment_mode() == EnvironmentMode::StructuredRoad) {
        return thesis_sim::structured_map_preset_name(world.structured_preset());
    }
    if (world.environment_mode() == EnvironmentMode::MixedRoadGates) {
        const Rect& bounds = world.bounds();
        const double span = std::max(bounds.max_x - bounds.min_x, bounds.max_y - bounds.min_y);
        if (world.structured_preset() == StructuredMapPreset::IdealCircle && !world.obstacles().empty()) {
            return "Mixed Ideal Hardware-Aligned";
        }
        if (span <= 1.20 && !world.obstacles().empty()) {
            return "Mixed Hardware Aligned";
        }
        if (span <= 1.20) {
            return "Mixed Hardware Runner";
        }
        return "Mixed Road Gate Validation";
    }
    return thesis_sim::unstructured_map_preset_name(world.unstructured_preset());
}

const char* mixed_map_preset_name(int preset) {
    switch (preset) {
        case 2:
            return "Ideal Hardware-Aligned";
        case 1:
            return "Hardware-Aligned Lab";
        case 0:
        default:
            return "Validation Road/Gates";
    }
}

int mixed_preset_from_world(const WorldMap& world) {
    if (world.environment_mode() != EnvironmentMode::MixedRoadGates) {
        return 0;
    }
    const Rect& bounds = world.bounds();
    const double span = std::max(bounds.max_x - bounds.min_x, bounds.max_y - bounds.min_y);
    if (world.structured_preset() == StructuredMapPreset::IdealCircle && !world.obstacles().empty()) {
        return 2;
    }
    return span <= 1.20 && !world.obstacles().empty() ? 1 : 0;
}

void sync_ui_state_from_sim(UiState* ui_state, const PlannerDrivenVehicleSim& sim) {
    if (ui_state == nullptr) {
        return;
    }
    ui_state->environment_mode = static_cast<int>(sim.environment_mode());
    ui_state->unstructured_preset = static_cast<int>(sim.world().unstructured_preset());
    ui_state->structured_preset = static_cast<int>(sim.world().structured_preset());
    ui_state->mixed_preset = mixed_preset_from_world(sim.world());
    ui_state->vehicle_model = static_cast<int>(sim.vehicle_model_kind());
    ui_state->ideal_simulation = sim.config().ideal_conditions;
    ui_state->dynamic_lidar_gates = sim.config().dynamic_lidar_gates;
    ui_state->gate_seed_input = static_cast<int>(sim.gate_seed());
    ui_state->last_validation_benchmark.clear();
    ui_state->scenario_editor_world = sim.world();
    ui_state->scenario_editor_dirty = false;
    ui_state->selected_editor_handle = {};
    ui_state->active_drag_handle = {};
    ui_state->drag_offset = {};
    ui_state->report_written = false;
    ui_state->last_report_path.clear();
    ui_state->last_csv_report_path.clear();
    ui_state->last_markdown_report_path.clear();
    ui_state->last_export_error.clear();
    ui_state->hardware_report_written = false;
    ui_state->last_hardware_report_path.clear();
    ui_state->last_hardware_csv_report_path.clear();
    ui_state->last_hardware_markdown_report_path.clear();
    ui_state->last_hardware_report_error.clear();
    ui_state->last_hardware_world_path.clear();
    ui_state->last_hardware_world_error.clear();
    ui_state->hardware_editor_world = make_world_from_mode(
        static_cast<EnvironmentMode>(ui_state->hardware_environment_mode),
        static_cast<UnstructuredMapPreset>(ui_state->hardware_unstructured_preset),
        static_cast<StructuredMapPreset>(ui_state->hardware_structured_preset),
        GateBehaviorMode::Static,
        0);
    ui_state->hardware_editor_dirty = false;
}

WorldMap world_from_ui_selection(const PlannerDrivenVehicleSim& sim, const UiState& ui_state) {
    const EnvironmentMode selected_mode = static_cast<EnvironmentMode>(ui_state.environment_mode);
    const UnstructuredMapPreset selected_preset = static_cast<UnstructuredMapPreset>(ui_state.unstructured_preset);
    const StructuredMapPreset selected_structured_preset = static_cast<StructuredMapPreset>(ui_state.structured_preset);
    if (selected_mode == EnvironmentMode::UnstructuredGates &&
        selected_preset == UnstructuredMapPreset::Custom &&
        ui_state.scenario_editor_world.environment_mode() == EnvironmentMode::UnstructuredGates) {
        WorldMap custom = ui_state.scenario_editor_world;
        custom.finalize_editor_changes();
        custom.set_gate_behavior(GateBehaviorMode::Static, 0);
        return custom;
    }
    if (selected_mode == EnvironmentMode::StructuredRoad &&
        selected_structured_preset == StructuredMapPreset::Custom &&
        ui_state.scenario_editor_world.environment_mode() == EnvironmentMode::StructuredRoad) {
        WorldMap custom = ui_state.scenario_editor_world;
        custom.finalize_editor_changes();
        return custom;
    }
    WorldMap world = make_world_from_mode(
        selected_mode,
        selected_preset,
        selected_structured_preset,
        sim.gate_behavior(),
        sim.gate_seed(),
        ui_state.mixed_preset);
    return fit_simulation_structured_world(
        std::move(world),
        static_cast<VehicleModelKind>(ui_state.vehicle_model));
}

WorldMap hardware_world_from_ui_selection(const UiState& ui_state) {
    const EnvironmentMode selected_mode = static_cast<EnvironmentMode>(ui_state.hardware_environment_mode);
    const UnstructuredMapPreset selected_preset = static_cast<UnstructuredMapPreset>(ui_state.hardware_unstructured_preset);
    const StructuredMapPreset selected_structured_preset =
        static_cast<StructuredMapPreset>(ui_state.hardware_structured_preset);
    if (selected_mode == EnvironmentMode::MixedRoadGates) {
        return WorldMap::mixed_hardware_demo();
    }
    if (selected_mode == EnvironmentMode::UnstructuredGates &&
        selected_preset == UnstructuredMapPreset::Custom &&
        ui_state.hardware_editor_world.environment_mode() == EnvironmentMode::UnstructuredGates) {
        WorldMap custom = ui_state.hardware_editor_world;
        custom.finalize_editor_changes();
        return sanitize_hardware_unstructured_world(std::move(custom));
    }
    if (selected_mode == EnvironmentMode::StructuredRoad &&
        selected_structured_preset == StructuredMapPreset::Custom &&
        ui_state.hardware_editor_world.environment_mode() == EnvironmentMode::StructuredRoad) {
        WorldMap custom = ui_state.hardware_editor_world;
        custom.finalize_editor_changes();
        return fit_hardware_structured_world(
            std::move(custom),
            static_cast<VehicleModelKind>(ui_state.hardware_vehicle_model));
    }
    WorldMap world = make_world_from_mode(selected_mode, selected_preset, selected_structured_preset, GateBehaviorMode::Static, 0);
    world = apply_hardware_track_scale(ui_state, std::move(world));
    return sanitize_hardware_unstructured_world(std::move(world));
}

WorldMap* active_editor_world(UiState* ui_state) {
    if (ui_state == nullptr) {
        return nullptr;
    }
    return workspace_source_is_hardware(ui_state->workspace_source)
               ? &ui_state->hardware_editor_world
               : &ui_state->scenario_editor_world;
}

bool* active_editor_dirty_flag(UiState* ui_state) {
    if (ui_state == nullptr) {
        return nullptr;
    }
    return workspace_source_is_hardware(ui_state->workspace_source)
               ? &ui_state->hardware_editor_dirty
               : &ui_state->scenario_editor_dirty;
}

void reset_editor_interaction(UiState* ui_state) {
    if (ui_state == nullptr) {
        return;
    }
    ui_state->selected_editor_handle = {};
    ui_state->active_drag_handle = {};
    ui_state->drag_offset = {};
}

struct MetricSummary {
    double avg = 0.0;
    double max = 0.0;
};

struct RunQualitySummary {
    bool has_data = false;
    double duration_s = 0.0;
    double avg_speed = 0.0;
    double max_speed = 0.0;
    double max_cross_track = 0.0;
    double max_heading_error_deg = 0.0;
    double min_lidar = 0.0;
    double avg_step_ms = 0.0;
    double max_step_ms = 0.0;
    double avg_planning_ms = 0.0;
    double max_planning_ms = 0.0;
    double mixed_switches = 0.0;
    double mixed_aborts = 0.0;
    double final_goal_distance = 0.0;
};

struct TimelineEvent {
    double time = 0.0;
    std::string label;
    std::string detail;
    ImVec4 color;
};

struct ValidationPreset {
    const char* name = "";
    const char* detail = "";
    EnvironmentMode environment = EnvironmentMode::StructuredRoad;
    UnstructuredMapPreset unstructured = UnstructuredMapPreset::RobotValidation;
    StructuredMapPreset structured = StructuredMapPreset::ValidationRoad;
    int mixed_preset = 0;
    VehicleModelKind vehicle = VehicleModelKind::CarLikeBicycle;
    bool dynamic_gates = false;
    bool ideal = false;
};

std::string replace_extension(const std::string& path, const char* extension) {
    std::filesystem::path out(path);
    out.replace_extension(extension != nullptr ? extension : "");
    return out.string();
}

RunQualitySummary summarize_run_quality(const std::vector<TelemetrySample>& history) {
    RunQualitySummary summary;
    if (history.empty()) {
        return summary;
    }
    summary.has_data = true;
    summary.duration_s = history.back().time;
    summary.min_lidar = std::numeric_limits<double>::infinity();
    double speed_total = 0.0;
    double step_total = 0.0;
    double planning_total = 0.0;
    for (const TelemetrySample& sample : history) {
        speed_total += sample.speed;
        step_total += sample.step_ms;
        planning_total += sample.planning_ms;
        summary.max_speed = std::max(summary.max_speed, sample.speed);
        summary.max_cross_track = std::max(summary.max_cross_track, std::abs(sample.tracker_cross_track));
        summary.max_heading_error_deg = std::max(summary.max_heading_error_deg, std::abs(sample.tracker_heading_error_deg));
        if (sample.min_lidar > 0.0) {
            summary.min_lidar = std::min(summary.min_lidar, sample.min_lidar);
        }
        summary.max_step_ms = std::max(summary.max_step_ms, sample.step_ms);
        summary.max_planning_ms = std::max(summary.max_planning_ms, sample.planning_ms);
    }
    const double count = static_cast<double>(history.size());
    summary.avg_speed = speed_total / count;
    summary.avg_step_ms = step_total / count;
    summary.avg_planning_ms = planning_total / count;
    summary.mixed_switches = history.back().mixed_switches;
    summary.mixed_aborts = history.back().mixed_aborts;
    summary.final_goal_distance = history.back().distance_to_goal;
    if (!std::isfinite(summary.min_lidar)) {
        summary.min_lidar = 0.0;
    }
    return summary;
}

RunQualitySummary summarize_run_quality(const std::vector<HardwareTelemetrySample>& history) {
    RunQualitySummary summary;
    if (history.empty()) {
        return summary;
    }
    summary.has_data = true;
    summary.duration_s = history.back().time;
    summary.min_lidar = std::numeric_limits<double>::infinity();
    double speed_total = 0.0;
    double step_total = 0.0;
    double planning_total = 0.0;
    for (const HardwareTelemetrySample& sample : history) {
        speed_total += sample.speed;
        step_total += sample.step_ms;
        planning_total += sample.planning_ms;
        summary.max_speed = std::max(summary.max_speed, sample.speed);
        summary.max_cross_track = std::max(summary.max_cross_track, std::abs(sample.tracker_cross_track));
        summary.max_heading_error_deg = std::max(summary.max_heading_error_deg, std::abs(sample.tracker_heading_error_deg));
        if (sample.min_lidar > 0.0) {
            summary.min_lidar = std::min(summary.min_lidar, sample.min_lidar);
        }
        summary.max_step_ms = std::max(summary.max_step_ms, sample.step_ms);
        summary.max_planning_ms = std::max(summary.max_planning_ms, sample.planning_ms);
    }
    const double count = static_cast<double>(history.size());
    summary.avg_speed = speed_total / count;
    summary.avg_step_ms = step_total / count;
    summary.avg_planning_ms = planning_total / count;
    summary.final_goal_distance = history.back().distance_to_goal;
    if (!std::isfinite(summary.min_lidar)) {
        summary.min_lidar = 0.0;
    }
    return summary;
}

void push_timeline_event(std::vector<TimelineEvent>* events,
                         double time,
                         const char* label,
                         const std::string& detail,
                         const ImVec4& color) {
    if (events == nullptr || label == nullptr) {
        return;
    }
    events->push_back({time, label, detail, color});
}

std::vector<TimelineEvent> build_sim_timeline(const PlannerDrivenVehicleSim& sim) {
    std::vector<TimelineEvent> events;
    const std::vector<TelemetrySample>& history = sim.history();
    if (history.empty()) {
        return events;
    }

    push_timeline_event(&events,
                        history.front().time,
                        "Run started",
                        std::string(thesis_sim::environment_mode_name(sim.environment_mode())) +
                            " / " + map_preset_name(sim.world()),
                        ImVec4(0.43f, 0.82f, 0.96f, 1.0f));

    bool had_reference = history.front().planner_has_reference > 0.5;
    bool in_mixed_gate = history.front().mixed_mode > 0.5;
    bool clearance_warning = history.front().min_lidar > 0.0 && history.front().min_lidar < 0.25;
    int chosen_gate = static_cast<int>(std::lround(history.front().chosen_gate_index));
    int mixed_switches = static_cast<int>(std::lround(history.front().mixed_switches));
    int mixed_aborts = static_cast<int>(std::lround(history.front().mixed_aborts));
    bool lag_warning = history.front().step_ms > 30.0;

    for (size_t i = 1; i < history.size(); ++i) {
        const TelemetrySample& sample = history[i];
        const bool has_reference = sample.planner_has_reference > 0.5;
        if (has_reference != had_reference) {
            push_timeline_event(&events,
                                sample.time,
                                has_reference ? "Planner reference acquired" : "Planner reference lost",
                                has_reference ? "MPC received a valid path reference." : "Controller is waiting for a usable reference.",
                                has_reference ? ImVec4(0.48f, 0.88f, 0.62f, 1.0f)
                                              : ImVec4(0.95f, 0.60f, 0.34f, 1.0f));
            had_reference = has_reference;
        }

        const int gate = static_cast<int>(std::lround(sample.chosen_gate_index));
        if (gate >= 0 && gate != chosen_gate) {
            push_timeline_event(&events,
                                sample.time,
                                "Gate selected",
                                "Gate index " + std::to_string(gate) +
                                    ", distance " + std::to_string(sample.chosen_gate_distance) + " m.",
                                ImVec4(0.97f, 0.89f, 0.45f, 1.0f));
        }
        chosen_gate = gate;

        const bool mixed_gate = sample.mixed_mode > 0.5;
        if (mixed_gate != in_mixed_gate) {
            push_timeline_event(&events,
                                sample.time,
                                mixed_gate ? "Mixed gate mode" : "Structured rejoin",
                                mixed_gate ? "The arbiter switched from road tracking to dynamic gate tracking."
                                           : "The vehicle returned to the structured road objective.",
                                mixed_gate ? ImVec4(0.96f, 0.66f, 0.28f, 1.0f)
                                           : ImVec4(0.36f, 0.73f, 0.98f, 1.0f));
            in_mixed_gate = mixed_gate;
        }

        const int next_switches = static_cast<int>(std::lround(sample.mixed_switches));
        if (next_switches > mixed_switches) {
            push_timeline_event(&events,
                                sample.time,
                                "Mixed switch counted",
                                "Gate score " + std::to_string(sample.mixed_gate_score) +
                                    ", road score " + std::to_string(sample.mixed_structured_score) + ".",
                                ImVec4(0.96f, 0.66f, 0.28f, 1.0f));
            mixed_switches = next_switches;
        }

        const int next_aborts = static_cast<int>(std::lround(sample.mixed_aborts));
        if (next_aborts > mixed_aborts) {
            push_timeline_event(&events,
                                sample.time,
                                "Mixed abort",
                                "Dynamic gate rejected; road tracking restored.",
                                ImVec4(0.95f, 0.40f, 0.34f, 1.0f));
            mixed_aborts = next_aborts;
        }

        const bool next_clearance_warning = sample.min_lidar > 0.0 && sample.min_lidar < 0.25;
        if (next_clearance_warning && !clearance_warning) {
            push_timeline_event(&events,
                                sample.time,
                                "Clearance warning",
                                "Minimum LiDAR distance below 0.25 m.",
                                ImVec4(0.95f, 0.60f, 0.34f, 1.0f));
        }
        clearance_warning = next_clearance_warning && !(sample.min_lidar > 0.36);

        const bool next_lag_warning = sample.step_ms > 30.0;
        if (next_lag_warning && !lag_warning) {
            push_timeline_event(&events,
                                sample.time,
                                "Frame budget spike",
                                "Step time exceeded 30 ms.",
                                ImVec4(0.80f, 0.62f, 0.34f, 1.0f));
        }
        lag_warning = next_lag_warning;
    }

    push_timeline_event(&events,
                        history.back().time,
                        sim.goal_reached() ? "Goal reached" : (sim.collision() ? "Collision" : "Latest sample"),
                        "Final goal distance " + std::to_string(history.back().distance_to_goal) + " m.",
                        sim.goal_reached() ? ImVec4(0.48f, 0.88f, 0.62f, 1.0f)
                                           : (sim.collision() ? ImVec4(0.95f, 0.40f, 0.34f, 1.0f)
                                                             : ImVec4(0.74f, 0.79f, 0.84f, 1.0f)));
    return events;
}

std::vector<TimelineEvent> build_hardware_timeline(const HardwareViewerState& hardware) {
    std::vector<TimelineEvent> events;
    const std::vector<HardwareTelemetrySample>& history = hardware.history;
    if (history.empty()) {
        return events;
    }

    push_timeline_event(&events,
                        history.front().time,
                        "Hardware stream started",
                        hardware.has_scene ? hardware.scene.vehicle_model_name : "Waiting for scene metadata.",
                        ImVec4(0.43f, 0.82f, 0.96f, 1.0f));

    bool had_reference = history.front().planner_has_reference > 0.5;
    bool safety_stop = history.front().safety_stop_active > 0.5;
    bool clearance_warning = history.front().front_lidar > 0.0 && history.front().front_lidar < 0.22;
    int chosen_gate = static_cast<int>(std::lround(history.front().chosen_gate_index));
    bool lag_warning = history.front().step_ms > 50.0;

    for (size_t i = 1; i < history.size(); ++i) {
        const HardwareTelemetrySample& sample = history[i];
        const bool has_reference = sample.planner_has_reference > 0.5;
        if (has_reference != had_reference) {
            push_timeline_event(&events,
                                sample.time,
                                has_reference ? "Reference ready" : "Reference lost",
                                has_reference ? "Planner trajectory is feeding the hardware controller."
                                              : "The runner is waiting for a valid path reference.",
                                has_reference ? ImVec4(0.48f, 0.88f, 0.62f, 1.0f)
                                              : ImVec4(0.95f, 0.60f, 0.34f, 1.0f));
            had_reference = has_reference;
        }

        const bool stop_active = sample.safety_stop_active > 0.5;
        if (stop_active != safety_stop) {
            push_timeline_event(&events,
                                sample.time,
                                stop_active ? "Safety stop active" : "Safety stop cleared",
                                stop_active ? "Controller command is blocked by clearance or safety policy."
                                            : "Motion commands are allowed again.",
                                stop_active ? ImVec4(0.95f, 0.40f, 0.34f, 1.0f)
                                            : ImVec4(0.48f, 0.88f, 0.62f, 1.0f));
            safety_stop = stop_active;
        }

        const int gate = static_cast<int>(std::lround(sample.chosen_gate_index));
        if (gate >= 0 && gate != chosen_gate) {
            push_timeline_event(&events,
                                sample.time,
                                "Hardware gate selected",
                                "Gate index " + std::to_string(gate) +
                                    ", distance " + std::to_string(sample.chosen_gate_distance) + " m.",
                                ImVec4(0.97f, 0.89f, 0.45f, 1.0f));
        }
        chosen_gate = gate;

        const bool next_clearance_warning = sample.front_lidar > 0.0 && sample.front_lidar < 0.22;
        if (next_clearance_warning && !clearance_warning) {
            push_timeline_event(&events,
                                sample.time,
                                "Front clearance warning",
                                "Front LiDAR distance below 0.22 m.",
                                ImVec4(0.95f, 0.60f, 0.34f, 1.0f));
        }
        clearance_warning = next_clearance_warning && !(sample.front_lidar > 0.34);

        const bool next_lag_warning = sample.step_ms > 50.0;
        if (next_lag_warning && !lag_warning) {
            push_timeline_event(&events,
                                sample.time,
                                "Hardware loop spike",
                                "Step time exceeded 50 ms.",
                                ImVec4(0.80f, 0.62f, 0.34f, 1.0f));
        }
        lag_warning = next_lag_warning;
    }

    push_timeline_event(&events,
                        history.back().time,
                        hardware.frame.goal_reached ? "Goal reached" : "Latest hardware sample",
                        "Current goal distance " + std::to_string(history.back().distance_to_goal) + " m.",
                        hardware.frame.goal_reached ? ImVec4(0.48f, 0.88f, 0.62f, 1.0f)
                                                    : ImVec4(0.74f, 0.79f, 0.84f, 1.0f));
    return events;
}

MetricSummary summarize_metric(const std::vector<TelemetrySample>& history, double TelemetrySample::*member) {
    MetricSummary summary;
    if (history.empty()) {
        return summary;
    }
    double total = 0.0;
    for (const TelemetrySample& sample : history) {
        const double value = sample.*member;
        total += value;
        summary.max = std::max(summary.max, value);
    }
    summary.avg = total / static_cast<double>(history.size());
    return summary;
}

MetricSummary summarize_metric(const std::vector<HardwareTelemetrySample>& history, double HardwareTelemetrySample::*member) {
    MetricSummary summary;
    if (history.empty()) {
        return summary;
    }
    double total = 0.0;
    for (const HardwareTelemetrySample& sample : history) {
        const double value = sample.*member;
        total += value;
        summary.max = std::max(summary.max, value);
    }
    summary.avg = total / static_cast<double>(history.size());
    return summary;
}

std::string report_status_string(const PlannerDrivenVehicleSim& sim) {
    if (sim.goal_reached()) {
        return "goal_reached";
    }
    if (sim.collision()) {
        return "collision";
    }
    return "stopped";
}

std::string default_report_path(const PlannerDrivenVehicleSim& sim, const char* source_tag) {
    namespace fs = std::filesystem;
    std::error_code ec;
    fs::path report_dir = fs::current_path(ec) / "reports";
    if (!ec) {
        fs::create_directories(report_dir, ec);
    }

    const auto now = std::chrono::system_clock::now();
    const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    std::tm local_tm{};
    if (const std::tm* tm_ptr = std::localtime(&now_time)) {
        local_tm = *tm_ptr;
    }
    const auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count() % 1000;

    std::ostringstream name;
    name << "thesis_planner_"
         << environment_mode_slug(sim.environment_mode());
    const std::string preset_name = map_preset_name(sim.world());
    name << "_" << slugify(preset_name);
    if (sim.vehicle_model_kind() != VehicleModelKind::CarLikeBicycle) {
        name << "_" << slugify(thesis_sim::vehicle_model_kind_name(sim.vehicle_model_kind()));
    }
    if (sim.config().dynamic_lidar_gates) {
        name << "_lidar_dynamic";
    }
    if (sim.config().ideal_conditions) {
        name << "_ideal";
    }
    name << "_" << source_tag << "_"
         << std::put_time(&local_tm, "%Y%m%d_%H%M%S")
         << "_" << std::setw(3) << std::setfill('0') << millis
         << ".json";
    return (report_dir / name.str()).string();
}

std::string default_hardware_report_path(const HardwareViewerState& hardware,
                                         const UiState& ui_state,
                                         const char* source_tag) {
    namespace fs = std::filesystem;
    std::error_code ec;
    fs::path report_dir = fs::current_path(ec) / "reports";
    if (!ec) {
        fs::create_directories(report_dir, ec);
    }

    const auto now = std::chrono::system_clock::now();
    const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    std::tm local_tm{};
    if (const std::tm* tm_ptr = std::localtime(&now_time)) {
        local_tm = *tm_ptr;
    }
    const auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count() % 1000;

    const EnvironmentMode environment = hardware.has_scene
                                            ? hardware.scene.world.environment_mode()
                                            : static_cast<EnvironmentMode>(ui_state.hardware_environment_mode);
    const std::string preset_name =
        environment == EnvironmentMode::MixedRoadGates
            ? "Mixed Hardware Road Gate"
            : (environment == EnvironmentMode::StructuredRoad
                   ? (hardware.has_scene
                          ? thesis_sim::structured_map_preset_name(hardware.scene.world.structured_preset())
                          : thesis_sim::structured_map_preset_name(
                                static_cast<StructuredMapPreset>(ui_state.hardware_structured_preset)))
                   : (hardware.has_scene
                          ? thesis_sim::unstructured_map_preset_name(hardware.scene.world.unstructured_preset())
                          : thesis_sim::unstructured_map_preset_name(
                                static_cast<UnstructuredMapPreset>(ui_state.hardware_unstructured_preset))));

    std::ostringstream name;
    name << "thesis_hardware_"
         << environment_mode_slug(environment)
         << "_" << slugify(preset_name)
         << "_" << source_tag << "_"
         << std::put_time(&local_tm, "%Y%m%d_%H%M%S")
         << "_" << std::setw(3) << std::setfill('0') << millis
         << ".json";
    return (report_dir / name.str()).string();
}

std::string default_hardware_world_path(const UiState& ui_state) {
    namespace fs = std::filesystem;
    const fs::path report_dir = fs::path("reports");
    std::error_code ec;
    fs::create_directories(report_dir, ec);

    const auto now = std::chrono::system_clock::now();
    const std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    std::tm local_tm{};
#if defined(_WIN32)
    localtime_s(&local_tm, &now_c);
#else
    localtime_r(&now_c, &local_tm);
#endif
    const auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count() % 1000;

    const EnvironmentMode environment = static_cast<EnvironmentMode>(ui_state.hardware_environment_mode);
    const std::string preset_name =
        environment == EnvironmentMode::MixedRoadGates
            ? "Mixed Hardware Road Gate"
            : (environment == EnvironmentMode::StructuredRoad
                   ? thesis_sim::structured_map_preset_name(static_cast<StructuredMapPreset>(ui_state.hardware_structured_preset))
                   : thesis_sim::unstructured_map_preset_name(static_cast<UnstructuredMapPreset>(ui_state.hardware_unstructured_preset)));

    std::ostringstream name;
    name << "thesis_hardware_world_"
         << environment_mode_slug(environment)
         << "_" << slugify(preset_name)
         << "_" << std::put_time(&local_tm, "%Y%m%d_%H%M%S")
         << "_" << std::setw(3) << std::setfill('0') << millis
         << ".thmap";
    return (report_dir / name.str()).string();
}

bool write_world_blob_file(const WorldMap& world, const std::string& file_path, std::string* error) {
    const std::vector<std::uint8_t> blob = thesis_sim::serialize_world_blob(world);
    std::ofstream out(file_path, std::ios::binary | std::ios::trunc);
    if (!out.is_open()) {
        if (error != nullptr) {
            *error = "Could not open output file.";
        }
        return false;
    }
    out.write(reinterpret_cast<const char*>(blob.data()), static_cast<std::streamsize>(blob.size()));
    if (!out.good()) {
        if (error != nullptr) {
            *error = "Could not write the world blob.";
        }
        return false;
    }
    if (error != nullptr) {
        error->clear();
    }
    return true;
}

void write_vec2_json(std::ostream& out, const Vec2& value) {
    out << "{\"x\":" << value.x << ",\"y\":" << value.y << "}";
}

void write_rect_json(std::ostream& out, const Rect& rect) {
    out << "{\"min_x\":" << rect.min_x
        << ",\"min_y\":" << rect.min_y
        << ",\"max_x\":" << rect.max_x
        << ",\"max_y\":" << rect.max_y
        << "}";
}

void write_world_json(std::ostream& out, const WorldMap& world) {
    out << "{\n";
    out << "      \"environment\": \"" << json_escape(thesis_sim::environment_mode_name(world.environment_mode())) << "\",\n";
    out << "      \"unstructured_preset\": \"" << json_escape(thesis_sim::unstructured_map_preset_name(world.unstructured_preset())) << "\",\n";
    out << "      \"structured_preset\": \"" << json_escape(thesis_sim::structured_map_preset_name(world.structured_preset())) << "\",\n";
    out << "      \"gate_behavior\": \"" << json_escape(thesis_sim::gate_behavior_mode_name(world.gate_behavior())) << "\",\n";
    out << "      \"gate_seed\": " << world.gate_seed() << ",\n";
    out << "      \"bounds\": ";
    write_rect_json(out, world.bounds());
    out << ",\n";
    out << "      \"start\": ";
    write_vec2_json(out, world.start());
    out << ",\n";
    out << "      \"goal\": ";
    write_vec2_json(out, world.goal());
    out << ",\n";
    out << "      \"start_heading_rad\": " << world.start_heading() << ",\n";
    out << "      \"obstacles\": [\n";
    for (size_t i = 0; i < world.obstacles().size(); ++i) {
        out << "        ";
        write_rect_json(out, world.obstacles()[i]);
        out << (i + 1 < world.obstacles().size() ? ",\n" : "\n");
    }
    out << "      ],\n";
    out << "      \"gates\": [\n";
    for (size_t i = 0; i < world.gates().size(); ++i) {
        const GateSpec& gate = world.gates()[i];
        out << "        {\"name\":\"" << json_escape(gate.name) << "\",\"position\":";
        write_vec2_json(out, gate.position);
        out << ",\"anchor_position\":";
        write_vec2_json(out, gate.anchor_position);
        out << ",\"motion_amplitude\":";
        write_vec2_json(out, gate.motion_amplitude);
        out << ",\"motion_frequency_hz\":" << gate.motion_frequency_hz
            << ",\"motion_phase_rad\":" << gate.motion_phase_rad
            << ",\"heading_hint\":" << gate.heading_hint
            << ",\"final\":" << (gate.final ? "true" : "false") << "}";
        out << (i + 1 < world.gates().size() ? ",\n" : "\n");
    }
    out << "      ],\n";
    out << "      \"road_centerline\": [\n";
    for (size_t i = 0; i < world.road_centerline().size(); ++i) {
        out << "        ";
        write_vec2_json(out, world.road_centerline()[i]);
        out << (i + 1 < world.road_centerline().size() ? ",\n" : "\n");
    }
    out << "      ]\n";
    out << "    }";
}

void write_geometry_json(std::ostream& out, const thesis_sim::VehicleGeometry& geometry) {
    out << "{"
        << "\"wheelbase\":" << geometry.wheelbase
        << ",\"cg_to_front\":" << geometry.cg_to_front
        << ",\"cg_to_rear\":" << geometry.cg_to_rear
        << ",\"track\":" << geometry.track
        << ",\"body_length\":" << geometry.body_length
        << ",\"body_width\":" << geometry.body_width
        << ",\"wheel_length\":" << geometry.wheel_length
        << ",\"wheel_width\":" << geometry.wheel_width
        << ",\"wheel_radius\":" << geometry.wheel_radius
        << ",\"max_steer_angle\":" << geometry.max_steer_angle
        << ",\"max_steer_rate\":" << geometry.max_steer_rate
        << ",\"max_curvature\":" << geometry.max_curvature
        << ",\"max_linear_speed\":" << geometry.max_linear_speed
        << ",\"max_yaw_rate\":" << geometry.max_yaw_rate
        << ",\"max_accel\":" << geometry.max_accel
        << ",\"max_decel\":" << geometry.max_decel
        << ",\"max_pwm\":" << geometry.max_pwm
        << ",\"min_effective_pwm\":" << geometry.min_effective_pwm
        << ",\"wheel_speed_to_pwm_gain\":" << geometry.wheel_speed_to_pwm_gain
        << ",\"wheel_speed_to_pwm_bias\":" << geometry.wheel_speed_to_pwm_bias
        << ",\"speed_estimate_per_pwm\":" << geometry.speed_estimate_per_pwm
        << ",\"left_pwm_scale\":" << geometry.left_pwm_scale
        << ",\"right_pwm_scale\":" << geometry.right_pwm_scale
        << ",\"yaw_response_scale\":" << geometry.yaw_response_scale
        << ",\"linear_feedback_gain\":" << geometry.linear_feedback_gain
        << ",\"yaw_feedback_gain\":" << geometry.yaw_feedback_gain
        << ",\"pwm_slew_rate\":" << geometry.pwm_slew_rate
        << ",\"motor_time_constant\":" << geometry.motor_time_constant
        << ",\"encoder_ticks_per_revolution\":" << geometry.encoder_ticks_per_revolution
        << "}";
}

void write_live_vehicle_state_json(std::ostream& out, const LiveVehicleState& vehicle) {
    out << "{"
        << "\"position\":";
    write_vec2_json(out, vehicle.position);
    out << ",\"yaw\":" << vehicle.yaw
        << ",\"speed\":" << vehicle.speed
        << ",\"accel\":" << vehicle.accel
        << ",\"curvature\":" << vehicle.curvature
        << ",\"steer_angle\":" << vehicle.steer_angle
        << ",\"yaw_rate\":" << vehicle.yaw_rate
        << ",\"sideslip\":" << vehicle.sideslip
        << ",\"left_wheel_speed\":" << vehicle.left_wheel_speed
        << ",\"right_wheel_speed\":" << vehicle.right_wheel_speed
        << ",\"target_speed\":" << vehicle.target_speed
        << ",\"target_yaw_rate\":" << vehicle.target_yaw_rate
        << ",\"target_steer_angle\":" << vehicle.target_steer_angle
        << ",\"left_encoder_ticks\":" << vehicle.left_encoder_ticks
        << ",\"right_encoder_ticks\":" << vehicle.right_encoder_ticks
        << ",\"left_encoder_delta\":" << vehicle.left_encoder_delta
        << ",\"right_encoder_delta\":" << vehicle.right_encoder_delta
        << ",\"left_pwm\":" << vehicle.left_pwm
        << ",\"right_pwm\":" << vehicle.right_pwm
        << ",\"encoder_dt_ms\":" << vehicle.encoder_dt_ms
        << "}";
}

void write_live_gate_frame_json(std::ostream& out, const LiveGateFrame& gate) {
    out << "{"
        << "\"name\":\"" << json_escape(gate.spec.name) << "\",\"position\":";
    write_vec2_json(out, gate.spec.position);
    out << ",\"anchor_position\":";
    write_vec2_json(out, gate.spec.anchor_position);
    out << ",\"motion_amplitude\":";
    write_vec2_json(out, gate.spec.motion_amplitude);
    out << ",\"motion_frequency_hz\":" << gate.spec.motion_frequency_hz
        << ",\"motion_phase_rad\":" << gate.spec.motion_phase_rad
        << ",\"heading_hint\":" << gate.spec.heading_hint
        << ",\"final\":" << (gate.spec.final ? "true" : "false")
        << ",\"passed\":" << (gate.passed ? "true" : "false")
        << "}";
}

void write_lidar_hit_json(std::ostream& out, const LidarHit& hit) {
    out << "{"
        << "\"angle\":" << hit.angle
        << ",\"distance\":" << hit.distance
        << ",\"point\":";
    write_vec2_json(out, hit.point);
    out << ",\"hit\":" << (hit.hit ? "true" : "false")
        << "}";
}

void write_hardware_sample_json(std::ostream& out, const HardwareTelemetrySample& sample) {
    out << "{"
        << "\"time\":" << sample.time
        << ",\"position_x\":" << sample.position_x
        << ",\"position_y\":" << sample.position_y
        << ",\"yaw\":" << sample.yaw
        << ",\"speed\":" << sample.speed
        << ",\"accel\":" << sample.accel
        << ",\"yaw_rate\":" << sample.yaw_rate
        << ",\"jerk\":" << sample.jerk
        << ",\"command_r\":" << sample.command_r
        << ",\"target_speed\":" << sample.target_speed
        << ",\"target_yaw_rate\":" << sample.target_yaw_rate
        << ",\"curvature\":" << sample.curvature
        << ",\"distance_to_goal\":" << sample.distance_to_goal
        << ",\"structured_track_s\":" << sample.structured_track_s
        << ",\"structured_progress_s\":" << sample.structured_progress_s
        << ",\"min_lidar\":" << sample.min_lidar
        << ",\"front_lidar\":" << sample.front_lidar
        << ",\"planner_speed_ref\":" << sample.planner_speed_ref
        << ",\"tracker_cross_track\":" << sample.tracker_cross_track
        << ",\"tracker_heading_error_deg\":" << sample.tracker_heading_error_deg
        << ",\"planning_ms\":" << sample.planning_ms
        << ",\"tracking_ms\":" << sample.tracking_ms
        << ",\"lidar_ms\":" << sample.lidar_ms
        << ",\"estimator_ms\":" << sample.estimator_ms
        << ",\"step_ms\":" << sample.step_ms
        << ",\"visible_gates\":" << sample.visible_gates
        << ",\"lidar_samples\":" << sample.lidar_samples
        << ",\"close_lidar_samples\":" << sample.close_lidar_samples
        << ",\"front_close_lidar_samples\":" << sample.front_close_lidar_samples
        << ",\"candidate_gates\":" << sample.candidate_gates
        << ",\"chosen_gate_distance\":" << sample.chosen_gate_distance
        << ",\"passed_gates\":" << sample.passed_gates
        << ",\"accumulated_lidar_points\":" << sample.accumulated_lidar_points
        << ",\"no_motion_cycles\":" << sample.no_motion_cycles
        << ",\"chosen_gate_index\":" << sample.chosen_gate_index
        << ",\"safety_stop_active\":" << sample.safety_stop_active
        << ",\"planner_has_reference\":" << sample.planner_has_reference
        << ",\"dynamic_gap_gates\":" << sample.dynamic_gap_gates
        << ",\"pwm_left\":" << sample.pwm_left
        << ",\"pwm_right\":" << sample.pwm_right
        << ",\"controller_pwm_left\":" << sample.controller_pwm_left
        << ",\"controller_pwm_right\":" << sample.controller_pwm_right
        << ",\"controller_target_pwm_left\":" << sample.controller_target_pwm_left
        << ",\"controller_target_pwm_right\":" << sample.controller_target_pwm_right
        << ",\"controller_left_encoder_ticks\":" << sample.controller_left_encoder_ticks
        << ",\"controller_right_encoder_ticks\":" << sample.controller_right_encoder_ticks
        << ",\"controller_left_encoder_delta\":" << sample.controller_left_encoder_delta
        << ",\"controller_right_encoder_delta\":" << sample.controller_right_encoder_delta
        << ",\"controller_encoder_dt_ms\":" << sample.controller_encoder_dt_ms
        << ",\"controller_safety_flags\":" << sample.controller_safety_flags
        << ",\"controller_motor_flags\":" << sample.controller_motor_flags
        << ",\"controller_status_flags\":" << sample.controller_status_flags
        << ",\"controller_error_code\":" << sample.controller_error_code
        << "}";
}

bool write_hardware_json_report(const HardwareViewerState& hardware,
                                const UiState& ui_state,
                                const LiveViewStreamServer& hardware_server,
                                const std::string& report_path) {
    std::ofstream out(report_path, std::ios::out | std::ios::trunc);
    if (!out.is_open()) {
        return false;
    }

    const std::string status =
        hardware.frame.goal_reached
            ? "goal_reached"
            : (hardware.frame.safety_stop_active ? "safety_stop"
                                                : (hardware_server.connected() || hardware.frame.connected
                                                       ? "live"
                                                       : (!hardware.history.empty() || hardware.frame.telemetry_ready || hardware.frame.step_count > 0
                                                              ? "disconnected"
                                                              : (hardware_server.listening() ? "listening" : "idle"))));

    const MetricSummary planning_summary = summarize_metric(hardware.history, &HardwareTelemetrySample::planning_ms);
    const MetricSummary tracking_summary = summarize_metric(hardware.history, &HardwareTelemetrySample::tracking_ms);
    const MetricSummary lidar_summary = summarize_metric(hardware.history, &HardwareTelemetrySample::lidar_ms);
    const MetricSummary estimator_summary = summarize_metric(hardware.history, &HardwareTelemetrySample::estimator_ms);
    const MetricSummary step_summary = summarize_metric(hardware.history, &HardwareTelemetrySample::step_ms);

    out << std::fixed << std::setprecision(6);
    out << "{\n";
    out << "  \"schema\": \"thesis_hardware_report_v1\",\n";
    out << "  \"status\": \"" << json_escape(status) << "\",\n";
    out << "  \"workspace_source\": \"" << json_escape(workspace_source_label(ui_state.workspace_source)) << "\",\n";
    out << "  \"listener\": {\n";
    out << "    \"listening\": " << (hardware_server.listening() ? "true" : "false") << ",\n";
    out << "    \"connected\": " << (hardware_server.connected() ? "true" : "false") << ",\n";
    out << "    \"port\": " << hardware_server.port() << ",\n";
    out << "    \"remote_endpoint\": \"" << json_escape(hardware_server.remote_endpoint()) << "\",\n";
    out << "    \"last_error\": \"" << json_escape(hardware_server.last_error()) << "\"\n";
    out << "  },\n";
    out << "  \"scene\": {\n";
    out << "    \"available\": " << (hardware.has_scene ? "true" : "false");
    if (hardware.has_scene) {
        out << ",\n";
        out << "    \"stream_label\": \"" << json_escape(hardware.scene.stream_label) << "\",\n";
        out << "    \"stream_profile\": \"" << json_escape(hardware.scene.stream_profile) << "\",\n";
        out << "    \"imu_enabled\": " << (hardware.scene.imu_enabled ? "true" : "false") << ",\n";
        out << "    \"lidar_enabled\": " << (hardware.scene.lidar_enabled ? "true" : "false") << ",\n";
        out << "    \"localization_mode\": \"" << json_escape(hardware.scene.localization_mode) << "\",\n";
        out << "    \"heading_source\": \"" << json_escape(hardware.scene.heading_source) << "\",\n";
        out << "    \"range_sensor_name\": \"" << json_escape(hardware.scene.range_sensor_name) << "\",\n";
        out << "    \"vehicle_model_name\": \"" << json_escape(hardware.scene.vehicle_model_name) << "\",\n";
        out << "    \"tracking_controller_name\": \"" << json_escape(hardware.scene.tracking_controller_name) << "\",\n";
        out << "    \"active_lidar_beams\": " << hardware.scene.active_lidar_beams << ",\n";
        out << "    \"active_lidar_fov_rad\": " << hardware.scene.active_lidar_fov_rad << ",\n";
        out << "    \"active_lidar_range\": " << hardware.scene.active_lidar_range << ",\n";
        out << "    \"geometry\": ";
        write_geometry_json(out, hardware.scene.geometry);
        out << ",\n";
        out << "    \"world\": ";
        write_world_json(out, hardware.scene.world);
        out << '\n';
    } else {
        out << '\n';
    }
    out << "  },\n";
    out << "  \"frame\": {\n";
    out << "    \"sim_time\": " << hardware.frame.sim_time << ",\n";
    out << "    \"step_count\": " << hardware.frame.step_count << ",\n";
    out << "    \"connected\": " << (hardware.frame.connected ? "true" : "false") << ",\n";
    out << "    \"telemetry_ready\": " << (hardware.frame.telemetry_ready ? "true" : "false") << ",\n";
    out << "    \"goal_reached\": " << (hardware.frame.goal_reached ? "true" : "false") << ",\n";
    out << "    \"safety_stop_active\": " << (hardware.frame.safety_stop_active ? "true" : "false") << ",\n";
    out << "    \"dynamic_gap_gates\": " << (hardware.frame.dynamic_gap_gates ? "true" : "false") << ",\n";
    out << "    \"planner_has_reference\": " << (hardware.frame.planner_has_reference ? "true" : "false") << ",\n";
    out << "    \"stall_boost_active\": " << (hardware.frame.stall_boost_active ? "true" : "false") << ",\n";
    out << "    \"distance_to_goal\": " << hardware.frame.distance_to_goal << ",\n";
    out << "    \"min_lidar_distance\": " << hardware.frame.min_lidar_distance << ",\n";
    out << "    \"front_lidar_distance\": " << hardware.frame.front_lidar_distance << ",\n";
    out << "    \"chosen_gate_distance\": " << hardware.frame.chosen_gate_distance << ",\n";
    out << "    \"last_j\": " << hardware.frame.last_j << ",\n";
    out << "    \"last_r\": " << hardware.frame.last_r << ",\n";
    out << "    \"planner_speed_ref\": " << hardware.frame.planner_speed_ref << ",\n";
    out << "    \"tracker_cross_track_error\": " << hardware.frame.tracker_cross_track_error << ",\n";
    out << "    \"tracker_heading_error_deg\": " << hardware.frame.tracker_heading_error_deg << ",\n";
    out << "    \"valid_lidar_samples\": " << hardware.frame.valid_lidar_samples << ",\n";
    out << "    \"close_lidar_samples\": " << hardware.frame.close_lidar_samples << ",\n";
    out << "    \"front_close_lidar_samples\": " << hardware.frame.front_close_lidar_samples << ",\n";
    out << "    \"candidate_gates\": " << hardware.frame.candidate_gates << ",\n";
    out << "    \"accumulated_lidar_points\": " << hardware.frame.accumulated_lidar_points << ",\n";
    out << "    \"no_motion_command_cycles\": " << hardware.frame.no_motion_command_cycles << ",\n";
    out << "    \"chosen_gate_index\": " << hardware.frame.chosen_gate_index << ",\n";
    out << "    \"passed_gates\": " << hardware.frame.passed_gates << ",\n";
    out << "    \"occupancy_cell_size_m\": " << hardware.frame.occupancy_cell_size_m << ",\n";
    out << "    \"vehicle\": ";
    write_live_vehicle_state_json(out, hardware.frame.vehicle);
    out << ",\n";
    out << "    \"navigation_position\": ";
    write_vec2_json(out, hardware.frame.navigation_position);
    out << ",\n";
    out << "    \"navigation_yaw\": " << hardware.frame.navigation_yaw << ",\n";
    out << "    \"navigation_yaw_rate\": " << hardware.frame.navigation_yaw_rate << ",\n";
    out << "    \"navigation_curvature\": " << hardware.frame.navigation_curvature << ",\n";
    out << "    \"navigation_speed\": " << hardware.frame.navigation_speed << ",\n";
    out << "    \"navigation_accel\": " << hardware.frame.navigation_accel << ",\n";
    out << "    \"has_last_mpc_command\": " << (hardware.frame.has_last_mpc_command ? "true" : "false") << ",\n";
    out << "    \"last_mpc_command\": ";
    if (hardware.frame.has_last_mpc_command) {
        out << "{\"accel_cmd\":" << hardware.frame.last_mpc_command.accel_cmd
            << ",\"steer_rate_cmd\":" << hardware.frame.last_mpc_command.steer_rate_cmd << "}";
    } else {
        out << "null";
    }
    out << ",\n";
    out << "    \"has_latest_sample\": " << (hardware.frame.has_latest_sample ? "true" : "false") << ",\n";
    out << "    \"latest_sample\": ";
    if (hardware.frame.has_latest_sample) {
        write_hardware_sample_json(out, hardware.frame.latest_sample);
    } else {
        out << "null";
    }
    out << ",\n";
    out << "    \"visible_gate_indices\": [";
    for (size_t i = 0; i < hardware.frame.visible_gate_indices.size(); ++i) {
        if (i > 0) {
            out << ',';
        }
        out << hardware.frame.visible_gate_indices[i];
    }
    out << "],\n";
    out << "    \"gates\": [\n";
    for (size_t i = 0; i < hardware.frame.gates.size(); ++i) {
        out << "      ";
        write_live_gate_frame_json(out, hardware.frame.gates[i]);
        out << (i + 1 < hardware.frame.gates.size() ? ",\n" : "\n");
    }
    out << "    ],\n";
    out << "    \"trail\": [\n";
    for (size_t i = 0; i < hardware.frame.trail.size(); ++i) {
        out << "      ";
        write_vec2_json(out, hardware.frame.trail[i]);
        out << (i + 1 < hardware.frame.trail.size() ? ",\n" : "\n");
    }
    out << "    ],\n";
    out << "    \"planned_trajectory\": [\n";
    for (size_t i = 0; i < hardware.frame.planned_trajectory.size(); ++i) {
        out << "      ";
        write_vec2_json(out, hardware.frame.planned_trajectory[i]);
        out << (i + 1 < hardware.frame.planned_trajectory.size() ? ",\n" : "\n");
    }
    out << "    ],\n";
    out << "    \"slam_points\": [\n";
    for (size_t i = 0; i < hardware.frame.slam_points.size(); ++i) {
        out << "      ";
        write_vec2_json(out, hardware.frame.slam_points[i]);
        out << (i + 1 < hardware.frame.slam_points.size() ? ",\n" : "\n");
    }
    out << "    ],\n";
    out << "    \"lidar_hits\": [\n";
    for (size_t i = 0; i < hardware.frame.lidar_hits.size(); ++i) {
        out << "      ";
        write_lidar_hit_json(out, hardware.frame.lidar_hits[i]);
        out << (i + 1 < hardware.frame.lidar_hits.size() ? ",\n" : "\n");
    }
    out << "    ]\n";
    out << "  },\n";
    out << "  \"performance\": {\n";
    out << "    \"history_samples\": " << hardware.history.size() << ",\n";
    out << "    \"planning_ms\": {\"avg\": " << planning_summary.avg << ",\"max\": " << planning_summary.max << "},\n";
    out << "    \"tracking_ms\": {\"avg\": " << tracking_summary.avg << ",\"max\": " << tracking_summary.max << "},\n";
    out << "    \"lidar_ms\": {\"avg\": " << lidar_summary.avg << ",\"max\": " << lidar_summary.max << "},\n";
    out << "    \"estimator_ms\": {\"avg\": " << estimator_summary.avg << ",\"max\": " << estimator_summary.max << "},\n";
    out << "    \"step_ms\": {\"avg\": " << step_summary.avg << ",\"max\": " << step_summary.max << "}\n";
    out << "  },\n";
    out << "  \"history\": [\n";
    for (size_t i = 0; i < hardware.history.size(); ++i) {
        out << "    ";
        write_hardware_sample_json(out, hardware.history[i]);
        out << (i + 1 < hardware.history.size() ? ",\n" : "\n");
    }
    out << "  ]\n";
    out << "}\n";
    return true;
}

bool write_json_report(const PlannerDrivenVehicleSim& sim,
                       const std::string& status,
                       const std::string& report_path) {
    std::ofstream out(report_path, std::ios::out | std::ios::trunc);
    if (!out.is_open()) {
        return false;
    }

    const int passed_gates = sim.passed_gate_count();

    const SimulationReport report{
        sim.goal_reached(),
        sim.collision(),
        sim.step_count(),
        sim.sim_time(),
        sim.vehicle().position,
        sim.distance_to_goal(),
        passed_gates,
    };
    const auto& world = sim.world();
    const auto& config = sim.config();
    const auto& history = sim.history();
    const size_t stride = plot_sample_stride(history.size());

    const MetricSummary planning_summary = summarize_metric(history, &TelemetrySample::planning_ms);
    const MetricSummary tracking_summary = summarize_metric(history, &TelemetrySample::tracking_ms);
    const MetricSummary lidar_summary = summarize_metric(history, &TelemetrySample::lidar_ms);
    const MetricSummary estimator_summary = summarize_metric(history, &TelemetrySample::estimator_ms);
    const MetricSummary step_summary = summarize_metric(history, &TelemetrySample::step_ms);

    out << std::fixed << std::setprecision(6);
    out << "{\n";
    out << "  \"schema\": \"thesis_planner_report_v1\",\n";
    out << "  \"status\": \"" << json_escape(status) << "\",\n";
    out << "  \"environment\": \"" << json_escape(thesis_sim::environment_mode_name(sim.environment_mode())) << "\",\n";
    out << "  \"unstructured_preset\": \"" << json_escape(thesis_sim::unstructured_map_preset_name(world.unstructured_preset())) << "\",\n";
    out << "  \"structured_preset\": \"" << json_escape(thesis_sim::structured_map_preset_name(world.structured_preset())) << "\",\n";
    out << "  \"gate_behavior\": \"" << json_escape(thesis_sim::gate_behavior_mode_name(sim.gate_behavior())) << "\",\n";
    out << "  \"gate_seed\": " << sim.gate_seed() << ",\n";
    out << "  \"config\": {\n";
    out << "    \"dt\": " << config.dt << ",\n";
    out << "    \"control_interval_steps\": " << config.control_interval_steps << ",\n";
    out << "    \"imu_enabled\": " << (config.imu_enabled ? "true" : "false") << ",\n";
    out << "    \"lidar_enabled\": " << (config.lidar_enabled ? "true" : "false") << ",\n";
    out << "    \"dynamic_lidar_gates\": " << (config.dynamic_lidar_gates ? "true" : "false") << ",\n";
    out << "    \"simulation_level\": \"" << (config.ideal_conditions ? "ideal" : "baseline") << "\",\n";
    out << "    \"gate_source\": \"" << (config.dynamic_lidar_gates ? "Simulated LiDAR dynamic gates" : "Static scenario gates") << "\",\n";
    out << "    \"range_sensor_profile\": \"" << json_escape(thesis_sim::range_sensor_profile_name(config.range_sensor_profile)) << "\",\n";
    out << "    \"lidar_beams\": " << sim.active_lidar_beams() << ",\n";
    out << "    \"lidar_fov_rad\": " << sim.active_lidar_fov_rad() << ",\n";
    out << "    \"lidar_range\": " << sim.active_lidar_range() << ",\n";
    out << "    \"vehicle_model\": \"" << json_escape(thesis_sim::vehicle_model_kind_name(sim.vehicle_model_kind())) << "\",\n";
    out << "    \"tracking_layer\": \"" << json_escape(thesis_sim::tracking_controller_mode_name(sim.tracking_controller_mode())) << "\",\n";
    out << "    \"vehicle_geometry\": {\n";
    out << "      \"wheelbase\": " << sim.geometry().wheelbase << ",\n";
    out << "      \"track\": " << sim.geometry().track << ",\n";
    out << "      \"body_length\": " << sim.geometry().body_length << ",\n";
    out << "      \"body_width\": " << sim.geometry().body_width << ",\n";
    out << "      \"max_linear_speed\": " << sim.geometry().max_linear_speed << ",\n";
    out << "      \"max_curvature\": " << sim.geometry().max_curvature << ",\n";
    out << "      \"max_steer_angle\": " << sim.geometry().max_steer_angle << ",\n";
    out << "      \"max_steer_rate\": " << sim.geometry().max_steer_rate << ",\n";
    out << "      \"yaw_response_scale\": " << sim.geometry().yaw_response_scale << "\n";
    out << "    }\n";
    out << "  },\n";
    out << "  \"scenario\": {\n";
    out << "    \"bounds\": ";
    write_rect_json(out, world.bounds());
    out << ",\n";
    out << "    \"start\": ";
    write_vec2_json(out, world.start());
    out << ",\n";
    out << "    \"goal\": ";
    write_vec2_json(out, world.goal());
    out << ",\n";
    out << "    \"start_heading_rad\": " << world.start_heading() << ",\n";
    out << "    \"obstacles\": [\n";
    for (size_t i = 0; i < world.obstacles().size(); ++i) {
        out << "      ";
        write_rect_json(out, world.obstacles()[i]);
        out << (i + 1 < world.obstacles().size() ? ",\n" : "\n");
    }
    out << "    ],\n";
    out << "    \"gates\": [\n";
    for (size_t i = 0; i < world.gates().size(); ++i) {
        const GateSpec& gate = world.gates()[i];
        out << "      {\"name\":\"" << json_escape(gate.name) << "\",\"position\":";
        write_vec2_json(out, gate.position);
        out << ",\"anchor_position\":";
        write_vec2_json(out, gate.anchor_position);
        out << ",\"motion_amplitude\":";
        write_vec2_json(out, gate.motion_amplitude);
        out << ",\"motion_frequency_hz\":" << gate.motion_frequency_hz
            << ",\"motion_phase_rad\":" << gate.motion_phase_rad
            << ",\"heading_hint\":" << gate.heading_hint
            << ",\"final\":" << (gate.final ? "true" : "false") << "}";
        out << (i + 1 < world.gates().size() ? ",\n" : "\n");
    }
    out << "    ],\n";
    out << "    \"road_centerline\": [\n";
    for (size_t i = 0; i < world.road_centerline().size(); ++i) {
        out << "      ";
        write_vec2_json(out, world.road_centerline()[i]);
        out << (i + 1 < world.road_centerline().size() ? ",\n" : "\n");
    }
    out << "    ]\n";
    out << "  },\n";
    out << "  \"summary\": {\n";
    out << "    \"steps\": " << report.steps << ",\n";
    out << "    \"sim_time\": " << report.sim_time << ",\n";
    out << "    \"distance_to_goal\": " << report.distance_to_goal << ",\n";
    out << "    \"passed_gates\": " << report.passed_gates << ",\n";
    out << "    \"final_position\": ";
    write_vec2_json(out, report.final_position);
    out << ",\n";
    out << "    \"final_yaw_rad\": " << sim.vehicle().yaw << ",\n";
    out << "    \"final_speed\": " << sim.vehicle().speed << "\n";
    out << "  },\n";
    out << "  \"performance\": {\n";
    out << "    \"planning_ms\": {\"avg\": " << planning_summary.avg << ",\"max\": " << planning_summary.max << "},\n";
    out << "    \"tracking_ms\": {\"avg\": " << tracking_summary.avg << ",\"max\": " << tracking_summary.max << "},\n";
    out << "    \"lidar_ms\": {\"avg\": " << lidar_summary.avg << ",\"max\": " << lidar_summary.max << "},\n";
    out << "    \"estimator_ms\": {\"avg\": " << estimator_summary.avg << ",\"max\": " << estimator_summary.max << "},\n";
    out << "    \"step_ms\": {\"avg\": " << step_summary.avg << ",\"max\": " << step_summary.max << "}\n";
    out << "  },\n";
    out << "  \"telemetry\": [\n";
    bool first = true;
    for (size_t i = 0; i < history.size(); i += stride) {
        const TelemetrySample& sample = history[i];
        if (!first) {
            out << ",\n";
        }
        first = false;
        out << "    {"
            << "\"time\":" << sample.time
            << ",\"x\":" << sample.x
            << ",\"y\":" << sample.y
            << ",\"speed\":" << sample.speed
            << ",\"accel\":" << sample.accel
            << ",\"yaw\":" << sample.yaw
            << ",\"curvature\":" << sample.curvature
            << ",\"yaw_rate\":" << sample.yaw_rate
            << ",\"steer_angle\":" << sample.steer_angle
            << ",\"target_steer_angle\":" << sample.target_steer_angle
            << ",\"target_speed\":" << sample.target_speed
            << ",\"target_yaw_rate\":" << sample.target_yaw_rate
            << ",\"left_wheel_speed\":" << sample.left_wheel_speed
            << ",\"right_wheel_speed\":" << sample.right_wheel_speed
            << ",\"left_pwm\":" << sample.left_pwm
            << ",\"right_pwm\":" << sample.right_pwm
            << ",\"distance_to_goal\":" << sample.distance_to_goal
            << ",\"nav_xy_error\":" << sample.nav_xy_error
            << ",\"nav_yaw_error_deg\":" << sample.nav_yaw_error_deg
            << ",\"tracker_cross_track\":" << sample.tracker_cross_track
            << ",\"tracker_heading_error_deg\":" << sample.tracker_heading_error_deg
            << ",\"planning_ms\":" << sample.planning_ms
            << ",\"tracking_ms\":" << sample.tracking_ms
            << ",\"lidar_ms\":" << sample.lidar_ms
            << ",\"estimator_ms\":" << sample.estimator_ms
            << ",\"step_ms\":" << sample.step_ms
            << ",\"visible_gates\":" << sample.visible_gates
            << ",\"lidar_samples\":" << sample.lidar_samples
            << ",\"chosen_gate_index\":" << sample.chosen_gate_index
            << ",\"chosen_gate_distance\":" << sample.chosen_gate_distance
            << ",\"passed_gates\":" << sample.passed_gates
            << ",\"candidate_gates\":" << sample.candidate_gates
            << ",\"dynamic_gap_gates\":" << sample.dynamic_gap_gates
            << ",\"planner_has_reference\":" << sample.planner_has_reference
            << ",\"mixed_mode\":" << sample.mixed_mode
            << ",\"mixed_gate_score\":" << sample.mixed_gate_score
            << ",\"mixed_structured_score\":" << sample.mixed_structured_score
            << ",\"mixed_gate_confidence\":" << sample.mixed_gate_confidence
            << ",\"mixed_switches\":" << sample.mixed_switches
            << ",\"mixed_aborts\":" << sample.mixed_aborts
            << "}";
    }
    out << "\n  ]\n";
    out << "}\n";
    return true;
}

bool write_sim_csv_report(const PlannerDrivenVehicleSim& sim, const std::string& report_path) {
    std::ofstream out(report_path, std::ios::out | std::ios::trunc);
    if (!out.is_open()) {
        return false;
    }
    out << std::fixed << std::setprecision(6);
    out << "time,x,y,yaw,speed,target_speed,target_yaw_rate,jerk,command_r,distance_to_goal,min_lidar,"
           "tracker_cross_track,tracker_heading_error_deg,planning_ms,tracking_ms,lidar_ms,estimator_ms,step_ms,"
           "chosen_gate_index,chosen_gate_distance,candidate_gates,mixed_mode,mixed_gate_score,mixed_structured_score,"
           "mixed_switches,mixed_aborts\n";
    for (const TelemetrySample& sample : sim.history()) {
        out << sample.time << ','
            << sample.x << ','
            << sample.y << ','
            << sample.yaw << ','
            << sample.speed << ','
            << sample.target_speed << ','
            << sample.target_yaw_rate << ','
            << sample.jerk << ','
            << sample.command_r << ','
            << sample.distance_to_goal << ','
            << sample.min_lidar << ','
            << sample.tracker_cross_track << ','
            << sample.tracker_heading_error_deg << ','
            << sample.planning_ms << ','
            << sample.tracking_ms << ','
            << sample.lidar_ms << ','
            << sample.estimator_ms << ','
            << sample.step_ms << ','
            << sample.chosen_gate_index << ','
            << sample.chosen_gate_distance << ','
            << sample.candidate_gates << ','
            << sample.mixed_mode << ','
            << sample.mixed_gate_score << ','
            << sample.mixed_structured_score << ','
            << sample.mixed_switches << ','
            << sample.mixed_aborts << '\n';
    }
    return true;
}

bool write_hardware_csv_report(const HardwareViewerState& hardware, const std::string& report_path) {
    std::ofstream out(report_path, std::ios::out | std::ios::trunc);
    if (!out.is_open()) {
        return false;
    }
    out << std::fixed << std::setprecision(6);
    out << "time,x,y,yaw,speed,target_speed,target_yaw_rate,jerk,command_r,distance_to_goal,min_lidar,front_lidar,"
           "tracker_cross_track,tracker_heading_error_deg,planning_ms,tracking_ms,lidar_ms,estimator_ms,step_ms,"
           "pwm_left,pwm_right,controller_pwm_left,controller_pwm_right,controller_target_pwm_left,controller_target_pwm_right,"
           "chosen_gate_index,chosen_gate_distance,candidate_gates,safety_stop_active,planner_has_reference,dynamic_gap_gates\n";
    for (const HardwareTelemetrySample& sample : hardware.history) {
        out << sample.time << ','
            << sample.position_x << ','
            << sample.position_y << ','
            << sample.yaw << ','
            << sample.speed << ','
            << sample.target_speed << ','
            << sample.target_yaw_rate << ','
            << sample.jerk << ','
            << sample.command_r << ','
            << sample.distance_to_goal << ','
            << sample.min_lidar << ','
            << sample.front_lidar << ','
            << sample.tracker_cross_track << ','
            << sample.tracker_heading_error_deg << ','
            << sample.planning_ms << ','
            << sample.tracking_ms << ','
            << sample.lidar_ms << ','
            << sample.estimator_ms << ','
            << sample.step_ms << ','
            << sample.pwm_left << ','
            << sample.pwm_right << ','
            << sample.controller_pwm_left << ','
            << sample.controller_pwm_right << ','
            << sample.controller_target_pwm_left << ','
            << sample.controller_target_pwm_right << ','
            << sample.chosen_gate_index << ','
            << sample.chosen_gate_distance << ','
            << sample.candidate_gates << ','
            << sample.safety_stop_active << ','
            << sample.planner_has_reference << ','
            << sample.dynamic_gap_gates << '\n';
    }
    return true;
}

bool write_sim_markdown_summary(const PlannerDrivenVehicleSim& sim,
                                const std::string& status,
                                const std::string& report_path) {
    std::ofstream out(report_path, std::ios::out | std::ios::trunc);
    if (!out.is_open()) {
        return false;
    }
    const RunQualitySummary quality = summarize_run_quality(sim.history());
    out << std::fixed << std::setprecision(3);
    out << "# Planner Run Summary\n\n";
    out << "- Status: `" << status << "`\n";
    out << "- Environment: `" << thesis_sim::environment_mode_name(sim.environment_mode()) << "`\n";
    out << "- Map: `" << map_preset_name(sim.world()) << "`\n";
    out << "- Vehicle model: `" << thesis_sim::vehicle_model_kind_name(sim.vehicle_model_kind()) << "`\n";
    out << "- Controller: `" << thesis_sim::tracking_controller_mode_name(sim.tracking_controller_mode()) << "`\n";
    out << "- Dynamic gates: `" << (sim.config().dynamic_lidar_gates ? "enabled" : "disabled") << "`\n";
    out << "- Sensor profile: `" << thesis_sim::range_sensor_profile_name(sim.range_sensor_profile()) << "`\n\n";
    out << "| Metric | Value |\n";
    out << "| --- | ---: |\n";
    out << "| Runtime | " << sim.sim_time() << " s |\n";
    out << "| Steps | " << sim.step_count() << " |\n";
    out << "| Final goal distance | " << sim.distance_to_goal() << " m |\n";
    out << "| Passed gates | " << sim.passed_gate_count() << " |\n";
    out << "| Average speed | " << quality.avg_speed << " m/s |\n";
    out << "| Max cross-track error | " << quality.max_cross_track << " m |\n";
    out << "| Max heading error | " << quality.max_heading_error_deg << " deg |\n";
    out << "| Min LiDAR clearance | " << quality.min_lidar << " m |\n";
    out << "| Avg step time | " << quality.avg_step_ms << " ms |\n";
    out << "| Max step time | " << quality.max_step_ms << " ms |\n";
    out << "| Mixed switches | " << quality.mixed_switches << " |\n";
    out << "| Mixed aborts | " << quality.mixed_aborts << " |\n\n";

    const std::vector<TimelineEvent> events = build_sim_timeline(sim);
    if (!events.empty()) {
        out << "## Timeline\n\n";
        const size_t first = events.size() > 24 ? events.size() - 24 : 0;
        for (size_t i = first; i < events.size(); ++i) {
            out << "- " << events[i].time << " s: **" << events[i].label << "**";
            if (!events[i].detail.empty()) {
                out << " - " << events[i].detail;
            }
            out << '\n';
        }
    }
    return true;
}

bool write_hardware_markdown_summary(const HardwareViewerState& hardware,
                                     const UiState& ui_state,
                                     const LiveViewStreamServer& hardware_server,
                                     const std::string& report_path) {
    std::ofstream out(report_path, std::ios::out | std::ios::trunc);
    if (!out.is_open()) {
        return false;
    }
    const RunQualitySummary quality = summarize_run_quality(hardware.history);
    const VehicleModelKind configured_model =
        static_cast<VehicleModelKind>(ui_state.hardware_vehicle_model);
    out << std::fixed << std::setprecision(3);
    out << "# Hardware Run Summary\n\n";
    out << "- Listener: `" << (hardware_server.listening() ? "listening" : "off") << "`\n";
    out << "- Connected: `" << (hardware_server.connected() ? "yes" : "no") << "`\n";
    out << "- Port: `" << hardware_server.port() << "`\n";
    out << "- Configured robot: `" << thesis_sim::vehicle_model_kind_name(configured_model) << "`\n";
    out << "- Live robot: `" << (hardware.has_scene ? hardware.scene.vehicle_model_name : "waiting for scene") << "`\n";
    out << "- Launch command: `" << hardware_launch_hint(ui_state) << "`\n\n";
    out << "| Metric | Value |\n";
    out << "| --- | ---: |\n";
    out << "| Samples | " << hardware.history.size() << " |\n";
    out << "| Runtime | " << hardware.frame.sim_time << " s |\n";
    out << "| Goal distance | " << hardware.frame.distance_to_goal << " m |\n";
    out << "| Average speed | " << quality.avg_speed << " m/s |\n";
    out << "| Max cross-track error | " << quality.max_cross_track << " m |\n";
    out << "| Max heading error | " << quality.max_heading_error_deg << " deg |\n";
    out << "| Min LiDAR clearance | " << quality.min_lidar << " m |\n";
    out << "| Avg step time | " << quality.avg_step_ms << " ms |\n";
    out << "| Max step time | " << quality.max_step_ms << " ms |\n";
    out << "| Planner reference | " << (hardware.frame.planner_has_reference ? "ready" : "waiting") << " |\n";
    out << "| Safety stop | " << (hardware.frame.safety_stop_active ? "active" : "clear") << " |\n\n";

    const std::vector<TimelineEvent> events = build_hardware_timeline(hardware);
    if (!events.empty()) {
        out << "## Timeline\n\n";
        const size_t first = events.size() > 24 ? events.size() - 24 : 0;
        for (size_t i = first; i < events.size(); ++i) {
            out << "- " << events[i].time << " s: **" << events[i].label << "**";
            if (!events[i].detail.empty()) {
                out << " - " << events[i].detail;
            }
            out << '\n';
        }
    }
    return true;
}

bool parse_double_cli_arg(const std::string& arg,
                          int* index,
                          int argc,
                          char** argv,
                          const char* name,
                          std::optional<double>* target) {
    if (target == nullptr) {
        return false;
    }

    const std::string flag = std::string("--") + name;
    const std::string prefix = flag + "=";
    if (arg == flag && index != nullptr && *index + 1 < argc) {
        *target = std::atof(argv[++(*index)]);
        return true;
    }
    if (arg.rfind(prefix, 0) == 0) {
        *target = std::atof(arg.substr(prefix.size()).c_str());
        return true;
    }
    return false;
}

bool has_tuning_overrides(const VehicleTuningOverrides& overrides) {
    return overrides.min_effective_pwm.has_value() ||
           overrides.speed_estimate_per_pwm.has_value() ||
           overrides.pwm_slew_rate.has_value() ||
           overrides.motor_time_constant.has_value() ||
           overrides.max_linear_speed.has_value() ||
           overrides.max_curvature.has_value() ||
           overrides.max_steer_angle.has_value() ||
           overrides.max_steer_rate.has_value() ||
           overrides.max_yaw_rate.has_value() ||
           overrides.linear_feedback_gain.has_value() ||
           overrides.yaw_feedback_gain.has_value() ||
           overrides.left_pwm_scale.has_value() ||
           overrides.right_pwm_scale.has_value() ||
           overrides.yaw_response_scale.has_value() ||
           overrides.cruise_speed_limit.has_value();
}

void apply_sim_tuning_overrides(PlannerDrivenVehicleSim* sim, const AppOptions& options) {
    if (sim == nullptr || !has_tuning_overrides(options.tuning_overrides)) {
        return;
    }
    sim->set_tuning_overrides(options.tuning_overrides);
}

void fill_structured_tank_ideal_tuning_defaults(VehicleTuningOverrides* tuning) {
    if (tuning == nullptr) {
        return;
    }
    // Best-case tracked response for the structured ideal toggle; mixed maps
    // keep their existing baseline tuning.
    tuning->min_effective_pwm = 35.0;
    tuning->speed_estimate_per_pwm = 0.0018;
    tuning->pwm_slew_rate = 720.0;
    tuning->motor_time_constant = 0.10;
    tuning->max_linear_speed = 0.18;
    tuning->max_curvature = 6.5;
    tuning->max_steer_angle = 1.35;
    tuning->max_steer_rate = 5.5;
    tuning->max_yaw_rate = 1.60;
    tuning->linear_feedback_gain = 60.0;
    tuning->yaw_feedback_gain = 40.0;
    tuning->yaw_response_scale = 1.0;
    tuning->cruise_speed_limit = 0.12;
}

void apply_ideal_simulation_defaults(AppOptions* options) {
    if (options == nullptr || !options->ideal_simulation) {
        return;
    }
    if (options->environment_mode == EnvironmentMode::StructuredRoad &&
        options->vehicle_model == VehicleModelKind::TrackedVehicle) {
        fill_structured_tank_ideal_tuning_defaults(&options->tuning_overrides);
    }
}

void apply_sim_level_to_config(const AppOptions& options, thesis_sim::SimConfig* config) {
    if (config == nullptr || !options.ideal_simulation) {
        return;
    }
    constexpr double kPi = 3.14159265358979323846;
    config->ideal_conditions = true;
    config->imu_enabled = true;
    config->lidar_enabled = true;
    config->range_sensor_profile = RangeSensorProfile::IdealLidar2D;
    config->lidar_beams = 360;
    config->lidar_fov_rad = 2.0 * kPi;
    config->lidar_range = 12.0;
    config->control_interval_steps = 1;
}

bool structured_preset_is_ideal(StructuredMapPreset preset) {
    return preset == StructuredMapPreset::IdealCircle;
}

bool unstructured_preset_is_ideal(UnstructuredMapPreset preset) {
    return preset == UnstructuredMapPreset::IdealValidation;
}

bool mixed_preset_is_ideal(int preset) {
    return preset == 2;
}

bool ui_selection_uses_ideal_map(const UiState& ui_state) {
    const EnvironmentMode mode = static_cast<EnvironmentMode>(ui_state.environment_mode);
    if (mode == EnvironmentMode::StructuredRoad) {
        return structured_preset_is_ideal(static_cast<StructuredMapPreset>(ui_state.structured_preset));
    }
    if (mode == EnvironmentMode::UnstructuredGates) {
        return unstructured_preset_is_ideal(static_cast<UnstructuredMapPreset>(ui_state.unstructured_preset));
    }
    if (mode == EnvironmentMode::MixedRoadGates) {
        return mixed_preset_is_ideal(ui_state.mixed_preset);
    }
    return false;
}

void set_ui_ideal_map_for_mode(UiState* ui_state) {
    if (ui_state == nullptr) {
        return;
    }
    const EnvironmentMode selected_mode = static_cast<EnvironmentMode>(ui_state->environment_mode);
    if (selected_mode == EnvironmentMode::StructuredRoad) {
        ui_state->structured_preset = static_cast<int>(StructuredMapPreset::IdealCircle);
    } else if (selected_mode == EnvironmentMode::UnstructuredGates) {
        ui_state->unstructured_preset = static_cast<int>(UnstructuredMapPreset::IdealValidation);
    } else if (selected_mode == EnvironmentMode::MixedRoadGates) {
        ui_state->mixed_preset = 2;
    }
}

void apply_gui_stack_for_selected_map(PlannerDrivenVehicleSim& sim, UiState* ui_state) {
    if (ui_state == nullptr) {
        return;
    }

    const EnvironmentMode selected_mode = static_cast<EnvironmentMode>(ui_state->environment_mode);
    const bool ideal_map = ui_selection_uses_ideal_map(*ui_state);
    if (ideal_map) {
        if (selected_mode == EnvironmentMode::MixedRoadGates ||
            selected_mode == EnvironmentMode::UnstructuredGates) {
            sim.set_dynamic_lidar_gates(true);
        }
        VehicleTuningOverrides ideal_tuning;
        if (selected_mode == EnvironmentMode::StructuredRoad &&
            static_cast<VehicleModelKind>(ui_state->vehicle_model) == VehicleModelKind::TrackedVehicle) {
            fill_structured_tank_ideal_tuning_defaults(&ideal_tuning);
        }
        sim.set_tuning_overrides(ideal_tuning);
        sim.set_ideal_conditions(true);
    } else {
        if (sim.config().ideal_conditions) {
            sim.set_tuning_overrides(VehicleTuningOverrides{});
            sim.set_ideal_conditions(false);
        }
        if (selected_mode == EnvironmentMode::MixedRoadGates && !sim.config().dynamic_lidar_gates) {
            sim.set_dynamic_lidar_gates(true);
        }
    }

    sync_ui_state_from_sim(ui_state, sim);
    ui_state->paused = true;
    ui_state->report_written = false;
    ui_state->last_report_path.clear();
}

const std::array<ValidationPreset, 6>& validation_presets() {
    static const std::array<ValidationPreset, 6> presets{{
        {"Bicycle structured",
         "Baseline structured-road validation.",
         EnvironmentMode::StructuredRoad,
         UnstructuredMapPreset::RobotValidation,
         StructuredMapPreset::ValidationRoad,
         0,
         VehicleModelKind::CarLikeBicycle,
         false,
         false},
        {"Tank structured",
         "Tracked vehicle on the same structured validation road.",
         EnvironmentMode::StructuredRoad,
         UnstructuredMapPreset::RobotValidation,
         StructuredMapPreset::ValidationRoad,
         0,
         VehicleModelKind::TrackedVehicle,
         false,
         false},
        {"Bicycle mixed",
         "Road/gate arbiter with LiDAR gaps and bicycle support.",
         EnvironmentMode::MixedRoadGates,
         UnstructuredMapPreset::RobotValidation,
         StructuredMapPreset::ValidationRoad,
         0,
         VehicleModelKind::CarLikeBicycle,
         true,
         false},
        {"Tank mixed",
         "Tracked vehicle with Sabrina angular primitive on mixed validation.",
         EnvironmentMode::MixedRoadGates,
         UnstructuredMapPreset::RobotValidation,
         StructuredMapPreset::ValidationRoad,
         0,
         VehicleModelKind::TrackedVehicle,
         true,
         false},
        {"Tank hardware lab",
         "Small hardware-aligned mixed map for the cingolato.",
         EnvironmentMode::MixedRoadGates,
         UnstructuredMapPreset::RobotValidation,
         StructuredMapPreset::HardwareTrack,
         1,
         VehicleModelKind::TrackedVehicle,
         true,
         false},
        {"Tank ideal aligned",
         "Ideal sensing plus hardware-aligned mixed map for paper figures.",
         EnvironmentMode::MixedRoadGates,
         UnstructuredMapPreset::IdealValidation,
         StructuredMapPreset::IdealCircle,
         2,
         VehicleModelKind::TrackedVehicle,
         true,
         true},
    }};
    return presets;
}

void apply_validation_preset(PlannerDrivenVehicleSim& sim,
                             UiState* ui_state,
                             const ValidationPreset& preset) {
    if (ui_state == nullptr) {
        return;
    }
    ui_state->environment_mode = static_cast<int>(preset.environment);
    ui_state->unstructured_preset = static_cast<int>(preset.unstructured);
    ui_state->structured_preset = static_cast<int>(preset.structured);
    ui_state->mixed_preset = preset.mixed_preset;
    ui_state->vehicle_model = static_cast<int>(preset.vehicle);
    ui_state->ideal_simulation = preset.ideal;
    ui_state->dynamic_lidar_gates = preset.dynamic_gates;

    WorldMap world = make_world_from_mode(
        preset.environment,
        preset.unstructured,
        preset.structured,
        sim.gate_behavior(),
        sim.gate_seed(),
        preset.mixed_preset);
    world = fit_simulation_structured_world(std::move(world), preset.vehicle);
    sim.load_world(std::move(world));
    sim.set_vehicle_stack(preset.vehicle, sim.tracking_controller_mode());
    if (preset.ideal) {
        VehicleTuningOverrides tuning;
        if (preset.environment == EnvironmentMode::StructuredRoad &&
            preset.vehicle == VehicleModelKind::TrackedVehicle) {
            fill_structured_tank_ideal_tuning_defaults(&tuning);
        }
        sim.set_tuning_overrides(tuning);
        sim.set_ideal_conditions(true);
    } else {
        sim.set_tuning_overrides(VehicleTuningOverrides{});
        sim.set_ideal_conditions(false);
    }
    sim.set_dynamic_lidar_gates(preset.dynamic_gates);
    sync_ui_state_from_sim(ui_state, sim);
    ui_state->workspace_source = kWorkspaceSourceSimulation;
    ui_state->workspace_view = kWorkspaceViewMission;
    ui_state->simulation_panel_tab = 0;
    ui_state->requested_simulation_panel_tab = 0;
    ui_state->paused = true;
}

std::string run_vehicle_benchmark(const PlannerDrivenVehicleSim& sim, const UiState& ui_state) {
    auto run_one = [&](VehicleModelKind model) {
        thesis_sim::SimConfig config = sim.config();
        config.vehicle_model = model;
        UiState candidate_ui = ui_state;
        candidate_ui.vehicle_model = static_cast<int>(model);
        PlannerDrivenVehicleSim candidate(world_from_ui_selection(sim, candidate_ui), config);
        const int max_steps =
            ui_state.environment_mode == static_cast<int>(EnvironmentMode::MixedRoadGates) ? 2200 : 1800;
        return candidate.run_headless(max_steps);
    };

    const SimulationReport bicycle = run_one(VehicleModelKind::CarLikeBicycle);
    const SimulationReport tank = run_one(VehicleModelKind::TrackedVehicle);
    auto status = [](const SimulationReport& report) {
        return report.goal_reached ? "goal" : (report.collision ? "collision" : "timeout");
    };

    std::ostringstream out;
    out << std::fixed << std::setprecision(3);
    out << "Bicycle: " << status(bicycle)
        << ", t " << bicycle.sim_time
        << " s, goal " << bicycle.distance_to_goal
        << " m | Tank: " << status(tank)
        << ", t " << tank.sim_time
        << " s, goal " << tank.distance_to_goal
        << " m";
    return out.str();
}

AppOptions parse_args(int argc, char** argv) {
    AppOptions options;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--headless") {
            options.headless = true;
        } else if (arg == "--dynamic-lidar-gates" || arg == "--lidar-dynamic-gates") {
            options.dynamic_lidar_gates = true;
        } else if (arg == "--static-gates") {
            options.dynamic_lidar_gates = false;
        } else if (arg == "--ideal-sim" || arg == "--perfect-sim") {
            options.ideal_simulation = true;
        } else if (arg == "--sim-level" && i + 1 < argc) {
            const std::string value = argv[++i];
            options.ideal_simulation = value == "ideal" || value == "perfect";
        } else if (arg.rfind("--sim-level=", 0) == 0) {
            const std::string value = arg.substr(std::strlen("--sim-level="));
            options.ideal_simulation = value == "ideal" || value == "perfect";
        } else if (arg == "--scenario" && i + 1 < argc) {
            const std::string value = argv[++i];
            if (value == "structured") {
                options.environment_mode = EnvironmentMode::StructuredRoad;
            } else if (value == "mixed" || value == "mixed_road_gates") {
                options.environment_mode = EnvironmentMode::MixedRoadGates;
                options.dynamic_lidar_gates = true;
            } else if (value == "mixed_hardware" || value == "mixed-hardware" ||
                       value == "mixed_hardware_aligned" || value == "mixed-hardware-aligned") {
                options.environment_mode = EnvironmentMode::MixedRoadGates;
                options.dynamic_lidar_gates = true;
                options.mixed_preset = 1;
            } else {
                options.environment_mode = EnvironmentMode::UnstructuredGates;
            }
        } else if (arg == "--scenario=structured") {
            options.environment_mode = EnvironmentMode::StructuredRoad;
        } else if (arg == "--scenario=unstructured") {
            options.environment_mode = EnvironmentMode::UnstructuredGates;
        } else if (arg == "--scenario=mixed") {
            options.environment_mode = EnvironmentMode::MixedRoadGates;
            options.dynamic_lidar_gates = true;
        } else if (arg == "--scenario=mixed_hardware" || arg == "--scenario=mixed-hardware" ||
                   arg == "--scenario=mixed_hardware_aligned" || arg == "--scenario=mixed-hardware-aligned") {
            options.environment_mode = EnvironmentMode::MixedRoadGates;
            options.dynamic_lidar_gates = true;
            options.mixed_preset = 1;
        } else if (arg == "--mixed-map" && i + 1 < argc) {
            const std::string value = argv[++i];
            if (value == "ideal" || value == "perfect") {
                options.mixed_preset = 2;
                options.ideal_simulation = true;
                options.dynamic_lidar_gates = true;
            } else {
                options.mixed_preset =
                    (value == "hardware" || value == "hardware_aligned" || value == "hardware-aligned" ||
                     value == "hardware_lab" || value == "hardware-lab") ? 1 : 0;
            }
        } else if (arg.rfind("--mixed-map=", 0) == 0) {
            const std::string value = arg.substr(std::strlen("--mixed-map="));
            if (value == "ideal" || value == "perfect") {
                options.mixed_preset = 2;
                options.ideal_simulation = true;
                options.dynamic_lidar_gates = true;
            } else {
                options.mixed_preset =
                    (value == "hardware" || value == "hardware_aligned" || value == "hardware-aligned" ||
                     value == "hardware_lab" || value == "hardware-lab") ? 1 : 0;
            }
        } else if (arg == "--unstructured-map" && i + 1 < argc) {
            const std::string value = argv[++i];
            if (value == "tight") {
                options.unstructured_preset = UnstructuredMapPreset::TightCorridor;
            } else if (value == "slalom") {
                options.unstructured_preset = UnstructuredMapPreset::WideSlalom;
            } else if (value == "lower") {
                options.unstructured_preset = UnstructuredMapPreset::LowerBypass;
            } else if (value == "hardware_lab" || value == "hardware" || value == "lab") {
                options.unstructured_preset = UnstructuredMapPreset::HardwareLab;
            } else if (value == "ideal" || value == "perfect" || value == "ideal_validation" ||
                       value == "perfect_validation") {
                options.unstructured_preset = UnstructuredMapPreset::IdealValidation;
                options.ideal_simulation = true;
                options.dynamic_lidar_gates = true;
            } else {
                options.unstructured_preset = UnstructuredMapPreset::RobotValidation;
            }
        } else if (arg == "--structured-map" && i + 1 < argc) {
            const std::string value = argv[++i];
            if (value == "circle") {
                options.structured_preset = StructuredMapPreset::CircleLoop;
            } else if (value == "zigzag") {
                options.structured_preset = StructuredMapPreset::ZigZag;
            } else if (value == "hardware" || value == "hardware_track") {
                options.structured_preset = StructuredMapPreset::HardwareTrack;
            } else if (value == "figure_eight" || value == "figure8" || value == "eight") {
                options.structured_preset = StructuredMapPreset::FigureEight;
            } else if (value == "tank_circuit" || value == "circuit" || value == "practice_circuit") {
                options.structured_preset = StructuredMapPreset::TankCircuit;
            } else if (value == "ideal" || value == "perfect" || value == "ideal_circle" ||
                       value == "perfect_circle") {
                options.structured_preset = StructuredMapPreset::IdealCircle;
                options.ideal_simulation = true;
            } else {
                options.structured_preset = StructuredMapPreset::ValidationRoad;
            }
        } else if (arg.rfind("--unstructured-map=", 0) == 0) {
            const std::string value = arg.substr(std::strlen("--unstructured-map="));
            if (value == "tight") {
                options.unstructured_preset = UnstructuredMapPreset::TightCorridor;
            } else if (value == "slalom") {
                options.unstructured_preset = UnstructuredMapPreset::WideSlalom;
            } else if (value == "lower") {
                options.unstructured_preset = UnstructuredMapPreset::LowerBypass;
            } else if (value == "hardware_lab" || value == "hardware" || value == "lab") {
                options.unstructured_preset = UnstructuredMapPreset::HardwareLab;
            } else if (value == "ideal" || value == "perfect" || value == "ideal_validation" ||
                       value == "perfect_validation") {
                options.unstructured_preset = UnstructuredMapPreset::IdealValidation;
                options.ideal_simulation = true;
                options.dynamic_lidar_gates = true;
            } else {
                options.unstructured_preset = UnstructuredMapPreset::RobotValidation;
            }
        } else if (arg.rfind("--structured-map=", 0) == 0) {
            const std::string value = arg.substr(std::strlen("--structured-map="));
            if (value == "circle") {
                options.structured_preset = StructuredMapPreset::CircleLoop;
            } else if (value == "zigzag") {
                options.structured_preset = StructuredMapPreset::ZigZag;
            } else if (value == "hardware" || value == "hardware_track") {
                options.structured_preset = StructuredMapPreset::HardwareTrack;
            } else if (value == "figure_eight" || value == "figure8" || value == "eight") {
                options.structured_preset = StructuredMapPreset::FigureEight;
            } else if (value == "tank_circuit" || value == "circuit" || value == "practice_circuit") {
                options.structured_preset = StructuredMapPreset::TankCircuit;
            } else if (value == "ideal" || value == "perfect" || value == "ideal_circle" ||
                       value == "perfect_circle") {
                options.structured_preset = StructuredMapPreset::IdealCircle;
                options.ideal_simulation = true;
            } else {
                options.structured_preset = StructuredMapPreset::ValidationRoad;
            }
        } else if (arg == "--vehicle-model" && i + 1 < argc) {
            const std::string value = argv[++i];
            if (value == "tracked" || value == "tracked_vehicle" || value == "skid" ||
                value == "skid_steer" || value == "tank") {
                options.vehicle_model = VehicleModelKind::TrackedVehicle;
            } else {
                options.vehicle_model = VehicleModelKind::CarLikeBicycle;
            }
        } else if (arg.rfind("--vehicle-model=", 0) == 0) {
            const std::string value = arg.substr(std::strlen("--vehicle-model="));
            if (value == "tracked" || value == "tracked_vehicle" || value == "skid" ||
                value == "skid_steer" || value == "tank") {
                options.vehicle_model = VehicleModelKind::TrackedVehicle;
            } else {
                options.vehicle_model = VehicleModelKind::CarLikeBicycle;
            }
        } else if (parse_double_cli_arg(arg, &i, argc, argv, "min-effective-pwm", &options.tuning_overrides.min_effective_pwm) ||
                   parse_double_cli_arg(arg, &i, argc, argv, "speed-estimate-per-pwm", &options.tuning_overrides.speed_estimate_per_pwm) ||
                   parse_double_cli_arg(arg, &i, argc, argv, "pwm-slew-rate", &options.tuning_overrides.pwm_slew_rate) ||
                   parse_double_cli_arg(arg, &i, argc, argv, "motor-time-constant", &options.tuning_overrides.motor_time_constant) ||
                   parse_double_cli_arg(arg, &i, argc, argv, "max-linear-speed", &options.tuning_overrides.max_linear_speed) ||
                   parse_double_cli_arg(arg, &i, argc, argv, "max-curvature", &options.tuning_overrides.max_curvature) ||
                   parse_double_cli_arg(arg, &i, argc, argv, "max-steer-angle", &options.tuning_overrides.max_steer_angle) ||
                   parse_double_cli_arg(arg, &i, argc, argv, "max-steer-rate", &options.tuning_overrides.max_steer_rate) ||
                   parse_double_cli_arg(arg, &i, argc, argv, "max-yaw-rate", &options.tuning_overrides.max_yaw_rate) ||
                   parse_double_cli_arg(arg, &i, argc, argv, "linear-feedback-gain", &options.tuning_overrides.linear_feedback_gain) ||
                   parse_double_cli_arg(arg, &i, argc, argv, "yaw-feedback-gain", &options.tuning_overrides.yaw_feedback_gain) ||
                   parse_double_cli_arg(arg, &i, argc, argv, "left-pwm-scale", &options.tuning_overrides.left_pwm_scale) ||
                   parse_double_cli_arg(arg, &i, argc, argv, "right-pwm-scale", &options.tuning_overrides.right_pwm_scale) ||
                   parse_double_cli_arg(arg, &i, argc, argv, "yaw-response-scale", &options.tuning_overrides.yaw_response_scale) ||
                   parse_double_cli_arg(arg, &i, argc, argv, "cruise-speed-limit", &options.tuning_overrides.cruise_speed_limit)) {
        } else if (arg == "--max-steps" && i + 1 < argc) {
            options.max_steps = std::atoi(argv[++i]);
        } else if (arg.rfind("--max-steps=", 0) == 0) {
            options.max_steps = std::atoi(arg.substr(std::strlen("--max-steps=")).c_str());
        }
    }
    options.max_steps = std::max(options.max_steps, 1);
    apply_ideal_simulation_defaults(&options);
    return options;
}

WorldMap make_world_from_mode(EnvironmentMode mode,
                              UnstructuredMapPreset preset,
                              StructuredMapPreset structured_preset,
                              GateBehaviorMode gate_behavior,
                              std::uint32_t gate_seed,
                              int mixed_preset) {
    if (mode == EnvironmentMode::StructuredRoad) {
        return WorldMap::structured_demo(structured_preset);
    }
    if (mode == EnvironmentMode::MixedRoadGates) {
        if (mixed_preset == 2) {
            return WorldMap::mixed_ideal_demo();
        }
        if (mixed_preset == 1) {
            return WorldMap::mixed_hardware_aligned_demo();
        }
        return WorldMap::mixed_demo();
    }
    if (preset == UnstructuredMapPreset::Custom) {
        return WorldMap::unstructured_demo(UnstructuredMapPreset::Custom, GateBehaviorMode::Static, 0);
    }
    return WorldMap::unstructured_demo(preset, gate_behavior, gate_seed);
}

void apply_style() {
    ImGuiStyle& style = ImGui::GetStyle();
    style.WindowBorderSize = 1.0f;
    style.ChildBorderSize = 1.0f;
    style.FrameBorderSize = 1.0f;
    style.WindowRounding = 14.0f;
    style.ChildRounding = 12.0f;
    style.FrameRounding = 8.0f;
    style.GrabRounding = 8.0f;
    style.PopupRounding = 10.0f;
    style.ScrollbarRounding = 12.0f;
    style.TabRounding = 10.0f;
    style.WindowPadding = ImVec2(14.0f, 14.0f);
    style.FramePadding = ImVec2(10.0f, 7.0f);
    style.ItemSpacing = ImVec2(10.0f, 10.0f);
    style.CellPadding = ImVec2(8.0f, 8.0f);
    style.ScrollbarSize = 14.0f;

    ImVec4* colors = style.Colors;
    colors[ImGuiCol_Text] = ImVec4(0.93f, 0.95f, 0.92f, 1.0f);
    colors[ImGuiCol_TextDisabled] = ImVec4(0.58f, 0.64f, 0.68f, 1.0f);
    colors[ImGuiCol_WindowBg] = ImVec4(0.05f, 0.07f, 0.09f, 1.0f);
    colors[ImGuiCol_ChildBg] = ImVec4(0.07f, 0.09f, 0.12f, 1.0f);
    colors[ImGuiCol_PopupBg] = ImVec4(0.09f, 0.11f, 0.14f, 0.98f);
    colors[ImGuiCol_Border] = ImVec4(0.20f, 0.25f, 0.29f, 1.0f);
    colors[ImGuiCol_FrameBg] = ImVec4(0.12f, 0.15f, 0.18f, 1.0f);
    colors[ImGuiCol_FrameBgHovered] = ImVec4(0.17f, 0.21f, 0.25f, 1.0f);
    colors[ImGuiCol_FrameBgActive] = ImVec4(0.20f, 0.25f, 0.30f, 1.0f);
    colors[ImGuiCol_Button] = ImVec4(0.13f, 0.23f, 0.27f, 1.0f);
    colors[ImGuiCol_ButtonHovered] = ImVec4(0.17f, 0.31f, 0.36f, 1.0f);
    colors[ImGuiCol_ButtonActive] = ImVec4(0.10f, 0.19f, 0.23f, 1.0f);
    colors[ImGuiCol_Header] = ImVec4(0.12f, 0.22f, 0.26f, 1.0f);
    colors[ImGuiCol_HeaderHovered] = ImVec4(0.16f, 0.30f, 0.35f, 1.0f);
    colors[ImGuiCol_HeaderActive] = ImVec4(0.14f, 0.26f, 0.31f, 1.0f);
    colors[ImGuiCol_TitleBg] = ImVec4(0.07f, 0.09f, 0.12f, 1.0f);
    colors[ImGuiCol_TitleBgActive] = ImVec4(0.09f, 0.13f, 0.16f, 1.0f);
    colors[ImGuiCol_Tab] = ImVec4(0.09f, 0.12f, 0.15f, 1.0f);
    colors[ImGuiCol_TabHovered] = ImVec4(0.14f, 0.25f, 0.30f, 1.0f);
    colors[ImGuiCol_TabActive] = ImVec4(0.12f, 0.19f, 0.23f, 1.0f);
    colors[ImGuiCol_TableBorderStrong] = ImVec4(0.18f, 0.23f, 0.27f, 1.0f);
    colors[ImGuiCol_TableBorderLight] = ImVec4(0.11f, 0.15f, 0.18f, 1.0f);
    colors[ImGuiCol_Separator] = ImVec4(0.20f, 0.25f, 0.29f, 1.0f);
    colors[ImGuiCol_PlotLines] = ImVec4(0.53f, 0.87f, 0.97f, 1.0f);
    colors[ImGuiCol_PlotHistogram] = ImVec4(0.96f, 0.67f, 0.31f, 1.0f);
}

CanvasTransform make_transform(const WorldMap& world, const ImVec2& canvas_pos, const ImVec2& canvas_size) {
    const Rect bounds = world.bounds();
    const float pad = 24.0f;
    const float width = static_cast<float>(bounds.max_x - bounds.min_x);
    const float height = static_cast<float>(bounds.max_y - bounds.min_y);
    const float scale_x = std::max(1.0f, (canvas_size.x - pad * 2.0f) / width);
    const float scale_y = std::max(1.0f, (canvas_size.y - pad * 2.0f) / height);

    CanvasTransform tx;
    tx.scale = std::min(scale_x, scale_y);
    tx.origin = ImVec2(
        canvas_pos.x + pad - static_cast<float>(bounds.min_x) * tx.scale,
        canvas_pos.y + canvas_size.y - pad + static_cast<float>(bounds.min_y) * tx.scale);
    return tx;
}

ImVec2 world_to_screen(const CanvasTransform& tx, const Vec2& p) {
    return ImVec2(
        tx.origin.x + static_cast<float>(p.x) * tx.scale,
        tx.origin.y - static_cast<float>(p.y) * tx.scale);
}

Vec2 screen_to_world(const CanvasTransform& tx, const ImVec2& p) {
    return {
        static_cast<double>((p.x - tx.origin.x) / tx.scale),
        static_cast<double>((tx.origin.y - p.y) / tx.scale),
    };
}

float distance_sq(const ImVec2& a, const ImVec2& b) {
    const float dx = a.x - b.x;
    const float dy = a.y - b.y;
    return dx * dx + dy * dy;
}

const char* map_editor_handle_type_name(MapEditorHandleType type) {
    switch (type) {
        case MapEditorHandleType::Start:
            return "Start";
        case MapEditorHandleType::Goal:
            return "Goal";
        case MapEditorHandleType::Obstacle:
            return "Obstacle";
        case MapEditorHandleType::Gate:
            return "Gate";
        case MapEditorHandleType::RoadPoint:
            return "Road Point";
        default:
            return "None";
    }
}

void draw_polyline(ImDrawList* draw_list, const CanvasTransform& tx, const std::vector<Vec2>& points, ImU32 color, float thickness) {
    if (points.size() < 2) {
        return;
    }

    std::vector<ImVec2> screen_points;
    screen_points.reserve(points.size());
    for (const Vec2& point : points) {
        screen_points.push_back(world_to_screen(tx, point));
    }

    const bool closed_loop = points.size() >= 3 && thesis_sim::distance(points.front(), points.back()) <= 0.45;
    draw_list->AddPolyline(
        screen_points.data(),
        static_cast<int>(screen_points.size()),
        color,
        closed_loop ? ImDrawFlags_Closed : 0,
        thickness);
}

double grid_step_for_bounds(const Rect& bounds) {
    const double span = std::max(bounds.max_x - bounds.min_x, bounds.max_y - bounds.min_y);
    if (span <= 1.2) {
        return 0.10;
    }
    if (span <= 3.0) {
        return 0.25;
    }
    if (span <= 8.0) {
        return 0.50;
    }
    return 2.0;
}

void draw_world_grid(ImDrawList* draw_list, const CanvasTransform& tx, const Rect& bounds) {
    const double step = grid_step_for_bounds(bounds);
    const double first_x = std::ceil(bounds.min_x / step) * step;
    const double first_y = std::ceil(bounds.min_y / step) * step;
    for (double gx = first_x; gx <= bounds.max_x + step * 0.25; gx += step) {
        draw_list->AddLine(world_to_screen(tx, {gx, bounds.min_y}),
                           world_to_screen(tx, {gx, bounds.max_y}),
                           kColorGrid,
                           1.0f);
    }
    for (double gy = first_y; gy <= bounds.max_y + step * 0.25; gy += step) {
        draw_list->AddLine(world_to_screen(tx, {bounds.min_x, gy}),
                           world_to_screen(tx, {bounds.max_x, gy}),
                           kColorGrid,
                           1.0f);
    }
    draw_list->AddRect(world_to_screen(tx, {bounds.min_x, bounds.max_y}),
                       world_to_screen(tx, {bounds.max_x, bounds.min_y}),
                       kColorBounds,
                       0.0f,
                       0,
                       1.3f);
}

double structured_road_width_for_world(const WorldMap& world) {
    if (world.environment_mode() != EnvironmentMode::StructuredRoad &&
        world.environment_mode() != EnvironmentMode::MixedRoadGates) {
        return 0.0;
    }
    if (world.environment_mode() == EnvironmentMode::MixedRoadGates) {
        return 1.20;
    }
    const Rect& bounds = world.bounds();
    const double span = std::max(bounds.max_x - bounds.min_x, bounds.max_y - bounds.min_y);
    if (span <= 0.35) {
        return std::clamp(span * 0.18, 0.045, 0.065);
    }
    return span <= 5.0 ? 0.14 : 3.0;
}

std::vector<Vec2> make_offset_polyline(const std::vector<Vec2>& points, double offset) {
    if (points.size() < 2) {
        return {};
    }

    std::vector<Vec2> base = points;
    const bool closed_loop = base.size() >= 3 && thesis_sim::distance(base.front(), base.back()) <= 0.45;
    if (closed_loop && base.size() > 2) {
        base.pop_back();
    }
    if (base.size() < 2) {
        return {};
    }

    const auto segment_normal = [&](size_t from, size_t to) {
        const Vec2& a = base[from];
        const Vec2& b = base[to];
        const double dx = b.x - a.x;
        const double dy = b.y - a.y;
        const double length = std::hypot(dx, dy);
        if (length <= 1e-9) {
            return Vec2{0.0, 0.0};
        }
        return Vec2{-dy / length, dx / length};
    };

    std::vector<Vec2> out;
    out.reserve(base.size() + (closed_loop ? 1 : 0));
    for (size_t i = 0; i < base.size(); ++i) {
        Vec2 normal{};
        if (closed_loop) {
            const size_t previous = i == 0 ? base.size() - 1 : i - 1;
            const size_t next = (i + 1) % base.size();
            const Vec2 prev_normal = segment_normal(previous, i);
            const Vec2 next_normal = segment_normal(i, next);
            normal = {prev_normal.x + next_normal.x, prev_normal.y + next_normal.y};
            const double normal_length = std::hypot(normal.x, normal.y);
            if (normal_length > 1e-9) {
                normal.x /= normal_length;
                normal.y /= normal_length;
            } else {
                normal = next_normal;
            }
        } else if (i == 0) {
            normal = segment_normal(0, 1);
        } else if (i + 1 == base.size()) {
            normal = segment_normal(i - 1, i);
        } else {
            const Vec2 prev_normal = segment_normal(i - 1, i);
            const Vec2 next_normal = segment_normal(i, i + 1);
            normal = {prev_normal.x + next_normal.x, prev_normal.y + next_normal.y};
            const double normal_length = std::hypot(normal.x, normal.y);
            if (normal_length > 1e-9) {
                normal.x /= normal_length;
                normal.y /= normal_length;
            } else {
                normal = next_normal;
            }
        }

        out.push_back({
            base[i].x + normal.x * offset,
            base[i].y + normal.y * offset,
        });
    }

    if (closed_loop && !out.empty()) {
        out.push_back(out.front());
    }
    return out;
}

void draw_structured_road_map(ImDrawList* draw_list, const CanvasTransform& tx, const WorldMap& world) {
    const std::vector<Vec2>& centerline = world.road_centerline();
    if (centerline.size() < 2) {
        return;
    }

    const double road_width = structured_road_width_for_world(world);
    const double half_width = 0.5 * road_width;
    const ImU32 road_bound_color = IM_COL32(255, 96, 96, 230);
    const ImU32 road_center_color = IM_COL32(122, 232, 158, 190);
    const ImU32 lane_color = IM_COL32(245, 213, 105, 150);

    draw_polyline(draw_list, tx, make_offset_polyline(centerline, half_width), road_bound_color, 2.4f);
    draw_polyline(draw_list, tx, make_offset_polyline(centerline, -half_width), road_bound_color, 2.4f);
    if (road_width >= 1.2) {
        draw_polyline(draw_list, tx, make_offset_polyline(centerline, road_width / 6.0), lane_color, 1.4f);
        draw_polyline(draw_list, tx, make_offset_polyline(centerline, -road_width / 6.0), lane_color, 1.4f);
    }
    draw_polyline(draw_list, tx, centerline, road_center_color, 2.0f);
}

float hardware_vehicle_visual_scale_for_world(const WorldMap& world) {
    if (world.environment_mode() == EnvironmentMode::MixedRoadGates) {
        const Rect& bounds = world.bounds();
        const double span = std::max(bounds.max_x - bounds.min_x, bounds.max_y - bounds.min_y);
        if (span <= 1.25) {
            return 0.55f;
        }
        if (span <= 2.50) {
            return 0.70f;
        }
        return 1.10f;
    }

    if (world.environment_mode() != EnvironmentMode::StructuredRoad) {
        return 1.45f;
    }

    const Rect content = structured_content_bounds(world);
    const double span = std::max(content.max_x - content.min_x, content.max_y - content.min_y);
    if (span <= 0.36) {
        return 0.16f;
    }
    if (span <= 0.50) {
        return 0.20f;
    }
    if (span <= 0.80) {
        return 0.28f;
    }
    return 0.42f;
}

ImU32 hardware_vehicle_body_color_for_world(const WorldMap& world) {
    if (world.environment_mode() == EnvironmentMode::MixedRoadGates) {
        const Rect& bounds = world.bounds();
        const double span = std::max(bounds.max_x - bounds.min_x, bounds.max_y - bounds.min_y);
        return span <= 1.25 ? IM_COL32(238, 239, 226, 210) : kColorBody;
    }

    if (world.environment_mode() != EnvironmentMode::StructuredRoad) {
        return kColorBody;
    }

    const Rect& bounds = world.bounds();
    const double span = std::max(bounds.max_x - bounds.min_x, bounds.max_y - bounds.min_y);
    return span <= 0.35 ? IM_COL32(238, 239, 226, 188) : kColorBody;
}

float simulation_vehicle_visual_scale_for_world(const WorldMap& world) {
    if (world.environment_mode() == EnvironmentMode::StructuredRoad ||
        (world.environment_mode() == EnvironmentMode::MixedRoadGates &&
         std::max(world.bounds().max_x - world.bounds().min_x,
                  world.bounds().max_y - world.bounds().min_y) <= 2.50)) {
        return hardware_vehicle_visual_scale_for_world(world);
    }

    const Rect& bounds = world.bounds();
    const double span = std::max(bounds.max_x - bounds.min_x, bounds.max_y - bounds.min_y);
    if (span <= 3.0) {
        return 1.55f;
    }
    if (span <= 8.0) {
        return 1.95f;
    }
    return 2.40f;
}

void draw_goal_marker(ImDrawList* draw_list, const CanvasTransform& tx, const Vec2& goal) {
    const ImVec2 screen_pos = world_to_screen(tx, goal);
    draw_list->AddCircleFilled(screen_pos, 6.8f, kColorGoal);
    draw_list->AddCircle(screen_pos, 10.8f, IM_COL32(245, 246, 240, 190), 0, 1.6f);
}

double mixed_gate_acceptance_radius_for_world(const WorldMap& world) {
    if (world.environment_mode() != EnvironmentMode::MixedRoadGates) {
        return 0.0;
    }
    const Rect& bounds = world.bounds();
    const double span = std::max(bounds.max_x - bounds.min_x, bounds.max_y - bounds.min_y);
    return span <= 1.25 ? 0.10 : 0.12;
}

void draw_mixed_gate_acceptance_ring(ImDrawList* draw_list,
                                     const CanvasTransform& tx,
                                     const WorldMap& world,
                                     const Vec2& gate_position) {
    const double radius_m = mixed_gate_acceptance_radius_for_world(world);
    if (!(radius_m > 0.0)) {
        return;
    }
    draw_list->AddCircle(
        world_to_screen(tx, gate_position),
        static_cast<float>(radius_m * tx.scale),
        IM_COL32(255, 206, 96, 150),
        0,
        1.8f);
}

void draw_vehicle(ImDrawList* draw_list,
                  const CanvasTransform& tx,
                  const VehicleSnapshot& vehicle,
                  const thesis_sim::VehicleGeometry& geometry,
                  float visual_scale = 1.0f,
                  ImU32 body_color = kColorBody) {
    std::array<ImVec2, 4> body{};
    for (size_t i = 0; i < body.size(); ++i) {
        const Vec2 scaled_corner{
            vehicle.position.x + (vehicle.body_corners[i].x - vehicle.position.x) * visual_scale,
            vehicle.position.y + (vehicle.body_corners[i].y - vehicle.position.y) * visual_scale,
        };
        body[i] = world_to_screen(tx, scaled_corner);
    }
    draw_list->AddConvexPolyFilled(body.data(), static_cast<int>(body.size()), body_color);
    draw_list->AddPolyline(body.data(), static_cast<int>(body.size()), IM_COL32(20, 22, 26, 255), ImDrawFlags_Closed, 2.5f);

    const bool render_tracks = vehicle.model_name.find("Tracked") != std::string::npos;
    if (render_tracks) {
        const double half_track = geometry.track * 0.5;
        for (double side : {1.0, -1.0}) {
            const Vec2 local_center{0.0, side * half_track};
            const Vec2 global_offset = thesis_sim::rotate(local_center, vehicle.yaw);
            const Vec2 track_center{
                vehicle.position.x + global_offset.x * visual_scale,
                vehicle.position.y + global_offset.y * visual_scale,
            };
            const auto track_box = thesis_sim::make_box_corners(
                track_center,
                vehicle.yaw,
                geometry.wheel_length * visual_scale,
                geometry.wheel_width * visual_scale);
            std::array<ImVec2, 4> track_points{};
            for (size_t i = 0; i < track_points.size(); ++i) {
                track_points[i] = world_to_screen(tx, track_box[i]);
            }
            draw_list->AddConvexPolyFilled(track_points.data(), static_cast<int>(track_points.size()), kColorWheel);
            draw_list->AddPolyline(track_points.data(), static_cast<int>(track_points.size()), IM_COL32(204, 207, 208, 255), ImDrawFlags_Closed, 1.4f);
        }
    } else {
        for (const WheelPose& wheel : vehicle.wheels) {
            const Vec2 scaled_wheel_center{
                vehicle.position.x + (wheel.center.x - vehicle.position.x) * visual_scale,
                vehicle.position.y + (wheel.center.y - vehicle.position.y) * visual_scale,
            };
            const auto wheel_box = thesis_sim::make_box_corners(
                scaled_wheel_center,
                wheel.yaw,
                geometry.wheel_length * visual_scale,
                geometry.wheel_width * visual_scale);
            std::array<ImVec2, 4> wheel_points{};
            for (size_t i = 0; i < wheel_points.size(); ++i) {
                wheel_points[i] = world_to_screen(tx, wheel_box[i]);
            }
            draw_list->AddConvexPolyFilled(wheel_points.data(), static_cast<int>(wheel_points.size()), kColorWheel);
            draw_list->AddPolyline(wheel_points.data(), static_cast<int>(wheel_points.size()), IM_COL32(200, 203, 207, 255), ImDrawFlags_Closed, 1.2f);
        }
    }

    const Vec2 nose{
        vehicle.position.x + std::cos(vehicle.yaw) * geometry.body_length * 0.70 * visual_scale,
        vehicle.position.y + std::sin(vehicle.yaw) * geometry.body_length * 0.70 * visual_scale,
    };
    draw_list->AddLine(world_to_screen(tx, vehicle.position), world_to_screen(tx, nose), kColorHeading, 3.6f);
}

VehicleSnapshot build_vehicle_snapshot_from_live(const LiveVehicleState& live,
                                                 const thesis_sim::VehicleGeometry& geometry) {
    VehicleSnapshot vehicle;
    vehicle.position = live.position;
    vehicle.yaw = live.yaw;
    vehicle.speed = live.speed;
    vehicle.accel = live.accel;
    vehicle.curvature = live.curvature;
    vehicle.steer_angle = live.steer_angle;
    vehicle.yaw_rate = live.yaw_rate;
    vehicle.sideslip = live.sideslip;
    vehicle.left_wheel_speed = live.left_wheel_speed;
    vehicle.right_wheel_speed = live.right_wheel_speed;
    vehicle.target_speed = live.target_speed;
    vehicle.target_yaw_rate = live.target_yaw_rate;
    vehicle.target_steer_angle = live.target_steer_angle;
    vehicle.left_encoder_ticks = live.left_encoder_ticks;
    vehicle.right_encoder_ticks = live.right_encoder_ticks;
    vehicle.left_encoder_delta = live.left_encoder_delta;
    vehicle.right_encoder_delta = live.right_encoder_delta;
    vehicle.left_pwm = live.left_pwm;
    vehicle.right_pwm = live.right_pwm;
    vehicle.encoder_dt_ms = live.encoder_dt_ms;
    vehicle.model_name = "Hardware estimate";
    vehicle.body_corners = thesis_sim::make_box_corners(
        vehicle.position,
        vehicle.yaw,
        geometry.body_length,
        geometry.body_width);

    const auto wheel_center = [&](double local_x, double local_y) {
        const Vec2 offset = thesis_sim::rotate({local_x, local_y}, vehicle.yaw);
        return Vec2{vehicle.position.x + offset.x, vehicle.position.y + offset.y};
    };
    const double half_track = geometry.track * 0.5;
    const double front_x = geometry.cg_to_front;
    const double rear_x = -geometry.cg_to_rear;
    vehicle.wheels[0] = {wheel_center(front_x, half_track), vehicle.yaw + vehicle.steer_angle, live.left_wheel_speed, true};
    vehicle.wheels[1] = {wheel_center(front_x, -half_track), vehicle.yaw + vehicle.steer_angle, live.right_wheel_speed, true};
    vehicle.wheels[2] = {wheel_center(rear_x, half_track), vehicle.yaw, live.left_wheel_speed, false};
    vehicle.wheels[3] = {wheel_center(rear_x, -half_track), vehicle.yaw, live.right_wheel_speed, false};
    return vehicle;
}

bool workspace_source_is_hardware(int source) {
    return source == kWorkspaceSourceHardwarePlanner;
}

const char* workspace_source_label(int source) {
    switch (source) {
        case kWorkspaceSourceHardwarePlanner:
            return "Hardware Planner";
        case kWorkspaceSourceSimulation:
        default:
            return "Simulation";
    }
}

const char* unstructured_map_cli_name(UnstructuredMapPreset preset) {
    switch (preset) {
        case UnstructuredMapPreset::TightCorridor:
            return "tight";
        case UnstructuredMapPreset::WideSlalom:
            return "slalom";
        case UnstructuredMapPreset::LowerBypass:
            return "lower";
        case UnstructuredMapPreset::HardwareLab:
            return "hardware_lab";
        case UnstructuredMapPreset::Custom:
            return "custom";
        case UnstructuredMapPreset::RobotValidation:
        default:
            return "robot_validation";
    }
}

const char* structured_map_cli_name(StructuredMapPreset preset) {
    switch (preset) {
        case StructuredMapPreset::CircleLoop:
            return "circle";
        case StructuredMapPreset::ZigZag:
            return "zigzag";
        case StructuredMapPreset::HardwareTrack:
            return "hardware_track";
        case StructuredMapPreset::FigureEight:
            return "figure_eight";
        case StructuredMapPreset::TankCircuit:
            return "tank_circuit";
        case StructuredMapPreset::Custom:
            return "custom";
        case StructuredMapPreset::ValidationRoad:
        default:
            return "validation";
    }
}

std::string stream_profile_label(const std::string& stream_profile) {
    if (stream_profile.empty() || stream_profile == "planner") {
        return "Planner";
    }
    return "Unsupported";
}

VehicleModelKind vehicle_model_from_live_name(const std::string& name) {
    if (name.find("Tracked") != std::string::npos ||
        name.find("tracked") != std::string::npos ||
        name.find("Tank") != std::string::npos ||
        name.find("tank") != std::string::npos) {
        return VehicleModelKind::TrackedVehicle;
    }
    return VehicleModelKind::CarLikeBicycle;
}

std::string hardware_goal_distance_label(const LiveFrameSnapshot& frame, EnvironmentMode mode) {
    char goal_buf[32];
    if (mode == EnvironmentMode::UnstructuredGates &&
        (frame.distance_to_goal < 0.0 || frame.chosen_gate_index < 0 || !frame.planner_has_reference)) {
        if (frame.goal_reached) {
            return "done";
        }
        return frame.dynamic_gap_gates ? "n/a" : "idle";
    }

    std::snprintf(goal_buf, sizeof(goal_buf), "%.2f m", frame.distance_to_goal);
    return goal_buf;
}

std::string hardware_launch_hint(const UiState& ui_state) {
    std::ostringstream cmd;
    const EnvironmentMode mode = static_cast<EnvironmentMode>(ui_state.hardware_environment_mode);
    const bool structured = mode == EnvironmentMode::StructuredRoad;
    const bool mixed = mode == EnvironmentMode::MixedRoadGates;
    bool custom_world = false;
    if (structured) {
        custom_world =
            static_cast<StructuredMapPreset>(ui_state.hardware_structured_preset) == StructuredMapPreset::Custom;
    } else if (!mixed) {
        custom_world =
            static_cast<UnstructuredMapPreset>(ui_state.hardware_unstructured_preset) == UnstructuredMapPreset::Custom;
    }
    cmd << "thesis_robot_runner"
        << " --controller-port /dev/ttyACM0";
    if (!structured) {
        cmd << " --lidar-port /dev/ttyUSB0"
            << " --enable-planner-safety";
    }
    cmd
        << " --stream-host <pc-ip>"
        << " --stream-port " << std::max(ui_state.hardware_listen_port, 1)
        << " --scenario " << environment_mode_slug(mode);
    if (static_cast<VehicleModelKind>(ui_state.hardware_vehicle_model) == VehicleModelKind::TrackedVehicle) {
        cmd << " --vehicle-model tank";
    } else {
        cmd << " --vehicle-model car";
    }
    if (custom_world) {
        cmd << " --world-file <copied-custom-map.thmap>";
    } else if (structured) {
        cmd << " --structured-map "
            << structured_map_cli_name(static_cast<StructuredMapPreset>(ui_state.hardware_structured_preset));
    } else if (!mixed) {
        cmd << " --unstructured-map "
            << unstructured_map_cli_name(static_cast<UnstructuredMapPreset>(ui_state.hardware_unstructured_preset));
    }
    return cmd.str();
}

void load_hardware_editor_from_selection(UiState* ui_state) {
    if (ui_state == nullptr) {
        return;
    }
    const EnvironmentMode mode = static_cast<EnvironmentMode>(ui_state->hardware_environment_mode);
    WorldMap selected_world =
        mode == EnvironmentMode::MixedRoadGates
            ? WorldMap::mixed_hardware_demo()
            : make_world_from_mode(
                  mode,
                  static_cast<UnstructuredMapPreset>(ui_state->hardware_unstructured_preset),
                  static_cast<StructuredMapPreset>(ui_state->hardware_structured_preset),
                  GateBehaviorMode::Static,
                  0);
    WorldMap world = apply_hardware_track_scale(*ui_state, std::move(selected_world));
    ui_state->hardware_editor_world = sanitize_hardware_unstructured_world(std::move(world));
    ui_state->hardware_editor_dirty = false;
    reset_editor_interaction(ui_state);
}

void clear_hardware_world_sync(UiState* ui_state,
                               LiveViewStreamServer* hardware_server,
                               const char* status_message = nullptr) {
    if (ui_state == nullptr) {
        return;
    }
    if (hardware_server != nullptr) {
        hardware_server->clear_pending_world();
        hardware_server->clear_pending_robot_profile();
    }
    ui_state->hardware_world_sync_pending = false;
    ui_state->hardware_world_sync_ok = false;
    ui_state->last_hardware_world_sync_status =
        status_message != nullptr ? status_message : "";
}

bool queue_current_hardware_world(UiState* ui_state,
                                  LiveViewStreamServer* hardware_server,
                                  const char* queued_message,
                                  const char* failure_message) {
    if (ui_state == nullptr || hardware_server == nullptr) {
        return false;
    }
    WorldMap streamed_world = hardware_world_from_ui_selection(*ui_state);
    // `hardware_world_from_ui_selection()` already finalizes true custom maps.
    // Avoid finalizing again here, otherwise built-in structured presets such as
    // Hardware Track get converted into StructuredMapPreset::Custom before they
    // are streamed to the Raspberry runner and later saved in reports.
    std::string validation_error;
    if (!validate_hardware_world(streamed_world, &validation_error)) {
        hardware_server->clear_pending_world();
        ui_state->hardware_world_sync_pending = false;
        ui_state->hardware_world_sync_ok = false;
        ui_state->last_hardware_world_error = validation_error;
        ui_state->last_hardware_world_sync_status = validation_error;
        return false;
    }
    if (hardware_server->queue_world(streamed_world)) {
        ui_state->hardware_editor_world = streamed_world;
        ui_state->hardware_editor_dirty = false;
        ui_state->hardware_world_sync_pending = true;
        ui_state->hardware_world_sync_ok = false;
        ui_state->last_hardware_world_sync_status =
            queued_message != nullptr ? queued_message : "Hardware map queued for Raspberry sync.";
        return true;
    }

    ui_state->hardware_world_sync_pending = false;
    ui_state->hardware_world_sync_ok = false;
    ui_state->last_hardware_world_sync_status =
        hardware_server->last_error().empty()
            ? (failure_message != nullptr ? failure_message : "Could not queue the hardware map for Raspberry sync.")
            : hardware_server->last_error();
    return false;
}

bool queue_hardware_robot_profile(UiState* ui_state,
                                  LiveViewStreamServer* hardware_server,
                                  VehicleModelKind vehicle_model,
                                  const char* queued_message,
                                  const char* failure_message) {
    if (ui_state == nullptr || hardware_server == nullptr) {
        return false;
    }
    ui_state->hardware_vehicle_model = static_cast<int>(vehicle_model);
    if (hardware_server->queue_robot_profile(vehicle_model)) {
        ui_state->hardware_world_sync_pending = true;
        ui_state->hardware_world_sync_ok = false;
        ui_state->last_hardware_world_sync_status =
            queued_message != nullptr ? queued_message : "Robot profile queued for Raspberry sync.";
        return true;
    }

    ui_state->hardware_world_sync_pending = false;
    ui_state->hardware_world_sync_ok = false;
    ui_state->last_hardware_world_sync_status =
        hardware_server->last_error().empty()
            ? (failure_message != nullptr ? failure_message : "Could not queue the robot profile for Raspberry sync.")
            : hardware_server->last_error();
    return false;
}

MapEditorHandle pick_editor_handle(const WorldMap& world, const CanvasTransform& tx, const ImVec2& mouse_pos) {
    constexpr float kHandleRadius = 12.0f;
    const float radius_sq = kHandleRadius * kHandleRadius;

    MapEditorHandle best;
    float best_dist_sq = radius_sq;

    const auto test_handle = [&](MapEditorHandleType type, int index, const Vec2& world_pos) {
        const float d_sq = distance_sq(world_to_screen(tx, world_pos), mouse_pos);
        if (d_sq <= best_dist_sq) {
            best = {type, index};
            best_dist_sq = d_sq;
        }
    };

    test_handle(MapEditorHandleType::Start, -1, world.start());
    test_handle(MapEditorHandleType::Goal, -1, world.goal());

    for (size_t i = 0; i < world.obstacles().size(); ++i) {
        const Rect& obstacle = world.obstacles()[i];
        const Vec2 center{
            0.5 * (obstacle.min_x + obstacle.max_x),
            0.5 * (obstacle.min_y + obstacle.max_y),
        };
        test_handle(MapEditorHandleType::Obstacle, static_cast<int>(i), center);
    }

    if (world.environment_mode() == EnvironmentMode::UnstructuredGates) {
        for (size_t i = 0; i < world.gates().size(); ++i) {
            const GateSpec& gate = world.gates()[i];
            if (gate.final) {
                continue;
            }
            test_handle(MapEditorHandleType::Gate, static_cast<int>(i), gate.anchor_position);
        }
    } else {
        for (size_t i = 1; i + 1 < world.road_centerline().size(); ++i) {
            test_handle(MapEditorHandleType::RoadPoint, static_cast<int>(i), world.road_centerline()[i]);
        }
    }

    return best;
}

void draw_editor_overlay(ImDrawList* draw_list, const CanvasTransform& tx, const WorldMap& world, const UiState& ui_state) {
    if (!ui_state.map_editor_enabled) {
        return;
    }

    for (const Rect& obstacle : world.obstacles()) {
        draw_list->AddRect(
            world_to_screen(tx, {obstacle.min_x, obstacle.max_y}),
            world_to_screen(tx, {obstacle.max_x, obstacle.min_y}),
            kColorEditorOverlay,
            4.0f,
            0,
            2.0f);

        const Vec2 center{
            0.5 * (obstacle.min_x + obstacle.max_x),
            0.5 * (obstacle.min_y + obstacle.max_y),
        };
        const ImVec2 screen_center = world_to_screen(tx, center);
        draw_list->AddCircleFilled(screen_center, 6.0f, kColorEditorHandle);
    }

    draw_list->AddCircleFilled(
        world_to_screen(tx, world.start()),
        7.0f,
        ui_state.selected_editor_handle.type == MapEditorHandleType::Start ? kColorEditorSelected : IM_COL32(120, 220, 150, 255));
    draw_list->AddCircleFilled(
        world_to_screen(tx, world.goal()),
        7.0f,
        ui_state.selected_editor_handle.type == MapEditorHandleType::Goal ? kColorEditorSelected : kColorGoal);

    if (world.environment_mode() == EnvironmentMode::UnstructuredGates) {
        for (size_t i = 0; i < world.gates().size(); ++i) {
            const GateSpec& gate = world.gates()[i];
            if (gate.final) {
                continue;
            }
            const bool selected =
                ui_state.selected_editor_handle.type == MapEditorHandleType::Gate &&
                ui_state.selected_editor_handle.index == static_cast<int>(i);
            draw_list->AddCircleFilled(
                world_to_screen(tx, gate.anchor_position),
                6.0f,
                selected ? kColorEditorSelected : kColorEditorHandle);
        }
    } else {
        draw_polyline(draw_list, tx, world.road_centerline(), kColorEditorOverlay, 2.0f);
        for (size_t i = 1; i + 1 < world.road_centerline().size(); ++i) {
            const bool selected =
                ui_state.selected_editor_handle.type == MapEditorHandleType::RoadPoint &&
                ui_state.selected_editor_handle.index == static_cast<int>(i);
            draw_list->AddCircleFilled(
                world_to_screen(tx, world.road_centerline()[i]),
                5.0f,
                selected ? kColorEditorSelected : kColorEditorHandle);
        }
    }
}

void update_editor_drag(UiState* ui_state, const CanvasTransform& tx, const ImVec2& mouse_pos) {
    if (ui_state == nullptr || !ui_state->map_editor_enabled) {
        return;
    }

    WorldMap* world_ptr = active_editor_world(ui_state);
    bool* dirty_flag = active_editor_dirty_flag(ui_state);
    if (world_ptr == nullptr || dirty_flag == nullptr) {
        return;
    }
    WorldMap& world = *world_ptr;
    const bool mouse_down = ImGui::IsMouseDown(ImGuiMouseButton_Left);

    if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
        ui_state->selected_editor_handle = pick_editor_handle(world, tx, mouse_pos);
        ui_state->active_drag_handle = ui_state->selected_editor_handle;

        const Vec2 world_mouse = screen_to_world(tx, mouse_pos);
        switch (ui_state->active_drag_handle.type) {
            case MapEditorHandleType::Start:
                ui_state->drag_offset = {world.start().x - world_mouse.x, world.start().y - world_mouse.y};
                break;
            case MapEditorHandleType::Goal:
                ui_state->drag_offset = {world.goal().x - world_mouse.x, world.goal().y - world_mouse.y};
                break;
            case MapEditorHandleType::Obstacle:
                if (ui_state->active_drag_handle.index >= 0 &&
                    ui_state->active_drag_handle.index < static_cast<int>(world.editable_obstacles().size())) {
                    const Rect& obstacle = world.editable_obstacles()[ui_state->active_drag_handle.index];
                    const Vec2 center{
                        0.5 * (obstacle.min_x + obstacle.max_x),
                        0.5 * (obstacle.min_y + obstacle.max_y),
                    };
                    ui_state->drag_offset = {center.x - world_mouse.x, center.y - world_mouse.y};
                }
                break;
            case MapEditorHandleType::Gate:
                if (ui_state->active_drag_handle.index >= 0 &&
                    ui_state->active_drag_handle.index < static_cast<int>(world.editable_gates().size())) {
                    const GateSpec& gate = world.editable_gates()[ui_state->active_drag_handle.index];
                    ui_state->drag_offset = {gate.anchor_position.x - world_mouse.x, gate.anchor_position.y - world_mouse.y};
                }
                break;
            case MapEditorHandleType::RoadPoint:
                if (ui_state->active_drag_handle.index >= 0 &&
                    ui_state->active_drag_handle.index < static_cast<int>(world.editable_road_centerline().size())) {
                    const Vec2& point = world.editable_road_centerline()[ui_state->active_drag_handle.index];
                    ui_state->drag_offset = {point.x - world_mouse.x, point.y - world_mouse.y};
                }
                break;
            default:
                break;
        }
    }

    if (!mouse_down) {
        ui_state->active_drag_handle = {};
        return;
    }

    if (ui_state->active_drag_handle.type == MapEditorHandleType::None) {
        return;
    }

    const Vec2 world_mouse = screen_to_world(tx, mouse_pos);
    const Vec2 new_pos{
        world_mouse.x + ui_state->drag_offset.x,
        world_mouse.y + ui_state->drag_offset.y,
    };

    switch (ui_state->active_drag_handle.type) {
        case MapEditorHandleType::Start:
            world.set_start(new_pos);
            *dirty_flag = true;
            break;
        case MapEditorHandleType::Goal:
            world.set_goal(new_pos);
            *dirty_flag = true;
            break;
        case MapEditorHandleType::Obstacle:
            if (ui_state->active_drag_handle.index >= 0 &&
                ui_state->active_drag_handle.index < static_cast<int>(world.editable_obstacles().size())) {
                Rect& obstacle = world.editable_obstacles()[ui_state->active_drag_handle.index];
                const double half_w = 0.5 * (obstacle.max_x - obstacle.min_x);
                const double half_h = 0.5 * (obstacle.max_y - obstacle.min_y);
                obstacle.min_x = new_pos.x - half_w;
                obstacle.max_x = new_pos.x + half_w;
                obstacle.min_y = new_pos.y - half_h;
                obstacle.max_y = new_pos.y + half_h;
                *dirty_flag = true;
            }
            break;
        case MapEditorHandleType::Gate:
            if (ui_state->active_drag_handle.index >= 0 &&
                ui_state->active_drag_handle.index < static_cast<int>(world.editable_gates().size())) {
                GateSpec& gate = world.editable_gates()[ui_state->active_drag_handle.index];
                gate.anchor_position = new_pos;
                gate.position = new_pos;
                *dirty_flag = true;
            }
            break;
        case MapEditorHandleType::RoadPoint:
            if (ui_state->active_drag_handle.index >= 0 &&
                ui_state->active_drag_handle.index < static_cast<int>(world.editable_road_centerline().size())) {
                world.editable_road_centerline()[ui_state->active_drag_handle.index] = new_pos;
                *dirty_flag = true;
            }
            break;
        default:
            break;
    }
}

void render_world_tab(PlannerDrivenVehicleSim& sim, UiState* ui_state) {
    if (ui_state == nullptr) {
        return;
    }

    if (!ImGui::BeginChild("WorldTabRoot", ImVec2(0.0f, 0.0f), false)) {
        ImGui::EndChild();
        return;
    }

    const std::string active_target =
        (sim.environment_mode() == EnvironmentMode::UnstructuredGates ||
         (sim.environment_mode() == EnvironmentMode::MixedRoadGates && sim.chosen_gate_index() >= 0))
            ? simulation_gate_label(sim, sim.chosen_gate_index())
            : (sim.environment_mode() == EnvironmentMode::MixedRoadGates ? "road / dynamic gate arbiter"
                                                                          : thesis_sim::structured_map_preset_name(sim.world().structured_preset()));

    char speed_buf[32];
    char goal_buf[32];
    char tracking_buf[32];
    char target_buf[48];
    std::snprintf(speed_buf, sizeof(speed_buf), "%.2f m/s", sim.vehicle().speed);
    std::snprintf(goal_buf, sizeof(goal_buf), "%.2f m", sim.distance_to_goal());
    std::snprintf(tracking_buf, sizeof(tracking_buf), "%.2f m / %.1f deg", sim.tracker_cross_track_error(), sim.tracker_heading_error_deg());
    std::snprintf(target_buf, sizeof(target_buf), "%s", active_target.c_str());

    if (ImGui::BeginChild("WorldToolbar", ImVec2(0.0f, 102.0f), true)) {
        if (ImGui::BeginTable("WorldToolbarLayout", 2, ImGuiTableFlags_SizingStretchProp)) {
            ImGui::TableSetupColumn("WorldMetrics", ImGuiTableColumnFlags_WidthStretch, 1.0f);
            ImGui::TableSetupColumn("WorldControls", ImGuiTableColumnFlags_WidthFixed, 320.0f);

            ImGui::TableNextColumn();
            if (ImGui::BeginTable("WorldMetricCards", 4, ImGuiTableFlags_SizingStretchSame)) {
                ImGui::TableNextColumn();
                metric_card("world_speed", "Vehicle", speed_buf, "Live speed", ImVec4(0.43f, 0.82f, 0.96f, 1.0f), 64.0f);
                ImGui::TableNextColumn();
                metric_card("world_goal", "Goal", goal_buf, "Distance remaining", ImVec4(0.99f, 0.70f, 0.32f, 1.0f), 64.0f);
                ImGui::TableNextColumn();
                metric_card("world_tracking", "Tracking", tracking_buf, "cte / heading", ImVec4(0.97f, 0.89f, 0.45f, 1.0f), 64.0f);
                ImGui::TableNextColumn();
                metric_card("world_target", "Target", target_buf,
                            sim.environment_mode() == EnvironmentMode::UnstructuredGates
                                ? "Active planner gate"
                                : (sim.environment_mode() == EnvironmentMode::MixedRoadGates ? "Mixed arbitration" : "Active structured loop"),
                            ImVec4(0.48f, 0.88f, 0.62f, 1.0f),
                            64.0f);
                ImGui::EndTable();
            }

            ImGui::TableNextColumn();
            ImGui::TextDisabled("VIEWPORT LAYERS");
            ImGui::Checkbox("Grid", &ui_state->show_grid);
            ImGui::SameLine();
            ImGui::Checkbox("Trails", &ui_state->show_trails);
            ImGui::Checkbox("LiDAR Rays", &ui_state->show_lidar_rays);
            ImGui::SameLine();
            ImGui::Checkbox("LiDAR Hits", &ui_state->show_lidar_hits);
            ImGui::Checkbox("Labels", &ui_state->show_gate_labels);
            ImGui::Checkbox("HUD", &ui_state->show_world_hud);
            ImGui::SameLine();
            ImGui::TextDisabled("%s", ui_state->map_editor_enabled ? "Editor enabled" : "Editor off");
            ImGui::TextWrapped("Use the viewport for live diagnosis first, then enable drag editing only when you want to reshape the map.");
            ImGui::EndTable();
        }
    }
    ImGui::EndChild();

    if (!ImGui::BeginChild("SimulationViewport", ImVec2(0.0f, 0.0f), true, ImGuiWindowFlags_NoScrollbar)) {
        ImGui::EndChild();
        ImGui::EndChild();
        return;
    }

    const ImVec2 canvas_pos = ImGui::GetCursorScreenPos();
    ImVec2 canvas_size = ImGui::GetContentRegionAvail();
    canvas_size.x = std::max(canvas_size.x, 320.0f);
    canvas_size.y = std::max(canvas_size.y, 320.0f);

    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    draw_list->AddRectFilledMultiColor(
        canvas_pos,
        ImVec2(canvas_pos.x + canvas_size.x, canvas_pos.y + canvas_size.y),
        IM_COL32(16, 22, 28, 255),
        IM_COL32(23, 32, 39, 255),
        IM_COL32(12, 18, 22, 255),
        IM_COL32(14, 19, 24, 255));
    draw_list->AddRect(canvas_pos, ImVec2(canvas_pos.x + canvas_size.x, canvas_pos.y + canvas_size.y), IM_COL32(71, 81, 88, 255), 14.0f, 0, 1.5f);

    const CanvasTransform tx = make_transform(sim.world(), canvas_pos, canvas_size);
    const Rect bounds = sim.world().bounds();
    if (ui_state->show_grid) {
        draw_world_grid(draw_list, tx, bounds);
    }

    for (const Rect& obstacle : sim.world().obstacles()) {
        draw_list->AddRectFilled(world_to_screen(tx, {obstacle.min_x, obstacle.max_y}),
                                 world_to_screen(tx, {obstacle.max_x, obstacle.min_y}),
                                 kColorObstacle, 4.0f);
    }

    if (!sim.world().road_centerline().empty()) {
        if (sim.world().environment_mode() == EnvironmentMode::StructuredRoad ||
            sim.world().environment_mode() == EnvironmentMode::MixedRoadGates) {
            draw_structured_road_map(draw_list, tx, sim.world());
        } else {
            draw_polyline(draw_list, tx, sim.world().road_centerline(), IM_COL32(113, 210, 255, 180), 4.0f);
        }
        if (sim.world().environment_mode() == EnvironmentMode::StructuredRoad ||
            sim.world().environment_mode() == EnvironmentMode::MixedRoadGates) {
            draw_goal_marker(draw_list, tx, sim.world().goal());
        }
    }

    if (ui_state->show_trails) {
        draw_polyline(draw_list, tx, sim.trail(), kColorTrail, 2.5f);
        draw_polyline(draw_list, tx, sim.estimated_trail(), kColorEstimateTrail, 1.8f);
        draw_polyline(draw_list, tx, sim.planned_trajectory(), kColorTrajectory, 3.0f);
    }

    if (sim.environment_mode() == EnvironmentMode::MixedRoadGates &&
        sim.config().dynamic_lidar_gates) {
        for (const GateSpec& gate : sim.world().gates()) {
            if (gate.final) {
                continue;
            }
            const ImVec2 screen_pos = world_to_screen(tx, gate.position);
            draw_list->AddCircle(screen_pos, 7.0f, IM_COL32(255, 184, 83, 170), 0, 2.0f);
            draw_list->AddLine(
                ImVec2(screen_pos.x - 6.0f, screen_pos.y),
                ImVec2(screen_pos.x + 6.0f, screen_pos.y),
                IM_COL32(255, 184, 83, 130),
                1.6f);
            if (ui_state->show_gate_labels) {
                draw_list->AddText(
                    ImVec2(screen_pos.x + 7.0f, screen_pos.y + 2.0f),
                    IM_COL32(255, 214, 152, 190),
                    gate.name.c_str());
            }
        }
    }

    if (sim.lidar_enabled() && (ui_state->show_lidar_rays || ui_state->show_lidar_hits)) {
        const Vec2 car_pos = sim.vehicle().position;
        const size_t lidar_stride = sim.lidar_hits().size() > 720 ? 2 : 1;
        if (ui_state->show_lidar_rays) {
            for (size_t i = 0; i < sim.lidar_hits().size(); i += lidar_stride) {
                const LidarHit& hit = sim.lidar_hits()[i];
                draw_list->AddLine(
                    world_to_screen(tx, car_pos),
                    world_to_screen(tx, hit.point),
                    hit.hit ? kColorLidar : kColorLidarMiss,
                    hit.hit ? 2.3f : 1.5f);
            }
        }

        if (ui_state->show_lidar_hits) {
            for (size_t i = 0; i < sim.lidar_hits().size(); i += lidar_stride) {
                const LidarHit& hit = sim.lidar_hits()[i];
                const ImVec2 hit_screen = world_to_screen(tx, hit.point);
                if (hit.hit) {
                    draw_list->AddCircleFilled(hit_screen, 3.6f, kColorLidarHit);
                    draw_list->AddCircle(hit_screen, 6.2f, IM_COL32(231, 255, 212, 220), 0, 1.4f);
                } else {
                    draw_list->AddCircleFilled(hit_screen, 2.2f, IM_COL32(150, 207, 255, 165));
                }
            }
        }
        draw_list->AddCircleFilled(world_to_screen(tx, car_pos), 4.5f, IM_COL32(170, 255, 208, 220));
    }

    for (size_t i = 0; i < sim.gates().size(); ++i) {
        const Vec2 gate_pos = simulation_gate_position(sim, i);
        const bool final_gate = simulation_gate_is_final(sim, i);
        const std::string gate_label = simulation_gate_label(sim, static_cast<int>(i));
        const auto screen_pos = world_to_screen(tx, gate_pos);
        const bool visible =
            std::find(sim.visible_gate_indices().begin(), sim.visible_gate_indices().end(), static_cast<int>(i)) !=
            sim.visible_gate_indices().end();

        ImU32 color = final_gate ? kColorGoal : kColorGate;
        if (sim.gates()[i].passed) {
            color = kColorGatePassed;
        } else if (visible) {
            color = kColorGateVisible;
        }
        if (static_cast<int>(i) == sim.chosen_gate_index()) {
            color = IM_COL32(255, 233, 118, 255);
        }

        if (sim.world().environment_mode() == EnvironmentMode::MixedRoadGates && !final_gate) {
            draw_mixed_gate_acceptance_ring(draw_list, tx, sim.world(), gate_pos);
        }
        const float radius = final_gate ? 7.0f : 5.0f;
        draw_list->AddCircleFilled(screen_pos, radius, color);
        draw_list->AddCircle(screen_pos, radius + 3.0f, IM_COL32(245, 246, 240, 180), 0, 1.5f);
        if (ui_state->show_gate_labels) {
            draw_list->AddText(ImVec2(screen_pos.x + 6.0f, screen_pos.y - 12.0f), IM_COL32(222, 227, 230, 255), gate_label.c_str());
        }
    }

    const float vehicle_visual_scale = simulation_vehicle_visual_scale_for_world(sim.world());
    draw_vehicle(draw_list,
                 tx,
                 sim.vehicle(),
                 sim.geometry(),
                 vehicle_visual_scale,
                 hardware_vehicle_body_color_for_world(sim.world()));
    const Vec2 nav_pos = sim.navigation_position();
    const Vec2 nav_nose{
        nav_pos.x + std::cos(sim.navigation_yaw()) * sim.geometry().body_length * 0.85 * vehicle_visual_scale,
        nav_pos.y + std::sin(sim.navigation_yaw()) * sim.geometry().body_length * 0.85 * vehicle_visual_scale,
    };
    draw_list->AddCircle(world_to_screen(tx, nav_pos), 11.0f, kColorEstimateTrail, 0, 2.8f);
    draw_list->AddLine(world_to_screen(tx, nav_pos), world_to_screen(tx, nav_nose), kColorEstimateTrail, 3.2f);

    if (ui_state->show_world_hud) {
        char hud_line[96];
        std::vector<OverlayLine> top_left = {
            {"Simulation viewport", IM_COL32(240, 243, 235, 255)},
            {thesis_sim::environment_mode_name(sim.environment_mode()), IM_COL32(170, 179, 185, 255)},
            {map_preset_name(sim.world()),
             IM_COL32(170, 179, 185, 255)},
            {thesis_sim::range_sensor_profile_name(sim.range_sensor_profile()), IM_COL32(170, 179, 185, 255)},
        };
        draw_overlay_panel(draw_list, ImVec2(canvas_pos.x + 16.0f, canvas_pos.y + 16.0f), 228.0f, top_left);

        std::snprintf(hud_line, sizeof(hud_line), "speed %.2f m/s", sim.vehicle().speed);
        std::vector<OverlayLine> top_right = {
            {sim.goal_reached() ? "Mission complete" : (sim.collision() ? "Collision detected" : "Mission active"),
             sim.goal_reached() ? IM_COL32(124, 238, 151, 255)
                                : (sim.collision() ? IM_COL32(255, 124, 102, 255) : IM_COL32(255, 221, 113, 255))},
            {hud_line, IM_COL32(240, 243, 235, 255)},
        };
        std::snprintf(hud_line, sizeof(hud_line), "goal %.2f m", sim.distance_to_goal());
        top_right.push_back({hud_line, IM_COL32(170, 179, 185, 255)});
        if (sim.environment_mode() == EnvironmentMode::UnstructuredGates ||
            sim.environment_mode() == EnvironmentMode::MixedRoadGates) {
            std::snprintf(hud_line, sizeof(hud_line), "gate %s | visible %d", active_target.c_str(), static_cast<int>(sim.visible_gate_indices().size()));
        } else {
            std::snprintf(hud_line, sizeof(hud_line), "road pts %d", static_cast<int>(sim.world().road_centerline().size()));
        }
        top_right.push_back({hud_line, IM_COL32(170, 179, 185, 255)});
        draw_overlay_panel(draw_list, ImVec2(canvas_pos.x + canvas_size.x - 236.0f, canvas_pos.y + 16.0f), 220.0f, top_right);

        std::vector<OverlayLine> legend = {
            {"blue: ground truth trail", kColorTrail},
            {"orange: fused state / estimated trail", kColorEstimateTrail},
            {"yellow: selected planner trajectory", kColorTrajectory},
        };
        if (sim.world().environment_mode() == EnvironmentMode::StructuredRoad ||
            sim.world().environment_mode() == EnvironmentMode::MixedRoadGates) {
            legend.push_back({"red: structured road bounds", IM_COL32(255, 96, 96, 230)});
        }
        if (sim.lidar_enabled() && ui_state->show_lidar_rays) {
            legend.push_back({"green: LiDAR hit rays", kColorLidar});
        }
        if (sim.lidar_enabled() && ui_state->show_lidar_hits) {
            legend.push_back({"lime/cyan: LiDAR collision points", kColorLidarHit});
        }
        const float legend_height = 20.0f + static_cast<float>(legend.size()) * (ImGui::GetFontSize() + 5.0f);
        draw_overlay_panel(draw_list,
                           ImVec2(canvas_pos.x + 16.0f, canvas_pos.y + canvas_size.y - legend_height),
                           286.0f,
                           legend);
    }

    if (ui_state != nullptr && ui_state->map_editor_enabled) {
        if (ui_state->show_world_hud) {
            draw_overlay_panel(draw_list,
                               ImVec2(canvas_pos.x + 16.0f, canvas_pos.y + 130.0f),
                               286.0f,
                               {{"Map editor active", kColorEditorOverlay},
                                {"Drag handles in the viewport, then apply changes.", IM_COL32(170, 179, 185, 255)}});
        }
        draw_editor_overlay(draw_list, tx, ui_state->scenario_editor_world, *ui_state);

        const ImVec2 mouse_pos = ImGui::GetIO().MousePos;
        const bool inside_canvas =
            mouse_pos.x >= canvas_pos.x && mouse_pos.x <= canvas_pos.x + canvas_size.x &&
            mouse_pos.y >= canvas_pos.y && mouse_pos.y <= canvas_pos.y + canvas_size.y;
        if (inside_canvas && ImGui::IsWindowHovered(ImGuiHoveredFlags_AllowWhenBlockedByActiveItem)) {
            update_editor_drag(ui_state, tx, mouse_pos);
        } else if (ui_state->active_drag_handle.type != MapEditorHandleType::None &&
                   !ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
            ui_state->active_drag_handle = {};
        }
    }

    ImGui::Dummy(canvas_size);
    ImGui::EndChild();
    ImGui::EndChild();
}

void render_plot_window(const char* title,
                        const char* y_axis_label,
                        const char* description,
                        const std::vector<double>& x,
                        const std::vector<double>& y0,
                        const char* label0,
                        const std::vector<double>* y1 = nullptr,
                        const char* label1 = nullptr,
                        const std::vector<double>* y2 = nullptr,
                        const char* label2 = nullptr) {
    ImGui::PushID(title);
    ImGui::AlignTextToFramePadding();
    ImGui::TextUnformatted(title);
    if (description != nullptr) {
        ImGui::SameLine();
        help_marker(description);
    }
    ImGui::SameLine();
    ImGui::TextDisabled("latest %.2fs", x.empty() ? 0.0 : x.back());

    if (!ImPlot::BeginPlot("##plot", ImVec2(-1, 220.0f), ImPlotFlags_NoMenus | ImPlotFlags_NoBoxSelect)) {
        ImGui::PopID();
        return;
    }

    const double x_min = x.empty() ? 0.0 : x.front();
    const double x_max = x.size() > 1 ? x.back() : (x_min + 1.0);
    ImPlot::SetupAxes("t [s]", y_axis_label, ImPlotAxisFlags_None, ImPlotAxisFlags_AutoFit);
    ImPlot::SetupLegend(ImPlotLocation_NorthEast, ImPlotLegendFlags_None);
    ImPlot::SetupAxisLimits(ImAxis_X1, x_min, x_max, ImGuiCond_Always);

    ImPlot::SetNextLineStyle(ImVec4(0.36f, 0.73f, 0.98f, 1.0f), 2.0f);
    ImPlot::PlotLine(label0, x.data(), y0.data(), static_cast<int>(x.size()));
    if (y1 != nullptr && label1 != nullptr) {
        ImPlot::SetNextLineStyle(ImVec4(0.96f, 0.66f, 0.28f, 1.0f), 2.0f);
        ImPlot::PlotLine(label1, x.data(), y1->data(), static_cast<int>(x.size()));
    }
    if (y2 != nullptr && label2 != nullptr) {
        ImPlot::SetNextLineStyle(ImVec4(0.48f, 0.87f, 0.60f, 1.0f), 2.0f);
        ImPlot::PlotLine(label2, x.data(), y2->data(), static_cast<int>(x.size()));
    }

    render_plot_hover_overlay(
        title,
        x,
        {
            {&y0, label0, ImVec4(0.36f, 0.73f, 0.98f, 1.0f)},
            {y1, label1, ImVec4(0.96f, 0.66f, 0.28f, 1.0f)},
            {y2, label2, ImVec4(0.48f, 0.87f, 0.60f, 1.0f)},
        });
    ImPlot::EndPlot();
    ImGui::PopID();
}

void plot_series(const std::vector<double>& time,
                 const std::vector<double>& series,
                 const char* label,
                 const ImVec4& color) {
    if (time.empty() || series.empty()) {
        return;
    }
    ImPlot::SetNextLineStyle(color, 2.0f);
    ImPlot::PlotLine(label, time.data(), series.data(), static_cast<int>(time.size()));
}

void render_plot_hover_overlay(const char* title,
                               const std::vector<double>& x,
                               std::initializer_list<PlotHoverSeries> series_list) {
    if (x.empty() || !ImPlot::IsPlotHovered()) {
        return;
    }

    const ImPlotPoint mouse = ImPlot::GetPlotMousePos();
    const auto it = std::lower_bound(x.begin(), x.end(), mouse.x);
    const size_t idx = static_cast<size_t>(std::clamp<long>(
        static_cast<long>(it == x.end() ? static_cast<long>(x.size()) - 1 : std::distance(x.begin(), it)),
        0L,
        static_cast<long>(x.size()) - 1));

    const double hovered_x = x[idx];
    const ImPlotRect limits = ImPlot::GetPlotLimits();
    ImDrawList* draw_list = ImPlot::GetPlotDrawList();
    const ImVec2 line_top = ImPlot::PlotToPixels(hovered_x, limits.Y.Max);
    const ImVec2 line_bottom = ImPlot::PlotToPixels(hovered_x, limits.Y.Min);
    draw_list->AddLine(line_top, line_bottom, IM_COL32(255, 255, 255, 120), 1.5f);
    ImPlot::TagX(hovered_x, ImVec4(0.95f, 0.96f, 0.98f, 1.0f), "t %.2f s", hovered_x);

    ImGui::BeginTooltip();
    ImGui::Text("%s", title);
    ImGui::Separator();
    ImGui::Text("t = %.2f s", hovered_x);
    for (const PlotHoverSeries& item : series_list) {
        if (item.values == nullptr || item.label == nullptr || idx >= item.values->size()) {
            continue;
        }
        const double y = (*item.values)[idx];
        const ImVec2 point_px = ImPlot::PlotToPixels(hovered_x, y);
        draw_list->AddCircleFilled(point_px, 4.0f, ImGui::ColorConvertFloat4ToU32(item.color));
        ImGui::PushStyleColor(ImGuiCol_Text, item.color);
        ImGui::Text("%s = %.3f", item.label, y);
        ImGui::PopStyleColor();
    }
    ImGui::EndTooltip();
}

void setup_time_plot_axes(const std::vector<double>& time, const char* y_axis_label) {
    const double x_min = time.empty() ? 0.0 : time.front();
    const double x_max = time.size() > 1 ? time.back() : (x_min + 1.0);
    ImPlot::SetupAxes("t [s]", y_axis_label, ImPlotAxisFlags_None, ImPlotAxisFlags_AutoFit);
    ImPlot::SetupLegend(ImPlotLocation_NorthEast, ImPlotLegendFlags_None);
    ImPlot::SetupAxisLimits(ImAxis_X1, x_min, x_max, ImGuiCond_Always);
}

void render_timeline_events(const std::vector<TimelineEvent>& events, const char* child_id) {
    if (events.empty()) {
        ImGui::TextDisabled("No mission events yet.");
        return;
    }

    const float line_height = ImGui::GetTextLineHeightWithSpacing();
    const float height = std::max(180.0f, ImGui::GetContentRegionAvail().y);
    if (!ImGui::BeginChild(child_id, ImVec2(0.0f, height), true)) {
        ImGui::EndChild();
        return;
    }

    const size_t max_events = 48;
    const size_t first = events.size() > max_events ? events.size() - max_events : 0;
    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    for (size_t i = first; i < events.size(); ++i) {
        const TimelineEvent& event = events[i];
        ImGui::PushID(static_cast<int>(i));
        const ImVec2 cursor = ImGui::GetCursorScreenPos();
        const ImVec2 marker(cursor.x + 7.0f, cursor.y + line_height * 0.58f);
        if (i > first) {
            draw_list->AddLine(
                ImVec2(marker.x, cursor.y - 4.0f),
                ImVec2(marker.x, marker.y - 7.0f),
                IM_COL32(81, 91, 99, 170),
                1.0f);
        }
        if (i + 1 < events.size()) {
            draw_list->AddLine(
                ImVec2(marker.x, marker.y + 7.0f),
                ImVec2(marker.x, cursor.y + line_height * 2.0f),
                IM_COL32(81, 91, 99, 170),
                1.0f);
        }
        draw_list->AddCircleFilled(marker, 5.0f, ImGui::ColorConvertFloat4ToU32(event.color));
        ImGui::Dummy(ImVec2(18.0f, 0.0f));
        ImGui::SameLine();
        ImGui::BeginGroup();
        ImGui::PushStyleColor(ImGuiCol_Text, event.color);
        ImGui::Text("%.2f s  %s", event.time, event.label.c_str());
        ImGui::PopStyleColor();
        if (!event.detail.empty()) {
            ImGui::TextWrapped("%s", event.detail.c_str());
        }
        ImGui::EndGroup();
        ImGui::Spacing();
        ImGui::PopID();
    }
    ImGui::EndChild();
}

void render_sim_telemetry_side_panel(const PlannerDrivenVehicleSim& sim) {
    if (!ImGui::BeginChild("SimulationTelemetrySidePanel", ImVec2(0.0f, 0.0f), true)) {
        ImGui::EndChild();
        return;
    }

    ImGui::TextUnformatted("Telemetry");
    const std::vector<TelemetrySample>& history = sim.history();
    if (history.empty()) {
        ImGui::TextDisabled("No samples yet.");
        ImGui::EndChild();
        return;
    }

    const TelemetrySample& latest = history.back();
    char speed_buf[32];
    char goal_buf[32];
    char lidar_buf[32];
    char err_buf[32];
    char samples_buf[32];
    std::snprintf(speed_buf, sizeof(speed_buf), "%.2f m/s", latest.speed);
    std::snprintf(goal_buf, sizeof(goal_buf), "%.2f m", latest.distance_to_goal);
    std::snprintf(lidar_buf, sizeof(lidar_buf), "%.2f m", latest.min_lidar);
    std::snprintf(err_buf, sizeof(err_buf), "%.2f / %.1f", latest.nav_xy_error, latest.nav_yaw_error_deg);
    std::snprintf(samples_buf, sizeof(samples_buf), "%d", static_cast<int>(history.size()));

    if (ImGui::BeginTable("SimTelemetryCards", 2, ImGuiTableFlags_SizingStretchSame)) {
        ImGui::TableNextColumn();
        metric_card("side_sim_speed", "Speed", speed_buf, "", ImVec4(0.37f, 0.63f, 0.76f, 1.0f), 54.0f);
        ImGui::TableNextColumn();
        metric_card("side_sim_goal", "Goal", goal_buf, "", ImVec4(0.80f, 0.62f, 0.34f, 1.0f), 54.0f);
        ImGui::TableNextColumn();
        metric_card("side_sim_lidar", "LiDAR", lidar_buf, "", ImVec4(0.42f, 0.70f, 0.49f, 1.0f), 54.0f);
        ImGui::TableNextColumn();
        metric_card("side_sim_ekf", "EKF", err_buf, "", ImVec4(0.78f, 0.72f, 0.38f, 1.0f), 54.0f);
        ImGui::EndTable();
    }

    ImGui::SeparatorText("Run");
    status_line("Samples", samples_buf, ImVec4(0.43f, 0.82f, 0.96f, 1.0f));
    status_line("Scenario",
                thesis_sim::environment_mode_name(sim.environment_mode()),
                ImVec4(0.48f, 0.88f, 0.62f, 1.0f));
    status_line("State",
                sim.goal_reached() ? "goal reached" : (sim.collision() ? "collision" : "running"),
                sim.goal_reached() ? ImVec4(0.48f, 0.88f, 0.62f, 1.0f)
                                   : (sim.collision() ? ImVec4(0.95f, 0.40f, 0.34f, 1.0f)
                                                      : ImVec4(0.87f, 0.79f, 0.39f, 1.0f)));

    ImGui::SeparatorText("Focus");
    ImGui::TextDisabled("Main");
    ImGui::TextWrapped("Velocity, planner commands, clearance, compute time.");
    ImGui::TextDisabled("Tracking");
    ImGui::TextWrapped("Steering, speed reference, cross-track and heading error.");

    std::vector<double> time;
    std::vector<double> speed;
    std::vector<double> goal;
    std::vector<double> tracking;
    std::vector<double> step_ms;
    const size_t stride = plot_sample_stride(history.size());
    for (size_t i = 0; i < history.size(); i += stride) {
        const TelemetrySample& sample = history[i];
        time.push_back(sample.time);
        speed.push_back(sample.speed);
        goal.push_back(sample.distance_to_goal);
        tracking.push_back(sample.tracker_cross_track);
        step_ms.push_back(sample.step_ms);
    }
    if ((history.size() - 1) % stride != 0) {
        const TelemetrySample& sample = history.back();
        time.push_back(sample.time);
        speed.push_back(sample.speed);
        goal.push_back(sample.distance_to_goal);
        tracking.push_back(sample.tracker_cross_track);
        step_ms.push_back(sample.step_ms);
    }

    ImGui::SeparatorText("Main Plots");
    if (ImPlot::BeginPlot("Speed / Goal##side_sim", ImVec2(-1.0f, 132.0f), ImPlotFlags_NoTitle)) {
        setup_time_plot_axes(time, "mission");
        plot_series(time, speed, "speed", ImVec4(0.36f, 0.73f, 0.98f, 1.0f));
        plot_series(time, goal, "goal", ImVec4(0.96f, 0.66f, 0.28f, 1.0f));
        ImPlot::EndPlot();
    }
    if (ImPlot::BeginPlot("Tracking / Step##side_sim", ImVec2(-1.0f, 132.0f), ImPlotFlags_NoTitle)) {
        setup_time_plot_axes(time, "tracking");
        plot_series(time, tracking, "cte", ImVec4(0.48f, 0.87f, 0.60f, 1.0f));
        plot_series(time, step_ms, "step ms", ImVec4(0.96f, 0.66f, 0.28f, 1.0f));
        ImPlot::EndPlot();
    }
    ImGui::EndChild();
}

void render_hardware_telemetry_side_panel(const HardwareViewerState& hardware) {
    if (!ImGui::BeginChild("HardwareTelemetrySidePanel", ImVec2(0.0f, 0.0f), true)) {
        ImGui::EndChild();
        return;
    }

    ImGui::TextUnformatted("Telemetry");
    if (hardware.history.empty()) {
        ImGui::TextDisabled("Waiting for live samples.");
        ImGui::EndChild();
        return;
    }

    const HardwareTelemetrySample& latest = hardware.history.back();
    char speed_buf[32];
    char goal_buf[32];
    char lidar_buf[32];
    char err_buf[32];
    char samples_buf[32];
    const std::string goal_label =
        hardware_goal_distance_label(hardware.frame, hardware.scene.world.environment_mode());
    std::snprintf(speed_buf, sizeof(speed_buf), "%.2f m/s", latest.speed);
    std::snprintf(goal_buf, sizeof(goal_buf), "%s", goal_label.c_str());
    std::snprintf(lidar_buf, sizeof(lidar_buf), "%.2f / %.2f m", latest.min_lidar, latest.front_lidar);
    std::snprintf(err_buf, sizeof(err_buf), "%.2f / %.1f", latest.tracker_cross_track, latest.tracker_heading_error_deg);
    std::snprintf(samples_buf, sizeof(samples_buf), "%d", static_cast<int>(hardware.history.size()));

    if (ImGui::BeginTable("HardwareTelemetryCards", 2, ImGuiTableFlags_SizingStretchSame)) {
        ImGui::TableNextColumn();
        metric_card("side_hw_speed", "Speed", speed_buf, "", ImVec4(0.37f, 0.63f, 0.76f, 1.0f), 54.0f);
        ImGui::TableNextColumn();
        metric_card("side_hw_goal", "Goal", goal_buf, "", ImVec4(0.80f, 0.62f, 0.34f, 1.0f), 54.0f);
        ImGui::TableNextColumn();
        metric_card("side_hw_lidar", "LiDAR", lidar_buf, "", ImVec4(0.42f, 0.70f, 0.49f, 1.0f), 54.0f);
        ImGui::TableNextColumn();
        metric_card("side_hw_tracking", "Tracking", err_buf, "", ImVec4(0.78f, 0.72f, 0.38f, 1.0f), 54.0f);
        ImGui::EndTable();
    }

    ImGui::SeparatorText("Run");
    status_line("Samples", samples_buf, ImVec4(0.43f, 0.82f, 0.96f, 1.0f));
    status_line("Planner",
                hardware.frame.planner_has_reference ? "reference ready" : "waiting",
                hardware.frame.planner_has_reference ? ImVec4(0.48f, 0.88f, 0.62f, 1.0f)
                                                     : ImVec4(0.87f, 0.79f, 0.39f, 1.0f));
    status_line("Safety",
                hardware.frame.safety_stop_active ? "stop active" : "clear",
                hardware.frame.safety_stop_active ? ImVec4(0.95f, 0.40f, 0.34f, 1.0f)
                                                  : ImVec4(0.48f, 0.88f, 0.62f, 1.0f));

    ImGui::SeparatorText("Focus");
    ImGui::TextDisabled("Main");
    ImGui::TextWrapped("Speed, road tracking, clearance, compute time.");
    ImGui::TextDisabled("Drivetrain");
    ImGui::TextWrapped("PWM echo, encoder deltas and controller timing.");

    std::vector<double> time;
    std::vector<double> speed;
    std::vector<double> ref_speed;
    std::vector<double> tracking;
    std::vector<double> clearance;
    const size_t stride = plot_sample_stride(hardware.history.size());
    for (size_t i = 0; i < hardware.history.size(); i += stride) {
        const HardwareTelemetrySample& sample = hardware.history[i];
        time.push_back(sample.time);
        speed.push_back(sample.speed);
        ref_speed.push_back(sample.planner_speed_ref);
        tracking.push_back(sample.tracker_cross_track);
        clearance.push_back(sample.front_lidar);
    }
    if ((hardware.history.size() - 1) % stride != 0) {
        const HardwareTelemetrySample& sample = hardware.history.back();
        time.push_back(sample.time);
        speed.push_back(sample.speed);
        ref_speed.push_back(sample.planner_speed_ref);
        tracking.push_back(sample.tracker_cross_track);
        clearance.push_back(sample.front_lidar);
    }

    ImGui::SeparatorText("Main Plots");
    if (ImPlot::BeginPlot("Speed / Ref##side_hw", ImVec2(-1.0f, 132.0f), ImPlotFlags_NoTitle)) {
        setup_time_plot_axes(time, "speed");
        plot_series(time, speed, "actual", ImVec4(0.36f, 0.73f, 0.98f, 1.0f));
        plot_series(time, ref_speed, "ref", ImVec4(0.96f, 0.66f, 0.28f, 1.0f));
        ImPlot::EndPlot();
    }
    if (ImPlot::BeginPlot("Tracking / Clearance##side_hw", ImVec2(-1.0f, 132.0f), ImPlotFlags_NoTitle)) {
        setup_time_plot_axes(time, "tracking");
        plot_series(time, tracking, "cte", ImVec4(0.48f, 0.87f, 0.60f, 1.0f));
        plot_series(time, clearance, "front", ImVec4(0.96f, 0.66f, 0.28f, 1.0f));
        ImPlot::EndPlot();
    }
    ImGui::EndChild();
}

void render_graphs_tab(PlannerDrivenVehicleSim& sim) {
    if (!ImGui::BeginChild("TelemetryViewport", ImVec2(0.0f, 0.0f), true)) {
        ImGui::EndChild();
        return;
    }

    const std::vector<TelemetrySample>& history = sim.history();
    if (history.empty()) {
        ImGui::TextUnformatted("No data yet.");
        ImGui::EndChild();
        return;
    }

    std::vector<double> time;
    std::vector<double> speed;
    std::vector<double> accel;
    std::vector<double> jerk;
    std::vector<double> command_r;
    std::vector<double> left_wheel_speed;
    std::vector<double> right_wheel_speed;
    std::vector<double> target_speed;
    std::vector<double> target_yaw_rate;
    std::vector<double> left_pwm;
    std::vector<double> right_pwm;
    std::vector<double> left_encoder_delta;
    std::vector<double> right_encoder_delta;
    std::vector<double> dist_goal;
    std::vector<double> min_lidar;
    std::vector<double> nav_xy_error;
    std::vector<double> nav_yaw_error_deg;
    std::vector<double> steer_angle;
    std::vector<double> target_steer_angle;
    std::vector<double> planner_speed_ref;
    std::vector<double> tracker_accel_cmd;
    std::vector<double> tracker_steer_rate_cmd;
    std::vector<double> tracker_cross_track;
    std::vector<double> tracker_heading_error_deg;
    std::vector<double> planning_ms;
    std::vector<double> tracking_ms;
    std::vector<double> lidar_ms;
    std::vector<double> estimator_ms;
    std::vector<double> step_ms;
    std::vector<double> visible_gates;
    std::vector<double> lidar_samples;
    time.reserve(history.size());
    speed.reserve(history.size());
    accel.reserve(history.size());
    jerk.reserve(history.size());
    command_r.reserve(history.size());
    left_wheel_speed.reserve(history.size());
    right_wheel_speed.reserve(history.size());
    target_speed.reserve(history.size());
    target_yaw_rate.reserve(history.size());
    left_pwm.reserve(history.size());
    right_pwm.reserve(history.size());
    left_encoder_delta.reserve(history.size());
    right_encoder_delta.reserve(history.size());
    dist_goal.reserve(history.size());
    min_lidar.reserve(history.size());
    nav_xy_error.reserve(history.size());
    nav_yaw_error_deg.reserve(history.size());
    steer_angle.reserve(history.size());
    target_steer_angle.reserve(history.size());
    planner_speed_ref.reserve(history.size());
    tracker_accel_cmd.reserve(history.size());
    tracker_steer_rate_cmd.reserve(history.size());
    tracker_cross_track.reserve(history.size());
    tracker_heading_error_deg.reserve(history.size());
    planning_ms.reserve(history.size());
    tracking_ms.reserve(history.size());
    lidar_ms.reserve(history.size());
    estimator_ms.reserve(history.size());
    step_ms.reserve(history.size());
    visible_gates.reserve(history.size());
    lidar_samples.reserve(history.size());

    const size_t stride = plot_sample_stride(history.size());
    for (size_t i = 0; i < history.size(); i += stride) {
        const TelemetrySample& sample = history[i];
        time.push_back(sample.time);
        speed.push_back(sample.speed);
        accel.push_back(sample.accel);
        jerk.push_back(sample.jerk);
        command_r.push_back(sample.command_r);
        left_wheel_speed.push_back(sample.left_wheel_speed);
        right_wheel_speed.push_back(sample.right_wheel_speed);
        target_speed.push_back(sample.target_speed);
        target_yaw_rate.push_back(sample.target_yaw_rate * 180.0 / 3.14159265358979323846);
        left_pwm.push_back(sample.left_pwm);
        right_pwm.push_back(sample.right_pwm);
        left_encoder_delta.push_back(sample.left_encoder_delta);
        right_encoder_delta.push_back(sample.right_encoder_delta);
        dist_goal.push_back(sample.distance_to_goal);
        min_lidar.push_back(sample.min_lidar);
        nav_xy_error.push_back(sample.nav_xy_error);
        nav_yaw_error_deg.push_back(sample.nav_yaw_error_deg);
        steer_angle.push_back(sample.steer_angle * 180.0 / 3.14159265358979323846);
        target_steer_angle.push_back(sample.target_steer_angle * 180.0 / 3.14159265358979323846);
        planner_speed_ref.push_back(sample.planner_speed_ref);
        tracker_accel_cmd.push_back(sample.tracker_accel_cmd);
        tracker_steer_rate_cmd.push_back(sample.tracker_steer_rate_cmd * 180.0 / 3.14159265358979323846);
        tracker_cross_track.push_back(sample.tracker_cross_track);
        tracker_heading_error_deg.push_back(sample.tracker_heading_error_deg);
        planning_ms.push_back(sample.planning_ms);
        tracking_ms.push_back(sample.tracking_ms);
        lidar_ms.push_back(sample.lidar_ms);
        estimator_ms.push_back(sample.estimator_ms);
        step_ms.push_back(sample.step_ms);
        visible_gates.push_back(sample.visible_gates);
        lidar_samples.push_back(sample.lidar_samples);
    }
    if ((history.size() - 1) % stride != 0) {
        const TelemetrySample& sample = history.back();
        time.push_back(sample.time);
        speed.push_back(sample.speed);
        accel.push_back(sample.accel);
        jerk.push_back(sample.jerk);
        command_r.push_back(sample.command_r);
        left_wheel_speed.push_back(sample.left_wheel_speed);
        right_wheel_speed.push_back(sample.right_wheel_speed);
        target_speed.push_back(sample.target_speed);
        target_yaw_rate.push_back(sample.target_yaw_rate * 180.0 / 3.14159265358979323846);
        left_pwm.push_back(sample.left_pwm);
        right_pwm.push_back(sample.right_pwm);
        left_encoder_delta.push_back(sample.left_encoder_delta);
        right_encoder_delta.push_back(sample.right_encoder_delta);
        dist_goal.push_back(sample.distance_to_goal);
        min_lidar.push_back(sample.min_lidar);
        nav_xy_error.push_back(sample.nav_xy_error);
        nav_yaw_error_deg.push_back(sample.nav_yaw_error_deg);
        steer_angle.push_back(sample.steer_angle * 180.0 / 3.14159265358979323846);
        target_steer_angle.push_back(sample.target_steer_angle * 180.0 / 3.14159265358979323846);
        planner_speed_ref.push_back(sample.planner_speed_ref);
        tracker_accel_cmd.push_back(sample.tracker_accel_cmd);
        tracker_steer_rate_cmd.push_back(sample.tracker_steer_rate_cmd * 180.0 / 3.14159265358979323846);
        tracker_cross_track.push_back(sample.tracker_cross_track);
        tracker_heading_error_deg.push_back(sample.tracker_heading_error_deg);
        planning_ms.push_back(sample.planning_ms);
        tracking_ms.push_back(sample.tracking_ms);
        lidar_ms.push_back(sample.lidar_ms);
        estimator_ms.push_back(sample.estimator_ms);
        step_ms.push_back(sample.step_ms);
        visible_gates.push_back(sample.visible_gates);
        lidar_samples.push_back(sample.lidar_samples);
    }

    if (ImGui::BeginTabBar("GraphTabs", ImGuiTabBarFlags_Reorderable)) {
        if (ImGui::BeginTabItem("Main")) {
            if (ImPlot::BeginSubplots("OverviewSubplots", 2, 2, ImVec2(-1.0f, -1.0f), ImPlotSubplotFlags_LinkAllX)) {
                if (ImPlot::BeginPlot("Velocity / Acceleration")) {
                    setup_time_plot_axes(time, "vehicle state");
                    plot_series(time, speed, "speed [m/s]", ImVec4(0.36f, 0.73f, 0.98f, 1.0f));
                    plot_series(time, accel, "accel [m/s^2]", ImVec4(0.96f, 0.66f, 0.28f, 1.0f));
                    render_plot_hover_overlay("Velocity / Acceleration", time, {
                                                                               {&speed, "speed [m/s]", ImVec4(0.36f, 0.73f, 0.98f, 1.0f)},
                                                                               {&accel, "accel [m/s^2]", ImVec4(0.96f, 0.66f, 0.28f, 1.0f)},
                                                                           });
                    ImPlot::EndPlot();
                }
                if (ImPlot::BeginPlot("Planner Commands")) {
                    setup_time_plot_axes(time, "planner command");
                    plot_series(time, jerk, "j [m/s^3]", ImVec4(0.36f, 0.73f, 0.98f, 1.0f));
                    plot_series(time, command_r, "r [1/(m*s)]", ImVec4(0.96f, 0.66f, 0.28f, 1.0f));
                    render_plot_hover_overlay("Planner Commands", time, {
                                                                           {&jerk, "j [m/s^3]", ImVec4(0.36f, 0.73f, 0.98f, 1.0f)},
                                                                           {&command_r, "r [1/(m*s)]", ImVec4(0.96f, 0.66f, 0.28f, 1.0f)},
                                                                       });
                    ImPlot::EndPlot();
                }
                if (ImPlot::BeginPlot("Distance / LiDAR")) {
                    setup_time_plot_axes(time, "distance");
                    plot_series(time, dist_goal, "goal distance [m]", ImVec4(0.36f, 0.73f, 0.98f, 1.0f));
                    plot_series(time, min_lidar, "LiDAR min [m]", ImVec4(0.48f, 0.87f, 0.60f, 1.0f));
                    render_plot_hover_overlay("Distance / LiDAR", time, {
                                                                         {&dist_goal, "goal distance [m]", ImVec4(0.36f, 0.73f, 0.98f, 1.0f)},
                                                                         {&min_lidar, "LiDAR min [m]", ImVec4(0.48f, 0.87f, 0.60f, 1.0f)},
                                                                     });
                    ImPlot::EndPlot();
                }
                if (ImPlot::BeginPlot("Step Timing")) {
                    setup_time_plot_axes(time, "ms");
                    plot_series(time, step_ms, "step total", ImVec4(0.96f, 0.66f, 0.28f, 1.0f));
                    plot_series(time, planning_ms, "planning", ImVec4(0.36f, 0.73f, 0.98f, 1.0f));
                    plot_series(time, tracking_ms, "tracking", ImVec4(0.48f, 0.87f, 0.60f, 1.0f));
                    render_plot_hover_overlay("Step Timing", time, {
                                                                      {&step_ms, "step total", ImVec4(0.96f, 0.66f, 0.28f, 1.0f)},
                                                                      {&planning_ms, "planning", ImVec4(0.36f, 0.73f, 0.98f, 1.0f)},
                                                                      {&tracking_ms, "tracking", ImVec4(0.48f, 0.87f, 0.60f, 1.0f)},
                                                                  });
                    ImPlot::EndPlot();
                }
                ImPlot::EndSubplots();
            }
            ImGui::EndTabItem();
        }

        if (ImGui::BeginTabItem("Timeline")) {
            ImGui::TextUnformatted("Mission events");
            ImGui::TextDisabled("Planner references, mixed arbitration, clearance warnings and performance spikes.");
            render_timeline_events(build_sim_timeline(sim), "SimulationTimelineEvents");
            ImGui::EndTabItem();
        }

        if (ImGui::BeginTabItem("Drivetrain")) {
            if (ImPlot::BeginSubplots("DrivetrainSubplots", 2, 2, ImVec2(-1.0f, -1.0f), ImPlotSubplotFlags_LinkAllX)) {
                if (ImPlot::BeginPlot("Target / Wheel Speed")) {
                    setup_time_plot_axes(time, "speed [m/s]");
                    plot_series(time, target_speed, "target v", ImVec4(0.36f, 0.73f, 0.98f, 1.0f));
                    plot_series(time, left_wheel_speed, "left wheel v", ImVec4(0.96f, 0.66f, 0.28f, 1.0f));
                    plot_series(time, right_wheel_speed, "right wheel v", ImVec4(0.48f, 0.87f, 0.60f, 1.0f));
                    render_plot_hover_overlay("Target / Wheel Speed", time, {
                                                                             {&target_speed, "target v", ImVec4(0.36f, 0.73f, 0.98f, 1.0f)},
                                                                             {&left_wheel_speed, "left wheel v", ImVec4(0.96f, 0.66f, 0.28f, 1.0f)},
                                                                             {&right_wheel_speed, "right wheel v", ImVec4(0.48f, 0.87f, 0.60f, 1.0f)},
                                                                         });
                    ImPlot::EndPlot();
                }
                if (ImPlot::BeginPlot("Target Yaw / PWM")) {
                    setup_time_plot_axes(time, "command / actuation");
                    plot_series(time, target_yaw_rate, "target wz [deg/s]", ImVec4(0.36f, 0.73f, 0.98f, 1.0f));
                    plot_series(time, left_pwm, "PWM left", ImVec4(0.96f, 0.66f, 0.28f, 1.0f));
                    plot_series(time, right_pwm, "PWM right", ImVec4(0.48f, 0.87f, 0.60f, 1.0f));
                    render_plot_hover_overlay("Target Yaw / PWM", time, {
                                                                          {&target_yaw_rate, "target wz [deg/s]", ImVec4(0.36f, 0.73f, 0.98f, 1.0f)},
                                                                          {&left_pwm, "PWM left", ImVec4(0.96f, 0.66f, 0.28f, 1.0f)},
                                                                          {&right_pwm, "PWM right", ImVec4(0.48f, 0.87f, 0.60f, 1.0f)},
                                                                      });
                    ImPlot::EndPlot();
                }
                if (ImPlot::BeginPlot("Encoder Delta")) {
                    setup_time_plot_axes(time, "ticks / step");
                    plot_series(time, left_encoder_delta, "dTicks left", ImVec4(0.36f, 0.73f, 0.98f, 1.0f));
                    plot_series(time, right_encoder_delta, "dTicks right", ImVec4(0.96f, 0.66f, 0.28f, 1.0f));
                    render_plot_hover_overlay("Encoder Delta", time, {
                                                                          {&left_encoder_delta, "dTicks left", ImVec4(0.36f, 0.73f, 0.98f, 1.0f)},
                                                                          {&right_encoder_delta, "dTicks right", ImVec4(0.96f, 0.66f, 0.28f, 1.0f)},
                                                                      });
                    ImPlot::EndPlot();
                }
                if (ImPlot::BeginPlot("Wheel Speed Balance")) {
                    setup_time_plot_axes(time, "wheel speed [m/s]");
                    plot_series(time, left_wheel_speed, "left wheel v", ImVec4(0.36f, 0.73f, 0.98f, 1.0f));
                    plot_series(time, right_wheel_speed, "right wheel v", ImVec4(0.96f, 0.66f, 0.28f, 1.0f));
                    render_plot_hover_overlay("Wheel Speed Balance", time, {
                                                                               {&left_wheel_speed, "left wheel v", ImVec4(0.36f, 0.73f, 0.98f, 1.0f)},
                                                                               {&right_wheel_speed, "right wheel v", ImVec4(0.96f, 0.66f, 0.28f, 1.0f)},
                                                                           });
                    ImPlot::EndPlot();
                }
                ImPlot::EndSubplots();
            }
            ImGui::EndTabItem();
        }

        if (ImGui::BeginTabItem("Localization")) {
            if (ImPlot::BeginSubplots("LocalizationSubplots", 2, 2, ImVec2(-1.0f, -1.0f), ImPlotSubplotFlags_LinkAllX)) {
                if (ImPlot::BeginPlot("Estimator Error")) {
                    setup_time_plot_axes(time, "EKF error");
                    plot_series(time, nav_xy_error, "xy err [m]", ImVec4(0.36f, 0.73f, 0.98f, 1.0f));
                    plot_series(time, nav_yaw_error_deg, "yaw err [deg]", ImVec4(0.96f, 0.66f, 0.28f, 1.0f));
                    render_plot_hover_overlay("Estimator Error", time, {
                                                                          {&nav_xy_error, "xy err [m]", ImVec4(0.36f, 0.73f, 0.98f, 1.0f)},
                                                                          {&nav_yaw_error_deg, "yaw err [deg]", ImVec4(0.96f, 0.66f, 0.28f, 1.0f)},
                                                                      });
                    ImPlot::EndPlot();
                }
                if (ImPlot::BeginPlot("LiDAR Compute / Samples")) {
                    setup_time_plot_axes(time, "sensor load");
                    plot_series(time, lidar_ms, "LiDAR ms", ImVec4(0.36f, 0.73f, 0.98f, 1.0f));
                    plot_series(time, lidar_samples, "LiDAR samples", ImVec4(0.48f, 0.87f, 0.60f, 1.0f));
                    render_plot_hover_overlay("LiDAR Compute / Samples", time, {
                                                                                 {&lidar_ms, "LiDAR ms", ImVec4(0.36f, 0.73f, 0.98f, 1.0f)},
                                                                                 {&lidar_samples, "LiDAR samples", ImVec4(0.48f, 0.87f, 0.60f, 1.0f)},
                                                                             });
                    ImPlot::EndPlot();
                }
                if (ImPlot::BeginPlot("Estimator Compute")) {
                    setup_time_plot_axes(time, "ms");
                    plot_series(time, estimator_ms, "EKF + fusion ms", ImVec4(0.96f, 0.66f, 0.28f, 1.0f));
                    render_plot_hover_overlay("Estimator Compute", time, {
                                                                            {&estimator_ms, "EKF + fusion ms", ImVec4(0.96f, 0.66f, 0.28f, 1.0f)},
                                                                        });
                    ImPlot::EndPlot();
                }
                if (ImPlot::BeginPlot("Goal / Visibility")) {
                    setup_time_plot_axes(time, "map context");
                    plot_series(time, dist_goal, "goal distance [m]", ImVec4(0.36f, 0.73f, 0.98f, 1.0f));
                    plot_series(time, visible_gates, "visible gates", ImVec4(0.96f, 0.66f, 0.28f, 1.0f));
                    render_plot_hover_overlay("Goal / Visibility", time, {
                                                                            {&dist_goal, "goal distance [m]", ImVec4(0.36f, 0.73f, 0.98f, 1.0f)},
                                                                            {&visible_gates, "visible gates", ImVec4(0.96f, 0.66f, 0.28f, 1.0f)},
                                                                        });
                    ImPlot::EndPlot();
                }
                ImPlot::EndSubplots();
            }
            ImGui::EndTabItem();
        }

        if (ImGui::BeginTabItem("Tracking")) {
            if (ImPlot::BeginSubplots("TrackingSubplots", 2, 2, ImVec2(-1.0f, -1.0f), ImPlotSubplotFlags_LinkAllX)) {
                if (ImPlot::BeginPlot("Steering Tracking")) {
                    setup_time_plot_axes(time, "steer [deg]");
                    plot_series(time, steer_angle, "steer actual", ImVec4(0.36f, 0.73f, 0.98f, 1.0f));
                    plot_series(time, target_steer_angle, "steer target", ImVec4(0.96f, 0.66f, 0.28f, 1.0f));
                    render_plot_hover_overlay("Steering Tracking", time, {
                                                                             {&steer_angle, "steer actual", ImVec4(0.36f, 0.73f, 0.98f, 1.0f)},
                                                                             {&target_steer_angle, "steer target", ImVec4(0.96f, 0.66f, 0.28f, 1.0f)},
                                                                         });
                    ImPlot::EndPlot();
                }
                if (ImPlot::BeginPlot("Follower Commands")) {
                    setup_time_plot_axes(time, "tracking command");
                    plot_series(time, tracker_accel_cmd, "accel cmd [m/s^2]", ImVec4(0.36f, 0.73f, 0.98f, 1.0f));
                    plot_series(time, tracker_steer_rate_cmd, "steer rate [deg/s]", ImVec4(0.96f, 0.66f, 0.28f, 1.0f));
                    plot_series(time, planner_speed_ref, "planner v ref", ImVec4(0.48f, 0.87f, 0.60f, 1.0f));
                    render_plot_hover_overlay("Follower Commands", time, {
                                                                            {&tracker_accel_cmd, "accel cmd [m/s^2]", ImVec4(0.36f, 0.73f, 0.98f, 1.0f)},
                                                                            {&tracker_steer_rate_cmd, "steer rate [deg/s]", ImVec4(0.96f, 0.66f, 0.28f, 1.0f)},
                                                                            {&planner_speed_ref, "planner v ref", ImVec4(0.48f, 0.87f, 0.60f, 1.0f)},
                                                                        });
                    ImPlot::EndPlot();
                }
                if (ImPlot::BeginPlot("Path Tracking Error")) {
                    setup_time_plot_axes(time, "tracking error");
                    plot_series(time, tracker_cross_track, "cross-track [m]", ImVec4(0.36f, 0.73f, 0.98f, 1.0f));
                    plot_series(time, tracker_heading_error_deg, "heading err [deg]", ImVec4(0.96f, 0.66f, 0.28f, 1.0f));
                    render_plot_hover_overlay("Path Tracking Error", time, {
                                                                                {&tracker_cross_track, "cross-track [m]", ImVec4(0.36f, 0.73f, 0.98f, 1.0f)},
                                                                                {&tracker_heading_error_deg, "heading err [deg]", ImVec4(0.96f, 0.66f, 0.28f, 1.0f)},
                                                                            });
                    ImPlot::EndPlot();
                }
                if (ImPlot::BeginPlot("Planner Ref vs Target Speed")) {
                    setup_time_plot_axes(time, "speed [m/s]");
                    plot_series(time, planner_speed_ref, "planner v ref", ImVec4(0.36f, 0.73f, 0.98f, 1.0f));
                    plot_series(time, target_speed, "target v", ImVec4(0.96f, 0.66f, 0.28f, 1.0f));
                    plot_series(time, speed, "actual v", ImVec4(0.48f, 0.87f, 0.60f, 1.0f));
                    render_plot_hover_overlay("Planner Ref vs Target Speed", time, {
                                                                                     {&planner_speed_ref, "planner v ref", ImVec4(0.36f, 0.73f, 0.98f, 1.0f)},
                                                                                     {&target_speed, "target v", ImVec4(0.96f, 0.66f, 0.28f, 1.0f)},
                                                                                     {&speed, "actual v", ImVec4(0.48f, 0.87f, 0.60f, 1.0f)},
                                                                                 });
                    ImPlot::EndPlot();
                }
                ImPlot::EndSubplots();
            }
            ImGui::EndTabItem();
        }

        if (ImGui::BeginTabItem("Performance")) {
            ImGui::TextWrapped("This tab is for lag diagnosis. In unstructured mode, spikes here usually come from repeated gate clothoid evaluation, LiDAR raycasting or state-estimation corrections.");
            if (ImPlot::BeginSubplots("PerformanceSubplots", 2, 2, ImVec2(-1.0f, -1.0f), ImPlotSubplotFlags_LinkAllX)) {
                if (ImPlot::BeginPlot("Planner / Tracking Compute")) {
                    setup_time_plot_axes(time, "ms");
                    plot_series(time, planning_ms, "planning ms", ImVec4(0.36f, 0.73f, 0.98f, 1.0f));
                    plot_series(time, tracking_ms, "tracking ms", ImVec4(0.96f, 0.66f, 0.28f, 1.0f));
                    render_plot_hover_overlay("Planner / Tracking Compute", time, {
                                                                                     {&planning_ms, "planning ms", ImVec4(0.36f, 0.73f, 0.98f, 1.0f)},
                                                                                     {&tracking_ms, "tracking ms", ImVec4(0.96f, 0.66f, 0.28f, 1.0f)},
                                                                                 });
                    ImPlot::EndPlot();
                }
                if (ImPlot::BeginPlot("Sensor / Estimator Compute")) {
                    setup_time_plot_axes(time, "ms");
                    plot_series(time, lidar_ms, "LiDAR ms", ImVec4(0.48f, 0.87f, 0.60f, 1.0f));
                    plot_series(time, estimator_ms, "EKF ms", ImVec4(0.96f, 0.66f, 0.28f, 1.0f));
                    plot_series(time, step_ms, "step total ms", ImVec4(0.36f, 0.73f, 0.98f, 1.0f));
                    render_plot_hover_overlay("Sensor / Estimator Compute", time, {
                                                                                     {&lidar_ms, "LiDAR ms", ImVec4(0.48f, 0.87f, 0.60f, 1.0f)},
                                                                                     {&estimator_ms, "EKF ms", ImVec4(0.96f, 0.66f, 0.28f, 1.0f)},
                                                                                     {&step_ms, "step total ms", ImVec4(0.36f, 0.73f, 0.98f, 1.0f)},
                                                                                 });
                    ImPlot::EndPlot();
                }
                if (ImPlot::BeginPlot("Visible Gates / LiDAR Samples")) {
                    setup_time_plot_axes(time, "load");
                    plot_series(time, visible_gates, "visible gates", ImVec4(0.36f, 0.73f, 0.98f, 1.0f));
                    plot_series(time, lidar_samples, "LiDAR samples", ImVec4(0.96f, 0.66f, 0.28f, 1.0f));
                    render_plot_hover_overlay("Visible Gates / LiDAR Samples", time, {
                                                                                     {&visible_gates, "visible gates", ImVec4(0.36f, 0.73f, 0.98f, 1.0f)},
                                                                                     {&lidar_samples, "LiDAR samples", ImVec4(0.96f, 0.66f, 0.28f, 1.0f)},
                                                                                 });
                    ImPlot::EndPlot();
                }
                if (ImPlot::BeginPlot("Step Total / Goal Distance")) {
                    setup_time_plot_axes(time, "mixed");
                    plot_series(time, step_ms, "step total ms", ImVec4(0.96f, 0.66f, 0.28f, 1.0f));
                    plot_series(time, dist_goal, "goal distance [m]", ImVec4(0.36f, 0.73f, 0.98f, 1.0f));
                    render_plot_hover_overlay("Step Total / Goal Distance", time, {
                                                                                 {&step_ms, "step total ms", ImVec4(0.96f, 0.66f, 0.28f, 1.0f)},
                                                                                 {&dist_goal, "goal distance [m]", ImVec4(0.36f, 0.73f, 0.98f, 1.0f)},
                                                                             });
                    ImPlot::EndPlot();
                }
                ImPlot::EndSubplots();
            }
            ImGui::EndTabItem();
        }

        ImGui::EndTabBar();
    }
    ImGui::EndChild();
}

void render_hardware_world_tab(const HardwareViewerState& hardware, UiState* ui_state) {
    if (ui_state == nullptr) {
        return;
    }

    if (!ImGui::BeginChild("HardwareWorldTabRoot", ImVec2(0.0f, 0.0f), false)) {
        ImGui::EndChild();
        return;
    }

    const bool show_live_scene = hardware.has_scene && !ui_state->hardware_world_sync_pending;
    const LiveFrameSnapshot& frame = hardware.frame;
    const WorldMap preview_world = show_live_scene ? hardware.scene.world : hardware_world_from_ui_selection(*ui_state);
    const WorldMap& world = preview_world;
    const char* active_target = "none";
    if (world.environment_mode() == EnvironmentMode::UnstructuredGates) {
        if (show_live_scene && frame.chosen_gate_index >= 0 && frame.chosen_gate_index < static_cast<int>(frame.gates.size())) {
            active_target = frame.gates[static_cast<std::size_t>(frame.chosen_gate_index)].spec.name.c_str();
        } else {
            active_target = thesis_sim::unstructured_map_preset_name(world.unstructured_preset());
        }
    } else {
        active_target = thesis_sim::structured_map_preset_name(world.structured_preset());
    }

    char speed_buf[32];
    char goal_buf[32];
    char tracking_buf[32];
    char target_buf[48];
    const std::string goal_label = hardware_goal_distance_label(frame, world.environment_mode());
    std::snprintf(speed_buf, sizeof(speed_buf), "%.2f m/s", frame.vehicle.speed);
    std::snprintf(goal_buf, sizeof(goal_buf), "%s", goal_label.c_str());
    std::snprintf(tracking_buf, sizeof(tracking_buf), "%.2f m / %.1f deg", frame.tracker_cross_track_error, frame.tracker_heading_error_deg);
    std::snprintf(target_buf, sizeof(target_buf), "%s", active_target);

    if (ImGui::BeginChild("HardwareWorldToolbar", ImVec2(0.0f, 84.0f), true)) {
        if (ImGui::BeginTable("HardwareWorldToolbarLayout", 2, ImGuiTableFlags_SizingStretchProp)) {
            ImGui::TableSetupColumn("WorldMetrics", ImGuiTableColumnFlags_WidthStretch, 1.0f);
            ImGui::TableSetupColumn("WorldControls", ImGuiTableColumnFlags_WidthFixed, 260.0f);

            ImGui::TableNextColumn();
            if (ImGui::BeginTable("HardwareMetricCards", 4, ImGuiTableFlags_SizingStretchSame)) {
                ImGui::TableNextColumn();
                metric_card("hw_world_speed", "Vehicle", speed_buf, "", ImVec4(0.43f, 0.82f, 0.96f, 1.0f), 56.0f);
                ImGui::TableNextColumn();
                metric_card("hw_world_goal", "Goal", goal_buf, "", ImVec4(0.99f, 0.70f, 0.32f, 1.0f), 56.0f);
                ImGui::TableNextColumn();
                metric_card("hw_world_tracking", "Tracking", tracking_buf, "", ImVec4(0.97f, 0.89f, 0.45f, 1.0f), 56.0f);
                ImGui::TableNextColumn();
                metric_card("hw_world_target", "Target", target_buf, "", ImVec4(0.48f, 0.88f, 0.62f, 1.0f), 56.0f);
                ImGui::EndTable();
            }

            ImGui::TableNextColumn();
            ImGui::TextDisabled("VIEW");
            ImGui::Checkbox("Grid", &ui_state->show_grid);
            ImGui::SameLine();
            ImGui::Checkbox("Path", &ui_state->show_trails);
            ImGui::SameLine();
            ImGui::Checkbox("HUD", &ui_state->show_world_hud);
            ImGui::Checkbox("Rays", &ui_state->show_lidar_rays);
            ImGui::SameLine();
            ImGui::Checkbox("Hits", &ui_state->show_lidar_hits);
            ImGui::SameLine();
            ImGui::Checkbox("Labels", &ui_state->show_gate_labels);
            ImGui::EndTable();
        }
    }
    ImGui::EndChild();

    if (!ImGui::BeginChild("HardwareViewport", ImVec2(0.0f, 0.0f), true, ImGuiWindowFlags_NoScrollbar)) {
        ImGui::EndChild();
        ImGui::EndChild();
        return;
    }

    const ImVec2 canvas_pos = ImGui::GetCursorScreenPos();
    ImVec2 canvas_size = ImGui::GetContentRegionAvail();
    canvas_size.x = std::max(canvas_size.x, 320.0f);
    canvas_size.y = std::max(canvas_size.y, 320.0f);

    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    draw_list->AddRectFilledMultiColor(
        canvas_pos,
        ImVec2(canvas_pos.x + canvas_size.x, canvas_pos.y + canvas_size.y),
        IM_COL32(16, 22, 28, 255),
        IM_COL32(23, 32, 39, 255),
        IM_COL32(12, 18, 22, 255),
        IM_COL32(14, 19, 24, 255));
    draw_list->AddRect(canvas_pos, ImVec2(canvas_pos.x + canvas_size.x, canvas_pos.y + canvas_size.y), IM_COL32(71, 81, 88, 255), 14.0f, 0, 1.5f);

    const CanvasTransform tx = make_transform(world, canvas_pos, canvas_size);
    const Rect bounds = world.bounds();
    if (ui_state->show_grid) {
        draw_world_grid(draw_list, tx, bounds);
    }

    for (const Rect& obstacle : world.obstacles()) {
        draw_list->AddRectFilled(world_to_screen(tx, {obstacle.min_x, obstacle.max_y}),
                                 world_to_screen(tx, {obstacle.max_x, obstacle.min_y}),
                                 kColorObstacle, 4.0f);
    }
    if (!world.road_centerline().empty()) {
        if (world.environment_mode() == EnvironmentMode::StructuredRoad) {
            draw_structured_road_map(draw_list, tx, world);
        } else {
            draw_polyline(draw_list, tx, world.road_centerline(), IM_COL32(113, 210, 255, 180), 4.0f);
        }
        if (world.environment_mode() == EnvironmentMode::StructuredRoad ||
            world.environment_mode() == EnvironmentMode::MixedRoadGates) {
            draw_goal_marker(draw_list, tx, world.goal());
        }
    }

    if (show_live_scene && ui_state->show_trails) {
        const ImU32 live_trail_color =
            world.environment_mode() == EnvironmentMode::StructuredRoad ? kColorTrail : kColorEstimateTrail;
        draw_polyline(draw_list, tx, hardware.frame.trail, live_trail_color, 2.5f);
        draw_polyline(draw_list, tx, hardware.frame.planned_trajectory, kColorTrajectory, 3.0f);
    }

    if (show_live_scene && !frame.slam_points.empty()) {
        const float occupancy_half_extent = std::max(
            1.5f,
            static_cast<float>(std::max(frame.occupancy_cell_size_m, 0.01) * tx.scale * 0.5));
        const size_t slam_stride = frame.slam_points.size() > 3200 ? (frame.slam_points.size() / 3200) + 1 : 1;
        for (size_t i = 0; i < frame.slam_points.size(); i += slam_stride) {
            const ImVec2 point_screen = world_to_screen(tx, frame.slam_points[i]);
            draw_list->AddRectFilled(
                ImVec2(point_screen.x - occupancy_half_extent, point_screen.y - occupancy_half_extent),
                ImVec2(point_screen.x + occupancy_half_extent, point_screen.y + occupancy_half_extent),
                IM_COL32(132, 255, 196, 44));
        }
    }

    if (show_live_scene && ui_state->show_lidar_hits && !frame.slam_points.empty()) {
        const size_t slam_stride = frame.slam_points.size() > 2800 ? (frame.slam_points.size() / 2800) + 1 : 1;
        for (size_t i = 0; i < frame.slam_points.size(); i += slam_stride) {
            const ImVec2 point_screen = world_to_screen(tx, frame.slam_points[i]);
            draw_list->AddCircleFilled(point_screen, 2.2f, IM_COL32(132, 255, 196, 92));
        }
    }

    if (show_live_scene && hardware.scene.lidar_enabled && (ui_state->show_lidar_rays || ui_state->show_lidar_hits)) {
        const Vec2 lidar_origin = frame.navigation_position;
        const size_t lidar_stride = frame.lidar_hits.size() > 720 ? 2 : 1;
        if (ui_state->show_lidar_rays) {
            for (size_t i = 0; i < frame.lidar_hits.size(); i += lidar_stride) {
                const LidarHit& hit = frame.lidar_hits[i];
                draw_list->AddLine(world_to_screen(tx, lidar_origin),
                                   world_to_screen(tx, hit.point),
                                   hit.hit ? kColorLidar : kColorLidarMiss,
                                   hit.hit ? 2.3f : 1.5f);
            }
        }
        if (ui_state->show_lidar_hits) {
            for (size_t i = 0; i < frame.lidar_hits.size(); i += lidar_stride) {
                const LidarHit& hit = frame.lidar_hits[i];
                const ImVec2 hit_screen = world_to_screen(tx, hit.point);
                if (hit.hit) {
                    draw_list->AddCircleFilled(hit_screen, 3.6f, kColorLidarHit);
                    draw_list->AddCircle(hit_screen, 6.2f, IM_COL32(231, 255, 212, 220), 0, 1.4f);
                } else {
                    draw_list->AddCircleFilled(hit_screen, 2.2f, IM_COL32(150, 207, 255, 165));
                }
            }
        }
        draw_list->AddCircleFilled(world_to_screen(tx, lidar_origin), 4.5f, IM_COL32(170, 255, 208, 220));
    }

    const size_t rendered_gate_count = show_live_scene ? frame.gates.size() : world.gates().size();
    for (size_t i = 0; i < rendered_gate_count; ++i) {
        const GateSpec& spec = show_live_scene ? frame.gates[i].spec : world.gates()[i];
        const bool passed = show_live_scene && frame.gates[i].passed;
        const bool visible =
            show_live_scene &&
            std::find(frame.visible_gate_indices.begin(), frame.visible_gate_indices.end(), static_cast<int>(i)) !=
                frame.visible_gate_indices.end();

        const ImVec2 screen_pos = world_to_screen(tx, spec.position);
        ImU32 color = spec.final ? kColorGoal : kColorGate;
        if (passed) {
            color = kColorGatePassed;
        } else if (visible) {
            color = kColorGateVisible;
        }
        if (show_live_scene && static_cast<int>(i) == frame.chosen_gate_index) {
            color = IM_COL32(255, 233, 118, 255);
        }

        if (world.environment_mode() == EnvironmentMode::MixedRoadGates && !spec.final) {
            draw_mixed_gate_acceptance_ring(draw_list, tx, world, spec.position);
        }
        const float radius = spec.final ? 7.0f : 5.0f;
        draw_list->AddCircleFilled(screen_pos, radius, color);
        draw_list->AddCircle(screen_pos, radius + 3.0f, IM_COL32(245, 246, 240, 180), 0, 1.5f);
        if (ui_state->show_gate_labels) {
            draw_list->AddText(ImVec2(screen_pos.x + 6.0f, screen_pos.y - 12.0f), IM_COL32(222, 227, 230, 255), spec.name.c_str());
        }
    }

    if (show_live_scene) {
        const VehicleSnapshot vehicle = build_vehicle_snapshot_from_live(frame.vehicle, hardware.scene.geometry);
        draw_vehicle(draw_list,
                     tx,
                     vehicle,
                     hardware.scene.geometry,
                     hardware_vehicle_visual_scale_for_world(world),
                     hardware_vehicle_body_color_for_world(world));
    }

    if (ui_state->show_world_hud) {
        char hud_line[96];
        char world_size_line[96];
        std::snprintf(world_size_line,
                      sizeof(world_size_line),
                      "area %.2f x %.2f m",
                      bounds.max_x - bounds.min_x,
                      bounds.max_y - bounds.min_y);
        std::vector<OverlayLine> top_left = {
            {show_live_scene
                 ? "Hardware viewport"
                 : "Hardware preview viewport",
             IM_COL32(240, 243, 235, 255)},
            {thesis_sim::environment_mode_name(world.environment_mode()), IM_COL32(170, 179, 185, 255)},
            {world.environment_mode() == EnvironmentMode::UnstructuredGates
                 ? thesis_sim::unstructured_map_preset_name(world.unstructured_preset())
                 : thesis_sim::structured_map_preset_name(world.structured_preset()),
             IM_COL32(170, 179, 185, 255)},
            {show_live_scene ? stream_profile_label(hardware.scene.stream_profile)
                                : "Local preview",
             IM_COL32(170, 179, 185, 255)},
            {world_size_line, IM_COL32(170, 179, 185, 255)},
            {show_live_scene ? hardware.scene.range_sensor_name : "Edit and export before Raspberry launch",
             IM_COL32(170, 179, 185, 255)},
        };
        draw_overlay_panel(draw_list, ImVec2(canvas_pos.x + 16.0f, canvas_pos.y + 16.0f), 228.0f, top_left);

        std::snprintf(hud_line, sizeof(hud_line), "speed %.2f m/s", frame.vehicle.speed);
        std::vector<OverlayLine> top_right = {
            {show_live_scene
                 ? (frame.goal_reached ? "Mission complete"
                                       : (frame.safety_stop_active ? "Safety stop active"
                                                                   : "Hardware stream live"))
                 : "Preview ready",
             show_live_scene
                 ? (frame.goal_reached ? IM_COL32(124, 238, 151, 255)
                                       : (frame.safety_stop_active ? IM_COL32(255, 124, 102, 255) : IM_COL32(255, 221, 113, 255)))
                 : IM_COL32(132, 214, 255, 255)},
            {hud_line, IM_COL32(240, 243, 235, 255)},
        };
        std::snprintf(hud_line, sizeof(hud_line), "goal %s", goal_label.c_str());
        top_right.push_back({hud_line, IM_COL32(170, 179, 185, 255)});
        if (world.environment_mode() == EnvironmentMode::UnstructuredGates) {
            if (show_live_scene &&
                frame.dynamic_gap_gates &&
                !frame.goal_reached &&
                frame.chosen_gate_index < 0 &&
                !frame.planner_has_reference) {
                std::snprintf(hud_line,
                              sizeof(hud_line),
                              "sensing | map %d",
                              static_cast<int>(frame.slam_points.size()));
            } else if (show_live_scene) {
                std::snprintf(hud_line,
                              sizeof(hud_line),
                              "gate %s | cand %d | map %d",
                              active_target,
                              frame.candidate_gates,
                              static_cast<int>(frame.slam_points.size()));
            } else {
                std::snprintf(hud_line, sizeof(hud_line), "preview gates %d", static_cast<int>(world.gates().size()));
            }
        } else {
            std::snprintf(hud_line, sizeof(hud_line), "road pts %d", static_cast<int>(world.road_centerline().size()));
        }
        top_right.push_back({hud_line, IM_COL32(170, 179, 185, 255)});
        draw_overlay_panel(draw_list, ImVec2(canvas_pos.x + canvas_size.x - 236.0f, canvas_pos.y + 16.0f), 220.0f, top_right);

        std::vector<OverlayLine> legend = {
            {world.environment_mode() == EnvironmentMode::StructuredRoad
                 ? "blue: estimated hardware trail"
                 : "orange: estimated hardware trail",
             world.environment_mode() == EnvironmentMode::StructuredRoad ? kColorTrail : kColorEstimateTrail},
            {"yellow: selected planner trajectory", kColorTrajectory},
        };
        if (world.environment_mode() == EnvironmentMode::StructuredRoad) {
            legend.push_back({"red: structured road bounds", IM_COL32(255, 96, 96, 230)});
        }
        if (show_live_scene && hardware.scene.lidar_enabled && ui_state->show_lidar_rays) {
            legend.push_back({"green: LiDAR hit rays", kColorLidar});
        }
        if (show_live_scene && hardware.scene.lidar_enabled && ui_state->show_lidar_hits) {
            legend.push_back({"lime/cyan: LiDAR collision points", kColorLidarHit});
        }
        if (show_live_scene && !frame.slam_points.empty() && ui_state->show_lidar_hits) {
            legend.push_back({"mint: occupied LiDAR map cells", IM_COL32(132, 255, 196, 200)});
            legend.push_back({"empty canvas can still be free space", IM_COL32(170, 179, 185, 255)});
        }
        if (!show_live_scene && ui_state->map_editor_enabled) {
            legend.push_back({"blue: editable preview road", kColorEditorOverlay});
        }
        const float legend_height = 20.0f + static_cast<float>(legend.size()) * (ImGui::GetFontSize() + 5.0f);
        draw_overlay_panel(draw_list,
                           ImVec2(canvas_pos.x + 16.0f, canvas_pos.y + canvas_size.y - legend_height),
                           304.0f,
                           legend);
    }

    if (!show_live_scene && ui_state->map_editor_enabled) {
        draw_editor_overlay(draw_list, tx, ui_state->hardware_editor_world, *ui_state);

        const ImVec2 mouse_pos = ImGui::GetIO().MousePos;
        const bool inside_canvas =
            mouse_pos.x >= canvas_pos.x && mouse_pos.x <= canvas_pos.x + canvas_size.x &&
            mouse_pos.y >= canvas_pos.y && mouse_pos.y <= canvas_pos.y + canvas_size.y;
        if (inside_canvas && ImGui::IsWindowHovered(ImGuiHoveredFlags_AllowWhenBlockedByActiveItem)) {
            update_editor_drag(ui_state, tx, mouse_pos);
        } else if (ui_state->active_drag_handle.type != MapEditorHandleType::None &&
                   !ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
            ui_state->active_drag_handle = {};
        }
    }

    ImGui::Dummy(canvas_size);
    ImGui::EndChild();
    ImGui::EndChild();
}

void render_hardware_graphs_tab(const HardwareViewerState& hardware) {
    if (!ImGui::BeginChild("HardwareTelemetryViewport", ImVec2(0.0f, 0.0f), true)) {
        ImGui::EndChild();
        return;
    }

    if (hardware.history.empty()) {
        ImGui::TextUnformatted("Waiting for live hardware telemetry samples.");
        ImGui::EndChild();
        return;
    }

    std::vector<double> time;
    std::vector<double> speed;
    std::vector<double> accel;
    std::vector<double> yaw_rate;
    std::vector<double> jerk;
    std::vector<double> command_r;
    std::vector<double> target_speed;
    std::vector<double> target_yaw_rate;
    std::vector<double> left_pwm;
    std::vector<double> right_pwm;
    std::vector<double> controller_left_pwm;
    std::vector<double> controller_right_pwm;
    std::vector<double> controller_target_pwm_left;
    std::vector<double> controller_target_pwm_right;
    std::vector<double> controller_left_encoder_ticks;
    std::vector<double> controller_right_encoder_ticks;
    std::vector<double> controller_left_encoder_delta;
    std::vector<double> controller_right_encoder_delta;
    std::vector<double> controller_encoder_dt_ms;
    std::vector<double> dist_goal;
    std::vector<double> min_lidar;
    std::vector<double> front_lidar;
    std::vector<double> planner_speed_ref;
    std::vector<double> tracker_cross_track;
    std::vector<double> tracker_heading_error_deg;
    std::vector<double> planning_ms;
    std::vector<double> tracking_ms;
    std::vector<double> lidar_ms;
    std::vector<double> estimator_ms;
    std::vector<double> step_ms;
    std::vector<double> visible_gates;

    time.reserve(hardware.history.size());
    speed.reserve(hardware.history.size());
    accel.reserve(hardware.history.size());
    yaw_rate.reserve(hardware.history.size());
    jerk.reserve(hardware.history.size());
    command_r.reserve(hardware.history.size());
    target_speed.reserve(hardware.history.size());
    target_yaw_rate.reserve(hardware.history.size());
    left_pwm.reserve(hardware.history.size());
    right_pwm.reserve(hardware.history.size());
    controller_left_pwm.reserve(hardware.history.size());
    controller_right_pwm.reserve(hardware.history.size());
    controller_target_pwm_left.reserve(hardware.history.size());
    controller_target_pwm_right.reserve(hardware.history.size());
    controller_left_encoder_ticks.reserve(hardware.history.size());
    controller_right_encoder_ticks.reserve(hardware.history.size());
    controller_left_encoder_delta.reserve(hardware.history.size());
    controller_right_encoder_delta.reserve(hardware.history.size());
    controller_encoder_dt_ms.reserve(hardware.history.size());
    dist_goal.reserve(hardware.history.size());
    min_lidar.reserve(hardware.history.size());
    front_lidar.reserve(hardware.history.size());
    planner_speed_ref.reserve(hardware.history.size());
    tracker_cross_track.reserve(hardware.history.size());
    tracker_heading_error_deg.reserve(hardware.history.size());
    planning_ms.reserve(hardware.history.size());
    tracking_ms.reserve(hardware.history.size());
    lidar_ms.reserve(hardware.history.size());
    estimator_ms.reserve(hardware.history.size());
    step_ms.reserve(hardware.history.size());
    visible_gates.reserve(hardware.history.size());

    const size_t stride = plot_sample_stride(hardware.history.size());
    for (size_t i = 0; i < hardware.history.size(); i += stride) {
        const HardwareTelemetrySample& sample = hardware.history[i];
        time.push_back(sample.time);
        speed.push_back(sample.speed);
        accel.push_back(sample.accel);
        yaw_rate.push_back(sample.yaw_rate * 180.0 / 3.14159265358979323846);
        jerk.push_back(sample.jerk);
        command_r.push_back(sample.command_r);
        target_speed.push_back(sample.target_speed);
        target_yaw_rate.push_back(sample.target_yaw_rate * 180.0 / 3.14159265358979323846);
        left_pwm.push_back(static_cast<double>(sample.pwm_left));
        right_pwm.push_back(static_cast<double>(sample.pwm_right));
        controller_left_pwm.push_back(static_cast<double>(sample.controller_pwm_left));
        controller_right_pwm.push_back(static_cast<double>(sample.controller_pwm_right));
        controller_target_pwm_left.push_back(static_cast<double>(sample.controller_target_pwm_left));
        controller_target_pwm_right.push_back(static_cast<double>(sample.controller_target_pwm_right));
        controller_left_encoder_ticks.push_back(static_cast<double>(sample.controller_left_encoder_ticks));
        controller_right_encoder_ticks.push_back(static_cast<double>(sample.controller_right_encoder_ticks));
        controller_left_encoder_delta.push_back(static_cast<double>(sample.controller_left_encoder_delta));
        controller_right_encoder_delta.push_back(static_cast<double>(sample.controller_right_encoder_delta));
        controller_encoder_dt_ms.push_back(sample.controller_encoder_dt_ms);
        dist_goal.push_back(sample.distance_to_goal);
        min_lidar.push_back(sample.min_lidar);
        front_lidar.push_back(sample.front_lidar);
        planner_speed_ref.push_back(sample.planner_speed_ref);
        tracker_cross_track.push_back(sample.tracker_cross_track);
        tracker_heading_error_deg.push_back(sample.tracker_heading_error_deg);
        planning_ms.push_back(sample.planning_ms);
        tracking_ms.push_back(sample.tracking_ms);
        lidar_ms.push_back(sample.lidar_ms);
        estimator_ms.push_back(sample.estimator_ms);
        step_ms.push_back(sample.step_ms);
        visible_gates.push_back(sample.visible_gates);
    }
    if ((hardware.history.size() - 1) % stride != 0) {
        const HardwareTelemetrySample& sample = hardware.history.back();
        time.push_back(sample.time);
        speed.push_back(sample.speed);
        accel.push_back(sample.accel);
        yaw_rate.push_back(sample.yaw_rate * 180.0 / 3.14159265358979323846);
        jerk.push_back(sample.jerk);
        command_r.push_back(sample.command_r);
        target_speed.push_back(sample.target_speed);
        target_yaw_rate.push_back(sample.target_yaw_rate * 180.0 / 3.14159265358979323846);
        left_pwm.push_back(static_cast<double>(sample.pwm_left));
        right_pwm.push_back(static_cast<double>(sample.pwm_right));
        controller_left_pwm.push_back(static_cast<double>(sample.controller_pwm_left));
        controller_right_pwm.push_back(static_cast<double>(sample.controller_pwm_right));
        controller_target_pwm_left.push_back(static_cast<double>(sample.controller_target_pwm_left));
        controller_target_pwm_right.push_back(static_cast<double>(sample.controller_target_pwm_right));
        controller_left_encoder_ticks.push_back(static_cast<double>(sample.controller_left_encoder_ticks));
        controller_right_encoder_ticks.push_back(static_cast<double>(sample.controller_right_encoder_ticks));
        controller_left_encoder_delta.push_back(static_cast<double>(sample.controller_left_encoder_delta));
        controller_right_encoder_delta.push_back(static_cast<double>(sample.controller_right_encoder_delta));
        controller_encoder_dt_ms.push_back(sample.controller_encoder_dt_ms);
        dist_goal.push_back(sample.distance_to_goal);
        min_lidar.push_back(sample.min_lidar);
        front_lidar.push_back(sample.front_lidar);
        planner_speed_ref.push_back(sample.planner_speed_ref);
        tracker_cross_track.push_back(sample.tracker_cross_track);
        tracker_heading_error_deg.push_back(sample.tracker_heading_error_deg);
        planning_ms.push_back(sample.planning_ms);
        tracking_ms.push_back(sample.tracking_ms);
        lidar_ms.push_back(sample.lidar_ms);
        estimator_ms.push_back(sample.estimator_ms);
        step_ms.push_back(sample.step_ms);
        visible_gates.push_back(sample.visible_gates);
    }

    if (ImGui::BeginTabBar("HardwareGraphTabs", ImGuiTabBarFlags_Reorderable)) {
        if (ImGui::BeginTabItem("Main")) {
            if (ImPlot::BeginSubplots("HardwareOverviewSubplots", 2, 2, ImVec2(-1.0f, -1.0f), ImPlotSubplotFlags_LinkAllX)) {
                if (ImPlot::BeginPlot("Velocity / Acceleration")) {
                    setup_time_plot_axes(time, "vehicle state");
                    plot_series(time, speed, "speed [m/s]", ImVec4(0.36f, 0.73f, 0.98f, 1.0f));
                    plot_series(time, accel, "accel [m/s^2]", ImVec4(0.96f, 0.66f, 0.28f, 1.0f));
                    render_plot_hover_overlay("Velocity / Acceleration", time, {
                                                                               {&speed, "speed [m/s]", ImVec4(0.36f, 0.73f, 0.98f, 1.0f)},
                                                                               {&accel, "accel [m/s^2]", ImVec4(0.96f, 0.66f, 0.28f, 1.0f)},
                                                                           });
                    ImPlot::EndPlot();
                }
                if (ImPlot::BeginPlot("Planner Commands")) {
                    setup_time_plot_axes(time, "planner command");
                    plot_series(time, jerk, "j [m/s^3]", ImVec4(0.36f, 0.73f, 0.98f, 1.0f));
                    plot_series(time, command_r, "r [1/(m*s)]", ImVec4(0.96f, 0.66f, 0.28f, 1.0f));
                    render_plot_hover_overlay("Planner Commands", time, {
                                                                           {&jerk, "j [m/s^3]", ImVec4(0.36f, 0.73f, 0.98f, 1.0f)},
                                                                           {&command_r, "r [1/(m*s)]", ImVec4(0.96f, 0.66f, 0.28f, 1.0f)},
                                                                       });
                    ImPlot::EndPlot();
                }
                if (ImPlot::BeginPlot("Distance / LiDAR")) {
                    setup_time_plot_axes(time, "distance");
                    plot_series(time, dist_goal, "goal distance [m]", ImVec4(0.36f, 0.73f, 0.98f, 1.0f));
                    plot_series(time, min_lidar, "LiDAR min [m]", ImVec4(0.48f, 0.87f, 0.60f, 1.0f));
                    plot_series(time, front_lidar, "LiDAR front [m]", ImVec4(0.96f, 0.66f, 0.28f, 1.0f));
                    render_plot_hover_overlay("Distance / LiDAR", time, {
                                                                         {&dist_goal, "goal distance [m]", ImVec4(0.36f, 0.73f, 0.98f, 1.0f)},
                                                                         {&min_lidar, "LiDAR min [m]", ImVec4(0.48f, 0.87f, 0.60f, 1.0f)},
                                                                         {&front_lidar, "LiDAR front [m]", ImVec4(0.96f, 0.66f, 0.28f, 1.0f)},
                                                                     });
                    ImPlot::EndPlot();
                }
                if (ImPlot::BeginPlot("Step Timing")) {
                    setup_time_plot_axes(time, "ms");
                    plot_series(time, step_ms, "step total", ImVec4(0.96f, 0.66f, 0.28f, 1.0f));
                    plot_series(time, planning_ms, "planning", ImVec4(0.36f, 0.73f, 0.98f, 1.0f));
                    plot_series(time, tracking_ms, "tracking", ImVec4(0.48f, 0.87f, 0.60f, 1.0f));
                    render_plot_hover_overlay("Step Timing", time, {
                                                                      {&step_ms, "step total", ImVec4(0.96f, 0.66f, 0.28f, 1.0f)},
                                                                      {&planning_ms, "planning", ImVec4(0.36f, 0.73f, 0.98f, 1.0f)},
                                                                      {&tracking_ms, "tracking", ImVec4(0.48f, 0.87f, 0.60f, 1.0f)},
                                                                  });
                    ImPlot::EndPlot();
                }
                ImPlot::EndSubplots();
            }
            ImGui::EndTabItem();
        }

        if (ImGui::BeginTabItem("Timeline")) {
            ImGui::TextUnformatted("Hardware events");
            ImGui::TextDisabled("Reference state, safety stops, front-clearance warnings and loop timing spikes.");
            render_timeline_events(build_hardware_timeline(hardware), "HardwareTimelineEvents");
            ImGui::EndTabItem();
        }

        if (ImGui::BeginTabItem("Tracking")) {
            if (ImPlot::BeginSubplots("HardwareTrackingSubplots", 2, 2, ImVec2(-1.0f, -1.0f), ImPlotSubplotFlags_LinkAllX)) {
                if (ImPlot::BeginPlot("Speed Tracking")) {
                    setup_time_plot_axes(time, "speed [m/s]");
                    plot_series(time, planner_speed_ref, "planner v ref", ImVec4(0.36f, 0.73f, 0.98f, 1.0f));
                    plot_series(time, target_speed, "target v", ImVec4(0.96f, 0.66f, 0.28f, 1.0f));
                    plot_series(time, speed, "actual v", ImVec4(0.48f, 0.87f, 0.60f, 1.0f));
                    render_plot_hover_overlay("Speed Tracking", time, {
                                                                           {&planner_speed_ref, "planner v ref", ImVec4(0.36f, 0.73f, 0.98f, 1.0f)},
                                                                           {&target_speed, "target v", ImVec4(0.96f, 0.66f, 0.28f, 1.0f)},
                                                                           {&speed, "actual v", ImVec4(0.48f, 0.87f, 0.60f, 1.0f)},
                                                                       });
                    ImPlot::EndPlot();
                }
                if (ImPlot::BeginPlot("Path Tracking Error")) {
                    setup_time_plot_axes(time, "tracking error");
                    plot_series(time, tracker_cross_track, "cross-track [m]", ImVec4(0.36f, 0.73f, 0.98f, 1.0f));
                    plot_series(time, tracker_heading_error_deg, "heading err [deg]", ImVec4(0.96f, 0.66f, 0.28f, 1.0f));
                    render_plot_hover_overlay("Path Tracking Error", time, {
                                                                                {&tracker_cross_track, "cross-track [m]", ImVec4(0.36f, 0.73f, 0.98f, 1.0f)},
                                                                                {&tracker_heading_error_deg, "heading err [deg]", ImVec4(0.96f, 0.66f, 0.28f, 1.0f)},
                                                                            });
                    ImPlot::EndPlot();
                }
                if (ImPlot::BeginPlot("Yaw Target / PWM")) {
                    setup_time_plot_axes(time, "yaw / actuation");
                    plot_series(time, target_yaw_rate, "target wz [deg/s]", ImVec4(0.36f, 0.73f, 0.98f, 1.0f));
                    plot_series(time, yaw_rate, "measured wz [deg/s]", ImVec4(0.48f, 0.87f, 0.60f, 1.0f));
                    plot_series(time, left_pwm, "PWM left", ImVec4(0.96f, 0.66f, 0.28f, 1.0f));
                    render_plot_hover_overlay("Yaw Target / PWM", time, {
                                                                          {&target_yaw_rate, "target wz [deg/s]", ImVec4(0.36f, 0.73f, 0.98f, 1.0f)},
                                                                          {&yaw_rate, "measured wz [deg/s]", ImVec4(0.48f, 0.87f, 0.60f, 1.0f)},
                                                                          {&left_pwm, "PWM left", ImVec4(0.96f, 0.66f, 0.28f, 1.0f)},
                                                                      });
                    ImPlot::EndPlot();
                }
                if (ImPlot::BeginPlot("Wheel Commands")) {
                    setup_time_plot_axes(time, "PWM");
                    plot_series(time, left_pwm, "PWM left", ImVec4(0.36f, 0.73f, 0.98f, 1.0f));
                    plot_series(time, right_pwm, "PWM right", ImVec4(0.96f, 0.66f, 0.28f, 1.0f));
                    render_plot_hover_overlay("Wheel Commands", time, {
                                                                          {&left_pwm, "PWM left", ImVec4(0.36f, 0.73f, 0.98f, 1.0f)},
                                                                          {&right_pwm, "PWM right", ImVec4(0.96f, 0.66f, 0.28f, 1.0f)},
                                                                      });
                    ImPlot::EndPlot();
                }
                ImPlot::EndSubplots();
            }
            ImGui::EndTabItem();
        }

        if (ImGui::BeginTabItem("Drivetrain")) {
            if (ImPlot::BeginSubplots("HardwareDrivetrainSubplots", 2, 2, ImVec2(-1.0f, -1.0f), ImPlotSubplotFlags_LinkAllX)) {
                if (ImPlot::BeginPlot("Controller PWM Echo")) {
                    setup_time_plot_axes(time, "PWM");
                    plot_series(time, controller_target_pwm_left, "target PWM left", ImVec4(0.36f, 0.73f, 0.98f, 1.0f));
                    plot_series(time, controller_target_pwm_right, "target PWM right", ImVec4(0.48f, 0.87f, 0.60f, 1.0f));
                    plot_series(time, controller_left_pwm, "PWM left", ImVec4(0.96f, 0.66f, 0.28f, 1.0f));
                    plot_series(time, controller_right_pwm, "PWM right", ImVec4(0.93f, 0.43f, 0.62f, 1.0f));
                    render_plot_hover_overlay("Controller PWM Echo", time, {
                                                                                {&controller_target_pwm_left, "target PWM left", ImVec4(0.36f, 0.73f, 0.98f, 1.0f)},
                                                                                {&controller_target_pwm_right, "target PWM right", ImVec4(0.48f, 0.87f, 0.60f, 1.0f)},
                                                                                {&controller_left_pwm, "PWM left", ImVec4(0.96f, 0.66f, 0.28f, 1.0f)},
                                                                                {&controller_right_pwm, "PWM right", ImVec4(0.93f, 0.43f, 0.62f, 1.0f)},
                                                                            });
                    ImPlot::EndPlot();
                }
                if (ImPlot::BeginPlot("Controller Encoder Delta")) {
                    setup_time_plot_axes(time, "ticks / sample");
                    plot_series(time, controller_left_encoder_delta, "dTicks left", ImVec4(0.36f, 0.73f, 0.98f, 1.0f));
                    plot_series(time, controller_right_encoder_delta, "dTicks right", ImVec4(0.96f, 0.66f, 0.28f, 1.0f));
                    render_plot_hover_overlay("Controller Encoder Delta", time, {
                                                                                   {&controller_left_encoder_delta, "dTicks left", ImVec4(0.36f, 0.73f, 0.98f, 1.0f)},
                                                                                   {&controller_right_encoder_delta, "dTicks right", ImVec4(0.96f, 0.66f, 0.28f, 1.0f)},
                                                                               });
                    ImPlot::EndPlot();
                }
                if (ImPlot::BeginPlot("Controller Encoder Ticks")) {
                    setup_time_plot_axes(time, "ticks total");
                    plot_series(time, controller_left_encoder_ticks, "ticks left", ImVec4(0.36f, 0.73f, 0.98f, 1.0f));
                    plot_series(time, controller_right_encoder_ticks, "ticks right", ImVec4(0.96f, 0.66f, 0.28f, 1.0f));
                    render_plot_hover_overlay("Controller Encoder Ticks", time, {
                                                                                   {&controller_left_encoder_ticks, "ticks left", ImVec4(0.36f, 0.73f, 0.98f, 1.0f)},
                                                                                   {&controller_right_encoder_ticks, "ticks right", ImVec4(0.96f, 0.66f, 0.28f, 1.0f)},
                                                                               });
                    ImPlot::EndPlot();
                }
                if (ImPlot::BeginPlot("Encoder dt / Speed")) {
                    setup_time_plot_axes(time, "mixed");
                    plot_series(time, controller_encoder_dt_ms, "enc dt [ms]", ImVec4(0.96f, 0.66f, 0.28f, 1.0f));
                    plot_series(time, speed, "speed [m/s]", ImVec4(0.36f, 0.73f, 0.98f, 1.0f));
                    render_plot_hover_overlay("Encoder dt / Speed", time, {
                                                                              {&controller_encoder_dt_ms, "enc dt [ms]", ImVec4(0.96f, 0.66f, 0.28f, 1.0f)},
                                                                              {&speed, "speed [m/s]", ImVec4(0.36f, 0.73f, 0.98f, 1.0f)},
                                                                          });
                    ImPlot::EndPlot();
                }
                ImPlot::EndSubplots();
            }
            ImGui::EndTabItem();
        }

        if (ImGui::BeginTabItem("Performance")) {
            if (ImPlot::BeginSubplots("HardwarePerformanceSubplots", 2, 2, ImVec2(-1.0f, -1.0f), ImPlotSubplotFlags_LinkAllX)) {
                if (ImPlot::BeginPlot("Planner / Tracking Compute")) {
                    setup_time_plot_axes(time, "ms");
                    plot_series(time, planning_ms, "planning ms", ImVec4(0.36f, 0.73f, 0.98f, 1.0f));
                    plot_series(time, tracking_ms, "tracking ms", ImVec4(0.96f, 0.66f, 0.28f, 1.0f));
                    render_plot_hover_overlay("Planner / Tracking Compute", time, {
                                                                                     {&planning_ms, "planning ms", ImVec4(0.36f, 0.73f, 0.98f, 1.0f)},
                                                                                     {&tracking_ms, "tracking ms", ImVec4(0.96f, 0.66f, 0.28f, 1.0f)},
                                                                                 });
                    ImPlot::EndPlot();
                }
                if (ImPlot::BeginPlot("Sensor / Estimator Compute")) {
                    setup_time_plot_axes(time, "ms");
                    plot_series(time, lidar_ms, "LiDAR ms", ImVec4(0.48f, 0.87f, 0.60f, 1.0f));
                    plot_series(time, estimator_ms, "EKF ms", ImVec4(0.96f, 0.66f, 0.28f, 1.0f));
                    plot_series(time, step_ms, "step total ms", ImVec4(0.36f, 0.73f, 0.98f, 1.0f));
                    render_plot_hover_overlay("Sensor / Estimator Compute", time, {
                                                                                     {&lidar_ms, "LiDAR ms", ImVec4(0.48f, 0.87f, 0.60f, 1.0f)},
                                                                                     {&estimator_ms, "EKF ms", ImVec4(0.96f, 0.66f, 0.28f, 1.0f)},
                                                                                     {&step_ms, "step total ms", ImVec4(0.36f, 0.73f, 0.98f, 1.0f)},
                                                                                 });
                    ImPlot::EndPlot();
                }
                if (ImPlot::BeginPlot("Visible Gates / Goal")) {
                    setup_time_plot_axes(time, "mission context");
                    plot_series(time, visible_gates, "visible gates", ImVec4(0.36f, 0.73f, 0.98f, 1.0f));
                    plot_series(time, dist_goal, "goal distance [m]", ImVec4(0.96f, 0.66f, 0.28f, 1.0f));
                    render_plot_hover_overlay("Visible Gates / Goal", time, {
                                                                             {&visible_gates, "visible gates", ImVec4(0.36f, 0.73f, 0.98f, 1.0f)},
                                                                             {&dist_goal, "goal distance [m]", ImVec4(0.96f, 0.66f, 0.28f, 1.0f)},
                                                                         });
                    ImPlot::EndPlot();
                }
                if (ImPlot::BeginPlot("LiDAR Clearance")) {
                    setup_time_plot_axes(time, "distance [m]");
                    plot_series(time, min_lidar, "LiDAR min [m]", ImVec4(0.48f, 0.87f, 0.60f, 1.0f));
                    plot_series(time, front_lidar, "LiDAR front [m]", ImVec4(0.96f, 0.66f, 0.28f, 1.0f));
                    render_plot_hover_overlay("LiDAR Clearance", time, {
                                                                           {&min_lidar, "LiDAR min [m]", ImVec4(0.48f, 0.87f, 0.60f, 1.0f)},
                                                                           {&front_lidar, "LiDAR front [m]", ImVec4(0.96f, 0.66f, 0.28f, 1.0f)},
                                                                       });
                    ImPlot::EndPlot();
                }
                ImPlot::EndSubplots();
            }
            ImGui::EndTabItem();
        }

        ImGui::EndTabBar();
    }
    ImGui::EndChild();
}

void render_run_quality_cards(const RunQualitySummary& quality, const char* id_prefix) {
    char runtime_buf[32];
    char clearance_buf[32];
    char tracking_buf[32];
    char compute_buf[32];
    std::snprintf(runtime_buf, sizeof(runtime_buf), "%.1f s", quality.duration_s);
    std::snprintf(clearance_buf, sizeof(clearance_buf), "%.2f m", quality.min_lidar);
    std::snprintf(tracking_buf, sizeof(tracking_buf), "%.2f m", quality.max_cross_track);
    std::snprintf(compute_buf, sizeof(compute_buf), "%.1f / %.1f ms", quality.avg_step_ms, quality.max_step_ms);

    if (ImGui::BeginTable((std::string(id_prefix) + "_quality_cards").c_str(), 4, ImGuiTableFlags_SizingStretchSame)) {
        ImGui::TableNextColumn();
        metric_card((std::string(id_prefix) + "_runtime").c_str(), "Runtime", runtime_buf, "Run duration", ImVec4(0.43f, 0.82f, 0.96f, 1.0f), 56.0f);
        ImGui::TableNextColumn();
        metric_card((std::string(id_prefix) + "_clearance").c_str(), "Clearance", clearance_buf, "Minimum LiDAR distance", ImVec4(0.48f, 0.88f, 0.62f, 1.0f), 56.0f);
        ImGui::TableNextColumn();
        metric_card((std::string(id_prefix) + "_tracking").c_str(), "Max CTE", tracking_buf, "Worst cross-track error", ImVec4(0.97f, 0.89f, 0.45f, 1.0f), 56.0f);
        ImGui::TableNextColumn();
        metric_card((std::string(id_prefix) + "_compute").c_str(), "Step ms", compute_buf, "Average / max", ImVec4(0.96f, 0.66f, 0.28f, 1.0f), 56.0f);
        ImGui::EndTable();
    }
}

void render_sim_mission_readiness(const PlannerDrivenVehicleSim& sim) {
    const RunQualitySummary quality = summarize_run_quality(sim.history());
    ImGui::SeparatorText("Mission Readiness");
    if (!quality.has_data) {
        ImGui::TextDisabled("Run or step the simulation to populate validation metrics.");
    } else {
        render_run_quality_cards(quality, "sim_ready");
    }
    const bool tank = sim.vehicle_model_kind() == VehicleModelKind::TrackedVehicle;
    status_line("Vehicle model",
                thesis_sim::vehicle_model_kind_name(sim.vehicle_model_kind()),
                tank ? ImVec4(0.96f, 0.66f, 0.28f, 1.0f) : ImVec4(0.43f, 0.82f, 0.96f, 1.0f));
    status_line("Planner primitive",
                tank ? "j + tracked yaw r" : "j + bicycle curvature r",
                tank ? ImVec4(0.96f, 0.66f, 0.28f, 1.0f) : ImVec4(0.48f, 0.88f, 0.62f, 1.0f));
    status_line("MPC layer",
                thesis_sim::tracking_controller_mode_name(sim.tracking_controller_mode()),
                ImVec4(0.48f, 0.88f, 0.62f, 1.0f));
    status_line("Dynamic gates",
                sim.config().dynamic_lidar_gates ? "enabled" : "disabled",
                sim.config().dynamic_lidar_gates ? ImVec4(0.48f, 0.88f, 0.62f, 1.0f)
                                                  : ImVec4(0.87f, 0.79f, 0.39f, 1.0f));
}

void render_validation_preset_grid(PlannerDrivenVehicleSim& sim, UiState* ui_state) {
    if (ui_state == nullptr) {
        return;
    }
    ImGui::SeparatorText("Validation Presets");
    const auto& presets = validation_presets();
    if (ImGui::BeginTable("ValidationPresetGrid", 2, ImGuiTableFlags_SizingStretchSame | ImGuiTableFlags_BordersInnerV)) {
        for (size_t i = 0; i < presets.size(); ++i) {
            const ValidationPreset& preset = presets[i];
            ImGui::TableNextColumn();
            ImGui::PushID(static_cast<int>(i));
            const bool current =
                sim.environment_mode() == preset.environment &&
                sim.vehicle_model_kind() == preset.vehicle &&
                sim.config().dynamic_lidar_gates == preset.dynamic_gates &&
                sim.config().ideal_conditions == preset.ideal &&
                (preset.environment != EnvironmentMode::MixedRoadGates || ui_state->mixed_preset == preset.mixed_preset);
            if (current) {
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.15f, 0.43f, 0.52f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.22f, 0.56f, 0.66f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.11f, 0.33f, 0.40f, 1.0f));
            }
            if (ImGui::Button(preset.name, ImVec2(-1.0f, 0.0f))) {
                apply_validation_preset(sim, ui_state, preset);
            }
            if (current) {
                ImGui::PopStyleColor(3);
            }
            ImGui::TextWrapped("%s", preset.detail);
            ImGui::PopID();
        }
        ImGui::EndTable();
    }
    if (ImGui::Button("Run Bicycle / Tank Benchmark", ImVec2(-1.0f, 0.0f))) {
        ui_state->last_validation_benchmark = run_vehicle_benchmark(sim, *ui_state);
    }
    if (!ui_state->last_validation_benchmark.empty()) {
        ImGui::TextWrapped("%s", ui_state->last_validation_benchmark.c_str());
    }
}

void render_hardware_readiness_panel(const HardwareViewerState& hardware,
                                     const UiState& ui_state,
                                     const LiveViewStreamServer& hardware_server) {
    ImGui::SeparatorText("Hardware Readiness");
    const VehicleModelKind configured_model =
        static_cast<VehicleModelKind>(ui_state.hardware_vehicle_model);
    const bool tank_configured = configured_model == VehicleModelKind::TrackedVehicle;
    const bool live_tank =
        hardware.has_scene &&
        vehicle_model_from_live_name(hardware.scene.vehicle_model_name) == VehicleModelKind::TrackedVehicle;
    status_line("Selected robot",
                thesis_sim::vehicle_model_kind_name(configured_model),
                tank_configured ? ImVec4(0.96f, 0.66f, 0.28f, 1.0f)
                                : ImVec4(0.43f, 0.82f, 0.96f, 1.0f));
    status_line("Runner model",
                hardware.has_scene ? hardware.scene.vehicle_model_name.c_str() : "waiting scene",
                hardware.has_scene
                    ? (live_tank == tank_configured ? ImVec4(0.48f, 0.88f, 0.62f, 1.0f)
                                                    : ImVec4(0.95f, 0.60f, 0.34f, 1.0f))
                    : ImVec4(0.87f, 0.79f, 0.39f, 1.0f));
    status_line("Profile sync",
                hardware_server.has_pending_robot_profile() ? "queued" : "idle",
                hardware_server.has_pending_robot_profile() ? ImVec4(0.43f, 0.82f, 0.96f, 1.0f)
                                                            : ImVec4(0.48f, 0.88f, 0.62f, 1.0f));
    status_line("Map sync",
                hardware_server.has_pending_world() ? "queued" : "idle",
                hardware_server.has_pending_world() ? ImVec4(0.43f, 0.82f, 0.96f, 1.0f)
                                                    : ImVec4(0.48f, 0.88f, 0.62f, 1.0f));
    status_line("Connection",
                hardware_server.connected() ? "connected" : (hardware_server.listening() ? "listening" : "off"),
                hardware_server.connected() ? ImVec4(0.48f, 0.88f, 0.62f, 1.0f)
                                            : (hardware_server.listening() ? ImVec4(0.87f, 0.79f, 0.39f, 1.0f)
                                                                          : ImVec4(0.95f, 0.60f, 0.34f, 1.0f)));
    if (tank_configured) {
        ImGui::TextWrapped("Tracked hardware path: GUI robot profile -> live stream packet -> runner apply_config -> Sabrina unicycle r primitive -> direct yaw-rate command -> wheel-speed/PWM mixer.");
    }
}

void render_editor_inspector(WorldMap& editor_world, UiState* ui_state, bool* dirty_flag) {
    if (ui_state == nullptr || dirty_flag == nullptr) {
        return;
    }

    ImGui::SeparatorText("Inspector");
    const MapEditorHandle handle = ui_state->selected_editor_handle;
    if (handle.type == MapEditorHandleType::None) {
        ImGui::TextDisabled("Select a handle in the viewport or add a map element.");
    } else {
        ImGui::Text("Selected = %s", map_editor_handle_type_name(handle.type));
    }

    auto edit_vec2 = [&](const char* label, Vec2* value) {
        if (value == nullptr) {
            return false;
        }
        bool changed = false;
        ImGui::PushID(label);
        ImGui::TextDisabled("%s", label);
        ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x * 0.48f);
        changed |= ImGui::InputDouble("x", &value->x, 0.01, 0.10, "%.3f");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x);
        changed |= ImGui::InputDouble("y", &value->y, 0.01, 0.10, "%.3f");
        ImGui::PopID();
        if (changed) {
            *dirty_flag = true;
        }
        return changed;
    };

    switch (handle.type) {
        case MapEditorHandleType::Start: {
            Vec2 start = editor_world.start();
            if (edit_vec2("Start", &start)) {
                editor_world.set_start(start);
            }
            break;
        }
        case MapEditorHandleType::Goal: {
            Vec2 goal = editor_world.goal();
            if (edit_vec2("Goal", &goal)) {
                editor_world.set_goal(goal);
            }
            break;
        }
        case MapEditorHandleType::Obstacle:
            if (handle.index >= 0 &&
                handle.index < static_cast<int>(editor_world.editable_obstacles().size())) {
                Rect& obstacle = editor_world.editable_obstacles()[handle.index];
                Vec2 center{
                    0.5 * (obstacle.min_x + obstacle.max_x),
                    0.5 * (obstacle.min_y + obstacle.max_y),
                };
                double size[2] = {
                    obstacle.max_x - obstacle.min_x,
                    obstacle.max_y - obstacle.min_y,
                };
                bool changed = edit_vec2("Center", &center);
                ImGui::TextDisabled("Size");
                ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x * 0.48f);
                changed |= ImGui::InputDouble("w##ObstacleSize", &size[0], 0.01, 0.10, "%.3f");
                ImGui::SameLine();
                ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x);
                changed |= ImGui::InputDouble("h##ObstacleSize", &size[1], 0.01, 0.10, "%.3f");
                size[0] = std::max(size[0], 0.05);
                size[1] = std::max(size[1], 0.05);
                if (changed) {
                    obstacle.min_x = center.x - 0.5 * size[0];
                    obstacle.max_x = center.x + 0.5 * size[0];
                    obstacle.min_y = center.y - 0.5 * size[1];
                    obstacle.max_y = center.y + 0.5 * size[1];
                    *dirty_flag = true;
                }
            }
            break;
        case MapEditorHandleType::Gate:
            if (handle.index >= 0 &&
                handle.index < static_cast<int>(editor_world.editable_gates().size())) {
                GateSpec& gate = editor_world.editable_gates()[handle.index];
                Vec2 position = gate.anchor_position;
                if (edit_vec2("Gate", &position)) {
                    gate.anchor_position = position;
                    gate.position = position;
                }
                double heading_deg = gate.heading_hint * 180.0 / 3.14159265358979323846;
                if (ImGui::InputDouble("Heading deg", &heading_deg, 1.0, 10.0, "%.2f")) {
                    gate.heading_hint = heading_deg * 3.14159265358979323846 / 180.0;
                    *dirty_flag = true;
                }
            }
            break;
        case MapEditorHandleType::RoadPoint:
            if (handle.index >= 0 &&
                handle.index < static_cast<int>(editor_world.editable_road_centerline().size())) {
                edit_vec2("Road point", &editor_world.editable_road_centerline()[handle.index]);
            }
            break;
        default:
            break;
    }

    const Rect bounds = editor_world.bounds();
    char size_buf[64];
    std::snprintf(size_buf,
                  sizeof(size_buf),
                  "%.2f x %.2f m",
                  bounds.max_x - bounds.min_x,
                  bounds.max_y - bounds.min_y);
    status_line("Map size", size_buf, ImVec4(0.43f, 0.82f, 0.96f, 1.0f));
    status_line("Dirty",
                *dirty_flag ? "unapplied changes" : "synchronized",
                *dirty_flag ? ImVec4(0.95f, 0.60f, 0.34f, 1.0f)
                            : ImVec4(0.48f, 0.88f, 0.62f, 1.0f));
}

void render_hardware_control_panel(const HardwareViewerState& hardware,
                                   UiState* ui_state,
                                   LiveViewStreamServer* hardware_server) {
    if (ui_state == nullptr || hardware_server == nullptr) {
        return;
    }

    if (!ImGui::BeginChild("HardwareConfigPanel", ImVec2(0.0f, 0.0f), true)) {
        ImGui::EndChild();
        return;
    }

    const std::string configured_profile = "Planner";
    const std::string live_profile =
        hardware.has_scene ? stream_profile_label(hardware.scene.stream_profile)
                           : configured_profile;
    const bool profile_mismatch =
        hardware.has_scene && !hardware.scene.stream_profile.empty() &&
        hardware.scene.stream_profile != "planner";

    const char* status = hardware_server->connected()
                             ? (hardware.frame.goal_reached ? "Goal reached" : (hardware.frame.safety_stop_active ? "Safety stop" : "Live"))
                             : (hardware_server->listening() ? "Listening" : "Idle");
    const ImVec4 status_color = hardware_server->connected()
                                    ? (hardware.frame.goal_reached ? ImVec4(0.45f, 0.86f, 0.53f, 1.0f)
                                                                   : (hardware.frame.safety_stop_active ? ImVec4(0.95f, 0.40f, 0.34f, 1.0f)
                                                                                                       : ImVec4(0.87f, 0.79f, 0.39f, 1.0f)))
                                    : (hardware_server->listening() ? ImVec4(0.43f, 0.82f, 0.96f, 1.0f)
                                                                    : ImVec4(0.74f, 0.79f, 0.84f, 1.0f));
    char time_buf[32];
    char goal_buf[32];
    char speed_buf[32];
    char tracking_buf[48];
    const std::string goal_label =
        hardware_goal_distance_label(hardware.frame, hardware.scene.world.environment_mode());
    std::snprintf(time_buf, sizeof(time_buf), "%.1f s", hardware.frame.sim_time);
    std::snprintf(goal_buf, sizeof(goal_buf), "%s", goal_label.c_str());
    std::snprintf(speed_buf, sizeof(speed_buf), "%.2f m/s", hardware.frame.vehicle.speed);
    std::snprintf(tracking_buf, sizeof(tracking_buf), "%.2f m", hardware.frame.tracker_cross_track_error);

    ImGui::TextUnformatted("Hardware Desk");
    ImGui::TextColored(status_color, "%s", status);
    ImGui::TextDisabled("configured %s | incoming %s",
                        configured_profile.c_str(),
                        live_profile.c_str());
    if (profile_mismatch) {
        ImGui::TextColored(ImVec4(0.95f, 0.60f, 0.34f, 1.0f),
                           "This UI expects the planner stream, but the current stream is %s.",
                           live_profile.c_str());
    }

    const bool map_sync_queued =
        ui_state->hardware_world_sync_pending ||
        hardware_server->has_pending_world() ||
        hardware_server->has_pending_robot_profile();
    const char* robot_state = hardware_server->connected()
                                  ? "connected"
                                  : (hardware_server->listening() ? "waiting" : "listener off");
    const ImVec4 robot_color = hardware_server->connected()
                                   ? ImVec4(0.48f, 0.88f, 0.62f, 1.0f)
                                   : (hardware_server->listening() ? ImVec4(0.43f, 0.82f, 0.96f, 1.0f)
                                                                   : ImVec4(0.74f, 0.79f, 0.84f, 1.0f));
    const char* map_state = ui_state->hardware_editor_dirty
                                ? "edited locally"
                                : (map_sync_queued ? "sync queued"
                                                   : (ui_state->hardware_world_sync_ok ? "synced" : "ready"));
    const ImVec4 map_color = ui_state->hardware_editor_dirty
                                 ? ImVec4(0.95f, 0.60f, 0.34f, 1.0f)
                                 : (map_sync_queued ? ImVec4(0.43f, 0.82f, 0.96f, 1.0f)
                                                    : ImVec4(0.48f, 0.88f, 0.62f, 1.0f));
    const char* planner_state = hardware.frame.safety_stop_active
                                    ? "safety stop"
                                    : (hardware.frame.planner_has_reference
                                           ? "reference ready"
                                           : (hardware_server->connected() ? "waiting reference" : "preview"));
    const ImVec4 planner_color = hardware.frame.safety_stop_active
                                     ? ImVec4(0.95f, 0.40f, 0.34f, 1.0f)
                                     : (hardware.frame.planner_has_reference ? ImVec4(0.48f, 0.88f, 0.62f, 1.0f)
                                                                            : ImVec4(0.87f, 0.79f, 0.39f, 1.0f));

    ImGui::SeparatorText("Demo State");
    status_line("Robot", robot_state, robot_color);
    status_line("Map", map_state, map_color);
    status_line("Planner", planner_state, planner_color);
    status_line("Mode", live_profile.c_str(), status_color);

    if (ImGui::BeginTable("HardwareHeroMetrics", 2, ImGuiTableFlags_SizingStretchSame)) {
        ImGui::TableNextColumn();
        metric_card("hw_control_time", "Runtime", time_buf, "", ImVec4(0.43f, 0.82f, 0.96f, 1.0f), 56.0f);
        ImGui::TableNextColumn();
        metric_card("hw_control_goal", "Goal", goal_buf, "", ImVec4(0.99f, 0.70f, 0.32f, 1.0f), 56.0f);
        ImGui::TableNextColumn();
        metric_card("hw_control_speed", "Speed", speed_buf, "", ImVec4(0.48f, 0.88f, 0.62f, 1.0f), 56.0f);
        ImGui::TableNextColumn();
        metric_card("hw_control_tracking", "Tracking", tracking_buf, "", ImVec4(0.97f, 0.89f, 0.45f, 1.0f), 56.0f);
        ImGui::EndTable();
    }

    render_hardware_readiness_panel(hardware, *ui_state, *hardware_server);

    if (ImGui::BeginTabBar("HardwareDeskTabs")) {
        if (ImGui::BeginTabItem("Setup", nullptr, requested_tab_flags(ui_state->requested_hardware_panel_tab, 0))) {
            activate_hardware_panel_tab(ui_state, 0, kWorkspaceViewMission, false);
            consume_requested_tab(&ui_state->requested_hardware_panel_tab, 0);
            ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
            if (ImGui::CollapsingHeader("Demo Setup", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::InputInt("Listen Port", &ui_state->hardware_listen_port);
        ui_state->hardware_listen_port = std::max(ui_state->hardware_listen_port, 1);

        const char* hardware_vehicle_items[] = {
            "Bicycle support",
            "Tank support",
        };
        int hardware_vehicle_selection =
            ui_state->hardware_vehicle_model == static_cast<int>(VehicleModelKind::TrackedVehicle) ? 1 : 0;
        if (ImGui::Combo("Robot Support", &hardware_vehicle_selection, hardware_vehicle_items, IM_ARRAYSIZE(hardware_vehicle_items))) {
            const VehicleModelKind next_model =
                hardware_vehicle_selection == 1 ? VehicleModelKind::TrackedVehicle : VehicleModelKind::CarLikeBicycle;
            queue_hardware_robot_profile(
                ui_state,
                hardware_server,
                next_model,
                hardware_server->connected()
                    ? "Robot support sent to the connected Raspberry runner. Waiting for runner ack."
                    : "Robot support queued in the GUI. It will be sent automatically to the next Raspberry runner that connects.",
                "Could not queue the robot support profile for Raspberry sync.");
            if (static_cast<EnvironmentMode>(ui_state->hardware_environment_mode) == EnvironmentMode::StructuredRoad) {
                load_hardware_editor_from_selection(ui_state);
                queue_current_hardware_world(
                    ui_state,
                    hardware_server,
                    "Structured map rescaled for the selected robot and queued for the Raspberry runner.",
                    "Robot support changed locally, but the rescaled structured map could not be queued.");
            }
        }
        if (static_cast<VehicleModelKind>(ui_state->hardware_vehicle_model) == VehicleModelKind::TrackedVehicle) {
            ImGui::TextWrapped("Tank support uses Sabrina's angular primitive as the tracked yaw-rate reference while keeping the MPC speed layer active.");
        }

        const char* environment_items[] = {
            thesis_sim::environment_mode_name(EnvironmentMode::UnstructuredGates),
            thesis_sim::environment_mode_name(EnvironmentMode::StructuredRoad),
            thesis_sim::environment_mode_name(EnvironmentMode::MixedRoadGates),
        };
        int hardware_environment = std::clamp(
            ui_state->hardware_environment_mode,
            static_cast<int>(EnvironmentMode::UnstructuredGates),
            static_cast<int>(EnvironmentMode::MixedRoadGates));
        const char* environment_label = "Planner Scenario";
        if (ImGui::Combo(environment_label, &hardware_environment, environment_items, IM_ARRAYSIZE(environment_items))) {
            ui_state->hardware_environment_mode = hardware_environment;
            load_hardware_editor_from_selection(ui_state);
            queue_current_hardware_world(
                ui_state,
                hardware_server,
                "Scenario map queued for the Raspberry runner.",
                "Scenario changed locally, but the map could not be queued for the Raspberry runner.");
        }

        if (static_cast<EnvironmentMode>(ui_state->hardware_environment_mode) == EnvironmentMode::StructuredRoad) {
            const StructuredMapPreset previous_preset =
                static_cast<StructuredMapPreset>(ui_state->hardware_structured_preset);
            const char* structured_items[] = {
                thesis_sim::structured_map_preset_name(StructuredMapPreset::ValidationRoad),
                thesis_sim::structured_map_preset_name(StructuredMapPreset::CircleLoop),
                thesis_sim::structured_map_preset_name(StructuredMapPreset::ZigZag),
                thesis_sim::structured_map_preset_name(StructuredMapPreset::HardwareTrack),
                thesis_sim::structured_map_preset_name(StructuredMapPreset::FigureEight),
                thesis_sim::structured_map_preset_name(StructuredMapPreset::TankCircuit),
                thesis_sim::structured_map_preset_name(StructuredMapPreset::Custom),
            };
            int selection = 0;
            switch (previous_preset) {
                case StructuredMapPreset::ValidationRoad:
                    selection = 0;
                    break;
                case StructuredMapPreset::CircleLoop:
                    selection = 1;
                    break;
                case StructuredMapPreset::ZigZag:
                    selection = 2;
                    break;
                case StructuredMapPreset::HardwareTrack:
                    selection = 3;
                    break;
                case StructuredMapPreset::FigureEight:
                    selection = 4;
                    break;
                case StructuredMapPreset::TankCircuit:
                    selection = 5;
                    break;
                case StructuredMapPreset::Custom:
                    selection = 6;
                    break;
                default:
                    selection = 0;
                    break;
            }
            if (ImGui::Combo("Structured Map", &selection, structured_items, IM_ARRAYSIZE(structured_items))) {
                StructuredMapPreset next_preset = StructuredMapPreset::ValidationRoad;
                switch (selection) {
                    case 0:
                        next_preset = StructuredMapPreset::ValidationRoad;
                        break;
                    case 1:
                        next_preset = StructuredMapPreset::CircleLoop;
                        break;
                    case 2:
                        next_preset = StructuredMapPreset::ZigZag;
                        break;
                    case 3:
                        next_preset = StructuredMapPreset::HardwareTrack;
                        break;
                    case 4:
                        next_preset = StructuredMapPreset::FigureEight;
                        break;
                    case 5:
                        next_preset = StructuredMapPreset::TankCircuit;
                        break;
                    case 6:
                        next_preset = StructuredMapPreset::Custom;
                        break;
                    default:
                        next_preset = StructuredMapPreset::ValidationRoad;
                        break;
                }
                if (next_preset == StructuredMapPreset::Custom &&
                    ui_state->hardware_editor_world.environment_mode() != EnvironmentMode::StructuredRoad) {
                    ui_state->hardware_editor_world = make_world_from_mode(
                        EnvironmentMode::StructuredRoad,
                        UnstructuredMapPreset::Custom,
                        previous_preset == StructuredMapPreset::Custom ? StructuredMapPreset::ValidationRoad : previous_preset,
                        GateBehaviorMode::Static,
                        0);
                    ui_state->hardware_editor_dirty = false;
                    reset_editor_interaction(ui_state);
                }
                ui_state->hardware_structured_preset = static_cast<int>(next_preset);
                if (next_preset != StructuredMapPreset::Custom) {
                    load_hardware_editor_from_selection(ui_state);
                    queue_current_hardware_world(
                        ui_state,
                        hardware_server,
                        "Structured preset queued for the Raspberry runner.",
                        "Structured preset restored locally, but it could not be queued for the Raspberry runner.");
                }
            }
        } else {
            const UnstructuredMapPreset previous_preset =
                static_cast<UnstructuredMapPreset>(ui_state->hardware_unstructured_preset);
            const char* unstructured_items[] = {
                thesis_sim::unstructured_map_preset_name(UnstructuredMapPreset::Custom),
            };
            int selection = 0;
            switch (previous_preset) {
                case UnstructuredMapPreset::Custom:
                    selection = 0;
                    break;
                default:
                    selection = 0;
                    break;
            }
            if (ImGui::Combo("Unstructured Map", &selection, unstructured_items, IM_ARRAYSIZE(unstructured_items))) {
                UnstructuredMapPreset next_preset = UnstructuredMapPreset::Custom;
                switch (selection) {
                    case 0:
                        next_preset = UnstructuredMapPreset::Custom;
                        break;
                    default:
                        next_preset = UnstructuredMapPreset::Custom;
                        break;
                }
                if (next_preset == UnstructuredMapPreset::Custom &&
                    ui_state->hardware_editor_world.environment_mode() != EnvironmentMode::UnstructuredGates) {
                    ui_state->hardware_editor_world = make_world_from_mode(
                        EnvironmentMode::UnstructuredGates,
                        previous_preset == UnstructuredMapPreset::Custom ? UnstructuredMapPreset::Custom : previous_preset,
                        StructuredMapPreset::HardwareTrack,
                        GateBehaviorMode::Static,
                        0);
                    ui_state->hardware_editor_dirty = false;
                    reset_editor_interaction(ui_state);
                }
                ui_state->hardware_unstructured_preset = static_cast<int>(next_preset);
                if (next_preset != UnstructuredMapPreset::Custom) {
                    load_hardware_editor_from_selection(ui_state);
                    queue_current_hardware_world(
                        ui_state,
                        hardware_server,
                        "Unstructured preset queued for the Raspberry runner.",
                        "Unstructured preset restored locally, but it could not be queued for the Raspberry runner.");
                }
            }
        }

        ImGui::SeparatorText("Connection");
        if (ImGui::BeginTable("HardwareListenerButtons", 2, ImGuiTableFlags_SizingStretchSame)) {
            ImGui::TableNextColumn();
            if (ImGui::Button(hardware_server->listening() ? "Restart Listener" : "Start Listener", ImVec2(-1.0f, 0.0f))) {
                hardware_server->start(static_cast<std::uint16_t>(ui_state->hardware_listen_port));
            }
            ImGui::TableNextColumn();
            if (!hardware_server->listening()) {
                ImGui::BeginDisabled();
            }
            if (ImGui::Button("Stop Listener", ImVec2(-1.0f, 0.0f))) {
                hardware_server->stop();
            }
            if (!hardware_server->listening()) {
                ImGui::EndDisabled();
            }
            ImGui::EndTable();
        }

        status_line("Listener", hardware_server->listening() ? "active" : "off", robot_color);
        if (hardware_server->listening()) {
            ImGui::Text("Port = %u", hardware_server->port());
        }
        if (hardware_server->connected()) {
            ImGui::Text("Remote = %s", hardware_server->remote_endpoint().c_str());
        } else {
            ImGui::TextDisabled("Remote = waiting for connection");
        }
        if (!hardware_server->last_error().empty()) {
            ImGui::TextWrapped("Last stream error: %s", hardware_server->last_error().c_str());
        }
    }

            ImGui::EndTabItem();
        }

        if (ImGui::BeginTabItem("Launch", nullptr, requested_tab_flags(ui_state->requested_hardware_panel_tab, 1))) {
            activate_hardware_panel_tab(ui_state, 1, kWorkspaceViewExport, false);
            consume_requested_tab(&ui_state->requested_hardware_panel_tab, 1);
            ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
            if (ImGui::CollapsingHeader("Raspberry Launch", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::TextWrapped("Launch on the Raspberry with:");
        ImGui::TextWrapped("%s", hardware_launch_hint(*ui_state).c_str());
        const bool custom_world_selected =
            static_cast<EnvironmentMode>(ui_state->hardware_environment_mode) == EnvironmentMode::StructuredRoad
                ? static_cast<StructuredMapPreset>(ui_state->hardware_structured_preset) == StructuredMapPreset::Custom
                : static_cast<UnstructuredMapPreset>(ui_state->hardware_unstructured_preset) == UnstructuredMapPreset::Custom;
        if (custom_world_selected) {
            if (ui_state->hardware_world_sync_pending || hardware_server->has_pending_world()) {
                ImGui::TextColored(ImVec4(0.43f, 0.82f, 0.96f, 1.0f),
                                   "Custom map sync is queued in the GUI and will be streamed automatically to the Raspberry runner on connect.");
            } else {
                ImGui::TextColored(ImVec4(0.48f, 0.88f, 0.62f, 1.0f),
                                   "The current hardware world will be streamed automatically to the Raspberry runner as soon as it connects.");
            }
        }
        ImGui::TextWrapped("If you want to transport it over SSH, expose this same port with a reverse tunnel.");
    }

            ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
            if (ImGui::CollapsingHeader("Telemetry Capture", ImGuiTreeNodeFlags_DefaultOpen)) {
        const bool can_capture =
            hardware.has_scene ||
            !hardware.history.empty() ||
            hardware.frame.connected ||
            hardware.frame.step_count > 0;
        ImGui::TextWrapped("Save a full hardware debug snapshot on demand. The file includes the current frame, controller flags, trajectory, LiDAR hits/map points, visible gates and the full workstation-side telemetry history.");
        if (!can_capture) {
            ImGui::BeginDisabled();
        }
        if (ImGui::Button("Save Hardware Telemetry", ImVec2(-1.0f, 0.0f))) {
            const std::string report_path = default_hardware_report_path(hardware, *ui_state, "gui_manual");
            if (write_hardware_json_report(hardware, *ui_state, *hardware_server, report_path)) {
                ui_state->hardware_report_written = true;
                ui_state->last_hardware_report_path = report_path;
                ui_state->last_hardware_csv_report_path.clear();
                ui_state->last_hardware_markdown_report_path.clear();
                ui_state->last_hardware_report_error.clear();
            } else {
                ui_state->hardware_report_written = false;
                ui_state->last_hardware_report_error = "Could not write the hardware telemetry report.";
            }
        }
        if (ImGui::Button("Save Hardware Paper Bundle", ImVec2(-1.0f, 0.0f))) {
            const std::string json_path = default_hardware_report_path(hardware, *ui_state, "gui_bundle");
            const std::string csv_path = replace_extension(json_path, ".csv");
            const std::string markdown_path = replace_extension(json_path, ".md");
            const bool json_ok = write_hardware_json_report(hardware, *ui_state, *hardware_server, json_path);
            const bool csv_ok = write_hardware_csv_report(hardware, csv_path);
            const bool markdown_ok = write_hardware_markdown_summary(hardware, *ui_state, *hardware_server, markdown_path);
            if (json_ok && csv_ok && markdown_ok) {
                ui_state->hardware_report_written = true;
                ui_state->last_hardware_report_path = json_path;
                ui_state->last_hardware_csv_report_path = csv_path;
                ui_state->last_hardware_markdown_report_path = markdown_path;
                ui_state->last_hardware_report_error.clear();
            } else {
                ui_state->hardware_report_written = false;
                ui_state->last_hardware_report_error = "Could not write the complete hardware paper bundle.";
            }
        }
        if (!can_capture) {
            ImGui::EndDisabled();
            ImGui::TextDisabled("Waiting for live hardware data before capture.");
        }
        ImGui::Text("History samples = %d", static_cast<int>(hardware.history.size()));
        ImGui::Text("Current LiDAR hits = %d   map points = %d",
                    static_cast<int>(hardware.frame.lidar_hits.size()),
                    static_cast<int>(hardware.frame.slam_points.size()));
        if (!ui_state->last_hardware_report_path.empty()) {
            ImGui::TextWrapped("JSON: %s", ui_state->last_hardware_report_path.c_str());
        }
        if (!ui_state->last_hardware_csv_report_path.empty()) {
            ImGui::TextWrapped("CSV: %s", ui_state->last_hardware_csv_report_path.c_str());
        }
        if (!ui_state->last_hardware_markdown_report_path.empty()) {
            ImGui::TextWrapped("Markdown: %s", ui_state->last_hardware_markdown_report_path.c_str());
        }
        if (!ui_state->last_hardware_report_error.empty()) {
            ImGui::TextColored(ImVec4(0.95f, 0.40f, 0.34f, 1.0f), "%s", ui_state->last_hardware_report_error.c_str());
        }
    }

            ImGui::EndTabItem();
        }

        if (ImGui::BeginTabItem("Map", nullptr, requested_tab_flags(ui_state->requested_hardware_panel_tab, 2))) {
            activate_hardware_panel_tab(ui_state, 2, kWorkspaceViewMap, true);
            consume_requested_tab(&ui_state->requested_hardware_panel_tab, 2);
            ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
            if (ImGui::CollapsingHeader("Track Map", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::SameLine();
        help_marker("Preview and reshape the hardware map before syncing it to the Raspberry runner.");

        WorldMap& editor_world = ui_state->hardware_editor_world;
        const bool hardware_structured_mode =
            static_cast<EnvironmentMode>(ui_state->hardware_environment_mode) == EnvironmentMode::StructuredRoad;
        const bool hardware_track_selected =
            static_cast<StructuredMapPreset>(ui_state->hardware_structured_preset) == StructuredMapPreset::HardwareTrack;
        const Rect editor_bounds = editor_world.bounds();
        char editor_size_buf[64];
        char road_points_buf[32];
        std::snprintf(editor_size_buf,
                      sizeof(editor_size_buf),
                      "%.2f x %.2f m",
                      editor_bounds.max_x - editor_bounds.min_x,
                      editor_bounds.max_y - editor_bounds.min_y);
        std::snprintf(road_points_buf,
                      sizeof(road_points_buf),
                      "%d",
                      static_cast<int>(editor_world.road_centerline().size()));
        const auto apply_editor_world = [&]() -> bool {
            const WorldMap previous_world = ui_state->hardware_editor_world;
            const bool previous_dirty = ui_state->hardware_editor_dirty;
            WorldMap applied_world = ui_state->hardware_editor_world;
            applied_world.finalize_editor_changes();
            applied_world = sanitize_hardware_unstructured_world(std::move(applied_world));
            applied_world = fit_hardware_structured_world(
                std::move(applied_world),
                static_cast<VehicleModelKind>(ui_state->hardware_vehicle_model));
            std::string validation_error;
            if (!validate_hardware_world(applied_world, &validation_error)) {
                ui_state->hardware_editor_world = previous_world;
                ui_state->hardware_editor_dirty = previous_dirty;
                ui_state->last_hardware_world_error = validation_error;
                ui_state->last_hardware_world_sync_status = validation_error;
                return false;
            }

            ui_state->hardware_editor_world = applied_world;
            ui_state->hardware_editor_dirty = false;
            ui_state->last_hardware_world_error.clear();
            if (applied_world.environment_mode() == EnvironmentMode::StructuredRoad) {
                ui_state->hardware_environment_mode = static_cast<int>(EnvironmentMode::StructuredRoad);
                ui_state->hardware_structured_preset = static_cast<int>(StructuredMapPreset::Custom);
            } else if (applied_world.environment_mode() == EnvironmentMode::MixedRoadGates) {
                ui_state->hardware_environment_mode = static_cast<int>(EnvironmentMode::MixedRoadGates);
            } else {
                ui_state->hardware_environment_mode = static_cast<int>(EnvironmentMode::UnstructuredGates);
                ui_state->hardware_unstructured_preset = static_cast<int>(UnstructuredMapPreset::Custom);
            }
            return true;
        };
        if (hardware_structured_mode && hardware_track_selected) {
            float track_scale = ui_state->hardware_track_scale;
            if (ImGui::SliderFloat("Track Scale", &track_scale, kHardwareTrackMinScale, kHardwareTrackMaxScale, "%.2fx")) {
                ui_state->hardware_track_scale = track_scale;
                load_hardware_editor_from_selection(ui_state);
                queue_current_hardware_world(
                    ui_state,
                    hardware_server,
                    "Hardware Track scale queued for the Raspberry runner.",
                    "Hardware Track scale updated locally, but it could not be queued for the Raspberry runner.");
            }
        }
        ImGui::Checkbox(hardware_structured_mode ? "Edit road by dragging" : "Edit map by dragging",
                        &ui_state->map_editor_enabled);

        if (ImGui::BeginTable("HardwareMapActions", 2, ImGuiTableFlags_SizingStretchSame)) {
            ImGui::TableNextColumn();
            const bool hardware_mixed_mode =
                editor_world.environment_mode() == EnvironmentMode::MixedRoadGates ||
                static_cast<EnvironmentMode>(ui_state->hardware_environment_mode) == EnvironmentMode::MixedRoadGates;
            const bool can_reset_track = hardware_structured_mode || hardware_mixed_mode;
            if (!can_reset_track) {
                ImGui::BeginDisabled();
            }
            const char* reset_label = hardware_mixed_mode ? "Reset Mixed" : "Reset Track";
            if (ImGui::Button(reset_label, ImVec2(-1.0f, 0.0f))) {
                if (hardware_mixed_mode) {
                    ui_state->hardware_environment_mode = static_cast<int>(EnvironmentMode::MixedRoadGates);
                } else {
                    ui_state->hardware_environment_mode = static_cast<int>(EnvironmentMode::StructuredRoad);
                    ui_state->hardware_structured_preset = static_cast<int>(StructuredMapPreset::HardwareTrack);
                }
                load_hardware_editor_from_selection(ui_state);
                queue_current_hardware_world(
                    ui_state,
                    hardware_server,
                    hardware_mixed_mode
                        ? "Mixed hardware road queued for the Raspberry runner."
                        : "Hardware Track queued for the Raspberry runner.",
                    hardware_mixed_mode
                        ? "Mixed hardware road restored locally, but it could not be queued for the Raspberry runner."
                        : "Hardware Track restored locally, but it could not be queued for the Raspberry runner.");
            }
            if (!can_reset_track) {
                ImGui::EndDisabled();
            }

            ImGui::TableNextColumn();
            const char* sync_label = ui_state->hardware_editor_dirty ? "Apply + Sync" : "Sync Map";
            if (ImGui::Button(sync_label, ImVec2(-1.0f, 0.0f))) {
                if (!ui_state->hardware_editor_dirty || apply_editor_world()) {
                    queue_current_hardware_world(
                        ui_state,
                        hardware_server,
                        hardware_server->connected()
                            ? "Current hardware world sent to the connected Raspberry runner. Waiting for runner ack."
                            : "Current hardware world queued in the GUI. It will be sent automatically to the next Raspberry runner that connects.",
                        "Could not queue the current hardware world for Raspberry sync.");
                }
            }
            ImGui::EndTable();
        }

        status_line("Track size", editor_size_buf, ImVec4(0.43f, 0.82f, 0.96f, 1.0f));
        status_line("Road points", road_points_buf, ImVec4(0.97f, 0.89f, 0.45f, 1.0f));
        status_line("Sync", map_state, map_color);

        if (ui_state->map_editor_enabled &&
            ImGui::CollapsingHeader("Edit Handles", ImGuiTreeNodeFlags_DefaultOpen)) {
            if (editor_world.environment_mode() == EnvironmentMode::UnstructuredGates) {
                if (ImGui::Button("Add Obstacle", ImVec2(-1.0f, 0.0f))) {
                    const Vec2 center{
                        0.5 * (editor_world.start().x + editor_world.goal().x),
                        0.5 * (editor_world.start().y + editor_world.goal().y),
                    };
                    editor_world.editable_obstacles().push_back({center.x - 1.0, center.y - 1.0, center.x + 1.0, center.y + 1.0});
                    ui_state->hardware_editor_dirty = true;
                    ui_state->selected_editor_handle = {MapEditorHandleType::Obstacle, static_cast<int>(editor_world.editable_obstacles().size()) - 1};
                }
                if (ImGui::Button("Add Gate", ImVec2(-1.0f, 0.0f))) {
                    const Vec2 midpoint{
                        0.5 * (editor_world.start().x + editor_world.goal().x),
                        0.5 * (editor_world.start().y + editor_world.goal().y),
                    };
                    GateSpec gate;
                    gate.name = "gate_" + std::to_string(editor_world.editable_gates().size() + 1);
                    gate.position = midpoint;
                    gate.anchor_position = midpoint;
                    editor_world.editable_gates().push_back(gate);
                    ui_state->hardware_editor_dirty = true;
                    ui_state->selected_editor_handle = {MapEditorHandleType::Gate, static_cast<int>(editor_world.editable_gates().size()) - 1};
                }
            } else {
                if (ImGui::Button("Add Road Point", ImVec2(-1.0f, 0.0f))) {
                    std::vector<Vec2>& road = editor_world.editable_road_centerline();
                    const bool closed_loop =
                        road.size() >= 3 && thesis_sim::distance(road.front(), road.back()) < 0.5;
                    if (closed_loop) {
                        road.insert(road.end() - 1, editor_world.goal());
                        ui_state->selected_editor_handle = {
                            MapEditorHandleType::RoadPoint,
                            static_cast<int>(road.size()) - 2,
                        };
                    } else {
                        road.push_back(editor_world.goal());
                        ui_state->selected_editor_handle = {
                            MapEditorHandleType::RoadPoint,
                            static_cast<int>(road.size()) - 1,
                        };
                    }
                    ui_state->hardware_editor_dirty = true;
                }
            }

            if (ui_state->selected_editor_handle.type != MapEditorHandleType::None) {
                ImGui::Text("Selected = %s", map_editor_handle_type_name(ui_state->selected_editor_handle.type));
                if (ImGui::Button("Remove Selected", ImVec2(-1.0f, 0.0f))) {
                    switch (ui_state->selected_editor_handle.type) {
                        case MapEditorHandleType::Obstacle:
                            if (ui_state->selected_editor_handle.index >= 0 &&
                                ui_state->selected_editor_handle.index < static_cast<int>(editor_world.editable_obstacles().size())) {
                                editor_world.editable_obstacles().erase(
                                    editor_world.editable_obstacles().begin() + ui_state->selected_editor_handle.index);
                                ui_state->hardware_editor_dirty = true;
                            }
                            break;
                        case MapEditorHandleType::Gate:
                            if (ui_state->selected_editor_handle.index >= 0 &&
                                ui_state->selected_editor_handle.index < static_cast<int>(editor_world.editable_gates().size())) {
                                editor_world.editable_gates().erase(
                                    editor_world.editable_gates().begin() + ui_state->selected_editor_handle.index);
                                ui_state->hardware_editor_dirty = true;
                            }
                            break;
                        case MapEditorHandleType::RoadPoint:
                            if (ui_state->selected_editor_handle.index > 0 &&
                                ui_state->selected_editor_handle.index + 1 < static_cast<int>(editor_world.editable_road_centerline().size())) {
                                editor_world.editable_road_centerline().erase(
                                    editor_world.editable_road_centerline().begin() + ui_state->selected_editor_handle.index);
                                ui_state->hardware_editor_dirty = true;
                            }
                            break;
                        default:
                            break;
                    }
                    reset_editor_interaction(ui_state);
                }
            } else {
                ImGui::TextDisabled("No editor handle selected.");
            }
        }

        render_editor_inspector(editor_world, ui_state, &ui_state->hardware_editor_dirty);

        if (!ui_state->last_hardware_world_path.empty()) {
            ImGui::TextWrapped("Last custom map: %s", ui_state->last_hardware_world_path.c_str());
        }
        if (!ui_state->last_hardware_world_error.empty()) {
            ImGui::TextColored(ImVec4(0.95f, 0.40f, 0.34f, 1.0f), "%s", ui_state->last_hardware_world_error.c_str());
        }
        if (!ui_state->last_hardware_world_sync_status.empty()) {
            const ImVec4 sync_color =
                ui_state->hardware_world_sync_pending
                    ? ImVec4(0.43f, 0.82f, 0.96f, 1.0f)
                    : (ui_state->hardware_world_sync_ok
                           ? ImVec4(0.48f, 0.88f, 0.62f, 1.0f)
                           : ImVec4(0.95f, 0.60f, 0.34f, 1.0f));
            ImGui::TextColored(sync_color, "%s", ui_state->last_hardware_world_sync_status.c_str());
        }
    }

            ImGui::EndTabItem();
        }

        if (ImGui::BeginTabItem("Diagnostics", nullptr, requested_tab_flags(ui_state->requested_hardware_panel_tab, 3))) {
            activate_hardware_panel_tab(ui_state, 3, kWorkspaceViewDiagnostics, false);
            consume_requested_tab(&ui_state->requested_hardware_panel_tab, 3);
            ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
            if (ImGui::CollapsingHeader("Sensors & Localization", ImGuiTreeNodeFlags_DefaultOpen)) {
        if (!hardware.has_scene) {
            ImGui::TextDisabled("Waiting for scene metadata...");
        } else {
            ImGui::Text("Stream profile = %s", live_profile.c_str());
            ImGui::Text("Localization = %s", hardware.scene.localization_mode.c_str());
            ImGui::Text("Heading source = %s", hardware.scene.heading_source.c_str());
            ImGui::Text("Stack = %s + %s",
                        hardware.scene.vehicle_model_name.c_str(),
                        hardware.scene.tracking_controller_name.c_str());
            ImGui::Text("LiDAR = %s, min %.2f m, front %.2f m",
                        hardware.scene.range_sensor_name.c_str(),
                        hardware.frame.min_lidar_distance,
                        hardware.frame.front_lidar_distance);
            ImGui::Text("LiDAR samples = %d valid, %d close, %d front-close",
                        hardware.frame.valid_lidar_samples,
                        hardware.frame.close_lidar_samples,
                        hardware.frame.front_close_lidar_samples);
            ImGui::Text("Telemetry ready = %s", hardware.frame.telemetry_ready ? "yes" : "no");
        }
    }

            ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
            if (ImGui::CollapsingHeader("Live Diagnostics", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Text("t = %.2f s   step = %d", hardware.frame.sim_time, hardware.frame.step_count);
        ImGui::Text("pos = (%.2f, %.2f) m", hardware.frame.navigation_position.x, hardware.frame.navigation_position.y);
        ImGui::Text("yaw = %.1f deg   v = %.2f m/s", hardware.frame.navigation_yaw * 180.0 / 3.14159265358979323846, hardware.frame.navigation_speed);
        ImGui::Text("goal distance = %s",
                    hardware_goal_distance_label(hardware.frame, hardware.scene.world.environment_mode()).c_str());
        ImGui::Text("tracking: cte %.2f m   hdg %.2f deg", hardware.frame.tracker_cross_track_error, hardware.frame.tracker_heading_error_deg);
        ImGui::Text("controller pwm = %d / %d   target = %d / %d",
                    hardware.frame.has_latest_sample ? static_cast<int>(hardware.frame.latest_sample.controller_pwm_left) : 0,
                    hardware.frame.has_latest_sample ? static_cast<int>(hardware.frame.latest_sample.controller_pwm_right) : 0,
                    hardware.frame.has_latest_sample ? static_cast<int>(hardware.frame.latest_sample.controller_target_pwm_left) : 0,
                    hardware.frame.has_latest_sample ? static_cast<int>(hardware.frame.latest_sample.controller_target_pwm_right) : 0);
        ImGui::Text("enc ticks = %d / %d   dTicks = %d / %d   dt = %.0f ms",
                    hardware.frame.vehicle.left_encoder_ticks,
                    hardware.frame.vehicle.right_encoder_ticks,
                    hardware.frame.vehicle.left_encoder_delta,
                    hardware.frame.vehicle.right_encoder_delta,
                    hardware.frame.vehicle.encoder_dt_ms);
        ImGui::Text("planner ref = %s   dynamic gaps = %s   stall boost = %s",
                    hardware.frame.planner_has_reference ? "yes" : "no",
                    hardware.frame.dynamic_gap_gates ? "yes" : "no",
                    hardware.frame.stall_boost_active ? "yes" : "no");
        ImGui::Text("gates = %d candidates   chosen distance = %.2f m",
                    hardware.frame.candidate_gates,
                    hardware.frame.chosen_gate_distance);
        ImGui::Text("LiDAR map pts = %d   no-motion cycles = %d",
                    hardware.frame.accumulated_lidar_points,
                    hardware.frame.no_motion_command_cycles);
        if (hardware.frame.has_last_mpc_command) {
            ImGui::Text("MPC: accel %.2f   steer rate %.2f deg/s",
                        hardware.frame.last_mpc_command.accel_cmd,
                        hardware.frame.last_mpc_command.steer_rate_cmd * 180.0 / 3.14159265358979323846);
        }
        if (hardware.has_scene && hardware.scene.world.environment_mode() == EnvironmentMode::UnstructuredGates) {
            const char* chosen_name =
                hardware.frame.chosen_gate_index >= 0 &&
                        hardware.frame.chosen_gate_index < static_cast<int>(hardware.frame.gates.size())
                    ? hardware.frame.gates[static_cast<std::size_t>(hardware.frame.chosen_gate_index)].spec.name.c_str()
                    : "none";
            ImGui::Text("chosen gate = %s   visible = %d", chosen_name, static_cast<int>(hardware.frame.visible_gate_indices.size()));
        }
    }

            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
    }

    ImGui::EndChild();
}

void render_control_panel(PlannerDrivenVehicleSim& sim, UiState* ui_state, LiveViewStreamServer* hardware_server) {
    if (!ImGui::BeginChild("ConfigPanel", ImVec2(0.0f, 0.0f), true)) {
        ImGui::EndChild();
        return;
    }

    const char* status = sim.goal_reached() ? "Goal reached" : (sim.collision() ? "Collision" : (ui_state->paused ? "Paused" : "Running"));
    const ImVec4 status_color = sim.goal_reached() ? ImVec4(0.45f, 0.86f, 0.53f, 1.0f)
                                                   : (sim.collision() ? ImVec4(0.95f, 0.40f, 0.34f, 1.0f)
                                                                      : (ui_state->paused ? ImVec4(0.74f, 0.79f, 0.84f, 1.0f)
                                                                                          : ImVec4(0.87f, 0.79f, 0.39f, 1.0f)));
    const auto& vehicle = sim.vehicle();
    const std::string chosen_name = simulation_gate_label(sim, sim.chosen_gate_index());
    char time_buf[32];
    char goal_buf[32];
    char speed_buf[32];
    char tracking_buf[48];
    std::snprintf(time_buf, sizeof(time_buf), "%.1f s", sim.sim_time());
    std::snprintf(goal_buf, sizeof(goal_buf), "%.2f m", sim.distance_to_goal());
    std::snprintf(speed_buf, sizeof(speed_buf), "%.2f m/s", vehicle.speed);
    std::snprintf(tracking_buf, sizeof(tracking_buf), "%.2f m", sim.tracker_cross_track_error());

    ImGui::TextUnformatted("Mission Desk");
    ImGui::TextColored(status_color, "%s", status);
    ImGui::TextWrapped("Drive the scenario, inspect sensing, and edit the map from one side panel without losing the live mission picture.");

    if (ImGui::BeginTable("ControlHeroMetrics", 2, ImGuiTableFlags_SizingStretchSame)) {
        ImGui::TableNextColumn();
        metric_card("control_time", "Runtime", time_buf, "Elapsed simulation time", ImVec4(0.43f, 0.82f, 0.96f, 1.0f), 64.0f);
        ImGui::TableNextColumn();
        metric_card("control_goal", "Goal", goal_buf, "Distance remaining", ImVec4(0.99f, 0.70f, 0.32f, 1.0f), 64.0f);
        ImGui::TableNextColumn();
        metric_card("control_speed", "Speed", speed_buf, "Current forward speed", ImVec4(0.48f, 0.88f, 0.62f, 1.0f), 64.0f);
        ImGui::TableNextColumn();
        metric_card("control_tracking", "Tracking", tracking_buf, "Cross-track error", ImVec4(0.97f, 0.89f, 0.45f, 1.0f), 64.0f);
        ImGui::EndTable();
    }

    render_sim_mission_readiness(sim);

    if (ImGui::BeginTabBar("SimulationDeskTabs")) {
        if (ImGui::BeginTabItem("Run", nullptr, requested_tab_flags(ui_state->requested_simulation_panel_tab, 0))) {
            activate_simulation_panel_tab(ui_state, 0, kWorkspaceViewMission, false);
            consume_requested_tab(&ui_state->requested_simulation_panel_tab, 0);
            render_validation_preset_grid(sim, ui_state);
            ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
            if (ImGui::CollapsingHeader("Playback", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::PushStyleColor(ImGuiCol_Button,
                              ui_state->paused ? ImVec4(0.15f, 0.43f, 0.52f, 1.0f) : ImVec4(0.66f, 0.47f, 0.16f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered,
                              ui_state->paused ? ImVec4(0.22f, 0.56f, 0.66f, 1.0f) : ImVec4(0.80f, 0.58f, 0.22f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonActive,
                              ui_state->paused ? ImVec4(0.11f, 0.33f, 0.40f, 1.0f) : ImVec4(0.51f, 0.35f, 0.10f, 1.0f));
        if (ImGui::Button(ui_state->paused ? "Resume Simulation" : "Pause Simulation", ImVec2(-1.0f, 0.0f))) {
            ui_state->paused = !ui_state->paused;
        }
        ImGui::PopStyleColor(3);

        if (ImGui::BeginTable("PlaybackButtons", 2, ImGuiTableFlags_SizingStretchSame)) {
            ImGui::TableNextColumn();
            if (ImGui::Button("Step Once", ImVec2(-1.0f, 0.0f))) {
                ui_state->single_step = true;
            }
            ImGui::TableNextColumn();
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.43f, 0.17f, 0.17f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.58f, 0.23f, 0.23f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.34f, 0.13f, 0.13f, 1.0f));
            if (ImGui::Button("Reset", ImVec2(-1.0f, 0.0f))) {
                sim.reset();
                ui_state->paused = true;
                ui_state->report_written = false;
                ui_state->last_report_path.clear();
            }
            ImGui::PopStyleColor(3);
            ImGui::EndTable();
        }
        ImGui::SliderInt("Steps / frame", &ui_state->steps_per_frame, 1, 8);
        ImGui::TextDisabled("Higher values speed up playback but make collisions and gate transitions harder to inspect.");
    }

            ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
            if (ImGui::CollapsingHeader("Live Diagnostics", ImGuiTreeNodeFlags_DefaultOpen)) {
                ImGui::Text("t = %.2f s   step = %d", sim.sim_time(), sim.step_count());
                ImGui::Text("pos = (%.2f, %.2f) m", vehicle.position.x, vehicle.position.y);
                ImGui::Text("yaw = %.1f deg   v = %.2f m/s", vehicle.yaw * 180.0 / 3.14159265358979323846, vehicle.speed);
                ImGui::Text("goal distance = %.2f m", sim.distance_to_goal());
                ImGui::Text("tracking: cte %.2f m   hdg %.2f deg", sim.tracker_cross_track_error(), sim.tracker_heading_error_deg());
                if (sim.last_mpc_command().has_value()) {
                    ImGui::Text("MPC: accel %.2f   steer rate %.2f deg/s",
                                sim.last_mpc_command()->accel_cmd,
                                sim.last_mpc_command()->steer_rate_cmd * 180.0 / 3.14159265358979323846);
                }
                if (sim.environment_mode() == EnvironmentMode::UnstructuredGates ||
                    sim.environment_mode() == EnvironmentMode::MixedRoadGates) {
                    ImGui::Text("chosen gate = %s   visible = %d", chosen_name.c_str(), static_cast<int>(sim.visible_gate_indices().size()));
                    if (sim.environment_mode() == EnvironmentMode::MixedRoadGates) {
                        const TelemetrySample* latest = sim.history().empty() ? nullptr : &sim.history().back();
                        ImGui::Text("mixed: mode %.0f   gate score %.2f   road score %.2f",
                                    latest != nullptr ? latest->mixed_mode : 0.0,
                                    latest != nullptr ? latest->mixed_gate_score : 0.0,
                                    latest != nullptr ? latest->mixed_structured_score : 0.0);
                    }
                } else {
                    ImGui::Text("road points = %d", static_cast<int>(sim.world().road_centerline().size()));
                }
            }
            ImGui::EndTabItem();
        }

        if (ImGui::BeginTabItem("Scenario", nullptr, requested_tab_flags(ui_state->requested_simulation_panel_tab, 1))) {
            activate_simulation_panel_tab(ui_state, 1, kWorkspaceViewMission, false);
            consume_requested_tab(&ui_state->requested_simulation_panel_tab, 1);
            ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
            if (ImGui::CollapsingHeader("Scenario & Planner", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::SameLine();
        help_marker("Switch between the gate-based planner and structured road behaviour without changing the rest of the simulator stack.");

        const char* environment_items[] = {
            thesis_sim::environment_mode_name(EnvironmentMode::UnstructuredGates),
            thesis_sim::environment_mode_name(EnvironmentMode::StructuredRoad),
            thesis_sim::environment_mode_name(EnvironmentMode::MixedRoadGates),
        };
        if (ImGui::Combo("Environment", &ui_state->environment_mode, environment_items, IM_ARRAYSIZE(environment_items))) {
            const bool ideal_active = ui_state->ideal_simulation || sim.config().ideal_conditions;
            if (ideal_active) {
                set_ui_ideal_map_for_mode(ui_state);
            } else {
                if (static_cast<EnvironmentMode>(ui_state->environment_mode) == EnvironmentMode::StructuredRoad &&
                    structured_preset_is_ideal(static_cast<StructuredMapPreset>(ui_state->structured_preset))) {
                    ui_state->structured_preset = static_cast<int>(StructuredMapPreset::ValidationRoad);
                } else if (static_cast<EnvironmentMode>(ui_state->environment_mode) == EnvironmentMode::UnstructuredGates &&
                           unstructured_preset_is_ideal(static_cast<UnstructuredMapPreset>(ui_state->unstructured_preset))) {
                    ui_state->unstructured_preset = static_cast<int>(UnstructuredMapPreset::RobotValidation);
                } else if (static_cast<EnvironmentMode>(ui_state->environment_mode) == EnvironmentMode::MixedRoadGates &&
                           mixed_preset_is_ideal(ui_state->mixed_preset)) {
                    ui_state->mixed_preset = 0;
                }
            }
            sim.load_world(world_from_ui_selection(sim, *ui_state));
            apply_gui_stack_for_selected_map(sim, ui_state);
        }

        const char* vehicle_items[] = {
            "Bicycle support",
            "Tank support",
        };
        int vehicle_selection = ui_state->vehicle_model == static_cast<int>(VehicleModelKind::TrackedVehicle) ? 1 : 0;
        if (ImGui::Combo("Motion Primitive Support", &vehicle_selection, vehicle_items, IM_ARRAYSIZE(vehicle_items))) {
            const VehicleModelKind next_model =
                vehicle_selection == 1 ? VehicleModelKind::TrackedVehicle : VehicleModelKind::CarLikeBicycle;
            ui_state->vehicle_model = static_cast<int>(next_model);
            sim.set_vehicle_stack(next_model, sim.tracking_controller_mode());
            if (static_cast<EnvironmentMode>(ui_state->environment_mode) == EnvironmentMode::StructuredRoad) {
                sim.load_world(world_from_ui_selection(sim, *ui_state));
            }
            apply_gui_stack_for_selected_map(sim, ui_state);
            sync_ui_state_from_sim(ui_state, sim);
            ui_state->paused = true;
        }
        if (sim.vehicle_model_kind() == VehicleModelKind::TrackedVehicle) {
            ImGui::TextWrapped("Tank support drives the tracked plant with the planner angular primitive while keeping the MPC speed layer active.");
        }

        if (sim.environment_mode() == EnvironmentMode::UnstructuredGates) {
            if (sim.world().unstructured_preset() == UnstructuredMapPreset::Custom) {
                ImGui::TextWrapped("Manual gate editor active: place gates and obstacles yourself, then let the planner reach each gate with the motion primitives on top of the mapped free space.");
            } else {
                ImGui::TextWrapped("Named gates are the planner's passage hypotheses through the free openings of the map. Keeping this section visible is useful when you debug wrong gate selection.");
            }

            const char* preset_items[] = {
                thesis_sim::unstructured_map_preset_name(UnstructuredMapPreset::RobotValidation),
                thesis_sim::unstructured_map_preset_name(UnstructuredMapPreset::TightCorridor),
                thesis_sim::unstructured_map_preset_name(UnstructuredMapPreset::WideSlalom),
                thesis_sim::unstructured_map_preset_name(UnstructuredMapPreset::LowerBypass),
                thesis_sim::unstructured_map_preset_name(UnstructuredMapPreset::Custom),
                thesis_sim::unstructured_map_preset_name(UnstructuredMapPreset::HardwareLab),
                thesis_sim::unstructured_map_preset_name(UnstructuredMapPreset::IdealValidation),
            };
            if (ImGui::Combo("Unstructured Map", &ui_state->unstructured_preset, preset_items, IM_ARRAYSIZE(preset_items))) {
                const bool manual_editor = ui_state->unstructured_preset == static_cast<int>(UnstructuredMapPreset::Custom);
                sim.load_world(world_from_ui_selection(sim, *ui_state));
                apply_gui_stack_for_selected_map(sim, ui_state);
                ui_state->map_editor_enabled = manual_editor;
            }
            ImGui::Text("Preset = %s", thesis_sim::unstructured_map_preset_name(sim.world().unstructured_preset()));
            bool dynamic_lidar_gates = sim.config().dynamic_lidar_gates;
            if (sim.config().ideal_conditions) {
                ImGui::TextColored(ImVec4(0.48f, 0.88f, 0.62f, 1.0f),
                                   "Ideal matched map: dynamic gates, Ideal 2D LiDAR, baseline vehicle limits");
            } else if (ImGui::Checkbox("LiDAR dynamic gates", &dynamic_lidar_gates)) {
                const bool was_paused = ui_state->paused;
                sim.set_dynamic_lidar_gates(dynamic_lidar_gates);
                sync_ui_state_from_sim(ui_state, sim);
                ui_state->paused = was_paused;
            }

            if (sim.world().unstructured_preset() == UnstructuredMapPreset::Custom) {
                ImGui::TextColored(ImVec4(1.0f, 0.82f, 0.42f, 1.0f), "User-defined gates stay static in manual editor mode.");
                ImGui::TextWrapped("Use the Map Editor section to add gates, move them directly in the viewport, and place obstacles the robot must sense and avoid.");
            } else {
                int gate_behavior = static_cast<int>(sim.gate_behavior());
                bool gate_changed = false;
                const char* gate_items[] = {
                    thesis_sim::gate_behavior_mode_name(GateBehaviorMode::Static),
                    thesis_sim::gate_behavior_mode_name(GateBehaviorMode::Randomized),
                    thesis_sim::gate_behavior_mode_name(GateBehaviorMode::Mobile),
                };
                if (ImGui::Combo("Gate Layout", &gate_behavior, gate_items, IM_ARRAYSIZE(gate_items))) {
                    gate_changed = true;
                }
                if (ImGui::InputInt("Gate Seed", &ui_state->gate_seed_input)) {
                    ui_state->gate_seed_input = std::max(ui_state->gate_seed_input, 0);
                    gate_changed = true;
                }
                if (gate_changed) {
                    sim.set_gate_behavior(
                        static_cast<GateBehaviorMode>(std::clamp(gate_behavior, 0, static_cast<int>(IM_ARRAYSIZE(gate_items)) - 1)),
                        static_cast<std::uint32_t>(ui_state->gate_seed_input));
                    ui_state->report_written = false;
                    ui_state->last_report_path.clear();
                }
                if (ImGui::Button("Regenerate Gates", ImVec2(-1.0f, 0.0f))) {
                    ui_state->gate_seed_input = std::max(ui_state->gate_seed_input + 1, 0);
                    sim.regenerate_gate_layout(static_cast<std::uint32_t>(ui_state->gate_seed_input));
                    ui_state->report_written = false;
                    ui_state->last_report_path.clear();
                }
                ImGui::Text("Gate mode = %s", thesis_sim::gate_behavior_mode_name(sim.gate_behavior()));
                ImGui::Text("Gate seed = %u", sim.gate_seed());
            }
        } else if (sim.environment_mode() == EnvironmentMode::MixedRoadGates) {
            ImGui::TextWrapped("Mixed mode follows the structured road while LiDAR gaps compete as dynamic gates. The arbiter only leaves the road when the gate is stable, aligned, safe and useful for progress.");
            const char* mixed_items[] = {
                mixed_map_preset_name(0),
                mixed_map_preset_name(1),
                mixed_map_preset_name(2),
            };
            int mixed_selection = std::clamp(ui_state->mixed_preset, 0, static_cast<int>(IM_ARRAYSIZE(mixed_items)) - 1);
            if (ImGui::Combo("Mixed Map", &mixed_selection, mixed_items, IM_ARRAYSIZE(mixed_items))) {
                ui_state->mixed_preset = mixed_selection;
                sim.load_world(world_from_ui_selection(sim, *ui_state));
                apply_gui_stack_for_selected_map(sim, ui_state);
            }
            ImGui::Text("Preset = %s", map_preset_name(sim.world()).c_str());
            bool dynamic_lidar_gates = sim.config().dynamic_lidar_gates;
            if (sim.config().ideal_conditions) {
                ImGui::TextColored(ImVec4(0.48f, 0.88f, 0.62f, 1.0f),
                                   "Ideal matched map: dynamic gates, Ideal 2D LiDAR, baseline vehicle limits");
            } else if (ImGui::Checkbox("LiDAR dynamic gates", &dynamic_lidar_gates)) {
                const bool was_paused = ui_state->paused;
                sim.set_dynamic_lidar_gates(dynamic_lidar_gates);
                sync_ui_state_from_sim(ui_state, sim);
                ui_state->paused = was_paused;
            }
            if (!sim.config().dynamic_lidar_gates) {
                ImGui::TextColored(ImVec4(1.0f, 0.82f, 0.42f, 1.0f), "Dynamic LiDAR gates are disabled; the mixed validation will remain on the structured road.");
            }
        } else {
            ImGui::TextWrapped("Structured mode keeps the planner on a fixed road loop while the EKF fuses encoder, IMU and LiDAR for the state estimate.");
            const char* structured_items[] = {
                thesis_sim::structured_map_preset_name(StructuredMapPreset::ValidationRoad),
                thesis_sim::structured_map_preset_name(StructuredMapPreset::CircleLoop),
                thesis_sim::structured_map_preset_name(StructuredMapPreset::ZigZag),
                thesis_sim::structured_map_preset_name(StructuredMapPreset::HardwareTrack),
                thesis_sim::structured_map_preset_name(StructuredMapPreset::FigureEight),
                thesis_sim::structured_map_preset_name(StructuredMapPreset::TankCircuit),
                thesis_sim::structured_map_preset_name(StructuredMapPreset::IdealCircle),
            };
            StructuredMapPreset previous_structured_preset =
                static_cast<StructuredMapPreset>(ui_state->structured_preset);
            int structured_selection = 0;
            switch (previous_structured_preset) {
                case StructuredMapPreset::ValidationRoad:
                    structured_selection = 0;
                    break;
                case StructuredMapPreset::CircleLoop:
                    structured_selection = 1;
                    break;
                case StructuredMapPreset::ZigZag:
                    structured_selection = 2;
                    break;
                case StructuredMapPreset::HardwareTrack:
                    structured_selection = 3;
                    break;
                case StructuredMapPreset::FigureEight:
                    structured_selection = 4;
                    break;
                case StructuredMapPreset::TankCircuit:
                    structured_selection = 5;
                    break;
                case StructuredMapPreset::IdealCircle:
                    structured_selection = 6;
                    break;
                case StructuredMapPreset::Custom:
                default:
                    structured_selection = 0;
                    break;
            }
            if (ImGui::Combo("Structured Map", &structured_selection, structured_items, IM_ARRAYSIZE(structured_items))) {
                StructuredMapPreset next_structured_preset = StructuredMapPreset::ValidationRoad;
                switch (structured_selection) {
                    case 1:
                        next_structured_preset = StructuredMapPreset::CircleLoop;
                        break;
                    case 2:
                        next_structured_preset = StructuredMapPreset::ZigZag;
                        break;
                    case 3:
                        next_structured_preset = StructuredMapPreset::HardwareTrack;
                        break;
                    case 4:
                        next_structured_preset = StructuredMapPreset::FigureEight;
                        break;
                    case 5:
                        next_structured_preset = StructuredMapPreset::TankCircuit;
                        break;
                    case 6:
                        next_structured_preset = StructuredMapPreset::IdealCircle;
                        break;
                    case 0:
                    default:
                        next_structured_preset = StructuredMapPreset::ValidationRoad;
                        break;
                }
                ui_state->structured_preset = static_cast<int>(next_structured_preset);
                sim.load_world(world_from_ui_selection(sim, *ui_state));
                apply_gui_stack_for_selected_map(sim, ui_state);
            }
            ImGui::Text("Preset = %s", thesis_sim::structured_map_preset_name(sim.world().structured_preset()));
            ImGui::Text("Road points = %d", static_cast<int>(sim.world().road_centerline().size()));
            if (sim.vehicle_model_kind() == VehicleModelKind::TrackedVehicle) {
                bool tank_ideal =
                    static_cast<StructuredMapPreset>(ui_state->structured_preset) == StructuredMapPreset::IdealCircle;
                if (ImGui::Checkbox("Tank ideal simulation", &tank_ideal)) {
                    ui_state->structured_preset = static_cast<int>(
                        tank_ideal ? StructuredMapPreset::IdealCircle : StructuredMapPreset::ValidationRoad);
                    ui_state->ideal_simulation = tank_ideal;
                    sim.load_world(world_from_ui_selection(sim, *ui_state));
                    apply_gui_stack_for_selected_map(sim, ui_state);
                }
                ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
                ImGui::TextWrapped("Tank baseline uses the validated 1 m structured road; ideal keeps the same road with ideal sensing and best-case tracked tuning.");
                ImGui::PopStyleColor();
            }
            if (sim.world().structured_preset() == StructuredMapPreset::Custom) {
                ImGui::TextColored(ImVec4(1.0f, 0.82f, 0.42f, 1.0f), "Current road = Custom edited scenario");
                ImGui::TextWrapped("The last edited structured road is active. Built-in presets in the combo restore the known-good loops.");
                if (ImGui::Button("Restore Validation Road", ImVec2(-1.0f, 0.0f))) {
                    ui_state->structured_preset = static_cast<int>(StructuredMapPreset::ValidationRoad);
                    sim.load_world(world_from_ui_selection(sim, *ui_state));
                    apply_gui_stack_for_selected_map(sim, ui_state);
                }
            }
            ImGui::TextDisabled("Gate layout controls are disabled in structured mode.");
        }
    }

            ImGui::EndTabItem();
        }

        if (ImGui::BeginTabItem("Sensors", nullptr, requested_tab_flags(ui_state->requested_simulation_panel_tab, 2))) {
            activate_simulation_panel_tab(ui_state, 2, kWorkspaceViewDiagnostics, false);
            consume_requested_tab(&ui_state->requested_simulation_panel_tab, 2);
            ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
            if (ImGui::CollapsingHeader("Sensors & Localization", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::SameLine();
        help_marker("These toggles affect the simulated measurements that feed the EKF. The planner still works on the fused state estimate.");
        bool imu_enabled = sim.imu_enabled();
        bool lidar_enabled = sim.lidar_enabled();
        int sensor_profile = static_cast<int>(sim.range_sensor_profile());
        bool sensor_changed = false;

        if (sim.config().ideal_conditions) {
            ImGui::TextColored(ImVec4(0.48f, 0.88f, 0.62f, 1.0f),
                               "Ideal preset locks IMU, LiDAR and range profile.");
        } else {
            if (ImGui::Checkbox("Enable IMU", &imu_enabled)) {
                sensor_changed = true;
            }
            if (ImGui::Checkbox("Enable LiDAR", &lidar_enabled)) {
                sensor_changed = true;
            }

            const char* sensor_items[] = {
                thesis_sim::range_sensor_profile_name(RangeSensorProfile::IdealLidar2D),
                thesis_sim::range_sensor_profile_name(RangeSensorProfile::RplidarA1),
                thesis_sim::range_sensor_profile_name(RangeSensorProfile::ShortRangeScanner),
            };
            if (ImGui::Combo("Range Sensor", &sensor_profile, sensor_items, IM_ARRAYSIZE(sensor_items))) {
                sensor_changed = true;
            }
            if (sensor_changed) {
                sim.set_sensor_suite(
                    imu_enabled,
                    lidar_enabled,
                    static_cast<RangeSensorProfile>(std::clamp(sensor_profile, 0, static_cast<int>(IM_ARRAYSIZE(sensor_items)) - 1)));
                sync_ui_state_from_sim(ui_state, sim);
                ui_state->report_written = false;
                ui_state->last_report_path.clear();
            }
        }

        ImGui::Text("Localization = %s", sim.localization_mode_name());
        ImGui::Text("Heading source = %s", sim.heading_source_name());
        ImGui::Text("Stack = %s + %s",
                    thesis_sim::vehicle_model_kind_name(sim.vehicle_model_kind()),
                    thesis_sim::tracking_controller_mode_name(sim.tracking_controller_mode()));
        if (sim.lidar_enabled()) {
            ImGui::Text("LiDAR = %s, min %.2f m",
                        thesis_sim::range_sensor_profile_name(sim.range_sensor_profile()),
                        sim.min_lidar_distance());
        } else {
            ImGui::Text("LiDAR = disabled");
        }
    }

            ImGui::EndTabItem();
        }

        if (ImGui::BeginTabItem("Map", nullptr, requested_tab_flags(ui_state->requested_simulation_panel_tab, 3))) {
            activate_simulation_panel_tab(ui_state, 3, kWorkspaceViewMap, true);
            consume_requested_tab(&ui_state->requested_simulation_panel_tab, 3);
            ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
            if (ImGui::CollapsingHeader("Map Editor", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::SameLine();
        help_marker("Enable drag mode, move handles directly in the viewport, then apply the edited scenario.");
        WorldMap& editor_world = ui_state->scenario_editor_world;
        ImGui::Checkbox("Enable drag editing", &ui_state->map_editor_enabled);
        if (ImGui::Button("Load Current Scenario", ImVec2(-1.0f, 0.0f))) {
            ui_state->scenario_editor_world = sim.world();
            ui_state->scenario_editor_dirty = false;
            ui_state->selected_editor_handle = {};
            ui_state->active_drag_handle = {};
        }
        if (ImGui::Button("Apply Edited Map", ImVec2(-1.0f, 0.0f))) {
            ui_state->scenario_editor_world.finalize_editor_changes();
            sim.load_world(ui_state->scenario_editor_world);
            sync_ui_state_from_sim(ui_state, sim);
            ui_state->paused = true;
        }

        if (ImGui::CollapsingHeader("Manual Map Tools", ImGuiTreeNodeFlags_DefaultOpen)) {
            if (ImGui::Button("Add Obstacle", ImVec2(-1.0f, 0.0f))) {
                const Vec2 center{
                    0.5 * (editor_world.start().x + editor_world.goal().x),
                    0.5 * (editor_world.start().y + editor_world.goal().y),
                };
                editor_world.editable_obstacles().push_back({center.x - 1.0, center.y - 1.0, center.x + 1.0, center.y + 1.0});
                ui_state->scenario_editor_dirty = true;
                ui_state->selected_editor_handle = {MapEditorHandleType::Obstacle, static_cast<int>(editor_world.editable_obstacles().size()) - 1};
            }

            if (editor_world.environment_mode() == EnvironmentMode::UnstructuredGates) {
                if (ImGui::Button("Add Gate", ImVec2(-1.0f, 0.0f))) {
                    const Vec2 midpoint{
                        0.5 * (editor_world.start().x + editor_world.goal().x),
                        0.5 * (editor_world.start().y + editor_world.goal().y),
                    };
                    GateSpec gate;
                    gate.name = "gate_" + std::to_string(editor_world.editable_gates().size() + 1);
                    gate.position = midpoint;
                    gate.anchor_position = midpoint;
                    editor_world.editable_gates().push_back(gate);
                    ui_state->scenario_editor_dirty = true;
                    ui_state->selected_editor_handle = {MapEditorHandleType::Gate, static_cast<int>(editor_world.editable_gates().size()) - 1};
                }
            } else {
                if (ImGui::Button("Add Road Point", ImVec2(-1.0f, 0.0f))) {
                    std::vector<Vec2>& road = editor_world.editable_road_centerline();
                    const bool closed_loop =
                        road.size() >= 3 && thesis_sim::distance(road.front(), road.back()) < 0.5;
                    if (closed_loop) {
                        road.insert(road.end() - 1, editor_world.goal());
                        ui_state->selected_editor_handle = {
                            MapEditorHandleType::RoadPoint,
                            static_cast<int>(road.size()) - 2,
                        };
                    } else {
                        road.push_back(editor_world.goal());
                        ui_state->selected_editor_handle = {
                            MapEditorHandleType::RoadPoint,
                            static_cast<int>(road.size()) - 1,
                        };
                    }
                    ui_state->scenario_editor_dirty = true;
                }
            }

            if (ui_state->selected_editor_handle.type != MapEditorHandleType::None) {
                ImGui::Text("Selected = %s", map_editor_handle_type_name(ui_state->selected_editor_handle.type));
                if (ImGui::Button("Remove Selected", ImVec2(-1.0f, 0.0f))) {
                    switch (ui_state->selected_editor_handle.type) {
                        case MapEditorHandleType::Obstacle:
                            if (ui_state->selected_editor_handle.index >= 0 &&
                                ui_state->selected_editor_handle.index < static_cast<int>(editor_world.editable_obstacles().size())) {
                                editor_world.editable_obstacles().erase(
                                    editor_world.editable_obstacles().begin() + ui_state->selected_editor_handle.index);
                                ui_state->scenario_editor_dirty = true;
                            }
                            break;
                        case MapEditorHandleType::Gate:
                            if (ui_state->selected_editor_handle.index >= 0 &&
                                ui_state->selected_editor_handle.index < static_cast<int>(editor_world.editable_gates().size())) {
                                editor_world.editable_gates().erase(
                                    editor_world.editable_gates().begin() + ui_state->selected_editor_handle.index);
                                ui_state->scenario_editor_dirty = true;
                            }
                            break;
                        case MapEditorHandleType::RoadPoint:
                            if (ui_state->selected_editor_handle.index > 0 &&
                                ui_state->selected_editor_handle.index + 1 < static_cast<int>(editor_world.editable_road_centerline().size())) {
                                editor_world.editable_road_centerline().erase(
                                    editor_world.editable_road_centerline().begin() + ui_state->selected_editor_handle.index);
                                ui_state->scenario_editor_dirty = true;
                            }
                            break;
                        default:
                            break;
                    }
                    ui_state->selected_editor_handle = {};
                    ui_state->active_drag_handle = {};
                }
            } else {
                ImGui::TextDisabled("No editor handle selected.");
            }

            render_editor_inspector(editor_world, ui_state, &ui_state->scenario_editor_dirty);
            ImGui::TextDisabled("%s", ui_state->scenario_editor_dirty ? "Edited scenario has unapplied changes." : "Edited scenario is synchronized.");
            if (ui_state->map_editor_enabled) {
                ImGui::TextWrapped("Drag start, goal, obstacle centers, gate anchors or interior road points directly in the viewport.");
            }
        }
    }

            ImGui::EndTabItem();
        }

        if (ImGui::BeginTabItem("Export", nullptr, requested_tab_flags(ui_state->requested_simulation_panel_tab, 4))) {
            activate_simulation_panel_tab(ui_state, 4, kWorkspaceViewExport, false);
            consume_requested_tab(&ui_state->requested_simulation_panel_tab, 4);
            ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
            if (ImGui::CollapsingHeader("Reports", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::SameLine();
        help_marker("Writes paper-ready artifacts: JSON for reproducibility, CSV for plots, Markdown for quick thesis notes.");
        if (ImGui::Button("Write JSON Report", ImVec2(-1.0f, 0.0f))) {
            const std::string report_path = default_report_path(sim, "gui");
            if (write_json_report(sim, report_status_string(sim), report_path)) {
                ui_state->report_written = true;
                ui_state->last_report_path = report_path;
                ui_state->last_export_error.clear();
            } else {
                ui_state->last_export_error = "Could not write JSON report.";
            }
        }
        if (ImGui::Button("Write Paper Bundle", ImVec2(-1.0f, 0.0f))) {
            const std::string json_path = default_report_path(sim, "gui_bundle");
            const std::string csv_path = replace_extension(json_path, ".csv");
            const std::string markdown_path = replace_extension(json_path, ".md");
            const bool json_ok = write_json_report(sim, report_status_string(sim), json_path);
            const bool csv_ok = write_sim_csv_report(sim, csv_path);
            const bool markdown_ok = write_sim_markdown_summary(sim, report_status_string(sim), markdown_path);
            if (json_ok && csv_ok && markdown_ok) {
                ui_state->report_written = true;
                ui_state->last_report_path = json_path;
                ui_state->last_csv_report_path = csv_path;
                ui_state->last_markdown_report_path = markdown_path;
                ui_state->last_export_error.clear();
            } else {
                ui_state->last_export_error = "Could not write the complete paper bundle.";
            }
        }
        if (!ui_state->last_report_path.empty()) {
            ImGui::TextWrapped("JSON: %s", ui_state->last_report_path.c_str());
        }
        if (!ui_state->last_csv_report_path.empty()) {
            ImGui::TextWrapped("CSV: %s", ui_state->last_csv_report_path.c_str());
        }
        if (!ui_state->last_markdown_report_path.empty()) {
            ImGui::TextWrapped("Markdown: %s", ui_state->last_markdown_report_path.c_str());
        }
        if (!ui_state->last_export_error.empty()) {
            ImGui::TextColored(ImVec4(0.95f, 0.40f, 0.34f, 1.0f), "%s", ui_state->last_export_error.c_str());
        }
    }

            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
    }

    ImGui::EndChild();
}

void render_workspace_top_bar(PlannerDrivenVehicleSim& sim,
                              const HardwareViewerState* hardware,
                              UiState* ui_state,
                              LiveViewStreamServer* hardware_server) {
    if (ui_state == nullptr) {
        return;
    }

    const bool hardware_mode = workspace_source_is_hardware(ui_state->workspace_source);
    const VehicleSnapshot& vehicle = sim.vehicle();
    const bool hardware_connected = hardware_server != nullptr && hardware_server->connected();
    const bool hardware_listening = hardware_server != nullptr && hardware_server->listening();

    const char* status_text = nullptr;
    ImVec4 status_color;
    std::string scenario_label;
    std::string map_label;
    std::string robot_label;
    char goal_buf[48];
    char speed_buf[48];

    if (hardware_mode) {
        const LiveFrameSnapshot* frame = hardware != nullptr ? &hardware->frame : nullptr;
        status_text = hardware_connected
                          ? ((frame != nullptr && frame->goal_reached) ? "Goal reached"
                                                                        : ((frame != nullptr && frame->safety_stop_active) ? "Safety stop" : "Live"))
                          : (hardware_listening ? "Listening" : "Preview");
        status_color = hardware_connected
                           ? ((frame != nullptr && frame->goal_reached) ? ImVec4(0.45f, 0.86f, 0.53f, 1.0f)
                                                                         : ((frame != nullptr && frame->safety_stop_active)
                                                                                ? ImVec4(0.95f, 0.40f, 0.34f, 1.0f)
                                                                                : ImVec4(0.87f, 0.79f, 0.39f, 1.0f)))
                           : (hardware_listening ? ImVec4(0.43f, 0.82f, 0.96f, 1.0f)
                                                 : ImVec4(0.74f, 0.79f, 0.84f, 1.0f));
        scenario_label = "Hardware Planner";
        map_label = hardware != nullptr && hardware->has_scene
                        ? map_preset_name(hardware->scene.world)
                        : "configured preview";
        robot_label = hardware != nullptr && hardware->has_scene
                          ? hardware->scene.vehicle_model_name
                          : thesis_sim::vehicle_model_kind_name(static_cast<VehicleModelKind>(ui_state->hardware_vehicle_model));
        const std::string goal_label =
            hardware != nullptr
                ? hardware_goal_distance_label(hardware->frame, hardware->scene.world.environment_mode())
                : "n/a";
        std::snprintf(goal_buf, sizeof(goal_buf), "%s", goal_label.c_str());
        std::snprintf(speed_buf,
                      sizeof(speed_buf),
                      "%.2f m/s",
                      frame != nullptr ? frame->vehicle.speed : 0.0);
    } else {
        status_text = sim.goal_reached() ? "Goal reached" : (sim.collision() ? "Collision" : (ui_state->paused ? "Paused" : "Running"));
        status_color = sim.goal_reached() ? ImVec4(0.45f, 0.86f, 0.53f, 1.0f)
                                          : (sim.collision() ? ImVec4(0.95f, 0.40f, 0.34f, 1.0f)
                                                             : (ui_state->paused ? ImVec4(0.74f, 0.79f, 0.84f, 1.0f)
                                                                                 : ImVec4(0.87f, 0.79f, 0.39f, 1.0f)));
        scenario_label = thesis_sim::environment_mode_name(sim.environment_mode());
        map_label = map_preset_name(sim.world());
        robot_label = thesis_sim::vehicle_model_kind_name(sim.vehicle_model_kind());
        std::snprintf(goal_buf, sizeof(goal_buf), "%.2f m", sim.distance_to_goal());
        std::snprintf(speed_buf, sizeof(speed_buf), "%.2f m/s", vehicle.speed);
    }

    ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 8.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(14.0f, 10.0f));
    ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.07f, 0.10f, 0.13f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0.18f, 0.25f, 0.29f, 1.0f));
    if (ImGui::BeginChild("WorkspaceTopBar", ImVec2(0.0f, 68.0f), true, ImGuiWindowFlags_NoScrollbar)) {
        if (ImGui::BeginTable("WorkspaceTopBarLayout", 4, ImGuiTableFlags_SizingStretchProp)) {
            ImGui::TableSetupColumn("Title", ImGuiTableColumnFlags_WidthStretch, 1.2f);
            ImGui::TableSetupColumn("Source", ImGuiTableColumnFlags_WidthFixed, 248.0f);
            ImGui::TableSetupColumn("State", ImGuiTableColumnFlags_WidthFixed, 230.0f);
            ImGui::TableSetupColumn("Vehicle", ImGuiTableColumnFlags_WidthFixed, 245.0f);
            ImGui::TableNextColumn();
            ImGui::TextUnformatted("Thesis Planner Mission Desk");
            ImGui::TextDisabled("%s  |  %s", scenario_label.c_str(), map_label.c_str());

            ImGui::TableNextColumn();
            ImGui::TextDisabled("Workspace");
            auto source_button = [&](const char* label, int source) {
                const bool selected = ui_state->workspace_source == source;
                if (selected) {
                    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.15f, 0.43f, 0.52f, 1.0f));
                    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.22f, 0.56f, 0.66f, 1.0f));
                    ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.11f, 0.33f, 0.40f, 1.0f));
                }
                if (ImGui::Button(label, ImVec2(112.0f, 0.0f))) {
                    ui_state->workspace_source = source;
                    if (workspace_source_is_hardware(source) &&
                        hardware_server != nullptr &&
                        !hardware_server->listening() &&
                        ui_state->hardware_listen_port > 0) {
                        hardware_server->start(static_cast<std::uint16_t>(ui_state->hardware_listen_port));
                    }
                }
                if (selected) {
                    ImGui::PopStyleColor(3);
                }
            };
            source_button("Simulation", kWorkspaceSourceSimulation);
            ImGui::SameLine();
            source_button("Hardware", kWorkspaceSourceHardwarePlanner);

            ImGui::TableNextColumn();
            ImGui::TextDisabled("Run State");
            ImGui::PushStyleColor(ImGuiCol_Text, status_color);
            ImGui::TextUnformatted(status_text);
            ImGui::PopStyleColor();
            ImGui::TextDisabled("goal %s", goal_buf);

            ImGui::TableNextColumn();
            ImGui::TextDisabled("Robot");
            ImGui::TextUnformatted(robot_label.c_str());
            ImGui::TextDisabled("speed %s", speed_buf);
            ImGui::EndTable();
        }
    }
    ImGui::EndChild();
    ImGui::PopStyleColor(2);
    ImGui::PopStyleVar(2);
}

void render_workspace_navigation_rail(UiState* ui_state) {
    if (ui_state == nullptr) {
        return;
    }

    ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 8.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(10.0f, 12.0f));
    ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.06f, 0.08f, 0.11f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0.17f, 0.23f, 0.28f, 1.0f));
    if (ImGui::BeginChild("WorkspaceNavigationRail", ImVec2(116.0f, 0.0f), true)) {
        ImGui::TextDisabled("Views");
        ImGui::Spacing();

        auto nav_button = [&](const char* label,
                              int view,
                              int sim_panel_tab,
                              int hardware_panel_tab,
                              bool editor_mode) {
            const bool selected = ui_state->workspace_view == view;
            if (selected) {
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.15f, 0.43f, 0.52f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.22f, 0.56f, 0.66f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.11f, 0.33f, 0.40f, 1.0f));
            }
            if (ImGui::Button(label, ImVec2(-1.0f, 38.0f))) {
                ui_state->workspace_view = view;
                ui_state->simulation_panel_tab = sim_panel_tab;
                ui_state->hardware_panel_tab = hardware_panel_tab;
                ui_state->requested_simulation_panel_tab = sim_panel_tab;
                ui_state->requested_hardware_panel_tab = hardware_panel_tab;
                ui_state->map_editor_enabled = editor_mode;
            }
            if (selected) {
                ImGui::PopStyleColor(3);
            }
        };

        nav_button("Mission", kWorkspaceViewMission, 0, 0, false);
        nav_button("Analytics", kWorkspaceViewAnalytics, 0, 3, false);
        nav_button("Map", kWorkspaceViewMap, 3, 2, true);
        nav_button("Diagnostics", kWorkspaceViewDiagnostics, 2, 3, false);
        nav_button("Export", kWorkspaceViewExport, 4, 1, false);
    }
    ImGui::EndChild();
    ImGui::PopStyleColor(2);
    ImGui::PopStyleVar(2);
}

void render_workspace(PlannerDrivenVehicleSim& sim,
                      HardwareViewerState* hardware,
                      UiState* ui_state,
                      LiveViewStreamServer* hardware_server) {
    ImGuiViewport* viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(viewport->WorkPos);
    ImGui::SetNextWindowSize(viewport->WorkSize);
    ImGuiWindowFlags flags =
        ImGuiWindowFlags_NoDecoration |
        ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoResize |
        ImGuiWindowFlags_NoSavedSettings;

    if (!ImGui::Begin("Thesis Planner Workspace", nullptr, flags)) {
        ImGui::End();
        return;
    }

    render_workspace_top_bar(sim, hardware, ui_state, hardware_server);
    ImGui::Spacing();

    const bool hardware_mode = ui_state != nullptr && workspace_source_is_hardware(ui_state->workspace_source);
    if (ui_state != nullptr) {
        ui_state->workspace_view = std::clamp(
            ui_state->workspace_view,
            static_cast<int>(kWorkspaceViewMission),
            static_cast<int>(kWorkspaceViewExport));
        const bool graph_view =
            ui_state->workspace_view == kWorkspaceViewAnalytics ||
            ui_state->workspace_view == kWorkspaceViewDiagnostics ||
            ui_state->workspace_view == kWorkspaceViewExport;
        ui_state->workspace_tab = graph_view ? 1 : 0;
    }
    const float workspace_width = ImGui::GetContentRegionAvail().x;
    const float context_width = std::clamp(workspace_width * 0.26f, 315.0f, 390.0f);
    if (ImGui::BeginTable("WorkspaceLayout", 3, ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_BordersInnerV)) {
        ImGui::TableSetupColumn("Navigation", ImGuiTableColumnFlags_WidthFixed, 124.0f);
        ImGui::TableSetupColumn("MainArea", ImGuiTableColumnFlags_WidthStretch, 1.0f);
        ImGui::TableSetupColumn("ContextArea", ImGuiTableColumnFlags_WidthFixed, context_width);

        ImGui::TableNextColumn();
        render_workspace_navigation_rail(ui_state);

        ImGui::TableNextColumn();
        const int view = ui_state != nullptr ? ui_state->workspace_view : kWorkspaceViewMission;
        switch (view) {
            case kWorkspaceViewAnalytics:
            case kWorkspaceViewDiagnostics:
            case kWorkspaceViewExport:
                if (hardware_mode && hardware != nullptr) {
                    render_hardware_graphs_tab(*hardware);
                } else {
                    render_graphs_tab(sim);
                }
                break;
            case kWorkspaceViewMap:
            case kWorkspaceViewMission:
            default:
                if (hardware_mode && hardware != nullptr) {
                    render_hardware_world_tab(*hardware, ui_state);
                } else {
                    render_world_tab(sim, ui_state);
                }
                break;
        }

        ImGui::TableNextColumn();
        const bool telemetry_context = view == kWorkspaceViewAnalytics;
        if (telemetry_context) {
            if (hardware_mode && hardware != nullptr) {
                render_hardware_telemetry_side_panel(*hardware);
            } else {
                render_sim_telemetry_side_panel(sim);
            }
        } else {
            if (hardware_mode && hardware != nullptr) {
                if (view == kWorkspaceViewAnalytics) {
                    ui_state->hardware_panel_tab = 3;
                }
                render_hardware_control_panel(*hardware, ui_state, hardware_server);
            } else {
                if (view == kWorkspaceViewAnalytics) {
                    ui_state->simulation_panel_tab = 0;
                }
                render_control_panel(sim, ui_state, hardware_server);
            }
        }
        ImGui::EndTable();
    }

    ImGui::End();
}

int run_headless(const AppOptions& options) {
    thesis_sim::SimConfig sim_config;
    sim_config.vehicle_model = options.vehicle_model;
    sim_config.dynamic_lidar_gates = options.dynamic_lidar_gates;
    apply_sim_level_to_config(options, &sim_config);
    WorldMap world = make_world_from_mode(
        options.environment_mode,
        options.unstructured_preset,
        options.structured_preset,
        GateBehaviorMode::Static,
        7,
        options.mixed_preset);
    world = fit_simulation_structured_world(std::move(world), options.vehicle_model);
    PlannerDrivenVehicleSim sim(std::move(world), sim_config);
    apply_sim_tuning_overrides(&sim, options);
    const SimulationReport report = sim.run_headless(options.max_steps);
    const std::string status = report.goal_reached ? "goal_reached" : (report.collision ? "collision" : "timeout");
    const std::string report_path = default_report_path(sim, "headless");
    const bool report_written = write_json_report(sim, status, report_path);

    std::cout << "status=" << status << '\n';
    std::cout << "steps=" << report.steps << '\n';
    std::cout << "time=" << report.sim_time << '\n';
    std::cout << "final_x=" << report.final_position.x << '\n';
    std::cout << "final_y=" << report.final_position.y << '\n';
    std::cout << "distance_to_goal=" << report.distance_to_goal << '\n';
    std::cout << "passed_gates=" << report.passed_gates << '\n';
    if (report_written) {
        std::cout << "report_json=" << report_path << '\n';
    }

    return report.goal_reached ? 0 : 1;
}

int run_gui(const AppOptions& options) {
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) != 0) {
        std::cerr << "SDL_Init failed: " << SDL_GetError() << '\n';
        return 1;
    }

    SDL_Window* window = SDL_CreateWindow(
        "Thesis Planner Simulator",
        SDL_WINDOWPOS_CENTERED,
        SDL_WINDOWPOS_CENTERED,
        1500,
        920,
        SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);

    if (window == nullptr) {
        std::cerr << "SDL_CreateWindow failed: " << SDL_GetError() << '\n';
        SDL_Quit();
        return 1;
    }

    SDL_Renderer* renderer = SDL_CreateRenderer(
        window,
        -1,
        SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);

    if (renderer == nullptr) {
        std::cerr << "SDL_CreateRenderer failed: " << SDL_GetError() << '\n';
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    apply_style();

    ImGui_ImplSDL2_InitForSDLRenderer(window, renderer);
    ImGui_ImplSDLRenderer2_Init(renderer);

    thesis_sim::SimConfig sim_config;
    sim_config.vehicle_model = options.vehicle_model;
    sim_config.dynamic_lidar_gates = options.dynamic_lidar_gates;
    apply_sim_level_to_config(options, &sim_config);
    WorldMap world = make_world_from_mode(
        options.environment_mode,
        options.unstructured_preset,
        options.structured_preset,
        GateBehaviorMode::Static,
        7,
        options.mixed_preset);
    world = fit_simulation_structured_world(std::move(world), options.vehicle_model);
    PlannerDrivenVehicleSim sim(std::move(world), sim_config);
    apply_sim_tuning_overrides(&sim, options);
    HardwareViewerState hardware_view;
    LiveViewStreamServer hardware_server;
    bool running = true;
    UiState ui_state;
    sync_ui_state_from_sim(&ui_state, sim);

    while (running) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            ImGui_ImplSDL2_ProcessEvent(&event);
            if (event.type == SDL_QUIT) {
                running = false;
            }
        }

        const LiveViewStreamServer::PollResult hardware_updates = hardware_server.poll();
        const bool hardware_stream_connected = hardware_server.connected();
        if (hardware_stream_connected && !ui_state.hardware_stream_connected_prev) {
            queue_current_hardware_world(
                &ui_state,
                &hardware_server,
                "Current hardware world detected and queued automatically for the newly connected Raspberry runner. Waiting for runner ack.",
                "Could not auto-queue the current hardware world for the newly connected Raspberry runner.");
            queue_hardware_robot_profile(
                &ui_state,
                &hardware_server,
                static_cast<VehicleModelKind>(ui_state.hardware_vehicle_model),
                "Current robot support queued automatically for the newly connected Raspberry runner. Waiting for runner ack.",
                "Could not auto-queue the robot support profile for the Raspberry runner.");
        }
        ui_state.hardware_stream_connected_prev = hardware_stream_connected;
        if (hardware_updates.scene_received && hardware_updates.scene.has_value()) {
            hardware_view.scene = *hardware_updates.scene;
            hardware_view.has_scene = true;
            ui_state.hardware_vehicle_model =
                static_cast<int>(vehicle_model_from_live_name(hardware_view.scene.vehicle_model_name));
            hardware_view.frame = {};
            hardware_view.history.clear();
            ui_state.hardware_report_written = false;
            ui_state.last_hardware_report_path.clear();
            ui_state.last_hardware_report_error.clear();
        }
        if (hardware_updates.frame_received && hardware_updates.frame.has_value()) {
            hardware_view.frame = *hardware_updates.frame;
            if (hardware_view.frame.has_latest_sample) {
                const bool duplicate =
                    !hardware_view.history.empty() &&
                    std::abs(hardware_view.history.back().time - hardware_view.frame.latest_sample.time) < 1e-9;
                if (!duplicate) {
                    hardware_view.history.push_back(hardware_view.frame.latest_sample);
                }
                constexpr int kMaxHardwareHistory = 2400;
                if (static_cast<int>(hardware_view.history.size()) > kMaxHardwareHistory) {
                    hardware_view.history.erase(hardware_view.history.begin());
                }
            }
        }
        if (hardware_updates.control_ack_received && hardware_updates.control_ack.has_value()) {
            ui_state.hardware_world_sync_pending = false;
            ui_state.hardware_world_sync_ok = hardware_updates.control_ack->ok;
            ui_state.last_hardware_world_sync_status = hardware_updates.control_ack->message;
        }

        if (ui_state.workspace_source == kWorkspaceSourceSimulation && !ui_state.paused && !sim.goal_reached() && !sim.collision()) {
            for (int i = 0; i < ui_state.steps_per_frame; ++i) {
                sim.step();
                if (sim.goal_reached() || sim.collision()) {
                    break;
                }
            }
        } else if (ui_state.workspace_source == kWorkspaceSourceSimulation && ui_state.single_step) {
            sim.step();
            ui_state.single_step = false;
        }

        if ((sim.goal_reached() || sim.collision()) && !ui_state.report_written) {
            const std::string report_path = default_report_path(sim, "gui_auto");
            if (write_json_report(sim, report_status_string(sim), report_path)) {
                ui_state.report_written = true;
                ui_state.last_report_path = report_path;
            }
        }

        ImGui_ImplSDLRenderer2_NewFrame();
        ImGui_ImplSDL2_NewFrame();
        ImGui::NewFrame();

        render_workspace(sim, &hardware_view, &ui_state, &hardware_server);

        ImGui::Render();
        SDL_SetRenderDrawColor(renderer, 9, 12, 15, 255);
        SDL_RenderClear(renderer);
        ImGui_ImplSDLRenderer2_RenderDrawData(ImGui::GetDrawData(), renderer);
        SDL_RenderPresent(renderer);
    }

    ImGui_ImplSDLRenderer2_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    const AppOptions options = parse_args(argc, argv);
    if (options.headless) {
        return run_headless(options);
    }
    return run_gui(options);
}
