#include <SDL.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "imgui.h"
#include "imgui_impl_sdl2.h"
#include "imgui_impl_sdlrenderer2.h"
#include "implot.h"

#include "mvc/controller/application/app_options.h"
#include "mvc/controller/application/simulation_report_controller.h"
#include "mvc/controller/simulation_planner/simulator.h"
#include "mvc/controller/slam/slam_toolbox_bridge.h"
#include "mvc/model/reporting/slam_reference_exporter.h"
#include "mvc/model/navigation/robot_calibration_profile.h"
#include "mvc/model/world/scenario_model.h"
#include "mvc/view/canvas/world_canvas_view.h"
#include "mvc/view/live_stream/live_view_stream.h"
#include "mvc/view/simulator_app/hardware_view_model.h"

namespace {

using thesis_sim::GateSpec;
using thesis_sim::GateBehaviorMode;
using thesis_sim::EnvironmentMode;
using thesis_sim::HardwareLocalizationPolicy;
using thesis_sim::HardwareTelemetrySample;
using thesis_sim::LidarHit;
using thesis_sim::LiveFrameSnapshot;
using thesis_sim::LiveGateFrame;
using thesis_sim::LiveSceneSnapshot;
using thesis_sim::LiveRuntimeSettings;
using thesis_sim::LiveVehicleState;
using thesis_sim::LiveViewStreamServer;
using thesis_sim::SlamToolboxBridgeClient;
using thesis_sim::SlamToolboxSnapshot;
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
using thesis_sim::mvc::controller::AppOptions;
using thesis_sim::mvc::controller::default_report_path;
using thesis_sim::mvc::controller::parse_app_options;
using thesis_sim::mvc::controller::report_status_string;
using thesis_sim::mvc::controller::write_json_report;
using thesis_sim::mvc::model::fit_hardware_structured_world;
using thesis_sim::mvc::model::load_robot_calibration_profile;
using thesis_sim::mvc::model::RobotCalibrationProfile;
using thesis_sim::mvc::model::fit_simulation_structured_world;
using thesis_sim::mvc::model::hardware_mixed_preset_from_world;
using thesis_sim::mvc::model::hardware_mixed_world_from_preset;
using thesis_sim::mvc::model::make_world_from_mode;
using thesis_sim::mvc::model::mixed_preset_from_world;
using thesis_sim::mvc::model::sanitize_hardware_unstructured_world;
using thesis_sim::mvc::model::scale_world_map;
using thesis_sim::mvc::model::structured_content_bounds;
using thesis_sim::mvc::model::validate_hardware_world;
using thesis_sim::mvc::view::CanvasTransform;
using thesis_sim::mvc::view::HardwareViewerState;
using thesis_sim::mvc::view::apply_style;
using thesis_sim::mvc::view::build_vehicle_snapshot_from_live;
using thesis_sim::mvc::view::distance_sq;
using thesis_sim::mvc::view::draw_goal_marker;
using thesis_sim::mvc::view::draw_lidar_scan;
using thesis_sim::mvc::view::draw_oriented_gate;
using thesis_sim::mvc::view::draw_polyline;
using thesis_sim::mvc::view::draw_structured_road_map;
using thesis_sim::mvc::view::draw_vehicle;
using thesis_sim::mvc::view::draw_viewport_background;
using thesis_sim::mvc::view::draw_world_grid;
using thesis_sim::mvc::view::hardware_vehicle_body_color_for_world;
using thesis_sim::mvc::view::hardware_vehicle_visual_scale_for_world;
using thesis_sim::mvc::view::make_transform;
using thesis_sim::mvc::view::screen_to_world;
using thesis_sim::mvc::view::simulation_vehicle_visual_scale_for_world;
using thesis_sim::mvc::view::world_to_screen;

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

constexpr double kDynamicObstacleSimulationLidarRangeM = 0.75;
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
    int hardware_unstructured_preset = static_cast<int>(UnstructuredMapPreset::HardwareLab);
    int hardware_structured_preset = static_cast<int>(StructuredMapPreset::ValidationRoad);
    int hardware_mixed_preset = 1;
    int hardware_vehicle_model = static_cast<int>(VehicleModelKind::CarLikeBicycle);
    int hardware_localization_policy = static_cast<int>(HardwareLocalizationPolicy::Auto);
    bool hardware_start_matching_enabled = true;
    bool hardware_slam_toolbox_enabled = true;
    std::uint64_t hardware_runtime_settings_revision = 1;
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
    int unstructured_preset = static_cast<int>(UnstructuredMapPreset::IdealValidation);
    int structured_preset = static_cast<int>(StructuredMapPreset::ValidationRoad);
    int mixed_preset = 1;
    int vehicle_model = static_cast<int>(VehicleModelKind::CarLikeBicycle);
    bool ideal_simulation = false;
    bool calibrated_simulation = false;
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
    std::string last_hardware_lidar_reconstruction_csv_path;
    std::string last_hardware_lidar_reconstruction_ply_path;
    std::string last_hardware_lidar_reconstruction_png_path;
    std::string last_hardware_report_error;
    std::string last_hardware_world_path;
    std::string last_hardware_world_error;
    bool hardware_world_sync_pending = false;
    bool hardware_world_sync_ok = false;
    bool hardware_stream_connected_prev = false;
    std::string last_hardware_world_sync_status;
};

struct HardwarePoseQuality {
    std::string source;
    std::string detail;
    double confidence = 0.0;
    ImVec4 color{0.87f, 0.79f, 0.39f, 1.0f};
};

struct HardwarePreflightSummary {
    int passed = 0;
    int total = 0;
    bool ready = false;
    bool warning = false;
};

struct HardwareThesisMetrics {
    bool has_data = false;
    double runtime_s = 0.0;
    double time_to_goal_s = -1.0;
    double final_goal_distance_m = 0.0;
    double min_clearance_m = 0.0;
    double max_cross_track_m = 0.0;
    double max_heading_error_deg = 0.0;
    double avg_speed_mps = 0.0;
    double max_speed_mps = 0.0;
    double encoder_distance_m = 0.0;
    double estimated_path_length_m = 0.0;
    double odom_pose_delta_m = 0.0;
    double structured_time_s = 0.0;
    double gate_time_s = 0.0;
    int passed_gates = 0;
    int switch_events = 0;
    int structured_to_gate_switches = 0;
    int gate_to_structured_switches = 0;
};

std::string hardware_launch_hint(const UiState& ui_state);
bool queue_hardware_runtime_settings(UiState* ui_state,
                                     LiveViewStreamServer* hardware_server,
                                     const char* queued_message,
                                     const char* failure_message);
VehicleModelKind vehicle_model_from_live_name(const std::string& name);

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

double simulation_gate_heading(const PlannerDrivenVehicleSim& sim, std::size_t index) {
    if (!simulation_uses_dynamic_gates(sim) && index < sim.world().gates().size()) {
        return sim.world().gates()[index].heading_hint;
    }
    if (index < sim.gates().size()) {
        return sim.gates()[index].road.PSI_end;
    }
    return 0.0;
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

bool workspace_source_is_hardware(int source);

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
        if (!world.dynamic_obstacles().empty()) {
            return "Mixed Dynamic Obstacle Road";
        }
        if (world.structured_preset() == StructuredMapPreset::IdealCircle && !world.obstacles().empty()) {
            return "Mixed Ideal Hardware-Aligned";
        }
        if (world.structured_preset() == StructuredMapPreset::TankCircuit) {
            return span <= 1.35 ? "Mixed Tank Hardware Lab" : "Mixed Closed Obstacle Road";
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
        case 5:
            return "Tank Hardware Lab";
        case 4:
            return "Dynamic Obstacle Road";
        case 3:
            return "Closed Obstacle Road";
        case 2:
            return "Ideal Hardware-Aligned";
        case 1:
            return "Hardware-Aligned Lab";
        case 0:
        default:
            return "Validation Road/Gates";
    }
}

const char* hardware_mixed_map_preset_name(int preset) {
    switch (preset) {
        case 5:
            return "Tank Hardware Lab";
        case 4:
            return "Dynamic Obstacle Road";
        case 3:
            return "Closed Obstacle Road";
        case 2:
            return "Ideal Hardware-Aligned";
        case 1:
            return "Hardware-Aligned Lab";
        case 0:
        default:
            return "Hardware Runner";
    }
}

const char* hardware_mixed_map_cli_name(int preset) {
    switch (preset) {
        case 5:
            return "tank";
        case 4:
            return "dynamic";
        case 3:
            return "obstacle";
        case 2:
            return "ideal";
        case 1:
            return "hardware_aligned";
        case 0:
        default:
            return "hardware";
    }
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
    ui_state->calibrated_simulation = sim.config().calibrated_conditions;
    ui_state->dynamic_lidar_gates = sim.config().dynamic_lidar_gates;
    ui_state->gate_seed_input = static_cast<int>(sim.gate_seed());
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
    WorldMap hardware_world = make_world_from_mode(
        static_cast<EnvironmentMode>(ui_state->hardware_environment_mode),
        static_cast<UnstructuredMapPreset>(ui_state->hardware_unstructured_preset),
        static_cast<StructuredMapPreset>(ui_state->hardware_structured_preset),
        GateBehaviorMode::Static,
        0);
    hardware_world = apply_hardware_track_scale(*ui_state, std::move(hardware_world));
    ui_state->hardware_editor_world =
        sanitize_hardware_unstructured_world(std::move(hardware_world));
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
        return hardware_mixed_world_from_preset(ui_state.hardware_mixed_preset);
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

std::string append_stem_suffix(const std::string& path, const char* suffix, const char* extension) {
    std::filesystem::path out(path);
    const std::string stem = out.stem().string();
    out.replace_filename(stem + (suffix != nullptr ? suffix : "") + (extension != nullptr ? extension : ""));
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

bool flag_set(std::uint16_t flags, thesis_sim::StatusFlag flag) {
    return (flags & static_cast<std::uint16_t>(flag)) != 0U;
}

bool flag_set(std::uint16_t flags, thesis_sim::MotorFlag flag) {
    return (flags & static_cast<std::uint16_t>(flag)) != 0U;
}

HardwarePoseQuality estimate_hardware_pose_quality(const HardwareViewerState& hardware) {
    HardwarePoseQuality quality;
    const bool has_sample = hardware.frame.has_latest_sample;
    const HardwareTelemetrySample& sample = hardware.frame.latest_sample;
    const bool has_lidar =
        hardware.has_scene &&
        hardware.scene.lidar_enabled &&
        hardware.frame.valid_lidar_samples >= 32;
    const bool imu_ready =
        has_sample && flag_set(sample.controller_status_flags, thesis_sim::StatusFlag::ImuReady);
    const bool encoders_ready =
        has_sample &&
        flag_set(sample.controller_status_flags, thesis_sim::StatusFlag::EncodersReady) &&
        sample.controller_encoder_dt_ms > 0.0;
    const bool motors_ready =
        has_sample &&
        flag_set(sample.controller_status_flags, thesis_sim::StatusFlag::MotorsReady);
    const bool pose_from_lidar =
        hardware.has_scene &&
        hardware.scene.localization_mode.find("LiDAR pose") != std::string::npos;
    const bool pose_encoder_primary =
        hardware.has_scene &&
        hardware.scene.localization_mode.find("encoder/IMU primary") != std::string::npos;

    quality.source =
        pose_from_lidar ? "LiDAR pose correction"
                        : (pose_encoder_primary ? "Encoder/IMU primary" : "Road/IMU constraint");
    double confidence = 0.10;
    if (hardware.frame.telemetry_ready) {
        confidence += 0.18;
    }
    if (imu_ready) {
        confidence += 0.18;
    }
    if (encoders_ready) {
        confidence += 0.24;
    }
    if (motors_ready) {
        confidence += 0.08;
    }
    if (!hardware.has_scene || !hardware.scene.lidar_enabled || has_lidar) {
        confidence += 0.14;
    }
    if (hardware.frame.planner_has_reference) {
        confidence += 0.08;
    }
    if (std::abs(hardware.frame.tracker_cross_track_error) < 0.08 &&
        std::abs(hardware.frame.tracker_heading_error_deg) < 25.0) {
        confidence += 0.08;
    }
    quality.confidence = std::clamp(confidence, 0.0, 1.0);
    quality.color = quality.confidence >= 0.78
                        ? ImVec4(0.48f, 0.88f, 0.62f, 1.0f)
                        : (quality.confidence >= 0.52
                               ? ImVec4(0.87f, 0.79f, 0.39f, 1.0f)
                               : ImVec4(0.95f, 0.40f, 0.34f, 1.0f));
    if (!hardware.frame.telemetry_ready) {
        quality.detail = "waiting for controller telemetry";
    } else if (!imu_ready) {
        quality.detail = "IMU not ready";
    } else if (!encoders_ready) {
        quality.detail = "encoder odometry not ready";
    } else if (hardware.has_scene && hardware.scene.lidar_enabled && !has_lidar) {
        quality.detail = "LiDAR perception is weak";
    } else if (pose_encoder_primary) {
        quality.detail = "pose is driven by odometry; LiDAR feeds gates and obstacles";
    } else {
        quality.detail = hardware.has_scene ? hardware.scene.localization_mode : "waiting for live scene";
    }
    return quality;
}

HardwarePreflightSummary summarize_hardware_preflight(const HardwareViewerState& hardware,
                                                      const UiState& ui_state,
                                                      const LiveViewStreamServer& hardware_server) {
    HardwarePreflightSummary summary;
    auto required = [&](bool ok) {
        ++summary.total;
        if (ok) {
            ++summary.passed;
        }
    };

    const bool has_sample = hardware.frame.has_latest_sample;
    const HardwareTelemetrySample& sample = hardware.frame.latest_sample;
    const VehicleModelKind configured_model =
        static_cast<VehicleModelKind>(ui_state.hardware_vehicle_model);
    const bool live_model_ok =
        hardware.has_scene &&
        vehicle_model_from_live_name(hardware.scene.vehicle_model_name) == configured_model;
    const bool imu_ready =
        has_sample && flag_set(sample.controller_status_flags, thesis_sim::StatusFlag::ImuReady);
    const bool encoders_ready =
        has_sample &&
        flag_set(sample.controller_status_flags, thesis_sim::StatusFlag::EncodersReady) &&
        sample.controller_encoder_dt_ms > 0.0;
    const bool motors_ready =
        has_sample && flag_set(sample.controller_status_flags, thesis_sim::StatusFlag::MotorsReady);
    const bool lidar_ready =
        !hardware.has_scene ||
        !hardware.scene.lidar_enabled ||
        hardware.frame.valid_lidar_samples >= 32;

    required(hardware_server.listening());
    required(hardware_server.connected() || hardware.frame.connected);
    required(hardware.has_scene);
    required(live_model_ok);
    required(!hardware_server.has_pending_world());
    required(hardware.frame.telemetry_ready);
    required(imu_ready);
    required(encoders_ready);
    required(motors_ready);
    required(lidar_ready);
    required(hardware.frame.planner_has_reference || hardware.frame.step_count == 0);

    summary.ready = summary.total > 0 && summary.passed == summary.total;
    summary.warning =
        summary.passed >= std::max(0, summary.total - 2) &&
        !summary.ready;
    return summary;
}

HardwareThesisMetrics summarize_hardware_thesis_metrics(const HardwareViewerState& hardware) {
    HardwareThesisMetrics metrics;
    if (hardware.history.empty()) {
        return metrics;
    }

    metrics.has_data = true;
    metrics.runtime_s = hardware.history.back().time;
    metrics.final_goal_distance_m = hardware.history.back().distance_to_goal;
    metrics.passed_gates =
        static_cast<int>(std::lround(hardware.history.back().passed_gates));
    metrics.min_clearance_m = std::numeric_limits<double>::infinity();

    const double ticks_to_distance =
        hardware.has_scene && hardware.scene.geometry.encoder_ticks_per_revolution > 0
            ? (2.0 * 3.14159265358979323846 * hardware.scene.geometry.wheel_radius) /
                  static_cast<double>(hardware.scene.geometry.encoder_ticks_per_revolution)
            : 0.0;

    double speed_total = 0.0;
    bool previous_gate_mode =
        hardware.history.front().dynamic_gap_gates > 0.5 &&
        hardware.history.front().chosen_gate_index >= 0.0;
    for (size_t i = 0; i < hardware.history.size(); ++i) {
        const HardwareTelemetrySample& sample = hardware.history[i];
        speed_total += sample.speed;
        metrics.max_speed_mps = std::max(metrics.max_speed_mps, sample.speed);
        metrics.max_cross_track_m =
            std::max(metrics.max_cross_track_m, std::abs(sample.tracker_cross_track));
        metrics.max_heading_error_deg =
            std::max(metrics.max_heading_error_deg, std::abs(sample.tracker_heading_error_deg));
        if (sample.min_lidar > 0.0) {
            metrics.min_clearance_m = std::min(metrics.min_clearance_m, sample.min_lidar);
        }
        if (metrics.time_to_goal_s < 0.0 && sample.distance_to_goal <= 1e-3) {
            metrics.time_to_goal_s = sample.time;
        }
        if (ticks_to_distance > 0.0) {
            metrics.encoder_distance_m +=
                0.5 *
                (std::abs(static_cast<double>(sample.controller_left_encoder_delta)) +
                 std::abs(static_cast<double>(sample.controller_right_encoder_delta))) *
                ticks_to_distance;
        }
        if (i > 0) {
            const HardwareTelemetrySample& prev = hardware.history[i - 1];
            metrics.estimated_path_length_m +=
                std::hypot(sample.position_x - prev.position_x,
                           sample.position_y - prev.position_y);
            const double dt = std::max(sample.time - prev.time, 0.0);
            const bool gate_mode =
                sample.dynamic_gap_gates > 0.5 &&
                sample.chosen_gate_index >= 0.0;
            if (gate_mode) {
                metrics.gate_time_s += dt;
            } else {
                metrics.structured_time_s += dt;
            }
            if (gate_mode != previous_gate_mode) {
                ++metrics.switch_events;
                if (gate_mode) {
                    ++metrics.structured_to_gate_switches;
                } else {
                    ++metrics.gate_to_structured_switches;
                }
                previous_gate_mode = gate_mode;
            }
        }
    }
    metrics.avg_speed_mps = speed_total / static_cast<double>(hardware.history.size());
    metrics.odom_pose_delta_m = std::abs(metrics.encoder_distance_m - metrics.estimated_path_length_m);
    if (!std::isfinite(metrics.min_clearance_m)) {
        metrics.min_clearance_m = 0.0;
    }
    if (hardware.frame.goal_reached && metrics.time_to_goal_s < 0.0) {
        metrics.time_to_goal_s = hardware.frame.sim_time;
    }
    return metrics;
}

void append_lidar_reconstruction_frame(HardwareViewerState* hardware) {
    if (hardware == nullptr || !hardware->has_scene || !hardware->scene.lidar_enabled) {
        return;
    }
    const LiveFrameSnapshot& frame = hardware->frame;
    if (frame.step_count == hardware->last_lidar_reconstruction_step || frame.lidar_hits.empty()) {
        return;
    }

    const double sample_time = frame.has_latest_sample ? frame.latest_sample.time : frame.sim_time;
    const Vec2 pose = frame.navigation_position;
    const double pose_yaw = frame.navigation_yaw;
    int beam_index = 0;
    for (const LidarHit& hit : frame.lidar_hits) {
        if (!hit.hit ||
            !(hit.distance > 0.0) ||
            !std::isfinite(hit.distance) ||
            !std::isfinite(hit.point.x) ||
            !std::isfinite(hit.point.y)) {
            ++beam_index;
            continue;
        }
        hardware->lidar_reconstruction.push_back({
            sample_time,
            frame.step_count,
            beam_index,
            pose.x,
            pose.y,
            pose_yaw,
            hit.distance,
            hit.angle,
            hit.point.x,
            hit.point.y,
        });
        ++beam_index;
    }
    hardware->last_lidar_reconstruction_step = frame.step_count;

    constexpr std::size_t kMaxReconstructionPoints = 1000000;
    if (hardware->lidar_reconstruction.size() > kMaxReconstructionPoints) {
        const std::size_t overflow = hardware->lidar_reconstruction.size() - kMaxReconstructionPoints;
        hardware->lidar_reconstruction.erase(
            hardware->lidar_reconstruction.begin(),
            hardware->lidar_reconstruction.begin() + static_cast<std::ptrdiff_t>(overflow));
    }
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
