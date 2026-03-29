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
using thesis_sim::WheelPose;
using thesis_sim::WorldMap;

struct AppOptions {
    bool headless = false;
    int max_steps = 6000;
    EnvironmentMode environment_mode = EnvironmentMode::StructuredRoad;
    UnstructuredMapPreset unstructured_preset = UnstructuredMapPreset::RobotValidation;
    StructuredMapPreset structured_preset = StructuredMapPreset::ValidationRoad;
};

struct CanvasTransform {
    ImVec2 origin;
    float scale = 1.0f;
};

enum WorkspaceSource {
    kWorkspaceSourceSimulation = 0,
    kWorkspaceSourceHardwarePlanner = 1,
    kWorkspaceSourceHardwareSlam = 2,
};

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
    int workspace_source = kWorkspaceSourceSimulation;
    int hardware_listen_port = 9559;
    int hardware_environment_mode = static_cast<int>(EnvironmentMode::StructuredRoad);
    int hardware_unstructured_preset = static_cast<int>(UnstructuredMapPreset::RobotValidation);
    int hardware_structured_preset = static_cast<int>(StructuredMapPreset::ValidationRoad);
    bool show_grid = true;
    bool show_trails = true;
    bool show_lidar_rays = true;
    bool show_lidar_hits = true;
    bool show_gate_labels = true;
    bool show_world_hud = true;
    int gate_seed_input = 7;
    int environment_mode = static_cast<int>(EnvironmentMode::StructuredRoad);
    int unstructured_preset = static_cast<int>(UnstructuredMapPreset::RobotValidation);
    int structured_preset = static_cast<int>(StructuredMapPreset::ValidationRoad);
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
    bool hardware_report_written = false;
    std::string last_hardware_report_path;
    std::string last_hardware_report_error;
    std::string last_hardware_world_path;
    std::string last_hardware_world_error;
    bool hardware_world_sync_pending = false;
    bool hardware_world_sync_ok = false;
    std::string last_hardware_world_sync_status;
};

struct HardwareViewerState {
    bool has_scene = false;
    LiveSceneSnapshot scene;
    LiveFrameSnapshot frame;
    std::vector<thesis_sim::HardwareTelemetrySample> history;
};

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
                              std::uint32_t gate_seed);
bool workspace_source_is_hardware(int source);

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

void metric_card(const char* id,
                 const char* label,
                 const char* value,
                 const char* detail,
                 const ImVec4& accent,
                 float height = 76.0f) {
    ImGui::PushID(id);
    ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 12.0f);
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
            12.0f);

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
                             12.0f);
    draw_list->AddRect(pos,
                       ImVec2(pos.x + width, pos.y + height),
                       IM_COL32(86, 99, 108, 200),
                       12.0f,
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
                              std::uint32_t gate_seed);

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

void sync_ui_state_from_sim(UiState* ui_state, const PlannerDrivenVehicleSim& sim) {
    if (ui_state == nullptr) {
        return;
    }
    ui_state->environment_mode = static_cast<int>(sim.environment_mode());
    ui_state->unstructured_preset = static_cast<int>(sim.world().unstructured_preset());
    ui_state->structured_preset = static_cast<int>(sim.world().structured_preset());
    ui_state->gate_seed_input = static_cast<int>(sim.gate_seed());
    ui_state->scenario_editor_world = sim.world();
    ui_state->scenario_editor_dirty = false;
    ui_state->selected_editor_handle = {};
    ui_state->active_drag_handle = {};
    ui_state->drag_offset = {};
    ui_state->report_written = false;
    ui_state->last_report_path.clear();
    ui_state->hardware_report_written = false;
    ui_state->last_hardware_report_path.clear();
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
    return make_world_from_mode(selected_mode, selected_preset, selected_structured_preset, sim.gate_behavior(), sim.gate_seed());
}

WorldMap hardware_world_from_ui_selection(const UiState& ui_state) {
    const EnvironmentMode selected_mode = static_cast<EnvironmentMode>(ui_state.hardware_environment_mode);
    const UnstructuredMapPreset selected_preset = static_cast<UnstructuredMapPreset>(ui_state.hardware_unstructured_preset);
    const StructuredMapPreset selected_structured_preset =
        static_cast<StructuredMapPreset>(ui_state.hardware_structured_preset);
    if (selected_mode == EnvironmentMode::UnstructuredGates &&
        selected_preset == UnstructuredMapPreset::Custom &&
        ui_state.hardware_editor_world.environment_mode() == EnvironmentMode::UnstructuredGates) {
        WorldMap custom = ui_state.hardware_editor_world;
        custom.finalize_editor_changes();
        custom.set_gate_behavior(GateBehaviorMode::Static, 0);
        return custom;
    }
    if (selected_mode == EnvironmentMode::StructuredRoad &&
        selected_structured_preset == StructuredMapPreset::Custom &&
        ui_state.hardware_editor_world.environment_mode() == EnvironmentMode::StructuredRoad) {
        WorldMap custom = ui_state.hardware_editor_world;
        custom.finalize_editor_changes();
        return custom;
    }
    return make_world_from_mode(selected_mode, selected_preset, selected_structured_preset, GateBehaviorMode::Static, 0);
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
         << (sim.environment_mode() == EnvironmentMode::StructuredRoad ? "structured" : "unstructured");
    const std::string preset_name = sim.environment_mode() == EnvironmentMode::StructuredRoad
                                        ? thesis_sim::structured_map_preset_name(sim.world().structured_preset())
                                        : thesis_sim::unstructured_map_preset_name(sim.world().unstructured_preset());
    name << "_" << slugify(preset_name);
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
        environment == EnvironmentMode::StructuredRoad
            ? (hardware.has_scene
                   ? thesis_sim::structured_map_preset_name(hardware.scene.world.structured_preset())
                   : thesis_sim::structured_map_preset_name(
                         static_cast<StructuredMapPreset>(ui_state.hardware_structured_preset)))
            : (hardware.has_scene
                   ? thesis_sim::unstructured_map_preset_name(hardware.scene.world.unstructured_preset())
                   : thesis_sim::unstructured_map_preset_name(
                         static_cast<UnstructuredMapPreset>(ui_state.hardware_unstructured_preset)));

    std::ostringstream name;
    name << "thesis_hardware_"
         << (environment == EnvironmentMode::StructuredRoad ? "structured" : "unstructured")
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
        environment == EnvironmentMode::StructuredRoad
            ? thesis_sim::structured_map_preset_name(static_cast<StructuredMapPreset>(ui_state.hardware_structured_preset))
            : thesis_sim::unstructured_map_preset_name(static_cast<UnstructuredMapPreset>(ui_state.hardware_unstructured_preset));

    std::ostringstream name;
    name << "thesis_hardware_world_"
         << (environment == EnvironmentMode::StructuredRoad ? "structured" : "unstructured")
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
        << ",\"wheel_radius\":" << geometry.wheel_radius
        << ",\"max_curvature\":" << geometry.max_curvature
        << ",\"max_linear_speed\":" << geometry.max_linear_speed
        << ",\"max_yaw_rate\":" << geometry.max_yaw_rate
        << ",\"max_accel\":" << geometry.max_accel
        << ",\"max_decel\":" << geometry.max_decel
        << ",\"max_pwm\":" << geometry.max_pwm
        << ",\"min_effective_pwm\":" << geometry.min_effective_pwm
        << ",\"wheel_speed_to_pwm_gain\":" << geometry.wheel_speed_to_pwm_gain
        << ",\"wheel_speed_to_pwm_bias\":" << geometry.wheel_speed_to_pwm_bias
        << ",\"left_pwm_scale\":" << geometry.left_pwm_scale
        << ",\"right_pwm_scale\":" << geometry.right_pwm_scale
        << ",\"linear_feedback_gain\":" << geometry.linear_feedback_gain
        << ",\"yaw_feedback_gain\":" << geometry.yaw_feedback_gain
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
        hardware_server.connected()
            ? (hardware.frame.goal_reached ? "goal_reached" : (hardware.frame.safety_stop_active ? "safety_stop" : "live"))
            : (hardware_server.listening() ? "listening" : "idle");

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

    int passed_gates = 0;
    for (const auto& gate : sim.gates()) {
        if (gate.passed) {
            ++passed_gates;
        }
    }

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
    out << "    \"range_sensor_profile\": \"" << json_escape(thesis_sim::range_sensor_profile_name(config.range_sensor_profile)) << "\",\n";
    out << "    \"lidar_beams\": " << sim.active_lidar_beams() << ",\n";
    out << "    \"lidar_fov_rad\": " << sim.active_lidar_fov_rad() << ",\n";
    out << "    \"lidar_range\": " << sim.active_lidar_range() << ",\n";
    out << "    \"vehicle_model\": \"" << json_escape(thesis_sim::vehicle_model_kind_name(sim.vehicle_model_kind())) << "\",\n";
    out << "    \"tracking_layer\": \"" << json_escape(thesis_sim::tracking_controller_mode_name(sim.tracking_controller_mode())) << "\"\n";
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
            << ",\"speed\":" << sample.speed
            << ",\"accel\":" << sample.accel
            << ",\"curvature\":" << sample.curvature
            << ",\"steer_angle\":" << sample.steer_angle
            << ",\"target_steer_angle\":" << sample.target_steer_angle
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
            << "}";
    }
    out << "\n  ]\n";
    out << "}\n";
    return true;
}

AppOptions parse_args(int argc, char** argv) {
    AppOptions options;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--headless") {
            options.headless = true;
        } else if (arg == "--scenario" && i + 1 < argc) {
            const std::string value = argv[++i];
            options.environment_mode = (value == "structured") ? EnvironmentMode::StructuredRoad
                                                               : EnvironmentMode::UnstructuredGates;
        } else if (arg == "--scenario=structured") {
            options.environment_mode = EnvironmentMode::StructuredRoad;
        } else if (arg == "--scenario=unstructured") {
            options.environment_mode = EnvironmentMode::UnstructuredGates;
        } else if (arg == "--unstructured-map" && i + 1 < argc) {
            const std::string value = argv[++i];
            if (value == "tight") {
                options.unstructured_preset = UnstructuredMapPreset::TightCorridor;
            } else if (value == "slalom") {
                options.unstructured_preset = UnstructuredMapPreset::WideSlalom;
            } else if (value == "lower") {
                options.unstructured_preset = UnstructuredMapPreset::LowerBypass;
            } else {
                options.unstructured_preset = UnstructuredMapPreset::RobotValidation;
            }
        } else if (arg == "--structured-map" && i + 1 < argc) {
            const std::string value = argv[++i];
            if (value == "circle") {
                options.structured_preset = StructuredMapPreset::CircleLoop;
            } else if (value == "zigzag") {
                options.structured_preset = StructuredMapPreset::ZigZag;
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
            } else {
                options.unstructured_preset = UnstructuredMapPreset::RobotValidation;
            }
        } else if (arg.rfind("--structured-map=", 0) == 0) {
            const std::string value = arg.substr(std::strlen("--structured-map="));
            if (value == "circle") {
                options.structured_preset = StructuredMapPreset::CircleLoop;
            } else if (value == "zigzag") {
                options.structured_preset = StructuredMapPreset::ZigZag;
            } else {
                options.structured_preset = StructuredMapPreset::ValidationRoad;
            }
        } else if (arg == "--max-steps" && i + 1 < argc) {
            options.max_steps = std::atoi(argv[++i]);
        } else if (arg.rfind("--max-steps=", 0) == 0) {
            options.max_steps = std::atoi(arg.substr(std::strlen("--max-steps=")).c_str());
        }
    }
    options.max_steps = std::max(options.max_steps, 1);
    return options;
}

WorldMap make_world_from_mode(EnvironmentMode mode,
                              UnstructuredMapPreset preset,
                              StructuredMapPreset structured_preset,
                              GateBehaviorMode gate_behavior,
                              std::uint32_t gate_seed) {
    if (mode == EnvironmentMode::StructuredRoad) {
        return WorldMap::structured_demo(structured_preset);
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
    colors[ImGuiCol_Button] = ImVec4(0.14f, 0.35f, 0.42f, 1.0f);
    colors[ImGuiCol_ButtonHovered] = ImVec4(0.20f, 0.49f, 0.57f, 1.0f);
    colors[ImGuiCol_ButtonActive] = ImVec4(0.12f, 0.29f, 0.35f, 1.0f);
    colors[ImGuiCol_Header] = ImVec4(0.13f, 0.31f, 0.37f, 1.0f);
    colors[ImGuiCol_HeaderHovered] = ImVec4(0.20f, 0.42f, 0.49f, 1.0f);
    colors[ImGuiCol_HeaderActive] = ImVec4(0.16f, 0.36f, 0.42f, 1.0f);
    colors[ImGuiCol_TitleBg] = ImVec4(0.07f, 0.09f, 0.12f, 1.0f);
    colors[ImGuiCol_TitleBgActive] = ImVec4(0.09f, 0.13f, 0.16f, 1.0f);
    colors[ImGuiCol_Tab] = ImVec4(0.09f, 0.12f, 0.15f, 1.0f);
    colors[ImGuiCol_TabHovered] = ImVec4(0.16f, 0.31f, 0.37f, 1.0f);
    colors[ImGuiCol_TabActive] = ImVec4(0.13f, 0.22f, 0.27f, 1.0f);
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

void draw_vehicle(ImDrawList* draw_list,
                  const CanvasTransform& tx,
                  const VehicleSnapshot& vehicle,
                  const thesis_sim::VehicleGeometry& geometry,
                  float visual_scale = 1.0f) {
    std::array<ImVec2, 4> body{};
    for (size_t i = 0; i < body.size(); ++i) {
        const Vec2 scaled_corner{
            vehicle.position.x + (vehicle.body_corners[i].x - vehicle.position.x) * visual_scale,
            vehicle.position.y + (vehicle.body_corners[i].y - vehicle.position.y) * visual_scale,
        };
        body[i] = world_to_screen(tx, scaled_corner);
    }
    draw_list->AddConvexPolyFilled(body.data(), static_cast<int>(body.size()), kColorBody);
    draw_list->AddPolyline(body.data(), static_cast<int>(body.size()), IM_COL32(20, 22, 26, 255), ImDrawFlags_Closed, 2.5f);

    for (const WheelPose& wheel : vehicle.wheels) {
        const auto wheel_box = thesis_sim::make_box_corners(
            wheel.center,
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
    return source == kWorkspaceSourceHardwarePlanner || source == kWorkspaceSourceHardwareSlam;
}

bool workspace_source_expects_slam(int source) {
    return source == kWorkspaceSourceHardwareSlam;
}

const char* workspace_source_label(int source) {
    switch (source) {
        case kWorkspaceSourceHardwarePlanner:
            return "Hardware Planner";
        case kWorkspaceSourceHardwareSlam:
            return "Hardware SLAM";
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
        case StructuredMapPreset::Custom:
            return "custom";
        case StructuredMapPreset::ValidationRoad:
        default:
            return "validation";
    }
}

std::string stream_profile_label(const std::string& stream_profile, int workspace_source) {
    if (stream_profile == "slam") {
        return "SLAM";
    }
    if (stream_profile == "planner") {
        return "Planner";
    }
    return workspace_source_expects_slam(workspace_source) ? "SLAM" : "Planner";
}

std::string hardware_launch_hint(const UiState& ui_state) {
    std::ostringstream cmd;
    const bool structured =
        static_cast<EnvironmentMode>(ui_state.hardware_environment_mode) == EnvironmentMode::StructuredRoad;
    const bool custom_world =
        structured
            ? static_cast<StructuredMapPreset>(ui_state.hardware_structured_preset) == StructuredMapPreset::Custom
            : static_cast<UnstructuredMapPreset>(ui_state.hardware_unstructured_preset) == UnstructuredMapPreset::Custom;
    if (workspace_source_expects_slam(ui_state.workspace_source)) {
        cmd << "thesis_robot_smoke_test"
            << " --stream-host <pc-ip>"
            << " --stream-port " << std::max(ui_state.hardware_listen_port, 1)
            << " --scenario " << (structured ? "structured" : "unstructured");
        if (custom_world) {
            cmd << " --world-file <copied-custom-map.thmap>";
        } else if (structured) {
            cmd << " --structured-map "
                << structured_map_cli_name(static_cast<StructuredMapPreset>(ui_state.hardware_structured_preset));
        } else {
            cmd << " --unstructured-map "
                << unstructured_map_cli_name(static_cast<UnstructuredMapPreset>(ui_state.hardware_unstructured_preset));
        }
        cmd << " --infinite";
    } else {
        cmd << "thesis_robot_runner"
            << " --stream-host <pc-ip>"
            << " --stream-port " << std::max(ui_state.hardware_listen_port, 1)
            << " --scenario " << (structured ? "structured" : "unstructured");
        if (!custom_world && structured) {
            cmd << " --structured-map "
                << structured_map_cli_name(static_cast<StructuredMapPreset>(ui_state.hardware_structured_preset));
        } else if (!custom_world) {
            cmd << " --unstructured-map "
                << unstructured_map_cli_name(static_cast<UnstructuredMapPreset>(ui_state.hardware_unstructured_preset));
        }
    }
    return cmd.str();
}

void load_hardware_editor_from_selection(UiState* ui_state) {
    if (ui_state == nullptr) {
        return;
    }
    ui_state->hardware_editor_world = make_world_from_mode(
        static_cast<EnvironmentMode>(ui_state->hardware_environment_mode),
        static_cast<UnstructuredMapPreset>(ui_state->hardware_unstructured_preset),
        static_cast<StructuredMapPreset>(ui_state->hardware_structured_preset),
        GateBehaviorMode::Static,
        0);
    ui_state->hardware_editor_dirty = false;
    reset_editor_interaction(ui_state);
}

void render_source_selector(UiState* ui_state, LiveViewStreamServer* hardware_server) {
    if (ui_state == nullptr) {
        return;
    }

    const char* source_items[] = {"Simulation", "Hardware Planner", "Hardware SLAM"};
    int source = ui_state->workspace_source;
    if (ImGui::Combo("Workspace Source", &source, source_items, IM_ARRAYSIZE(source_items))) {
        ui_state->workspace_source = std::clamp(source, static_cast<int>(kWorkspaceSourceSimulation), static_cast<int>(kWorkspaceSourceHardwareSlam));
        if (workspace_source_is_hardware(ui_state->workspace_source) &&
            hardware_server != nullptr &&
            !hardware_server->listening() &&
            ui_state->hardware_listen_port > 0) {
            hardware_server->start(static_cast<std::uint16_t>(ui_state->hardware_listen_port));
        }
    }
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

    const char* active_target = sim.environment_mode() == EnvironmentMode::UnstructuredGates
                                    ? (sim.chosen_gate_index() >= 0 ? sim.world().gates()[sim.chosen_gate_index()].name.c_str() : "none")
                                    : thesis_sim::structured_map_preset_name(sim.world().structured_preset());

    char speed_buf[32];
    char goal_buf[32];
    char tracking_buf[32];
    char target_buf[48];
    std::snprintf(speed_buf, sizeof(speed_buf), "%.2f m/s", sim.vehicle().speed);
    std::snprintf(goal_buf, sizeof(goal_buf), "%.2f m", sim.distance_to_goal());
    std::snprintf(tracking_buf, sizeof(tracking_buf), "%.2f m / %.1f deg", sim.tracker_cross_track_error(), sim.tracker_heading_error_deg());
    std::snprintf(target_buf, sizeof(target_buf), "%s", active_target);

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
                            sim.environment_mode() == EnvironmentMode::UnstructuredGates ? "Active planner gate" : "Active structured loop",
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
        for (int gx = static_cast<int>(bounds.min_x); gx <= static_cast<int>(bounds.max_x); gx += 2) {
            draw_list->AddLine(world_to_screen(tx, {static_cast<double>(gx), bounds.min_y}),
                               world_to_screen(tx, {static_cast<double>(gx), bounds.max_y}),
                               kColorGrid, 1.0f);
        }
        for (int gy = static_cast<int>(bounds.min_y); gy <= static_cast<int>(bounds.max_y); gy += 2) {
            draw_list->AddLine(world_to_screen(tx, {bounds.min_x, static_cast<double>(gy)}),
                               world_to_screen(tx, {bounds.max_x, static_cast<double>(gy)}),
                               kColorGrid, 1.0f);
        }
    }

    for (const Rect& obstacle : sim.world().obstacles()) {
        draw_list->AddRectFilled(world_to_screen(tx, {obstacle.min_x, obstacle.max_y}),
                                 world_to_screen(tx, {obstacle.max_x, obstacle.min_y}),
                                 kColorObstacle, 4.0f);
    }

    if (!sim.world().road_centerline().empty()) {
        draw_polyline(draw_list, tx, sim.world().road_centerline(), IM_COL32(113, 210, 255, 180), 4.0f);
    }

    if (ui_state->show_trails) {
        draw_polyline(draw_list, tx, sim.trail(), kColorTrail, 2.5f);
        draw_polyline(draw_list, tx, sim.estimated_trail(), kColorEstimateTrail, 1.8f);
        draw_polyline(draw_list, tx, sim.planned_trajectory(), kColorTrajectory, 3.0f);
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
        const GateSpec& spec = sim.world().gates()[i];
        const auto screen_pos = world_to_screen(tx, spec.position);
        const bool visible =
            std::find(sim.visible_gate_indices().begin(), sim.visible_gate_indices().end(), static_cast<int>(i)) !=
            sim.visible_gate_indices().end();

        ImU32 color = spec.final ? kColorGoal : kColorGate;
        if (sim.gates()[i].passed) {
            color = kColorGatePassed;
        } else if (visible) {
            color = kColorGateVisible;
        }
        if (static_cast<int>(i) == sim.chosen_gate_index()) {
            color = IM_COL32(255, 233, 118, 255);
        }

        const float radius = spec.final ? 7.0f : 5.0f;
        draw_list->AddCircleFilled(screen_pos, radius, color);
        draw_list->AddCircle(screen_pos, radius + 3.0f, IM_COL32(245, 246, 240, 180), 0, 1.5f);
        if (ui_state->show_gate_labels) {
            draw_list->AddText(ImVec2(screen_pos.x + 6.0f, screen_pos.y - 12.0f), IM_COL32(222, 227, 230, 255), spec.name.c_str());
        }
    }

    draw_vehicle(draw_list, tx, sim.vehicle(), sim.geometry(), 2.60f);
    const Vec2 nav_pos = sim.navigation_position();
    const Vec2 nav_nose{
        nav_pos.x + std::cos(sim.navigation_yaw()) * sim.geometry().body_length * 0.85,
        nav_pos.y + std::sin(sim.navigation_yaw()) * sim.geometry().body_length * 0.85,
    };
    draw_list->AddCircle(world_to_screen(tx, nav_pos), 11.0f, kColorEstimateTrail, 0, 2.8f);
    draw_list->AddLine(world_to_screen(tx, nav_pos), world_to_screen(tx, nav_nose), kColorEstimateTrail, 3.2f);

    if (ui_state->show_world_hud) {
        char hud_line[96];
        std::vector<OverlayLine> top_left = {
            {"Simulation viewport", IM_COL32(240, 243, 235, 255)},
            {thesis_sim::environment_mode_name(sim.environment_mode()), IM_COL32(170, 179, 185, 255)},
            {sim.environment_mode() == EnvironmentMode::UnstructuredGates
                 ? thesis_sim::unstructured_map_preset_name(sim.world().unstructured_preset())
                 : thesis_sim::structured_map_preset_name(sim.world().structured_preset()),
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
        if (sim.environment_mode() == EnvironmentMode::UnstructuredGates) {
            std::snprintf(hud_line, sizeof(hud_line), "gate %s | visible %d", active_target, static_cast<int>(sim.visible_gate_indices().size()));
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

    const TelemetrySample& latest = history.back();
    char speed_buf[32];
    char goal_buf[32];
    char lidar_buf[32];
    char err_buf[32];
    std::snprintf(speed_buf, sizeof(speed_buf), "%.2f m/s", latest.speed);
    std::snprintf(goal_buf, sizeof(goal_buf), "%.2f m", latest.distance_to_goal);
    std::snprintf(lidar_buf, sizeof(lidar_buf), "%.2f m", latest.min_lidar);
    std::snprintf(err_buf, sizeof(err_buf), "%.2f / %.2f", latest.nav_xy_error, latest.nav_yaw_error_deg);

    ImGui::TextWrapped("Telemetry is grouped by intent so you can move from mission health to low-level diagnosis without leaving the graphs workspace.");
    ImGui::SeparatorText("Telemetry Overview");
    if (ImGui::BeginTable("GraphSummary", 4, ImGuiTableFlags_SizingStretchSame)) {
        ImGui::TableNextColumn();
        metric_card("graph_speed", "Speed", speed_buf, "Latest chassis speed", ImVec4(0.43f, 0.82f, 0.96f, 1.0f), 64.0f);
        ImGui::TableNextColumn();
        metric_card("graph_goal", "Goal", goal_buf, "Remaining mission distance", ImVec4(0.99f, 0.70f, 0.32f, 1.0f), 64.0f);
        ImGui::TableNextColumn();
        metric_card("graph_lidar", "LiDAR Min", lidar_buf, "Nearest observed return", ImVec4(0.48f, 0.88f, 0.62f, 1.0f), 64.0f);
        ImGui::TableNextColumn();
        metric_card("graph_ekf", "EKF err", err_buf, "xy error / yaw error deg", ImVec4(0.97f, 0.89f, 0.45f, 1.0f), 64.0f);
        ImGui::EndTable();
    }
    ImGui::SeparatorText("Plots");

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
        if (ImGui::BeginTabItem("Overview")) {
            ImGui::TextWrapped("Overview of vehicle evolution, planner output, distance trends and total compute cost.");
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

        if (ImGui::BeginTabItem("Drivetrain")) {
            ImGui::TextWrapped("Actuation and wheel-side telemetry. This tab is useful to compare what the controller wants against what the vehicle model actually does.");
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
            ImGui::TextWrapped("State-estimation quality and sensor-side load. Useful to see whether the lag comes from scan handling or from EKF correction.");
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
            ImGui::TextWrapped("Low-level execution of the selected planner trajectory. Compare steering, follower commands and path errors on the same time base.");
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

    const LiveFrameSnapshot& frame = hardware.frame;
    const WorldMap preview_world = hardware.has_scene ? hardware.scene.world : hardware_world_from_ui_selection(*ui_state);
    const WorldMap& world = preview_world;
    const char* active_target = "none";
    if (world.environment_mode() == EnvironmentMode::UnstructuredGates) {
        if (hardware.has_scene && frame.chosen_gate_index >= 0 && frame.chosen_gate_index < static_cast<int>(frame.gates.size())) {
            active_target = frame.gates[static_cast<std::size_t>(frame.chosen_gate_index)].spec.name.c_str();
        } else if (hardware.has_scene && hardware.scene.stream_profile == "slam") {
            active_target = thesis_sim::unstructured_map_preset_name(world.unstructured_preset());
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
    std::snprintf(speed_buf, sizeof(speed_buf), "%.2f m/s", frame.vehicle.speed);
    std::snprintf(goal_buf, sizeof(goal_buf), "%.2f m", frame.distance_to_goal);
    std::snprintf(tracking_buf, sizeof(tracking_buf), "%.2f m / %.1f deg", frame.tracker_cross_track_error, frame.tracker_heading_error_deg);
    std::snprintf(target_buf, sizeof(target_buf), "%s", active_target);

    if (ImGui::BeginChild("HardwareWorldToolbar", ImVec2(0.0f, 102.0f), true)) {
        if (ImGui::BeginTable("HardwareWorldToolbarLayout", 2, ImGuiTableFlags_SizingStretchProp)) {
            ImGui::TableSetupColumn("WorldMetrics", ImGuiTableColumnFlags_WidthStretch, 1.0f);
            ImGui::TableSetupColumn("WorldControls", ImGuiTableColumnFlags_WidthFixed, 320.0f);

            ImGui::TableNextColumn();
            if (ImGui::BeginTable("HardwareMetricCards", 4, ImGuiTableFlags_SizingStretchSame)) {
                ImGui::TableNextColumn();
                metric_card("hw_world_speed", "Vehicle", speed_buf, "Estimated chassis speed", ImVec4(0.43f, 0.82f, 0.96f, 1.0f), 64.0f);
                ImGui::TableNextColumn();
                metric_card("hw_world_goal", "Goal", goal_buf, "Distance remaining", ImVec4(0.99f, 0.70f, 0.32f, 1.0f), 64.0f);
                ImGui::TableNextColumn();
                metric_card("hw_world_tracking", "Tracking", tracking_buf, "cte / heading", ImVec4(0.97f, 0.89f, 0.45f, 1.0f), 64.0f);
                ImGui::TableNextColumn();
                metric_card("hw_world_target", "Target", target_buf,
                            world.environment_mode() == EnvironmentMode::UnstructuredGates
                                ? ((hardware.has_scene && hardware.scene.stream_profile == "slam")
                                       ? "Selected unstructured SLAM context"
                                       : (hardware.has_scene ? "Active planner gate" : "Local hardware preview target"))
                                : ((hardware.has_scene && hardware.scene.stream_profile == "slam")
                                       ? "Selected structured SLAM context"
                                       : (hardware.has_scene ? "Active structured loop" : "Local hardware preview loop")),
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
            ImGui::TextWrapped(
                hardware.has_scene
                    ? "The hardware viewport reuses the same scene diagnostics, but all poses, LiDAR and trajectories now come from the remote robot stream."
                    : "No live stream yet: this viewport is previewing the selected hardware map locally, so you can edit it before launching the runner on the Raspberry.");
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
        for (int gx = static_cast<int>(bounds.min_x); gx <= static_cast<int>(bounds.max_x); gx += 2) {
            draw_list->AddLine(world_to_screen(tx, {static_cast<double>(gx), bounds.min_y}),
                               world_to_screen(tx, {static_cast<double>(gx), bounds.max_y}),
                               kColorGrid, 1.0f);
        }
        for (int gy = static_cast<int>(bounds.min_y); gy <= static_cast<int>(bounds.max_y); gy += 2) {
            draw_list->AddLine(world_to_screen(tx, {bounds.min_x, static_cast<double>(gy)}),
                               world_to_screen(tx, {bounds.max_x, static_cast<double>(gy)}),
                               kColorGrid, 1.0f);
        }
    }

    for (const Rect& obstacle : world.obstacles()) {
        draw_list->AddRectFilled(world_to_screen(tx, {obstacle.min_x, obstacle.max_y}),
                                 world_to_screen(tx, {obstacle.max_x, obstacle.min_y}),
                                 kColorObstacle, 4.0f);
    }
    if (!world.road_centerline().empty()) {
        draw_polyline(draw_list, tx, world.road_centerline(), IM_COL32(113, 210, 255, 180), 4.0f);
    }

    if (hardware.has_scene && ui_state->show_trails) {
        draw_polyline(draw_list, tx, hardware.frame.trail, kColorEstimateTrail, 2.5f);
        draw_polyline(draw_list, tx, hardware.frame.planned_trajectory, kColorTrajectory, 3.0f);
    }

    if (hardware.has_scene && ui_state->show_lidar_hits && !frame.slam_points.empty()) {
        const size_t slam_stride = frame.slam_points.size() > 2800 ? (frame.slam_points.size() / 2800) + 1 : 1;
        for (size_t i = 0; i < frame.slam_points.size(); i += slam_stride) {
            const ImVec2 point_screen = world_to_screen(tx, frame.slam_points[i]);
            draw_list->AddCircleFilled(point_screen, 2.2f, IM_COL32(132, 255, 196, 92));
        }
    }

    if (hardware.has_scene && hardware.scene.lidar_enabled && (ui_state->show_lidar_rays || ui_state->show_lidar_hits)) {
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

    if (hardware.has_scene) {
        for (size_t i = 0; i < frame.gates.size(); ++i) {
            const LiveGateFrame& gate = frame.gates[i];
            const ImVec2 screen_pos = world_to_screen(tx, gate.spec.position);
            const bool visible =
                std::find(frame.visible_gate_indices.begin(), frame.visible_gate_indices.end(), static_cast<int>(i)) !=
                frame.visible_gate_indices.end();

            ImU32 color = gate.spec.final ? kColorGoal : kColorGate;
            if (gate.passed) {
                color = kColorGatePassed;
            } else if (visible) {
                color = kColorGateVisible;
            }
            if (static_cast<int>(i) == frame.chosen_gate_index) {
                color = IM_COL32(255, 233, 118, 255);
            }

            const float radius = gate.spec.final ? 7.0f : 5.0f;
            draw_list->AddCircleFilled(screen_pos, radius, color);
            draw_list->AddCircle(screen_pos, radius + 3.0f, IM_COL32(245, 246, 240, 180), 0, 1.5f);
            if (ui_state->show_gate_labels) {
                draw_list->AddText(ImVec2(screen_pos.x + 6.0f, screen_pos.y - 12.0f), IM_COL32(222, 227, 230, 255), gate.spec.name.c_str());
            }
        }
    }

    if (hardware.has_scene) {
        const VehicleSnapshot vehicle = build_vehicle_snapshot_from_live(frame.vehicle, hardware.scene.geometry);
        draw_vehicle(draw_list, tx, vehicle, hardware.scene.geometry, 2.60f);
    }

    if (ui_state->show_world_hud) {
        char hud_line[96];
        std::vector<OverlayLine> top_left = {
            {hardware.has_scene
                 ? (hardware.scene.stream_profile == "slam" ? "Hardware SLAM viewport" : "Hardware viewport")
                 : "Hardware preview viewport",
             IM_COL32(240, 243, 235, 255)},
            {thesis_sim::environment_mode_name(world.environment_mode()), IM_COL32(170, 179, 185, 255)},
            {world.environment_mode() == EnvironmentMode::UnstructuredGates
                 ? thesis_sim::unstructured_map_preset_name(world.unstructured_preset())
                 : thesis_sim::structured_map_preset_name(world.structured_preset()),
             IM_COL32(170, 179, 185, 255)},
            {hardware.has_scene ? stream_profile_label(hardware.scene.stream_profile, ui_state->workspace_source)
                                : "Local preview",
             IM_COL32(170, 179, 185, 255)},
            {hardware.has_scene ? hardware.scene.range_sensor_name : "Edit and export before Raspberry launch",
             IM_COL32(170, 179, 185, 255)},
        };
        draw_overlay_panel(draw_list, ImVec2(canvas_pos.x + 16.0f, canvas_pos.y + 16.0f), 228.0f, top_left);

        std::snprintf(hud_line, sizeof(hud_line), "speed %.2f m/s", frame.vehicle.speed);
        std::vector<OverlayLine> top_right = {
            {hardware.has_scene
                 ? (frame.goal_reached ? "Mission complete"
                                       : (frame.safety_stop_active ? "Safety stop active"
                                                                   : (hardware.scene.stream_profile == "slam" ? "SLAM stream live" : "Hardware stream live")))
                 : "Preview ready",
             hardware.has_scene
                 ? (frame.goal_reached ? IM_COL32(124, 238, 151, 255)
                                       : (frame.safety_stop_active ? IM_COL32(255, 124, 102, 255) : IM_COL32(255, 221, 113, 255)))
                 : IM_COL32(132, 214, 255, 255)},
            {hud_line, IM_COL32(240, 243, 235, 255)},
        };
        std::snprintf(hud_line, sizeof(hud_line), "goal %.2f m", frame.distance_to_goal);
        top_right.push_back({hud_line, IM_COL32(170, 179, 185, 255)});
        if (world.environment_mode() == EnvironmentMode::UnstructuredGates) {
            if (hardware.has_scene && hardware.scene.stream_profile == "slam") {
                std::snprintf(hud_line, sizeof(hud_line), "context %s | map pts %d", active_target, static_cast<int>(frame.slam_points.size()));
            } else if (hardware.has_scene) {
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
            {"orange: estimated hardware trail", kColorEstimateTrail},
            {"yellow: selected planner trajectory", kColorTrajectory},
        };
        if (hardware.has_scene && hardware.scene.lidar_enabled && ui_state->show_lidar_rays) {
            legend.push_back({"green: LiDAR hit rays", kColorLidar});
        }
        if (hardware.has_scene && hardware.scene.lidar_enabled && ui_state->show_lidar_hits) {
            legend.push_back({"lime/cyan: LiDAR collision points", kColorLidarHit});
        }
        if (hardware.has_scene && !frame.slam_points.empty() && ui_state->show_lidar_hits) {
            legend.push_back({"mint: accumulated LiDAR map", IM_COL32(132, 255, 196, 200)});
        }
        if (!hardware.has_scene && ui_state->map_editor_enabled) {
            legend.push_back({"blue: editable preview road", kColorEditorOverlay});
        }
        const float legend_height = 20.0f + static_cast<float>(legend.size()) * (ImGui::GetFontSize() + 5.0f);
        draw_overlay_panel(draw_list,
                           ImVec2(canvas_pos.x + 16.0f, canvas_pos.y + canvas_size.y - legend_height),
                           304.0f,
                           legend);
    }

    if (!hardware.has_scene && ui_state->map_editor_enabled) {
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

    const HardwareTelemetrySample& latest = hardware.history.back();
    char speed_buf[32];
    char goal_buf[32];
    char lidar_buf[32];
    char err_buf[32];
    std::snprintf(speed_buf, sizeof(speed_buf), "%.2f m/s", latest.speed);
    std::snprintf(goal_buf, sizeof(goal_buf), "%.2f m", latest.distance_to_goal);
    std::snprintf(lidar_buf, sizeof(lidar_buf), "%.2f / %.2f m", latest.min_lidar, latest.front_lidar);
    std::snprintf(err_buf, sizeof(err_buf), "%.2f / %.2f", latest.tracker_cross_track, latest.tracker_heading_error_deg);

    ImGui::TextWrapped("Hardware telemetry is built live from the incoming robot stream. The history lives on the workstation, so the Raspberry only needs to send the newest sample.");
    ImGui::SeparatorText("Telemetry Overview");
    if (ImGui::BeginTable("HardwareGraphSummary", 4, ImGuiTableFlags_SizingStretchSame)) {
        ImGui::TableNextColumn();
        metric_card("hw_graph_speed", "Speed", speed_buf, "Latest chassis speed", ImVec4(0.43f, 0.82f, 0.96f, 1.0f), 64.0f);
        ImGui::TableNextColumn();
        metric_card("hw_graph_goal", "Goal", goal_buf, "Remaining mission distance", ImVec4(0.99f, 0.70f, 0.32f, 1.0f), 64.0f);
        ImGui::TableNextColumn();
        metric_card("hw_graph_lidar", "LiDAR", lidar_buf, "min / front range", ImVec4(0.48f, 0.88f, 0.62f, 1.0f), 64.0f);
        ImGui::TableNextColumn();
        metric_card("hw_graph_tracking", "Tracking", err_buf, "cross-track / heading", ImVec4(0.97f, 0.89f, 0.45f, 1.0f), 64.0f);
        ImGui::EndTable();
    }
    ImGui::SeparatorText("Plots");

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
        if (ImGui::BeginTabItem("Overview")) {
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

    render_source_selector(ui_state, hardware_server);
    ImGui::Separator();

    const bool slam_source = workspace_source_expects_slam(ui_state->workspace_source);
    const std::string configured_profile = slam_source ? "SLAM" : "Planner";
    const std::string live_profile =
        hardware.has_scene ? stream_profile_label(hardware.scene.stream_profile, ui_state->workspace_source)
                           : configured_profile;
    const bool profile_mismatch =
        hardware.has_scene && !hardware.scene.stream_profile.empty() &&
        ((slam_source && hardware.scene.stream_profile != "slam") ||
         (!slam_source && hardware.scene.stream_profile == "slam"));

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
    std::snprintf(time_buf, sizeof(time_buf), "%.1f s", hardware.frame.sim_time);
    std::snprintf(goal_buf, sizeof(goal_buf), "%.2f m", hardware.frame.distance_to_goal);
    std::snprintf(speed_buf, sizeof(speed_buf), "%.2f m/s", hardware.frame.vehicle.speed);
    std::snprintf(tracking_buf, sizeof(tracking_buf), "%.2f m", hardware.frame.tracker_cross_track_error);

    ImGui::TextUnformatted(slam_source ? "Hardware SLAM Desk" : "Hardware Desk");
    ImGui::TextColored(status_color, "%s", status);
    ImGui::TextWrapped(
        slam_source
            ? "This mode keeps the simulator UI but expects the online SLAM smoke-test stream, so the viewport can accumulate and visualize live obstacle points around the robot."
            : "This mode keeps the simulator UI but replaces the backend with live planner snapshots coming from the Raspberry.");
    ImGui::Text("Configured session = %s", configured_profile.c_str());
    ImGui::Text("Incoming stream = %s", live_profile.c_str());
    if (profile_mismatch) {
        ImGui::TextColored(ImVec4(0.95f, 0.60f, 0.34f, 1.0f),
                           "The menu is set to %s but the current stream is %s.",
                           configured_profile.c_str(),
                           live_profile.c_str());
    }

    if (ImGui::BeginTable("HardwareHeroMetrics", 2, ImGuiTableFlags_SizingStretchSame)) {
        ImGui::TableNextColumn();
        metric_card("hw_control_time", "Runtime", time_buf, "Elapsed remote runtime", ImVec4(0.43f, 0.82f, 0.96f, 1.0f), 64.0f);
        ImGui::TableNextColumn();
        metric_card("hw_control_goal", "Goal", goal_buf, "Distance remaining", ImVec4(0.99f, 0.70f, 0.32f, 1.0f), 64.0f);
        ImGui::TableNextColumn();
        metric_card("hw_control_speed", "Speed", speed_buf, "Estimated forward speed", ImVec4(0.48f, 0.88f, 0.62f, 1.0f), 64.0f);
        ImGui::TableNextColumn();
        metric_card("hw_control_tracking", "Tracking", tracking_buf, "Cross-track error", ImVec4(0.97f, 0.89f, 0.45f, 1.0f), 64.0f);
        ImGui::EndTable();
    }

    ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
    if (ImGui::CollapsingHeader("Live Stream", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::InputInt("Listen Port", &ui_state->hardware_listen_port);
        ui_state->hardware_listen_port = std::max(ui_state->hardware_listen_port, 1);

        const char* environment_items[] = {
            thesis_sim::environment_mode_name(EnvironmentMode::UnstructuredGates),
            thesis_sim::environment_mode_name(EnvironmentMode::StructuredRoad),
        };
        int hardware_environment = ui_state->hardware_environment_mode == static_cast<int>(EnvironmentMode::UnstructuredGates) ? 0 : 1;
        const char* environment_label = slam_source ? "SLAM Context" : "Planner Scenario";
        if (ImGui::Combo(environment_label, &hardware_environment, environment_items, IM_ARRAYSIZE(environment_items))) {
            ui_state->hardware_environment_mode =
                hardware_environment == 0 ? static_cast<int>(EnvironmentMode::UnstructuredGates)
                                          : static_cast<int>(EnvironmentMode::StructuredRoad);
            load_hardware_editor_from_selection(ui_state);
        }

        if (static_cast<EnvironmentMode>(ui_state->hardware_environment_mode) == EnvironmentMode::StructuredRoad) {
            const StructuredMapPreset previous_preset =
                static_cast<StructuredMapPreset>(ui_state->hardware_structured_preset);
            const char* structured_items[] = {
                thesis_sim::structured_map_preset_name(StructuredMapPreset::ValidationRoad),
                thesis_sim::structured_map_preset_name(StructuredMapPreset::CircleLoop),
                thesis_sim::structured_map_preset_name(StructuredMapPreset::ZigZag),
                thesis_sim::structured_map_preset_name(StructuredMapPreset::HardwareTrack),
                thesis_sim::structured_map_preset_name(StructuredMapPreset::Custom),
            };
            int selection = 0;
            switch (previous_preset) {
                case StructuredMapPreset::CircleLoop:
                    selection = 1;
                    break;
                case StructuredMapPreset::ZigZag:
                    selection = 2;
                    break;
                case StructuredMapPreset::HardwareTrack:
                    selection = 3;
                    break;
                case StructuredMapPreset::Custom:
                    selection = 4;
                    break;
                case StructuredMapPreset::ValidationRoad:
                default:
                    selection = 0;
                    break;
            }
            if (ImGui::Combo("Structured Map", &selection, structured_items, IM_ARRAYSIZE(structured_items))) {
                StructuredMapPreset next_preset = StructuredMapPreset::ValidationRoad;
                switch (selection) {
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
                        next_preset = StructuredMapPreset::Custom;
                        break;
                    case 0:
                    default:
                        next_preset = StructuredMapPreset::ValidationRoad;
                        break;
                }
                if (next_preset == StructuredMapPreset::Custom &&
                    ui_state->hardware_editor_world.environment_mode() != EnvironmentMode::StructuredRoad) {
                    ui_state->hardware_editor_world = make_world_from_mode(
                        EnvironmentMode::StructuredRoad,
                        UnstructuredMapPreset::RobotValidation,
                        previous_preset == StructuredMapPreset::Custom ? StructuredMapPreset::ValidationRoad : previous_preset,
                        GateBehaviorMode::Static,
                        0);
                    ui_state->hardware_editor_dirty = false;
                    reset_editor_interaction(ui_state);
                }
                ui_state->hardware_structured_preset = static_cast<int>(next_preset);
                if (next_preset != StructuredMapPreset::Custom) {
                    load_hardware_editor_from_selection(ui_state);
                }
            }
        } else {
            const UnstructuredMapPreset previous_preset =
                static_cast<UnstructuredMapPreset>(ui_state->hardware_unstructured_preset);
            const char* unstructured_items[] = {
                thesis_sim::unstructured_map_preset_name(UnstructuredMapPreset::RobotValidation),
                thesis_sim::unstructured_map_preset_name(UnstructuredMapPreset::TightCorridor),
                thesis_sim::unstructured_map_preset_name(UnstructuredMapPreset::WideSlalom),
                thesis_sim::unstructured_map_preset_name(UnstructuredMapPreset::LowerBypass),
                thesis_sim::unstructured_map_preset_name(UnstructuredMapPreset::HardwareLab),
                thesis_sim::unstructured_map_preset_name(UnstructuredMapPreset::Custom),
            };
            int selection = 0;
            switch (previous_preset) {
                case UnstructuredMapPreset::TightCorridor:
                    selection = 1;
                    break;
                case UnstructuredMapPreset::WideSlalom:
                    selection = 2;
                    break;
                case UnstructuredMapPreset::LowerBypass:
                    selection = 3;
                    break;
                case UnstructuredMapPreset::HardwareLab:
                    selection = 4;
                    break;
                case UnstructuredMapPreset::Custom:
                    selection = 5;
                    break;
                case UnstructuredMapPreset::RobotValidation:
                default:
                    selection = 0;
                    break;
            }
            if (ImGui::Combo("Unstructured Map", &selection, unstructured_items, IM_ARRAYSIZE(unstructured_items))) {
                UnstructuredMapPreset next_preset = UnstructuredMapPreset::RobotValidation;
                switch (selection) {
                    case 1:
                        next_preset = UnstructuredMapPreset::TightCorridor;
                        break;
                    case 2:
                        next_preset = UnstructuredMapPreset::WideSlalom;
                        break;
                    case 3:
                        next_preset = UnstructuredMapPreset::LowerBypass;
                        break;
                    case 4:
                        next_preset = UnstructuredMapPreset::HardwareLab;
                        break;
                    case 5:
                        next_preset = UnstructuredMapPreset::Custom;
                        break;
                    case 0:
                    default:
                        next_preset = UnstructuredMapPreset::RobotValidation;
                        break;
                }
                if (next_preset == UnstructuredMapPreset::Custom &&
                    ui_state->hardware_editor_world.environment_mode() != EnvironmentMode::UnstructuredGates) {
                    ui_state->hardware_editor_world = make_world_from_mode(
                        EnvironmentMode::UnstructuredGates,
                        previous_preset == UnstructuredMapPreset::Custom ? UnstructuredMapPreset::RobotValidation : previous_preset,
                        StructuredMapPreset::ValidationRoad,
                        GateBehaviorMode::Static,
                        0);
                    ui_state->hardware_editor_dirty = false;
                    reset_editor_interaction(ui_state);
                }
                ui_state->hardware_unstructured_preset = static_cast<int>(next_preset);
                if (next_preset != UnstructuredMapPreset::Custom) {
                    load_hardware_editor_from_selection(ui_state);
                }
            }
        }

        ImGui::TextWrapped(
            slam_source
                ? "The SLAM context tells the app and the remote smoke test whether the online map should be framed as structured or unstructured."
                : "This selection defines the planner world the Raspberry should use. Choose Custom after editing/exporting a reduced map for the real room.");
        ImGui::Separator();

        if (!hardware_server->listening()) {
            if (ImGui::Button("Start Listening", ImVec2(-1.0f, 0.0f))) {
                hardware_server->start(static_cast<std::uint16_t>(ui_state->hardware_listen_port));
            }
        } else {
            if (ImGui::Button("Restart Listener", ImVec2(-1.0f, 0.0f))) {
                hardware_server->start(static_cast<std::uint16_t>(ui_state->hardware_listen_port));
            }
            if (ImGui::Button("Stop Listener", ImVec2(-1.0f, 0.0f))) {
                hardware_server->stop();
            }
        }

        ImGui::Text("Listener = %s", hardware_server->listening() ? "active" : "off");
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
        ImGui::Separator();
        ImGui::TextWrapped("Launch on the Raspberry with:");
        ImGui::TextWrapped("%s", hardware_launch_hint(*ui_state).c_str());
        const bool custom_world_selected =
            static_cast<EnvironmentMode>(ui_state->hardware_environment_mode) == EnvironmentMode::StructuredRoad
                ? static_cast<StructuredMapPreset>(ui_state->hardware_structured_preset) == StructuredMapPreset::Custom
                : static_cast<UnstructuredMapPreset>(ui_state->hardware_unstructured_preset) == UnstructuredMapPreset::Custom;
        if (custom_world_selected) {
            if (workspace_source_expects_slam(ui_state->workspace_source)) {
                if (!ui_state->last_hardware_world_path.empty()) {
                    ImGui::TextWrapped("Exported custom world: %s", ui_state->last_hardware_world_path.c_str());
                    ImGui::TextWrapped("Copy that `.thmap` file onto the Raspberry and replace `<copied-custom-map.thmap>` with its Pi path.");
                } else {
                    ImGui::TextColored(ImVec4(0.95f, 0.60f, 0.34f, 1.0f),
                                       "Export the custom map file first, then copy it to the Raspberry and launch with `--world-file`.");
                }
            } else if (ui_state->hardware_world_sync_pending || hardware_server->has_pending_world()) {
                ImGui::TextColored(ImVec4(0.43f, 0.82f, 0.96f, 1.0f),
                                   "Custom map sync is queued in the GUI and will be streamed to the Raspberry runner on connect.");
            } else {
                ImGui::TextColored(ImVec4(0.95f, 0.60f, 0.34f, 1.0f),
                                   "Press `Send Map To Raspberry` after confirming the editor world. No `--world-file` is needed for the streamed workflow.");
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
                ui_state->last_hardware_report_error.clear();
            } else {
                ui_state->hardware_report_written = false;
                ui_state->last_hardware_report_error = "Could not write the hardware telemetry report.";
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
            ImGui::TextWrapped("Last hardware report: %s", ui_state->last_hardware_report_path.c_str());
        }
        if (!ui_state->last_hardware_report_error.empty()) {
            ImGui::TextColored(ImVec4(0.95f, 0.40f, 0.34f, 1.0f), "%s", ui_state->last_hardware_report_error.c_str());
        }
    }

    ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
    if (ImGui::CollapsingHeader("Map Editor", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::SameLine();
        help_marker("Preview and reshape the hardware map locally before you launch the Raspberry runner. You can now queue the confirmed map and stream it directly to the next connected runner.");

        WorldMap& editor_world = ui_state->hardware_editor_world;
        ImGui::Checkbox("Enable drag editing", &ui_state->map_editor_enabled);
        if (ImGui::Button("Load Selected Scenario", ImVec2(-1.0f, 0.0f))) {
            load_hardware_editor_from_selection(ui_state);
        }
        if (hardware.has_scene) {
            if (ImGui::Button("Load Live Stream Scene", ImVec2(-1.0f, 0.0f))) {
                ui_state->hardware_editor_world = hardware.scene.world;
                ui_state->hardware_editor_dirty = false;
                reset_editor_interaction(ui_state);
            }
        }
        if (ImGui::Button("Apply Edited Map", ImVec2(-1.0f, 0.0f))) {
            ui_state->hardware_editor_world.finalize_editor_changes();
            ui_state->hardware_editor_dirty = false;
            if (ui_state->hardware_editor_world.environment_mode() == EnvironmentMode::StructuredRoad) {
                ui_state->hardware_environment_mode = static_cast<int>(EnvironmentMode::StructuredRoad);
                ui_state->hardware_structured_preset = static_cast<int>(StructuredMapPreset::Custom);
            } else {
                ui_state->hardware_environment_mode = static_cast<int>(EnvironmentMode::UnstructuredGates);
                ui_state->hardware_unstructured_preset = static_cast<int>(UnstructuredMapPreset::Custom);
            }
        }
        if (!workspace_source_expects_slam(ui_state->workspace_source)) {
            if (ImGui::Button("Send Map To Raspberry", ImVec2(-1.0f, 0.0f))) {
                WorldMap streamed_world = ui_state->hardware_editor_world;
                streamed_world.finalize_editor_changes();
                if (streamed_world.environment_mode() == EnvironmentMode::StructuredRoad) {
                    ui_state->hardware_environment_mode = static_cast<int>(EnvironmentMode::StructuredRoad);
                    ui_state->hardware_structured_preset = static_cast<int>(StructuredMapPreset::Custom);
                } else {
                    ui_state->hardware_environment_mode = static_cast<int>(EnvironmentMode::UnstructuredGates);
                    ui_state->hardware_unstructured_preset = static_cast<int>(UnstructuredMapPreset::Custom);
                }
                if (hardware_server->queue_world(streamed_world)) {
                    ui_state->hardware_editor_world = streamed_world;
                    ui_state->hardware_editor_dirty = false;
                    ui_state->hardware_world_sync_pending = true;
                    ui_state->hardware_world_sync_ok = false;
                    ui_state->last_hardware_world_sync_status =
                        hardware_server->connected()
                            ? "Custom map queued and sent to the connected Raspberry runner. Waiting for runner ack."
                            : "Custom map queued in the GUI. It will be sent to the next Raspberry runner that connects.";
                } else {
                    ui_state->hardware_world_sync_pending = false;
                    ui_state->hardware_world_sync_ok = false;
                    ui_state->last_hardware_world_sync_status =
                        hardware_server->last_error().empty()
                            ? "Could not queue the custom map for Raspberry sync."
                            : hardware_server->last_error();
                }
            }
        }
        if (ImGui::Button("Export Custom Map File", ImVec2(-1.0f, 0.0f))) {
            WorldMap export_world = ui_state->hardware_editor_world;
            export_world.finalize_editor_changes();
            const std::string world_path = default_hardware_world_path(*ui_state);
            std::string error;
            if (write_world_blob_file(export_world, world_path, &error)) {
                ui_state->last_hardware_world_path = world_path;
                ui_state->last_hardware_world_error.clear();
                ui_state->hardware_editor_world = export_world;
                ui_state->hardware_editor_dirty = false;
                if (export_world.environment_mode() == EnvironmentMode::StructuredRoad) {
                    ui_state->hardware_environment_mode = static_cast<int>(EnvironmentMode::StructuredRoad);
                    ui_state->hardware_structured_preset = static_cast<int>(StructuredMapPreset::Custom);
                } else {
                    ui_state->hardware_environment_mode = static_cast<int>(EnvironmentMode::UnstructuredGates);
                    ui_state->hardware_unstructured_preset = static_cast<int>(UnstructuredMapPreset::Custom);
                }
            } else {
                ui_state->last_hardware_world_error = error.empty() ? "Could not export the hardware map file." : error;
            }
        }

        if (ImGui::CollapsingHeader("Manual Map Tools", ImGuiTreeNodeFlags_DefaultOpen)) {
            if (ImGui::Button("Add Obstacle", ImVec2(-1.0f, 0.0f))) {
                const Vec2 center{
                    0.5 * (editor_world.start().x + editor_world.goal().x),
                    0.5 * (editor_world.start().y + editor_world.goal().y),
                };
                editor_world.editable_obstacles().push_back({center.x - 1.0, center.y - 1.0, center.x + 1.0, center.y + 1.0});
                ui_state->hardware_editor_dirty = true;
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

            ImGui::Text("Road points = %d", static_cast<int>(editor_world.road_centerline().size()));
            ImGui::TextDisabled("%s", ui_state->hardware_editor_dirty ? "Hardware editor has unapplied changes." : "Hardware editor is synchronized.");
            if (ui_state->map_editor_enabled) {
                ImGui::TextWrapped("Use the hardware viewport as a local preview canvas: drag start, goal, obstacle centers, gate anchors or interior road points.");
            }
        }

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
        ImGui::Text("goal distance = %.2f m", hardware.frame.distance_to_goal);
        ImGui::Text("tracking: cte %.2f m   hdg %.2f deg", hardware.frame.tracker_cross_track_error, hardware.frame.tracker_heading_error_deg);
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

    ImGui::EndChild();
}

void render_control_panel(PlannerDrivenVehicleSim& sim, UiState* ui_state, LiveViewStreamServer* hardware_server) {
    if (!ImGui::BeginChild("ConfigPanel", ImVec2(0.0f, 0.0f), true)) {
        ImGui::EndChild();
        return;
    }

    render_source_selector(ui_state, hardware_server);
    ImGui::Separator();

    const char* status = sim.goal_reached() ? "Goal reached" : (sim.collision() ? "Collision" : (ui_state->paused ? "Paused" : "Running"));
    const ImVec4 status_color = sim.goal_reached() ? ImVec4(0.45f, 0.86f, 0.53f, 1.0f)
                                                   : (sim.collision() ? ImVec4(0.95f, 0.40f, 0.34f, 1.0f)
                                                                      : (ui_state->paused ? ImVec4(0.74f, 0.79f, 0.84f, 1.0f)
                                                                                          : ImVec4(0.87f, 0.79f, 0.39f, 1.0f)));
    const auto& vehicle = sim.vehicle();
    const char* chosen_name = sim.chosen_gate_index() >= 0 ? sim.world().gates()[sim.chosen_gate_index()].name.c_str() : "none";
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
    if (ImGui::CollapsingHeader("Scenario & Planner", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::SameLine();
        help_marker("Switch between the gate-based planner and structured road behaviour without changing the rest of the simulator stack.");

        const char* environment_items[] = {
            thesis_sim::environment_mode_name(EnvironmentMode::UnstructuredGates),
            thesis_sim::environment_mode_name(EnvironmentMode::StructuredRoad),
        };
        if (ImGui::Combo("Environment", &ui_state->environment_mode, environment_items, IM_ARRAYSIZE(environment_items))) {
            sim.load_world(world_from_ui_selection(sim, *ui_state));
            sync_ui_state_from_sim(ui_state, sim);
            ui_state->paused = true;
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
            };
            if (ImGui::Combo("Unstructured Map", &ui_state->unstructured_preset, preset_items, IM_ARRAYSIZE(preset_items))) {
                const bool manual_editor = ui_state->unstructured_preset == static_cast<int>(UnstructuredMapPreset::Custom);
                sim.load_world(world_from_ui_selection(sim, *ui_state));
                sync_ui_state_from_sim(ui_state, sim);
                ui_state->map_editor_enabled = manual_editor;
                ui_state->paused = true;
            }
            ImGui::Text("Preset = %s", thesis_sim::unstructured_map_preset_name(sim.world().unstructured_preset()));

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
        } else {
            ImGui::TextWrapped("Structured mode keeps the planner on a fixed road loop while the EKF fuses encoder, IMU and LiDAR for the state estimate.");
            const char* structured_items[] = {
                thesis_sim::structured_map_preset_name(StructuredMapPreset::ValidationRoad),
                thesis_sim::structured_map_preset_name(StructuredMapPreset::CircleLoop),
                thesis_sim::structured_map_preset_name(StructuredMapPreset::ZigZag),
            };
            int structured_selection = ui_state->structured_preset;
            if (structured_selection == static_cast<int>(StructuredMapPreset::Custom)) {
                structured_selection = static_cast<int>(StructuredMapPreset::ValidationRoad);
            }
            if (ImGui::Combo("Structured Map", &structured_selection, structured_items, IM_ARRAYSIZE(structured_items))) {
                ui_state->structured_preset = structured_selection;
                sim.load_world(world_from_ui_selection(sim, *ui_state));
                sync_ui_state_from_sim(ui_state, sim);
                ui_state->paused = true;
            }
            ImGui::Text("Preset = %s", thesis_sim::structured_map_preset_name(sim.world().structured_preset()));
            ImGui::Text("Road points = %d", static_cast<int>(sim.world().road_centerline().size()));
            if (sim.world().structured_preset() == StructuredMapPreset::Custom) {
                ImGui::TextColored(ImVec4(1.0f, 0.82f, 0.42f, 1.0f), "Current road = Custom edited scenario");
                ImGui::TextWrapped("The last edited structured road is active. Built-in presets in the combo restore the known-good loops.");
                if (ImGui::Button("Restore Validation Road", ImVec2(-1.0f, 0.0f))) {
                    ui_state->structured_preset = static_cast<int>(StructuredMapPreset::ValidationRoad);
                    sim.load_world(make_world_from_mode(
                        EnvironmentMode::StructuredRoad,
                        static_cast<UnstructuredMapPreset>(ui_state->unstructured_preset),
                        StructuredMapPreset::ValidationRoad,
                        sim.gate_behavior(),
                        sim.gate_seed()));
                    sync_ui_state_from_sim(ui_state, sim);
                    ui_state->paused = true;
                }
            }
            ImGui::TextDisabled("Gate layout controls are disabled in structured mode.");
        }
    }

    ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
    if (ImGui::CollapsingHeader("Sensors & Localization", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::SameLine();
        help_marker("These toggles affect the simulated measurements that feed the EKF. The planner still works on the fused state estimate.");
        bool imu_enabled = sim.imu_enabled();
        bool lidar_enabled = sim.lidar_enabled();
        int sensor_profile = static_cast<int>(sim.range_sensor_profile());
        bool sensor_changed = false;

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
            ui_state->report_written = false;
            ui_state->last_report_path.clear();
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

    ImGui::SetNextItemOpen(false, ImGuiCond_FirstUseEver);
    if (ImGui::CollapsingHeader("Map Editor")) {
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

            ImGui::TextDisabled("%s", ui_state->scenario_editor_dirty ? "Edited scenario has unapplied changes." : "Edited scenario is synchronized.");
            if (ui_state->map_editor_enabled) {
                ImGui::TextWrapped("Drag start, goal, obstacle centers, gate anchors or interior road points directly in the viewport.");
            }
        }
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
        if (sim.environment_mode() == EnvironmentMode::UnstructuredGates) {
            ImGui::Text("chosen gate = %s   visible = %d", chosen_name, static_cast<int>(sim.visible_gate_indices().size()));
        } else {
            ImGui::Text("road points = %d", static_cast<int>(sim.world().road_centerline().size()));
        }
    }

    ImGui::SetNextItemOpen(false, ImGuiCond_FirstUseEver);
    if (ImGui::CollapsingHeader("Reports")) {
        ImGui::SameLine();
        help_marker("Writes a JSON report with scenario, configuration, summary metrics and decimated telemetry so multiple tests can be compared offline.");
        if (ImGui::Button("Write JSON Report", ImVec2(-1.0f, 0.0f))) {
            const std::string report_path = default_report_path(sim, "gui");
            if (write_json_report(sim, report_status_string(sim), report_path)) {
                ui_state->report_written = true;
                ui_state->last_report_path = report_path;
            }
        }
        if (!ui_state->last_report_path.empty()) {
            ImGui::TextWrapped("Last report: %s", ui_state->last_report_path.c_str());
        }
    }

    ImGui::EndChild();
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

    if (ImGui::BeginTable("WorkspaceLayout", 2, ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_BordersInnerV)) {
        ImGui::TableSetupColumn("MainArea", ImGuiTableColumnFlags_WidthStretch, 1.0f);
        ImGui::TableSetupColumn("ConfigArea", ImGuiTableColumnFlags_WidthFixed, 390.0f);

        const bool hardware_mode = ui_state != nullptr && workspace_source_is_hardware(ui_state->workspace_source);
        ImGui::TableNextColumn();
        if (ImGui::BeginTabBar("WorkspaceTabs")) {
            if (ImGui::BeginTabItem(hardware_mode ? workspace_source_label(ui_state->workspace_source) : "Simulation")) {
                if (hardware_mode && hardware != nullptr) {
                    render_hardware_world_tab(*hardware, ui_state);
                } else {
                    render_world_tab(sim, ui_state);
                }
                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem("Graphs")) {
                if (hardware_mode && hardware != nullptr) {
                    render_hardware_graphs_tab(*hardware);
                } else {
                    render_graphs_tab(sim);
                }
                ImGui::EndTabItem();
            }
            ImGui::EndTabBar();
        }

        ImGui::TableNextColumn();
        if (hardware_mode && hardware != nullptr) {
            render_hardware_control_panel(*hardware, ui_state, hardware_server);
        } else {
            render_control_panel(sim, ui_state, hardware_server);
        }
        ImGui::EndTable();
    }

    ImGui::End();
}

int run_headless(const AppOptions& options) {
    PlannerDrivenVehicleSim sim(make_world_from_mode(
        options.environment_mode,
        options.unstructured_preset,
        options.structured_preset,
        GateBehaviorMode::Static,
        7));
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

    PlannerDrivenVehicleSim sim(make_world_from_mode(
        options.environment_mode,
        options.unstructured_preset,
        options.structured_preset,
        GateBehaviorMode::Static,
        7));
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
        if (hardware_updates.scene_received && hardware_updates.scene.has_value()) {
            hardware_view.scene = *hardware_updates.scene;
            hardware_view.has_scene = true;
            hardware_view.frame = {};
            hardware_view.history.clear();
            ui_state.hardware_report_written = false;
            ui_state.last_hardware_report_path.clear();
            ui_state.last_hardware_report_error.clear();
            ui_state.hardware_environment_mode = static_cast<int>(hardware_view.scene.world.environment_mode());
            ui_state.hardware_unstructured_preset = static_cast<int>(hardware_view.scene.world.unstructured_preset());
            ui_state.hardware_structured_preset = static_cast<int>(hardware_view.scene.world.structured_preset());
            ui_state.hardware_editor_world = hardware_view.scene.world;
            ui_state.hardware_editor_dirty = false;
            reset_editor_interaction(&ui_state);
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
