#include <SDL.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

#include "imgui.h"
#include "imgui_impl_sdl2.h"
#include "imgui_impl_sdlrenderer2.h"
#include "implot.h"

#include "planner_sim.h"

namespace {

using thesis_sim::GateSpec;
using thesis_sim::GateBehaviorMode;
using thesis_sim::LidarHit;
using thesis_sim::PlannerDrivenVehicleSim;
using thesis_sim::RangeSensorProfile;
using thesis_sim::Rect;
using thesis_sim::SimulationReport;
using thesis_sim::TelemetrySample;
using thesis_sim::Vec2;
using thesis_sim::VehicleSnapshot;
using thesis_sim::WheelPose;
using thesis_sim::WorldMap;

struct AppOptions {
    bool headless = false;
    int max_steps = 6000;
};

struct CanvasTransform {
    ImVec2 origin;
    float scale = 1.0f;
};

struct UiState {
    bool paused = true;
    bool single_step = false;
    int steps_per_frame = 1;
    int gate_seed_input = 7;
};

constexpr ImU32 kColorCanvas = IM_COL32(19, 24, 28, 255);
constexpr ImU32 kColorGrid = IM_COL32(39, 49, 57, 255);
constexpr ImU32 kColorBounds = IM_COL32(148, 162, 170, 255);
constexpr ImU32 kColorObstacle = IM_COL32(86, 95, 105, 255);
constexpr ImU32 kColorTrail = IM_COL32(95, 186, 255, 255);
constexpr ImU32 kColorLidar = IM_COL32(132, 232, 147, 70);
constexpr ImU32 kColorLidarHit = IM_COL32(198, 255, 172, 255);
constexpr ImU32 kColorGate = IM_COL32(73, 198, 236, 255);
constexpr ImU32 kColorGateVisible = IM_COL32(255, 196, 61, 255);
constexpr ImU32 kColorGatePassed = IM_COL32(130, 138, 148, 255);
constexpr ImU32 kColorGoal = IM_COL32(248, 109, 76, 255);
constexpr ImU32 kColorBody = IM_COL32(241, 239, 228, 255);
constexpr ImU32 kColorWheel = IM_COL32(52, 55, 61, 255);
constexpr ImU32 kColorHeading = IM_COL32(255, 142, 79, 255);

AppOptions parse_args(int argc, char** argv) {
    AppOptions options;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--headless") {
            options.headless = true;
        } else if (arg == "--max-steps" && i + 1 < argc) {
            options.max_steps = std::atoi(argv[++i]);
        } else if (arg.rfind("--max-steps=", 0) == 0) {
            options.max_steps = std::atoi(arg.substr(std::strlen("--max-steps=")).c_str());
        }
    }
    options.max_steps = std::max(options.max_steps, 1);
    return options;
}

void apply_style() {
    ImGuiStyle& style = ImGui::GetStyle();
    style.WindowRounding = 10.0f;
    style.ChildRounding = 10.0f;
    style.FrameRounding = 8.0f;
    style.GrabRounding = 8.0f;
    style.PopupRounding = 10.0f;
    style.ScrollbarRounding = 12.0f;
    style.WindowPadding = ImVec2(12.0f, 12.0f);
    style.FramePadding = ImVec2(10.0f, 6.0f);
    style.ItemSpacing = ImVec2(10.0f, 8.0f);

    ImVec4* colors = style.Colors;
    colors[ImGuiCol_WindowBg] = ImVec4(0.07f, 0.09f, 0.11f, 1.0f);
    colors[ImGuiCol_ChildBg] = ImVec4(0.09f, 0.11f, 0.14f, 1.0f);
    colors[ImGuiCol_FrameBg] = ImVec4(0.13f, 0.16f, 0.19f, 1.0f);
    colors[ImGuiCol_FrameBgHovered] = ImVec4(0.18f, 0.22f, 0.27f, 1.0f);
    colors[ImGuiCol_FrameBgActive] = ImVec4(0.21f, 0.26f, 0.31f, 1.0f);
    colors[ImGuiCol_Button] = ImVec4(0.17f, 0.38f, 0.46f, 1.0f);
    colors[ImGuiCol_ButtonHovered] = ImVec4(0.24f, 0.53f, 0.62f, 1.0f);
    colors[ImGuiCol_ButtonActive] = ImVec4(0.16f, 0.31f, 0.38f, 1.0f);
    colors[ImGuiCol_Header] = ImVec4(0.16f, 0.34f, 0.40f, 1.0f);
    colors[ImGuiCol_HeaderHovered] = ImVec4(0.24f, 0.45f, 0.53f, 1.0f);
    colors[ImGuiCol_HeaderActive] = ImVec4(0.19f, 0.39f, 0.45f, 1.0f);
    colors[ImGuiCol_TitleBg] = ImVec4(0.08f, 0.10f, 0.12f, 1.0f);
    colors[ImGuiCol_TitleBgActive] = ImVec4(0.11f, 0.15f, 0.18f, 1.0f);
    colors[ImGuiCol_Tab] = ImVec4(0.12f, 0.16f, 0.19f, 1.0f);
    colors[ImGuiCol_TabHovered] = ImVec4(0.18f, 0.34f, 0.40f, 1.0f);
    colors[ImGuiCol_TabActive] = ImVec4(0.15f, 0.26f, 0.31f, 1.0f);
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

void draw_polyline(ImDrawList* draw_list, const CanvasTransform& tx, const std::vector<Vec2>& points, ImU32 color, float thickness) {
    if (points.size() < 2) {
        return;
    }
    for (size_t i = 1; i < points.size(); ++i) {
        draw_list->AddLine(world_to_screen(tx, points[i - 1]), world_to_screen(tx, points[i]), color, thickness);
    }
}

void draw_vehicle(ImDrawList* draw_list, const CanvasTransform& tx, const VehicleSnapshot& vehicle, const thesis_sim::VehicleGeometry& geometry) {
    std::array<ImVec2, 4> body{};
    for (size_t i = 0; i < body.size(); ++i) {
        body[i] = world_to_screen(tx, vehicle.body_corners[i]);
    }
    draw_list->AddConvexPolyFilled(body.data(), static_cast<int>(body.size()), kColorBody);
    draw_list->AddPolyline(body.data(), static_cast<int>(body.size()), IM_COL32(20, 22, 26, 255), ImDrawFlags_Closed, 2.0f);

    for (const WheelPose& wheel : vehicle.wheels) {
        const auto wheel_box = thesis_sim::make_box_corners(wheel.center, wheel.yaw, geometry.wheel_length, geometry.wheel_width);
        std::array<ImVec2, 4> wheel_points{};
        for (size_t i = 0; i < wheel_points.size(); ++i) {
            wheel_points[i] = world_to_screen(tx, wheel_box[i]);
        }
        draw_list->AddConvexPolyFilled(wheel_points.data(), static_cast<int>(wheel_points.size()), kColorWheel);
        draw_list->AddPolyline(wheel_points.data(), static_cast<int>(wheel_points.size()), IM_COL32(200, 203, 207, 255), ImDrawFlags_Closed, 1.0f);
    }

    const Vec2 nose{
        vehicle.position.x + std::cos(vehicle.yaw) * geometry.body_length * 0.65,
        vehicle.position.y + std::sin(vehicle.yaw) * geometry.body_length * 0.65,
    };
    draw_list->AddLine(world_to_screen(tx, vehicle.position), world_to_screen(tx, nose), kColorHeading, 3.0f);
}

void render_world_tab(PlannerDrivenVehicleSim& sim) {
    if (!ImGui::BeginChild("SimulationViewport", ImVec2(0.0f, 0.0f), true, ImGuiWindowFlags_NoScrollbar)) {
        ImGui::EndChild();
        return;
    }

    const ImVec2 canvas_pos = ImGui::GetCursorScreenPos();
    ImVec2 canvas_size = ImGui::GetContentRegionAvail();
    canvas_size.x = std::max(canvas_size.x, 320.0f);
    canvas_size.y = std::max(canvas_size.y, 320.0f);

    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    draw_list->AddRectFilled(canvas_pos, ImVec2(canvas_pos.x + canvas_size.x, canvas_pos.y + canvas_size.y), kColorCanvas, 14.0f);
    draw_list->AddRect(canvas_pos, ImVec2(canvas_pos.x + canvas_size.x, canvas_pos.y + canvas_size.y), IM_COL32(71, 81, 88, 255), 14.0f, 0, 1.5f);

    const CanvasTransform tx = make_transform(sim.world(), canvas_pos, canvas_size);
    const Rect bounds = sim.world().bounds();
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

    for (const Rect& obstacle : sim.world().obstacles()) {
        draw_list->AddRectFilled(world_to_screen(tx, {obstacle.min_x, obstacle.max_y}),
                                 world_to_screen(tx, {obstacle.max_x, obstacle.min_y}),
                                 kColorObstacle, 4.0f);
    }

    draw_polyline(draw_list, tx, sim.trail(), kColorTrail, 2.5f);

    if (sim.lidar_enabled()) {
        const Vec2 car_pos = sim.vehicle().position;
        for (const LidarHit& hit : sim.lidar_hits()) {
            draw_list->AddLine(world_to_screen(tx, car_pos), world_to_screen(tx, hit.point), kColorLidar, 1.0f);
        }

        for (const LidarHit& hit : sim.lidar_hits()) {
            draw_list->AddCircleFilled(world_to_screen(tx, hit.point), 2.0f, kColorLidarHit);
        }
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
        draw_list->AddText(ImVec2(screen_pos.x + 6.0f, screen_pos.y - 12.0f), IM_COL32(222, 227, 230, 255), spec.name.c_str());
    }

    draw_vehicle(draw_list, tx, sim.vehicle(), sim.geometry());

    draw_list->AddText(ImVec2(canvas_pos.x + 16.0f, canvas_pos.y + 14.0f),
                       IM_COL32(233, 236, 229, 255),
                       "Simulation viewport");
    draw_list->AddText(ImVec2(canvas_pos.x + 16.0f, canvas_pos.y + 34.0f),
                       IM_COL32(170, 179, 185, 255),
                       thesis_sim::range_sensor_profile_name(sim.range_sensor_profile()));
    draw_list->AddText(ImVec2(canvas_pos.x + 16.0f, canvas_pos.y + 54.0f),
                       IM_COL32(170, 179, 185, 255),
                       thesis_sim::gate_behavior_mode_name(sim.gate_behavior()));

    ImGui::Dummy(canvas_size);
    ImGui::EndChild();
}

void render_plot_window(const char* title,
                        const std::vector<double>& x,
                        const std::vector<double>& y0,
                        const char* label0,
                        const std::vector<double>* y1 = nullptr,
                        const char* label1 = nullptr) {
    if (!ImPlot::BeginPlot(title, ImVec2(-1, 190.0f))) {
        return;
    }
    ImPlot::SetupAxes("t [s]", nullptr, ImPlotAxisFlags_NoTickLabels, 0);
    ImPlot::SetupAxisLimits(ImAxis_X1, x.front(), x.back(), ImGuiCond_Always);
    ImPlot::PlotLine(label0, x.data(), y0.data(), static_cast<int>(x.size()));
    if (y1 != nullptr && label1 != nullptr) {
        ImPlot::PlotLine(label1, x.data(), y1->data(), static_cast<int>(x.size()));
    }
    ImPlot::EndPlot();
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

    for (const TelemetrySample& sample : history) {
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
    }

    render_plot_window("Velocity / Acceleration", time, speed, "v [m/s]", &accel, "a [m/s^2]");
    render_plot_window("Planner Commands", time, jerk, "j [m/s^3]", &command_r, "r [1/(m*s)]");
    render_plot_window("Target / Wheel Speed", time, target_speed, "v target [m/s]", &left_wheel_speed, "v left [m/s]");
    render_plot_window("Right Wheel / Target Yaw", time, right_wheel_speed, "v right [m/s]", &target_yaw_rate, "wz target [deg/s]");
    render_plot_window("Motor PWM", time, left_pwm, "PWM left", &right_pwm, "PWM right");
    render_plot_window("Encoder Delta", time, left_encoder_delta, "dTicks left", &right_encoder_delta, "dTicks right");
    render_plot_window("Distance Metrics", time, dist_goal, "goal [m]", &min_lidar, "lidar [m]");
    ImGui::EndChild();
}

void render_control_panel(PlannerDrivenVehicleSim& sim, UiState* ui_state) {
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
    const char* chosen_name = sim.chosen_gate_index() >= 0 ? sim.world().gates()[sim.chosen_gate_index()].name.c_str() : "none";

    ImGui::TextUnformatted("Simulation Control");
    ImGui::TextColored(status_color, "%s", status);
    ImGui::SeparatorText("Run");

    if (ImGui::Button(ui_state->paused ? "Resume" : "Pause", ImVec2(-1.0f, 0.0f))) {
        ui_state->paused = !ui_state->paused;
    }
    if (ImGui::Button("Step", ImVec2(-1.0f, 0.0f))) {
        ui_state->single_step = true;
    }
    if (ImGui::Button("Reset", ImVec2(-1.0f, 0.0f))) {
        sim.reset();
        ui_state->paused = true;
    }
    ImGui::SliderInt("Steps / frame", &ui_state->steps_per_frame, 1, 8);

    ImGui::SeparatorText("Scenario");
    ImGui::TextWrapped("Named gates are passage hypotheses through the free openings of the unstructured map. They guide the planner toward feasible corridors.");

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
    }
    if (ImGui::Button("Regenerate Gates", ImVec2(-1.0f, 0.0f))) {
        ui_state->gate_seed_input = std::max(ui_state->gate_seed_input + 1, 0);
        sim.regenerate_gate_layout(static_cast<std::uint32_t>(ui_state->gate_seed_input));
    }
    ImGui::Text("Gate mode = %s", thesis_sim::gate_behavior_mode_name(sim.gate_behavior()));
    ImGui::Text("Gate seed = %u", sim.gate_seed());

    ImGui::SeparatorText("Sensors");
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
    }

    ImGui::Text("Profile = %s", thesis_sim::range_sensor_profile_name(sim.range_sensor_profile()));
    ImGui::Text("Beams = %d", sim.active_lidar_beams());
    ImGui::Text("FOV = %.0f deg", sim.active_lidar_fov_rad() * 180.0 / 3.14159265358979323846);
    ImGui::Text("Range = %.1f m", sim.active_lidar_range());
    if (sim.imu_enabled()) {
        ImGui::Text("IMU yaw rate = %.2f deg/s", vehicle.yaw_rate * 180.0 / 3.14159265358979323846);
    } else {
        ImGui::TextDisabled("IMU disabled");
    }
    if (sim.lidar_enabled()) {
        ImGui::Text("LiDAR min range = %.2f m", sim.min_lidar_distance());
    } else {
        ImGui::TextDisabled("LiDAR disabled");
    }

    ImGui::SeparatorText("State");
    ImGui::Text("model = %s", vehicle.model_name.c_str());
    ImGui::Text("t = %.2f s", sim.sim_time());
    ImGui::Text("step = %d", sim.step_count());
    ImGui::Text("pos = (%.2f, %.2f) m", vehicle.position.x, vehicle.position.y);
    ImGui::Text("yaw = %.1f deg", vehicle.yaw * 180.0 / 3.14159265358979323846);
    ImGui::Text("v = %.2f m/s", vehicle.speed);
    ImGui::Text("a = %.2f m/s^2", vehicle.accel);
    ImGui::Text("j = %.2f m/s^3", sim.last_j());
    ImGui::Text("r = %.2f 1/(m*s)", sim.last_r());
    ImGui::Text("kappa = %.3f 1/m", vehicle.curvature);
    ImGui::Text("target v = %.2f m/s", vehicle.target_speed);
    ImGui::Text("target wz = %.2f deg/s", vehicle.target_yaw_rate * 180.0 / 3.14159265358979323846);
    ImGui::Text("wheel v = (%.2f, %.2f) m/s", vehicle.left_wheel_speed, vehicle.right_wheel_speed);
    ImGui::Text("PWM = (%d, %d)", vehicle.left_pwm, vehicle.right_pwm);
    ImGui::Text("enc ticks = (%d, %d)", vehicle.left_encoder_ticks, vehicle.right_encoder_ticks);
    ImGui::Text("dTicks = (%d, %d)", vehicle.left_encoder_delta, vehicle.right_encoder_delta);
    ImGui::Text("goal distance = %.2f m", sim.distance_to_goal());
    ImGui::Text("chosen gate = %s", chosen_name);
    ImGui::Text("visible gates = %d", static_cast<int>(sim.visible_gate_indices().size()));

    ImGui::EndChild();
}

void render_workspace(PlannerDrivenVehicleSim& sim, UiState* ui_state) {
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
        ImGui::TableSetupColumn("ConfigArea", ImGuiTableColumnFlags_WidthFixed, 350.0f);

        ImGui::TableNextColumn();
        if (ImGui::BeginTabBar("WorkspaceTabs")) {
            if (ImGui::BeginTabItem("Simulation")) {
                render_world_tab(sim);
                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem("Graphs")) {
                render_graphs_tab(sim);
                ImGui::EndTabItem();
            }
            ImGui::EndTabBar();
        }

        ImGui::TableNextColumn();
        render_control_panel(sim, ui_state);
        ImGui::EndTable();
    }

    ImGui::End();
}

int run_headless(const AppOptions& options) {
    PlannerDrivenVehicleSim sim(WorldMap::thesis_demo());
    const SimulationReport report = sim.run_headless(options.max_steps);

    std::cout << "status=" << (report.goal_reached ? "goal_reached" : (report.collision ? "collision" : "timeout")) << '\n';
    std::cout << "steps=" << report.steps << '\n';
    std::cout << "time=" << report.sim_time << '\n';
    std::cout << "final_x=" << report.final_position.x << '\n';
    std::cout << "final_y=" << report.final_position.y << '\n';
    std::cout << "distance_to_goal=" << report.distance_to_goal << '\n';
    std::cout << "passed_gates=" << report.passed_gates << '\n';

    return report.goal_reached ? 0 : 1;
}

int run_gui() {
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

    PlannerDrivenVehicleSim sim(WorldMap::thesis_demo());
    bool running = true;
    UiState ui_state;
    ui_state.gate_seed_input = static_cast<int>(sim.gate_seed());

    while (running) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            ImGui_ImplSDL2_ProcessEvent(&event);
            if (event.type == SDL_QUIT) {
                running = false;
            }
        }

        if (!ui_state.paused && !sim.goal_reached() && !sim.collision()) {
            for (int i = 0; i < ui_state.steps_per_frame; ++i) {
                sim.step();
                if (sim.goal_reached() || sim.collision()) {
                    break;
                }
            }
        } else if (ui_state.single_step) {
            sim.step();
            ui_state.single_step = false;
        }

        ImGui_ImplSDLRenderer2_NewFrame();
        ImGui_ImplSDL2_NewFrame();
        ImGui::NewFrame();

        render_workspace(sim, &ui_state);

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
    return run_gui();
}
