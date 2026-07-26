#include "mvc/view/canvas/world_canvas_view.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "mvc/model/world/scenario_model.h"

namespace thesis_sim::mvc::view {

constexpr ImU32 kColorGrid = IM_COL32(39, 49, 57, 255);
constexpr ImU32 kColorBounds = IM_COL32(148, 162, 170, 255);
constexpr ImU32 kColorLidar = IM_COL32(132, 232, 147, 145);
constexpr ImU32 kColorLidarMiss = IM_COL32(108, 188, 255, 95);
constexpr ImU32 kColorLidarHit = IM_COL32(214, 255, 184, 255);
constexpr ImU32 kColorGoal = IM_COL32(248, 109, 76, 255);
constexpr ImU32 kColorBody = IM_COL32(241, 239, 228, 255);
constexpr ImU32 kColorWheel = IM_COL32(52, 55, 61, 255);
constexpr ImU32 kColorHeading = IM_COL32(255, 142, 79, 255);

using model::structured_content_bounds;

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
        if (world.structured_preset() == StructuredMapPreset::TankCircuit &&
            world.unstructured_preset() == UnstructuredMapPreset::HardwareLab &&
            !world.obstacles().empty()) {
            const Rect& bounds = world.bounds();
            const double span = std::max(bounds.max_x - bounds.min_x,
                                         bounds.max_y - bounds.min_y);
            if (span <= 2.50) {
                return std::clamp(span * 0.41, 0.70, 0.76);
            }
            return 2.40;
        }
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
    const bool aruco_reference_mixed_map =
        world.environment_mode() == EnvironmentMode::MixedRoadGates &&
        world.structured_preset() == StructuredMapPreset::TankCircuit &&
        !world.obstacles().empty();

    if (!aruco_reference_mixed_map) {
        draw_polyline(draw_list, tx, make_offset_polyline(centerline, half_width), road_bound_color, 2.4f);
        draw_polyline(draw_list, tx, make_offset_polyline(centerline, -half_width), road_bound_color, 2.4f);
    }
    if (!aruco_reference_mixed_map && road_width >= 1.2) {
        draw_polyline(draw_list, tx, make_offset_polyline(centerline, road_width / 6.0), lane_color, 1.4f);
        draw_polyline(draw_list, tx, make_offset_polyline(centerline, -road_width / 6.0), lane_color, 1.4f);
    }
    draw_polyline(draw_list, tx, centerline, road_center_color, 2.0f);
}

float hardware_vehicle_visual_scale_for_world(const WorldMap& world) {
    if (world.environment_mode() == EnvironmentMode::MixedRoadGates) {
        const Rect& bounds = world.bounds();
        const double span = std::max(bounds.max_x - bounds.min_x, bounds.max_y - bounds.min_y);
        if (span <= 0.65) {
            return 0.24f;
        }
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

void draw_viewport_background(ImDrawList* draw_list,
                              const ImVec2& canvas_pos,
                              const ImVec2& canvas_size,
                              bool slam_view) {
    const ImU32 top_left = slam_view ? IM_COL32(18, 25, 28, 255) : IM_COL32(16, 22, 28, 255);
    const ImU32 top_right = slam_view ? IM_COL32(20, 31, 32, 255) : IM_COL32(23, 32, 39, 255);
    draw_list->AddRectFilledMultiColor(
        canvas_pos,
        ImVec2(canvas_pos.x + canvas_size.x, canvas_pos.y + canvas_size.y),
        top_left,
        top_right,
        IM_COL32(12, 18, 22, 255),
        IM_COL32(14, 19, 24, 255));
    draw_list->AddRect(
        canvas_pos,
        ImVec2(canvas_pos.x + canvas_size.x, canvas_pos.y + canvas_size.y),
        slam_view ? IM_COL32(42, 116, 112, 230) : IM_COL32(71, 81, 88, 255),
        8.0f,
        0,
        1.5f);
}

void draw_lidar_scan(ImDrawList* draw_list,
                     const CanvasTransform& tx,
                     const Vec2& origin,
                     const std::vector<LidarHit>& hits,
                     bool show_rays,
                     bool show_hits) {
    const size_t stride = hits.size() > 720 ? 2 : 1;
    if (show_rays) {
        for (size_t i = 0; i < hits.size(); i += stride) {
            const LidarHit& hit = hits[i];
            draw_list->AddLine(
                world_to_screen(tx, origin),
                world_to_screen(tx, hit.point),
                hit.hit ? kColorLidar : kColorLidarMiss,
                hit.hit ? 1.8f : 1.0f);
        }
    }
    if (show_hits) {
        for (size_t i = 0; i < hits.size(); i += stride) {
            const LidarHit& hit = hits[i];
            const ImVec2 hit_screen = world_to_screen(tx, hit.point);
            if (hit.hit) {
                draw_list->AddCircleFilled(hit_screen, 2.7f, kColorLidarHit);
                draw_list->AddCircle(hit_screen, 4.8f, IM_COL32(231, 255, 212, 205), 0, 1.1f);
            } else {
                draw_list->AddCircleFilled(hit_screen, 1.6f, IM_COL32(150, 207, 255, 135));
            }
        }
    }
    draw_list->AddCircleFilled(world_to_screen(tx, origin), 3.8f, IM_COL32(170, 255, 208, 220));
}

void draw_oriented_gate(ImDrawList* draw_list,
                        const CanvasTransform& tx,
                        const Vec2& position,
                        double heading,
                        double length_m,
                        ImU32 color,
                        bool selected) {
    const double half_length = 0.5 * std::max(length_m, 0.08);
    // Match the AVEC planner convention: PSI_end is the vehicle heading at
    // the target, while the gate itself is the transverse segment through it.
    const Vec2 normal{std::sin(heading), -std::cos(heading)};
    const Vec2 from{
        position.x + normal.x * half_length,
        position.y + normal.y * half_length,
    };
    const Vec2 to{
        position.x - normal.x * half_length,
        position.y - normal.y * half_length,
    };
    const float thickness = selected ? 4.8f : 3.2f;
    draw_list->AddLine(world_to_screen(tx, from), world_to_screen(tx, to), color, thickness);
}


void draw_vehicle(ImDrawList* draw_list,
                  const CanvasTransform& tx,
                  const VehicleSnapshot& vehicle,
                  const thesis_sim::VehicleGeometry& geometry,
                  float visual_scale,
                  ImU32 body_color) {
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


}  // namespace thesis_sim::mvc::view
