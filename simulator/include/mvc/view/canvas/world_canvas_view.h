#pragma once

#include <vector>

#include "imgui.h"
#include "mvc/controller/simulation_planner/simulator.h"
#include "mvc/view/live_stream/live_view_stream.h"

namespace thesis_sim::mvc::view {

struct CanvasTransform {
    ImVec2 origin;
    float scale = 1.0f;
};

void apply_style();
CanvasTransform make_transform(const WorldMap& world, const ImVec2& canvas_pos, const ImVec2& canvas_size);
ImVec2 world_to_screen(const CanvasTransform& transform, const Vec2& point);
Vec2 screen_to_world(const CanvasTransform& transform, const ImVec2& point);
float distance_sq(const ImVec2& a, const ImVec2& b);
void draw_polyline(ImDrawList* draw_list, const CanvasTransform& transform,
                   const std::vector<Vec2>& points, ImU32 color, float thickness);
void draw_world_grid(ImDrawList* draw_list, const CanvasTransform& transform, const Rect& bounds);
void draw_structured_road_map(ImDrawList* draw_list, const CanvasTransform& transform,
                              const WorldMap& world);
float hardware_vehicle_visual_scale_for_world(const WorldMap& world);
ImU32 hardware_vehicle_body_color_for_world(const WorldMap& world);
float simulation_vehicle_visual_scale_for_world(const WorldMap& world);
void draw_goal_marker(ImDrawList* draw_list, const CanvasTransform& transform, const Vec2& goal);
void draw_viewport_background(ImDrawList* draw_list, const ImVec2& canvas_pos,
                              const ImVec2& canvas_size, bool slam_view = false);
void draw_lidar_scan(ImDrawList* draw_list, const CanvasTransform& transform,
                     const Vec2& origin, const std::vector<LidarHit>& hits,
                     bool show_rays, bool show_hits);
void draw_oriented_gate(ImDrawList* draw_list, const CanvasTransform& transform,
                        const Vec2& position, double heading, double length_m,
                        ImU32 color, bool selected);
void draw_vehicle(ImDrawList* draw_list, const CanvasTransform& transform,
                  const VehicleSnapshot& vehicle, const VehicleGeometry& geometry,
                  float visual_scale = 1.0f,
                  ImU32 body_color = IM_COL32(241, 239, 228, 255));
VehicleSnapshot build_vehicle_snapshot_from_live(const LiveVehicleState& live,
                                                 const VehicleGeometry& geometry);

}  // namespace thesis_sim::mvc::view
