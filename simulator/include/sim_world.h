#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace thesis_sim {

enum class GateBehaviorMode {
    Static = 0,
    Randomized = 1,
    Mobile = 2,
};

const char* gate_behavior_mode_name(GateBehaviorMode mode);

enum class EnvironmentMode {
    UnstructuredGates = 0,
    StructuredRoad = 1,
    MixedRoadGates = 2,
};

const char* environment_mode_name(EnvironmentMode mode);

enum class UnstructuredMapPreset {
    RobotValidation = 0,
    TightCorridor = 1,
    WideSlalom = 2,
    LowerBypass = 3,
    Custom = 4,
    HardwareLab = 5,
};

const char* unstructured_map_preset_name(UnstructuredMapPreset preset);

enum class StructuredMapPreset {
    ValidationRoad = 0,
    CircleLoop = 1,
    ZigZag = 2,
    Custom = 3,
    HardwareTrack = 4,
    FigureEight = 5,
};

const char* structured_map_preset_name(StructuredMapPreset preset);

struct Vec2 {
    double x = 0.0;
    double y = 0.0;
};

struct Rect {
    double min_x = 0.0;
    double min_y = 0.0;
    double max_x = 0.0;
    double max_y = 0.0;
};

struct GateSpec {
    std::string name;
    Vec2 position;
    Vec2 anchor_position;
    Vec2 motion_amplitude;
    double motion_frequency_hz = 0.0;
    double motion_phase_rad = 0.0;
    double heading_hint = 0.0;
    bool final = false;
};

struct LidarHit {
    double angle = 0.0;
    double distance = 0.0;
    Vec2 point;
    bool hit = false;
};

class WorldMap {
  public:
    static WorldMap unstructured_demo(UnstructuredMapPreset preset = UnstructuredMapPreset::RobotValidation,
                                      GateBehaviorMode gate_behavior = GateBehaviorMode::Static,
                                      std::uint32_t gate_seed = 0);
    static WorldMap structured_demo(StructuredMapPreset preset = StructuredMapPreset::ValidationRoad);
    static WorldMap mixed_demo();
    static WorldMap mixed_hardware_aligned_demo();
    static WorldMap mixed_hardware_demo();
    static WorldMap thesis_demo(UnstructuredMapPreset preset = UnstructuredMapPreset::RobotValidation,
                                GateBehaviorMode gate_behavior = GateBehaviorMode::Static,
                                std::uint32_t gate_seed = 0);

    const Rect& bounds() const { return bounds_; }
    const std::vector<Rect>& obstacles() const { return obstacles_; }
    const std::vector<GateSpec>& gates() const { return gates_; }
    const std::vector<Vec2>& road_centerline() const { return road_centerline_; }
    const Vec2& start() const { return start_; }
    const Vec2& goal() const { return goal_; }
    double start_heading() const { return start_heading_; }
    GateBehaviorMode gate_behavior() const { return gate_behavior_; }
    std::uint32_t gate_seed() const { return gate_seed_; }
    EnvironmentMode environment_mode() const { return environment_mode_; }
    UnstructuredMapPreset unstructured_preset() const { return unstructured_preset_; }
    StructuredMapPreset structured_preset() const { return structured_preset_; }

    bool line_of_sight(const Vec2& from, const Vec2& to, double padding = 0.15) const;
    std::vector<LidarHit> raycast(const Vec2& origin, double heading, int beams, double fov_rad, double max_range) const;
    bool collides(const std::array<Vec2, 4>& polygon, double padding = 0.05) const;
    void set_gate_behavior(GateBehaviorMode gate_behavior, std::uint32_t gate_seed);
    void reset_gate_layout(std::uint32_t gate_seed);
    void update_gate_layout(double sim_time_s);
    void set_bounds(const Rect& bounds) { bounds_ = bounds; }
    void set_start(const Vec2& start) { start_ = start; }
    void set_goal(const Vec2& goal) { goal_ = goal; }
    void set_start_heading(double start_heading) { start_heading_ = start_heading; }
    std::vector<Rect>& editable_obstacles() { return obstacles_; }
    std::vector<GateSpec>& editable_gates() { return gates_; }
    std::vector<Vec2>& editable_road_centerline() { return road_centerline_; }
    void finalize_editor_changes();

  private:
    Rect bounds_;
    Vec2 start_;
    Vec2 goal_;
    double start_heading_ = 0.0;
    std::vector<Rect> obstacles_;
    std::vector<GateSpec> gates_;
    std::vector<GateSpec> gate_templates_;
    std::vector<Vec2> road_centerline_;
    EnvironmentMode environment_mode_ = EnvironmentMode::UnstructuredGates;
    UnstructuredMapPreset unstructured_preset_ = UnstructuredMapPreset::RobotValidation;
    StructuredMapPreset structured_preset_ = StructuredMapPreset::ValidationRoad;
    GateBehaviorMode gate_behavior_ = GateBehaviorMode::Static;
    std::uint32_t gate_seed_ = 0;
};

double distance(const Vec2& a, const Vec2& b);
double angle_to(const Vec2& from, const Vec2& to);
Vec2 rotate(const Vec2& point, double yaw);
std::array<Vec2, 4> make_box_corners(const Vec2& center, double yaw, double length, double width);

}  // namespace thesis_sim
