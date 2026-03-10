#pragma once

#include <array>
#include <string>
#include <vector>

namespace thesis_sim {

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
    static WorldMap thesis_demo();

    const Rect& bounds() const { return bounds_; }
    const std::vector<Rect>& obstacles() const { return obstacles_; }
    const std::vector<GateSpec>& gates() const { return gates_; }
    const Vec2& start() const { return start_; }
    const Vec2& goal() const { return goal_; }
    double start_heading() const { return start_heading_; }

    bool line_of_sight(const Vec2& from, const Vec2& to, double padding = 0.15) const;
    std::vector<LidarHit> raycast(const Vec2& origin, double heading, int beams, double fov_rad, double max_range) const;
    bool collides(const std::array<Vec2, 4>& polygon, double padding = 0.05) const;

  private:
    Rect bounds_;
    Vec2 start_;
    Vec2 goal_;
    double start_heading_ = 0.0;
    std::vector<Rect> obstacles_;
    std::vector<GateSpec> gates_;
};

double distance(const Vec2& a, const Vec2& b);
double angle_to(const Vec2& from, const Vec2& to);
Vec2 rotate(const Vec2& point, double yaw);
std::array<Vec2, 4> make_box_corners(const Vec2& center, double yaw, double length, double width);

}  // namespace thesis_sim
