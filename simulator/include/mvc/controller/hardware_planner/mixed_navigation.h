#pragma once

// Hardware-only navigation utilities. No simulator runtime uses this module.
#include "Clothoids.hh"
#include "mvc/model/navigation/mpc_path_follower.h"
#include <deque>
#include <string>

namespace thesis_sim::hardware_mixed {

enum class State { RoadFollowing, GateAcquisition, GateFollowing, RoadRejoining, SafeStop, GoalReached };
const char* state_name(State state);

struct Config {
    bool enabled = false;
    // In this frame the captured robot pose defines (0,0,0), not an arena corner.
    bool start_reference_frame = false;
    Vec2 arena_size_m{2.00, 1.70};
    std::vector<double> rejoin_lookaheads_m{0.35, 0.55, 0.80, 1.10, 1.50};
    double road_preview_m = 0.65;
    double lateral_tolerance_m = 0.045;
    double heading_tolerance_rad = 0.12;
    double completion_progress_m = 0.15;
    double obstacle_margin_m = 0.035;
    double pose_margin_m = 0.025;
    double max_pose_sigma_m = 0.60;
    double max_yaw_sigma_rad = 0.35;
    double projection_back_m = 0.08;
    double projection_forward_m = 0.25;
    double max_pose_jump_m = 0.15;
    double max_yaw_jump_rad = 0.35;
    double replan_interval_s = 0.60;
    double observation_memory_s = 0.60;
    double max_beam_gap_rad = 0.055;
    double braking_decel_mps2 = 0.20;
    double reaction_time_s = 0.40;
    double max_yaw_accel_rad_s2 = 0.60;
    double sample_spacing_m = 0.02;
    double gate_timeout_s = 30.0;
    double gate_reject_radius_m = 0.22;
    int completion_frames = 3;
};

struct Projection {
    bool valid = false;
    double s = 0.0, n = 0.0, heading_error = 0.0, distance = 0.0;
};

class ReferenceRoad {
public:
    void build(const std::vector<Vec2>& points, double start_yaw);
    double length() const { return curve_.length(); }
    ReferenceWaypoint sample(double s, double speed = 0.0) const;
    Projection project(const Vec2& position, double yaw, double hint,
                       double backward, double forward) const;
    std::vector<double> targets(double s, const std::vector<double>& offsets) const;
    std::vector<ReferenceWaypoint> span(double s, double extent, double spacing, double speed) const;
private:
    G2lib::ClothoidList curve_{"hardware_mixed_reference"};
};

struct PathResult {
    std::vector<ReferenceWaypoint> path;
    std::string reason;
    double length = 0.0, bending = 0.0, clearance = 0.0;
    bool valid() const { return path.size() >= 2 && reason == "accepted"; }
};

// Uses the same G2 clothoid library as the existing Sabrina primitive planner.
PathResult connect(const ReferenceWaypoint& from, const ReferenceWaypoint& to,
                   const VehicleGeometry& geometry, VehicleModelKind model,
                   const Config& config, double speed, double current_speed);

struct Ray { double angle = 0.0, range = 0.0; bool occupied = true; };
struct Scan {
    Vec2 origin;
    double timestamp = 0.0;
    double motion_uncertainty_m = 0.0;
    std::vector<Ray> rays;
    std::vector<Vec2> occupied_points;
};
class ObservedSpace {
public:
    void clear() { scans_.clear(); }
    void add(Scan scan, double memory_s);
    bool known_free(const Vec2& point, double now, const Config& config) const;
    PathResult validate(const std::vector<ReferenceWaypoint>& path,
                        const ReferenceWaypoint& current, const VehicleGeometry& geometry,
                        const Rect& bounds, double now, const Config& config) const;
private:
    std::deque<Scan> scans_;
};

bool gate_cleared(const Vec2& position, double yaw, const Vec2& entry,
                  const Vec2& center, double normal_yaw, double width,
                  const VehicleGeometry& geometry, double margin);
bool rejoin_complete(const Projection& p, double entry_s, double target_s,
                     bool next_road_clear, const Config& config);

} // namespace thesis_sim::hardware_mixed
