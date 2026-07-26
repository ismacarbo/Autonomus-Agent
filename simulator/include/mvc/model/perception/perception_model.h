#pragma once

#include <cstdint>
#include <random>
#include <unordered_set>
#include <vector>

#include "mvc/model/world/world.h"

namespace thesis_sim {

enum class RangeSensorProfile {
    IdealLidar2D = 0,
    RplidarA1 = 1,
    ShortRangeScanner = 2,
};

const char* range_sensor_profile_name(RangeSensorProfile profile);

}  // namespace thesis_sim

namespace thesis_sim::mvc::model {

struct RangeSensorSpec {
    int beams = 0;
    double fov_rad = 0.0;
    double range_m = 0.0;
};

struct SimulatedLidarConfig {
    bool enabled = true;
    RangeSensorProfile profile = RangeSensorProfile::RplidarA1;
    int fallback_beams = 181;
    double fallback_fov_rad = 3.14159265358979323846;
    double fallback_range_m = 8.0;
    bool calibrated = false;
    double range_noise_std_m = 0.0;
    double dropout_probability = 0.0;
};

RangeSensorSpec range_sensor_spec(const SimulatedLidarConfig& config);

// Simulated sensor acquisition. The world update belongs here because
// perception-only obstacles must become visible at scan time, not at render
// time or during planner selection.
std::vector<LidarHit> acquire_simulated_lidar_scan(
    WorldMap* world,
    const Vec2& sensor_position,
    double sensor_yaw,
    double simulation_time_s,
    double structured_progress_m,
    const SimulatedLidarConfig& config,
    std::mt19937* random_engine);

double minimum_lidar_distance(const std::vector<LidarHit>& hits,
                              double maximum_range_m);

// Occupancy endpoint accumulation shared by report export and the GUI's
// fallback SLAM representation. Pose estimation remains outside this model.
void accumulate_slam_endpoints(
    const std::vector<LidarHit>& hits,
    double physical_sensor_yaw,
    const Vec2& navigation_position,
    double navigation_yaw,
    const Rect& bounds,
    std::vector<Vec2>* slam_points,
    std::unordered_set<std::uint64_t>* occupied_cells,
    std::size_t maximum_points = 90000);

}  // namespace thesis_sim::mvc::model
