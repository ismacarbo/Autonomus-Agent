#include "mvc/model/perception/perception_model.h"

#include <algorithm>
#include <cmath>

namespace thesis_sim::mvc::model {
namespace {

constexpr double kPi = 3.14159265358979323846;

double clamp_value(double value, double lo, double hi) {
    return std::max(lo, std::min(value, hi));
}

double wrap_angle(double angle) {
    while (angle > kPi) {
        angle -= 2.0 * kPi;
    }
    while (angle < -kPi) {
        angle += 2.0 * kPi;
    }
    return angle;
}

}  // namespace

RangeSensorSpec range_sensor_spec(const SimulatedLidarConfig& config) {
    switch (config.profile) {
        case RangeSensorProfile::IdealLidar2D:
            return {
                std::max(config.fallback_beams, 1),
                config.fallback_fov_rad,
                config.fallback_range_m,
            };
        case RangeSensorProfile::RplidarA1:
            return {360, 2.0 * kPi, std::max(config.fallback_range_m, 0.10)};
        case RangeSensorProfile::ShortRangeScanner:
            return {121, 2.0 * kPi / 3.0, 4.5};
        default:
            return {
                std::max(config.fallback_beams, 1),
                config.fallback_fov_rad,
                config.fallback_range_m,
            };
    }
}

std::vector<LidarHit> acquire_simulated_lidar_scan(
    WorldMap* world,
    const Vec2& sensor_position,
    double sensor_yaw,
    double simulation_time_s,
    double structured_progress_m,
    const SimulatedLidarConfig& config,
    std::mt19937* random_engine) {
    if (world == nullptr || !config.enabled) {
        return {};
    }

    const RangeSensorSpec sensor = range_sensor_spec(config);
    world->update_perception_obstacles(
        simulation_time_s,
        structured_progress_m,
        &sensor_position,
        sensor.range_m);
    std::vector<LidarHit> hits = world->raycast(
        sensor_position,
        sensor_yaw,
        sensor.beams,
        sensor.fov_rad,
        sensor.range_m,
        true);

    if (!config.calibrated || random_engine == nullptr) {
        return hits;
    }

    std::normal_distribution<double> range_noise(
        0.0,
        std::max(config.range_noise_std_m, 0.0));
    std::bernoulli_distribution dropout(
        clamp_value(config.dropout_probability, 0.0, 1.0));
    for (LidarHit& hit : hits) {
        if (!hit.hit) {
            continue;
        }
        if (dropout(*random_engine)) {
            hit.hit = false;
            hit.distance = sensor.range_m;
        } else {
            hit.distance = clamp_value(
                hit.distance + range_noise(*random_engine),
                0.02,
                sensor.range_m);
        }
        hit.point = {
            sensor_position.x + std::cos(hit.angle) * hit.distance,
            sensor_position.y + std::sin(hit.angle) * hit.distance,
        };
    }
    return hits;
}

double minimum_lidar_distance(const std::vector<LidarHit>& hits,
                              double maximum_range_m) {
    double minimum = maximum_range_m;
    for (const LidarHit& hit : hits) {
        minimum = std::min(minimum, hit.distance);
    }
    return minimum;
}

void accumulate_slam_endpoints(
    const std::vector<LidarHit>& hits,
    double physical_sensor_yaw,
    const Vec2& navigation_position,
    double navigation_yaw,
    const Rect& bounds,
    std::vector<Vec2>* slam_points,
    std::unordered_set<std::uint64_t>* occupied_cells,
    std::size_t maximum_points) {
    if (slam_points == nullptr || occupied_cells == nullptr ||
        slam_points->size() >= maximum_points) {
        return;
    }

    const double span =
        std::max(bounds.max_x - bounds.min_x, bounds.max_y - bounds.min_y);
    const double resolution = span <= 2.0 ? 0.015 : (span <= 8.0 ? 0.035 : 0.08);
    for (const LidarHit& hit : hits) {
        if (!hit.hit || hit.distance <= 0.02) {
            continue;
        }
        const double local_angle = wrap_angle(hit.angle - physical_sensor_yaw);
        const double mapped_angle = navigation_yaw + local_angle;
        const Vec2 mapped_point{
            navigation_position.x + std::cos(mapped_angle) * hit.distance,
            navigation_position.y + std::sin(mapped_angle) * hit.distance,
        };
        if (mapped_point.x < bounds.min_x - 0.05 ||
            mapped_point.x > bounds.max_x + 0.05 ||
            mapped_point.y < bounds.min_y - 0.05 ||
            mapped_point.y > bounds.max_y + 0.05) {
            continue;
        }

        const std::int32_t cell_x =
            static_cast<std::int32_t>(std::floor(mapped_point.x / resolution));
        const std::int32_t cell_y =
            static_cast<std::int32_t>(std::floor(mapped_point.y / resolution));
        const std::uint64_t key =
            (static_cast<std::uint64_t>(static_cast<std::uint32_t>(cell_x)) << 32U) |
            static_cast<std::uint32_t>(cell_y);
        if (occupied_cells->insert(key).second) {
            slam_points->push_back({
                (static_cast<double>(cell_x) + 0.5) * resolution,
                (static_cast<double>(cell_y) + 0.5) * resolution,
            });
            if (slam_points->size() >= maximum_points) {
                break;
            }
        }
    }
}

}  // namespace thesis_sim::mvc::model
