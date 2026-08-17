#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "mvc/model/world/world.h"

namespace thesis_sim::mvc::model {

struct InitialLidarReferenceMetadata {
    std::string schema = "thesis_initial_lidar_reference_v2";
    std::string created_utc;
    std::string lidar_serial;
    std::string lidar_firmware;
    int lidar_hardware = 0;
    double lidar_x_offset_m = 0.075;
    double lidar_y_offset_m = 0.040;
    double lidar_yaw_offset_rad = 2.82743338823;
    bool lidar_flip_left_right = true;
    double min_range_m = 0.12;
    double max_range_m = 3.0;
    double voxel_size_m = 0.018;
    double body_length_m = 0.25;
    double body_width_m = 0.15;
    int source_scan_count = 0;
    int source_raw_point_count = 0;
    bool stability_valid = false;
    double stability_translation_m = 0.0;
    double stability_yaw_rad = 0.0;
    double stability_rmse_m = 0.0;
    double stability_inlier_ratio = 0.0;
    std::string raw_scan_log_path;
    std::string content_hash_fnv1a64;
};

struct InitialLidarReferenceCloud {
    InitialLidarReferenceMetadata metadata{};
    std::vector<Vec2> points;
    bool integrity_verified = false;
    std::string computed_content_hash;
};

struct InitialLidarMatcherConfig {
    double search_translation_m = 0.30;
    double coarse_translation_step_m = 0.04;
    double fine_translation_step_m = 0.010;
    double search_yaw_rad = 0.523598775598;
    double coarse_yaw_step_rad = 0.052359877560;
    double fine_yaw_step_rad = 0.008726646260;
    double max_correspondence_distance_m = 0.11;
    double trim_fraction = 0.78;
    int minimum_points = 60;
    int minimum_matched_points = 42;
    double maximum_rmse_m = 0.045;
    double minimum_inlier_ratio = 0.55;
    double minimum_ambiguity_margin = 0.025;
    double maximum_start_translation_m = 0.080;
    double maximum_start_yaw_rad = 0.139626340160;
    double maximum_compute_time_ms = 20000.0;
};

struct InitialLidarMatchResult {
    bool valid = false;
    bool accepted = false;
    Vec2 position_offset_m{};
    double yaw_offset_rad = 0.0;
    double score_m = 0.0;
    double rmse_m = 0.0;
    double inlier_ratio = 0.0;
    double ambiguity_margin = 0.0;
    double confidence = 0.0;
    double covariance_x_m2 = 0.0;
    double covariance_y_m2 = 0.0;
    double covariance_yaw_rad2 = 0.0;
    double observability_x = 0.0;
    double observability_y = 0.0;
    double observability_yaw = 0.0;
    double coarse_compute_ms = 0.0;
    double fine_compute_ms = 0.0;
    double total_compute_ms = 0.0;
    std::uint64_t evaluated_candidates = 0;
    bool timed_out = false;
    int reference_points = 0;
    int current_points = 0;
    int matched_points = 0;
    std::string status;
};

struct InitialLidarConfirmationConfig {
    int required_matches = 3;
    double maximum_position_spread_m = 0.025;
    double maximum_yaw_spread_rad = 0.034906585040;
};

struct InitialLidarConfirmationResult {
    bool valid = false;
    bool accepted = false;
    InitialLidarMatchResult fused_match{};
    int supplied_matches = 0;
    int valid_matches = 0;
    int accepted_matches = 0;
    double maximum_position_spread_m = 0.0;
    double maximum_yaw_spread_rad = 0.0;
    std::string status;
};

class InitialLidarMatcher {
  public:
    explicit InitialLidarMatcher(InitialLidarMatcherConfig config = {});

    InitialLidarMatchResult match(const std::vector<Vec2>& reference_points,
                                  const std::vector<Vec2>& current_points) const;

    const InitialLidarMatcherConfig& config() const { return config_; }

  private:
    InitialLidarMatcherConfig config_;
};

InitialLidarConfirmationResult confirm_initial_lidar_matches(
    const std::vector<InitialLidarMatchResult>& matches,
    const InitialLidarConfirmationConfig& config = {});

std::vector<Vec2> voxelize_initial_lidar_cloud(const std::vector<Vec2>& points,
                                               double voxel_size_m,
                                               int minimum_observations = 1);

std::vector<Vec2> transform_initial_lidar_cloud(const std::vector<Vec2>& points,
                                                const Vec2& translation,
                                                double yaw);

bool write_initial_lidar_reference(const std::string& path,
                                   const InitialLidarReferenceCloud& reference,
                                   std::string* error = nullptr);

bool read_initial_lidar_reference(const std::string& path,
                                  InitialLidarReferenceCloud* reference,
                                  std::string* error = nullptr);

std::string initial_lidar_reference_hash(
    const InitialLidarReferenceCloud& reference);

bool write_initial_lidar_cloud_comparison_csv(
    const std::string& path,
    const std::vector<Vec2>& reference,
    const std::vector<Vec2>& current,
    const std::vector<Vec2>& aligned,
    std::string* error = nullptr);

bool write_initial_lidar_cloud_comparison_ply(
    const std::string& path,
    const std::vector<Vec2>& reference,
    const std::vector<Vec2>& current,
    const std::vector<Vec2>& aligned,
    std::string* error = nullptr);

}  // namespace thesis_sim::mvc::model
