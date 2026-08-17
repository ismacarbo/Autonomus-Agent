#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "mvc/model/navigation/initial_lidar_matcher.h"

namespace {

constexpr double kPi = 3.14159265358979323846;

std::vector<thesis_sim::Vec2> make_asymmetric_reference() {
    std::vector<thesis_sim::Vec2> points;
    for (int i = 0; i <= 80; ++i) {
        const double alpha = static_cast<double>(i) / 80.0;
        points.push_back({-0.85 + 1.70 * alpha, -0.62});
    }
    for (int i = 0; i <= 55; ++i) {
        const double alpha = static_cast<double>(i) / 55.0;
        points.push_back({0.83, -0.62 + 1.10 * alpha});
    }
    for (int i = 0; i <= 35; ++i) {
        const double alpha = static_cast<double>(i) / 35.0;
        points.push_back({-0.72 + 0.48 * alpha, 0.46 + 0.10 * std::sin(alpha * kPi)});
    }
    for (int i = 0; i <= 30; ++i) {
        const double alpha = static_cast<double>(i) / 30.0;
        points.push_back({0.08 + 0.24 * alpha, -0.12 + 0.42 * alpha});
    }
    return points;
}

std::vector<thesis_sim::Vec2> express_reference_in_current_frame(
    const std::vector<thesis_sim::Vec2>& reference,
    const thesis_sim::Vec2& current_position,
    double current_yaw) {
    std::vector<thesis_sim::Vec2> current;
    current.reserve(reference.size());
    const double c = std::cos(current_yaw);
    const double s = std::sin(current_yaw);
    for (const thesis_sim::Vec2& point : reference) {
        const double dx = point.x - current_position.x;
        const double dy = point.y - current_position.y;
        current.push_back({c * dx + s * dy, -s * dx + c * dy});
    }
    return current;
}

bool close_to(double actual, double expected, double tolerance, const char* label) {
    if (std::abs(actual - expected) <= tolerance) return true;
    std::cerr << label << " expected=" << expected << " actual=" << actual << '\n';
    return false;
}

}  // namespace

int main() {
    using thesis_sim::mvc::model::InitialLidarConfirmationConfig;
    using thesis_sim::mvc::model::InitialLidarReferenceCloud;
    using thesis_sim::mvc::model::InitialLidarMatcher;
    using thesis_sim::mvc::model::InitialLidarMatcherConfig;
    using thesis_sim::mvc::model::confirm_initial_lidar_matches;
    using thesis_sim::mvc::model::read_initial_lidar_reference;
    using thesis_sim::mvc::model::write_initial_lidar_reference;

    const std::vector<thesis_sim::Vec2> reference = make_asymmetric_reference();
    const thesis_sim::Vec2 expected_position{0.035, -0.025};
    const double expected_yaw = 0.061;
    const std::vector<thesis_sim::Vec2> current = express_reference_in_current_frame(
        reference, expected_position, expected_yaw);

    InitialLidarMatcherConfig config;
    config.minimum_points = 50;
    config.minimum_matched_points = 40;
    config.maximum_start_translation_m = 0.08;
    config.maximum_start_yaw_rad = 8.0 * kPi / 180.0;
    const auto result = InitialLidarMatcher(config).match(reference, current);
    if (!result.valid || !result.accepted ||
        !close_to(result.position_offset_m.x, expected_position.x, 0.012, "x") ||
        !close_to(result.position_offset_m.y, expected_position.y, 0.012, "y") ||
        !close_to(result.yaw_offset_rad, expected_yaw, 0.012, "yaw") ||
        result.inlier_ratio < 0.90 || result.rmse_m > 0.018) {
        std::cerr << "matching_failed status=" << result.status
                  << " rmse=" << result.rmse_m
                  << " inliers=" << result.inlier_ratio << '\n';
        return 1;
    }

    const thesis_sim::Vec2 outside_position{0.14, -0.02};
    const auto outside = InitialLidarMatcher(config).match(
        reference,
        express_reference_in_current_frame(reference, outside_position, 0.02));
    if (!outside.valid || outside.accepted ||
        outside.status != "start_pose_outside_tolerance") {
        std::cerr << "start_tolerance_failed status=" << outside.status << '\n';
        return 1;
    }

    const std::vector<thesis_sim::Vec2> voxel_input{
        {0.001, 0.001}, {0.004, 0.003}, {0.101, 0.101}, {0.104, 0.102}};
    const auto voxelized = thesis_sim::mvc::model::voxelize_initial_lidar_cloud(
        voxel_input, 0.02, 2);
    if (voxelized.size() != 2U) {
        std::cerr << "voxelization_failed size=" << voxelized.size() << '\n';
        return 1;
    }

    std::vector<thesis_sim::mvc::model::InitialLidarMatchResult> confirmations{
        result, result, result};
    confirmations[0].position_offset_m.x -= 0.003;
    confirmations[1].position_offset_m.y += 0.002;
    confirmations[2].yaw_offset_rad += 0.004;
    InitialLidarConfirmationConfig confirmation_config;
    const auto confirmed = confirm_initial_lidar_matches(
        confirmations, confirmation_config);
    if (!confirmed.valid || !confirmed.accepted ||
        confirmed.status != "start_pose_confirmed" ||
        confirmed.valid_matches != 3 || confirmed.accepted_matches != 3) {
        std::cerr << "confirmation_failed status=" << confirmed.status << '\n';
        return 1;
    }

    confirmations[2].position_offset_m.x += 0.08;
    const auto inconsistent = confirm_initial_lidar_matches(
        confirmations, confirmation_config);
    if (!inconsistent.valid || inconsistent.accepted ||
        inconsistent.status != "confirmation_matches_inconsistent") {
        std::cerr << "confirmation_consistency_failed status="
                  << inconsistent.status << '\n';
        return 1;
    }

    InitialLidarReferenceCloud reference_file;
    reference_file.metadata.created_utc = "2026-08-06T00:00:00Z";
    reference_file.metadata.lidar_serial = "unit-test-lidar";
    reference_file.metadata.lidar_firmware = "1.2";
    reference_file.metadata.source_scan_count = 8;
    reference_file.metadata.source_raw_point_count = 1234;
    reference_file.metadata.stability_valid = true;
    reference_file.metadata.stability_translation_m = 0.002;
    reference_file.metadata.stability_yaw_rad = 0.003;
    reference_file.metadata.stability_rmse_m = 0.006;
    reference_file.metadata.stability_inlier_ratio = 0.91;
    reference_file.metadata.raw_scan_log_path = "raw_scan_test.csv";
    reference_file.points = reference;
    const std::filesystem::path reference_path =
        std::filesystem::temp_directory_path() /
        "thesis_initial_lidar_reference_integrity_test.csv";
    std::string file_error;
    if (!write_initial_lidar_reference(
            reference_path.string(), reference_file, &file_error)) {
        std::cerr << "reference_write_failed error=" << file_error << '\n';
        return 1;
    }
    InitialLidarReferenceCloud read_back;
    if (!read_initial_lidar_reference(
            reference_path.string(), &read_back, &file_error) ||
        !read_back.integrity_verified ||
        read_back.metadata.lidar_serial != "unit-test-lidar") {
        std::cerr << "reference_integrity_roundtrip_failed error=" << file_error << '\n';
        std::filesystem::remove(reference_path);
        return 1;
    }
    {
        std::ofstream tampered(reference_path, std::ios::out | std::ios::app);
        tampered << "0.123456789,0.987654321\n";
    }
    InitialLidarReferenceCloud rejected;
    if (read_initial_lidar_reference(
            reference_path.string(), &rejected, &file_error) ||
        file_error.find("integrity check failed") == std::string::npos) {
        std::cerr << "reference_tamper_detection_failed error=" << file_error << '\n';
        std::filesystem::remove(reference_path);
        return 1;
    }
    std::filesystem::remove(reference_path);
    return 0;
}
