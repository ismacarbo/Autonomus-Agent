#include "mvc/model/navigation/initial_lidar_matcher.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <unordered_map>
#include <utility>

namespace thesis_sim::mvc::model {
namespace {

constexpr double kPi = 3.14159265358979323846;

double clamp_value(double value, double lo, double hi) {
    return std::max(lo, std::min(value, hi));
}

double wrap_angle(double angle) {
    while (angle > kPi) angle -= 2.0 * kPi;
    while (angle < -kPi) angle += 2.0 * kPi;
    return angle;
}

double elapsed_ms(std::chrono::steady_clock::time_point start,
                  std::chrono::steady_clock::time_point end) {
    return std::chrono::duration<double, std::milli>(end - start).count();
}

std::uint64_t cell_key(int x, int y) {
    return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(x)) << 32U) |
           static_cast<std::uint32_t>(y);
}

struct AccumulatedPoint {
    Vec2 sum{};
    int count = 0;
};

struct MatchEvaluation {
    bool valid = false;
    double score = std::numeric_limits<double>::infinity();
    double rmse = std::numeric_limits<double>::infinity();
    double inlier_ratio = 0.0;
    int matched = 0;
};

struct MatchCandidate {
    Vec2 translation{};
    double yaw = 0.0;
    MatchEvaluation evaluation{};
};

class ReferenceIndex {
  public:
    ReferenceIndex(const std::vector<Vec2>& points, double cell_size)
        : cell_size_(std::max(cell_size, 1e-4)) {
        for (const Vec2& point : points) {
            if (!std::isfinite(point.x) || !std::isfinite(point.y)) continue;
            cells_[key_for(point)].push_back(point);
        }
    }

    double nearest_distance_sq(const Vec2& query, double maximum_distance) const {
        const int cell_x = static_cast<int>(std::floor(query.x / cell_size_));
        const int cell_y = static_cast<int>(std::floor(query.y / cell_size_));
        const int radius = std::max(1, static_cast<int>(std::ceil(maximum_distance / cell_size_)));
        double best = maximum_distance * maximum_distance;
        bool found = false;
        for (int dx = -radius; dx <= radius; ++dx) {
            for (int dy = -radius; dy <= radius; ++dy) {
                const auto it = cells_.find(cell_key(cell_x + dx, cell_y + dy));
                if (it == cells_.end()) continue;
                for (const Vec2& point : it->second) {
                    const double ex = query.x - point.x;
                    const double ey = query.y - point.y;
                    const double distance_sq = ex * ex + ey * ey;
                    if (distance_sq <= best) {
                        best = distance_sq;
                        found = true;
                    }
                }
            }
        }
        return found ? best : std::numeric_limits<double>::infinity();
    }

  private:
    std::uint64_t key_for(const Vec2& point) const {
        return cell_key(static_cast<int>(std::floor(point.x / cell_size_)),
                        static_cast<int>(std::floor(point.y / cell_size_)));
    }

    double cell_size_ = 0.1;
    std::unordered_map<std::uint64_t, std::vector<Vec2>> cells_;
};

MatchEvaluation evaluate_candidate(const ReferenceIndex& reference,
                                   const std::vector<Vec2>& current_points,
                                   const InitialLidarMatcherConfig& config,
                                   const Vec2& translation,
                                   double yaw) {
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    std::vector<double> squared_distances;
    squared_distances.reserve(current_points.size());
    for (const Vec2& point : current_points) {
        const Vec2 transformed{
            translation.x + c * point.x - s * point.y,
            translation.y + s * point.x + c * point.y,
        };
        const double distance_sq = reference.nearest_distance_sq(
            transformed, config.max_correspondence_distance_m);
        if (std::isfinite(distance_sq)) squared_distances.push_back(distance_sq);
    }

    MatchEvaluation evaluation;
    evaluation.matched = static_cast<int>(squared_distances.size());
    evaluation.inlier_ratio = current_points.empty()
                                  ? 0.0
                                  : static_cast<double>(evaluation.matched) /
                                        static_cast<double>(current_points.size());
    if (evaluation.matched < config.minimum_matched_points) return evaluation;

    std::sort(squared_distances.begin(), squared_distances.end());
    const int retained = std::clamp(
        static_cast<int>(std::ceil(config.trim_fraction * squared_distances.size())),
        config.minimum_matched_points,
        static_cast<int>(squared_distances.size()));
    double sum = 0.0;
    for (int i = 0; i < retained; ++i) sum += squared_distances[static_cast<std::size_t>(i)];
    evaluation.rmse = std::sqrt(sum / static_cast<double>(retained));
    const double unmatched_penalty =
        0.65 * config.max_correspondence_distance_m * (1.0 - evaluation.inlier_ratio);
    evaluation.score = evaluation.rmse + unmatched_penalty;
    evaluation.valid = std::isfinite(evaluation.score);
    return evaluation;
}

bool candidate_better(const MatchCandidate& lhs, const MatchCandidate& rhs) {
    return lhs.evaluation.score < rhs.evaluation.score;
}

std::vector<Vec2> uniformly_sample_cloud(const std::vector<Vec2>& points,
                                         std::size_t maximum_points) {
    if (points.size() <= maximum_points || maximum_points == 0U) return points;
    std::vector<Vec2> sampled;
    sampled.reserve(maximum_points);
    const double stride = static_cast<double>(points.size()) /
                          static_cast<double>(maximum_points);
    for (std::size_t i = 0; i < maximum_points; ++i) {
        const std::size_t index = std::min(
            static_cast<std::size_t>(std::floor(static_cast<double>(i) * stride)),
            points.size() - 1U);
        sampled.push_back(points[index]);
    }
    return sampled;
}

void consider_candidate(const MatchCandidate& candidate,
                        MatchCandidate* best,
                        std::vector<MatchCandidate>* coarse_candidates) {
    if (!candidate.evaluation.valid) return;
    if (!best->evaluation.valid || candidate_better(candidate, *best)) *best = candidate;
    if (coarse_candidates == nullptr) return;
    coarse_candidates->push_back(candidate);
}

bool search_grid(const ReferenceIndex& reference,
                 const std::vector<Vec2>& current_points,
                 const InitialLidarMatcherConfig& config,
                 const Vec2& center,
                 double center_yaw,
                 double translation_radius,
                 double translation_step,
                 double yaw_radius,
                 double yaw_step,
                 std::chrono::steady_clock::time_point deadline,
                 std::uint64_t* evaluated_candidates,
                 MatchCandidate* best,
                 std::vector<MatchCandidate>* candidates = nullptr) {
    const int xy_steps = std::max(0, static_cast<int>(std::ceil(translation_radius /
                                                               std::max(translation_step, 1e-5))));
    const int yaw_steps = std::max(0, static_cast<int>(std::ceil(yaw_radius /
                                                                std::max(yaw_step, 1e-5))));
    for (int ix = -xy_steps; ix <= xy_steps; ++ix) {
        for (int iy = -xy_steps; iy <= xy_steps; ++iy) {
            for (int ia = -yaw_steps; ia <= yaw_steps; ++ia) {
                if (evaluated_candidates != nullptr &&
                    ((*evaluated_candidates) & 0x3FU) == 0U &&
                    std::chrono::steady_clock::now() >= deadline) {
                    return false;
                }
                MatchCandidate candidate;
                candidate.translation = {
                    center.x + static_cast<double>(ix) * translation_step,
                    center.y + static_cast<double>(iy) * translation_step,
                };
                candidate.yaw = wrap_angle(center_yaw + static_cast<double>(ia) * yaw_step);
                candidate.evaluation = evaluate_candidate(
                    reference, current_points, config, candidate.translation, candidate.yaw);
                if (evaluated_candidates != nullptr) ++(*evaluated_candidates);
                consider_candidate(candidate, best, candidates);
            }
        }
    }
    return true;
}

bool parse_metadata_line(const std::string& line,
                         InitialLidarReferenceMetadata* metadata) {
    if (line.size() < 3U || line[0] != '#') return false;
    const std::size_t equals = line.find('=');
    if (equals == std::string::npos) return false;
    const std::string key = line.substr(1, equals - 1);
    const std::string value = line.substr(equals + 1);
    try {
        if (key == "schema") metadata->schema = value;
        else if (key == "created_utc") metadata->created_utc = value;
        else if (key == "lidar_serial") metadata->lidar_serial = value;
        else if (key == "lidar_firmware") metadata->lidar_firmware = value;
        else if (key == "lidar_hardware") metadata->lidar_hardware = std::stoi(value);
        else if (key == "lidar_x_offset_m") metadata->lidar_x_offset_m = std::stod(value);
        else if (key == "lidar_y_offset_m") metadata->lidar_y_offset_m = std::stod(value);
        else if (key == "lidar_yaw_offset_rad") metadata->lidar_yaw_offset_rad = std::stod(value);
        else if (key == "lidar_flip_left_right") metadata->lidar_flip_left_right = std::stoi(value) != 0;
        else if (key == "min_range_m") metadata->min_range_m = std::stod(value);
        else if (key == "max_range_m") metadata->max_range_m = std::stod(value);
        else if (key == "voxel_size_m") metadata->voxel_size_m = std::stod(value);
        else if (key == "body_length_m") metadata->body_length_m = std::stod(value);
        else if (key == "body_width_m") metadata->body_width_m = std::stod(value);
        else if (key == "source_scan_count") metadata->source_scan_count = std::stoi(value);
        else if (key == "source_raw_point_count") metadata->source_raw_point_count = std::stoi(value);
        else if (key == "stability_valid") metadata->stability_valid = std::stoi(value) != 0;
        else if (key == "stability_translation_m") metadata->stability_translation_m = std::stod(value);
        else if (key == "stability_yaw_rad") metadata->stability_yaw_rad = std::stod(value);
        else if (key == "stability_rmse_m") metadata->stability_rmse_m = std::stod(value);
        else if (key == "stability_inlier_ratio") metadata->stability_inlier_ratio = std::stod(value);
        else if (key == "raw_scan_log_path") metadata->raw_scan_log_path = value;
        else if (key == "content_hash_fnv1a64") metadata->content_hash_fnv1a64 = value;
        else return false;
        return true;
    } catch (...) {
        return false;
    }
}

void fnv1a_update(std::uint64_t* hash, const std::string& value) {
    constexpr std::uint64_t kPrime = 1099511628211ULL;
    for (unsigned char byte : value) {
        *hash ^= static_cast<std::uint64_t>(byte);
        *hash *= kPrime;
    }
}

std::string reference_hash_material(const InitialLidarReferenceCloud& reference) {
    std::ostringstream material;
    const InitialLidarReferenceMetadata& metadata = reference.metadata;
    material << std::setprecision(17)
             << metadata.schema << '\n'
             << metadata.created_utc << '\n'
             << metadata.lidar_serial << '\n'
             << metadata.lidar_firmware << '\n'
             << metadata.lidar_hardware << '\n'
             << metadata.lidar_x_offset_m << '\n'
             << metadata.lidar_y_offset_m << '\n'
             << metadata.lidar_yaw_offset_rad << '\n'
             << metadata.lidar_flip_left_right << '\n'
             << metadata.min_range_m << '\n'
             << metadata.max_range_m << '\n'
             << metadata.voxel_size_m << '\n'
             << metadata.body_length_m << '\n'
             << metadata.body_width_m << '\n'
             << metadata.source_scan_count << '\n'
             << metadata.source_raw_point_count << '\n'
             << metadata.stability_valid << '\n'
             << metadata.stability_translation_m << '\n'
             << metadata.stability_yaw_rad << '\n'
             << metadata.stability_rmse_m << '\n'
             << metadata.stability_inlier_ratio << '\n'
             << metadata.raw_scan_log_path << '\n';
    for (const Vec2& point : reference.points) {
        material << point.x << ',' << point.y << '\n';
    }
    return material.str();
}

}  // namespace

InitialLidarMatcher::InitialLidarMatcher(InitialLidarMatcherConfig config)
    : config_(std::move(config)) {}

InitialLidarMatchResult InitialLidarMatcher::match(
    const std::vector<Vec2>& reference_points,
    const std::vector<Vec2>& current_points) const {
    const auto match_start = std::chrono::steady_clock::now();
    const auto deadline = match_start + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                            std::chrono::duration<double, std::milli>(
                                                std::max(config_.maximum_compute_time_ms, 1.0)));
    InitialLidarMatchResult result;
    result.reference_points = static_cast<int>(reference_points.size());
    result.current_points = static_cast<int>(current_points.size());
    if (result.reference_points < config_.minimum_points ||
        result.current_points < config_.minimum_points) {
        result.status = "too_few_points";
        result.total_compute_ms = elapsed_ms(match_start, std::chrono::steady_clock::now());
        return result;
    }
    if (!(config_.coarse_translation_step_m > 0.0) ||
        !(config_.fine_translation_step_m > 0.0) ||
        !(config_.coarse_yaw_step_rad > 0.0) ||
        !(config_.fine_yaw_step_rad > 0.0) ||
        !(config_.max_correspondence_distance_m > 0.0)) {
        result.status = "invalid_matcher_configuration";
        result.total_compute_ms = elapsed_ms(match_start, std::chrono::steady_clock::now());
        return result;
    }

    const ReferenceIndex index(reference_points, config_.max_correspondence_distance_m);
    const std::vector<Vec2> search_points = uniformly_sample_cloud(current_points, 260U);
    MatchCandidate best;
    std::vector<MatchCandidate> coarse_candidates;
    const auto coarse_start = std::chrono::steady_clock::now();
    const bool coarse_complete = search_grid(index,
                                             search_points,
                                             config_,
                                             {},
                                             0.0,
                                             config_.search_translation_m,
                                             config_.coarse_translation_step_m,
                                             config_.search_yaw_rad,
                                             config_.coarse_yaw_step_rad,
                                             deadline,
                                             &result.evaluated_candidates,
                                             &best,
                                             &coarse_candidates);
    result.coarse_compute_ms = elapsed_ms(coarse_start, std::chrono::steady_clock::now());
    if (!coarse_complete) {
        result.timed_out = true;
        result.status = "matcher_timeout_during_coarse_search";
        result.total_compute_ms = elapsed_ms(match_start, std::chrono::steady_clock::now());
        return result;
    }
    if (!best.evaluation.valid) {
        result.status = "no_valid_correspondence_set";
        result.total_compute_ms = elapsed_ms(match_start, std::chrono::steady_clock::now());
        return result;
    }
    const double coarse_best_score = best.evaluation.score;

    std::sort(coarse_candidates.begin(), coarse_candidates.end(), candidate_better);
    double second_distinct_score = std::numeric_limits<double>::infinity();
    for (const MatchCandidate& candidate : coarse_candidates) {
        const double dx = candidate.translation.x - best.translation.x;
        const double dy = candidate.translation.y - best.translation.y;
        const double translation_delta = std::hypot(dx, dy);
        const double yaw_delta = std::abs(wrap_angle(candidate.yaw - best.yaw));
        if (translation_delta >= 2.0 * config_.coarse_translation_step_m ||
            yaw_delta >= 2.0 * config_.coarse_yaw_step_rad) {
            second_distinct_score = candidate.evaluation.score;
            break;
        }
    }

    const Vec2 coarse_position = best.translation;
    const double coarse_yaw = best.yaw;
    const auto fine_start = std::chrono::steady_clock::now();
    bool fine_complete = search_grid(index,
                                     search_points,
                                     config_,
                                     coarse_position,
                                     coarse_yaw,
                                     config_.coarse_translation_step_m,
                                     config_.fine_translation_step_m,
                                     config_.coarse_yaw_step_rad,
                                     config_.fine_yaw_step_rad,
                                     deadline,
                                     &result.evaluated_candidates,
                                     &best);
    if (fine_complete) {
        fine_complete = search_grid(index,
                                    search_points,
                                    config_,
                                    best.translation,
                                    best.yaw,
                                    1.5 * config_.fine_translation_step_m,
                                    0.5 * config_.fine_translation_step_m,
                                    1.5 * config_.fine_yaw_step_rad,
                                    0.5 * config_.fine_yaw_step_rad,
                                    deadline,
                                    &result.evaluated_candidates,
                                    &best);
    }
    result.fine_compute_ms = elapsed_ms(fine_start, std::chrono::steady_clock::now());
    if (!fine_complete) {
        result.timed_out = true;
        result.status = "matcher_timeout_during_fine_search";
        result.total_compute_ms = elapsed_ms(match_start, std::chrono::steady_clock::now());
        return result;
    }

    best.evaluation = evaluate_candidate(
        index, current_points, config_, best.translation, best.yaw);
    if (!best.evaluation.valid) {
        result.status = "refined_match_has_too_few_correspondences";
        result.total_compute_ms = elapsed_ms(match_start, std::chrono::steady_clock::now());
        return result;
    }

    result.valid = true;
    result.position_offset_m = best.translation;
    result.yaw_offset_rad = wrap_angle(best.yaw);
    result.score_m = best.evaluation.score;
    result.rmse_m = best.evaluation.rmse;
    result.inlier_ratio = best.evaluation.inlier_ratio;
    result.matched_points = best.evaluation.matched;
    result.ambiguity_margin = std::isfinite(second_distinct_score)
                                  ? std::max(0.0,
                                             (second_distinct_score - coarse_best_score) /
                                                 std::max(coarse_best_score, 1e-6))
                                  : 1.0;

    const double xy_probe = std::max(0.5 * config_.fine_translation_step_m, 0.003);
    const double yaw_probe = std::max(0.5 * config_.fine_yaw_step_rad, 0.003);
    const auto probe_score = [&](double dx, double dy, double dyaw) {
        return evaluate_candidate(index,
                                  current_points,
                                  config_,
                                  {best.translation.x + dx, best.translation.y + dy},
                                  wrap_angle(best.yaw + dyaw))
            .score;
    };
    const auto relative_cost_rise = [&](double minus_score, double plus_score) {
        if (!std::isfinite(minus_score) || !std::isfinite(plus_score)) return 1.0;
        return clamp_value(
            (0.5 * (minus_score + plus_score) - best.evaluation.score) /
                std::max(0.25 * config_.maximum_rmse_m, 1e-6),
            0.0,
            1.0);
    };
    result.observability_x = relative_cost_rise(
        probe_score(-xy_probe, 0.0, 0.0), probe_score(xy_probe, 0.0, 0.0));
    result.observability_y = relative_cost_rise(
        probe_score(0.0, -xy_probe, 0.0), probe_score(0.0, xy_probe, 0.0));
    result.observability_yaw = relative_cost_rise(
        probe_score(0.0, 0.0, -yaw_probe), probe_score(0.0, 0.0, yaw_probe));
    result.evaluated_candidates += 6U;

    const double base_sigma_xy = clamp_value(
        result.rmse_m / std::sqrt(static_cast<double>(std::max(result.matched_points, 1))),
        0.002,
        0.08);
    double mean_radius = 0.0;
    for (const Vec2& point : current_points) mean_radius += std::hypot(point.x, point.y);
    mean_radius /= static_cast<double>(current_points.size());
    const double x_observability_scale = 1.0 / std::sqrt(std::max(result.observability_x, 0.08));
    const double y_observability_scale = 1.0 / std::sqrt(std::max(result.observability_y, 0.08));
    const double yaw_observability_scale = 1.0 / std::sqrt(std::max(result.observability_yaw, 0.08));
    const double sigma_x = clamp_value(base_sigma_xy * x_observability_scale, 0.002, 0.12);
    const double sigma_y = clamp_value(base_sigma_xy * y_observability_scale, 0.002, 0.12);
    const double sigma_yaw = clamp_value(
        base_sigma_xy / std::max(mean_radius, 0.20) * yaw_observability_scale,
        0.003,
        0.25);
    result.covariance_x_m2 = sigma_x * sigma_x;
    result.covariance_y_m2 = sigma_y * sigma_y;
    result.covariance_yaw_rad2 = sigma_yaw * sigma_yaw;

    const double rmse_quality = clamp_value(1.0 - result.rmse_m /
                                                      std::max(config_.maximum_rmse_m, 1e-6),
                                             0.0,
                                             1.0);
    const double inlier_quality = clamp_value(
        (result.inlier_ratio - config_.minimum_inlier_ratio) /
            std::max(1.0 - config_.minimum_inlier_ratio, 1e-6),
        0.0,
        1.0);
    const double ambiguity_quality = clamp_value(
        result.ambiguity_margin / std::max(config_.minimum_ambiguity_margin, 1e-6),
        0.0,
        1.0);
    result.confidence = 0.45 * rmse_quality +
                        0.35 * inlier_quality +
                        0.20 * ambiguity_quality;

    const double start_translation = std::hypot(result.position_offset_m.x,
                                                result.position_offset_m.y);
    const bool geometry_ok = result.rmse_m <= config_.maximum_rmse_m &&
                             result.inlier_ratio >= config_.minimum_inlier_ratio &&
                             result.ambiguity_margin >= config_.minimum_ambiguity_margin;
    const bool start_ok = start_translation <= config_.maximum_start_translation_m &&
                          std::abs(result.yaw_offset_rad) <= config_.maximum_start_yaw_rad;
    result.accepted = geometry_ok && start_ok;
    if (!geometry_ok) result.status = "match_quality_rejected";
    else if (!start_ok) result.status = "start_pose_outside_tolerance";
    else result.status = "start_pose_accepted";
    result.total_compute_ms = elapsed_ms(match_start, std::chrono::steady_clock::now());
    return result;
}

InitialLidarConfirmationResult confirm_initial_lidar_matches(
    const std::vector<InitialLidarMatchResult>& matches,
    const InitialLidarConfirmationConfig& config) {
    InitialLidarConfirmationResult confirmation;
    confirmation.supplied_matches = static_cast<int>(matches.size());
    const int required = std::max(config.required_matches, 1);
    if (confirmation.supplied_matches < required) {
        confirmation.status = "insufficient_confirmation_matches";
        return confirmation;
    }

    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_yaw_sin = 0.0;
    double sum_yaw_cos = 0.0;
    for (const InitialLidarMatchResult& match : matches) {
        if (!match.valid) continue;
        ++confirmation.valid_matches;
        if (match.accepted) ++confirmation.accepted_matches;
        sum_x += match.position_offset_m.x;
        sum_y += match.position_offset_m.y;
        sum_yaw_sin += std::sin(match.yaw_offset_rad);
        sum_yaw_cos += std::cos(match.yaw_offset_rad);
    }
    if (confirmation.valid_matches < required ||
        confirmation.valid_matches != confirmation.supplied_matches) {
        confirmation.status = "one_or_more_confirmation_matches_invalid";
        return confirmation;
    }

    const double count = static_cast<double>(confirmation.valid_matches);
    InitialLidarMatchResult& fused = confirmation.fused_match;
    fused.valid = true;
    fused.position_offset_m = {sum_x / count, sum_y / count};
    fused.yaw_offset_rad = std::atan2(sum_yaw_sin, sum_yaw_cos);
    fused.ambiguity_margin = std::numeric_limits<double>::infinity();
    fused.confidence = std::numeric_limits<double>::infinity();
    fused.observability_x = std::numeric_limits<double>::infinity();
    fused.observability_y = std::numeric_limits<double>::infinity();
    fused.observability_yaw = std::numeric_limits<double>::infinity();

    double between_x_variance = 0.0;
    double between_y_variance = 0.0;
    double between_yaw_variance = 0.0;
    for (const InitialLidarMatchResult& match : matches) {
        const double dx = match.position_offset_m.x - fused.position_offset_m.x;
        const double dy = match.position_offset_m.y - fused.position_offset_m.y;
        const double dyaw = wrap_angle(match.yaw_offset_rad - fused.yaw_offset_rad);
        confirmation.maximum_position_spread_m = std::max(
            confirmation.maximum_position_spread_m, std::hypot(dx, dy));
        confirmation.maximum_yaw_spread_rad = std::max(
            confirmation.maximum_yaw_spread_rad, std::abs(dyaw));
        between_x_variance += dx * dx;
        between_y_variance += dy * dy;
        between_yaw_variance += dyaw * dyaw;

        fused.score_m += match.score_m;
        fused.rmse_m += match.rmse_m;
        fused.inlier_ratio += match.inlier_ratio;
        fused.ambiguity_margin = std::min(fused.ambiguity_margin, match.ambiguity_margin);
        fused.confidence = std::min(fused.confidence, match.confidence);
        fused.observability_x = std::min(fused.observability_x, match.observability_x);
        fused.observability_y = std::min(fused.observability_y, match.observability_y);
        fused.observability_yaw = std::min(fused.observability_yaw, match.observability_yaw);
        fused.covariance_x_m2 += match.covariance_x_m2;
        fused.covariance_y_m2 += match.covariance_y_m2;
        fused.covariance_yaw_rad2 += match.covariance_yaw_rad2;
        fused.coarse_compute_ms += match.coarse_compute_ms;
        fused.fine_compute_ms += match.fine_compute_ms;
        fused.total_compute_ms += match.total_compute_ms;
        fused.evaluated_candidates += match.evaluated_candidates;
        fused.timed_out = fused.timed_out || match.timed_out;
        fused.reference_points = std::max(fused.reference_points, match.reference_points);
        fused.current_points += match.current_points;
        fused.matched_points += match.matched_points;
    }
    fused.score_m /= count;
    fused.rmse_m /= count;
    fused.inlier_ratio /= count;
    fused.current_points = static_cast<int>(std::lround(fused.current_points / count));
    fused.matched_points = static_cast<int>(std::lround(fused.matched_points / count));
    const double variance_denominator = count > 1.0 ? count - 1.0 : 1.0;
    fused.covariance_x_m2 = fused.covariance_x_m2 / count +
                            between_x_variance / variance_denominator;
    fused.covariance_y_m2 = fused.covariance_y_m2 / count +
                            between_y_variance / variance_denominator;
    fused.covariance_yaw_rad2 = fused.covariance_yaw_rad2 / count +
                                between_yaw_variance / variance_denominator;

    confirmation.valid = true;
    const bool all_individually_accepted =
        confirmation.accepted_matches == confirmation.supplied_matches;
    const bool consistent =
        confirmation.maximum_position_spread_m <= config.maximum_position_spread_m &&
        confirmation.maximum_yaw_spread_rad <= config.maximum_yaw_spread_rad;
    confirmation.accepted = all_individually_accepted && consistent;
    fused.accepted = confirmation.accepted;
    if (!all_individually_accepted) {
        confirmation.status = "one_or_more_confirmation_matches_rejected";
    } else if (!consistent) {
        confirmation.status = "confirmation_matches_inconsistent";
    } else {
        confirmation.status = "start_pose_confirmed";
    }
    fused.status = confirmation.status;
    return confirmation;
}

std::vector<Vec2> voxelize_initial_lidar_cloud(const std::vector<Vec2>& points,
                                               double voxel_size_m,
                                               int minimum_observations) {
    const double resolution = std::max(voxel_size_m, 1e-4);
    std::unordered_map<std::uint64_t, AccumulatedPoint> voxels;
    for (const Vec2& point : points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y)) continue;
        const int x = static_cast<int>(std::floor(point.x / resolution));
        const int y = static_cast<int>(std::floor(point.y / resolution));
        AccumulatedPoint& cell = voxels[cell_key(x, y)];
        cell.sum.x += point.x;
        cell.sum.y += point.y;
        ++cell.count;
    }
    std::vector<Vec2> output;
    output.reserve(voxels.size());
    for (const auto& entry : voxels) {
        const AccumulatedPoint& cell = entry.second;
        if (cell.count < std::max(minimum_observations, 1)) continue;
        output.push_back({cell.sum.x / static_cast<double>(cell.count),
                          cell.sum.y / static_cast<double>(cell.count)});
    }
    std::sort(output.begin(), output.end(), [](const Vec2& lhs, const Vec2& rhs) {
        return lhs.x < rhs.x || (lhs.x == rhs.x && lhs.y < rhs.y);
    });
    return output;
}

std::vector<Vec2> transform_initial_lidar_cloud(const std::vector<Vec2>& points,
                                                const Vec2& translation,
                                                double yaw) {
    std::vector<Vec2> transformed;
    transformed.reserve(points.size());
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    for (const Vec2& point : points) {
        transformed.push_back({translation.x + c * point.x - s * point.y,
                               translation.y + s * point.x + c * point.y});
    }
    return transformed;
}

std::string initial_lidar_reference_hash(
    const InitialLidarReferenceCloud& reference) {
    std::uint64_t hash = 14695981039346656037ULL;
    fnv1a_update(&hash, reference_hash_material(reference));
    std::ostringstream encoded;
    encoded << std::hex << std::setfill('0') << std::setw(16) << hash;
    return encoded.str();
}

bool write_initial_lidar_reference(const std::string& path,
                                   const InitialLidarReferenceCloud& reference,
                                   std::string* error) {
    std::ofstream out(path, std::ios::out | std::ios::trunc);
    if (!out.is_open()) {
        if (error != nullptr) *error = "cannot open reference output: " + path;
        return false;
    }
    const std::string content_hash = initial_lidar_reference_hash(reference);
    out << std::setprecision(17)
        << "#schema=" << reference.metadata.schema << '\n'
        << "#created_utc=" << reference.metadata.created_utc << '\n'
        << "#lidar_serial=" << reference.metadata.lidar_serial << '\n'
        << "#lidar_firmware=" << reference.metadata.lidar_firmware << '\n'
        << "#lidar_hardware=" << reference.metadata.lidar_hardware << '\n'
        << "#lidar_x_offset_m=" << reference.metadata.lidar_x_offset_m << '\n'
        << "#lidar_y_offset_m=" << reference.metadata.lidar_y_offset_m << '\n'
        << "#lidar_yaw_offset_rad=" << reference.metadata.lidar_yaw_offset_rad << '\n'
        << "#lidar_flip_left_right=" << (reference.metadata.lidar_flip_left_right ? 1 : 0) << '\n'
        << "#min_range_m=" << reference.metadata.min_range_m << '\n'
        << "#max_range_m=" << reference.metadata.max_range_m << '\n'
        << "#voxel_size_m=" << reference.metadata.voxel_size_m << '\n'
        << "#body_length_m=" << reference.metadata.body_length_m << '\n'
        << "#body_width_m=" << reference.metadata.body_width_m << '\n'
        << "#source_scan_count=" << reference.metadata.source_scan_count << '\n'
        << "#source_raw_point_count=" << reference.metadata.source_raw_point_count << '\n'
        << "#stability_valid=" << (reference.metadata.stability_valid ? 1 : 0) << '\n'
        << "#stability_translation_m=" << reference.metadata.stability_translation_m << '\n'
        << "#stability_yaw_rad=" << reference.metadata.stability_yaw_rad << '\n'
        << "#stability_rmse_m=" << reference.metadata.stability_rmse_m << '\n'
        << "#stability_inlier_ratio=" << reference.metadata.stability_inlier_ratio << '\n'
        << "#raw_scan_log_path=" << reference.metadata.raw_scan_log_path << '\n'
        << "#content_hash_fnv1a64=" << content_hash << '\n'
        << "x_m,y_m\n";
    for (const Vec2& point : reference.points) out << point.x << ',' << point.y << '\n';
    if (!out.good()) {
        if (error != nullptr) *error = "failed while writing reference: " + path;
        return false;
    }
    if (error != nullptr) error->clear();
    return true;
}

bool read_initial_lidar_reference(const std::string& path,
                                  InitialLidarReferenceCloud* reference,
                                  std::string* error) {
    if (reference == nullptr) {
        if (error != nullptr) *error = "null reference output";
        return false;
    }
    std::ifstream in(path);
    if (!in.is_open()) {
        if (error != nullptr) *error = "cannot open reference: " + path;
        return false;
    }
    InitialLidarReferenceCloud parsed;
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        if (line[0] == '#') {
            parse_metadata_line(line, &parsed.metadata);
            continue;
        }
        if (line == "x_m,y_m") continue;
        std::istringstream row(line);
        std::string x_text;
        std::string y_text;
        if (!std::getline(row, x_text, ',') || !std::getline(row, y_text)) continue;
        try {
            const Vec2 point{std::stod(x_text), std::stod(y_text)};
            if (std::isfinite(point.x) && std::isfinite(point.y)) parsed.points.push_back(point);
        } catch (...) {
        }
    }
    if (parsed.metadata.schema != "thesis_initial_lidar_reference_v1" &&
        parsed.metadata.schema != "thesis_initial_lidar_reference_v2") {
        if (error != nullptr) *error = "unsupported reference schema: " + parsed.metadata.schema;
        return false;
    }
    if (parsed.points.empty()) {
        if (error != nullptr) *error = "reference contains no valid points";
        return false;
    }
    parsed.computed_content_hash = initial_lidar_reference_hash(parsed);
    if (parsed.metadata.schema == "thesis_initial_lidar_reference_v2") {
        if (parsed.metadata.content_hash_fnv1a64.empty()) {
            if (error != nullptr) *error = "v2 reference is missing its integrity hash";
            return false;
        }
        if (parsed.metadata.content_hash_fnv1a64 != parsed.computed_content_hash) {
            if (error != nullptr) {
                *error = "reference integrity check failed: expected " +
                         parsed.metadata.content_hash_fnv1a64 + " computed " +
                         parsed.computed_content_hash;
            }
            return false;
        }
        parsed.integrity_verified = true;
    }
    *reference = std::move(parsed);
    if (error != nullptr) error->clear();
    return true;
}

bool write_initial_lidar_cloud_comparison_csv(
    const std::string& path,
    const std::vector<Vec2>& reference,
    const std::vector<Vec2>& current,
    const std::vector<Vec2>& aligned,
    std::string* error) {
    std::ofstream out(path, std::ios::out | std::ios::trunc);
    if (!out.is_open()) {
        if (error != nullptr) *error = "cannot open point-cloud CSV: " + path;
        return false;
    }
    out << std::setprecision(12) << "cloud,x_m,y_m\n";
    for (const Vec2& point : reference) out << "reference," << point.x << ',' << point.y << '\n';
    for (const Vec2& point : current) out << "current_unaligned," << point.x << ',' << point.y << '\n';
    for (const Vec2& point : aligned) out << "current_aligned," << point.x << ',' << point.y << '\n';
    if (!out.good()) {
        if (error != nullptr) *error = "failed while writing point-cloud CSV: " + path;
        return false;
    }
    if (error != nullptr) error->clear();
    return true;
}

bool write_initial_lidar_cloud_comparison_ply(
    const std::string& path,
    const std::vector<Vec2>& reference,
    const std::vector<Vec2>& current,
    const std::vector<Vec2>& aligned,
    std::string* error) {
    std::ofstream out(path, std::ios::out | std::ios::trunc);
    if (!out.is_open()) {
        if (error != nullptr) *error = "cannot open point-cloud PLY: " + path;
        return false;
    }
    out << "ply\nformat ascii 1.0\n"
        << "comment red=reference blue=current_unaligned green=current_aligned\n"
        << "element vertex " << (reference.size() + current.size() + aligned.size()) << "\n"
        << "property float x\nproperty float y\nproperty float z\n"
        << "property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n"
        << std::setprecision(9);
    for (const Vec2& point : reference) out << point.x << ' ' << point.y << " 0 255 60 60\n";
    for (const Vec2& point : current) out << point.x << ' ' << point.y << " 0 70 120 255\n";
    for (const Vec2& point : aligned) out << point.x << ' ' << point.y << " 0 70 255 100\n";
    if (!out.good()) {
        if (error != nullptr) *error = "failed while writing point-cloud PLY: " + path;
        return false;
    }
    if (error != nullptr) error->clear();
    return true;
}

}  // namespace thesis_sim::mvc::model
