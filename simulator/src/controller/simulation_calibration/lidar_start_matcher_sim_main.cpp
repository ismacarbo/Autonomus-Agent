#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "mvc/model/navigation/initial_lidar_matcher.h"
#include "mvc/model/navigation/vehicle_dynamics.h"
#include "mvc/model/perception/perception_model.h"
#include "mvc/model/world/scenario_model.h"

namespace {

constexpr double kPi = 3.14159265358979323846;

using thesis_sim::EnvironmentMode;
using thesis_sim::GateBehaviorMode;
using thesis_sim::LidarHit;
using thesis_sim::RangeSensorProfile;
using thesis_sim::StructuredMapPreset;
using thesis_sim::UnstructuredMapPreset;
using thesis_sim::Vec2;
using thesis_sim::VehicleModelKind;
using thesis_sim::WorldMap;
using thesis_sim::mvc::model::InitialLidarMatchResult;
using thesis_sim::mvc::model::InitialLidarMatcher;
using thesis_sim::mvc::model::InitialLidarMatcherConfig;
using thesis_sim::mvc::model::InitialLidarConfirmationConfig;
using thesis_sim::mvc::model::InitialLidarConfirmationResult;
using thesis_sim::mvc::model::InitialLidarReferenceCloud;
using thesis_sim::mvc::model::InitialLidarReferenceMetadata;
using thesis_sim::mvc::model::SimulatedLidarConfig;
using thesis_sim::mvc::model::acquire_simulated_lidar_scan;
using thesis_sim::mvc::model::fit_hardware_structured_world;
using thesis_sim::mvc::model::fit_hardware_unstructured_world;
using thesis_sim::mvc::model::hardware_mixed_world_from_preset;
using thesis_sim::mvc::model::make_world_from_mode;
using thesis_sim::mvc::model::transform_initial_lidar_cloud;
using thesis_sim::mvc::model::voxelize_initial_lidar_cloud;
using thesis_sim::mvc::model::write_initial_lidar_cloud_comparison_csv;
using thesis_sim::mvc::model::write_initial_lidar_cloud_comparison_ply;
using thesis_sim::mvc::model::write_initial_lidar_reference;
using thesis_sim::mvc::model::confirm_initial_lidar_matches;

struct Pose2D {
    Vec2 position{};
    double yaw = 0.0;
};

struct AppOptions {
    EnvironmentMode environment_mode = EnvironmentMode::MixedRoadGates;
    StructuredMapPreset structured_preset = StructuredMapPreset::ValidationRoad;
    UnstructuredMapPreset unstructured_preset = UnstructuredMapPreset::HardwareLab;
    int mixed_preset = 3;
    VehicleModelKind vehicle_model = VehicleModelKind::CarLikeBicycle;
    Vec2 ground_truth_offset{0.035, -0.025};
    double ground_truth_yaw_offset_rad = 3.5 * kPi / 180.0;
    int scan_count = 8;
    int runs = 1;
    bool random_offsets = false;
    double random_translation_limit_m = 0.14;
    double random_yaw_limit_rad = 15.0 * kPi / 180.0;
    std::uint32_t seed = 20260730U;
    bool calibrated = true;
    double range_noise_std_m = 0.005;
    double dropout_probability = 0.006;
    std::string output_directory = "reports";
    std::string reference_output_path;
    InitialLidarReferenceMetadata sensor{};
    InitialLidarMatcherConfig matcher{};
    InitialLidarConfirmationConfig confirmation{};
    bool show_help = false;
};

struct MatrixRunResult {
    int run_index = 0;
    std::uint32_t seed = 0;
    Vec2 ground_truth_offset{};
    double ground_truth_yaw_offset_rad = 0.0;
    bool expected_acceptance = false;
    std::vector<InitialLidarMatchResult> matches;
    InitialLidarConfirmationResult confirmation{};
    Vec2 error_position{};
    double error_yaw_rad = 0.0;
    bool estimate_accurate = false;
    std::string classification;
    std::vector<Vec2> current_points;
};

double parse_double(const std::string& text, const std::string& option) {
    std::size_t used = 0;
    const double value = std::stod(text, &used);
    if (used != text.size() || !std::isfinite(value)) {
        throw std::invalid_argument("invalid value for " + option);
    }
    return value;
}

int parse_int(const std::string& text, const std::string& option) {
    std::size_t used = 0;
    const int value = std::stoi(text, &used);
    if (used != text.size()) throw std::invalid_argument("invalid value for " + option);
    return value;
}

std::uint32_t parse_seed(const std::string& text) {
    std::size_t used = 0;
    const unsigned long value = std::stoul(text, &used);
    if (used != text.size()) throw std::invalid_argument("invalid --seed");
    return static_cast<std::uint32_t>(value);
}

void print_usage(const char* executable) {
    std::cout
        << "Usage: " << executable << " [options]\n\n"
        << "Runs the same initial LiDAR matcher used by hardware against a known simulated offset.\n\n"
        << "Scenario:\n"
        << "  --scenario MODE           structured | unstructured | mixed (default mixed)\n"
        << "  --structured-map NAME     validation | circle\n"
        << "  --unstructured-map NAME   robot_validation | tight | slalom | lower | hardware_lab\n"
        << "  --mixed-map NAME          hardware | obstacle (default obstacle)\n"
        << "  --vehicle-model NAME      car | tank\n\n"
        << "Ground-truth start perturbation in the nominal start frame:\n"
        << "  --offset-x M              default +0.035\n"
        << "  --offset-y M              default -0.025\n"
        << "  --offset-yaw-deg DEG      default +3.5\n\n"
        << "Sensor and repeatability:\n"
        << "  --seed N                  default 20260730\n"
        << "  --scan-count N            scans per cloud, default 8\n"
        << "  --confirmations N         independent clouds per decision, default 3\n"
        << "  --runs N                 number of start decisions, default 1\n"
        << "  --random-offsets          sample positive and negative start cases\n"
        << "  --random-translation M    radial outside-case limit, default 0.14\n"
        << "  --random-yaw-deg DEG      random yaw limit, default 15\n"
        << "  --calibrated              enable noise/dropout (default)\n"
        << "  --ideal                   disable noise/dropout\n"
        << "  --range-noise M           default 0.005\n"
        << "  --dropout P               default 0.006\n"
        << "  --max-range M             default 3.0\n"
        << "  --voxel-size M            default 0.018\n\n"
        << "Matcher acceptance:\n"
        << "  --max-start-offset M      default 0.08\n"
        << "  --max-start-yaw-deg DEG   default 8\n"
        << "  --max-rmse M              default 0.045\n"
        << "  --min-inlier-ratio R      default 0.55\n"
        << "  --min-ambiguity R         default 0.025\n"
        << "  --max-confirm-spread M    default 0.025\n"
        << "  --max-confirm-yaw-deg D   default 2\n"
        << "  --matcher-timeout-ms N    per-match timeout, default 20000\n\n"
        << "Artifacts:\n"
        << "  --output-dir PATH         default reports\n"
        << "  --reference-out PATH      optional reusable simulated reference CSV\n";
}

StructuredMapPreset parse_structured_map(const std::string& value) {
    return value == "circle" || value == "circle_loop"
               ? StructuredMapPreset::CircleLoop
               : StructuredMapPreset::ValidationRoad;
}

UnstructuredMapPreset parse_unstructured_map(const std::string& value) {
    if (value == "tight") return UnstructuredMapPreset::TightCorridor;
    if (value == "slalom") return UnstructuredMapPreset::WideSlalom;
    if (value == "lower") return UnstructuredMapPreset::LowerBypass;
    if (value == "hardware" || value == "hardware_lab" || value == "lab") {
        return UnstructuredMapPreset::HardwareLab;
    }
    return UnstructuredMapPreset::RobotValidation;
}

AppOptions parse_options(int argc, char** argv) {
    AppOptions options;
    for (int i = 1; i < argc; ++i) {
        const std::string argument = argv[i];
        const auto require_value = [&](const std::string& option) -> std::string {
            if (i + 1 >= argc) throw std::invalid_argument("missing value for " + option);
            return argv[++i];
        };
        if (argument == "--scenario") {
            const std::string value = require_value(argument);
            if (value == "structured") options.environment_mode = EnvironmentMode::StructuredRoad;
            else if (value == "unstructured") options.environment_mode = EnvironmentMode::UnstructuredGates;
            else if (value == "mixed") options.environment_mode = EnvironmentMode::MixedRoadGates;
            else throw std::invalid_argument("unknown scenario: " + value);
        } else if (argument == "--structured-map") {
            options.structured_preset = parse_structured_map(require_value(argument));
        } else if (argument == "--unstructured-map") {
            options.unstructured_preset = parse_unstructured_map(require_value(argument));
        } else if (argument == "--mixed-map") {
            const std::string value = require_value(argument);
            if (value == "hardware" || value == "hardware_lab") options.mixed_preset = 1;
            else if (value == "obstacle" || value == "closed_obstacle") options.mixed_preset = 3;
            else throw std::invalid_argument("unknown mixed map: " + value);
        } else if (argument == "--vehicle-model") {
            const std::string value = require_value(argument);
            if (value == "tank" || value == "tracked") {
                options.vehicle_model = VehicleModelKind::TrackedVehicle;
                options.sensor.body_length_m = 0.165;
                options.sensor.body_width_m = 0.146;
            } else if (value == "car") {
                options.vehicle_model = VehicleModelKind::CarLikeBicycle;
                options.sensor.body_length_m = 0.25;
                options.sensor.body_width_m = 0.15;
            } else {
                throw std::invalid_argument("unknown vehicle model: " + value);
            }
        } else if (argument == "--offset-x") {
            options.ground_truth_offset.x = parse_double(require_value(argument), argument);
        } else if (argument == "--offset-y") {
            options.ground_truth_offset.y = parse_double(require_value(argument), argument);
        } else if (argument == "--offset-yaw-deg") {
            options.ground_truth_yaw_offset_rad = parse_double(require_value(argument), argument) * kPi / 180.0;
        } else if (argument == "--seed") {
            options.seed = parse_seed(require_value(argument));
        } else if (argument == "--scan-count") {
            options.scan_count = parse_int(require_value(argument), argument);
        } else if (argument == "--confirmations") {
            options.confirmation.required_matches = parse_int(require_value(argument), argument);
        } else if (argument == "--runs") {
            options.runs = parse_int(require_value(argument), argument);
        } else if (argument == "--random-offsets") {
            options.random_offsets = true;
        } else if (argument == "--random-translation") {
            options.random_translation_limit_m = parse_double(require_value(argument), argument);
        } else if (argument == "--random-yaw-deg") {
            options.random_yaw_limit_rad =
                parse_double(require_value(argument), argument) * kPi / 180.0;
        } else if (argument == "--calibrated") {
            options.calibrated = true;
        } else if (argument == "--ideal") {
            options.calibrated = false;
        } else if (argument == "--range-noise") {
            options.range_noise_std_m = parse_double(require_value(argument), argument);
        } else if (argument == "--dropout") {
            options.dropout_probability = parse_double(require_value(argument), argument);
        } else if (argument == "--max-range") {
            options.sensor.max_range_m = parse_double(require_value(argument), argument);
        } else if (argument == "--voxel-size") {
            options.sensor.voxel_size_m = parse_double(require_value(argument), argument);
        } else if (argument == "--max-start-offset") {
            options.matcher.maximum_start_translation_m = parse_double(require_value(argument), argument);
        } else if (argument == "--max-start-yaw-deg") {
            options.matcher.maximum_start_yaw_rad = parse_double(require_value(argument), argument) * kPi / 180.0;
        } else if (argument == "--max-rmse") {
            options.matcher.maximum_rmse_m = parse_double(require_value(argument), argument);
        } else if (argument == "--min-inlier-ratio") {
            options.matcher.minimum_inlier_ratio = parse_double(require_value(argument), argument);
        } else if (argument == "--min-ambiguity") {
            options.matcher.minimum_ambiguity_margin = parse_double(require_value(argument), argument);
        } else if (argument == "--max-confirm-spread") {
            options.confirmation.maximum_position_spread_m = parse_double(require_value(argument), argument);
        } else if (argument == "--max-confirm-yaw-deg") {
            options.confirmation.maximum_yaw_spread_rad =
                parse_double(require_value(argument), argument) * kPi / 180.0;
        } else if (argument == "--matcher-timeout-ms") {
            options.matcher.maximum_compute_time_ms = parse_double(require_value(argument), argument);
        } else if (argument == "--output-dir") {
            options.output_directory = require_value(argument);
        } else if (argument == "--reference-out") {
            options.reference_output_path = require_value(argument);
        } else if (argument == "--help" || argument == "-h") {
            options.show_help = true;
        } else {
            throw std::invalid_argument("unknown option: " + argument);
        }
    }
    options.scan_count = std::clamp(options.scan_count, 1, 50);
    options.confirmation.required_matches = std::clamp(
        options.confirmation.required_matches, 1, 10);
    options.runs = std::clamp(options.runs, 1, 500);
    options.random_translation_limit_m = std::max(options.random_translation_limit_m, 0.0);
    options.random_yaw_limit_rad = std::max(options.random_yaw_limit_rad, 0.0);
    options.dropout_probability = std::clamp(options.dropout_probability, 0.0, 1.0);
    options.range_noise_std_m = std::max(options.range_noise_std_m, 0.0);
    if (!(options.sensor.max_range_m > options.sensor.min_range_m) ||
        !(options.sensor.voxel_size_m > 0.0)) {
        throw std::invalid_argument("invalid sensor range or voxel size");
    }
    return options;
}

double wrap_angle(double angle) {
    while (angle > kPi) angle -= 2.0 * kPi;
    while (angle < -kPi) angle += 2.0 * kPi;
    return angle;
}

WorldMap make_validation_world(const AppOptions& options) {
    if (options.environment_mode == EnvironmentMode::MixedRoadGates) {
        return hardware_mixed_world_from_preset(options.mixed_preset);
    }
    WorldMap world = make_world_from_mode(
        options.environment_mode,
        options.unstructured_preset,
        options.structured_preset,
        GateBehaviorMode::Static,
        options.seed,
        options.mixed_preset);
    if (options.environment_mode == EnvironmentMode::StructuredRoad) {
        return fit_hardware_structured_world(std::move(world), options.vehicle_model);
    }
    return fit_hardware_unstructured_world(std::move(world));
}

Pose2D compose_start_offset(const Pose2D& nominal,
                            const Vec2& local_offset,
                            double yaw_offset) {
    const double c = std::cos(nominal.yaw);
    const double s = std::sin(nominal.yaw);
    return {
        {nominal.position.x + c * local_offset.x - s * local_offset.y,
         nominal.position.y + s * local_offset.x + c * local_offset.y},
        wrap_angle(nominal.yaw + yaw_offset),
    };
}

Vec2 lidar_origin_world(const Pose2D& base,
                        const InitialLidarReferenceMetadata& sensor) {
    const double c = std::cos(base.yaw);
    const double s = std::sin(base.yaw);
    return {
        base.position.x + c * sensor.lidar_x_offset_m - s * sensor.lidar_y_offset_m,
        base.position.y + s * sensor.lidar_x_offset_m + c * sensor.lidar_y_offset_m,
    };
}

std::vector<Vec2> hits_to_base_points(const std::vector<LidarHit>& hits,
                                      const Pose2D& base,
                                      const InitialLidarReferenceMetadata& sensor) {
    std::vector<Vec2> points;
    points.reserve(hits.size());
    const double c = std::cos(base.yaw);
    const double s = std::sin(base.yaw);
    for (const LidarHit& hit : hits) {
        if (!hit.hit || !std::isfinite(hit.distance) ||
            hit.distance < sensor.min_range_m || hit.distance > sensor.max_range_m) {
            continue;
        }
        const double dx = hit.point.x - base.position.x;
        const double dy = hit.point.y - base.position.y;
        points.push_back({c * dx + s * dy, -s * dx + c * dy});
    }
    return points;
}

std::vector<Vec2> acquire_stationary_cloud(WorldMap* world,
                                           const Pose2D& base,
                                           const AppOptions& options,
                                           std::mt19937* random_engine) {
    SimulatedLidarConfig lidar;
    lidar.enabled = true;
    lidar.profile = RangeSensorProfile::RplidarA1;
    lidar.fallback_range_m = options.sensor.max_range_m;
    lidar.calibrated = options.calibrated;
    lidar.range_noise_std_m = options.range_noise_std_m;
    lidar.dropout_probability = options.dropout_probability;

    std::vector<Vec2> accumulated;
    for (int scan_index = 0; scan_index < options.scan_count; ++scan_index) {
        const Vec2 origin = lidar_origin_world(base, options.sensor);
        const std::vector<LidarHit> hits = acquire_simulated_lidar_scan(
            world,
            origin,
            base.yaw + options.sensor.lidar_yaw_offset_rad,
            0.0,
            0.0,
            lidar,
            random_engine);
        const std::vector<Vec2> points = hits_to_base_points(hits, base, options.sensor);
        accumulated.insert(accumulated.end(), points.begin(), points.end());
    }
    return voxelize_initial_lidar_cloud(
        accumulated,
        options.sensor.voxel_size_m,
        options.scan_count >= 3 ? 2 : 1);
}

std::string timestamp_token() {
    const auto now = std::chrono::system_clock::now();
    const auto seconds = std::chrono::system_clock::to_time_t(now);
    const auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(
                                  now.time_since_epoch()) %
                              1000;
    std::tm local{};
    localtime_r(&seconds, &local);
    std::ostringstream token;
    token << std::put_time(&local, "%Y%m%d_%H%M%S_")
          << std::setfill('0') << std::setw(3) << milliseconds.count();
    return token.str();
}

std::string utc_timestamp_iso() {
    const auto now = std::chrono::system_clock::now();
    const std::time_t seconds = std::chrono::system_clock::to_time_t(now);
    std::tm utc{};
    gmtime_r(&seconds, &utc);
    std::ostringstream timestamp;
    timestamp << std::put_time(&utc, "%Y-%m-%dT%H:%M:%SZ");
    return timestamp.str();
}

std::string mode_name(EnvironmentMode mode) {
    if (mode == EnvironmentMode::StructuredRoad) return "structured";
    if (mode == EnvironmentMode::UnstructuredGates) return "unstructured";
    return "mixed";
}

double percentile(std::vector<double> values, double quantile) {
    if (values.empty()) return 0.0;
    std::sort(values.begin(), values.end());
    const double position = std::clamp(quantile, 0.0, 1.0) *
                            static_cast<double>(values.size() - 1U);
    const std::size_t lower = static_cast<std::size_t>(std::floor(position));
    const std::size_t upper = static_cast<std::size_t>(std::ceil(position));
    const double alpha = position - static_cast<double>(lower);
    return values[lower] * (1.0 - alpha) + values[upper] * alpha;
}

bool write_matrix_csv(const std::string& path,
                      const std::vector<MatrixRunResult>& runs) {
    std::ofstream out(path, std::ios::out | std::ios::trunc);
    if (!out.is_open()) return false;
    out << std::setprecision(12)
        << "run,seed,ground_truth_x_m,ground_truth_y_m,ground_truth_yaw_rad,"
           "expected_acceptance,valid,accepted,classification,estimated_x_m,"
           "estimated_y_m,estimated_yaw_rad,error_translation_m,error_yaw_rad,"
           "estimate_accurate,confirmation_position_spread_m,"
           "confirmation_yaw_spread_rad,rmse_m,inlier_ratio,ambiguity_margin,"
           "confidence,observability_x,observability_y,observability_yaw,"
           "covariance_x_m2,covariance_y_m2,covariance_yaw_rad2,compute_ms,"
           "evaluated_candidates,status\n";
    for (const MatrixRunResult& run : runs) {
        const InitialLidarMatchResult& result = run.confirmation.fused_match;
        out << run.run_index << ',' << run.seed << ','
            << run.ground_truth_offset.x << ',' << run.ground_truth_offset.y << ','
            << run.ground_truth_yaw_offset_rad << ','
            << (run.expected_acceptance ? 1 : 0) << ','
            << (run.confirmation.valid ? 1 : 0) << ','
            << (run.confirmation.accepted ? 1 : 0) << ','
            << run.classification << ','
            << result.position_offset_m.x << ',' << result.position_offset_m.y << ','
            << result.yaw_offset_rad << ','
            << std::hypot(run.error_position.x, run.error_position.y) << ','
            << run.error_yaw_rad << ',' << (run.estimate_accurate ? 1 : 0) << ','
            << run.confirmation.maximum_position_spread_m << ','
            << run.confirmation.maximum_yaw_spread_rad << ','
            << result.rmse_m << ',' << result.inlier_ratio << ','
            << result.ambiguity_margin << ',' << result.confidence << ','
            << result.observability_x << ',' << result.observability_y << ','
            << result.observability_yaw << ',' << result.covariance_x_m2 << ','
            << result.covariance_y_m2 << ',' << result.covariance_yaw_rad2 << ','
            << result.total_compute_ms << ',' << result.evaluated_candidates << ','
            << run.confirmation.status << '\n';
    }
    return out.good();
}

bool write_report(const std::string& path,
                  const AppOptions& options,
                  const std::vector<MatrixRunResult>& runs,
                  const std::string& reference_path,
                  const std::string& cloud_path,
                  const std::string& ply_path,
                  const std::string& matrix_csv_path) {
    int true_positive = 0;
    int true_negative = 0;
    int false_positive = 0;
    int false_negative = 0;
    int invalid = 0;
    int inaccurate_accepted = 0;
    std::vector<double> position_errors;
    std::vector<double> yaw_errors;
    std::vector<double> compute_times;
    for (const MatrixRunResult& run : runs) {
        if (run.classification == "true_positive") ++true_positive;
        else if (run.classification == "true_negative") ++true_negative;
        else if (run.classification == "false_positive") ++false_positive;
        else if (run.classification == "false_negative") ++false_negative;
        else ++invalid;
        if (run.confirmation.accepted && !run.estimate_accurate) ++inaccurate_accepted;
        if (run.confirmation.valid) {
            position_errors.push_back(std::hypot(run.error_position.x, run.error_position.y));
            yaw_errors.push_back(std::abs(run.error_yaw_rad));
            compute_times.push_back(run.confirmation.fused_match.total_compute_ms);
        }
    }
    const auto mean = [](const std::vector<double>& values) {
        if (values.empty()) return 0.0;
        double sum = 0.0;
        for (double value : values) sum += value;
        return sum / static_cast<double>(values.size());
    };
    std::ofstream out(path, std::ios::out | std::ios::trunc);
    if (!out.is_open()) return false;
    out << std::boolalpha << std::setprecision(12)
        << "{\n"
        << "  \"schema\": \"thesis_sim_initial_lidar_match_matrix_v2\",\n"
        << "  \"scenario\": \"" << mode_name(options.environment_mode) << "\",\n"
        << "  \"base_seed\": " << options.seed << ",\n"
        << "  \"calibrated\": " << options.calibrated << ",\n"
        << "  \"scan_count\": " << options.scan_count << ",\n"
        << "  \"confirmations_per_run\": " << options.confirmation.required_matches << ",\n"
        << "  \"run_count\": " << runs.size() << ",\n"
        << "  \"random_offsets\": " << options.random_offsets << ",\n"
        << "  \"range_noise_std_m\": " << options.range_noise_std_m << ",\n"
        << "  \"dropout_probability\": " << options.dropout_probability << ",\n"
        << "  \"true_positive\": " << true_positive << ",\n"
        << "  \"true_negative\": " << true_negative << ",\n"
        << "  \"false_positive\": " << false_positive << ",\n"
        << "  \"false_negative\": " << false_negative << ",\n"
        << "  \"invalid\": " << invalid << ",\n"
        << "  \"inaccurate_accepted\": " << inaccurate_accepted << ",\n"
        << "  \"mean_position_error_m\": " << mean(position_errors) << ",\n"
        << "  \"p95_position_error_m\": " << percentile(position_errors, 0.95) << ",\n"
        << "  \"mean_abs_yaw_error_rad\": " << mean(yaw_errors) << ",\n"
        << "  \"p95_abs_yaw_error_rad\": " << percentile(yaw_errors, 0.95) << ",\n"
        << "  \"mean_compute_ms\": " << mean(compute_times) << ",\n"
        << "  \"p95_compute_ms\": " << percentile(compute_times, 0.95) << ",\n"
        << "  \"reference_csv\": \"" << reference_path << "\",\n"
        << "  \"cloud_csv\": \"" << cloud_path << "\",\n"
        << "  \"comparison_ply\": \"" << ply_path << "\",\n"
        << "  \"matrix_csv\": \"" << matrix_csv_path << "\"\n"
        << "}\n";
    return out.good();
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const AppOptions options = parse_options(argc, argv);
        if (options.show_help) {
            print_usage(argv[0]);
            return 0;
        }

        WorldMap world = make_validation_world(options);
        const Pose2D nominal{world.start(), world.start_heading()};
        std::mt19937 reference_random(options.seed);
        const std::vector<Vec2> reference_points = acquire_stationary_cloud(
            &world, nominal, options, &reference_random);
        if (reference_points.size() <
            static_cast<std::size_t>(options.matcher.minimum_points)) {
            throw std::runtime_error("simulated reference has too few usable points");
        }

        std::mt19937 offset_random(options.seed ^ 0xA511E9B3U);
        std::uniform_real_distribution<double> unit_distribution(0.0, 1.0);
        const InitialLidarMatcher matcher(options.matcher);
        std::vector<MatrixRunResult> runs;
        runs.reserve(static_cast<std::size_t>(options.runs));
        for (int run_index = 0; run_index < options.runs; ++run_index) {
            MatrixRunResult run;
            run.run_index = run_index;
            run.seed = options.seed +
                       0x9E3779B9U * static_cast<std::uint32_t>(run_index + 1);
            run.ground_truth_offset = options.ground_truth_offset;
            run.ground_truth_yaw_offset_rad = options.ground_truth_yaw_offset_rad;
            if (options.random_offsets) {
                const bool positive_case = (run_index % 2) == 0;
                const double angle = 2.0 * kPi * unit_distribution(offset_random);
                const double inside_radius =
                    0.85 * options.matcher.maximum_start_translation_m *
                    std::sqrt(unit_distribution(offset_random));
                double radius = inside_radius;
                double yaw_limit = 0.85 * options.matcher.maximum_start_yaw_rad;
                run.ground_truth_yaw_offset_rad =
                    (2.0 * unit_distribution(offset_random) - 1.0) * yaw_limit;
                if (!positive_case && (run_index % 4) == 1) {
                    const double outside_minimum =
                        1.15 * options.matcher.maximum_start_translation_m;
                    const double outside_limit = std::max(
                        options.random_translation_limit_m, outside_minimum);
                    radius = outside_minimum + unit_distribution(offset_random) *
                                                   (outside_limit - outside_minimum);
                } else if (!positive_case) {
                    const double outside_minimum =
                        1.15 * options.matcher.maximum_start_yaw_rad;
                    const double outside_limit = std::max(
                        options.random_yaw_limit_rad, outside_minimum);
                    const double magnitude = outside_minimum +
                                             unit_distribution(offset_random) *
                                                 (outside_limit - outside_minimum);
                    run.ground_truth_yaw_offset_rad =
                        unit_distribution(offset_random) < 0.5 ? -magnitude : magnitude;
                }
                run.ground_truth_offset = {
                    radius * std::cos(angle), radius * std::sin(angle)};
            }
            run.expected_acceptance =
                std::hypot(run.ground_truth_offset.x, run.ground_truth_offset.y) <=
                    options.matcher.maximum_start_translation_m &&
                std::abs(run.ground_truth_yaw_offset_rad) <=
                    options.matcher.maximum_start_yaw_rad;
            const Pose2D current_pose = compose_start_offset(
                nominal,
                run.ground_truth_offset,
                run.ground_truth_yaw_offset_rad);
            std::vector<Vec2> combined_current;
            run.matches.reserve(
                static_cast<std::size_t>(options.confirmation.required_matches));
            for (int confirmation_index = 0;
                 confirmation_index < options.confirmation.required_matches;
                 ++confirmation_index) {
                const std::uint32_t acquisition_seed =
                    run.seed ^ (0x85EBCA6BU *
                                static_cast<std::uint32_t>(confirmation_index + 1));
                std::mt19937 current_random(acquisition_seed);
                const std::vector<Vec2> current_points = acquire_stationary_cloud(
                    &world, current_pose, options, &current_random);
                combined_current.insert(combined_current.end(),
                                        current_points.begin(),
                                        current_points.end());
                run.matches.push_back(matcher.match(reference_points, current_points));
            }
            run.confirmation = confirm_initial_lidar_matches(
                run.matches, options.confirmation);
            const InitialLidarMatchResult& result = run.confirmation.fused_match;
            run.error_position = {
                result.position_offset_m.x - run.ground_truth_offset.x,
                result.position_offset_m.y - run.ground_truth_offset.y,
            };
            run.error_yaw_rad = wrap_angle(
                result.yaw_offset_rad - run.ground_truth_yaw_offset_rad);
            run.estimate_accurate =
                run.confirmation.valid &&
                std::hypot(run.error_position.x, run.error_position.y) <= 0.025 &&
                std::abs(run.error_yaw_rad) <= 1.5 * kPi / 180.0;
            if (!run.confirmation.valid) {
                run.classification = "invalid";
            } else if (run.expected_acceptance && run.confirmation.accepted) {
                run.classification = "true_positive";
            } else if (!run.expected_acceptance && !run.confirmation.accepted) {
                run.classification = "true_negative";
            } else if (!run.expected_acceptance && run.confirmation.accepted) {
                run.classification = "false_positive";
            } else {
                run.classification = "false_negative";
            }
            run.current_points = voxelize_initial_lidar_cloud(
                combined_current, options.sensor.voxel_size_m, 1);
            std::cout << "run=" << (run_index + 1) << '/' << options.runs
                      << " seed=" << run.seed
                      << " classification=" << run.classification
                      << " status=" << run.confirmation.status
                      << " gt_translation_m="
                      << std::hypot(run.ground_truth_offset.x,
                                    run.ground_truth_offset.y)
                      << " error_m="
                      << std::hypot(run.error_position.x, run.error_position.y)
                      << " error_yaw_deg=" << run.error_yaw_rad * 180.0 / kPi
                      << '\n';
            runs.push_back(std::move(run));
        }

        std::filesystem::create_directories(options.output_directory);
        const std::string stem = options.output_directory +
                                 "/thesis_sim_lidar_start_match_" + timestamp_token();
        const std::string report_path = stem + ".json";
        const std::string matrix_path = stem + "_matrix.csv";
        const std::string cloud_path = stem + "_clouds.csv";
        const std::string ply_path = stem + "_comparison.ply";
        const std::string generated_reference_path = options.reference_output_path.empty()
                                                         ? stem + "_reference.csv"
                                                         : options.reference_output_path;
        const std::filesystem::path reference_path(generated_reference_path);
        if (!reference_path.parent_path().empty()) {
            std::filesystem::create_directories(reference_path.parent_path());
        }
        InitialLidarReferenceCloud reference;
        reference.metadata = options.sensor;
        reference.metadata.created_utc = utc_timestamp_iso();
        reference.metadata.lidar_serial = "simulated_rplidar_a1";
        reference.metadata.lidar_firmware = "simulation";
        reference.metadata.source_scan_count = options.scan_count;
        reference.metadata.source_raw_point_count =
            static_cast<int>(reference_points.size());
        reference.metadata.stability_valid = true;
        reference.metadata.stability_translation_m = 0.0;
        reference.metadata.stability_yaw_rad = 0.0;
        reference.metadata.stability_rmse_m = 0.0;
        reference.metadata.stability_inlier_ratio = 1.0;
        reference.points = reference_points;
        const MatrixRunResult& first_run = runs.front();
        const InitialLidarMatchResult& first_result =
            first_run.confirmation.fused_match;
        const std::vector<Vec2> aligned = transform_initial_lidar_cloud(
            first_run.current_points,
            first_result.position_offset_m,
            first_result.yaw_offset_rad);
        std::string artifact_error;
        if (!write_initial_lidar_reference(generated_reference_path, reference, &artifact_error) ||
            !write_initial_lidar_cloud_comparison_csv(
                cloud_path,
                reference_points,
                first_run.current_points,
                aligned,
                &artifact_error) ||
            !write_initial_lidar_cloud_comparison_ply(
                ply_path,
                reference_points,
                first_run.current_points,
                aligned,
                &artifact_error) ||
            !write_matrix_csv(matrix_path, runs) ||
            !write_report(report_path,
                          options,
                          runs,
                          generated_reference_path,
                          cloud_path,
                          ply_path,
                          matrix_path)) {
            throw std::runtime_error(
                artifact_error.empty() ? "failed to write simulation artifacts" : artifact_error);
        }

        int true_positive = 0;
        int true_negative = 0;
        int false_positive = 0;
        int false_negative = 0;
        int invalid = 0;
        int inaccurate_accepted = 0;
        for (const MatrixRunResult& run : runs) {
            if (run.classification == "true_positive") ++true_positive;
            else if (run.classification == "true_negative") ++true_negative;
            else if (run.classification == "false_positive") ++false_positive;
            else if (run.classification == "false_negative") ++false_negative;
            else ++invalid;
            if (run.confirmation.accepted && !run.estimate_accurate) {
                ++inaccurate_accepted;
            }
        }
        std::cout << std::fixed << std::setprecision(6)
                  << "status=matrix_complete"
                  << "\nscenario=" << mode_name(options.environment_mode)
                  << "\nbase_seed=" << options.seed
                  << "\ncalibrated=" << (options.calibrated ? 1 : 0)
                  << "\nruns=" << runs.size()
                  << "\nconfirmations_per_run=" << options.confirmation.required_matches
                  << "\ntrue_positive=" << true_positive
                  << "\ntrue_negative=" << true_negative
                  << "\nfalse_positive=" << false_positive
                  << "\nfalse_negative=" << false_negative
                  << "\ninvalid=" << invalid
                  << "\ninaccurate_accepted=" << inaccurate_accepted
                  << "\nreport_json=" << report_path
                  << "\nmatrix_csv=" << matrix_path
                  << "\nreference_csv=" << generated_reference_path
                  << "\npoint_cloud_csv=" << cloud_path
                  << "\ncomparison_ply=" << ply_path << '\n';

        if (invalid > 0 || false_positive > 0 || inaccurate_accepted > 0) return 4;
        if (false_negative > 0) return 3;
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "sim_lidar_start_matcher_error=" << error.what() << '\n';
        return 1;
    }
}
