#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "mvc/controller/hardware_io/rplidar_a1.h"
#include "mvc/model/navigation/initial_lidar_matcher.h"

namespace {

constexpr double kPi = 3.14159265358979323846;

using thesis_sim::RPLidarA1;
using thesis_sim::Vec2;
using thesis_sim::mvc::model::InitialLidarMatchResult;
using thesis_sim::mvc::model::InitialLidarMatcher;
using thesis_sim::mvc::model::InitialLidarMatcherConfig;
using thesis_sim::mvc::model::InitialLidarConfirmationConfig;
using thesis_sim::mvc::model::InitialLidarConfirmationResult;
using thesis_sim::mvc::model::InitialLidarReferenceCloud;
using thesis_sim::mvc::model::InitialLidarReferenceMetadata;
using thesis_sim::mvc::model::read_initial_lidar_reference;
using thesis_sim::mvc::model::transform_initial_lidar_cloud;
using thesis_sim::mvc::model::voxelize_initial_lidar_cloud;
using thesis_sim::mvc::model::write_initial_lidar_reference;
using thesis_sim::mvc::model::write_initial_lidar_cloud_comparison_csv;
using thesis_sim::mvc::model::write_initial_lidar_cloud_comparison_ply;
using thesis_sim::mvc::model::confirm_initial_lidar_matches;

enum class RunMode {
    None,
    Capture,
    Match,
};

struct AppOptions {
    RunMode mode = RunMode::None;
    std::string lidar_port;
    int lidar_baudrate = 115200;
    int scan_count = 8;
    int minimum_scan_points = 120;
    std::string reference_path;
    std::string output_directory = "reports";
    InitialLidarReferenceMetadata sensor{};
    InitialLidarMatcherConfig matcher{};
    InitialLidarConfirmationConfig confirmation{};
    bool show_help = false;
};

struct RawLidarSample {
    int acquisition_index = 0;
    int scan_index = 0;
    int beam_index = 0;
    double scan_start_timestamp_s = 0.0;
    double scan_mid_timestamp_s = 0.0;
    double scan_end_timestamp_s = 0.0;
    double scan_duration_s = 0.0;
    double beam_timestamp_s = 0.0;
    int quality = 0;
    double angle_deg = 0.0;
    double range_m = 0.0;
    bool accepted = false;
    std::string rejection_reason;
    Vec2 base_point{};
};

struct AcquiredCloud {
    std::vector<Vec2> points;
    std::vector<std::vector<Vec2>> points_by_scan;
    std::vector<RawLidarSample> raw_samples;
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

void print_usage(const char* executable) {
    std::cout
        << "Usage:\n"
        << "  " << executable << " --lidar-port /dev/ttyUSB0 --capture-reference PATH [options]\n"
        << "  " << executable << " --lidar-port /dev/ttyUSB0 --reference PATH [options]\n\n"
        << "This tool opens only the LiDAR. It never opens the motor controller and cannot move the robot.\n\n"
        << "Acquisition options:\n"
        << "  --lidar-baudrate N        default 115200\n"
        << "  --scan-count N            stationary scans to aggregate, default 8\n"
        << "  --confirmations N         independent matches required, default 3\n"
        << "  --min-scan-points N       minimum returns per scan, default 120\n"
        << "  --output-dir PATH         match reports, default reports\n\n"
        << "Reference sensor geometry (capture mode):\n"
        << "  --vehicle-model NAME      car (25x15 cm) | tank (16.5x14.6 cm)\n"
        << "  --lidar-x M               base_link to LiDAR x, default 0.075\n"
        << "  --lidar-y M               base_link to LiDAR y, default 0.040\n"
        << "  --lidar-yaw-deg DEG       LiDAR yaw in base_link, default 162\n"
        << "  --lidar-no-mirror         do not invert the raw angular direction\n"
        << "  --min-range M             default 0.12\n"
        << "  --max-range M             default 3.0\n"
        << "  --voxel-size M            default 0.018\n\n"
        << "Acceptance options (match mode):\n"
        << "  --max-start-offset M      default 0.08\n"
        << "  --max-start-yaw-deg DEG   default 8\n"
        << "  --search-translation M    default +/-0.30\n"
        << "  --search-yaw-deg DEG      default +/-30\n"
        << "  --max-rmse M              default 0.045\n"
        << "  --min-inlier-ratio R      default 0.55\n"
        << "  --min-ambiguity R         default 0.025\n"
        << "  --max-confirm-spread M    default 0.025\n"
        << "  --max-confirm-yaw-deg D   default 2\n"
        << "  --matcher-timeout-ms N    per-match timeout, default 20000\n";
}

AppOptions parse_options(int argc, char** argv) {
    AppOptions options;
    for (int i = 1; i < argc; ++i) {
        const std::string argument = argv[i];
        const auto require_value = [&](const std::string& option) -> std::string {
            if (i + 1 >= argc) throw std::invalid_argument("missing value for " + option);
            return argv[++i];
        };
        if (argument == "--lidar-port") {
            options.lidar_port = require_value(argument);
        } else if (argument == "--lidar-baudrate") {
            options.lidar_baudrate = parse_int(require_value(argument), argument);
        } else if (argument == "--scan-count") {
            options.scan_count = parse_int(require_value(argument), argument);
        } else if (argument == "--confirmations") {
            options.confirmation.required_matches = parse_int(require_value(argument), argument);
        } else if (argument == "--min-scan-points") {
            options.minimum_scan_points = parse_int(require_value(argument), argument);
        } else if (argument == "--capture-reference") {
            if (options.mode != RunMode::None) throw std::invalid_argument("select only one run mode");
            options.mode = RunMode::Capture;
            options.reference_path = require_value(argument);
        } else if (argument == "--reference") {
            if (options.mode != RunMode::None) throw std::invalid_argument("select only one run mode");
            options.mode = RunMode::Match;
            options.reference_path = require_value(argument);
        } else if (argument == "--output-dir") {
            options.output_directory = require_value(argument);
        } else if (argument == "--vehicle-model") {
            const std::string model = require_value(argument);
            if (model == "tank" || model == "tracked") {
                options.sensor.body_length_m = 0.165;
                options.sensor.body_width_m = 0.146;
            } else if (model == "car") {
                options.sensor.body_length_m = 0.25;
                options.sensor.body_width_m = 0.15;
            } else {
                throw std::invalid_argument("unknown --vehicle-model: " + model);
            }
        } else if (argument == "--lidar-x") {
            options.sensor.lidar_x_offset_m = parse_double(require_value(argument), argument);
        } else if (argument == "--lidar-y") {
            options.sensor.lidar_y_offset_m = parse_double(require_value(argument), argument);
        } else if (argument == "--lidar-yaw-deg") {
            options.sensor.lidar_yaw_offset_rad =
                parse_double(require_value(argument), argument) * kPi / 180.0;
        } else if (argument == "--lidar-no-mirror") {
            options.sensor.lidar_flip_left_right = false;
        } else if (argument == "--min-range") {
            options.sensor.min_range_m = parse_double(require_value(argument), argument);
        } else if (argument == "--max-range") {
            options.sensor.max_range_m = parse_double(require_value(argument), argument);
        } else if (argument == "--voxel-size") {
            options.sensor.voxel_size_m = parse_double(require_value(argument), argument);
        } else if (argument == "--max-start-offset") {
            options.matcher.maximum_start_translation_m = parse_double(require_value(argument), argument);
        } else if (argument == "--max-start-yaw-deg") {
            options.matcher.maximum_start_yaw_rad =
                parse_double(require_value(argument), argument) * kPi / 180.0;
        } else if (argument == "--search-translation") {
            options.matcher.search_translation_m = parse_double(require_value(argument), argument);
        } else if (argument == "--search-yaw-deg") {
            options.matcher.search_yaw_rad = parse_double(require_value(argument), argument) * kPi / 180.0;
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
        } else if (argument == "--help" || argument == "-h") {
            options.show_help = true;
        } else {
            throw std::invalid_argument("unknown option: " + argument);
        }
    }
    options.scan_count = std::clamp(options.scan_count, 1, 50);
    options.confirmation.required_matches = std::clamp(
        options.confirmation.required_matches, 1, 10);
    options.minimum_scan_points = std::clamp(options.minimum_scan_points, 20, 1000);
    if (!options.show_help) {
        if (options.mode == RunMode::None) throw std::invalid_argument("select --capture-reference or --reference");
        if (options.lidar_port.empty()) throw std::invalid_argument("--lidar-port is required");
        if (options.reference_path.empty()) throw std::invalid_argument("reference path is empty");
        if (!(options.sensor.min_range_m >= 0.0) ||
            !(options.sensor.max_range_m > options.sensor.min_range_m) ||
            !(options.sensor.voxel_size_m > 0.0)) {
            throw std::invalid_argument("invalid LiDAR range or voxel configuration");
        }
    }
    return options;
}

double wrap_angle(double angle) {
    while (angle > kPi) angle -= 2.0 * kPi;
    while (angle < -kPi) angle += 2.0 * kPi;
    return angle;
}

double wall_clock_seconds() {
    return std::chrono::duration<double>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
}

std::vector<Vec2> scan_to_base_points(
    const std::vector<RPLidarA1::ScanPoint>& scan,
    const InitialLidarReferenceMetadata& sensor,
    int acquisition_index,
    int scan_index,
    double scan_start_timestamp_s,
    double scan_end_timestamp_s,
    std::vector<RawLidarSample>* raw_samples) {
    std::vector<Vec2> points;
    points.reserve(scan.size());
    const double half_length = 0.5 * sensor.body_length_m + 0.015;
    const double half_width = 0.5 * sensor.body_width_m + 0.015;
    for (std::size_t beam_index = 0; beam_index < scan.size(); ++beam_index) {
        const RPLidarA1::ScanPoint& point = scan[beam_index];
        RawLidarSample raw;
        raw.acquisition_index = acquisition_index;
        raw.scan_index = scan_index;
        raw.beam_index = static_cast<int>(beam_index);
        raw.scan_start_timestamp_s = scan_start_timestamp_s;
        raw.scan_end_timestamp_s = scan_end_timestamp_s;
        raw.scan_mid_timestamp_s = 0.5 * (scan_start_timestamp_s + scan_end_timestamp_s);
        raw.scan_duration_s = std::max(0.0, scan_end_timestamp_s - scan_start_timestamp_s);
        const double beam_fraction = scan.size() > 1U
                                         ? static_cast<double>(beam_index) /
                                               static_cast<double>(scan.size() - 1U)
                                         : 0.5;
        raw.beam_timestamp_s = scan_start_timestamp_s +
                               beam_fraction * raw.scan_duration_s;
        raw.quality = static_cast<int>(point.quality);
        raw.angle_deg = point.angle_deg;
        raw.range_m = point.distance_m;
        if (!std::isfinite(point.distance_m) ||
            point.distance_m < sensor.min_range_m ||
            point.distance_m > sensor.max_range_m) {
            raw.rejection_reason = "invalid_range";
            if (raw_samples != nullptr) raw_samples->push_back(std::move(raw));
            continue;
        }
        const double raw_angle = point.angle_deg * kPi / 180.0;
        const double sensor_angle = sensor.lidar_flip_left_right ? -raw_angle : raw_angle;
        const double body_angle = wrap_angle(sensor.lidar_yaw_offset_rad + sensor_angle);
        const Vec2 endpoint{
            sensor.lidar_x_offset_m + std::cos(body_angle) * point.distance_m,
            sensor.lidar_y_offset_m + std::sin(body_angle) * point.distance_m,
        };
        raw.base_point = endpoint;
        if (std::abs(endpoint.x) <= half_length && std::abs(endpoint.y) <= half_width) {
            raw.rejection_reason = "self_hit";
            if (raw_samples != nullptr) raw_samples->push_back(std::move(raw));
            continue;
        }
        raw.accepted = true;
        raw.rejection_reason = "accepted";
        if (raw_samples != nullptr) raw_samples->push_back(std::move(raw));
        points.push_back(endpoint);
    }
    return points;
}

class LidarSession {
  public:
    LidarSession(const std::string& port, int baudrate)
        : lidar_(port, baudrate, 1.0) {
        lidar_.connect();
        info_ = lidar_.get_info();
        health_ = lidar_.get_health();
        if (health_.status_code == 2U) {
            throw std::runtime_error("LiDAR health error 0x" + health_code_text());
        }
        lidar_.start_scan();
    }

    ~LidarSession() {
        try {
            lidar_.stop_scan(true);
        } catch (...) {
        }
        lidar_.disconnect();
    }

    std::vector<RPLidarA1::ScanPoint> grab(int minimum_points) {
        return lidar_.grab_scan(minimum_points);
    }

    const RPLidarA1::DeviceInfo& info() const { return info_; }
    const RPLidarA1::Health& health() const { return health_; }

  private:
    std::string health_code_text() const {
        std::ostringstream text;
        text << std::hex << health_.error_code;
        return text.str();
    }

    RPLidarA1 lidar_;
    RPLidarA1::DeviceInfo info_{};
    RPLidarA1::Health health_{};
};

AcquiredCloud acquire_stationary_cloud(LidarSession* lidar,
                                       int acquisition_index,
                                       int scan_count,
                                       int minimum_scan_points,
                                       const InitialLidarReferenceMetadata& sensor) {
    AcquiredCloud acquired;
    std::vector<Vec2> accumulated;
    acquired.points_by_scan.reserve(static_cast<std::size_t>(scan_count));
    for (int scan_index = 0; scan_index < scan_count; ++scan_index) {
        const double scan_start_timestamp_s = wall_clock_seconds();
        const std::vector<RPLidarA1::ScanPoint> scan = lidar->grab(minimum_scan_points);
        const double scan_end_timestamp_s = wall_clock_seconds();
        const std::vector<Vec2> points = scan_to_base_points(
            scan,
            sensor,
            acquisition_index,
            scan_index,
            scan_start_timestamp_s,
            scan_end_timestamp_s,
            &acquired.raw_samples);
        accumulated.insert(accumulated.end(), points.begin(), points.end());
        acquired.points_by_scan.push_back(points);
        std::cout << "acquisition=" << (acquisition_index + 1)
                  << " scan=" << (scan_index + 1) << '/' << scan_count
                  << " raw_points=" << scan.size()
                  << " accepted_points=" << points.size()
                  << " duration_s=" << (scan_end_timestamp_s - scan_start_timestamp_s)
                  << '\n';
    }
    acquired.points = voxelize_initial_lidar_cloud(
        accumulated,
        sensor.voxel_size_m,
        scan_count >= 3 ? 2 : 1);
    return acquired;
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

std::string append_path_suffix(const std::string& path,
                               const std::string& suffix) {
    const std::filesystem::path source(path);
    return (source.parent_path() /
            (source.stem().string() + suffix + source.extension().string()))
        .string();
}

bool write_raw_scan_csv(const std::string& path,
                        const std::vector<AcquiredCloud>& acquisitions) {
    std::ofstream out(path, std::ios::out | std::ios::trunc);
    if (!out.is_open()) return false;
    out << std::setprecision(12)
        << "acquisition_index,scan_index,beam_index,scan_start_timestamp_s,"
           "scan_mid_timestamp_s,scan_end_timestamp_s,scan_duration_s,"
           "beam_timestamp_s,beam_timestamp_interpolated,quality,"
           "angle_deg,range_m,accepted,rejection_reason,base_x_m,base_y_m\n";
    for (const AcquiredCloud& acquisition : acquisitions) {
        for (const RawLidarSample& sample : acquisition.raw_samples) {
            out << sample.acquisition_index << ','
                << sample.scan_index << ','
                << sample.beam_index << ','
                << sample.scan_start_timestamp_s << ','
                << sample.scan_mid_timestamp_s << ','
                << sample.scan_end_timestamp_s << ','
                << sample.scan_duration_s << ','
                << sample.beam_timestamp_s << ",1,"
                << sample.quality << ','
                << sample.angle_deg << ','
                << sample.range_m << ','
                << (sample.accepted ? 1 : 0) << ','
                << sample.rejection_reason << ','
                << sample.base_point.x << ','
                << sample.base_point.y << '\n';
        }
    }
    return out.good();
}

InitialLidarMatchResult evaluate_reference_stability(
    const AcquiredCloud& acquisition,
    const InitialLidarReferenceMetadata& sensor,
    const InitialLidarMatcherConfig& matcher_config) {
    std::vector<Vec2> even_points;
    std::vector<Vec2> odd_points;
    for (std::size_t index = 0; index < acquisition.points_by_scan.size(); ++index) {
        std::vector<Vec2>& destination = (index % 2U) == 0U ? even_points : odd_points;
        destination.insert(destination.end(),
                           acquisition.points_by_scan[index].begin(),
                           acquisition.points_by_scan[index].end());
    }
    even_points = voxelize_initial_lidar_cloud(even_points, sensor.voxel_size_m, 1);
    odd_points = voxelize_initial_lidar_cloud(odd_points, sensor.voxel_size_m, 1);
    InitialLidarMatcherConfig stability_config = matcher_config;
    stability_config.search_translation_m = std::min(stability_config.search_translation_m, 0.08);
    stability_config.search_yaw_rad = std::min(stability_config.search_yaw_rad, 8.0 * kPi / 180.0);
    stability_config.maximum_start_translation_m = 0.020;
    stability_config.maximum_start_yaw_rad = 2.0 * kPi / 180.0;
    return InitialLidarMatcher(stability_config).match(even_points, odd_points);
}

std::string json_escape(const std::string& value) {
    std::string escaped;
    escaped.reserve(value.size());
    for (char ch : value) {
        if (ch == '\\' || ch == '"') escaped.push_back('\\');
        escaped.push_back(ch);
    }
    return escaped;
}

bool write_match_report(const std::string& path,
                        const AppOptions& options,
                        const InitialLidarReferenceCloud& reference,
                        const InitialLidarConfirmationResult& confirmation,
                        const std::vector<InitialLidarMatchResult>& matches,
                        const RPLidarA1::DeviceInfo& info,
                        const RPLidarA1::Health& health,
                        const std::string& raw_scan_csv,
                        const std::string& cloud_csv,
                        const std::string& comparison_ply) {
    std::ofstream out(path, std::ios::out | std::ios::trunc);
    if (!out.is_open()) return false;
    const InitialLidarMatchResult& result = confirmation.fused_match;
    const double translation = std::hypot(result.position_offset_m.x, result.position_offset_m.y);
    out << std::boolalpha << std::setprecision(12)
        << "{\n"
        << "  \"schema\": \"thesis_initial_lidar_match_v2\",\n"
        << "  \"reference_path\": \"" << json_escape(options.reference_path) << "\",\n"
        << "  \"reference_schema\": \"" << json_escape(reference.metadata.schema) << "\",\n"
        << "  \"reference_hash\": \"" << json_escape(reference.computed_content_hash) << "\",\n"
        << "  \"reference_integrity_verified\": " << reference.integrity_verified << ",\n"
        << "  \"reference_stability_valid\": " << reference.metadata.stability_valid << ",\n"
        << "  \"lidar_serial\": \"" << json_escape(info.serial_hex) << "\",\n"
        << "  \"lidar_firmware\": \"" << static_cast<int>(info.firmware_major) << '.'
        << static_cast<int>(info.firmware_minor) << "\",\n"
        << "  \"lidar_health\": \"" << json_escape(health.status) << "\",\n"
        << "  \"scan_count\": " << options.scan_count << ",\n"
        << "  \"confirmation_required\": " << options.confirmation.required_matches << ",\n"
        << "  \"confirmation_valid_matches\": " << confirmation.valid_matches << ",\n"
        << "  \"confirmation_accepted_matches\": " << confirmation.accepted_matches << ",\n"
        << "  \"confirmation_position_spread_m\": " << confirmation.maximum_position_spread_m << ",\n"
        << "  \"confirmation_yaw_spread_rad\": " << confirmation.maximum_yaw_spread_rad << ",\n"
        << "  \"reference_points\": " << result.reference_points << ",\n"
        << "  \"current_points\": " << result.current_points << ",\n"
        << "  \"matched_points\": " << result.matched_points << ",\n"
        << "  \"valid\": " << confirmation.valid << ",\n"
        << "  \"accepted\": " << confirmation.accepted << ",\n"
        << "  \"status\": \"" << json_escape(confirmation.status) << "\",\n"
        << "  \"offset_x_m\": " << result.position_offset_m.x << ",\n"
        << "  \"offset_y_m\": " << result.position_offset_m.y << ",\n"
        << "  \"offset_translation_m\": " << translation << ",\n"
        << "  \"offset_yaw_rad\": " << result.yaw_offset_rad << ",\n"
        << "  \"offset_yaw_deg\": " << result.yaw_offset_rad * 180.0 / kPi << ",\n"
        << "  \"score_m\": " << result.score_m << ",\n"
        << "  \"rmse_m\": " << result.rmse_m << ",\n"
        << "  \"inlier_ratio\": " << result.inlier_ratio << ",\n"
        << "  \"ambiguity_margin\": " << result.ambiguity_margin << ",\n"
        << "  \"confidence\": " << result.confidence << ",\n"
        << "  \"covariance_x_m2\": " << result.covariance_x_m2 << ",\n"
        << "  \"covariance_y_m2\": " << result.covariance_y_m2 << ",\n"
        << "  \"covariance_yaw_rad2\": " << result.covariance_yaw_rad2 << ",\n"
        << "  \"observability_x\": " << result.observability_x << ",\n"
        << "  \"observability_y\": " << result.observability_y << ",\n"
        << "  \"observability_yaw\": " << result.observability_yaw << ",\n"
        << "  \"coarse_compute_ms\": " << result.coarse_compute_ms << ",\n"
        << "  \"fine_compute_ms\": " << result.fine_compute_ms << ",\n"
        << "  \"total_compute_ms\": " << result.total_compute_ms << ",\n"
        << "  \"evaluated_candidates\": " << result.evaluated_candidates << ",\n"
        << "  \"timed_out\": " << result.timed_out << ",\n"
        << "  \"maximum_start_translation_m\": " << options.matcher.maximum_start_translation_m << ",\n"
        << "  \"maximum_start_yaw_rad\": " << options.matcher.maximum_start_yaw_rad << ",\n"
        << "  \"matches\": [\n";
    for (std::size_t index = 0; index < matches.size(); ++index) {
        const InitialLidarMatchResult& match = matches[index];
        out << "    {\"index\": " << index
            << ", \"valid\": " << match.valid
            << ", \"accepted\": " << match.accepted
            << ", \"status\": \"" << json_escape(match.status) << "\""
            << ", \"x_m\": " << match.position_offset_m.x
            << ", \"y_m\": " << match.position_offset_m.y
            << ", \"yaw_rad\": " << match.yaw_offset_rad
            << ", \"rmse_m\": " << match.rmse_m
            << ", \"inlier_ratio\": " << match.inlier_ratio
            << ", \"ambiguity_margin\": " << match.ambiguity_margin
            << ", \"compute_ms\": " << match.total_compute_ms << "}"
            << (index + 1U < matches.size() ? ",\n" : "\n");
    }
    out << "  ],\n"
        << "  \"raw_scan_csv\": \"" << json_escape(raw_scan_csv) << "\",\n"
        << "  \"cloud_csv\": \"" << json_escape(cloud_csv) << "\",\n"
        << "  \"comparison_ply\": \"" << json_escape(comparison_ply) << "\"\n"
        << "}\n";
    return out.good();
}

}  // namespace

int main(int argc, char** argv) {
    try {
        AppOptions options = parse_options(argc, argv);
        if (options.show_help) {
            print_usage(argv[0]);
            return 0;
        }

        InitialLidarReferenceCloud reference;
        if (options.mode == RunMode::Match) {
            std::string error;
            if (!read_initial_lidar_reference(options.reference_path, &reference, &error)) {
                throw std::runtime_error(error);
            }
            options.sensor = reference.metadata;
        }

        std::cout << "motor_controller_opened=0\n"
                  << "lidar_start_match_mode="
                  << (options.mode == RunMode::Capture ? "capture" : "match") << '\n'
                  << "Keep the robot completely still while scans are acquired.\n";
        LidarSession lidar(options.lidar_port, options.lidar_baudrate);
        std::cout << "lidar_serial=" << lidar.info().serial_hex
                  << "\nlidar_health=" << lidar.health().status << '\n';

        if (options.mode == RunMode::Capture) {
            if (options.scan_count < 4) {
                throw std::runtime_error(
                    "reference capture requires at least 4 scans for its stability check");
            }
            const std::filesystem::path reference_path(options.reference_path);
            if (!reference_path.parent_path().empty()) {
                std::filesystem::create_directories(reference_path.parent_path());
            }
            const AcquiredCloud capture = acquire_stationary_cloud(
                &lidar,
                0,
                options.scan_count,
                options.minimum_scan_points,
                options.sensor);
            const std::string raw_path = append_path_suffix(
                options.reference_path, "_raw_scans");
            if (!write_raw_scan_csv(raw_path, {capture})) {
                throw std::runtime_error("failed to write reference raw scan log");
            }
            const InitialLidarMatchResult stability = evaluate_reference_stability(
                capture, options.sensor, options.matcher);
            reference.metadata = options.sensor;
            reference.metadata.created_utc = utc_timestamp_iso();
            reference.metadata.lidar_serial = lidar.info().serial_hex;
            reference.metadata.lidar_firmware =
                std::to_string(static_cast<int>(lidar.info().firmware_major)) + "." +
                std::to_string(static_cast<int>(lidar.info().firmware_minor));
            reference.metadata.lidar_hardware = static_cast<int>(lidar.info().hardware);
            reference.metadata.source_scan_count = options.scan_count;
            reference.metadata.source_raw_point_count =
                static_cast<int>(capture.raw_samples.size());
            reference.metadata.stability_valid = stability.valid && stability.accepted;
            reference.metadata.stability_translation_m = std::hypot(
                stability.position_offset_m.x, stability.position_offset_m.y);
            reference.metadata.stability_yaw_rad = stability.yaw_offset_rad;
            reference.metadata.stability_rmse_m = stability.rmse_m;
            reference.metadata.stability_inlier_ratio = stability.inlier_ratio;
            reference.metadata.raw_scan_log_path = raw_path;
            reference.points = capture.points;
            std::cout << "voxelized_points=" << reference.points.size()
                      << "\nreference_stability_status=" << stability.status
                      << "\nreference_stability_translation_m="
                      << reference.metadata.stability_translation_m
                      << "\nreference_stability_yaw_deg="
                      << reference.metadata.stability_yaw_rad * 180.0 / kPi
                      << "\nreference_stability_rmse_m="
                      << reference.metadata.stability_rmse_m
                      << "\nreference_stability_inlier_ratio="
                      << reference.metadata.stability_inlier_ratio << '\n';
            if (reference.points.size() <
                    static_cast<std::size_t>(options.matcher.minimum_points) ||
                !reference.metadata.stability_valid) {
                std::cout << "status=reference_stability_rejected\n"
                          << "raw_scan_csv=" << raw_path << '\n';
                return 4;
            }
            std::string error;
            if (!write_initial_lidar_reference(options.reference_path, reference, &error)) {
                throw std::runtime_error(error);
            }
            InitialLidarReferenceCloud verified_reference;
            if (!read_initial_lidar_reference(
                    options.reference_path, &verified_reference, &error) ||
                !verified_reference.integrity_verified) {
                throw std::runtime_error(
                    error.empty() ? "reference write-back integrity check failed" : error);
            }
            std::cout << "status=reference_captured\n"
                      << "reference_path=" << options.reference_path << '\n'
                      << "reference_points=" << reference.points.size()
                      << "\nreference_hash=" << verified_reference.computed_content_hash
                      << "\nreference_integrity_verified=1"
                      << "\nraw_scan_csv=" << raw_path << '\n';
            return 0;
        }

        if (reference.metadata.schema == "thesis_initial_lidar_reference_v2" &&
            (!reference.integrity_verified || !reference.metadata.stability_valid)) {
            throw std::runtime_error(
                "reference v2 failed integrity or capture-stability validation");
        }
        if (!reference.metadata.lidar_serial.empty() &&
            reference.metadata.lidar_serial != lidar.info().serial_hex) {
            throw std::runtime_error(
                "LiDAR serial does not match the sensor used to capture the reference");
        }

        InitialLidarMatcher matcher(options.matcher);
        std::vector<AcquiredCloud> acquisitions;
        std::vector<InitialLidarMatchResult> matches;
        acquisitions.reserve(static_cast<std::size_t>(options.confirmation.required_matches));
        matches.reserve(static_cast<std::size_t>(options.confirmation.required_matches));
        std::vector<Vec2> combined_current_points;
        for (int confirmation_index = 0;
             confirmation_index < options.confirmation.required_matches;
             ++confirmation_index) {
            AcquiredCloud acquisition = acquire_stationary_cloud(
                &lidar,
                confirmation_index,
                options.scan_count,
                options.minimum_scan_points,
                options.sensor);
            std::cout << "acquisition_voxelized_points=" << acquisition.points.size() << '\n';
            matches.push_back(matcher.match(reference.points, acquisition.points));
            const InitialLidarMatchResult& match = matches.back();
            std::cout << "match=" << (confirmation_index + 1)
                      << " status=" << match.status
                      << " x_m=" << match.position_offset_m.x
                      << " y_m=" << match.position_offset_m.y
                      << " yaw_deg=" << match.yaw_offset_rad * 180.0 / kPi
                      << " compute_ms=" << match.total_compute_ms << '\n';
            combined_current_points.insert(combined_current_points.end(),
                                           acquisition.points.begin(),
                                           acquisition.points.end());
            acquisitions.push_back(std::move(acquisition));
        }
        const InitialLidarConfirmationResult confirmation =
            confirm_initial_lidar_matches(matches, options.confirmation);
        const InitialLidarMatchResult& result = confirmation.fused_match;
        const std::vector<Vec2> current = voxelize_initial_lidar_cloud(
            combined_current_points, options.sensor.voxel_size_m, 1);
        const std::vector<Vec2> aligned = transform_initial_lidar_cloud(
            current, result.position_offset_m, result.yaw_offset_rad);

        std::filesystem::create_directories(options.output_directory);
        const std::string stem = options.output_directory +
                                 "/thesis_hardware_lidar_start_match_" + timestamp_token();
        const std::string report_path = stem + ".json";
        const std::string raw_path = stem + "_raw_scans.csv";
        const std::string cloud_path = stem + "_clouds.csv";
        const std::string ply_path = stem + "_comparison.ply";
        if (!write_raw_scan_csv(raw_path, acquisitions) ||
            !write_initial_lidar_cloud_comparison_csv(
                cloud_path, reference.points, current, aligned) ||
            !write_initial_lidar_cloud_comparison_ply(
                ply_path, reference.points, current, aligned) ||
            !write_match_report(report_path,
                                options,
                                reference,
                                confirmation,
                                matches,
                                lidar.info(),
                                lidar.health(),
                                raw_path,
                                cloud_path,
                                ply_path)) {
            throw std::runtime_error("failed to write matcher artifacts");
        }

        std::cout << std::fixed << std::setprecision(6)
                  << "status=" << confirmation.status
                  << "\nstart_pose_accepted=" << (confirmation.accepted ? 1 : 0)
                  << "\nconfirmation_position_spread_m="
                  << confirmation.maximum_position_spread_m
                  << "\nconfirmation_yaw_spread_deg="
                  << confirmation.maximum_yaw_spread_rad * 180.0 / kPi
                  << "\noffset_x_m=" << result.position_offset_m.x
                  << "\noffset_y_m=" << result.position_offset_m.y
                  << "\noffset_translation_m="
                  << std::hypot(result.position_offset_m.x, result.position_offset_m.y)
                  << "\noffset_yaw_deg=" << result.yaw_offset_rad * 180.0 / kPi
                  << "\nrmse_m=" << result.rmse_m
                  << "\ninlier_ratio=" << result.inlier_ratio
                  << "\nambiguity_margin=" << result.ambiguity_margin
                  << "\nconfidence=" << result.confidence
                  << "\nobservability_x=" << result.observability_x
                  << "\nobservability_y=" << result.observability_y
                  << "\nobservability_yaw=" << result.observability_yaw
                  << "\ntotal_compute_ms=" << result.total_compute_ms
                  << "\nevaluated_candidates=" << result.evaluated_candidates
                  << "\nreport_json=" << report_path
                  << "\nraw_scan_csv=" << raw_path
                  << "\npoint_cloud_csv=" << cloud_path
                  << "\ncomparison_ply=" << ply_path << '\n';
        if (!confirmation.valid) return 4;
        return confirmation.accepted ? 0 : 3;
    } catch (const std::exception& error) {
        std::cerr << "lidar_start_matcher_error=" << error.what() << '\n';
        return 1;
    }
}
