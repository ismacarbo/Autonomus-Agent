#include "mvc/controller/application/simulation_report_controller.h"

#include "mvc/model/reporting/slam_reference_exporter.h"

#include <algorithm>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>

namespace thesis_sim::mvc::controller {
namespace {

struct MetricSummary {
    double avg = 0.0;
    double max = 0.0;
};

size_t plot_sample_stride(size_t point_count) {
    constexpr size_t kMaxReportPoints = 600;
    return point_count <= kMaxReportPoints
               ? 1
               : std::max<size_t>(1, point_count / kMaxReportPoints);
}

std::string json_escape(const std::string& value) {
    std::string escaped;
    escaped.reserve(value.size() + 8);
    for (const char ch : value) {
        switch (ch) {
            case '\\': escaped += "\\\\"; break;
            case '"': escaped += "\\\""; break;
            case '\n': escaped += "\\n"; break;
            case '\r': escaped += "\\r"; break;
            case '\t': escaped += "\\t"; break;
            default: escaped.push_back(ch); break;
        }
    }
    return escaped;
}

std::string slugify(std::string value) {
    for (char& ch : value) {
        if (ch >= 'A' && ch <= 'Z') {
            ch = static_cast<char>(ch - 'A' + 'a');
        } else if (!((ch >= 'a' && ch <= 'z') ||
                     (ch >= '0' && ch <= '9'))) {
            ch = '_';
        }
    }
    return value;
}

const char* environment_mode_slug(EnvironmentMode mode) {
    switch (mode) {
        case EnvironmentMode::StructuredRoad: return "structured";
        case EnvironmentMode::MixedRoadGates: return "mixed";
        case EnvironmentMode::UnstructuredGates:
        default: return "unstructured";
    }
}

std::string map_preset_name(const WorldMap& world) {
    if (world.environment_mode() == EnvironmentMode::StructuredRoad) {
        return structured_map_preset_name(world.structured_preset());
    }
    if (world.environment_mode() == EnvironmentMode::MixedRoadGates) {
        const Rect& bounds = world.bounds();
        const double span =
            std::max(bounds.max_x - bounds.min_x,
                     bounds.max_y - bounds.min_y);
        if (!world.dynamic_obstacles().empty()) {
            return "Mixed Dynamic Obstacle Road";
        }
        if (world.structured_preset() == StructuredMapPreset::IdealCircle &&
            !world.obstacles().empty()) {
            return "Mixed Ideal Hardware-Aligned";
        }
        if (world.structured_preset() == StructuredMapPreset::TankCircuit) {
            return span <= 1.35
                       ? "Mixed Tank Hardware Lab"
                       : "Mixed Closed Obstacle Road";
        }
        if (span <= 1.20 && !world.obstacles().empty()) {
            return "Mixed Hardware Aligned";
        }
        if (span <= 1.20) {
            return "Mixed Hardware Runner";
        }
        return "Mixed Road Gate Validation";
    }
    return unstructured_map_preset_name(world.unstructured_preset());
}

std::string append_stem_suffix(const std::string& path,
                               const char* suffix,
                               const char* extension) {
    std::filesystem::path output(path);
    output.replace_filename(
        output.stem().string() +
        (suffix != nullptr ? suffix : "") +
        (extension != nullptr ? extension : ""));
    return output.string();
}

void write_vec2_json(std::ostream& out, const Vec2& value) {
    out << "{\"x\":" << value.x << ",\"y\":" << value.y << "}";
}

void write_rect_json(std::ostream& out, const Rect& rect) {
    out << "{\"min_x\":" << rect.min_x
        << ",\"min_y\":" << rect.min_y
        << ",\"max_x\":" << rect.max_x
        << ",\"max_y\":" << rect.max_y
        << "}";
}

MetricSummary summarize_metric(const std::vector<TelemetrySample>& history,
                               double TelemetrySample::*member) {
    MetricSummary summary;
    if (history.empty()) {
        return summary;
    }
    double total = 0.0;
    for (const TelemetrySample& sample : history) {
        const double value = sample.*member;
        total += value;
        summary.max = std::max(summary.max, value);
    }
    summary.avg = total / static_cast<double>(history.size());
    return summary;
}

bool write_sim_slam_reference_png(const PlannerDrivenVehicleSim& sim,
                                  const std::string& report_path) {
    const WorldMap& world = sim.world();
    model::SlamReferenceArtifact artifact;
    artifact.bounds = world.bounds();
    artifact.reference_obstacles = world.obstacles();
    artifact.reference_obstacles.insert(
        artifact.reference_obstacles.end(),
        world.perception_obstacles().begin(),
        world.perception_obstacles().end());
    artifact.reference_road = world.road_centerline();
    artifact.occupied_points = sim.slam_points();
    artifact.estimated_trail =
        sim.estimated_trail().empty() ? sim.trail() : sim.estimated_trail();
    artifact.start = world.start();
    artifact.goal = world.goal();
    artifact.current = sim.navigation_position();
    return model::write_slam_reference_png(artifact, report_path);
}

}  // namespace

std::string report_status_string(const PlannerDrivenVehicleSim& sim) {
    if (sim.goal_reached()) {
        return "goal_reached";
    }
    if (sim.collision()) {
        return "collision";
    }
    return "stopped";
}

std::string default_report_path(const PlannerDrivenVehicleSim& sim,
                                const char* source_tag) {
    namespace fs = std::filesystem;
    std::error_code ec;
    fs::path report_dir = fs::current_path(ec) / "reports";
    if (!ec) {
        fs::create_directories(report_dir, ec);
    }

    const auto now = std::chrono::system_clock::now();
    const std::time_t now_time =
        std::chrono::system_clock::to_time_t(now);
    std::tm local_tm{};
    if (const std::tm* tm_ptr = std::localtime(&now_time)) {
        local_tm = *tm_ptr;
    }
    const auto millis =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch()).count() % 1000;

    std::ostringstream name;
    name << "thesis_planner_"
         << environment_mode_slug(sim.environment_mode())
         << "_" << slugify(map_preset_name(sim.world()));
    if (sim.vehicle_model_kind() != VehicleModelKind::CarLikeBicycle) {
        name << "_"
             << slugify(vehicle_model_kind_name(sim.vehicle_model_kind()));
    }
    if (sim.config().dynamic_lidar_gates) {
        name << "_lidar_dynamic";
    }
    if (sim.config().ideal_conditions) {
        name << "_ideal";
    } else if (sim.config().calibrated_conditions) {
        name << "_calibrated";
    }
    name << "_" << (source_tag != nullptr ? source_tag : "run") << "_"
         << std::put_time(&local_tm, "%Y%m%d_%H%M%S")
         << "_" << std::setw(3) << std::setfill('0') << millis
         << ".json";
    return (report_dir / name.str()).string();
}

bool write_json_report(const PlannerDrivenVehicleSim& sim,
                       const std::string& status,
                       const std::string& report_path) {
    std::ofstream out(report_path, std::ios::out | std::ios::trunc);
    if (!out.is_open()) {
        return false;
    }

    const int passed_gates = sim.passed_gate_count();

    const SimulationReport report{
        sim.goal_reached(),
        sim.collision(),
        sim.step_count(),
        sim.sim_time(),
        sim.vehicle().position,
        sim.distance_to_goal(),
        passed_gates,
    };
    const auto& world = sim.world();
    const auto& config = sim.config();
    const auto& history = sim.history();
    const size_t stride = plot_sample_stride(history.size());

    const MetricSummary planning_summary = summarize_metric(history, &TelemetrySample::planning_ms);
    const MetricSummary tracking_summary = summarize_metric(history, &TelemetrySample::tracking_ms);
    const MetricSummary lidar_summary = summarize_metric(history, &TelemetrySample::lidar_ms);
    const MetricSummary estimator_summary = summarize_metric(history, &TelemetrySample::estimator_ms);
    const MetricSummary step_summary = summarize_metric(history, &TelemetrySample::step_ms);

    out << std::fixed << std::setprecision(6);
    out << "{\n";
    out << "  \"schema\": \"thesis_planner_report_v1\",\n";
    out << "  \"status\": \"" << json_escape(status) << "\",\n";
    out << "  \"environment\": \"" << json_escape(thesis_sim::environment_mode_name(sim.environment_mode())) << "\",\n";
    out << "  \"unstructured_preset\": \"" << json_escape(thesis_sim::unstructured_map_preset_name(world.unstructured_preset())) << "\",\n";
    out << "  \"structured_preset\": \"" << json_escape(thesis_sim::structured_map_preset_name(world.structured_preset())) << "\",\n";
    out << "  \"gate_behavior\": \"" << json_escape(thesis_sim::gate_behavior_mode_name(sim.gate_behavior())) << "\",\n";
    out << "  \"gate_seed\": " << sim.gate_seed() << ",\n";
    out << "  \"config\": {\n";
    out << "    \"dt\": " << config.dt << ",\n";
    out << "    \"control_interval_steps\": " << config.control_interval_steps << ",\n";
    out << "    \"imu_enabled\": " << (config.imu_enabled ? "true" : "false") << ",\n";
    out << "    \"lidar_enabled\": " << (config.lidar_enabled ? "true" : "false") << ",\n";
    out << "    \"dynamic_lidar_gates\": " << (config.dynamic_lidar_gates ? "true" : "false") << ",\n";
    out << "    \"simulation_level\": \""
        << (config.ideal_conditions
                ? "ideal"
                : (config.calibrated_conditions ? "calibrated" : "reference"))
        << "\",\n";
    out << "    \"gate_source\": \"" << (config.dynamic_lidar_gates ? "Simulated LiDAR dynamic gates" : "Static scenario gates") << "\",\n";
    out << "    \"range_sensor_profile\": \"" << json_escape(thesis_sim::range_sensor_profile_name(config.range_sensor_profile)) << "\",\n";
    out << "    \"lidar_beams\": " << sim.active_lidar_beams() << ",\n";
    out << "    \"lidar_fov_rad\": " << sim.active_lidar_fov_rad() << ",\n";
    out << "    \"lidar_range\": " << sim.active_lidar_range() << ",\n";
    out << "    \"vehicle_model\": \"" << json_escape(thesis_sim::vehicle_model_kind_name(sim.vehicle_model_kind())) << "\",\n";
    out << "    \"tracking_layer\": \"" << json_escape(thesis_sim::tracking_controller_mode_name(sim.tracking_controller_mode())) << "\",\n";
    out << "    \"calibrated_model\": {\n";
    out << "      \"enabled\": " << (config.calibrated_conditions ? "true" : "false") << ",\n";
    out << "      \"seed\": " << config.simulation_seed << ",\n";
    out << "      \"command_delay_steps\": " << config.command_delay_steps << ",\n";
    out << "      \"encoder_distance_noise_std_m\": " << config.encoder_distance_noise_std_m << ",\n";
    out << "      \"encoder_left_scale\": " << config.encoder_left_scale << ",\n";
    out << "      \"encoder_right_scale\": " << config.encoder_right_scale << ",\n";
    out << "      \"imu_yaw_noise_std_rad\": " << config.imu_yaw_noise_std_rad << ",\n";
    out << "      \"imu_yaw_rate_noise_std_rad_s\": " << config.imu_yaw_rate_noise_std_rad_s << ",\n";
    out << "      \"imu_yaw_bias_walk_std_rad\": " << config.imu_yaw_bias_walk_std_rad << ",\n";
    out << "      \"lidar_range_noise_std_m\": " << config.lidar_range_noise_std_m << ",\n";
    out << "      \"lidar_dropout_probability\": " << config.lidar_dropout_probability << ",\n";
    out << "      \"actuator_left_scale\": " << config.actuator_left_scale << ",\n";
    out << "      \"actuator_right_scale\": " << config.actuator_right_scale << ",\n";
    out << "      \"actuator_time_constant_scale\": " << config.actuator_time_constant_scale << "\n";
    out << "    },\n";
    out << "    \"vehicle_geometry\": {\n";
    out << "      \"wheelbase\": " << sim.geometry().wheelbase << ",\n";
    out << "      \"track\": " << sim.geometry().track << ",\n";
    out << "      \"body_length\": " << sim.geometry().body_length << ",\n";
    out << "      \"body_width\": " << sim.geometry().body_width << ",\n";
    out << "      \"max_linear_speed\": " << sim.geometry().max_linear_speed << ",\n";
    out << "      \"max_curvature\": " << sim.geometry().max_curvature << ",\n";
    out << "      \"max_steer_angle\": " << sim.geometry().max_steer_angle << ",\n";
    out << "      \"max_steer_rate\": " << sim.geometry().max_steer_rate << ",\n";
    out << "      \"yaw_response_scale\": " << sim.geometry().yaw_response_scale << "\n";
    out << "    }\n";
    out << "  },\n";
    out << "  \"scenario\": {\n";
    out << "    \"bounds\": ";
    write_rect_json(out, world.bounds());
    out << ",\n";
    out << "    \"start\": ";
    write_vec2_json(out, world.start());
    out << ",\n";
    out << "    \"goal\": ";
    write_vec2_json(out, world.goal());
    out << ",\n";
    out << "    \"start_heading_rad\": " << world.start_heading() << ",\n";
    out << "    \"obstacles\": [\n";
    for (size_t i = 0; i < world.obstacles().size(); ++i) {
        out << "      ";
        write_rect_json(out, world.obstacles()[i]);
        out << (i + 1 < world.obstacles().size() ? ",\n" : "\n");
    }
    out << "    ],\n";
    out << "    \"perception_obstacles\": [\n";
    for (size_t i = 0; i < world.perception_obstacles().size(); ++i) {
        out << "      ";
        write_rect_json(out, world.perception_obstacles()[i]);
        out << (i + 1 < world.perception_obstacles().size() ? ",\n" : "\n");
    }
    out << "    ],\n";
    out << "    \"gates\": [\n";
    for (size_t i = 0; i < world.gates().size(); ++i) {
        const GateSpec& gate = world.gates()[i];
        out << "      {\"name\":\"" << json_escape(gate.name) << "\",\"position\":";
        write_vec2_json(out, gate.position);
        out << ",\"anchor_position\":";
        write_vec2_json(out, gate.anchor_position);
        out << ",\"motion_amplitude\":";
        write_vec2_json(out, gate.motion_amplitude);
        out << ",\"motion_frequency_hz\":" << gate.motion_frequency_hz
            << ",\"motion_phase_rad\":" << gate.motion_phase_rad
            << ",\"heading_hint\":" << gate.heading_hint
            << ",\"final\":" << (gate.final ? "true" : "false") << "}";
        out << (i + 1 < world.gates().size() ? ",\n" : "\n");
    }
    out << "    ],\n";
    out << "    \"road_centerline\": [\n";
    for (size_t i = 0; i < world.road_centerline().size(); ++i) {
        out << "      ";
        write_vec2_json(out, world.road_centerline()[i]);
        out << (i + 1 < world.road_centerline().size() ? ",\n" : "\n");
    }
    out << "    ]\n";
    out << "  },\n";
    out << "  \"summary\": {\n";
    out << "    \"steps\": " << report.steps << ",\n";
    out << "    \"sim_time\": " << report.sim_time << ",\n";
    out << "    \"distance_to_goal\": " << report.distance_to_goal << ",\n";
    out << "    \"passed_gates\": " << report.passed_gates << ",\n";
    out << "    \"final_position\": ";
    write_vec2_json(out, report.final_position);
    out << ",\n";
    out << "    \"final_yaw_rad\": " << sim.vehicle().yaw << ",\n";
    out << "    \"final_speed\": " << sim.vehicle().speed << "\n";
    out << "  },\n";
    out << "  \"performance\": {\n";
    out << "    \"planning_ms\": {\"avg\": " << planning_summary.avg << ",\"max\": " << planning_summary.max << "},\n";
    out << "    \"tracking_ms\": {\"avg\": " << tracking_summary.avg << ",\"max\": " << tracking_summary.max << "},\n";
    out << "    \"lidar_ms\": {\"avg\": " << lidar_summary.avg << ",\"max\": " << lidar_summary.max << "},\n";
    out << "    \"estimator_ms\": {\"avg\": " << estimator_summary.avg << ",\"max\": " << estimator_summary.max << "},\n";
    out << "    \"step_ms\": {\"avg\": " << step_summary.avg << ",\"max\": " << step_summary.max << "}\n";
    out << "  },\n";
    out << "  \"telemetry\": [\n";
    bool first = true;
    for (size_t i = 0; i < history.size(); i += stride) {
        const TelemetrySample& sample = history[i];
        if (!first) {
            out << ",\n";
        }
        first = false;
        out << "    {"
            << "\"time\":" << sample.time
            << ",\"x\":" << sample.x
            << ",\"y\":" << sample.y
            << ",\"speed\":" << sample.speed
            << ",\"accel\":" << sample.accel
            << ",\"yaw\":" << sample.yaw
            << ",\"curvature\":" << sample.curvature
            << ",\"yaw_rate\":" << sample.yaw_rate
            << ",\"steer_angle\":" << sample.steer_angle
            << ",\"target_steer_angle\":" << sample.target_steer_angle
            << ",\"target_speed\":" << sample.target_speed
            << ",\"target_yaw_rate\":" << sample.target_yaw_rate
            << ",\"left_wheel_speed\":" << sample.left_wheel_speed
            << ",\"right_wheel_speed\":" << sample.right_wheel_speed
            << ",\"left_pwm\":" << sample.left_pwm
            << ",\"right_pwm\":" << sample.right_pwm
            << ",\"distance_to_goal\":" << sample.distance_to_goal
            << ",\"nav_xy_error\":" << sample.nav_xy_error
            << ",\"nav_yaw_error_deg\":" << sample.nav_yaw_error_deg
            << ",\"tracker_cross_track\":" << sample.tracker_cross_track
            << ",\"tracker_heading_error_deg\":" << sample.tracker_heading_error_deg
            << ",\"planning_ms\":" << sample.planning_ms
            << ",\"tracking_ms\":" << sample.tracking_ms
            << ",\"lidar_ms\":" << sample.lidar_ms
            << ",\"estimator_ms\":" << sample.estimator_ms
            << ",\"step_ms\":" << sample.step_ms
            << ",\"visible_gates\":" << sample.visible_gates
            << ",\"lidar_samples\":" << sample.lidar_samples
            << ",\"chosen_gate_index\":" << sample.chosen_gate_index
            << ",\"chosen_gate_distance\":" << sample.chosen_gate_distance
            << ",\"passed_gates\":" << sample.passed_gates
            << ",\"candidate_gates\":" << sample.candidate_gates
            << ",\"dynamic_gap_gates\":" << sample.dynamic_gap_gates
            << ",\"planner_has_reference\":" << sample.planner_has_reference
            << ",\"mixed_mode\":" << sample.mixed_mode
            << ",\"mixed_gate_score\":" << sample.mixed_gate_score
            << ",\"mixed_structured_score\":" << sample.mixed_structured_score
            << ",\"mixed_gate_confidence\":" << sample.mixed_gate_confidence
            << ",\"mixed_switches\":" << sample.mixed_switches
            << ",\"mixed_aborts\":" << sample.mixed_aborts
            << "}";
    }
    out << "\n  ]\n";
    out << "}\n";
    const bool json_ok = out.good();
    out.close();
    const std::string slam_path = append_stem_suffix(report_path, "_slam_reference", ".png");
    return json_ok && write_sim_slam_reference_png(sim, slam_path);
}

}  // namespace thesis_sim::mvc::controller
