#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <vector>

#include "mvc/controller/hardware_planner/runner.h"
#include "mvc/model/world/scenario_model.h"

namespace {

constexpr double kPi = 3.14159265358979323846;

int fail(const char* message) {
    std::cerr << "unstructured_exploration_smoke: " << message << '\n';
    return 1;
}

thesis_sim::HardwarePlannerConfig make_config() {
    thesis_sim::HardwarePlannerConfig config;
    config.nominal_dt = 0.10;
    config.mapping_lidar_enabled = true;
    config.start_matching.enabled = false;
    config.slam_toolbox_enabled = true;
    config.slam_pose_feedback_enabled = false;
    config.planner_safety_stop_enabled = false;
    config.localization.min_scan_points = 20;
    config.localization.min_valid_range_m = 0.04;
    config.localization.max_range_m = 1.20;
    config.localization.obstacle_stop_distance_m = 0.19;
    config.gap_extraction.startup_scan_duration_s = 0.30;
    config.gap_extraction.planning_max_range_m = 1.20;
    config.gap_extraction.free_distance_threshold_m = 0.24;
    config.gap_extraction.min_gap_depth_contrast_m = 0.070;
    config.gap_extraction.gap_aperture_target_margin_m = 0.080;
    config.gap_extraction.path_clearance_radius_m = 0.010;
    config.gap_extraction.target_clearance_radius_m = 0.095;
    config.gap_extraction.map_point_resolution_m = 0.022;
    config.gap_extraction.occupancy_confirm_hits = 1;
    config.gap_extraction.min_gap_width_m = 0.20;
    config.gap_extraction.min_target_distance_m = 0.18;
    config.gap_extraction.max_target_distance_m = 0.58;
    config.gap_extraction.gap_goal_tolerance_m = 0.070;
    config.gap_extraction.gap_goal_acceptance_radius_m = 0.105;
    config.gap_extraction.gap_goal_acceptance_lateral_slack_m = 0.060;
    config.gap_extraction.gap_crossing_margin_m = 0.018;
    config.gap_extraction.gap_goal_cruise_speed_mps = 0.060;
    config.frontier_exploration.enabled = true;
    config.frontier_exploration.grid_resolution_m = 0.04;
    config.frontier_exploration.minimum_frontier_distance_m = 0.10;
    config.frontier_exploration.maximum_frontier_distance_m = 0.55;
    config.frontier_exploration.path_lookahead_m = 0.20;
    config.frontier_exploration.target_reached_radius_m = 0.06;
    config.frontier_exploration.replan_interval_steps = 5;
    config.frontier_exploration.obstacle_inflation_m = 0.09;
    return config;
}

std::vector<thesis_sim::RPLidarA1::ScanPoint> make_scan(
    const thesis_sim::WorldMap& sensed_world,
    const thesis_sim::Vec2& position,
    double yaw,
    const thesis_sim::HardwarePlannerConfig& config) {
    constexpr int kBeams = 360;
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);
    const thesis_sim::Vec2 origin{
        position.x +
            cos_yaw * config.localization.lidar_x_offset -
            sin_yaw * config.localization.lidar_y_offset,
        position.y +
            sin_yaw * config.localization.lidar_x_offset +
            cos_yaw * config.localization.lidar_y_offset,
    };
    const std::vector<thesis_sim::LidarHit> hits = sensed_world.raycast(
        origin,
        yaw + config.localization.lidar_yaw_offset,
        kBeams,
        2.0 * kPi,
        config.localization.max_range_m,
        true);
    std::vector<thesis_sim::RPLidarA1::ScanPoint> scan;
    scan.reserve(hits.size());
    for (std::size_t i = 0; i < hits.size(); ++i) {
        const double alpha = hits.size() > 1
            ? static_cast<double>(i) / static_cast<double>(hits.size() - 1U)
            : 0.0;
        const double angle_deg = config.localization.lidar_flip_left_right
            ? 180.0 - 360.0 * alpha
            : -180.0 + 360.0 * alpha;
        const double angle_rad = angle_deg * kPi / 180.0;
        const double range_m = hits[i].distance;
        scan.push_back({
            static_cast<std::uint8_t>(
                range_m < config.localization.max_range_m - 1e-6 ? 15 : 0),
            angle_deg,
            range_m * 1000.0,
            range_m,
            std::cos(angle_rad) * range_m,
            std::sin(angle_rad) * range_m,
        });
    }
    return scan;
}

thesis_sim::RealRobotObservation make_observation(
    double timestamp,
    const std::vector<thesis_sim::RPLidarA1::ScanPoint>& scan) {
    thesis_sim::RealRobotObservation observation;
    observation.host_timestamp_s = timestamp;
    observation.have_controller_telemetry = true;
    observation.controller.ms =
        static_cast<std::uint32_t>(std::lround(timestamp * 1000.0));
    observation.controller.rx_timestamp_s = timestamp;
    observation.controller.imu_rx_timestamp_s = timestamp;
    observation.controller.safety_rx_timestamp_s = timestamp;
    observation.controller.encoder_rx_timestamp_s = timestamp;
    observation.controller.motor_rx_timestamp_s = timestamp;
    observation.controller.heartbeat_rx_timestamp_s = timestamp;
    observation.controller.imu_host_timestamp_s = timestamp;
    observation.controller.safety_host_timestamp_s = timestamp;
    observation.controller.encoder_host_timestamp_s = timestamp;
    observation.controller.motor_host_timestamp_s = timestamp;
    observation.controller.heartbeat_host_timestamp_s = timestamp;
    observation.controller.enc_dt_ms = 100;
    observation.controller.motor_flags =
        static_cast<std::uint16_t>(thesis_sim::MotorFlag::Enabled) |
        static_cast<std::uint16_t>(thesis_sim::MotorFlag::StbyHigh);
    observation.controller.status_flags =
        static_cast<std::uint16_t>(thesis_sim::StatusFlag::ImuReady) |
        static_cast<std::uint16_t>(thesis_sim::StatusFlag::EncodersReady) |
        static_cast<std::uint16_t>(thesis_sim::StatusFlag::MotorsReady) |
        static_cast<std::uint16_t>(thesis_sim::StatusFlag::HostLinkOk);
    observation.controller.enc_flags =
        static_cast<std::uint16_t>(thesis_sim::EncoderFlag::LeftValid) |
        static_cast<std::uint16_t>(thesis_sim::EncoderFlag::RightValid);
    observation.controller.have_imu = true;
    observation.controller.have_motor = true;
    observation.controller.have_encoder = true;
    observation.controller.have_heartbeat = true;
    observation.have_lidar_scan = !scan.empty();
    observation.lidar_scan = scan;
    observation.lidar_scan_start_timestamp_s = timestamp - 0.10;
    observation.lidar_scan_mid_timestamp_s = timestamp - 0.05;
    observation.lidar_scan_end_timestamp_s = timestamp;
    observation.lidar_scan_duration_s = 0.10;
    observation.lidar_scan_age_s = 0.0;
    return observation;
}

}  // namespace

int main() {
    thesis_sim::HardwarePlannerConfig config = make_config();
    const thesis_sim::WorldMap physical_fixture =
        thesis_sim::WorldMap::unstructured_demo(
            thesis_sim::UnstructuredMapPreset::HardwareLab,
            thesis_sim::GateBehaviorMode::Static,
            0);
    const thesis_sim::WorldMap exploration_world =
        thesis_sim::mvc::model::sanitize_hardware_unstructured_world(
            physical_fixture);
    thesis_sim::RealRobotBridge::Options bridge_options;

    thesis_sim::HardwarePlannerRunner runner(
        exploration_world, bridge_options, config);
    const std::vector<thesis_sim::RPLidarA1::ScanPoint> empty_arena_scan =
        make_scan(exploration_world, exploration_world.start(),
                  exploration_world.start_heading(), config);

    // A partial optimized map models the normal state after the initial
    // stationary scan: the observed corridor is free and its far edge is an
    // information frontier. Unknown cells beyond x=0.62 are deliberately not
    // transmitted by OccupancyGrid.
    thesis_sim::SlamToolboxSnapshot bootstrap_slam;
    bootstrap_slam.session_id = "bootstrap";
    bootstrap_slam.connected = true;
    bootstrap_slam.map_updates = 1;
    for (double x = 0.18; x <= 0.62; x += 0.04) {
        for (double y = 0.48; y <= 0.72; y += 0.04) {
            bootstrap_slam.free_points.push_back({x, y});
        }
        bootstrap_slam.occupied_points.push_back({x, 0.40});
        bootstrap_slam.occupied_points.push_back({x, 0.80});
    }
    runner.apply_slam_toolbox_snapshot(bootstrap_slam);

    bool saw_frontier = false;
    bool saw_gate_control = false;
    bool saw_recovery_arc = false;
    bool saw_forward_exploration_motion = false;
    bool saw_curved_exploration_command = false;
    for (int step = 0; step < 20; ++step) {
        runner.step_with_observation(
            make_observation(1.0 + 0.10 * step, empty_arena_scan),
            0.10,
            false);
        if (runner.last_command().pwm_left * runner.last_command().pwm_right < 0) {
            return fail("unstructured exploration issued opposing-wheel PWM");
        }
        if (step < 2 &&
            (runner.last_command().pwm_left != 0 ||
             runner.last_command().pwm_right != 0 ||
             std::abs(runner.last_command().target_yaw_rate) > 1e-9)) {
            return fail("initial observation phase issued a motion command");
        }
        saw_frontier = saw_frontier ||
            runner.diagnostics().selected_frontier_valid ||
            runner.diagnostics().control_source ==
                thesis_sim::HardwareControlSource::FrontierMpc;
        saw_gate_control = saw_gate_control ||
            runner.diagnostics().control_source ==
                thesis_sim::HardwareControlSource::GateMpc;
        saw_recovery_arc = saw_recovery_arc ||
            runner.diagnostics().control_source ==
                thesis_sim::HardwareControlSource::Recovery;
        if (runner.diagnostics().control_source ==
                thesis_sim::HardwareControlSource::FrontierMpc) {
            saw_forward_exploration_motion = saw_forward_exploration_motion ||
                (runner.last_command().target_speed > 1e-4 &&
                 runner.last_command().pwm_left > 0 &&
                 runner.last_command().pwm_right > 0);
            saw_curved_exploration_command = saw_curved_exploration_command ||
                std::abs(runner.last_command().target_yaw_rate) > 0.015;
        }
    }
    if (!saw_frontier) {
        return fail("HardwareLab did not transition from stationary scan to frontier exploration");
    }
    if (saw_gate_control) {
        return fail("a frontier target was misclassified as a physical gate");
    }
    if (saw_recovery_arc) {
        return fail("empty-arena exploration fell back to the legacy search arc");
    }
    if (!saw_forward_exploration_motion) {
        return fail("straight frontier exploration never issued a forward command");
    }
    if (saw_curved_exploration_command) {
        return fail("straight frontier exploration issued a curved command");
    }
    if (runner.global_free_points().empty() ||
        runner.global_occupied_points().empty()) {
        return fail("persistent exploration map was not populated");
    }

    // Real A1 reports often contain only 70--100 usable beams after filtering.
    // Reproduce that condition without a preloaded SLAM map: the persistent
    // local grid must still release INITIAL_SCAN and command straight motion.
    std::vector<thesis_sim::RPLidarA1::ScanPoint> sparse_scan;
    for (std::size_t i = 0; i < empty_arena_scan.size(); i += 4U) {
        sparse_scan.push_back(empty_arena_scan[i]);
    }
    thesis_sim::HardwarePlannerRunner sparse_runner(
        exploration_world, bridge_options, config);
    bool sparse_runner_moved = false;
    for (int step = 0; step < 35; ++step) {
        sparse_runner.step_with_observation(
            make_observation(6.0 + 0.10 * step, sparse_scan), 0.10, false);
        const auto& command = sparse_runner.last_command();
        if (command.pwm_left * command.pwm_right < 0) {
            return fail("sparse LiDAR exploration issued opposing-wheel PWM");
        }
        sparse_runner_moved = sparse_runner_moved ||
            (command.target_speed > 1e-4 &&
             command.pwm_left > 0 && command.pwm_right > 0);
    }
    if (!sparse_runner_moved) {
        return fail("sparse LiDAR exploration remained stuck in INITIAL_SCAN");
    }

    bootstrap_slam.connected = false;
    runner.apply_slam_toolbox_snapshot(bootstrap_slam);
    const std::size_t persistent_occupied =
        runner.global_occupied_points().size();
    if (persistent_occupied == 0U) {
        return fail("local persistent map was not available after SLAM fallback");
    }
    for (int step = 0; step < 5; ++step) {
        runner.step_with_observation(
            make_observation(4.0 + 0.10 * step, {}), 0.10, false);
    }
    if (runner.global_occupied_points().size() != persistent_occupied) {
        return fail("global fallback map decayed when scans stopped");
    }

    thesis_sim::SlamToolboxSnapshot slam_snapshot;
    slam_snapshot.session_id = "smoke_session";
    slam_snapshot.connected = true;
    slam_snapshot.map_updates = 3;
    slam_snapshot.free_points = {{0.30, 0.30}, {0.34, 0.30}};
    slam_snapshot.occupied_points = {{0.42, 0.42}};
    slam_snapshot.status = "smoke";
    runner.apply_slam_toolbox_snapshot(slam_snapshot);
    if (runner.diagnostics().map_source !=
            thesis_sim::ExplorationMapSource::SlamToolbox ||
        runner.global_occupied_points().size() != 1U) {
        return fail("connected SLAM Toolbox map did not preempt the fallback grid");
    }
    slam_snapshot.connected = false;
    runner.apply_slam_toolbox_snapshot(slam_snapshot);
    if (runner.diagnostics().map_source !=
            thesis_sim::ExplorationMapSource::LocalPersistentGrid ||
        runner.global_occupied_points().size() != persistent_occupied) {
        return fail("SLAM disconnect did not restore the persistent fallback grid");
    }

    thesis_sim::HardwarePlannerConfig blocked_config = config;
    blocked_config.start_matching.enabled = true;
    blocked_config.start_matching.reference_path =
        "/definitely/missing/start-reference.csv";
    thesis_sim::HardwarePlannerRunner blocked_runner(
        exploration_world, bridge_options, blocked_config);
    blocked_runner.step_with_observation(
        make_observation(1.0, empty_arena_scan), 0.10, false);
    if (blocked_runner.start_matching_status().accepted ||
        blocked_runner.last_command().pwm_left != 0 ||
        blocked_runner.last_command().pwm_right != 0 ||
        blocked_runner.diagnostics().exploration_state !=
            thesis_sim::UnstructuredExplorationState::StartMatching) {
        return fail("missing start reference did not fail closed");
    }
    blocked_config.start_matching.enabled = false;
    blocked_runner.apply_config(blocked_config);
    if (!blocked_runner.start_matching_status().accepted ||
        blocked_runner.start_matching_status().status !=
            "disabled_by_runtime_setting") {
        return fail("debug start-matching toggle did not release the interlock");
    }

    // Feed the same planner a scan from the physical two-wall gate fixture.
    // A confirmed aperture must replace any active frontier and become the
    // clothoid/MPC control source.
    thesis_sim::HardwarePlannerRunner gate_runner(
        exploration_world, bridge_options, config);
    const std::vector<thesis_sim::RPLidarA1::ScanPoint> gate_scan =
        make_scan(physical_fixture, exploration_world.start(),
                  exploration_world.start_heading(), config);
    bool saw_physical_gate = false;
    int maximum_gate_candidates = 0;
    int maximum_reference_points = 0;
    int maximum_chosen_gate = -1;
    for (int step = 0; step < 35; ++step) {
        gate_runner.step_with_observation(
            make_observation(1.0 + 0.10 * step, gate_scan), 0.10, false);
        if (gate_runner.last_command().pwm_left *
                gate_runner.last_command().pwm_right < 0) {
            return fail("unstructured gate tracking issued opposing-wheel PWM");
        }
        saw_physical_gate = saw_physical_gate ||
            gate_runner.diagnostics().control_source ==
                thesis_sim::HardwareControlSource::GateMpc;
        maximum_gate_candidates = std::max(
            maximum_gate_candidates,
            gate_runner.diagnostics().candidate_gates);
        maximum_reference_points = std::max(
            maximum_reference_points,
            static_cast<int>(gate_runner.reference_trajectory().size()));
        maximum_chosen_gate = std::max(
            maximum_chosen_gate,
            gate_runner.chosen_gate_index());
    }
    if (!saw_physical_gate) {
        std::cerr << "gate_debug candidates=" << maximum_gate_candidates
                  << " state="
                  << thesis_sim::unstructured_exploration_state_name(
                         gate_runner.diagnostics().exploration_state)
                  << " control="
                  << thesis_sim::hardware_control_source_name(
                         gate_runner.diagnostics().control_source)
                  << " map_free=" << gate_runner.global_free_points().size()
                  << " map_occupied=" << gate_runner.global_occupied_points().size()
                  << " max_reference=" << maximum_reference_points
                  << " max_chosen=" << maximum_chosen_gate
                  << " invalidation='"
                  << gate_runner.diagnostics().reference_invalidation_reason
                  << "'"
                  << " gate_target="
                  << (gate_runner.gate_specs().empty()
                          ? -1.0
                          : gate_runner.gate_specs().front().position.x)
                  << ','
                  << (gate_runner.gate_specs().empty()
                          ? -1.0
                          : gate_runner.gate_specs().front().position.y)
                  << '\n';
        return fail("LiDAR-confirmed gate did not preempt frontier navigation");
    }

    // The closed mixed hardware road used in the reported run may legitimately
    // contain curved road/clothoid references, but a car-like controller must
    // never replace them with an in-place differential recovery command.
    thesis_sim::HardwarePlannerRunner mixed_runner(
        thesis_sim::WorldMap::mixed_closed_obstacle_hardware_demo(),
        bridge_options,
        config);
    const thesis_sim::WorldMap mixed_fixture =
        thesis_sim::WorldMap::mixed_closed_obstacle_hardware_demo();
    const auto mixed_scan = make_scan(
        mixed_fixture,
        mixed_fixture.start(),
        mixed_fixture.start_heading(),
        config);
    for (int step = 0; step < 40; ++step) {
        mixed_runner.step_with_observation(
            make_observation(10.0 + 0.10 * step, mixed_scan), 0.10, false);
        const auto& command = mixed_runner.last_command();
        if ((command.pwm_left < 0 && command.pwm_right > 0) ||
            (command.pwm_left > 0 && command.pwm_right < 0)) {
            return fail("car-like mixed mode issued opposing-wheel recovery PWM");
        }
        if (std::abs(command.target_speed) <= 1e-4 &&
            std::abs(command.target_yaw_rate) > 1e-4) {
            return fail("car-like mixed mode issued an in-place yaw command");
        }
    }

    std::cout << "unstructured_exploration_smoke: ok"
              << " fallback_occupied=" << persistent_occupied
              << " state="
              << thesis_sim::unstructured_exploration_state_name(
                     runner.diagnostics().exploration_state)
              << '\n';
    return 0;
}
