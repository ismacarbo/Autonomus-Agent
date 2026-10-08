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
    config.pwm.left_command_scale = 1.0;
    config.pwm.right_command_scale = 0.8494623655913978;
    config.pwm.left_command_offset = 0.0;
    config.pwm.right_command_offset = 26.021505376344084;
    config.pwm.yaw_rate_tracking_kp = 0.45;
    config.pwm.yaw_rate_tracking_ki = 0.15;
    config.pwm.yaw_rate_error_integral_limit = 0.25;
    config.pwm.yaw_rate_feedback_filter_tau_s = 1.00;
    config.pwm.yaw_rate_feedback_deadband_rad_s = 0.012;
    config.pwm.yaw_rate_feedback_correction_limit_rad_s = 0.22;
    config.pwm.yaw_rate_target_slew_rate_rad_s2 = 1.80;
    config.pwm.yaw_rate_sign_preservation_threshold_rad_s = 0.020;
    config.pwm.gate_positive_turn_max_pwm_delta = 70;
    config.pwm.gate_negative_turn_max_pwm_delta = 40;
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

std::vector<thesis_sim::RPLidarA1::ScanPoint> make_false_wide_sector_scan(
    const thesis_sim::HardwarePlannerConfig& config) {
    constexpr int kBeams = 360;
    std::vector<thesis_sim::RPLidarA1::ScanPoint> scan;
    scan.reserve(kBeams);
    for (int i = 0; i < kBeams; ++i) {
        const double local_angle_deg =
            -180.0 + static_cast<double>(i) * 360.0 /
                         static_cast<double>(kBeams - 1);
        // Endpoints at +/-80 degrees and 33 cm are about 65 cm apart, but
        // their midpoint is only 5.7 cm in front of the sensor. This is the
        // same degeneracy that generated the false crossing in report 211728.
        const double range_m = std::abs(local_angle_deg) < 80.0 ? 1.0 : 0.33;
        const double sensor_angle_deg = config.localization.lidar_flip_left_right
            ? -(local_angle_deg -
                config.localization.lidar_yaw_offset * 180.0 / kPi)
            : local_angle_deg -
                  config.localization.lidar_yaw_offset * 180.0 / kPi;
        const double sensor_angle_rad = sensor_angle_deg * kPi / 180.0;
        scan.push_back({
            15,
            sensor_angle_deg,
            range_m * 1000.0,
            range_m,
            std::cos(sensor_angle_rad) * range_m,
            std::sin(sensor_angle_rad) * range_m,
        });
    }
    return scan;
}

thesis_sim::RealRobotObservation make_observation(
    double timestamp,
    const std::vector<thesis_sim::RPLidarA1::ScanPoint>& scan,
    std::int32_t left_encoder_ticks = 0,
    std::int32_t right_encoder_ticks = 0,
    double imu_yaw = 0.0,
    double imu_yaw_rate = 0.0) {
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
    observation.controller.encoder_ms = observation.controller.ms;
    observation.controller.enc_dt_ms = 100;
    observation.controller.imu_ms =
        static_cast<std::uint32_t>(std::lround(timestamp * 1000.0));
    observation.controller.yaw_mrad =
        static_cast<std::int32_t>(std::lround(imu_yaw * 1000.0));
    observation.controller.yaw_rate_mrad_s =
        static_cast<std::int32_t>(std::lround(imu_yaw_rate * 1000.0));
    observation.controller.ticks_left = left_encoder_ticks;
    observation.controller.ticks_right = right_encoder_ticks;
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

    // Keep a partial optimized map connected. In open-ended exploration it
    // must remain available for visualization without becoming a navigation
    // target: motion is driven by the live LiDAR/gate pipeline.
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

    bool saw_frontier_control = false;
    bool saw_gate_control = false;
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
        saw_frontier_control = saw_frontier_control ||
            runner.diagnostics().control_source ==
                thesis_sim::HardwareControlSource::FrontierMpc;
        saw_gate_control = saw_gate_control ||
            runner.diagnostics().control_source ==
                thesis_sim::HardwareControlSource::GateMpc;
        if (runner.diagnostics().control_source ==
                thesis_sim::HardwareControlSource::StraightExploration) {
            saw_forward_exploration_motion = saw_forward_exploration_motion ||
                (runner.last_command().target_speed > 1e-4 &&
                 runner.last_command().pwm_left > 0 &&
                 runner.last_command().pwm_right > 0);
            saw_curved_exploration_command = saw_curved_exploration_command ||
                std::abs(runner.last_command().target_yaw_rate) > 0.015;
        }
    }
    if (saw_frontier_control || runner.diagnostics().selected_frontier_valid) {
        return fail("open-ended exploration incorrectly used a SLAM frontier as a motion target");
    }
    if (saw_gate_control) {
        return fail("a frontier target was misclassified as a physical gate");
    }
    if (!saw_forward_exploration_motion) {
        return fail("live LiDAR exploration did not move from the initial pose");
    }
    if (saw_curved_exploration_command) {
        return fail("open-corridor LiDAR exploration issued a curved command");
    }
    if (runner.global_free_points().empty() ||
        runner.global_occupied_points().empty()) {
        return fail("persistent exploration map was not populated");
    }

    // Regression for the former 1.20 m virtual geofence. Even if an old GUI
    // payload still carries compact bounds, unstructured hardware exploration
    // must keep moving and accumulating free space beyond that rectangle. The
    // bounds are a canvas, while safety comes from the live scan.
    thesis_sim::WorldMap legacy_canvas_world = physical_fixture;
    legacy_canvas_world.editable_obstacles().clear();
    legacy_canvas_world.editable_perception_obstacles().clear();
    legacy_canvas_world.editable_gates().clear();
    legacy_canvas_world.set_bounds({0.0, 0.0, 1.20, 1.20});
    legacy_canvas_world.set_start({1.16, 0.60});
    legacy_canvas_world.set_goal({0.20, 0.60});
    legacy_canvas_world.set_start_heading(0.0);
    legacy_canvas_world.finalize_editor_changes();
    thesis_sim::WorldMap open_sensing_world = legacy_canvas_world;
    open_sensing_world.set_bounds({-2.0, -2.0, 4.0, 3.0});
    thesis_sim::HardwarePlannerRunner legacy_canvas_runner(
        legacy_canvas_world, bridge_options, config);
    const auto open_field_scan = make_scan(
        open_sensing_world,
        legacy_canvas_world.start(),
        legacy_canvas_world.start_heading(),
        config);
    bool moved_beyond_legacy_canvas = false;
    for (int step = 0; step < 12; ++step) {
        legacy_canvas_runner.step_with_observation(
            make_observation(3.0 + 0.10 * step, open_field_scan),
            0.10,
            false);
        moved_beyond_legacy_canvas = moved_beyond_legacy_canvas ||
            (legacy_canvas_runner.last_command().target_speed > 1e-4 &&
             !legacy_canvas_runner.safety_stop_active());
    }
    if (!moved_beyond_legacy_canvas) {
        return fail("legacy 1.20 m canvas still stopped open-ended exploration");
    }
    const bool mapped_beyond_legacy_canvas = std::any_of(
        legacy_canvas_runner.global_free_points().begin(),
        legacy_canvas_runner.global_free_points().end(),
        [](const thesis_sim::Vec2& point) { return point.x > 1.20; });
    if (!mapped_beyond_legacy_canvas) {
        return fail("persistent LiDAR map was still clipped at the legacy 1.20 m canvas");
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

    // A map frontier/clothoid is an optimization, not a prerequisite for
    // exploration. Force every possible frontier below the configured minimum
    // and verify that a fresh, free LiDAR corridor still produces a purely
    // straight command until a physical gate is found.
    thesis_sim::HardwarePlannerConfig direct_config = config;
    direct_config.frontier_exploration.minimum_frontier_distance_m = 2.0;
    direct_config.frontier_exploration.maximum_frontier_distance_m = 2.0;
    thesis_sim::HardwarePlannerRunner direct_runner(
        exploration_world, bridge_options, direct_config);
    bool saw_direct_straight_exploration = false;
    int consecutive_stationary_exploration_steps = 0;
    int maximum_stationary_exploration_steps = 0;
    for (int step = 0; step < 35; ++step) {
        direct_runner.step_with_observation(
            make_observation(10.0 + 0.10 * step, sparse_scan), 0.10, false);
        if (!direct_runner.safety_stop_active() &&
            std::abs(direct_runner.last_command().target_speed) <= 1e-4) {
            ++consecutive_stationary_exploration_steps;
            maximum_stationary_exploration_steps = std::max(
                maximum_stationary_exploration_steps,
                consecutive_stationary_exploration_steps);
        } else {
            consecutive_stationary_exploration_steps = 0;
        }
        if (direct_runner.diagnostics().control_source !=
            thesis_sim::HardwareControlSource::StraightExploration) {
            continue;
        }
        saw_direct_straight_exploration = true;
        const auto& command = direct_runner.last_command();
        if (direct_runner.diagnostics().planner_has_reference ||
            !direct_runner.reference_trajectory().empty()) {
            return fail("straight LiDAR fallback unexpectedly required a clothoid");
        }
        if (command.target_speed <= 1e-4 ||
            command.pwm_left <= 0 ||
            command.pwm_right <= 0) {
            return fail("no-clothoid LiDAR exploration did not move forward");
        }
        if (std::abs(command.target_curvature) > 1e-9 ||
            std::abs(command.target_yaw_rate) > 1e-9) {
            return fail("no-clothoid LiDAR exploration issued a curved command");
        }
    }
    if (!saw_direct_straight_exploration) {
        return fail("no-clothoid LiDAR corridor remained in HOLD");
    }
    if (maximum_stationary_exploration_steps > 1) {
        return fail("open-ended no-gate exploration stopped without a safety condition");
    }

    // With no gate the nominal request remains straight. If the BNO080 sees
    // the measured negative-yaw drift from the 2026-08-24 ground test, the
    // outer loop must generate a positive correction while preserving two
    // forward wheel commands. This is actuator compensation, not a planned
    // search arc.
    thesis_sim::HardwarePlannerRunner yaw_correction_runner(
        exploration_world, bridge_options, direct_config);
    bool saw_body_yaw_correction = false;
    double maximum_body_yaw_correction = 0.0;
    for (int step = 0; step < 35; ++step) {
        yaw_correction_runner.step_with_observation(
            make_observation(
                14.0 + 0.10 * step,
                sparse_scan,
                step * 2,
                step * 2,
                -0.014 * step,
                -0.14),
            0.10,
            false);
        const auto& command = yaw_correction_runner.last_command();
        if (command.pwm_left < 0 || command.pwm_right < 0) {
            return fail("body-yaw correction violated forward-only exploration");
        }
        maximum_body_yaw_correction = std::max(
            maximum_body_yaw_correction,
            command.target_yaw_rate);
        saw_body_yaw_correction = saw_body_yaw_correction ||
            (yaw_correction_runner.diagnostics().control_source ==
                 thesis_sim::HardwareControlSource::StraightExploration &&
             command.target_yaw_rate > 0.015 &&
             command.pwm_right > command.pwm_left);
    }
    if (!saw_body_yaw_correction) {
        std::cerr << "yaw_correction_debug max="
                  << maximum_body_yaw_correction
                  << " estimate=" << yaw_correction_runner.estimate().yaw_rate
                  << " feedback="
                  << yaw_correction_runner.last_command().yaw_rate_feedback_measurement
                  << " planner="
                  << yaw_correction_runner.last_command().planner_target_yaw_rate
                  << " speed=" << yaw_correction_runner.last_command().target_speed
                  << " source="
                  << thesis_sim::hardware_control_source_name(
                         yaw_correction_runner.diagnostics().control_source)
                  << '\n';
        return fail("negative IMU drift did not produce a positive body-yaw correction");
    }
    if (yaw_correction_runner.diagnostics().imu_yaw_rate_fallback_active) {
        return fail("coherent IMU yaw and gyro incorrectly activated the yaw-rate fallback");
    }

    // A one-tick difference at this encoder resolution is ~0.42 rad/s.
    // It must not reverse the response to independently coherent body motion.
    for (double body_rate : {-0.14, 0.14, 0.0}) {
        thesis_sim::HardwarePlannerRunner angular_runner(
            exploration_world, bridge_options, direct_config);
        for (int step = 0; step < 35; ++step) {
            angular_runner.step_with_observation(
                make_observation(40.0 + 0.1 * step, sparse_scan,
                                 step * (body_rate > 0.0 ? 4 : 3),
                                 step * (body_rate > 0.0 ? 3 : 4),
                                 body_rate * 0.1 * step,
                                 body_rate == 0.0 ? 0.65 : body_rate),
                0.1, false);
        }
        const auto& command = angular_runner.last_command();
        const auto& sample = angular_runner.history().back();
        if (std::abs(angular_runner.estimate().yaw_rate - body_rate) > 0.02 ||
            std::abs(command.yaw_rate_feedback_measurement - body_rate) > 0.02) {
            return fail("encoder disagreement contaminated the inertial body rate");
        }
        if (body_rate == 0.0) {
            if (sample.body_yaw_rate_source != 2.0 ||
                std::abs(command.target_yaw_rate) > 0.015) {
                return fail("yaw fallback with unequal encoder ticks still manufactured a turn");
            }
        } else if (sample.body_yaw_rate_source != 1.0 ||
                   command.target_yaw_rate * body_rate >= 0.0) {
            return fail("coherent body drift was corrected in the wrong direction");
        }
    }

    // Cumulative ticks and elapsed MCU time must describe the same interval,
    // even when the host consumes two or three wire packets at once.
    const double meters_per_tick = 2.0 * kPi * config.drive.wheel_radius /
                                  config.drive.encoder_ticks_per_revolution;
    for (int packet_factor : {1, 2, 3}) {
        thesis_sim::HardwarePlannerRunner timing_runner(
            exploration_world, bridge_options, direct_config);
        for (int step = 0; step < 12; ++step) {
            const auto before = timing_runner.estimate().position;
            timing_runner.step_with_observation(
                make_observation(50.0 + step * packet_factor * 0.1, {},
                                 step * 2 * packet_factor, step * 2 * packet_factor),
                packet_factor * 0.1, false);
            if (step > 4 &&
                (std::abs(timing_runner.estimate().speed - 20.0 * meters_per_tick) > 1e-6 ||
                 std::abs(std::hypot(timing_runner.estimate().position.x - before.x,
                                    timing_runner.estimate().position.y - before.y) -
                          2.0 * packet_factor * meters_per_tick) > 1e-6 ||
                 timing_runner.history().back().controller_encoder_dt_ms != 100.0 * packet_factor)) {
                std::cerr << "timing factor=" << packet_factor << " step=" << step
                          << " speed=" << timing_runner.estimate().speed
                          << " expected=" << 20.0 * meters_per_tick
                          << " delta=" << std::hypot(timing_runner.estimate().position.x - before.x,
                                                     timing_runner.estimate().position.y - before.y)
                          << " dt=" << timing_runner.history().back().controller_encoder_dt_ms << '\n';
                return fail("skipped encoder packets changed physical speed or distance");
            }
        }
    }

    for (int packet_factor : {1, 2}) {
        thesis_sim::HardwarePlannerRunner structured_timing_runner(
            thesis_sim::WorldMap::structured_demo(), bridge_options, direct_config);
        for (int step = 0; step < 12; ++step) {
            structured_timing_runner.step_with_observation(
                make_observation(60.0 + 0.1 * packet_factor * step, {},
                                 2 * packet_factor * step, 2 * packet_factor * step),
                0.1 * packet_factor, false);
        }
        if (std::abs(structured_timing_runner.estimate().speed - 20.0 * meters_per_tick) > 1e-6) {
            return fail("structured odometry still used the last wire packet duration");
        }
    }

    // Cached packets are not new zero-motion samples. MCU restart and millis/
    // signed counter rollover must never create a fictitious displacement.
    thesis_sim::HardwarePlannerRunner timestamp_runner(
        exploration_world, bridge_options, direct_config);
    auto last_timestamp_observation = make_observation(70.0, {});
    for (int step = 0; step < 8; ++step) {
        last_timestamp_observation = make_observation(70.0 + 0.1 * step, {}, 2 * step, 2 * step);
        timestamp_runner.step_with_observation(last_timestamp_observation, 0.1, false);
    }
    const auto before_duplicate = timestamp_runner.estimate().position;
    last_timestamp_observation.host_timestamp_s += 0.02;
    timestamp_runner.step_with_observation(last_timestamp_observation, 0.02, false);
    if (std::hypot(timestamp_runner.estimate().position.x - before_duplicate.x,
                   timestamp_runner.estimate().position.y - before_duplicate.y) > 1e-9 ||
        timestamp_runner.history().back().encoder_sample_new != 0.0 ||
        timestamp_runner.history().back().imu_sample_new != 0.0) {
        return fail("cached telemetry was integrated twice");
    }
    auto restarted = make_observation(70.8, {});
    restarted.controller.encoder_ms = 1U;
    timestamp_runner.step_with_observation(restarted, 0.1, false);
    if (std::hypot(timestamp_runner.estimate().position.x - before_duplicate.x,
                   timestamp_runner.estimate().position.y - before_duplicate.y) > 1e-9 ||
        timestamp_runner.history().back().encoder_sample_valid != 0.0) {
        return fail("MCU reset injected an encoder displacement");
    }
    for (int step = 1; step <= 5; ++step) {
        restarted = make_observation(70.8 + 0.1 * step, {}, 2 * step, 2 * step);
        restarted.controller.encoder_ms = 1U + 100U * step;
        timestamp_runner.step_with_observation(restarted, 0.1, false);
    }
    if (std::abs(timestamp_runner.estimate().speed - 20.0 * meters_per_tick) > 1e-6) {
        return fail("encoder odometry did not recover after an MCU restart");
    }
    thesis_sim::HardwarePlannerRunner rollover_runner(
        exploration_world, bridge_options, direct_config);
    for (int step = 0; step < 8; ++step) {
        const std::int64_t total = 2147483642LL + 2 * step;
        const std::int32_t ticks = static_cast<std::int32_t>(
            total > 2147483647LL ? total - 4294967296LL : total);
        auto obs = make_observation(80.0 + 0.1 * step, {}, ticks, ticks);
        obs.controller.encoder_ms = 0xffffff00U + 100U * step;
        rollover_runner.step_with_observation(obs, 0.1, false);
    }
    if (std::abs(rollover_runner.estimate().speed - 20.0 * meters_per_tick) > 1e-6) {
        return fail("counter rollover corrupted the encoder interval");
    }

    // A one-frame-delayed BNO rate can alternate around zero while the car is
    // mechanically settling. The filtered actuator trim must not turn that
    // measurement noise into alternating left/right exploration commands.
    thesis_sim::HardwarePlannerRunner alternating_yaw_runner(
        exploration_world, bridge_options, direct_config);
    int alternating_trim_steps = 0;
    for (int step = 0; step < 35; ++step) {
        const double alternating_rate = (step % 2 == 0) ? 0.65 : -0.65;
        alternating_yaw_runner.step_with_observation(
            make_observation(
                18.0 + 0.10 * step,
                sparse_scan,
                step * 2,
                step * 2,
                0.0,
                alternating_rate),
            0.10,
            false);
        const auto& command = alternating_yaw_runner.last_command();
        if (alternating_yaw_runner.diagnostics().control_source ==
                thesis_sim::HardwareControlSource::StraightExploration &&
            std::abs(command.target_yaw_rate) > 0.015) {
            ++alternating_trim_steps;
        }
    }
    if (alternating_trim_steps > 1) {
        return fail("alternating IMU rate noise still generated alternating exploration turns");
    }
    if (!alternating_yaw_runner.diagnostics().imu_yaw_rate_fallback_active) {
        return fail("yaw/gyro disagreement did not activate the yaw-derived rate fallback");
    }
    if (std::abs(alternating_yaw_runner.last_command().yaw_rate_feedback_measurement) > 0.08) {
        return fail("incoherent gyro remained authoritative after fallback activation");
    }

    // Once the host timestamp says that an IMU sample is stale, neither its
    // cached yaw nor its cached gyro may be applied to the EKF again.
    thesis_sim::HardwarePlannerRunner stale_imu_runner(
        exploration_world, bridge_options, direct_config);
    for (int step = 0; step < 8; ++step) {
        stale_imu_runner.step_with_observation(
            make_observation(
                20.0 + 0.10 * step,
                sparse_scan,
                step * 2,
                step * 2,
                0.0,
                0.0),
            0.10,
            false);
    }
    const double yaw_before_stale_imu = stale_imu_runner.estimate().yaw;
    auto stale_imu_observation = make_observation(
        20.8,
        sparse_scan,
        16,
        16,
        1.20,
        1.00);
    stale_imu_observation.controller.imu_host_timestamp_s = 19.0;
    stale_imu_runner.step_with_observation(stale_imu_observation, 0.10, false);
    const double stale_imu_yaw_delta = std::abs(std::atan2(
        std::sin(stale_imu_runner.estimate().yaw - yaw_before_stale_imu),
        std::cos(stale_imu_runner.estimate().yaw - yaw_before_stale_imu)));
    if (!stale_imu_runner.diagnostics().imu_yaw_rate_fallback_active ||
        stale_imu_yaw_delta > 0.12) {
        return fail("stale cached IMU data was still applied to the pose estimator");
    }

    if (stale_imu_runner.history().back().body_yaw_rate_source != 0.0) {
        return fail("stale IMU was still advertised as valid body-rate feedback");
    }
    auto missing_imu = make_observation(20.9, sparse_scan, 18, 18, 1.2, 1.0);
    missing_imu.controller.have_imu = false;
    stale_imu_runner.step_with_observation(missing_imu, 0.1, false);
    if (stale_imu_runner.history().back().body_yaw_rate_source != 0.0 ||
        stale_imu_runner.history().back().imu_sample_new != 0.0) {
        return fail("absent IMU was accepted through a fresh controller timestamp");
    }

    // If one encoder advances and the other stays at zero, briefly hold the
    // healthy side while pulsing the stalled side. Otherwise the healthy side
    // drags the chassis into the long arc seen at the end of report 224457.
    thesis_sim::HardwarePlannerRunner one_wheel_stall_runner(
        exploration_world, bridge_options, direct_config);
    bool saw_isolated_right_breakaway = false;
    for (int step = 0; step < 24; ++step) {
        one_wheel_stall_runner.step_with_observation(
            make_observation(
                21.5 + 0.10 * step,
                sparse_scan,
                step * 3,
                0,
                0.0,
                0.0),
            0.10,
            false);
        const auto& command = one_wheel_stall_runner.last_command();
        saw_isolated_right_breakaway = saw_isolated_right_breakaway ||
            (one_wheel_stall_runner.diagnostics().stall_boost_active &&
             command.target_right_wheel_speed_mps > 0.010 &&
             command.pwm_left == 0 &&
             command.pwm_right >= direct_config.pwm.start_motion_pwm);
    }
    if (!saw_isolated_right_breakaway) {
        return fail("stalled right wheel was not pulsed without dragging the healthy left wheel");
    }

    const double right_stall_samples = one_wheel_stall_runner.history().back().right_wheel_stall_cycles;
    auto duplicate_stall = make_observation(23.8, sparse_scan, 23 * 3, 0, 0.0, 0.0);
    duplicate_stall.host_timestamp_s += 0.02;
    one_wheel_stall_runner.step_with_observation(duplicate_stall, 0.02, false);
    if (one_wheel_stall_runner.history().back().right_wheel_stall_cycles != right_stall_samples) {
        return fail("cached encoder packet altered the single-wheel stall streak");
    }

    // Reproduce the final pose of hardware report 20260818_212510_695. The
    // previous controller held here despite 1.20 m frontal clearance because
    // it had no map frontier. The oriented future footprint still fits in the
    // 1.20 m arena and must allow a short forward exploration command.
    thesis_sim::WorldMap reported_pose_world = exploration_world;
    reported_pose_world.set_start({0.840404, 0.518557});
    reported_pose_world.set_start_heading(-0.188000);
    thesis_sim::HardwarePlannerRunner reported_pose_runner(
        reported_pose_world, bridge_options, direct_config);
    const auto reported_pose_scan = make_scan(
        reported_pose_world,
        reported_pose_world.start(),
        reported_pose_world.start_heading(),
        direct_config);
    bool reported_pose_kept_exploring = false;
    for (int step = 0; step < 20; ++step) {
        reported_pose_runner.step_with_observation(
            make_observation(20.0 + 0.10 * step, reported_pose_scan),
            0.10,
            false);
        const auto& command = reported_pose_runner.last_command();
        const auto source = reported_pose_runner.diagnostics().control_source;
        reported_pose_kept_exploring = reported_pose_kept_exploring ||
            ((source == thesis_sim::HardwareControlSource::StraightExploration ||
              source == thesis_sim::HardwareControlSource::ForwardSearch) &&
             command.target_speed > 1e-4 &&
             command.pwm_left > 0 &&
             command.pwm_right > 0 &&
             !reported_pose_runner.diagnostics().planner_has_reference);
    }
    if (!reported_pose_kept_exploring) {
        std::cerr << "reported_pose_debug state="
                  << thesis_sim::unstructured_exploration_state_name(
                         reported_pose_runner.diagnostics().exploration_state)
                  << " control="
                  << thesis_sim::hardware_control_source_name(
                         reported_pose_runner.diagnostics().control_source)
                  << " speed=" << reported_pose_runner.last_command().target_speed
                  << " yaw_rate=" << reported_pose_runner.last_command().target_yaw_rate
                  << " pwm=" << reported_pose_runner.last_command().pwm_left
                  << ',' << reported_pose_runner.last_command().pwm_right
                  << " front=" << reported_pose_runner.estimate().front_lidar_distance
                  << " min=" << reported_pose_runner.estimate().min_lidar_distance
                  << '\n';
        return fail("reported no-clothoid pose still remained in HOLD");
    }

    // Exact terminal pose of report 20260819_215354_454. A temporary loss of
    // candidates/map readiness must not reset open-ended mode to a stationary
    // INITIAL_SCAN loop; current LiDAR must continue to select forward search
    // (or a safe reverse at a true geometric dead end).
    thesis_sim::WorldMap no_gate_hold_world = exploration_world;
    no_gate_hold_world.set_start({0.901537, 0.444385});
    no_gate_hold_world.set_start_heading(-0.377000);
    thesis_sim::HardwarePlannerRunner no_gate_hold_runner(
        no_gate_hold_world, bridge_options, direct_config);
    const auto no_gate_hold_scan = make_scan(
        no_gate_hold_world,
        no_gate_hold_world.start(),
        no_gate_hold_world.start_heading(),
        direct_config);
    int consecutive_no_gate_holds = 0;
    int maximum_no_gate_holds = 0;
    bool no_gate_hold_pose_moved = false;
    for (int step = 0; step < 35; ++step) {
        no_gate_hold_runner.step_with_observation(
            make_observation(23.0 + 0.10 * step, no_gate_hold_scan),
            0.10,
            false);
        const auto& command = no_gate_hold_runner.last_command();
        no_gate_hold_pose_moved = no_gate_hold_pose_moved ||
            std::abs(command.target_speed) > 1e-4;
        if (!no_gate_hold_runner.safety_stop_active() &&
            std::abs(command.target_speed) <= 1e-4) {
            ++consecutive_no_gate_holds;
            maximum_no_gate_holds = std::max(
                maximum_no_gate_holds,
                consecutive_no_gate_holds);
        } else {
            consecutive_no_gate_holds = 0;
        }
    }
    if (!no_gate_hold_pose_moved || maximum_no_gate_holds > 1) {
        return fail("report 215354 no-gate pose still re-entered stationary scanning");
    }

    // The terminal pose from report 20260818_214343_936 used to trigger an arc
    // solely because it was close to the authored 1.20 m rectangle. With an
    // open workspace and no sensed obstacle it must continue straight.
    thesis_sim::WorldMap search_pose_world = exploration_world;
    search_pose_world.set_start({0.920000, 0.509180});
    search_pose_world.set_start_heading(-0.629000);
    thesis_sim::HardwarePlannerRunner search_pose_runner(
        search_pose_world, bridge_options, direct_config);
    const auto search_pose_scan = make_scan(
        search_pose_world,
        search_pose_world.start(),
        search_pose_world.start_heading(),
        direct_config);
    bool saw_open_field_progress = false;
    for (int step = 0; step < 20; ++step) {
        search_pose_runner.step_with_observation(
            make_observation(25.0 + 0.10 * step, search_pose_scan),
            0.10,
            false);
        const auto& command = search_pose_runner.last_command();
        if (command.pwm_left * command.pwm_right < 0) {
            return fail("boundary LiDAR search issued opposing-wheel PWM");
        }
        saw_open_field_progress = saw_open_field_progress ||
            (search_pose_runner.diagnostics().control_source ==
                 thesis_sim::HardwareControlSource::StraightExploration &&
             command.target_speed > 1e-4 &&
             std::abs(command.target_yaw_rate) <= 0.015 &&
             command.pwm_left > 0 &&
             command.pwm_right > 0);
    }
    if (!saw_open_field_progress) {
        std::cerr << "search_pose_debug state="
                  << thesis_sim::unstructured_exploration_state_name(
                         search_pose_runner.diagnostics().exploration_state)
                  << " control="
                  << thesis_sim::hardware_control_source_name(
                         search_pose_runner.diagnostics().control_source)
                  << " speed=" << search_pose_runner.last_command().target_speed
                  << " yaw_rate=" << search_pose_runner.last_command().target_yaw_rate
                  << " pwm=" << search_pose_runner.last_command().pwm_left
                  << ',' << search_pose_runner.last_command().pwm_right
                  << " front=" << search_pose_runner.estimate().front_lidar_distance
                  << " min=" << search_pose_runner.estimate().min_lidar_distance
                  << " valid=" << search_pose_runner.diagnostics().valid_lidar_points
                  << " invalidation='"
                  << search_pose_runner.diagnostics().reference_invalidation_reason
                  << "'\n";
        return fail("former boundary pose did not continue straight in the open field");
    }

    // Likewise, the former right-hand arena edge must not manufacture a
    // reverse recovery when the LiDAR reports free space.
    thesis_sim::WorldMap reverse_pose_world = exploration_world;
    reverse_pose_world.set_start({1.02, 0.58});
    reverse_pose_world.set_start_heading(0.0);
    thesis_sim::HardwarePlannerRunner reverse_pose_runner(
        reverse_pose_world, bridge_options, direct_config);
    bool saw_forward_after_former_edge = false;
    for (int step = 0; step < 35; ++step) {
        const auto live_scan = make_scan(
            reverse_pose_world,
            reverse_pose_runner.estimate().position,
            reverse_pose_runner.estimate().yaw,
            direct_config);
        reverse_pose_runner.step_with_observation(
            make_observation(
                28.0 + 0.10 * step,
                live_scan,
                0,
                0,
                0.0),
            0.10,
            false);
        if (reverse_pose_runner.last_command().target_speed < -1e-4) {
            return fail("former arena edge still manufactured reverse recovery");
        }
        saw_forward_after_former_edge = saw_forward_after_former_edge ||
            reverse_pose_runner.last_command().target_speed > 1e-4;
    }
    if (!saw_forward_after_former_edge) {
        return fail("former arena edge did not permit open-field progress");
    }

    bootstrap_slam.connected = false;
    runner.apply_slam_toolbox_snapshot(bootstrap_slam);
    const std::size_t persistent_free =
        runner.global_free_points().size();
    if (persistent_free == 0U) {
        return fail("local persistent map was not available after SLAM fallback");
    }
    for (int step = 0; step < 5; ++step) {
        runner.step_with_observation(
            make_observation(4.0 + 0.10 * step, {}), 0.10, false);
    }
    if (runner.global_free_points().size() != persistent_free) {
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
        runner.global_free_points().size() != persistent_free) {
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

    // Observe the same opening off-centre, then rotate the whole fixture and
    // robot together. The aperture normal must follow the wall, not the bearing from
    // the robot to its midpoint. Check persistence over repeated observations.
    for (double rotation : {0.0, 0.35, -0.35}) {
        auto oblique_world = exploration_world;
        oblique_world.set_start({0.22, 0.50});
        oblique_world.set_start_heading(rotation);
        auto oblique_fixture = physical_fixture;
        oblique_fixture.set_bounds({-3.0, -3.0, 3.0, 3.0});
        oblique_fixture.editable_obstacles() = {
            {0.49, -3.0, 0.53, 0.42}, {0.49, 0.78, 0.53, 3.0}};
        auto oblique_scan = make_scan(oblique_fixture, oblique_world.start(), 0.0, config);
        thesis_sim::HardwarePlannerRunner oblique_runner(
            oblique_world, bridge_options, config);
        const thesis_sim::Vec2 sensor_origin{
            oblique_world.start().x + std::cos(rotation) * config.localization.lidar_x_offset -
                std::sin(rotation) * config.localization.lidar_y_offset,
            oblique_world.start().y + std::sin(rotation) * config.localization.lidar_x_offset +
                std::cos(rotation) * config.localization.lidar_y_offset};
        const double aperture_dx = 0.49 - oblique_world.start().x - config.localization.lidar_x_offset;
        const double aperture_dy = 0.60 - oblique_world.start().y - config.localization.lidar_y_offset;
        const thesis_sim::Vec2 expected_aperture{
            sensor_origin.x + std::cos(rotation) * aperture_dx - std::sin(rotation) * aperture_dy,
            sensor_origin.y + std::sin(rotation) * aperture_dx + std::cos(rotation) * aperture_dy};
        int observed_normals = 0;
        for (int step = 0; step < 12; ++step) {
            oblique_runner.step_with_observation(
                make_observation(90.0 + 0.1 * step, oblique_scan), 0.1, false);
            for (const auto& spec : oblique_runner.gate_specs()) {
                // Other scan sectors may contain separate openings at the
                // fixture boundaries; select the known two-post aperture.
                if (std::hypot(spec.anchor_position.x - expected_aperture.x,
                               spec.anchor_position.y - expected_aperture.y) > 0.06) {
                    continue;
                }
                const double dx = spec.position.x - spec.anchor_position.x;
                const double dy = spec.position.y - spec.anchor_position.y;
                if (std::hypot(dx, dy) < 0.029) {
                    continue;
                }
                const double heading_error = std::atan2(
                    std::sin(spec.heading_hint - rotation),
                    std::cos(spec.heading_hint - rotation));
                if (std::abs(heading_error) > 0.06 ||
                    dx * std::cos(rotation) + dy * std::sin(rotation) < 0.029) {
                    std::cerr << "oblique rotation=" << rotation << " heading=" << spec.heading_hint
                              << " target=" << spec.position.x << ',' << spec.position.y
                              << " anchor=" << spec.anchor_position.x << ',' << spec.anchor_position.y << '\n';
                    return fail("gate terminal heading followed the view bearing instead of aperture normal");
                }
                ++observed_normals;
            }
        }
        if (observed_normals < 3) {
            std::cerr << "oblique rotation=" << rotation << " observed=" << observed_normals
                      << " candidates=" << oblique_runner.gate_specs().size() << '\n';
            return fail("oblique aperture was not retained across observations");
        }
    }

    // Feed the same planner a scan from the physical two-wall gate fixture.
    // A confirmed aperture must replace any active frontier and become the
    // clothoid/MPC control source.
    thesis_sim::WorldMap offset_gate_world = exploration_world;
    offset_gate_world.set_start_heading(0.25);
    thesis_sim::HardwarePlannerRunner gate_runner(
        offset_gate_world, bridge_options, config);
    const std::vector<thesis_sim::RPLidarA1::ScanPoint> gate_scan =
        make_scan(physical_fixture, offset_gate_world.start(),
                  offset_gate_world.start_heading(), config);
    bool saw_physical_gate = false;
    bool saw_gate_steering_authority = false;
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
        if (gate_runner.diagnostics().planner_has_reference &&
            !gate_runner.safety_stop_active() &&
            gate_runner.diagnostics().control_source !=
                thesis_sim::HardwareControlSource::GateMpc) {
            return fail("valid gate clothoid lost control authority to an exploration fallback");
        }
        saw_gate_steering_authority = saw_gate_steering_authority ||
            (gate_runner.diagnostics().control_source ==
                 thesis_sim::HardwareControlSource::GateMpc &&
             gate_runner.last_command().target_speed > 1e-4 &&
             std::abs(gate_runner.last_command().target_yaw_rate) > 0.015 &&
             gate_runner.last_command().pwm_left > 0 &&
             gate_runner.last_command().pwm_right > 0 &&
             gate_runner.last_command().pwm_left !=
                 gate_runner.last_command().pwm_right);
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
    if (!saw_gate_steering_authority) {
        return fail("gate MPC did not produce a forward differential steering command");
    }
    if (gate_runner.gate_crossing_lateral_tolerance() > 0.080001) {
        return fail("compact gate crossing retained an excessively wide lateral acceptance band");
    }

    thesis_sim::HardwarePlannerRunner delayed_yaw_gate_runner(
        offset_gate_world, bridge_options, config);
    bool checked_delayed_gate_yaw = false;
    for (int step = 0; step < 45; ++step) {
        const double delayed_rate = (step % 2 == 0) ? 1.10 : -1.10;
        delayed_yaw_gate_runner.step_with_observation(
            make_observation(
                24.0 + 0.10 * step,
                gate_scan,
                0,
                0,
                0.0,
                delayed_rate),
            0.10,
            false);
        const auto& command = delayed_yaw_gate_runner.last_command();
        if (delayed_yaw_gate_runner.diagnostics().control_source !=
                thesis_sim::HardwareControlSource::GateMpc ||
            std::abs(command.planner_target_yaw_rate) < 0.020) {
            continue;
        }
        checked_delayed_gate_yaw = true;
        if (command.target_yaw_rate * command.planner_target_yaw_rate <= 0.0) {
            return fail("delayed yaw feedback reversed the MPC gate turn");
        }
    }
    if (!checked_delayed_gate_yaw) {
        return fail("delayed-yaw gate regression never produced an MPC turn request");
    }

    // A large free sector bounded by returns on opposite sides of the LiDAR
    // is free space, not a gate. Its midpoint must not become a crossing plane
    // underneath the car and must therefore never increment passed_gates.
    thesis_sim::HardwarePlannerRunner false_sector_runner(
        exploration_world, bridge_options, config);
    const auto false_sector_scan = make_false_wide_sector_scan(config);
    bool false_sector_became_gate = false;
    for (int step = 0; step < 35; ++step) {
        false_sector_runner.step_with_observation(
            make_observation(30.0 + 0.10 * step, false_sector_scan),
            0.10,
            false);
        false_sector_became_gate = false_sector_became_gate ||
            false_sector_runner.diagnostics().candidate_gates > 0 ||
            false_sector_runner.diagnostics().control_source ==
                thesis_sim::HardwareControlSource::GateMpc;
    }
    if (false_sector_became_gate || false_sector_runner.passed_gate_count() != 0) {
        return fail("opposite-side free sector was accepted as a physical gate");
    }

    // Regression for hardware report 20260819_012532_889.  The robot had a
    // valid GateMpc reference, reached VERIFYING_GATE_CROSSING, then the next
    // LiDAR/planning cycle erased the lock before the crossing could be
    // counted.  Drive encoder odometry through a narrow, slightly offset
    // aperture while regenerating the scan from the moving estimate.  A
    // detected physical gate must remain authoritative until its plane is
    // crossed, even when its two posts become close-range footprint returns.
    thesis_sim::WorldMap narrow_gate_fixture = physical_fixture;
    narrow_gate_fixture.editable_obstacles() = {
        {0.49, 0.00, 0.53, 0.460},
        {0.49, 0.740, 0.53, 1.20},
    };
    narrow_gate_fixture.finalize_editor_changes();

    // Regression for reports 215120/215227: an aperture can remain visible
    // behind the car after a failed approach. It is a useful candidate for a
    // later pass, but it must neither be locked as a forward target nor block
    // continued free-space exploration ahead.
    thesis_sim::WorldMap rear_gate_fixture = physical_fixture;
    rear_gate_fixture.editable_obstacles() = {
        {0.55, -0.50, 0.59, 0.450},
        {0.55, 0.750, 0.59, 1.70},
    };
    // The physical scan need not contain a wall at the virtual navigation
    // canvas. Keep the synthetic raycast boundary beyond LiDAR range: it is
    // not a physical wall in an open-ended hardware mission.
    rear_gate_fixture.set_bounds({-2.0, -2.0, 3.0, 3.0});
    rear_gate_fixture.finalize_editor_changes();
    thesis_sim::WorldMap rear_gate_world = exploration_world;
    rear_gate_world.set_start({1.02, 0.60});
    rear_gate_world.set_start_heading(0.0);
    thesis_sim::HardwarePlannerRunner rear_gate_runner(
        rear_gate_world, bridge_options, direct_config);
    bool saw_rear_candidate = false;
    bool rear_candidate_allowed_forward = false;
    bool rear_candidate_started_gate_mpc = false;
    bool rear_candidate_was_classified = false;
    bool rear_candidate_kept_straight = false;
    for (int step = 0; step < 30; ++step) {
        const auto rear_gate_scan = make_scan(
            rear_gate_fixture,
            rear_gate_runner.estimate().position,
            rear_gate_runner.estimate().yaw,
            direct_config);
        rear_gate_runner.step_with_observation(
            make_observation(36.0 + 0.10 * step, rear_gate_scan),
            0.10,
            false);
        saw_rear_candidate = saw_rear_candidate ||
            rear_gate_runner.diagnostics().candidate_gates > 0;
        rear_candidate_allowed_forward = rear_candidate_allowed_forward ||
            (rear_gate_runner.diagnostics().candidate_gates > 0 &&
             rear_gate_runner.last_command().target_speed > 1e-4 &&
             (rear_gate_runner.diagnostics().control_source ==
                  thesis_sim::HardwareControlSource::StraightExploration ||
              rear_gate_runner.diagnostics().control_source ==
                  thesis_sim::HardwareControlSource::ForwardSearch));
        rear_candidate_started_gate_mpc = rear_candidate_started_gate_mpc ||
            rear_gate_runner.diagnostics().control_source ==
                thesis_sim::HardwareControlSource::GateMpc;
        rear_candidate_was_classified = rear_candidate_was_classified ||
            rear_gate_runner.diagnostics().gate_candidate_status ==
                thesis_sim::GateCandidateStatus::RearSectorRejected;
        rear_candidate_kept_straight = rear_candidate_kept_straight ||
            (rear_gate_runner.diagnostics().candidate_gates > 0 &&
             rear_gate_runner.diagnostics().control_source ==
                 thesis_sim::HardwareControlSource::StraightExploration &&
             std::abs(rear_gate_runner.last_command().target_yaw_rate) <= 0.015);
    }
    if (!saw_rear_candidate) {
        std::cerr << "rear_gate_debug state="
                  << thesis_sim::unstructured_exploration_state_name(
                         rear_gate_runner.diagnostics().exploration_state)
                  << " control="
                  << thesis_sim::hardware_control_source_name(
                         rear_gate_runner.diagnostics().control_source)
                  << " speed=" << rear_gate_runner.last_command().target_speed
                  << " yaw_rate=" << rear_gate_runner.last_command().target_yaw_rate
                  << " front=" << rear_gate_runner.estimate().front_lidar_distance
                  << " min=" << rear_gate_runner.estimate().min_lidar_distance
                  << " map=" << rear_gate_runner.global_occupied_points().size()
                  << '\n';
        return fail("rear-gate regression did not reproduce the LiDAR candidate");
    }
    if (!rear_candidate_allowed_forward || rear_candidate_started_gate_mpc) {
        std::cerr << "rear_gate_motion_debug candidates="
                  << rear_gate_runner.diagnostics().candidate_gates
                  << " state="
                  << thesis_sim::unstructured_exploration_state_name(
                         rear_gate_runner.diagnostics().exploration_state)
                  << " control="
                  << thesis_sim::hardware_control_source_name(
                         rear_gate_runner.diagnostics().control_source)
                  << " speed=" << rear_gate_runner.last_command().target_speed
                  << " yaw_rate=" << rear_gate_runner.last_command().target_yaw_rate
                  << " front=" << rear_gate_runner.estimate().front_lidar_distance
                  << " min=" << rear_gate_runner.estimate().min_lidar_distance
                  << " ref=" << rear_gate_runner.diagnostics().planner_has_reference
                  << '\n';
        return fail("an unreachable rear gate candidate still blocked forward exploration");
    }
    if (!rear_candidate_was_classified || !rear_candidate_kept_straight) {
        return fail("rear gate was not rejected while preserving straight exploration");
    }

    // A gate just outside the clothoid-reachable sector may be approached by
    // a bounded forward arc, but it must not monopolize exploration forever.
    // With a fixed pose it stays near 100 degrees: acquisition must time out
    // and control must return to unconstrained free-space exploration.
    thesis_sim::HardwarePlannerConfig acquisition_config = direct_config;
    acquisition_config.gap_extraction.gate_search_acquire_timeout_s = 0.80;
    acquisition_config.gap_extraction.gate_search_acquire_cooldown_s = 0.80;
    thesis_sim::WorldMap side_gate_world = exploration_world;
    side_gate_world.set_start({1.02, 0.60});
    side_gate_world.set_start_heading(80.0 * kPi / 180.0);
    thesis_sim::HardwarePlannerRunner side_gate_runner(
        side_gate_world, bridge_options, acquisition_config);
    bool saw_bounded_acquisition = false;
    bool saw_acquisition_timeout = false;
    bool saw_free_exploration_after_timeout = false;
    bool side_gate_started_mpc = false;
    for (int step = 0; step < 35; ++step) {
        const auto side_gate_scan = make_scan(
            rear_gate_fixture,
            side_gate_runner.estimate().position,
            side_gate_runner.estimate().yaw,
            acquisition_config);
        side_gate_runner.step_with_observation(
            make_observation(39.0 + 0.10 * step, side_gate_scan),
            0.10,
            false);
        const auto& diagnostics = side_gate_runner.diagnostics();
        const auto& command = side_gate_runner.last_command();
        saw_bounded_acquisition = saw_bounded_acquisition ||
            (diagnostics.gate_candidate_status ==
                 thesis_sim::GateCandidateStatus::OutsideReachableSector &&
             diagnostics.gate_acquisition_active &&
             std::abs(command.target_yaw_rate) <=
                 acquisition_config.gap_extraction.gate_search_acquire_max_yaw_rate + 1e-6);
        saw_acquisition_timeout = saw_acquisition_timeout ||
            diagnostics.gate_candidate_status ==
                thesis_sim::GateCandidateStatus::AcquisitionTimedOut;
        saw_free_exploration_after_timeout = saw_free_exploration_after_timeout ||
            (saw_acquisition_timeout &&
             !diagnostics.gate_acquisition_active &&
             (diagnostics.control_source ==
                  thesis_sim::HardwareControlSource::StraightExploration ||
              diagnostics.control_source ==
                  thesis_sim::HardwareControlSource::ForwardSearch) &&
             command.target_speed > 1e-4 &&
             !diagnostics.planner_has_reference);
        side_gate_started_mpc = side_gate_started_mpc ||
            diagnostics.control_source == thesis_sim::HardwareControlSource::GateMpc;
    }
    if (!saw_bounded_acquisition || !saw_acquisition_timeout ||
        !saw_free_exploration_after_timeout || side_gate_started_mpc) {
        std::cerr << "side_gate_debug acquisition=" << saw_bounded_acquisition
                  << " timeout=" << saw_acquisition_timeout
                  << " free_after_timeout=" << saw_free_exploration_after_timeout
                  << " mpc=" << side_gate_started_mpc
                  << " candidates=" << side_gate_runner.diagnostics().candidate_gates
                  << " raw=" << side_gate_runner.diagnostics().raw_gate_candidates
                  << " heading_deg="
                  << side_gate_runner.diagnostics().candidate_gate_heading_error_rad *
                         180.0 / kPi
                  << " status="
                  << thesis_sim::gate_candidate_status_name(
                         side_gate_runner.diagnostics().gate_candidate_status)
                  << " state="
                  << thesis_sim::unstructured_exploration_state_name(
                         side_gate_runner.diagnostics().exploration_state)
                  << " control="
                  << thesis_sim::hardware_control_source_name(
                         side_gate_runner.diagnostics().control_source)
                  << " yaw_rate=" << side_gate_runner.last_command().target_yaw_rate
                  << '\n';
        return fail("side-gate acquisition did not time out back to free exploration");
    }

    thesis_sim::WorldMap crossing_world = exploration_world;
    crossing_world.set_start({0.28, 0.57});
    crossing_world.set_start_heading(0.0);
    thesis_sim::HardwarePlannerRunner crossing_runner(
        crossing_world, bridge_options, config);
    std::int32_t crossing_ticks = 0;
    bool crossing_saw_gate_mpc = false;
    bool crossing_saw_verification = false;
    bool crossing_lock_dropped = false;
    bool crossing_used_free_search_while_locked = false;
    for (int step = 0; step < 80 && crossing_runner.passed_gate_count() == 0;
         ++step) {
        const auto moving_scan = make_scan(
            narrow_gate_fixture,
            crossing_runner.estimate().position,
            crossing_runner.estimate().yaw,
            config);
        crossing_runner.step_with_observation(
            make_observation(
                40.0 + 0.10 * step,
                moving_scan,
                crossing_ticks,
                crossing_ticks,
                0.0),
            0.10,
            false);
        crossing_saw_gate_mpc = crossing_saw_gate_mpc ||
            crossing_runner.diagnostics().control_source ==
                thesis_sim::HardwareControlSource::GateMpc;
        crossing_saw_verification = crossing_saw_verification ||
            crossing_runner.diagnostics().exploration_state ==
                thesis_sim::UnstructuredExplorationState::VerifyingGateCrossing;
        if (crossing_saw_gate_mpc &&
            crossing_runner.passed_gate_count() == 0 &&
            crossing_runner.diagnostics().candidate_gates > 0 &&
            crossing_runner.diagnostics().control_source ==
                thesis_sim::HardwareControlSource::ForwardSearch) {
            crossing_used_free_search_while_locked = true;
        }
        if (crossing_saw_gate_mpc &&
            crossing_runner.passed_gate_count() == 0 &&
            crossing_runner.diagnostics().candidate_gates == 0 &&
            !crossing_runner.diagnostics().planner_has_reference) {
            crossing_lock_dropped = true;
        }
        if (crossing_runner.last_command().target_speed > 1e-4) {
            crossing_ticks += 2;
        }
    }
    if (!crossing_saw_gate_mpc) {
        return fail("moving narrow-gate regression never acquired GateMpc");
    }
    if (!crossing_saw_verification) {
        return fail("moving narrow-gate regression never reached crossing verification");
    }
    if (crossing_lock_dropped) {
        std::cerr << "crossing_drop_debug position="
                  << crossing_runner.estimate().position.x << ','
                  << crossing_runner.estimate().position.y
                  << " state="
                  << thesis_sim::unstructured_exploration_state_name(
                         crossing_runner.diagnostics().exploration_state)
                  << " safety="
                  << crossing_runner.safety_stop_active()
                  << " invalidation='"
                  << crossing_runner.diagnostics().reference_invalidation_reason
                  << "'\n";
        return fail("physical gate lock was erased during the committed approach");
    }
    if (crossing_used_free_search_while_locked) {
        return fail("committed gate tracking fell back to free-space search");
    }
    if (crossing_runner.passed_gate_count() < 1) {
        std::cerr << "crossing_debug position="
                  << crossing_runner.estimate().position.x << ','
                  << crossing_runner.estimate().position.y
                  << " state="
                  << thesis_sim::unstructured_exploration_state_name(
                         crossing_runner.diagnostics().exploration_state)
                  << " control="
                  << thesis_sim::hardware_control_source_name(
                         crossing_runner.diagnostics().control_source)
                  << " candidates="
                  << crossing_runner.diagnostics().candidate_gates
                  << " reference="
                  << crossing_runner.diagnostics().planner_has_reference
                  << " invalidation='"
                  << crossing_runner.diagnostics().reference_invalidation_reason
                  << "'\n";
        return fail("moving narrow-gate regression did not count the crossing");
    }
    if (crossing_runner.estimate().position.x < 0.49) {
        return fail("gate was counted before the robot centre crossed the aperture plane");
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
              << " fallback_free=" << persistent_free
              << " state="
              << thesis_sim::unstructured_exploration_state_name(
                     runner.diagnostics().exploration_state)
              << '\n';
    return 0;
}
