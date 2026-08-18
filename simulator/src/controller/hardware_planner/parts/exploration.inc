#include "support.inc"

#include <queue>

namespace thesis_sim {

const char* exploration_map_source_name(ExplorationMapSource source) {
    return source == ExplorationMapSource::SlamToolbox
               ? "SLAM_TOOLBOX"
               : "LOCAL_PERSISTENT_GRID";
}

const char* unstructured_exploration_state_name(UnstructuredExplorationState state) {
    switch (state) {
        case UnstructuredExplorationState::StartMatching: return "START_MATCHING";
        case UnstructuredExplorationState::InitialScan: return "INITIAL_SCAN";
        case UnstructuredExplorationState::ExploringFrontier: return "EXPLORING_FRONTIER";
        case UnstructuredExplorationState::GateLocked: return "GATE_LOCKED";
        case UnstructuredExplorationState::TrackingGateClothoid: return "TRACKING_GATE_CLOTHOID";
        case UnstructuredExplorationState::VerifyingGateCrossing: return "VERIFYING_GATE_CROSSING";
        case UnstructuredExplorationState::Hold: return "HOLD";
        case UnstructuredExplorationState::Complete: return "COMPLETE";
        case UnstructuredExplorationState::AdvancingStraight: return "ADVANCING_STRAIGHT";
        case UnstructuredExplorationState::SearchingFreeSpace: return "SEARCHING_FREE_SPACE";
        case UnstructuredExplorationState::Disabled:
        default: return "DISABLED";
    }
}

const char* hardware_control_source_name(HardwareControlSource source) {
    switch (source) {
        case HardwareControlSource::FrontierMpc: return "FRONTIER_MPC";
        case HardwareControlSource::GateMpc: return "GATE_MPC";
        case HardwareControlSource::Recovery: return "RECOVERY";
        case HardwareControlSource::StructuredMpc: return "STRUCTURED_MPC";
        case HardwareControlSource::SafetyStop: return "SAFETY_STOP";
        case HardwareControlSource::StraightExploration: return "STRAIGHT_EXPLORATION";
        case HardwareControlSource::ForwardSearch: return "FORWARD_SEARCH";
        case HardwareControlSource::Hold:
        default: return "HOLD";
    }
}

void HardwarePlannerRunner::reset_start_matching() {
    start_matching_status_ = {};
    start_matching_status_.enabled =
        config_.start_matching.enabled &&
        world_.environment_mode() == EnvironmentMode::UnstructuredGates &&
        lidar_enabled_for_current_mode();
    start_match_group_points_.clear();
    start_match_results_.clear();
    start_match_group_scan_count_ = 0;
    start_reference_ = {};
    if (!start_matching_status_.enabled) {
        start_matching_status_.complete = true;
        start_matching_status_.accepted = true;
        start_matching_status_.status = "disabled_by_runtime_setting";
        return;
    }
    if (config_.start_matching.reference_path.empty()) {
        start_matching_status_.status = "reference_path_missing";
        return;
    }
    std::string error;
    if (!mvc::model::read_initial_lidar_reference(
            config_.start_matching.reference_path,
            &start_reference_,
            &error)) {
        start_matching_status_.status = "reference_load_failed: " + error;
        return;
    }
    if (!start_reference_.integrity_verified || start_reference_.points.empty()) {
        start_matching_status_.status = "reference_integrity_failed";
        return;
    }
    if (start_reference_.metadata.schema == "thesis_initial_lidar_reference_v2" &&
        !start_reference_.metadata.stability_valid) {
        start_matching_status_.status = "reference_capture_stability_failed";
        return;
    }
    start_matching_status_.reference_loaded = true;
    start_matching_status_.status = "waiting_for_stationary_scans";
}

bool HardwarePlannerRunner::motion_permitted_by_start_match() const {
    return !start_matching_status_.enabled ||
           (start_matching_status_.complete && start_matching_status_.accepted);
}

void HardwarePlannerRunner::update_start_matching(
    const std::vector<RPLidarA1::ScanPoint>& scan) {
    if (!start_matching_status_.enabled || start_matching_status_.complete ||
        !start_matching_status_.reference_loaded || scan.empty()) {
        return;
    }

    if (!start_reference_.metadata.lidar_serial.empty()) {
        const auto& device_info = bridge_.lidar_device_info();
        if (!device_info.has_value()) {
            start_matching_status_.status = "waiting_for_lidar_identity";
            return;
        }
        if (device_info->serial_hex != start_reference_.metadata.lidar_serial) {
            start_matching_status_.complete = true;
            start_matching_status_.accepted = false;
            start_matching_status_.status = "lidar_serial_mismatch";
            return;
        }
    }

    const auto& metadata = start_reference_.metadata;
    std::vector<Vec2> scan_points;
    scan_points.reserve(scan.size());
    const double half_length = 0.5 * metadata.body_length_m + 0.015;
    const double half_width = 0.5 * metadata.body_width_m + 0.015;
    for (const RPLidarA1::ScanPoint& point : scan) {
        if (!std::isfinite(point.distance_m) ||
            point.distance_m < metadata.min_range_m ||
            point.distance_m > metadata.max_range_m) {
            continue;
        }
        const double raw_angle = point.angle_deg * kPi / 180.0;
        const double sensor_angle = metadata.lidar_flip_left_right ? -raw_angle : raw_angle;
        const double body_angle = wrap_angle(metadata.lidar_yaw_offset_rad + sensor_angle);
        const Vec2 endpoint{
            metadata.lidar_x_offset_m + std::cos(body_angle) * point.distance_m,
            metadata.lidar_y_offset_m + std::sin(body_angle) * point.distance_m,
        };
        if (std::abs(endpoint.x) <= half_length && std::abs(endpoint.y) <= half_width) {
            continue;
        }
        scan_points.push_back(endpoint);
    }
    if (static_cast<int>(scan_points.size()) <
        std::max(config_.start_matching.minimum_points_per_scan, 20)) {
        start_matching_status_.status = "waiting_for_sufficient_scan_points";
        return;
    }

    start_match_group_points_.insert(
        start_match_group_points_.end(), scan_points.begin(), scan_points.end());
    ++start_match_group_scan_count_;
    ++start_matching_status_.collected_scans;
    const int scans_per_match = std::max(config_.start_matching.scans_per_confirmation, 1);
    if (start_match_group_scan_count_ < scans_per_match) {
        start_matching_status_.status = "collecting_confirmation_cloud";
        return;
    }

    const std::vector<Vec2> current_cloud =
        mvc::model::voxelize_initial_lidar_cloud(
            start_match_group_points_,
            std::max(metadata.voxel_size_m, 0.005),
            1);
    mvc::model::InitialLidarMatcher matcher(config_.start_matching.matcher);
    start_match_results_.push_back(
        matcher.match(start_reference_.points, current_cloud));
    start_match_group_points_.clear();
    start_match_group_scan_count_ = 0;
    start_matching_status_.completed_matches =
        static_cast<int>(start_match_results_.size());

    const int required = std::max(config_.start_matching.required_confirmations, 1);
    if (static_cast<int>(start_match_results_.size()) < required) {
        start_matching_status_.status = "collecting_independent_confirmations";
        return;
    }

    mvc::model::InitialLidarConfirmationConfig confirmation =
        config_.start_matching.confirmation;
    confirmation.required_matches = required;
    const mvc::model::InitialLidarConfirmationResult result =
        mvc::model::confirm_initial_lidar_matches(start_match_results_, confirmation);
    start_matching_status_.complete = true;
    start_matching_status_.accepted = result.accepted;
    start_matching_status_.status = result.status;
    start_matching_status_.position_offset_m = result.fused_match.position_offset_m;
    start_matching_status_.yaw_offset_rad = result.fused_match.yaw_offset_rad;
    start_matching_status_.confidence = result.fused_match.confidence;
    if (!result.accepted) {
        return;
    }

    const double c = std::cos(world_.start_heading());
    const double s = std::sin(world_.start_heading());
    const Vec2 offset = result.fused_match.position_offset_m;
    const Vec2 matched_position{
        world_.start().x + c * offset.x - s * offset.y,
        world_.start().y + s * offset.x + c * offset.y,
    };
    reset_pose(
        matched_position,
        wrap_angle(world_.start_heading() + result.fused_match.yaw_offset_rad));
}

void HardwarePlannerRunner::apply_slam_toolbox_snapshot(
    const SlamToolboxSnapshot& snapshot) {
    slam_toolbox_snapshot_ = snapshot;
    slam_snapshot_received_sim_time_s_ = sim_time_;
    if (!config_.slam_toolbox_enabled || !snapshot.connected ||
        snapshot.occupied_points.empty() || snapshot.free_points.empty()) {
        exploration_map_source_ = ExplorationMapSource::LocalPersistentGrid;
        rebuild_global_map_vectors();
        diagnostics_.map_source = exploration_map_source_;
        diagnostics_.global_free_cells = static_cast<int>(global_free_points_.size());
        diagnostics_.global_occupied_cells =
            static_cast<int>(global_occupied_points_.size());
        return;
    }
    exploration_map_source_ = ExplorationMapSource::SlamToolbox;
    global_free_points_ = snapshot.free_points;
    global_occupied_points_ = snapshot.occupied_points;
    diagnostics_.map_source = exploration_map_source_;
    diagnostics_.global_free_cells = static_cast<int>(global_free_points_.size());
    diagnostics_.global_occupied_cells =
        static_cast<int>(global_occupied_points_.size());
    diagnostics_.slam_map_age_s = 0.0;
    if (config_.slam_pose_feedback_enabled && snapshot.pose_valid) {
        apply_slam_pose_correction(snapshot.corrected_position, snapshot.corrected_yaw);
    }
}

void HardwarePlannerRunner::update_global_exploration_map(
    const std::vector<RPLidarA1::ScanPoint>& scan) {
    if (world_.environment_mode() != EnvironmentMode::UnstructuredGates || scan.empty()) {
        return;
    }
    const double resolution = std::max(config_.frontier_exploration.grid_resolution_m, 0.02);
    const Vec2 origin = lidar_origin_world(estimate_.position, estimate_.yaw, config_.localization);
    for (const RPLidarA1::ScanPoint& point : scan) {
        if (!(point.distance_m > 0.0) ||
            point.distance_m < config_.localization.min_valid_range_m ||
            point.distance_m > config_.localization.max_range_m ||
            scan_point_is_self_hit(point)) {
            continue;
        }
        const double angle = wrap_angle(
            estimate_.yaw + lidar_body_local_angle_rad(config_.localization, point.angle_deg));
        const bool endpoint_occupied =
            point.distance_m < config_.localization.max_range_m - 0.03;
        const double free_distance = endpoint_occupied
            ? std::max(0.0, point.distance_m - 1.5 * resolution)
            : point.distance_m;
        const Vec2 free_end{
            origin.x + std::cos(angle) * free_distance,
            origin.y + std::sin(angle) * free_distance,
        };
        sample_segment_points(origin, free_end, 0.70 * resolution, true,
            [&](const Vec2& sample) {
                if (!is_inside_bounds(world_, sample)) return;
                const std::uint64_t key = quantized_point_key(sample, resolution);
                ExplorationOccupancyCell& cell = exploration_cells_[key];
                cell.center = quantized_point_center(sample, resolution);
                cell.free = true;
                cell.occupied = false;
                cell.observations = std::min(cell.observations + 1, 1000000);
                cell.last_observed_step = step_count_;
            });
        if (endpoint_occupied) {
            const Vec2 hit{
                origin.x + std::cos(angle) * point.distance_m,
                origin.y + std::sin(angle) * point.distance_m,
            };
            if (is_inside_bounds(world_, hit)) {
                const std::uint64_t key = quantized_point_key(hit, resolution);
                ExplorationOccupancyCell& cell = exploration_cells_[key];
                cell.center = quantized_point_center(hit, resolution);
                cell.free = false;
                cell.occupied = true;
                cell.observations = std::min(cell.observations + 3, 1000000);
                cell.last_observed_step = step_count_;
            }
        }
    }
    if (exploration_map_source_ != ExplorationMapSource::SlamToolbox ||
        !slam_toolbox_snapshot_.connected) {
        exploration_map_source_ = ExplorationMapSource::LocalPersistentGrid;
        rebuild_global_map_vectors();
    }
}

void HardwarePlannerRunner::rebuild_global_map_vectors() {
    global_free_points_.clear();
    global_occupied_points_.clear();
    global_free_points_.reserve(exploration_cells_.size());
    global_occupied_points_.reserve(exploration_cells_.size());
    for (const auto& [key, cell] : exploration_cells_) {
        (void)key;
        if (cell.occupied) global_occupied_points_.push_back(cell.center);
        else if (cell.free) global_free_points_.push_back(cell.center);
    }
}

void HardwarePlannerRunner::clear_exploration_target(const std::string& reason) {
    if (navigation_target_kind_ == NavigationTargetKind::ExplorationFrontier) {
        clear_locked_gap_goal();
    }
    selected_frontier_.reset();
    exploration_path_.clear();
    if (!reason.empty()) diagnostics_.reference_invalidation_reason = reason;
}

void HardwarePlannerRunner::set_exploration_target(const Vec2& target, double heading) {
    set_locked_gap_goal(target, "exploration_frontier", target);
    navigation_target_kind_ = NavigationTargetKind::ExplorationFrontier;
    locked_gap_approach_direction_ = {std::cos(heading), std::sin(heading)};
    locked_gap_corridor_half_width_m_ = std::max(0.5 * geometry_.body_width + 0.015, 0.08);
    publish_locked_gap_goal();
    exploration_state_ = UnstructuredExplorationState::ExploringFrontier;
}

bool HardwarePlannerRunner::select_exploration_frontier() {
    if (!config_.frontier_exploration.enabled || global_free_points_.empty()) {
        return false;
    }
    const Rect bounds = world_.bounds();
    const double resolution = std::max(config_.frontier_exploration.grid_resolution_m, 0.02);
    LocalOccupancyGrid grid;
    grid.resolution_m = resolution;
    grid.origin = {bounds.min_x, bounds.min_y};
    grid.width = std::max(1, static_cast<int>(std::ceil((bounds.max_x - bounds.min_x) / resolution)));
    grid.height = std::max(1, static_cast<int>(std::ceil((bounds.max_y - bounds.min_y) / resolution)));
    const std::size_t cell_count = static_cast<std::size_t>(grid.width * grid.height);
    grid.free_votes.assign(cell_count, 0U);
    grid.occupied_votes.assign(cell_count, 0U);
    for (const Vec2& point : global_free_points_) grid.add_free_sample(point);
    for (const Vec2& point : global_occupied_points_) grid.add_occupied_sample(point);
    grid.finalize(std::max(config_.frontier_exploration.obstacle_inflation_m,
                           0.5 * geometry_.body_width + 0.02));

    int start_x = 0;
    int start_y = 0;
    if (!grid.world_to_cell(estimate_.position, &start_x, &start_y)) return false;
    if (!grid.free_mask[static_cast<std::size_t>(grid.index(start_x, start_y))]) {
        bool found = false;
        for (int radius = 1; radius <= 4 && !found; ++radius) {
            for (int dy = -radius; dy <= radius && !found; ++dy) {
                for (int dx = -radius; dx <= radius; ++dx) {
                    const int x = start_x + dx;
                    const int y = start_y + dy;
                    if (grid.valid_cell(x, y) &&
                        grid.free_mask[static_cast<std::size_t>(grid.index(x, y))]) {
                        start_x = x;
                        start_y = y;
                        found = true;
                        break;
                    }
                }
            }
        }
        if (!found) return false;
    }

    const double safe_center_margin =
        0.5 * std::hypot(geometry_.body_length, geometry_.body_width) + 0.015;
    const bool straight_ahead_car =
        config_.vehicle_model == VehicleModelKind::CarLikeBicycle &&
        manual_unstructured_exploration(world_);
    if (straight_ahead_car) {
        // Exploration is deliberately different from gate tracking.  While no
        // physical aperture is locked, the car may only extend its current
        // LiDAR-visible corridor straight ahead.  Selecting a lateral frontier
        // here made the gate planner manufacture a clothoid/search arc even
        // though no gate had been detected.
        const double minimum_distance = std::max(
            config_.frontier_exploration.minimum_frontier_distance_m,
            0.10);
        const double maximum_distance = std::max(
            minimum_distance,
            config_.frontier_exploration.maximum_frontier_distance_m);
        const double sample_step = std::max(0.5 * resolution, 0.015);
        const Vec2 forward{std::cos(estimate_.yaw), std::sin(estimate_.yaw)};
        std::vector<Vec2> straight_path;
        straight_path.push_back(estimate_.position);
        double farthest_distance = 0.0;
        for (double d = sample_step;
             d <= maximum_distance + 1e-9;
             d += sample_step) {
            const Vec2 point{
                estimate_.position.x + d * forward.x,
                estimate_.position.y + d * forward.y,
            };
            if (!is_inside_bounds(world_, point, -safe_center_margin)) {
                break;
            }
            if (!grid.is_free(point) || grid.is_inflated(point)) {
                // A few centimetres around the LiDAR origin can remain unknown
                // because of self-hit rejection.  Beyond that local blind spot,
                // free-space support must be continuous.
                if (d <= std::min(0.06, 0.5 * minimum_distance)) {
                    continue;
                }
                break;
            }
            straight_path.push_back(point);
            farthest_distance = d;
        }
        if (farthest_distance < minimum_distance || straight_path.size() < 2U) {
            exploration_state_ = UnstructuredExplorationState::Hold;
            diagnostics_.reference_invalidation_reason =
                "no_straight_known_free_frontier";
            return false;
        }

        exploration_path_ = straight_path;
        selected_frontier_ = straight_path.back();
        const double local_target_distance = std::min(
            farthest_distance,
            std::max(config_.frontier_exploration.path_lookahead_m,
                     minimum_distance));
        Vec2 local_target = straight_path.back();
        for (const Vec2& point : straight_path) {
            if (distance(estimate_.position, point) >= local_target_distance) {
                local_target = point;
                break;
            }
        }
        set_exploration_target(local_target, estimate_.yaw);
        last_frontier_plan_step_ = step_count_;
        diagnostics_.reference_invalidation_reason.clear();
        return true;
    }

    const int start_index = grid.index(start_x, start_y);
    std::vector<int> distance_steps(cell_count, -1);
    std::vector<int> parent(cell_count, -1);
    std::queue<int> queue;
    distance_steps[static_cast<std::size_t>(start_index)] = 0;
    queue.push(start_index);
    constexpr int kNeighbors[8][2] = {
        {-1, -1}, {0, -1}, {1, -1}, {-1, 0},
        {1, 0}, {-1, 1}, {0, 1}, {1, 1},
    };
    int best_index = -1;
    double best_score = -std::numeric_limits<double>::infinity();
    while (!queue.empty()) {
        const int current = queue.front();
        queue.pop();
        const int x = current % grid.width;
        const int y = current / grid.width;
        const int steps = distance_steps[static_cast<std::size_t>(current)];
        const Vec2 point = grid.cell_center(x, y);
        const double path_distance = steps * resolution;

        int unknown_neighbors = 0;
        for (const auto& delta : kNeighbors) {
            const int nx = x + delta[0];
            const int ny = y + delta[1];
            if (!grid.valid_cell(nx, ny)) continue;
            const int neighbor = grid.index(nx, ny);
            if (grid.free_votes[static_cast<std::size_t>(neighbor)] == 0U &&
                grid.occupied_votes[static_cast<std::size_t>(neighbor)] == 0U) {
                ++unknown_neighbors;
            }
        }
        const bool visited = std::any_of(
            visited_frontiers_.begin(), visited_frontiers_.end(),
            [&](const Vec2& previous) { return distance(previous, point) <= 0.12; });
        if (unknown_neighbors > 0 && !visited &&
            path_distance >= config_.frontier_exploration.minimum_frontier_distance_m &&
            path_distance <= config_.frontier_exploration.maximum_frontier_distance_m &&
            is_inside_bounds(world_, point, -safe_center_margin)) {
            const double heading_error = std::abs(wrap_angle(angle_to(estimate_.position, point) - estimate_.yaw));
            const double score =
                2.4 * static_cast<double>(unknown_neighbors) -
                1.2 * path_distance -
                0.18 * heading_error;
            if (score > best_score) {
                best_score = score;
                best_index = current;
            }
        }

        for (const auto& delta : kNeighbors) {
            const int nx = x + delta[0];
            const int ny = y + delta[1];
            if (!grid.valid_cell(nx, ny)) continue;
            const int neighbor = grid.index(nx, ny);
            if (distance_steps[static_cast<std::size_t>(neighbor)] >= 0 ||
                !grid.free_mask[static_cast<std::size_t>(neighbor)]) continue;
            distance_steps[static_cast<std::size_t>(neighbor)] = steps + 1;
            parent[static_cast<std::size_t>(neighbor)] = current;
            queue.push(neighbor);
        }
    }
    if (best_index < 0) return false;

    std::vector<Vec2> reversed_path;
    for (int current = best_index; current >= 0; current = parent[static_cast<std::size_t>(current)]) {
        reversed_path.push_back(grid.cell_center(current % grid.width, current / grid.width));
        if (current == start_index) break;
    }
    if (reversed_path.empty() ||
        distance(reversed_path.back(), grid.cell_center(start_x, start_y)) > 1.5 * resolution) {
        return false;
    }
    exploration_path_.assign(reversed_path.rbegin(), reversed_path.rend());
    selected_frontier_ = exploration_path_.back();
    Vec2 local_target = selected_frontier_.value();
    double accumulated = 0.0;
    std::size_t target_index = exploration_path_.size() - 1U;
    for (std::size_t i = 1; i < exploration_path_.size(); ++i) {
        accumulated += distance(exploration_path_[i - 1U], exploration_path_[i]);
        if (accumulated >= config_.frontier_exploration.path_lookahead_m) {
            local_target = exploration_path_[i];
            target_index = i;
            break;
        }
    }
    const std::size_t heading_from = target_index > 0U ? target_index - 1U : 0U;
    const double heading = angle_to(exploration_path_[heading_from], local_target);
    set_exploration_target(local_target, heading);
    last_frontier_plan_step_ = step_count_;
    diagnostics_.reference_invalidation_reason.clear();
    return true;
}

bool HardwarePlannerRunner::update_exploration_target() {
    if (!manual_unstructured_exploration(world_) ||
        !config_.frontier_exploration.enabled ||
        !motion_permitted_by_start_match()) {
        return false;
    }
    if (selected_frontier_.has_value() &&
        distance(estimate_.position, *selected_frontier_) <=
            config_.frontier_exploration.target_reached_radius_m) {
        visited_frontiers_.push_back(*selected_frontier_);
        clear_exploration_target("frontier_reached");
    }
    const bool replan_due =
        last_frontier_plan_step_ < 0 ||
        step_count_ - last_frontier_plan_step_ >=
            std::max(config_.frontier_exploration.replan_interval_steps, 1);
    if (selected_frontier_.has_value() &&
        navigation_target_kind_ == NavigationTargetKind::None &&
        !exploration_path_.empty() && !replan_due) {
        std::size_t nearest_index = 0U;
        double nearest_distance = std::numeric_limits<double>::infinity();
        for (std::size_t i = 0; i < exploration_path_.size(); ++i) {
            const double candidate_distance =
                distance(estimate_.position, exploration_path_[i]);
            if (candidate_distance < nearest_distance) {
                nearest_distance = candidate_distance;
                nearest_index = i;
            }
        }
        std::size_t target_index = nearest_index;
        double accumulated = 0.0;
        for (std::size_t i = nearest_index + 1U;
             i < exploration_path_.size(); ++i) {
            accumulated += distance(
                exploration_path_[i - 1U], exploration_path_[i]);
            target_index = i;
            if (accumulated >=
                config_.frontier_exploration.path_lookahead_m) {
                break;
            }
        }
        const std::size_t heading_from =
            target_index > 0U ? target_index - 1U : target_index;
        const double heading = angle_to(
            exploration_path_[heading_from],
            exploration_path_[target_index]);
        set_exploration_target(exploration_path_[target_index], heading);
        return true;
    }
    if (navigation_target_kind_ == NavigationTargetKind::ExplorationFrontier &&
        locked_gap_goal_.has_value() && !replan_due) {
        publish_locked_gap_goal();
        return true;
    }
    if (navigation_target_kind_ == NavigationTargetKind::ExplorationFrontier) {
        clear_locked_gap_goal();
    }
    if (select_exploration_frontier()) return true;
    exploration_state_ = UnstructuredExplorationState::Hold;
    return false;
}

}  // namespace thesis_sim
