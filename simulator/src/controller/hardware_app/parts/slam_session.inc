// Included in hardware_app's anonymous namespace and exercised by the UDP test.
class HardwareSlamSession {
  public:
    void update(thesis_sim::HardwarePlannerRunner& runner,
                thesis_sim::SlamToolboxBridgeClient& bridge,
                const thesis_sim::RealRobotObservation& observation,
                const std::string& run_id, int configuration_epoch) {
        const bool ready = runner.config().slam_toolbox_enabled && runner.slam_start_ready();
        const std::string session = run_id + "_" + std::to_string(configuration_epoch) +
            "_pose_" + std::to_string(runner.pose_epoch());
        if (!ready || session != session_) {
            bridge.reset_session();
            if (!session_.empty() || runner.slam_toolbox_snapshot().connected) {
                runner.apply_slam_toolbox_snapshot({});
            }
            session_ = ready ? session : std::string{};
            last_scan_timestamp_ = -1.0;
        }
        if (!ready || !bridge.active()) return;

        const double timestamp = observation.lidar_scan_mid_timestamp_s > 0.0
            ? observation.lidar_scan_mid_timestamp_s : observation.host_timestamp_s;
        if (observation.have_lidar_scan && !observation.lidar_scan_reused &&
            runner.diagnostics().lidar_age_s <= runner.config().localization.max_lidar_age_s &&
            timestamp > last_scan_timestamp_ && !runner.lidar_hits().empty()) {
            if (bridge.submit_scan(session_, ++sequence_, timestamp,
                    runner.estimate().position, runner.estimate().yaw, runner.lidar_hits(),
                    runner.config().localization.min_valid_range_m,
                    runner.config().localization.max_range_m)) {
                last_scan_timestamp_ = timestamp;
            }
        }
        const bool updated = bridge.poll();
        const auto& snapshot = bridge.snapshot();
        if (updated || runner.slam_toolbox_snapshot().connected != snapshot.connected) {
            runner.apply_slam_toolbox_snapshot(snapshot);
        }
    }

  private:
    std::string session_;
    double last_scan_timestamp_ = -1.0;
    std::uint64_t sequence_ = 0;
};
