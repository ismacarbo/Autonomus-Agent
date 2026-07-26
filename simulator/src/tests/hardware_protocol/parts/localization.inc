class EkfSlamLocalizer {
  public:
    explicit EkfSlamLocalizer(const DriveConfig& drive)
        : geometry_(make_vehicle_geometry(drive)) {
        ekf_.set_geometry(geometry_);
        sync_estimate();
    }

    void reset(const Pose2D& pose) {
        ekf_.reset({pose.x, pose.y}, pose.yaw);
        yaw_offset_ = 0.0;
        yaw_offset_initialized_ = false;
        encoder_ticks_initialized_ = false;
        last_left_ticks_ = 0;
        last_right_ticks_ = 0;
        have_raw_imu_yaw_ = false;
        last_raw_imu_yaw_ = 0.0;
        sync_estimate();
    }

    OdomUpdate fuse_controller(const ControllerTelemetry& telemetry, double fallback_dt_s) {
        OdomUpdate out{};
        const double imu_yaw = static_cast<double>(telemetry.yaw_mrad) / 1000.0;
        last_raw_imu_yaw_ = imu_yaw;
        have_raw_imu_yaw_ = true;

        if (!yaw_offset_initialized_) {
            yaw_offset_ = wrap_angle(estimate_.pose.yaw - imu_yaw);
            yaw_offset_initialized_ = true;
        }

        const double measured_yaw = wrap_angle(imu_yaw + yaw_offset_);
        const double measured_yaw_rate = static_cast<double>(telemetry.yaw_rate_mrad_s) / 1000.0;

        if (!encoder_ticks_initialized_) {
            last_left_ticks_ = telemetry.ticks_left;
            last_right_ticks_ = telemetry.ticks_right;
            encoder_ticks_initialized_ = true;
            ekf_.update_imu(measured_yaw, measured_yaw_rate);
            sync_estimate();
            return out;
        }

        out.left_delta_ticks = telemetry.ticks_left - last_left_ticks_;
        out.right_delta_ticks = telemetry.ticks_right - last_right_ticks_;
        last_left_ticks_ = telemetry.ticks_left;
        last_right_ticks_ = telemetry.ticks_right;
        out.encoder_dt_s = telemetry.enc_dt_ms > 0
                               ? clamp_value(static_cast<double>(telemetry.enc_dt_ms) / 1000.0, 0.01, 0.30)
                               : std::max(0.01, fallback_dt_s);

        const double ticks_to_distance =
            (2.0 * kPi * geometry_.wheel_radius) /
            static_cast<double>(std::max<std::int32_t>(geometry_.encoder_ticks_per_revolution, 1));
        out.left_distance_m = static_cast<double>(out.left_delta_ticks) * ticks_to_distance;
        out.right_distance_m = static_cast<double>(out.right_delta_ticks) * ticks_to_distance;
        out.distance_m = 0.5 * (out.left_distance_m + out.right_distance_m);
        const double odom_delta_yaw =
            std::abs(geometry_.track) > 1e-6 ? (out.right_distance_m - out.left_distance_m) / geometry_.track : 0.0;
        out.odom_speed_mps = out.encoder_dt_s > 1e-6 ? out.distance_m / out.encoder_dt_s : 0.0;
        out.odom_yaw_rate_rps = out.encoder_dt_s > 1e-6 ? odom_delta_yaw / out.encoder_dt_s : 0.0;
        out.have_encoder_delta = true;

        ekf_.predict(out.encoder_dt_s, out.odom_speed_mps, out.odom_yaw_rate_rps);
        ekf_.update_imu(measured_yaw, measured_yaw_rate);
        sync_estimate();
        return out;
    }

    void fuse_lidar_pose(const Pose2D& pose_measurement, bool include_yaw) {
        ekf_.update_lidar_pose({pose_measurement.x, pose_measurement.y}, pose_measurement.yaw, include_yaw);
        sync_estimate();
        if (have_raw_imu_yaw_) {
            yaw_offset_ = wrap_angle(estimate_.pose.yaw - last_raw_imu_yaw_);
            yaw_offset_initialized_ = true;
        }
    }

    const SlamEstimate& estimate() const {
        return estimate_;
    }

    const VehicleGeometry& geometry() const {
        return geometry_;
    }

  private:
    void sync_estimate() {
        const thesis_sim::EkfState& state = ekf_.state();
        estimate_.pose = {state.position.x, state.position.y, state.yaw};
        estimate_.speed = state.speed;
        estimate_.accel = state.accel;
        estimate_.yaw_rate = state.yaw_rate;
        estimate_.curvature = std::abs(state.speed) > 0.05
                                  ? clamp_value(state.yaw_rate / state.speed,
                                                -geometry_.max_curvature,
                                                geometry_.max_curvature)
                                  : 0.0;
        estimate_.localized = true;
    }

    VehicleGeometry geometry_{};
    KinematicBicycleEkf ekf_{};
    SlamEstimate estimate_{};
    double yaw_offset_ = 0.0;
    bool yaw_offset_initialized_ = false;
    bool encoder_ticks_initialized_ = false;
    std::int32_t last_left_ticks_ = 0;
    std::int32_t last_right_ticks_ = 0;
    bool have_raw_imu_yaw_ = false;
    double last_raw_imu_yaw_ = 0.0;
};

class OccupancyGrid {
  public:
    OccupancyGrid(double size_m, double resolution_m)
        : resolution_m_(std::max(0.02, resolution_m)) {
        width_ = std::max(80, static_cast<int>(std::ceil(std::max(size_m, 2.0) / resolution_m_)));
        height_ = width_;
        if ((width_ % 2) == 0) {
            ++width_;
            ++height_;
        }
        origin_world_ = {-0.5 * static_cast<double>(width_) * resolution_m_,
                         -0.5 * static_cast<double>(height_) * resolution_m_};
        log_odds_.assign(static_cast<std::size_t>(width_ * height_), 0.0F);
    }

    bool ready_for_matching() const {
        return observed_updates_ > 400;
    }

    int observed_updates() const {
        return observed_updates_;
    }

    void integrate_scan(const Pose2D& pose,
                        const std::vector<RPLidarA1::ScanPoint>& scan,
                        const MappingConfig& config) {
        int start_x = 0;
        int start_y = 0;
        if (!world_to_cell(lidar_origin_world(pose, config), &start_x, &start_y)) {
            return;
        }

        const int stride = std::max(config.lidar_downsample, 1);
        for (std::size_t i = 0; i < scan.size(); i += static_cast<std::size_t>(stride)) {
            const RPLidarA1::ScanPoint& point = scan[i];
            if (point.distance_m < config.lidar_min_range_m ||
                point.distance_m > config.lidar_max_range_m) {
                continue;
            }

            int end_x = 0;
            int end_y = 0;
            if (!world_to_cell(scan_point_world(pose, point, config), &end_x, &end_y)) {
                continue;
            }
            trace_ray(start_x, start_y, end_x, end_y);
        }
    }

    double score_scan(const Pose2D& pose,
                      const std::vector<RPLidarA1::ScanPoint>& scan,
                      const MappingConfig& config) const {
        if (!ready_for_matching()) {
            return 0.0;
        }

        double score = 0.0;
        int used = 0;
        const int stride = std::max(config.lidar_downsample, 1);
        for (std::size_t i = 0; i < scan.size(); i += static_cast<std::size_t>(stride)) {
            const RPLidarA1::ScanPoint& point = scan[i];
            if (point.distance_m < config.lidar_min_range_m ||
                point.distance_m > config.lidar_max_range_m) {
                continue;
            }

            int cell_x = 0;
            int cell_y = 0;
            if (!world_to_cell(scan_point_world(pose, point, config), &cell_x, &cell_y)) {
                continue;
            }
            score += static_cast<double>(log_odds_[index(cell_x, cell_y)]);
            ++used;
        }

        if (used < 10) {
            return -std::numeric_limits<double>::infinity();
        }
        return score / static_cast<double>(used);
    }

    std::vector<Vec2> extract_occupied_points(std::size_t max_points) const {
        std::vector<Vec2> occupied;
        occupied.reserve(static_cast<std::size_t>(width_ * height_ / 8));
        for (int y = 0; y < height_; ++y) {
            for (int x = 0; x < width_; ++x) {
                if (log_odds_[index(x, y)] <= 0.85F) {
                    continue;
                }
                occupied.push_back(cell_center(x, y));
            }
        }

        if (occupied.size() <= max_points || max_points == 0U) {
            return occupied;
        }

        std::vector<Vec2> sampled;
        sampled.reserve(max_points);
        const std::size_t stride = (occupied.size() / max_points) + 1U;
        for (std::size_t i = 0; i < occupied.size() && sampled.size() < max_points; i += stride) {
            sampled.push_back(occupied[i]);
        }
        return sampled;
    }

    bool save_pgm(const std::string& path, const std::vector<Pose2D>& trail) const {
        std::vector<std::uint8_t> pixels(static_cast<std::size_t>(width_ * height_), 205U);
        for (int y = 0; y < height_; ++y) {
            for (int x = 0; x < width_; ++x) {
                const float value = log_odds_[index(x, y)];
                std::uint8_t pixel = 205U;
                if (value > 0.75F) {
                    pixel = 20U;
                } else if (value < -0.30F) {
                    pixel = 248U;
                }
                pixels[static_cast<std::size_t>((height_ - 1 - y) * width_ + x)] = pixel;
            }
        }

        for (const Pose2D& pose : trail) {
            int cell_x = 0;
            int cell_y = 0;
            if (!world_to_cell(to_vec2(pose), &cell_x, &cell_y)) {
                continue;
            }
            pixels[static_cast<std::size_t>((height_ - 1 - cell_y) * width_ + cell_x)] = 94U;
        }
        if (!trail.empty()) {
            int cell_x = 0;
            int cell_y = 0;
            if (world_to_cell(to_vec2(trail.back()), &cell_x, &cell_y)) {
                pixels[static_cast<std::size_t>((height_ - 1 - cell_y) * width_ + cell_x)] = 255U;
            }
        }

        std::ofstream out(path, std::ios::binary);
        if (!out.is_open()) {
            return false;
        }
        out << "P5\n" << width_ << ' ' << height_ << "\n255\n";
        out.write(reinterpret_cast<const char*>(pixels.data()), static_cast<std::streamsize>(pixels.size()));
        return out.good();
    }

  private:
    int index(int x, int y) const {
        return y * width_ + x;
    }

    Vec2 cell_center(int x, int y) const {
        return {
            origin_world_.x + (static_cast<double>(x) + 0.5) * resolution_m_,
            origin_world_.y + (static_cast<double>(y) + 0.5) * resolution_m_,
        };
    }

    bool world_to_cell(const Vec2& point, int* x, int* y) const {
        const int cell_x = static_cast<int>(std::floor((point.x - origin_world_.x) / resolution_m_));
        const int cell_y = static_cast<int>(std::floor((point.y - origin_world_.y) / resolution_m_));
        if (cell_x < 0 || cell_x >= width_ || cell_y < 0 || cell_y >= height_) {
            return false;
        }
        if (x != nullptr) {
            *x = cell_x;
        }
        if (y != nullptr) {
            *y = cell_y;
        }
        return true;
    }

    void update_cell(int x, int y, float delta) {
        float& cell = log_odds_[index(x, y)];
        cell = std::clamp(cell + delta, -4.0F, 4.0F);
        ++observed_updates_;
    }

    void trace_ray(int x0, int y0, int x1, int y1) {
        int x = x0;
        int y = y0;
        const int dx = std::abs(x1 - x0);
        const int dy = -std::abs(y1 - y0);
        const int step_x = x0 < x1 ? 1 : -1;
        const int step_y = y0 < y1 ? 1 : -1;
        int err = dx + dy;

        while (true) {
            const bool endpoint = (x == x1 && y == y1);
            update_cell(x, y, endpoint ? 0.72F : -0.10F);
            if (endpoint) {
                break;
            }
            const int twice_err = 2 * err;
            if (twice_err >= dy) {
                err += dy;
                x += step_x;
            }
            if (twice_err <= dx) {
                err += dx;
                y += step_y;
            }
        }
    }

    double resolution_m_ = 0.05;
    int width_ = 0;
    int height_ = 0;
    Vec2 origin_world_{};
    std::vector<float> log_odds_;
    int observed_updates_ = 0;
};

