#pragma once

#include <Eigen/Dense>

#include "mvc/model/navigation/vehicle_dynamics.h"
#include "mvc/model/world/world.h"

namespace thesis_sim {

struct EkfConfig {
    double q_x = 0.02;
    double q_y = 0.02;
    double q_yaw = 0.01;
    double q_v = 0.08;
    double q_yaw_rate = 0.08;
    double r_imu_yaw = 0.02;
    double r_imu_yaw_rate = 0.03;
    double r_lidar_x = 0.08;
    double r_lidar_y = 0.08;
    double r_lidar_yaw = 0.05;
    double lidar_position_gate_chi2 = 9.21;
    double lidar_pose_gate_chi2 = 11.34;
    double gyro_bias_learning_rate = 0.015;
    double gyro_bias_limit_rad_s = 0.25;
    double turn_process_noise_gain = 1.5;
    double tracked_turn_process_noise_gain = 3.5;
};

struct EkfState {
    Vec2 position;
    double yaw = 0.0;
    double speed = 0.0;
    double accel = 0.0;
    double yaw_rate = 0.0;
    double gyro_bias = 0.0;
    Eigen::Matrix<double, 5, 5> covariance = Eigen::Matrix<double, 5, 5>::Identity();
};

class KinematicBicycleEkf {
  public:
    explicit KinematicBicycleEkf(EkfConfig config = {});

    void set_geometry(const VehicleGeometry& geometry);
    void set_tracked_vehicle(bool tracked) { tracked_vehicle_ = tracked; }
    void reset(const Vec2& position, double yaw);
    void predict(double dt, double odom_speed, double odom_yaw_rate);
    void update_imu(double measured_yaw, double measured_yaw_rate);
    void update_lidar_pose(const Vec2& measured_position, double measured_yaw, bool include_yaw);
    void override_pose(const Vec2& position, double yaw);

    const EkfState& state() const { return state_; }
    const EkfConfig& config() const { return config_; }
    int lidar_updates_accepted() const { return lidar_updates_accepted_; }
    int lidar_updates_rejected() const { return lidar_updates_rejected_; }
    int consecutive_lidar_rejections() const { return consecutive_lidar_rejections_; }
    bool lidar_recovery_recommended() const { return consecutive_lidar_rejections_ >= 5; }

  private:
    static double wrap_angle(double angle);
    void finalize_predict(double odom_speed, double odom_yaw_rate, double dt);

    EkfConfig config_;
    VehicleGeometry geometry_{};
    EkfState state_{};
    bool tracked_vehicle_ = false;
    double last_odom_speed_ = 0.0;
    double last_odom_yaw_rate_ = 0.0;
    int lidar_updates_accepted_ = 0;
    int lidar_updates_rejected_ = 0;
    int consecutive_lidar_rejections_ = 0;
};

}  // namespace thesis_sim
