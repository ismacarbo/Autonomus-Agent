#pragma once

#include <Eigen/Dense>

#include "sim_world.h"
#include "vehicle_dynamics.h"

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
};

struct EkfState {
    Vec2 position;
    double yaw = 0.0;
    double speed = 0.0;
    double accel = 0.0;
    double yaw_rate = 0.0;
    Eigen::Matrix<double, 5, 5> covariance = Eigen::Matrix<double, 5, 5>::Identity();
};

class KinematicBicycleEkf {
  public:
    explicit KinematicBicycleEkf(EkfConfig config = {});

    void set_geometry(const VehicleGeometry& geometry);
    void reset(const Vec2& position, double yaw);
    void predict(double dt, double odom_speed, double odom_yaw_rate);
    void update_imu(double measured_yaw, double measured_yaw_rate);
    void update_lidar_pose(const Vec2& measured_position, double measured_yaw, bool include_yaw);
    void override_pose(const Vec2& position, double yaw);

    const EkfState& state() const { return state_; }
    const EkfConfig& config() const { return config_; }

  private:
    static double wrap_angle(double angle);
    void finalize_predict(double odom_speed, double odom_yaw_rate, double dt);

    EkfConfig config_;
    VehicleGeometry geometry_{};
    EkfState state_{};
};

}  // namespace thesis_sim
