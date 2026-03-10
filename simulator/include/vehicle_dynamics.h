#pragma once

#include <memory>
#include <string>

#include <Eigen/Dense>

#include "sim_world.h"

namespace thesis_sim {

struct VehicleGeometry {
    double wheelbase = 1.8;
    double cg_to_front = 0.95;
    double cg_to_rear = 0.85;
    double track = 1.2;
    double body_length = 2.7;
    double body_width = 1.55;
    double wheel_length = 0.38;
    double wheel_width = 0.16;
    double max_steer_angle = 0.6;
    double max_curvature = 0.7;
};

struct VehicleModelState {
    Vec2 position;
    double yaw = 0.0;
    double speed = 0.0;
    double accel = 0.0;
    double steer_angle = 0.0;
    double curvature = 0.0;
    double yaw_rate = 0.0;
    double sideslip = 0.0;
    Eigen::VectorXd internal_state;
};

class VehicleDynamicsModel {
  public:
    virtual ~VehicleDynamicsModel() = default;

    virtual void reset(const Vec2& position, double yaw) = 0;
    virtual void step(double dt, double jerk_cmd, double planner_r_cmd) = 0;

    virtual const VehicleGeometry& geometry() const = 0;
    virtual const VehicleModelState& state() const = 0;
    virtual const Eigen::MatrixXd& A() const = 0;
    virtual const Eigen::MatrixXd& B() const = 0;
    virtual std::string name() const = 0;
};

std::unique_ptr<VehicleDynamicsModel> make_four_wheel_car_model(const VehicleGeometry& geometry = {});

}  // namespace thesis_sim
