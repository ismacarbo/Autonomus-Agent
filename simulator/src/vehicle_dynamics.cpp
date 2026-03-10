#include "vehicle_dynamics.h"

#include <algorithm>
#include <cmath>

namespace thesis_sim {

namespace {

constexpr double kPi = 3.14159265358979323846;

double clamp_value(double value, double lo, double hi) {
    return std::max(lo, std::min(value, hi));
}

double wrap_angle(double angle) {
    while (angle > kPi) {
        angle -= 2.0 * kPi;
    }
    while (angle < -kPi) {
        angle += 2.0 * kPi;
    }
    return angle;
}

class FourWheelCarModel final : public VehicleDynamicsModel {
  public:
    explicit FourWheelCarModel(VehicleGeometry geometry)
        : geometry_(std::move(geometry)),
          A_(Eigen::MatrixXd::Identity(4, 4)),
          B_(Eigen::MatrixXd::Zero(4, 2)),
          state_vector_(Eigen::VectorXd::Zero(4)) {
        model_state_.internal_state = state_vector_;
    }

    void reset(const Vec2& position, double yaw) override {
        position_ = position;
        yaw_ = yaw;
        curvature_ref_ = 0.0;
        state_vector_.setZero();
        update_model_state();
    }

    void step(double dt, double jerk_cmd, double planner_r_cmd) override {
        update_matrices(dt);

        const double speed = std::max(0.0, state_vector_(0));
        curvature_ref_ = clamp_value(
            curvature_ref_ + speed * planner_r_cmd * dt,
            -geometry_.max_curvature,
            geometry_.max_curvature);

        const double delta_ref = clamp_value(
            std::atan(curvature_ref_ * geometry_.wheelbase),
            -geometry_.max_steer_angle,
            geometry_.max_steer_angle);

        Eigen::Vector2d input;
        input << jerk_cmd, delta_ref;

        state_vector_ = A_ * state_vector_ + B_ * input;
        state_vector_(0) = std::max(0.0, state_vector_(0));
        state_vector_(1) = clamp_value(state_vector_(1), -3.5, 2.5);
        state_vector_(2) = clamp_value(state_vector_(2), -geometry_.max_steer_angle, geometry_.max_steer_angle);
        state_vector_(3) = clamp_value(state_vector_(3), -2.5, 2.5);

        const double steer = state_vector_(2);
        const double beta = std::atan((geometry_.cg_to_rear / geometry_.wheelbase) * std::tan(steer));
        const double yaw_rate = state_vector_(0) * std::cos(beta) * std::tan(steer) / geometry_.wheelbase;

        position_.x += state_vector_(0) * std::cos(yaw_ + beta) * dt;
        position_.y += state_vector_(0) * std::sin(yaw_ + beta) * dt;
        yaw_ = wrap_angle(yaw_ + yaw_rate * dt);

        update_model_state();
    }

    const VehicleGeometry& geometry() const override {
        return geometry_;
    }

    const VehicleModelState& state() const override {
        return model_state_;
    }

    const Eigen::MatrixXd& A() const override {
        return A_;
    }

    const Eigen::MatrixXd& B() const override {
        return B_;
    }

    std::string name() const override {
        return "4WD Car State-Space";
    }

  private:
    void update_matrices(double dt) {
        constexpr double steer_nat_freq = 12.0;
        constexpr double steer_damping = 1.0;

        A_.setZero();
        B_.setZero();

        A_(0, 0) = 1.0;
        A_(0, 1) = dt;
        A_(1, 1) = 1.0;
        B_(0, 0) = 0.5 * dt * dt;
        B_(1, 0) = dt;

        A_(2, 2) = 1.0;
        A_(2, 3) = dt;
        A_(3, 2) = -dt * steer_nat_freq * steer_nat_freq;
        A_(3, 3) = 1.0 - 2.0 * steer_damping * steer_nat_freq * dt;
        B_(3, 1) = dt * steer_nat_freq * steer_nat_freq;
    }

    void update_model_state() {
        model_state_.position = position_;
        model_state_.yaw = yaw_;
        model_state_.speed = state_vector_(0);
        model_state_.accel = state_vector_(1);
        model_state_.steer_angle = state_vector_(2);
        model_state_.sideslip = std::atan((geometry_.cg_to_rear / geometry_.wheelbase) * std::tan(model_state_.steer_angle));
        model_state_.yaw_rate = model_state_.speed * std::cos(model_state_.sideslip) * std::tan(model_state_.steer_angle) / geometry_.wheelbase;
        model_state_.curvature = std::tan(model_state_.steer_angle) / geometry_.wheelbase;
        model_state_.internal_state = state_vector_;
    }

    VehicleGeometry geometry_;
    Eigen::MatrixXd A_;
    Eigen::MatrixXd B_;
    Eigen::VectorXd state_vector_;
    VehicleModelState model_state_;
    Vec2 position_{};
    double yaw_ = 0.0;
    double curvature_ref_ = 0.0;
};

}  // namespace

std::unique_ptr<VehicleDynamicsModel> make_four_wheel_car_model(const VehicleGeometry& geometry) {
    return std::make_unique<FourWheelCarModel>(geometry);
}

}  // namespace thesis_sim
