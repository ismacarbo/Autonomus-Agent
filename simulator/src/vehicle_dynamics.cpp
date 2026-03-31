#include "vehicle_dynamics.h"

#include <algorithm>
#include <cmath>
#include <utility>

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

double sign_of(double value) {
    if (value > 0.0) {
        return 1.0;
    }
    if (value < 0.0) {
        return -1.0;
    }
    return 0.0;
}

class CarLikeBicycleModel final : public VehicleDynamicsModel {
  public:
    explicit CarLikeBicycleModel(VehicleGeometry geometry)
        : geometry_(std::move(geometry)),
          A_(Eigen::MatrixXd::Identity(6, 6)),
          B_(Eigen::MatrixXd::Zero(6, 2)),
          state_vector_(Eigen::VectorXd::Zero(6)) {
        model_state_.internal_state = state_vector_;
    }

    void reset(const Vec2& position, double yaw) override {
        position_ = position;
        yaw_ = yaw;
        state_vector_.setZero();
        raw_left_ticks_ = 0.0;
        raw_right_ticks_ = 0.0;
        left_ticks_total_ = 0;
        right_ticks_total_ = 0;
        left_tick_delta_ = 0;
        right_tick_delta_ = 0;
        update_model_state(0.0);
    }

    void step(double dt, const VehicleControlInput& input) override {
        if (dt <= 0.0) {
            update_model_state(0.0);
            return;
        }

        update_matrices(dt);

        double& speed = state_vector_(0);
        double& accel = state_vector_(1);
        double& steer_angle = state_vector_(2);
        double& drive_pwm = state_vector_(3);
        double& rear_left_speed = state_vector_(4);
        double& rear_right_speed = state_vector_(5);

        const double bounded_accel = clamp_value(
            input.accel_cmd,
            -geometry_.max_decel,
            geometry_.max_accel);
        const double bounded_steer_rate = clamp_value(
            input.steer_rate_cmd,
            -geometry_.max_steer_rate,
            geometry_.max_steer_rate);

        accel = bounded_accel;
        speed = clamp_value(speed + accel * dt, 0.0, geometry_.max_linear_speed);
        steer_angle = clamp_value(
            steer_angle + bounded_steer_rate * dt,
            -geometry_.max_steer_angle,
            geometry_.max_steer_angle);

        const double beta = std::atan2(
            geometry_.cg_to_rear * std::tan(steer_angle),
            std::max(geometry_.wheelbase, 1e-6));
        const double yaw_rate = speed * std::cos(beta) * std::tan(steer_angle) /
                                std::max(geometry_.wheelbase, 1e-6);
        const double curvature = std::abs(speed) > 1e-4 ? yaw_rate / speed : 0.0;

        position_.x += speed * std::cos(yaw_ + beta) * dt;
        position_.y += speed * std::sin(yaw_ + beta) * dt;
        yaw_ = wrap_angle(yaw_ + yaw_rate * dt);

        const double half_track = geometry_.track * 0.5;
        rear_left_speed = speed * (1.0 - half_track * curvature);
        rear_right_speed = speed * (1.0 + half_track * curvature);

        const double wheel_circumference = 2.0 * kPi * geometry_.wheel_radius;
        raw_left_ticks_ += (rear_left_speed * dt / wheel_circumference) *
                           static_cast<double>(geometry_.encoder_ticks_per_revolution);
        raw_right_ticks_ += (rear_right_speed * dt / wheel_circumference) *
                            static_cast<double>(geometry_.encoder_ticks_per_revolution);

        const std::int32_t left_ticks_now = static_cast<std::int32_t>(std::llround(raw_left_ticks_));
        const std::int32_t right_ticks_now = static_cast<std::int32_t>(std::llround(raw_right_ticks_));
        left_tick_delta_ = left_ticks_now - left_ticks_total_;
        right_tick_delta_ = right_ticks_now - right_ticks_total_;
        left_ticks_total_ = left_ticks_now;
        right_ticks_total_ = right_ticks_now;

        const int target_pwm = wheel_speed_to_pwm(speed);
        drive_pwm = approach_pwm(drive_pwm, static_cast<double>(target_pwm), dt);

        update_model_state(
            dt * 1000.0,
            yaw_rate,
            curvature,
            input.target_speed,
            input.target_steer_angle);
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
        return "Car-Like Bicycle";
    }

  private:
    void update_matrices(double dt) {
        A_.setIdentity();
        B_.setZero();
        A_(0, 1) = dt;
        A_(2, 2) = 1.0;
        B_(0, 0) = 0.5 * dt * dt;
        B_(1, 0) = dt;
        B_(2, 1) = dt;
    }

    double approach_pwm(double current, double target, double dt) const {
        const double max_delta = geometry_.pwm_slew_rate * dt;
        if (target > current) {
            return std::min(current + max_delta, target);
        }
        if (target < current) {
            return std::max(current - max_delta, target);
        }
        return current;
    }

    int wheel_speed_to_pwm(double speed_mps) const {
        if (std::abs(speed_mps) < 1e-4) {
            return 0;
        }
        const int magnitude = geometry_.max_pwm;
        return static_cast<int>(std::lround(sign_of(speed_mps) * static_cast<double>(magnitude)));
    }

    void update_model_state(double encoder_dt_ms,
                            double yaw_rate_override = 0.0,
                            double curvature_override = 0.0,
                            double target_speed_override = 0.0,
                            double target_steer_angle_override = 0.0) {
        model_state_.position = position_;
        model_state_.yaw = yaw_;
        model_state_.speed = state_vector_(0);
        model_state_.accel = state_vector_(1);
        model_state_.steer_angle = state_vector_(2);
        model_state_.sideslip = std::atan2(
            geometry_.cg_to_rear * std::tan(state_vector_(2)),
            std::max(geometry_.wheelbase, 1e-6));
        model_state_.yaw_rate = yaw_rate_override;
        model_state_.curvature = curvature_override;
        model_state_.left_wheel_speed = state_vector_(4);
        model_state_.right_wheel_speed = state_vector_(5);
        model_state_.target_speed = target_speed_override;
        model_state_.target_steer_angle = target_steer_angle_override;
        model_state_.target_yaw_rate = target_speed_override * std::tan(target_steer_angle_override) /
                                       std::max(geometry_.wheelbase, 1e-6);
        model_state_.left_encoder_ticks = left_ticks_total_;
        model_state_.right_encoder_ticks = right_ticks_total_;
        model_state_.left_encoder_delta = left_tick_delta_;
        model_state_.right_encoder_delta = right_tick_delta_;
        model_state_.left_pwm = static_cast<int>(std::lround(state_vector_(3)));
        model_state_.right_pwm = static_cast<int>(std::lround(state_vector_(3)));
        model_state_.encoder_dt_ms = encoder_dt_ms;
        model_state_.internal_state = state_vector_;
    }

    VehicleGeometry geometry_;
    Eigen::MatrixXd A_;
    Eigen::MatrixXd B_;
    Eigen::VectorXd state_vector_;
    VehicleModelState model_state_;
    Vec2 position_{};
    double yaw_ = 0.0;
    double raw_left_ticks_ = 0.0;
    double raw_right_ticks_ = 0.0;
    std::int32_t left_ticks_total_ = 0;
    std::int32_t right_ticks_total_ = 0;
    std::int32_t left_tick_delta_ = 0;
    std::int32_t right_tick_delta_ = 0;
};

}  // namespace

const char* vehicle_model_kind_name(VehicleModelKind kind) {
    switch (kind) {
        case VehicleModelKind::CarLikeBicycle:
            return "Car-Like Bicycle";
        default:
            return "Unknown";
    }
}

std::unique_ptr<VehicleDynamicsModel> make_four_wheel_car_model(const VehicleGeometry& geometry) {
    return std::make_unique<CarLikeBicycleModel>(geometry);
}

}  // namespace thesis_sim
