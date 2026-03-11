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

class DifferentialDriveRobotModel final : public VehicleDynamicsModel {
  public:
    explicit DifferentialDriveRobotModel(VehicleGeometry geometry)
        : geometry_(std::move(geometry)),
          A_(Eigen::MatrixXd::Identity(7, 7)),
          B_(Eigen::MatrixXd::Zero(7, 2)),
          state_vector_(Eigen::VectorXd::Zero(7)) {
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

    void step(double dt, double jerk_cmd, double planner_r_cmd) override {
        if (dt <= 0.0) {
            update_model_state(0.0);
            return;
        }

        update_matrices(dt);

        double& speed_ref = state_vector_(0);
        double& accel_ref = state_vector_(1);
        double& curvature_ref = state_vector_(2);
        double& pwm_left_state = state_vector_(3);
        double& pwm_right_state = state_vector_(4);
        double& wheel_speed_left = state_vector_(5);
        double& wheel_speed_right = state_vector_(6);

        accel_ref = clamp_value(accel_ref + jerk_cmd * dt, -2.0, 1.5);
        speed_ref = clamp_value(speed_ref + accel_ref * dt, 0.0, geometry_.max_linear_speed);

        const double curvature_speed = std::max(speed_ref, 0.05);
        curvature_ref = clamp_value(
            curvature_ref + curvature_speed * planner_r_cmd * dt,
            -geometry_.max_curvature,
            geometry_.max_curvature);

        const double yaw_rate_target = clamp_value(
            speed_ref * curvature_ref,
            -geometry_.max_yaw_rate,
            geometry_.max_yaw_rate);

        const double half_track = geometry_.track * 0.5;
        const double target_left_speed = speed_ref - yaw_rate_target * half_track;
        const double target_right_speed = speed_ref + yaw_rate_target * half_track;

        const double current_body_speed = 0.5 * (wheel_speed_left + wheel_speed_right);
        const double current_yaw_rate = (wheel_speed_right - wheel_speed_left) / geometry_.track;
        const int ff_left_pwm = wheel_speed_to_pwm(target_left_speed, geometry_.left_pwm_scale);
        const int ff_right_pwm = wheel_speed_to_pwm(target_right_speed, geometry_.right_pwm_scale);
        const int fb_linear_pwm = static_cast<int>(std::lround(
            geometry_.linear_feedback_gain * (speed_ref - current_body_speed)));
        const int fb_yaw_pwm = static_cast<int>(std::lround(
            geometry_.yaw_feedback_gain * (yaw_rate_target - current_yaw_rate)));
        const int target_left_pwm = static_cast<int>(clamp_value(
            static_cast<double>(ff_left_pwm + fb_linear_pwm - fb_yaw_pwm),
            -geometry_.max_pwm,
            geometry_.max_pwm));
        const int target_right_pwm = static_cast<int>(clamp_value(
            static_cast<double>(ff_right_pwm + fb_linear_pwm + fb_yaw_pwm),
            -geometry_.max_pwm,
            geometry_.max_pwm));

        pwm_left_state = approach_pwm(pwm_left_state, static_cast<double>(target_left_pwm), dt);
        pwm_right_state = approach_pwm(pwm_right_state, static_cast<double>(target_right_pwm), dt);

        const double alpha = 1.0 - std::exp(-dt / std::max(geometry_.motor_time_constant, 1e-3));
        wheel_speed_left += (pwm_to_speed(static_cast<int>(std::lround(pwm_left_state))) - wheel_speed_left) * alpha;
        wheel_speed_right += (pwm_to_speed(static_cast<int>(std::lround(pwm_right_state))) - wheel_speed_right) * alpha;

        const double previous_speed = model_state_.speed;
        const double body_speed = 0.5 * (wheel_speed_left + wheel_speed_right);
        const double yaw_rate = (wheel_speed_right - wheel_speed_left) / geometry_.track;

        position_.x += body_speed * std::cos(yaw_) * dt;
        position_.y += body_speed * std::sin(yaw_) * dt;
        yaw_ = wrap_angle(yaw_ + yaw_rate * dt);

        const double wheel_circumference = 2.0 * kPi * geometry_.wheel_radius;
        raw_left_ticks_ += (wheel_speed_left * dt / wheel_circumference) *
                           static_cast<double>(geometry_.encoder_ticks_per_revolution);
        raw_right_ticks_ += (wheel_speed_right * dt / wheel_circumference) *
                            static_cast<double>(geometry_.encoder_ticks_per_revolution);

        const std::int32_t left_ticks_now = static_cast<std::int32_t>(std::llround(raw_left_ticks_));
        const std::int32_t right_ticks_now = static_cast<std::int32_t>(std::llround(raw_right_ticks_));
        left_tick_delta_ = left_ticks_now - left_ticks_total_;
        right_tick_delta_ = right_ticks_now - right_ticks_total_;
        left_ticks_total_ = left_ticks_now;
        right_ticks_total_ = right_ticks_now;

        const double accel = (body_speed - previous_speed) / dt;
        update_model_state(dt * 1000.0, accel, yaw_rate, target_left_pwm, target_right_pwm);
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
        return "Differential Drive Robot";
    }

  private:
    void update_matrices(double dt) {
        A_.setIdentity();
        B_.setZero();

        A_(0, 1) = dt;
        B_(0, 0) = 0.5 * dt * dt;
        B_(1, 0) = dt;
        B_(2, 1) = std::max(state_vector_(0), 0.05) * dt;

        const double pwm_alpha = clamp_value(dt * geometry_.pwm_slew_rate / std::max(1.0, static_cast<double>(geometry_.max_pwm)), 0.0, 1.0);
        A_(3, 3) = 1.0 - pwm_alpha;
        A_(4, 4) = 1.0 - pwm_alpha;

        const double motor_alpha = 1.0 - std::exp(-dt / std::max(geometry_.motor_time_constant, 1e-3));
        A_(5, 5) = 1.0 - motor_alpha;
        A_(6, 6) = 1.0 - motor_alpha;
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

    int wheel_speed_to_pwm(double wheel_speed_mps, double scale) const {
        if (std::abs(wheel_speed_mps) < 1e-4) {
            return 0;
        }

        double pwm = geometry_.wheel_speed_to_pwm_bias +
                     std::abs(wheel_speed_mps) * geometry_.wheel_speed_to_pwm_gain;
        pwm *= scale;
        pwm = clamp_value(
            pwm,
            static_cast<double>(geometry_.min_effective_pwm),
            static_cast<double>(geometry_.max_pwm));
        return static_cast<int>(std::lround(sign_of(wheel_speed_mps) * pwm));
    }

    double pwm_to_speed(int pwm) const {
        if (std::abs(pwm) < geometry_.min_effective_pwm) {
            return 0.0;
        }

        const double sign = pwm >= 0 ? 1.0 : -1.0;
        const double magnitude = std::max(
            0.0,
            static_cast<double>(std::abs(pwm) - geometry_.min_effective_pwm));
        return sign * magnitude * geometry_.speed_estimate_per_pwm;
    }

    void update_model_state(double encoder_dt_ms,
                            double accel_override = 0.0,
                            double yaw_rate_override = 0.0,
                            int target_left_pwm = 0,
                            int target_right_pwm = 0) {
        const double wheel_speed_left = state_vector_(5);
        const double wheel_speed_right = state_vector_(6);
        const double body_speed = 0.5 * (wheel_speed_left + wheel_speed_right);
        const double yaw_rate = yaw_rate_override;

        model_state_.position = position_;
        model_state_.yaw = yaw_;
        model_state_.speed = body_speed;
        model_state_.accel = accel_override;
        model_state_.steer_angle = 0.0;
        model_state_.sideslip = 0.0;
        model_state_.yaw_rate = yaw_rate;
        model_state_.curvature = std::abs(body_speed) > 0.03
                                     ? clamp_value(yaw_rate / body_speed,
                                                   -geometry_.max_curvature,
                                                   geometry_.max_curvature)
                                     : 0.0;
        model_state_.left_wheel_speed = wheel_speed_left;
        model_state_.right_wheel_speed = wheel_speed_right;
        model_state_.target_speed = state_vector_(0);
        model_state_.target_yaw_rate = clamp_value(
            state_vector_(0) * state_vector_(2),
            -geometry_.max_yaw_rate,
            geometry_.max_yaw_rate);
        model_state_.left_encoder_ticks = left_ticks_total_;
        model_state_.right_encoder_ticks = right_ticks_total_;
        model_state_.left_encoder_delta = left_tick_delta_;
        model_state_.right_encoder_delta = right_tick_delta_;
        model_state_.left_pwm = static_cast<int>(std::lround(state_vector_(3)));
        model_state_.right_pwm = static_cast<int>(std::lround(state_vector_(4)));
        model_state_.encoder_dt_ms = encoder_dt_ms;
        model_state_.internal_state = state_vector_;

        (void)target_left_pwm;
        (void)target_right_pwm;
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

std::unique_ptr<VehicleDynamicsModel> make_differential_drive_robot_model(const VehicleGeometry& geometry) {
    return std::make_unique<DifferentialDriveRobotModel>(geometry);
}

std::unique_ptr<VehicleDynamicsModel> make_four_wheel_car_model(const VehicleGeometry& geometry) {
    return make_differential_drive_robot_model(geometry);
}

}  // namespace thesis_sim
