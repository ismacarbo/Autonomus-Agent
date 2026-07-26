#include "mvc/model/navigation/vehicle_dynamics.h"

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

double bounded_yaw_response_scale(double value) {
    return clamp_value(value, 0.05, 2.0);
}

class CarLikeBicycleModel final : public VehicleDynamicsModel {
  public:
    explicit CarLikeBicycleModel(VehicleGeometry geometry)
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

    void step(double dt, const VehicleControlInput& input) override {
        if (dt <= 0.0) {
            update_model_state(0.0);
            return;
        }

        update_matrices(dt);

        double& speed = state_vector_(0);
        double& accel = state_vector_(1);
        double& steer_angle = state_vector_(2);
        double& left_pwm = state_vector_(3);
        double& right_pwm = state_vector_(4);
        double& rear_left_speed = state_vector_(5);
        double& rear_right_speed = state_vector_(6);

        const double bounded_accel = clamp_value(
            input.accel_cmd,
            -geometry_.max_decel,
            geometry_.max_accel);
        const double bounded_steer_rate = clamp_value(
            input.steer_rate_cmd,
            -geometry_.max_steer_rate,
            geometry_.max_steer_rate);
        const double previous_speed = speed;

        accel = 0.0;
        steer_angle = clamp_value(
            steer_angle + bounded_steer_rate * dt,
            -geometry_.max_steer_angle,
            geometry_.max_steer_angle);

        const double half_track = geometry_.track * 0.5;
        const double requested_target_steer = clamp_value(
            std::isfinite(input.target_steer_angle)
                ? input.target_steer_angle
                : std::atan(std::max(geometry_.wheelbase, 1e-6) * input.target_curvature),
            -geometry_.max_steer_angle,
            geometry_.max_steer_angle);
        const double accel_based_target_speed =
            clamp_value(previous_speed + bounded_accel * dt,
                        -geometry_.max_linear_speed,
                        geometry_.max_linear_speed);
        const bool target_speed_valid =
            std::isfinite(input.target_speed) && std::abs(input.target_speed) > 1e-4;
        const double requested_target_speed =
            target_speed_valid ? input.target_speed : accel_based_target_speed;
        const double target_speed = clamp_value(
            requested_target_speed,
            -geometry_.max_linear_speed,
            geometry_.max_linear_speed);
        const double target_curvature = clamp_value(
            std::tan(steer_angle) / std::max(geometry_.wheelbase, 1e-6),
            -geometry_.max_curvature,
            geometry_.max_curvature);
        const double target_yaw_rate = clamp_value(
            target_speed * target_curvature,
            -geometry_.max_yaw_rate,
            geometry_.max_yaw_rate);
        const double target_left_wheel_speed = target_speed - target_yaw_rate * half_track;
        const double target_right_wheel_speed = target_speed + target_yaw_rate * half_track;

        const int ff_left_pwm = wheel_speed_to_pwm(target_left_wheel_speed, geometry_.left_pwm_scale);
        const int ff_right_pwm = wheel_speed_to_pwm(target_right_wheel_speed, geometry_.right_pwm_scale);
        const double yaw_response_scale = bounded_yaw_response_scale(geometry_.yaw_response_scale);
        const double raw_measured_yaw_rate =
            std::abs(geometry_.track) > 1e-6 ? (rear_right_speed - rear_left_speed) / geometry_.track : 0.0;
        const double measured_yaw_rate_for_feedback = raw_measured_yaw_rate * yaw_response_scale;
        const double measured_speed_for_feedback =
            clamp_value(previous_speed,
                        -std::max(geometry_.max_linear_speed * 1.35, 0.28),
                        std::max(geometry_.max_linear_speed * 1.35, 0.28));
        const int fb_linear = static_cast<int>(std::lround(
            geometry_.linear_feedback_gain * (target_speed - measured_speed_for_feedback)));
        const int fb_yaw = static_cast<int>(std::lround(
            geometry_.yaw_feedback_gain * (target_yaw_rate - measured_yaw_rate_for_feedback)));
        const int commanded_left_pwm = static_cast<int>(std::lround(clamp_value(
            static_cast<double>(ff_left_pwm + fb_linear - fb_yaw),
            -static_cast<double>(geometry_.max_pwm),
            static_cast<double>(geometry_.max_pwm))));
        const int commanded_right_pwm = static_cast<int>(std::lround(clamp_value(
            static_cast<double>(ff_right_pwm + fb_linear + fb_yaw),
            -static_cast<double>(geometry_.max_pwm),
            static_cast<double>(geometry_.max_pwm))));
        left_pwm = approach_pwm(left_pwm, static_cast<double>(commanded_left_pwm), dt);
        right_pwm = approach_pwm(right_pwm, static_cast<double>(commanded_right_pwm), dt);

        const double wheel_alpha = 1.0 - std::exp(-dt / std::max(geometry_.motor_time_constant, 1e-3));
        const double left_target_from_pwm =
            wheel_speed_from_pwm(static_cast<int>(std::lround(left_pwm)), geometry_.left_pwm_scale);
        const double right_target_from_pwm =
            wheel_speed_from_pwm(static_cast<int>(std::lround(right_pwm)), geometry_.right_pwm_scale);
        rear_left_speed += wheel_alpha * (left_target_from_pwm - rear_left_speed);
        rear_right_speed += wheel_alpha * (right_target_from_pwm - rear_right_speed);

        speed = clamp_value(
            0.5 * (rear_left_speed + rear_right_speed),
            -geometry_.max_linear_speed,
            geometry_.max_linear_speed);
        const double raw_yaw_rate =
            std::abs(geometry_.track) > 1e-6 ? (rear_right_speed - rear_left_speed) / geometry_.track : 0.0;
        const double yaw_rate = raw_yaw_rate * yaw_response_scale;
        const double curvature = std::abs(speed) > 1e-4 ? yaw_rate / speed : 0.0;
        accel = clamp_value(
            (speed - previous_speed) / std::max(dt, 1e-6),
            -geometry_.max_decel,
            geometry_.max_accel);

        const double delta_yaw = yaw_rate * dt;
        const double mid_yaw = wrap_angle(yaw_ + 0.5 * delta_yaw);
        position_.x += speed * std::cos(mid_yaw) * dt;
        position_.y += speed * std::sin(mid_yaw) * dt;
        yaw_ = wrap_angle(yaw_ + delta_yaw);

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

        update_model_state(
            dt * 1000.0,
            yaw_rate,
            curvature,
            target_speed,
            requested_target_steer,
            target_yaw_rate);
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

    double wheel_speed_from_pwm(int pwm, double scale) const {
        const double effective_pwm = std::abs(static_cast<double>(pwm));
        if (effective_pwm < static_cast<double>(geometry_.min_effective_pwm) ||
            geometry_.speed_estimate_per_pwm <= 0.0) {
            return 0.0;
        }
        const double scaled_magnitude =
            std::max(0.0, effective_pwm - static_cast<double>(geometry_.min_effective_pwm));
        const double signed_speed =
            scaled_magnitude * geometry_.speed_estimate_per_pwm * std::max(std::abs(scale), 1e-3);
        return std::copysign(signed_speed, static_cast<double>(pwm));
    }

    int wheel_speed_to_pwm(double speed_mps, double scale) const {
        if (std::abs(speed_mps) < 1e-4) {
            return 0;
        }
        const double scaled_speed = std::abs(speed_mps) * std::max(std::abs(scale), 1e-3);
        const double pwm =
            static_cast<double>(geometry_.min_effective_pwm) + geometry_.wheel_speed_to_pwm_bias +
            geometry_.wheel_speed_to_pwm_gain * scaled_speed;
        const int sign = speed_mps >= 0.0 ? 1 : -1;
        return sign * static_cast<int>(std::lround(clamp_value(
                          pwm,
                          static_cast<double>(geometry_.min_effective_pwm),
                          static_cast<double>(geometry_.max_pwm))));
    }

    void update_model_state(double encoder_dt_ms,
                            double yaw_rate_override = 0.0,
                            double curvature_override = 0.0,
                            double target_speed_override = 0.0,
                            double target_steer_angle_override = 0.0,
                            double target_yaw_rate_override = 0.0) {
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
        model_state_.left_wheel_speed = state_vector_(5);
        model_state_.right_wheel_speed = state_vector_(6);
        model_state_.target_speed = target_speed_override;
        model_state_.target_steer_angle = target_steer_angle_override;
        model_state_.target_yaw_rate = target_yaw_rate_override;
        model_state_.left_encoder_ticks = left_ticks_total_;
        model_state_.right_encoder_ticks = right_ticks_total_;
        model_state_.left_encoder_delta = left_tick_delta_;
        model_state_.right_encoder_delta = right_tick_delta_;
        model_state_.left_pwm = static_cast<int>(std::lround(state_vector_(3)));
        model_state_.right_pwm = static_cast<int>(std::lround(state_vector_(4)));
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

class TrackedVehicleModel final : public VehicleDynamicsModel {
  public:
    explicit TrackedVehicleModel(VehicleGeometry geometry)
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
        double& yaw_rate = state_vector_(1);
        double& left_pwm = state_vector_(2);
        double& right_pwm = state_vector_(3);
        double& left_track_speed = state_vector_(4);
        double& right_track_speed = state_vector_(5);

        const double previous_speed = speed;
        const double bounded_accel = clamp_value(
            input.accel_cmd,
            -geometry_.max_decel,
            geometry_.max_accel);
        const double accel_limited_speed = clamp_value(
            previous_speed + bounded_accel * dt,
            -geometry_.max_linear_speed,
            geometry_.max_linear_speed);
        const double requested_target_speed =
            std::isfinite(input.target_speed) ? input.target_speed : accel_limited_speed;
        const double target_speed = clamp_value(
            requested_target_speed,
            -geometry_.max_linear_speed,
            geometry_.max_linear_speed);
        const double curvature_yaw_rate =
            std::abs(target_speed) > 1e-4 ? target_speed * input.target_curvature : 0.0;
        const double requested_target_yaw_rate =
            std::isfinite(input.target_yaw_rate) ? input.target_yaw_rate : curvature_yaw_rate;
        const double target_yaw_rate = clamp_value(
            requested_target_yaw_rate,
            -geometry_.max_yaw_rate,
            geometry_.max_yaw_rate);

        const double half_track = geometry_.track * 0.5;
        const double target_left_track_speed = target_speed - target_yaw_rate * half_track;
        const double target_right_track_speed = target_speed + target_yaw_rate * half_track;

        const int ff_left_pwm = wheel_speed_to_pwm(target_left_track_speed, geometry_.left_pwm_scale);
        const int ff_right_pwm = wheel_speed_to_pwm(target_right_track_speed, geometry_.right_pwm_scale);
        const int fb_linear = static_cast<int>(std::lround(
            geometry_.linear_feedback_gain * (target_speed - previous_speed)));
        const int fb_yaw = static_cast<int>(std::lround(
            geometry_.yaw_feedback_gain * (target_yaw_rate - yaw_rate)));
        const int commanded_left_pwm = static_cast<int>(std::lround(clamp_value(
            static_cast<double>(ff_left_pwm + fb_linear - fb_yaw),
            -static_cast<double>(geometry_.max_pwm),
            static_cast<double>(geometry_.max_pwm))));
        const int commanded_right_pwm = static_cast<int>(std::lround(clamp_value(
            static_cast<double>(ff_right_pwm + fb_linear + fb_yaw),
            -static_cast<double>(geometry_.max_pwm),
            static_cast<double>(geometry_.max_pwm))));

        left_pwm = approach_pwm(left_pwm, static_cast<double>(commanded_left_pwm), dt);
        right_pwm = approach_pwm(right_pwm, static_cast<double>(commanded_right_pwm), dt);

        const double wheel_alpha = 1.0 - std::exp(-dt / std::max(geometry_.motor_time_constant, 1e-3));
        const double left_target_from_pwm =
            wheel_speed_from_pwm(static_cast<int>(std::lround(left_pwm)), geometry_.left_pwm_scale);
        const double right_target_from_pwm =
            wheel_speed_from_pwm(static_cast<int>(std::lround(right_pwm)), geometry_.right_pwm_scale);
        left_track_speed += wheel_alpha * (left_target_from_pwm - left_track_speed);
        right_track_speed += wheel_alpha * (right_target_from_pwm - right_track_speed);

        speed = clamp_value(
            0.5 * (left_track_speed + right_track_speed),
            -geometry_.max_linear_speed,
            geometry_.max_linear_speed);
        const double raw_yaw_rate =
            std::abs(geometry_.track) > 1e-6 ? (right_track_speed - left_track_speed) / geometry_.track : 0.0;
        yaw_rate = clamp_value(
            raw_yaw_rate * bounded_yaw_response_scale(geometry_.yaw_response_scale),
            -geometry_.max_yaw_rate,
            geometry_.max_yaw_rate);
        const double curvature = std::abs(speed) > 1e-4 ? yaw_rate / speed : 0.0;
        const double accel = clamp_value(
            (speed - previous_speed) / std::max(dt, 1e-6),
            -geometry_.max_decel,
            geometry_.max_accel);

        const double delta_yaw = yaw_rate * dt;
        const double mid_yaw = wrap_angle(yaw_ + 0.5 * delta_yaw);
        position_.x += speed * std::cos(mid_yaw) * dt;
        position_.y += speed * std::sin(mid_yaw) * dt;
        yaw_ = wrap_angle(yaw_ + delta_yaw);

        const double wheel_circumference = 2.0 * kPi * geometry_.wheel_radius;
        raw_left_ticks_ += (left_track_speed * dt / wheel_circumference) *
                           static_cast<double>(geometry_.encoder_ticks_per_revolution);
        raw_right_ticks_ += (right_track_speed * dt / wheel_circumference) *
                            static_cast<double>(geometry_.encoder_ticks_per_revolution);

        const std::int32_t left_ticks_now = static_cast<std::int32_t>(std::llround(raw_left_ticks_));
        const std::int32_t right_ticks_now = static_cast<std::int32_t>(std::llround(raw_right_ticks_));
        left_tick_delta_ = left_ticks_now - left_ticks_total_;
        right_tick_delta_ = right_ticks_now - right_ticks_total_;
        left_ticks_total_ = left_ticks_now;
        right_ticks_total_ = right_ticks_now;

        update_model_state(
            dt * 1000.0,
            accel,
            curvature,
            target_speed,
            target_yaw_rate);
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
        return "Tracked Vehicle";
    }

  private:
    void update_matrices(double dt) {
        A_.setIdentity();
        B_.setZero();
        A_(0, 0) = 1.0;
        A_(1, 1) = 1.0;
        B_(0, 0) = dt;
        B_(1, 1) = dt;
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

    double wheel_speed_from_pwm(int pwm, double scale) const {
        const double effective_pwm = std::abs(static_cast<double>(pwm));
        if (effective_pwm < static_cast<double>(geometry_.min_effective_pwm) ||
            geometry_.speed_estimate_per_pwm <= 0.0) {
            return 0.0;
        }
        const double scaled_magnitude =
            std::max(0.0, effective_pwm - static_cast<double>(geometry_.min_effective_pwm));
        const double signed_speed =
            scaled_magnitude * geometry_.speed_estimate_per_pwm * std::max(std::abs(scale), 1e-3);
        return std::copysign(signed_speed, static_cast<double>(pwm));
    }

    int wheel_speed_to_pwm(double speed_mps, double scale) const {
        if (std::abs(speed_mps) < 1e-4) {
            return 0;
        }
        const double scaled_speed = std::abs(speed_mps) * std::max(std::abs(scale), 1e-3);
        const double pwm =
            static_cast<double>(geometry_.min_effective_pwm) + geometry_.wheel_speed_to_pwm_bias +
            geometry_.wheel_speed_to_pwm_gain * scaled_speed;
        const int sign = speed_mps >= 0.0 ? 1 : -1;
        return sign * static_cast<int>(std::lround(clamp_value(
                          pwm,
                          static_cast<double>(geometry_.min_effective_pwm),
                          static_cast<double>(geometry_.max_pwm))));
    }

    void update_model_state(double encoder_dt_ms,
                            double accel_override = 0.0,
                            double curvature_override = 0.0,
                            double target_speed_override = 0.0,
                            double target_yaw_rate_override = 0.0) {
        model_state_.position = position_;
        model_state_.yaw = yaw_;
        model_state_.speed = state_vector_(0);
        model_state_.accel = accel_override;
        model_state_.steer_angle = 0.0;
        model_state_.sideslip = 0.0;
        model_state_.yaw_rate = state_vector_(1);
        model_state_.curvature = curvature_override;
        model_state_.left_wheel_speed = state_vector_(4);
        model_state_.right_wheel_speed = state_vector_(5);
        model_state_.target_speed = target_speed_override;
        model_state_.target_steer_angle = 0.0;
        model_state_.target_yaw_rate = target_yaw_rate_override;
        model_state_.left_encoder_ticks = left_ticks_total_;
        model_state_.right_encoder_ticks = right_ticks_total_;
        model_state_.left_encoder_delta = left_tick_delta_;
        model_state_.right_encoder_delta = right_tick_delta_;
        model_state_.left_pwm = static_cast<int>(std::lround(state_vector_(2)));
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
        case VehicleModelKind::TrackedVehicle:
            return "Tracked Vehicle";
        default:
            return "Unknown";
    }
}

std::unique_ptr<VehicleDynamicsModel> make_four_wheel_car_model(const VehicleGeometry& geometry) {
    return std::make_unique<CarLikeBicycleModel>(geometry);
}

std::unique_ptr<VehicleDynamicsModel> make_tracked_vehicle_model(const VehicleGeometry& geometry) {
    return std::make_unique<TrackedVehicleModel>(geometry);
}

}  // namespace thesis_sim
