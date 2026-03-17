#include "state_estimator_ekf.h"

#include <algorithm>
#include <cmath>

namespace thesis_sim {

namespace {

using Vector5 = Eigen::Matrix<double, 5, 1>;
using Matrix5 = Eigen::Matrix<double, 5, 5>;

double clamp_value(double value, double lo, double hi) {
    return std::max(lo, std::min(value, hi));
}

double infer_sideslip(const VehicleGeometry& geometry, double speed, double yaw_rate) {
    if (std::abs(speed) <= 1e-4 || geometry.cg_to_rear <= 1e-6) {
        return 0.0;
    }

    const double curvature = clamp_value(
        yaw_rate / speed,
        -geometry.max_curvature,
        geometry.max_curvature);
    const double beta_arg = clamp_value(geometry.cg_to_rear * curvature, -1.0, 1.0);
    return std::asin(beta_arg);
}

}  // namespace

KinematicBicycleEkf::KinematicBicycleEkf(EkfConfig config)
    : config_(config) {}

double KinematicBicycleEkf::wrap_angle(double angle) {
    constexpr double kPi = 3.14159265358979323846;
    while (angle > kPi) {
        angle -= 2.0 * kPi;
    }
    while (angle < -kPi) {
        angle += 2.0 * kPi;
    }
    return angle;
}

void KinematicBicycleEkf::set_geometry(const VehicleGeometry& geometry) {
    geometry_ = geometry;
}

void KinematicBicycleEkf::reset(const Vec2& position, double yaw) {
    state_ = {};
    state_.position = position;
    state_.yaw = yaw;
    state_.covariance.setIdentity();
    state_.covariance *= 0.05;
}

void KinematicBicycleEkf::finalize_predict(double odom_speed, double odom_yaw_rate, double dt) {
    const double prev_speed = state_.speed;
    state_.speed = odom_speed;
    state_.yaw_rate = odom_yaw_rate;
    state_.accel = dt > 1e-6 ? (state_.speed - prev_speed) / dt : 0.0;
    state_.yaw = wrap_angle(state_.yaw);
}

void KinematicBicycleEkf::predict(double dt, double odom_speed, double odom_yaw_rate) {
    if (dt <= 1e-6) {
        finalize_predict(odom_speed, odom_yaw_rate, dt);
        return;
    }

    Vector5 x;
    x << state_.position.x, state_.position.y, state_.yaw, state_.speed, state_.yaw_rate;

    const double beta = infer_sideslip(geometry_, odom_speed, odom_yaw_rate);
    const double yaw_mid = x(2) + 0.5 * odom_yaw_rate * dt;
    const double heading_mid = yaw_mid + beta;
    Vector5 x_pred = x;
    x_pred(0) += odom_speed * std::cos(heading_mid) * dt;
    x_pred(1) += odom_speed * std::sin(heading_mid) * dt;
    x_pred(2) = wrap_angle(x(2) + odom_yaw_rate * dt);
    x_pred(3) = odom_speed;
    x_pred(4) = odom_yaw_rate;

    Matrix5 F = Matrix5::Zero();
    F(0, 0) = 1.0;
    F(1, 1) = 1.0;
    F(2, 2) = 1.0;
    F(0, 2) = -odom_speed * std::sin(heading_mid) * dt;
    F(1, 2) = odom_speed * std::cos(heading_mid) * dt;

    Matrix5 Q = Matrix5::Zero();
    Q(0, 0) = config_.q_x * dt;
    Q(1, 1) = config_.q_y * dt;
    Q(2, 2) = config_.q_yaw * dt;
    Q(3, 3) = config_.q_v * dt;
    Q(4, 4) = config_.q_yaw_rate * dt;

    state_.covariance = F * state_.covariance * F.transpose() + Q;
    state_.position = {x_pred(0), x_pred(1)};
    state_.yaw = x_pred(2);
    finalize_predict(odom_speed, odom_yaw_rate, dt);
}

void KinematicBicycleEkf::update_imu(double measured_yaw, double measured_yaw_rate) {
    Eigen::Matrix<double, 2, 5> H = Eigen::Matrix<double, 2, 5>::Zero();
    H(0, 2) = 1.0;
    H(1, 4) = 1.0;

    Eigen::Matrix<double, 2, 2> R = Eigen::Matrix<double, 2, 2>::Zero();
    R(0, 0) = config_.r_imu_yaw;
    R(1, 1) = config_.r_imu_yaw_rate;

    Eigen::Matrix<double, 5, 1> x;
    x << state_.position.x, state_.position.y, state_.yaw, state_.speed, state_.yaw_rate;

    Eigen::Matrix<double, 2, 1> z;
    z << measured_yaw, measured_yaw_rate;

    Eigen::Matrix<double, 2, 1> h;
    h << state_.yaw, state_.yaw_rate;

    Eigen::Matrix<double, 2, 1> y = z - h;
    y(0) = wrap_angle(y(0));

    const Eigen::Matrix<double, 2, 2> S = H * state_.covariance * H.transpose() + R;
    const Eigen::Matrix<double, 5, 2> K = state_.covariance * H.transpose() * S.inverse();
    x += K * y;
    state_.covariance = (Matrix5::Identity() - K * H) * state_.covariance;

    state_.position = {x(0), x(1)};
    state_.yaw = wrap_angle(x(2));
    state_.speed = x(3);
    state_.yaw_rate = x(4);
}

void KinematicBicycleEkf::update_lidar_pose(const Vec2& measured_position,
                                            double measured_yaw,
                                            bool include_yaw) {
    if (include_yaw) {
        Eigen::Matrix<double, 3, 5> H = Eigen::Matrix<double, 3, 5>::Zero();
        H(0, 0) = 1.0;
        H(1, 1) = 1.0;
        H(2, 2) = 1.0;

        Eigen::Matrix<double, 3, 3> R = Eigen::Matrix<double, 3, 3>::Zero();
        R(0, 0) = config_.r_lidar_x;
        R(1, 1) = config_.r_lidar_y;
        R(2, 2) = config_.r_lidar_yaw;

        Eigen::Matrix<double, 5, 1> x;
        x << state_.position.x, state_.position.y, state_.yaw, state_.speed, state_.yaw_rate;

        Eigen::Matrix<double, 3, 1> z;
        z << measured_position.x, measured_position.y, measured_yaw;

        Eigen::Matrix<double, 3, 1> h;
        h << state_.position.x, state_.position.y, state_.yaw;

        Eigen::Matrix<double, 3, 1> y = z - h;
        y(2) = wrap_angle(y(2));

        const Eigen::Matrix<double, 3, 3> S = H * state_.covariance * H.transpose() + R;
        const Eigen::Matrix<double, 5, 3> K = state_.covariance * H.transpose() * S.inverse();
        x += K * y;
        state_.covariance = (Matrix5::Identity() - K * H) * state_.covariance;

        state_.position = {x(0), x(1)};
        state_.yaw = wrap_angle(x(2));
        state_.speed = x(3);
        state_.yaw_rate = x(4);
        return;
    }

    Eigen::Matrix<double, 2, 5> H = Eigen::Matrix<double, 2, 5>::Zero();
    H(0, 0) = 1.0;
    H(1, 1) = 1.0;

    Eigen::Matrix<double, 2, 2> R = Eigen::Matrix<double, 2, 2>::Zero();
    R(0, 0) = config_.r_lidar_x;
    R(1, 1) = config_.r_lidar_y;

    Eigen::Matrix<double, 5, 1> x;
    x << state_.position.x, state_.position.y, state_.yaw, state_.speed, state_.yaw_rate;

    Eigen::Matrix<double, 2, 1> z;
    z << measured_position.x, measured_position.y;

    Eigen::Matrix<double, 2, 1> h;
    h << state_.position.x, state_.position.y;

    const Eigen::Matrix<double, 2, 1> y = z - h;
    const Eigen::Matrix<double, 2, 2> S = H * state_.covariance * H.transpose() + R;
    const Eigen::Matrix<double, 5, 2> K = state_.covariance * H.transpose() * S.inverse();
    x += K * y;
    state_.covariance = (Matrix5::Identity() - K * H) * state_.covariance;

    state_.position = {x(0), x(1)};
    state_.yaw = wrap_angle(x(2));
    state_.speed = x(3);
    state_.yaw_rate = x(4);
}

}  // namespace thesis_sim
