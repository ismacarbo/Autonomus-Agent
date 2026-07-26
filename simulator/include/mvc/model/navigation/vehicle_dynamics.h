#pragma once

#include <cstdint>
#include <limits>
#include <memory>
#include <string>

#include <Eigen/Dense>

#include "mvc/model/world/world.h"

namespace thesis_sim {

namespace measured_robot {

inline constexpr double kCarBodyLengthM = 0.250;
inline constexpr double kCarBodyWidthM = 0.150;
inline constexpr double kCarWheelRadiusM = 0.0327;
inline constexpr double kCarWheelDiameterM = 0.0654;
inline constexpr std::int32_t kCarEncoderTicksPerRevolution = 38;

inline constexpr double kTankBodyLengthM = 0.165;
inline constexpr double kTankBodyWidthM = 0.146;
inline constexpr double kTankBeltLengthM = 0.158;
inline constexpr double kTankBeltWidthM = 0.0447;
// Distance between belt centre lines: total width minus one belt width.
inline constexpr double kTankTrackCenterDistanceM =
    kTankBodyWidthM - kTankBeltWidthM;
// Effective sprocket radius still needs a dedicated physical measurement.
// Keep the previously validated encoder conversion until that value is known.
inline constexpr double kTankEffectiveDriveRadiusDefaultM = 0.032;

}  // namespace measured_robot

enum class VehicleModelKind {
    CarLikeBicycle = 0,
    TrackedVehicle = 1,
};

const char* vehicle_model_kind_name(VehicleModelKind kind);

struct VehicleGeometry {
    // Physical footprint shared by the two thesis robots. Scenario-specific
    // miniature presets may override it explicitly, but hardware-aligned maps
    // must start from the measured 25 x 15 cm body.
    double wheelbase = 0.18;
    double cg_to_front = 0.09;
    double cg_to_rear = 0.09;
    double track = 0.13;
    double body_length = measured_robot::kCarBodyLengthM;
    double body_width = measured_robot::kCarBodyWidthM;
    double wheel_length = 0.09;
    double wheel_width = 0.03;
    double wheel_radius = measured_robot::kCarWheelRadiusM;
    double max_steer_angle = 0.52;
    double max_steer_rate = 1.8;
    double max_curvature = 2.1;
    double max_linear_speed = 0.90;
    double max_yaw_rate = 4.00;
    double max_accel = 1.4;
    double max_decel = 1.8;
    int max_pwm = 255;
    int min_effective_pwm = 55;
    double wheel_speed_to_pwm_gain = 190.0;
    double wheel_speed_to_pwm_bias = 28.0;
    double speed_estimate_per_pwm = 0.0016;
    double left_pwm_scale = 1.00;
    double right_pwm_scale = 1.00;
    double yaw_response_scale = 1.00;
    double linear_feedback_gain = 75.0;
    double yaw_feedback_gain = 55.0;
    double pwm_slew_rate = 450.0;
    double motor_time_constant = 0.24;
    std::int32_t encoder_ticks_per_revolution = 360;
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
    double left_wheel_speed = 0.0;
    double right_wheel_speed = 0.0;
    double target_speed = 0.0;
    double target_yaw_rate = 0.0;
    double target_steer_angle = 0.0;
    std::int32_t left_encoder_ticks = 0;
    std::int32_t right_encoder_ticks = 0;
    std::int32_t left_encoder_delta = 0;
    std::int32_t right_encoder_delta = 0;
    int left_pwm = 0;
    int right_pwm = 0;
    double encoder_dt_ms = 0.0;
    Eigen::VectorXd internal_state;
};

struct VehicleControlInput {
    double jerk_cmd = 0.0;
    double planner_r_cmd = 0.0;
    double accel_cmd = 0.0;
    double steer_rate_cmd = 0.0;
    double target_speed = 0.0;
    double target_yaw_rate = std::numeric_limits<double>::quiet_NaN();
    double target_curvature = 0.0;
    double target_steer_angle = 0.0;
};

class VehicleDynamicsModel {
  public:
    virtual ~VehicleDynamicsModel() = default;

    virtual void reset(const Vec2& position, double yaw) = 0;
    virtual void step(double dt, const VehicleControlInput& input) = 0;

    virtual const VehicleGeometry& geometry() const = 0;
    virtual const VehicleModelState& state() const = 0;
    virtual const Eigen::MatrixXd& A() const = 0;
    virtual const Eigen::MatrixXd& B() const = 0;
    virtual std::string name() const = 0;
};

std::unique_ptr<VehicleDynamicsModel> make_four_wheel_car_model(const VehicleGeometry& geometry = {});
std::unique_ptr<VehicleDynamicsModel> make_tracked_vehicle_model(const VehicleGeometry& geometry = {});

}  // namespace thesis_sim
