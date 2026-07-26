#include "mvc/model/navigation/mpc_path_follower.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

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

struct TrackingError {
    double cross_track = 0.0;
    double heading = 0.0;
};

struct PredictState {
    Vec2 position;
    double yaw = 0.0;
    double speed = 0.0;
    double steer_angle = 0.0;
    double yaw_rate = 0.0;
};

TrackingError compute_tracking_error(const PredictState& state, const ReferenceWaypoint& ref) {
    const double dx = state.position.x - ref.position.x;
    const double dy = state.position.y - ref.position.y;
    const double c = std::cos(ref.yaw);
    const double s = std::sin(ref.yaw);
    return {
        -s * dx + c * dy,
        wrap_angle(state.yaw - ref.yaw),
    };
}

double pursuit_curvature(const PredictState& state,
                         const ReferenceWaypoint& target,
                         double min_lookahead_distance) {
    const double dx = target.position.x - state.position.x;
    const double dy = target.position.y - state.position.y;
    const double lookahead = std::max(std::hypot(dx, dy), std::max(min_lookahead_distance, 1e-3));
    const double alpha = wrap_angle(std::atan2(dy, dx) - state.yaw);
    return 2.0 * std::sin(alpha) / lookahead;
}

int find_anchor_index(const Vec2& position,
                      const std::vector<ReferenceWaypoint>& reference,
                      int hint_index,
                      int search_back,
                      int search_forward) {
    if (reference.empty()) {
        return 0;
    }

    const int last_index = static_cast<int>(reference.size()) - 1;
    const auto search_window = [&](int first, int last) {
        int best_index = first;
        double best_score = std::numeric_limits<double>::infinity();
        for (int i = first; i <= last; ++i) {
            const double dx = position.x - reference[static_cast<size_t>(i)].position.x;
            const double dy = position.y - reference[static_cast<size_t>(i)].position.y;
            const double distance_sq = dx * dx + dy * dy;
            const double progress_penalty = 0.06 * static_cast<double>(i - first) * static_cast<double>(i - first);
            const double score = distance_sq + progress_penalty;
            if (score < best_score) {
                best_score = score;
                best_index = i;
            }
        }
        return std::pair<int, double>{best_index, best_score};
    };

    const int clamped_hint = std::clamp(hint_index, 0, last_index);
    const int first_index = std::clamp(clamped_hint - std::max(search_back, 0), 0, last_index);
    const int last_window_index = std::clamp(clamped_hint + std::max(search_forward, 0), 0, last_index);
    const auto [local_index, local_score] = search_window(first_index, last_window_index);

    if (local_score <= 9.0 || (first_index == 0 && last_window_index == last_index)) {
        return local_index;
    }

    return search_window(0, last_index).first;
}

int find_preview_index(int anchor_index,
                       const std::vector<ReferenceWaypoint>& reference,
                       double preview_distance,
                       int preview_min_index) {
    if (reference.empty()) {
        return 0;
    }

    const int last_index = static_cast<int>(reference.size()) - 1;
    int preview_index = std::clamp(anchor_index + std::max(preview_min_index, 0), 0, last_index);
    if (preview_distance <= 1e-6 || preview_index >= last_index) {
        return preview_index;
    }

    double accumulated_distance = 0.0;
    for (int i = std::clamp(anchor_index, 0, last_index); i < last_index; ++i) {
        accumulated_distance += distance(reference[static_cast<size_t>(i)].position,
                                         reference[static_cast<size_t>(i + 1)].position);
        preview_index = i + 1;
        if (preview_index >= anchor_index + std::max(preview_min_index, 0) &&
            accumulated_distance >= preview_distance) {
            break;
        }
    }

    return std::clamp(preview_index, 0, last_index);
}

double steering_from_curvature(const VehicleGeometry& geometry, double curvature) {
    const double bounded_curvature = clamp_value(curvature, -geometry.max_curvature, geometry.max_curvature);
    const double steer_angle = std::atan(geometry.wheelbase * bounded_curvature);
    return clamp_value(steer_angle, -geometry.max_steer_angle, geometry.max_steer_angle);
}

void integrate_bicycle(const VehicleGeometry& geometry,
                       double dt,
                       double accel_cmd,
                       double steer_rate_cmd,
                       PredictState* state) {
    if (state == nullptr || dt <= 0.0) {
        return;
    }

    state->speed = clamp_value(state->speed + accel_cmd * dt, 0.0, geometry.max_linear_speed);
    state->steer_angle = clamp_value(
        state->steer_angle + steer_rate_cmd * dt,
        -geometry.max_steer_angle,
        geometry.max_steer_angle);

    const double beta = std::atan2(
        geometry.cg_to_rear * std::tan(state->steer_angle),
        std::max(geometry.wheelbase, 1e-6));
    const double yaw_rate = state->speed * std::cos(beta) * std::tan(state->steer_angle) /
                            std::max(geometry.wheelbase, 1e-6);

    state->position.x += state->speed * std::cos(state->yaw + beta) * dt;
    state->position.y += state->speed * std::sin(state->yaw + beta) * dt;
    state->yaw = wrap_angle(state->yaw + yaw_rate * dt);
    state->yaw_rate = yaw_rate;
}

void integrate_differential(const VehicleGeometry& geometry,
                            double dt,
                            double accel_cmd,
                            double yaw_accel_cmd,
                            PredictState* state) {
    if (state == nullptr || dt <= 0.0) {
        return;
    }

    state->speed = clamp_value(
        state->speed + accel_cmd * dt,
        0.0,
        geometry.max_linear_speed);
    state->yaw_rate = clamp_value(
        state->yaw_rate + yaw_accel_cmd * dt,
        -geometry.max_yaw_rate,
        geometry.max_yaw_rate);
    state->position.x += state->speed * std::cos(state->yaw) * dt;
    state->position.y += state->speed * std::sin(state->yaw) * dt;
    state->yaw = wrap_angle(state->yaw + state->yaw_rate * dt);
}

}  // namespace

const char* tracking_controller_mode_name(TrackingControllerMode mode) {
    switch (mode) {
        case TrackingControllerMode::PlannerCommand:
            return "Planner Direct";
        case TrackingControllerMode::MpcPathFollower:
            return "MPC Follower";
        default:
            return "Unknown";
    }
}

ModelPredictivePathFollower::ModelPredictivePathFollower(MpcFollowerConfig config)
    : config_(config) {}

MpcCommand ModelPredictivePathFollower::solve(const VehicleGeometry& geometry,
                                             const VehicleModelState& vehicle_state,
                                             const std::vector<ReferenceWaypoint>& reference,
                                             double desired_speed,
                                             int anchor_hint_index,
                                             VehicleModelKind vehicle_model) const {
    MpcCommand best{};
    if (reference.size() < 2) {
        return best;
    }

    const int anchor_index = find_anchor_index(
        vehicle_state.position,
        reference,
        anchor_hint_index,
        0,
        std::max(config_.preview_min_index + 4, 6));
    const ReferenceWaypoint& anchor_ref = reference[static_cast<size_t>(anchor_index)];
    const int preview_index = find_preview_index(
        anchor_index,
        reference,
        config_.preview_distance,
        config_.preview_min_index);
    const ReferenceWaypoint& preview_ref = reference[static_cast<size_t>(preview_index)];
    const PredictState current_state{
        vehicle_state.position,
        vehicle_state.yaw,
        vehicle_state.speed,
        vehicle_state.steer_angle,
        vehicle_state.yaw_rate,
    };
    const TrackingError anchor_error = compute_tracking_error(current_state, anchor_ref);
    const double preview_curvature = pursuit_curvature(
        current_state,
        preview_ref,
        config_.min_lookahead_distance);
    const double corrective_curvature =
        0.35 * preview_curvature +
        0.65 * preview_ref.curvature +
        0.08 * anchor_error.cross_track -
        0.20 * anchor_error.heading;
    const double desired_steer = steering_from_curvature(geometry, corrective_curvature);
    const double base_steer_rate = clamp_value(
        (desired_steer - vehicle_state.steer_angle) / std::max(config_.horizon_dt, 1e-3),
        -config_.max_steer_rate,
        config_.max_steer_rate);

    const double speed_error = desired_speed - vehicle_state.speed;
    const double base_accel = clamp_value(speed_error / std::max(config_.horizon_dt, 1e-3),
                                          config_.min_accel,
                                          config_.max_accel);

    if (vehicle_model == VehicleModelKind::TrackedVehicle) {
        const double desired_yaw_rate = clamp_value(
            desired_speed * corrective_curvature -
                1.10 * anchor_error.heading -
                0.35 * anchor_error.cross_track,
            -geometry.max_yaw_rate,
            geometry.max_yaw_rate);
        const double base_yaw_accel = clamp_value(
            (desired_yaw_rate - vehicle_state.yaw_rate) /
                std::max(config_.horizon_dt, 1e-3),
            -config_.max_yaw_accel,
            config_.max_yaw_accel);
        const std::array<double, 5> accel_offsets{-0.9, -0.45, 0.0, 0.45, 0.9};
        const std::array<double, 5> yaw_accel_offsets{-1.0, -0.5, 0.0, 0.5, 1.0};

        best.cost = std::numeric_limits<double>::infinity();
        for (double accel_offset : accel_offsets) {
            const double accel_cmd = clamp_value(
                base_accel + accel_offset,
                config_.min_accel,
                config_.max_accel);
            for (double yaw_offset : yaw_accel_offsets) {
                const double yaw_accel_cmd = clamp_value(
                    base_yaw_accel + yaw_offset * config_.max_yaw_accel,
                    -config_.max_yaw_accel,
                    config_.max_yaw_accel);
                PredictState predicted = current_state;
                double total_cost = 0.0;
                for (int step = 0; step < std::max(config_.horizon_steps, 1); ++step) {
                    const int ref_index = std::min(
                        anchor_index + step,
                        static_cast<int>(reference.size()) - 1);
                    const ReferenceWaypoint& ref = reference[static_cast<size_t>(ref_index)];
                    const TrackingError error = compute_tracking_error(predicted, ref);
                    const double speed_tracking_error = predicted.speed - ref.speed;
                    const double reference_yaw_rate = ref.speed * ref.curvature;
                    const double yaw_rate_error = predicted.yaw_rate - reference_yaw_rate;
                    const double stage_weight =
                        (step + 1 == config_.horizon_steps) ? config_.w_terminal : 1.0;
                    total_cost += stage_weight * (
                        config_.w_cross_track * error.cross_track * error.cross_track +
                        config_.w_heading * error.heading * error.heading +
                        config_.w_speed * speed_tracking_error * speed_tracking_error +
                        config_.w_yaw_rate * yaw_rate_error * yaw_rate_error);
                    total_cost += config_.w_accel * accel_cmd * accel_cmd +
                                  config_.w_yaw_accel * yaw_accel_cmd * yaw_accel_cmd;
                    integrate_differential(
                        geometry,
                        config_.horizon_dt,
                        accel_cmd,
                        yaw_accel_cmd,
                        &predicted);
                }

                if (total_cost < best.cost) {
                    best.accel_cmd = accel_cmd;
                    best.steer_rate_cmd = 0.0;
                    best.yaw_accel_cmd = yaw_accel_cmd;
                    best.target_speed = desired_speed;
                    best.target_steer_angle = 0.0;
                    best.target_yaw_rate = desired_yaw_rate;
                    best.cost = total_cost;
                    best.cross_track_error = anchor_error.cross_track;
                    best.heading_error = anchor_error.heading;
                    best.anchor_index = anchor_index;
                    best.valid = true;
                }
            }
        }
        return best;
    }

    const std::array<double, 5> accel_offsets{-0.9, -0.45, 0.0, 0.45, 0.9};
    const std::array<double, 5> steer_offsets{-1.0, -0.5, 0.0, 0.5, 1.0};

    best.cost = std::numeric_limits<double>::infinity();
    for (double accel_offset : accel_offsets) {
        const double accel_cmd = clamp_value(
            base_accel + accel_offset,
            config_.min_accel,
            config_.max_accel);

        for (double steer_offset : steer_offsets) {
            const double steer_rate_cmd = clamp_value(
                base_steer_rate + steer_offset,
                -config_.max_steer_rate,
                config_.max_steer_rate);

            PredictState predicted{
                vehicle_state.position,
                vehicle_state.yaw,
                vehicle_state.speed,
                vehicle_state.steer_angle,
            };

            double total_cost = 0.0;
            for (int step = 0; step < std::max(config_.horizon_steps, 1); ++step) {
                const int ref_index = std::min(
                    anchor_index + step,
                    static_cast<int>(reference.size()) - 1);
                const ReferenceWaypoint& ref = reference[static_cast<size_t>(ref_index)];
                const TrackingError error = compute_tracking_error(predicted, ref);
                const double speed_tracking_error = predicted.speed - ref.speed;
                const double steer_tracking_error =
                    predicted.steer_angle - steering_from_curvature(geometry, ref.curvature);
                const double stage_weight = (step + 1 == config_.horizon_steps) ? config_.w_terminal : 1.0;

                total_cost += stage_weight * (
                    config_.w_cross_track * error.cross_track * error.cross_track +
                    config_.w_heading * error.heading * error.heading +
                    config_.w_speed * speed_tracking_error * speed_tracking_error +
                    config_.w_steer * steer_tracking_error * steer_tracking_error);
                total_cost += config_.w_accel * accel_cmd * accel_cmd +
                              config_.w_steer_rate * steer_rate_cmd * steer_rate_cmd;

                integrate_bicycle(geometry, config_.horizon_dt, accel_cmd, steer_rate_cmd, &predicted);
            }

            if (total_cost < best.cost) {
                best.accel_cmd = accel_cmd;
                best.steer_rate_cmd = steer_rate_cmd;
                best.target_speed = desired_speed;
                best.target_steer_angle = desired_steer;
                best.target_yaw_rate = desired_speed * corrective_curvature;
                best.cost = total_cost;
                best.cross_track_error = anchor_error.cross_track;
                best.heading_error = anchor_error.heading;
                best.anchor_index = anchor_index;
                best.valid = true;
            }
        }
    }

    return best;
}

}  // namespace thesis_sim
