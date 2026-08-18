#include "mvc/view/live_stream/live_view_stream.h"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <new>
#include <string_view>
#include <type_traits>
#include <utility>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netdb.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

namespace thesis_sim {
namespace {

constexpr std::uint32_t kPacketMagic = 0x54485631U;  // THV1
constexpr std::uint16_t kPacketVersion = 12U;
constexpr std::uint32_t kWorldBlobMagic = 0x5448574DU;  // THWM
constexpr std::uint16_t kWorldBlobVersion = 2U;
constexpr std::uint16_t kPacketHello = 0U;
constexpr std::uint16_t kPacketScene = 1U;
constexpr std::uint16_t kPacketFrame = 2U;
constexpr std::uint16_t kPacketWorld = 3U;
constexpr std::uint16_t kPacketControlAck = 4U;
constexpr std::uint16_t kPacketRobotProfile = 5U;
constexpr std::uint16_t kPacketRuntimeSettings = 6U;
constexpr std::size_t kPacketHeaderSize = sizeof(std::uint32_t) + sizeof(std::uint16_t) + sizeof(std::uint16_t) + sizeof(std::uint32_t);
constexpr std::uint32_t kMaxPacketBytes = 32U * 1024U * 1024U;
constexpr std::uint32_t kMaxVectorElements = 200000U;
constexpr std::uint32_t kMaxStringBytes = 1U * 1024U * 1024U;
constexpr double kTwoPi = 6.28318530717958647692;
constexpr int kHelloTimeoutMs = 5000;

template <typename T>
void write_pod(std::vector<std::uint8_t>* out, const T& value) {
    static_assert(std::is_trivially_copyable_v<T>, "POD serializer requires trivially copyable types");
    if (out == nullptr) {
        return;
    }
    const std::size_t offset = out->size();
    out->resize(offset + sizeof(T));
    std::memcpy(out->data() + offset, &value, sizeof(T));
}

template <typename T>
bool read_pod(const std::vector<std::uint8_t>& data, std::size_t* offset, T* value) {
    static_assert(std::is_trivially_copyable_v<T>, "POD deserializer requires trivially copyable types");
    if (offset == nullptr || value == nullptr || *offset + sizeof(T) > data.size()) {
        return false;
    }
    std::memcpy(value, data.data() + *offset, sizeof(T));
    *offset += sizeof(T);
    return true;
}

void write_bool(std::vector<std::uint8_t>* out, bool value) {
    write_pod<std::uint8_t>(out, value ? 1U : 0U);
}

bool read_bool(const std::vector<std::uint8_t>& data, std::size_t* offset, bool* value) {
    std::uint8_t raw = 0U;
    if (!read_pod(data, offset, &raw) || value == nullptr) {
        return false;
    }
    *value = raw != 0U;
    return true;
}

void write_string(std::vector<std::uint8_t>* out, const std::string& value) {
    write_pod<std::uint32_t>(out, static_cast<std::uint32_t>(value.size()));
    const std::size_t offset = out->size();
    out->resize(offset + value.size());
    if (!value.empty()) {
        std::memcpy(out->data() + offset, value.data(), value.size());
    }
}

bool read_string(const std::vector<std::uint8_t>& data, std::size_t* offset, std::string* value) {
    std::uint32_t size = 0U;
    if (!read_pod(data, offset, &size) ||
        value == nullptr ||
        size > kMaxStringBytes ||
        *offset + size > data.size()) {
        return false;
    }
    value->assign(reinterpret_cast<const char*>(data.data() + *offset), size);
    *offset += size;
    return true;
}

void write_vec2(std::vector<std::uint8_t>* out, const Vec2& value) {
    write_pod(out, value.x);
    write_pod(out, value.y);
}

bool read_vec2(const std::vector<std::uint8_t>& data, std::size_t* offset, Vec2* value) {
    return value != nullptr &&
           read_pod(data, offset, &value->x) &&
           read_pod(data, offset, &value->y);
}

void write_rect(std::vector<std::uint8_t>* out, const Rect& value) {
    write_pod(out, value.min_x);
    write_pod(out, value.min_y);
    write_pod(out, value.max_x);
    write_pod(out, value.max_y);
}

bool read_rect(const std::vector<std::uint8_t>& data, std::size_t* offset, Rect* value) {
    return value != nullptr &&
           read_pod(data, offset, &value->min_x) &&
           read_pod(data, offset, &value->min_y) &&
           read_pod(data, offset, &value->max_x) &&
           read_pod(data, offset, &value->max_y);
}

void write_gate_spec(std::vector<std::uint8_t>* out, const GateSpec& gate) {
    write_string(out, gate.name);
    write_vec2(out, gate.position);
    write_vec2(out, gate.anchor_position);
    write_vec2(out, gate.motion_amplitude);
    write_pod(out, gate.motion_frequency_hz);
    write_pod(out, gate.motion_phase_rad);
    write_pod(out, gate.heading_hint);
    write_bool(out, gate.final);
}

bool read_gate_spec(const std::vector<std::uint8_t>& data, std::size_t* offset, GateSpec* gate) {
    return gate != nullptr &&
           read_string(data, offset, &gate->name) &&
           read_vec2(data, offset, &gate->position) &&
           read_vec2(data, offset, &gate->anchor_position) &&
           read_vec2(data, offset, &gate->motion_amplitude) &&
           read_pod(data, offset, &gate->motion_frequency_hz) &&
           read_pod(data, offset, &gate->motion_phase_rad) &&
           read_pod(data, offset, &gate->heading_hint) &&
           read_bool(data, offset, &gate->final);
}

void write_lidar_hit(std::vector<std::uint8_t>* out, const LidarHit& hit) {
    write_pod(out, hit.angle);
    write_pod(out, hit.distance);
    write_vec2(out, hit.point);
    write_bool(out, hit.hit);
}

bool read_lidar_hit(const std::vector<std::uint8_t>& data, std::size_t* offset, LidarHit* hit) {
    return hit != nullptr &&
           read_pod(data, offset, &hit->angle) &&
           read_pod(data, offset, &hit->distance) &&
           read_vec2(data, offset, &hit->point) &&
           read_bool(data, offset, &hit->hit);
}

std::vector<std::uint8_t> serialize_control_ack(const LiveControlAck& ack) {
    std::vector<std::uint8_t> out;
    out.reserve(96);
    write_bool(&out, ack.ok);
    write_string(&out, ack.message);
    return out;
}

bool deserialize_control_ack(const std::vector<std::uint8_t>& data, LiveControlAck* ack) {
    if (ack == nullptr) {
        return false;
    }
    std::size_t offset = 0;
    LiveControlAck parsed;
    if (!read_bool(data, &offset, &parsed.ok) ||
        !read_string(data, &offset, &parsed.message) ||
        offset != data.size()) {
        return false;
    }
    *ack = std::move(parsed);
    return true;
}

void write_geometry(std::vector<std::uint8_t>* out, const VehicleGeometry& geometry) {
    write_pod(out, geometry.wheelbase);
    write_pod(out, geometry.cg_to_front);
    write_pod(out, geometry.cg_to_rear);
    write_pod(out, geometry.track);
    write_pod(out, geometry.body_length);
    write_pod(out, geometry.body_width);
    write_pod(out, geometry.wheel_length);
    write_pod(out, geometry.wheel_width);
    write_pod(out, geometry.wheel_radius);
    write_pod(out, geometry.max_steer_angle);
    write_pod(out, geometry.max_steer_rate);
    write_pod(out, geometry.max_curvature);
    write_pod(out, geometry.max_linear_speed);
    write_pod(out, geometry.max_yaw_rate);
    write_pod(out, geometry.max_accel);
    write_pod(out, geometry.max_decel);
    write_pod(out, geometry.max_pwm);
    write_pod(out, geometry.min_effective_pwm);
    write_pod(out, geometry.wheel_speed_to_pwm_gain);
    write_pod(out, geometry.wheel_speed_to_pwm_bias);
    write_pod(out, geometry.speed_estimate_per_pwm);
    write_pod(out, geometry.left_pwm_scale);
    write_pod(out, geometry.right_pwm_scale);
    write_pod(out, geometry.linear_feedback_gain);
    write_pod(out, geometry.yaw_feedback_gain);
    write_pod(out, geometry.pwm_slew_rate);
    write_pod(out, geometry.motor_time_constant);
    write_pod(out, geometry.encoder_ticks_per_revolution);
}

bool read_geometry(const std::vector<std::uint8_t>& data, std::size_t* offset, VehicleGeometry* geometry) {
    return geometry != nullptr &&
           read_pod(data, offset, &geometry->wheelbase) &&
           read_pod(data, offset, &geometry->cg_to_front) &&
           read_pod(data, offset, &geometry->cg_to_rear) &&
           read_pod(data, offset, &geometry->track) &&
           read_pod(data, offset, &geometry->body_length) &&
           read_pod(data, offset, &geometry->body_width) &&
           read_pod(data, offset, &geometry->wheel_length) &&
           read_pod(data, offset, &geometry->wheel_width) &&
           read_pod(data, offset, &geometry->wheel_radius) &&
           read_pod(data, offset, &geometry->max_steer_angle) &&
           read_pod(data, offset, &geometry->max_steer_rate) &&
           read_pod(data, offset, &geometry->max_curvature) &&
           read_pod(data, offset, &geometry->max_linear_speed) &&
           read_pod(data, offset, &geometry->max_yaw_rate) &&
           read_pod(data, offset, &geometry->max_accel) &&
           read_pod(data, offset, &geometry->max_decel) &&
           read_pod(data, offset, &geometry->max_pwm) &&
           read_pod(data, offset, &geometry->min_effective_pwm) &&
           read_pod(data, offset, &geometry->wheel_speed_to_pwm_gain) &&
           read_pod(data, offset, &geometry->wheel_speed_to_pwm_bias) &&
           read_pod(data, offset, &geometry->speed_estimate_per_pwm) &&
           read_pod(data, offset, &geometry->left_pwm_scale) &&
           read_pod(data, offset, &geometry->right_pwm_scale) &&
           read_pod(data, offset, &geometry->linear_feedback_gain) &&
           read_pod(data, offset, &geometry->yaw_feedback_gain) &&
           read_pod(data, offset, &geometry->pwm_slew_rate) &&
           read_pod(data, offset, &geometry->motor_time_constant) &&
           read_pod(data, offset, &geometry->encoder_ticks_per_revolution);
}

template <typename T, typename WriteFn>
void write_vector(std::vector<std::uint8_t>* out, const std::vector<T>& values, WriteFn write_fn) {
    write_pod<std::uint32_t>(out, static_cast<std::uint32_t>(values.size()));
    for (const T& value : values) {
        write_fn(out, value);
    }
}

template <typename T, typename ReadFn>
bool read_vector(const std::vector<std::uint8_t>& data, std::size_t* offset, std::vector<T>* values, ReadFn read_fn) {
    if (offset == nullptr || values == nullptr) {
        return false;
    }
    std::uint32_t count = 0U;
    if (!read_pod(data, offset, &count)) {
        return false;
    }
    const std::size_t remaining_bytes = data.size() - *offset;
    if (count > kMaxVectorElements || static_cast<std::size_t>(count) > remaining_bytes) {
        return false;
    }
    values->clear();
    try {
        values->reserve(count);
    } catch (const std::bad_alloc&) {
        return false;
    }
    for (std::uint32_t i = 0; i < count; ++i) {
        T value{};
        if (!read_fn(data, offset, &value)) {
            return false;
        }
        values->push_back(std::move(value));
    }
    return true;
}

void write_world(std::vector<std::uint8_t>* out, const WorldMap& world) {
    write_pod<std::int32_t>(out, static_cast<std::int32_t>(world.environment_mode()));
    write_pod<std::int32_t>(out, static_cast<std::int32_t>(world.unstructured_preset()));
    write_pod<std::int32_t>(out, static_cast<std::int32_t>(world.structured_preset()));
    write_pod<std::int32_t>(out, static_cast<std::int32_t>(world.gate_behavior()));
    write_pod<std::uint32_t>(out, world.gate_seed());
    write_rect(out, world.bounds());
    write_vec2(out, world.start());
    write_vec2(out, world.goal());
    write_pod(out, world.start_heading());
    write_vector(out, world.obstacles(), write_rect);
    write_vector(out, world.perception_obstacles(), write_rect);
    write_vector(out, world.gates(), write_gate_spec);
    write_vector(out, world.road_centerline(), write_vec2);
}

bool read_world(const std::vector<std::uint8_t>& data, std::size_t* offset, WorldMap* world) {
    if (world == nullptr) {
        return false;
    }

    std::int32_t environment_mode = 0;
    std::int32_t unstructured_preset = 0;
    std::int32_t structured_preset = 0;
    std::int32_t gate_behavior = 0;
    std::uint32_t gate_seed = 0;
    Rect bounds{};
    Vec2 start{};
    Vec2 goal{};
    double start_heading = 0.0;
    std::vector<Rect> obstacles;
    std::vector<Rect> perception_obstacles;
    std::vector<GateSpec> gates;
    std::vector<Vec2> road_centerline;

    if (!read_pod(data, offset, &environment_mode) ||
        !read_pod(data, offset, &unstructured_preset) ||
        !read_pod(data, offset, &structured_preset) ||
        !read_pod(data, offset, &gate_behavior) ||
        !read_pod(data, offset, &gate_seed) ||
        !read_rect(data, offset, &bounds) ||
        !read_vec2(data, offset, &start) ||
        !read_vec2(data, offset, &goal) ||
        !read_pod(data, offset, &start_heading) ||
        !read_vector(data, offset, &obstacles, read_rect) ||
        !read_vector(data, offset, &perception_obstacles, read_rect) ||
        !read_vector(data, offset, &gates, read_gate_spec) ||
        !read_vector(data, offset, &road_centerline, read_vec2)) {
        return false;
    }

    const EnvironmentMode env = static_cast<EnvironmentMode>(environment_mode);
    WorldMap restored =
        env == EnvironmentMode::StructuredRoad
            ? WorldMap::structured_demo(static_cast<StructuredMapPreset>(structured_preset))
            : (env == EnvironmentMode::MixedRoadGates
                   ? WorldMap::mixed_hardware_demo()
                   : WorldMap::unstructured_demo(
                         static_cast<UnstructuredMapPreset>(unstructured_preset),
                         static_cast<GateBehaviorMode>(gate_behavior),
                         gate_seed));

    restored.set_bounds(bounds);
    restored.set_start(start);
    restored.set_goal(goal);
    restored.set_start_heading(start_heading);
    restored.editable_obstacles() = std::move(obstacles);
    restored.editable_perception_obstacles() = std::move(perception_obstacles);
    const bool serialized_gates_empty = gates.empty();
    restored.editable_gates() = std::move(gates);
    restored.editable_road_centerline() = std::move(road_centerline);
    // Do not finalize deserialized stream worlds here. The sender already
    // sanitizes custom maps before queueing them; finalizing again would
    // downgrade built-in structured presets (for example Hardware Track or
    // Circle Loop) to Custom and can also re-close/open the centerline.
    if (env == EnvironmentMode::UnstructuredGates &&
        !serialized_gates_empty &&
        static_cast<GateBehaviorMode>(gate_behavior) != GateBehaviorMode::Static) {
        restored.set_gate_behavior(static_cast<GateBehaviorMode>(gate_behavior), gate_seed);
    }
    *world = std::move(restored);
    (void)bounds;
    return true;
}

void write_vehicle_state(std::vector<std::uint8_t>* out, const LiveVehicleState& vehicle) {
    write_vec2(out, vehicle.position);
    write_pod(out, vehicle.yaw);
    write_pod(out, vehicle.speed);
    write_pod(out, vehicle.accel);
    write_pod(out, vehicle.curvature);
    write_pod(out, vehicle.steer_angle);
    write_pod(out, vehicle.yaw_rate);
    write_pod(out, vehicle.sideslip);
    write_pod(out, vehicle.left_wheel_speed);
    write_pod(out, vehicle.right_wheel_speed);
    write_pod(out, vehicle.target_speed);
    write_pod(out, vehicle.target_yaw_rate);
    write_pod(out, vehicle.target_steer_angle);
    write_pod(out, vehicle.left_encoder_ticks);
    write_pod(out, vehicle.right_encoder_ticks);
    write_pod(out, vehicle.left_encoder_delta);
    write_pod(out, vehicle.right_encoder_delta);
    write_pod(out, vehicle.left_pwm);
    write_pod(out, vehicle.right_pwm);
    write_pod(out, vehicle.encoder_dt_ms);
}

bool read_vehicle_state(const std::vector<std::uint8_t>& data, std::size_t* offset, LiveVehicleState* vehicle) {
    return vehicle != nullptr &&
           read_vec2(data, offset, &vehicle->position) &&
           read_pod(data, offset, &vehicle->yaw) &&
           read_pod(data, offset, &vehicle->speed) &&
           read_pod(data, offset, &vehicle->accel) &&
           read_pod(data, offset, &vehicle->curvature) &&
           read_pod(data, offset, &vehicle->steer_angle) &&
           read_pod(data, offset, &vehicle->yaw_rate) &&
           read_pod(data, offset, &vehicle->sideslip) &&
           read_pod(data, offset, &vehicle->left_wheel_speed) &&
           read_pod(data, offset, &vehicle->right_wheel_speed) &&
           read_pod(data, offset, &vehicle->target_speed) &&
           read_pod(data, offset, &vehicle->target_yaw_rate) &&
           read_pod(data, offset, &vehicle->target_steer_angle) &&
           read_pod(data, offset, &vehicle->left_encoder_ticks) &&
           read_pod(data, offset, &vehicle->right_encoder_ticks) &&
           read_pod(data, offset, &vehicle->left_encoder_delta) &&
           read_pod(data, offset, &vehicle->right_encoder_delta) &&
           read_pod(data, offset, &vehicle->left_pwm) &&
           read_pod(data, offset, &vehicle->right_pwm) &&
           read_pod(data, offset, &vehicle->encoder_dt_ms);
}

void write_mpc_command(std::vector<std::uint8_t>* out, const LiveMpcCommandView& command) {
    write_pod(out, command.accel_cmd);
    write_pod(out, command.steer_rate_cmd);
}

bool read_mpc_command(const std::vector<std::uint8_t>& data, std::size_t* offset, LiveMpcCommandView* command) {
    return command != nullptr &&
           read_pod(data, offset, &command->accel_cmd) &&
           read_pod(data, offset, &command->steer_rate_cmd);
}

void write_gate_frame(std::vector<std::uint8_t>* out, const LiveGateFrame& gate) {
    write_gate_spec(out, gate.spec);
    write_bool(out, gate.passed);
}

bool read_gate_frame(const std::vector<std::uint8_t>& data, std::size_t* offset, LiveGateFrame* gate) {
    return gate != nullptr &&
           read_gate_spec(data, offset, &gate->spec) &&
           read_bool(data, offset, &gate->passed);
}

void write_hardware_sample(std::vector<std::uint8_t>* out, const HardwareTelemetrySample& sample) {
    write_pod(out, sample.time);
    write_pod(out, sample.position_x);
    write_pod(out, sample.position_y);
    write_pod(out, sample.yaw);
    write_pod(out, sample.speed);
    write_pod(out, sample.accel);
    write_pod(out, sample.yaw_rate);
    write_pod(out, sample.jerk);
    write_pod(out, sample.command_r);
    write_pod(out, sample.target_speed);
    write_pod(out, sample.target_yaw_rate);
    write_pod(out, sample.curvature);
    write_pod(out, sample.distance_to_goal);
    write_pod(out, sample.structured_track_s);
    write_pod(out, sample.structured_progress_s);
    write_pod(out, sample.min_lidar);
    write_pod(out, sample.front_lidar);
    write_pod(out, sample.planner_speed_ref);
    write_pod(out, sample.tracker_cross_track);
    write_pod(out, sample.tracker_heading_error_deg);
    write_pod(out, sample.planning_ms);
    write_pod(out, sample.tracking_ms);
    write_pod(out, sample.lidar_ms);
    write_pod(out, sample.estimator_ms);
    write_pod(out, sample.step_ms);
    write_pod(out, sample.visible_gates);
    write_pod(out, sample.lidar_samples);
    write_pod(out, sample.close_lidar_samples);
    write_pod(out, sample.front_close_lidar_samples);
    write_pod(out, sample.candidate_gates);
    write_pod(out, sample.chosen_gate_distance);
    write_pod(out, sample.passed_gates);
    write_pod(out, sample.accumulated_lidar_points);
    write_pod(out, sample.no_motion_cycles);
    write_pod(out, sample.chosen_gate_index);
    write_pod(out, sample.safety_stop_active);
    write_pod(out, sample.planner_has_reference);
    write_pod(out, sample.dynamic_gap_gates);
    write_pod(out, sample.pwm_left);
    write_pod(out, sample.pwm_right);
    write_pod(out, sample.controller_pwm_left);
    write_pod(out, sample.controller_pwm_right);
    write_pod(out, sample.controller_target_pwm_left);
    write_pod(out, sample.controller_target_pwm_right);
    write_pod(out, sample.controller_left_encoder_ticks);
    write_pod(out, sample.controller_right_encoder_ticks);
    write_pod(out, sample.controller_left_encoder_delta);
    write_pod(out, sample.controller_right_encoder_delta);
    write_pod(out, sample.controller_encoder_dt_ms);
    write_pod(out, sample.controller_safety_flags);
    write_pod(out, sample.controller_motor_flags);
    write_pod(out, sample.controller_status_flags);
    write_pod(out, sample.controller_error_code);
    write_pod(out, sample.external_pose_valid);
    write_pod(out, sample.external_pose_source_timestamp_s);
    write_pod(out, sample.external_pose_x);
    write_pod(out, sample.external_pose_y);
    write_pod(out, sample.external_pose_yaw);
    write_pod(out, sample.external_pose_quality);
    write_pod(out, sample.external_pose_age_s);
    write_pod(out, sample.external_position_error_m);
    write_pod(out, sample.external_yaw_error_deg);
    write_pod(out, sample.controller_age_s);
    write_pod(out, sample.lidar_age_s);
    write_pod(out, sample.lidar_scan_duration_s);
    write_pod(out, sample.lidar_scan_reused);
    write_pod(out, sample.mcu_clock_offset_s);
    write_pod(out, sample.mcu_clock_jitter_s);
    write_pod(out, sample.imu_age_s);
    write_pod(out, sample.encoder_age_s);
    write_pod(out, sample.slam_pose_valid);
    write_pod(out, sample.slam_pose_x);
    write_pod(out, sample.slam_pose_y);
    write_pod(out, sample.slam_pose_yaw);
    write_pod(out, sample.slam_position_innovation_m);
    write_pod(out, sample.slam_yaw_innovation_deg);
    write_pod(out, sample.slam_correction_accepted);
    write_pod(out, sample.stall_boost_active);
    write_pod(out, sample.left_wheel_stall_cycles);
    write_pod(out, sample.right_wheel_stall_cycles);
    write_pod(out, sample.encoder_slip_guard_active);
    write_pod(out, sample.exploration_map_source);
    write_pod(out, sample.exploration_state);
    write_pod(out, sample.control_source);
    write_pod(out, sample.selected_frontier_valid);
    write_pod(out, sample.selected_frontier_x);
    write_pod(out, sample.selected_frontier_y);
    write_pod(out, sample.global_free_cells);
    write_pod(out, sample.global_occupied_cells);
    write_pod(out, sample.start_matching_enabled);
    write_pod(out, sample.start_matching_complete);
    write_pod(out, sample.start_matching_accepted);
    write_pod(out, sample.start_matching_confidence);
    write_pod(out, sample.slam_toolbox_enabled);
    write_pod(out, sample.slam_toolbox_connected);
    write_pod(out, sample.slam_map_updates);
    write_pod(out, sample.slam_graph_nodes);
    write_pod(out, sample.slam_loop_edges);
    write_pod(out, sample.slam_map_age_s);
}

bool read_hardware_sample(const std::vector<std::uint8_t>& data, std::size_t* offset, HardwareTelemetrySample* sample) {
    return sample != nullptr &&
           read_pod(data, offset, &sample->time) &&
           read_pod(data, offset, &sample->position_x) &&
           read_pod(data, offset, &sample->position_y) &&
           read_pod(data, offset, &sample->yaw) &&
           read_pod(data, offset, &sample->speed) &&
           read_pod(data, offset, &sample->accel) &&
           read_pod(data, offset, &sample->yaw_rate) &&
           read_pod(data, offset, &sample->jerk) &&
           read_pod(data, offset, &sample->command_r) &&
           read_pod(data, offset, &sample->target_speed) &&
           read_pod(data, offset, &sample->target_yaw_rate) &&
           read_pod(data, offset, &sample->curvature) &&
           read_pod(data, offset, &sample->distance_to_goal) &&
           read_pod(data, offset, &sample->structured_track_s) &&
           read_pod(data, offset, &sample->structured_progress_s) &&
           read_pod(data, offset, &sample->min_lidar) &&
           read_pod(data, offset, &sample->front_lidar) &&
           read_pod(data, offset, &sample->planner_speed_ref) &&
           read_pod(data, offset, &sample->tracker_cross_track) &&
           read_pod(data, offset, &sample->tracker_heading_error_deg) &&
           read_pod(data, offset, &sample->planning_ms) &&
           read_pod(data, offset, &sample->tracking_ms) &&
           read_pod(data, offset, &sample->lidar_ms) &&
           read_pod(data, offset, &sample->estimator_ms) &&
           read_pod(data, offset, &sample->step_ms) &&
           read_pod(data, offset, &sample->visible_gates) &&
           read_pod(data, offset, &sample->lidar_samples) &&
           read_pod(data, offset, &sample->close_lidar_samples) &&
           read_pod(data, offset, &sample->front_close_lidar_samples) &&
           read_pod(data, offset, &sample->candidate_gates) &&
           read_pod(data, offset, &sample->chosen_gate_distance) &&
           read_pod(data, offset, &sample->passed_gates) &&
           read_pod(data, offset, &sample->accumulated_lidar_points) &&
           read_pod(data, offset, &sample->no_motion_cycles) &&
           read_pod(data, offset, &sample->chosen_gate_index) &&
           read_pod(data, offset, &sample->safety_stop_active) &&
           read_pod(data, offset, &sample->planner_has_reference) &&
           read_pod(data, offset, &sample->dynamic_gap_gates) &&
           read_pod(data, offset, &sample->pwm_left) &&
           read_pod(data, offset, &sample->pwm_right) &&
           read_pod(data, offset, &sample->controller_pwm_left) &&
           read_pod(data, offset, &sample->controller_pwm_right) &&
           read_pod(data, offset, &sample->controller_target_pwm_left) &&
           read_pod(data, offset, &sample->controller_target_pwm_right) &&
           read_pod(data, offset, &sample->controller_left_encoder_ticks) &&
           read_pod(data, offset, &sample->controller_right_encoder_ticks) &&
           read_pod(data, offset, &sample->controller_left_encoder_delta) &&
           read_pod(data, offset, &sample->controller_right_encoder_delta) &&
           read_pod(data, offset, &sample->controller_encoder_dt_ms) &&
           read_pod(data, offset, &sample->controller_safety_flags) &&
           read_pod(data, offset, &sample->controller_motor_flags) &&
           read_pod(data, offset, &sample->controller_status_flags) &&
           read_pod(data, offset, &sample->controller_error_code) &&
           read_pod(data, offset, &sample->external_pose_valid) &&
           read_pod(data, offset, &sample->external_pose_source_timestamp_s) &&
           read_pod(data, offset, &sample->external_pose_x) &&
           read_pod(data, offset, &sample->external_pose_y) &&
           read_pod(data, offset, &sample->external_pose_yaw) &&
           read_pod(data, offset, &sample->external_pose_quality) &&
           read_pod(data, offset, &sample->external_pose_age_s) &&
           read_pod(data, offset, &sample->external_position_error_m) &&
           read_pod(data, offset, &sample->external_yaw_error_deg) &&
           read_pod(data, offset, &sample->controller_age_s) &&
           read_pod(data, offset, &sample->lidar_age_s) &&
           read_pod(data, offset, &sample->lidar_scan_duration_s) &&
           read_pod(data, offset, &sample->lidar_scan_reused) &&
           read_pod(data, offset, &sample->mcu_clock_offset_s) &&
           read_pod(data, offset, &sample->mcu_clock_jitter_s) &&
           read_pod(data, offset, &sample->imu_age_s) &&
           read_pod(data, offset, &sample->encoder_age_s) &&
           read_pod(data, offset, &sample->slam_pose_valid) &&
           read_pod(data, offset, &sample->slam_pose_x) &&
           read_pod(data, offset, &sample->slam_pose_y) &&
           read_pod(data, offset, &sample->slam_pose_yaw) &&
           read_pod(data, offset, &sample->slam_position_innovation_m) &&
           read_pod(data, offset, &sample->slam_yaw_innovation_deg) &&
           read_pod(data, offset, &sample->slam_correction_accepted) &&
           read_pod(data, offset, &sample->stall_boost_active) &&
           read_pod(data, offset, &sample->left_wheel_stall_cycles) &&
           read_pod(data, offset, &sample->right_wheel_stall_cycles) &&
           read_pod(data, offset, &sample->encoder_slip_guard_active) &&
           read_pod(data, offset, &sample->exploration_map_source) &&
           read_pod(data, offset, &sample->exploration_state) &&
           read_pod(data, offset, &sample->control_source) &&
           read_pod(data, offset, &sample->selected_frontier_valid) &&
           read_pod(data, offset, &sample->selected_frontier_x) &&
           read_pod(data, offset, &sample->selected_frontier_y) &&
           read_pod(data, offset, &sample->global_free_cells) &&
           read_pod(data, offset, &sample->global_occupied_cells) &&
           read_pod(data, offset, &sample->start_matching_enabled) &&
           read_pod(data, offset, &sample->start_matching_complete) &&
           read_pod(data, offset, &sample->start_matching_accepted) &&
           read_pod(data, offset, &sample->start_matching_confidence) &&
           read_pod(data, offset, &sample->slam_toolbox_enabled) &&
           read_pod(data, offset, &sample->slam_toolbox_connected) &&
           read_pod(data, offset, &sample->slam_map_updates) &&
           read_pod(data, offset, &sample->slam_graph_nodes) &&
           read_pod(data, offset, &sample->slam_loop_edges) &&
           read_pod(data, offset, &sample->slam_map_age_s);
}

std::vector<std::uint8_t> serialize_scene(const LiveSceneSnapshot& scene) {
    std::vector<std::uint8_t> out;
    out.reserve(4096);
    write_string(&out, scene.stream_label);
    write_string(&out, scene.stream_profile);
    write_world(&out, scene.world);
    write_geometry(&out, scene.geometry);
    write_bool(&out, scene.imu_enabled);
    write_bool(&out, scene.lidar_enabled);
    write_string(&out, scene.localization_mode);
    write_string(&out, scene.heading_source);
    write_string(&out, scene.range_sensor_name);
    write_string(&out, scene.vehicle_model_name);
    write_string(&out, scene.tracking_controller_name);
    write_string(&out, scene.calibration_profile_name);
    write_string(&out, scene.calibration_profile_version);
    write_string(&out, scene.calibration_profile_hash);
    write_bool(&out, scene.wheel_radius_calibrated);
    write_bool(&out, scene.mcu_velocity_closed_loop);
    write_bool(&out, scene.start_matching_enabled);
    write_bool(&out, scene.slam_toolbox_enabled);
    write_pod(&out, scene.active_lidar_beams);
    write_pod(&out, scene.active_lidar_fov_rad);
    write_pod(&out, scene.active_lidar_range);
    return out;
}

bool deserialize_scene(const std::vector<std::uint8_t>& data, LiveSceneSnapshot* scene) {
    if (scene == nullptr) {
        return false;
    }
    std::size_t offset = 0;
    LiveSceneSnapshot parsed;
    if (!read_string(data, &offset, &parsed.stream_label) ||
        !read_string(data, &offset, &parsed.stream_profile) ||
        !read_world(data, &offset, &parsed.world) ||
        !read_geometry(data, &offset, &parsed.geometry) ||
        !read_bool(data, &offset, &parsed.imu_enabled) ||
        !read_bool(data, &offset, &parsed.lidar_enabled) ||
        !read_string(data, &offset, &parsed.localization_mode) ||
        !read_string(data, &offset, &parsed.heading_source) ||
        !read_string(data, &offset, &parsed.range_sensor_name) ||
        !read_string(data, &offset, &parsed.vehicle_model_name) ||
        !read_string(data, &offset, &parsed.tracking_controller_name) ||
        !read_string(data, &offset, &parsed.calibration_profile_name) ||
        !read_string(data, &offset, &parsed.calibration_profile_version) ||
        !read_string(data, &offset, &parsed.calibration_profile_hash) ||
        !read_bool(data, &offset, &parsed.wheel_radius_calibrated) ||
        !read_bool(data, &offset, &parsed.mcu_velocity_closed_loop) ||
        !read_bool(data, &offset, &parsed.start_matching_enabled) ||
        !read_bool(data, &offset, &parsed.slam_toolbox_enabled) ||
        !read_pod(data, &offset, &parsed.active_lidar_beams) ||
        !read_pod(data, &offset, &parsed.active_lidar_fov_rad) ||
        !read_pod(data, &offset, &parsed.active_lidar_range) ||
        offset != data.size()) {
        return false;
    }
    *scene = std::move(parsed);
    return true;
}

std::vector<std::uint8_t> serialize_frame(const LiveFrameSnapshot& frame) {
    std::vector<std::uint8_t> out;
    out.reserve(16384);
    write_pod(&out, frame.sim_time);
    write_pod(&out, frame.step_count);
    write_bool(&out, frame.connected);
    write_bool(&out, frame.telemetry_ready);
    write_bool(&out, frame.goal_reached);
    write_bool(&out, frame.safety_stop_active);
    write_bool(&out, frame.dynamic_gap_gates);
    write_bool(&out, frame.planner_has_reference);
    write_bool(&out, frame.stall_boost_active);
    write_pod(&out, frame.distance_to_goal);
    write_pod(&out, frame.min_lidar_distance);
    write_pod(&out, frame.front_lidar_distance);
    write_pod(&out, frame.chosen_gate_distance);
    write_pod(&out, frame.last_j);
    write_pod(&out, frame.last_r);
    write_pod(&out, frame.planner_speed_ref);
    write_pod(&out, frame.tracker_cross_track_error);
    write_pod(&out, frame.tracker_heading_error_deg);
    write_pod(&out, frame.valid_lidar_samples);
    write_pod(&out, frame.close_lidar_samples);
    write_pod(&out, frame.front_close_lidar_samples);
    write_pod(&out, frame.candidate_gates);
    write_pod(&out, frame.accumulated_lidar_points);
    write_pod(&out, frame.no_motion_command_cycles);
    write_pod(&out, frame.chosen_gate_index);
    write_pod(&out, frame.passed_gates);
    write_pod<std::int32_t>(&out, static_cast<std::int32_t>(frame.map_source));
    write_pod<std::int32_t>(&out, static_cast<std::int32_t>(frame.exploration_state));
    write_pod<std::int32_t>(&out, static_cast<std::int32_t>(frame.control_source));
    write_bool(&out, frame.selected_frontier_valid);
    write_vec2(&out, frame.selected_frontier);
    write_bool(&out, frame.start_matching_enabled);
    write_bool(&out, frame.start_matching_complete);
    write_bool(&out, frame.start_matching_accepted);
    write_pod(&out, frame.start_matching_confidence);
    write_string(&out, frame.start_matching_status);
    write_bool(&out, frame.slam_enabled);
    write_bool(&out, frame.slam_connected);
    write_bool(&out, frame.slam_pose_valid);
    write_pod(&out, frame.slam_map_updates);
    write_pod(&out, frame.slam_graph_nodes);
    write_pod(&out, frame.slam_loop_edges);
    write_pod(&out, frame.slam_map_resolution_m);
    write_pod(&out, frame.slam_map_age_s);
    write_string(&out, frame.slam_session_id);
    write_string(&out, frame.slam_reset_reason);
    write_string(&out, frame.slam_status);
    write_string(&out, frame.reference_invalidation_reason);
    write_vehicle_state(&out, frame.vehicle);
    write_vec2(&out, frame.navigation_position);
    write_pod(&out, frame.navigation_yaw);
    write_pod(&out, frame.navigation_yaw_rate);
    write_pod(&out, frame.navigation_curvature);
    write_pod(&out, frame.navigation_speed);
    write_pod(&out, frame.navigation_accel);
    write_vector(&out, frame.gates, write_gate_frame);
    write_vector(&out, frame.visible_gate_indices, [](std::vector<std::uint8_t>* buffer, const int& value) {
        write_pod(buffer, value);
    });
    write_vector(&out, frame.trail, write_vec2);
    write_vector(&out, frame.planned_trajectory, write_vec2);
    write_vector(&out, frame.local_safety_occupied_points, write_vec2);
    write_vector(&out, frame.global_free_points, write_vec2);
    write_vector(&out, frame.global_occupied_points, write_vec2);
    write_vector(&out, frame.lidar_hits, write_lidar_hit);
    write_bool(&out, frame.has_last_mpc_command);
    if (frame.has_last_mpc_command) {
        write_mpc_command(&out, frame.last_mpc_command);
    }
    write_bool(&out, frame.has_latest_sample);
    if (frame.has_latest_sample) {
        write_hardware_sample(&out, frame.latest_sample);
    }
    return out;
}

bool deserialize_frame(const std::vector<std::uint8_t>& data, LiveFrameSnapshot* frame) {
    if (frame == nullptr) {
        return false;
    }
    std::size_t offset = 0;
    LiveFrameSnapshot parsed;
    std::int32_t map_source = 0;
    std::int32_t exploration_state = 0;
    std::int32_t control_source = 0;
    if (!read_pod(data, &offset, &parsed.sim_time) ||
        !read_pod(data, &offset, &parsed.step_count) ||
        !read_bool(data, &offset, &parsed.connected) ||
        !read_bool(data, &offset, &parsed.telemetry_ready) ||
        !read_bool(data, &offset, &parsed.goal_reached) ||
        !read_bool(data, &offset, &parsed.safety_stop_active) ||
        !read_bool(data, &offset, &parsed.dynamic_gap_gates) ||
        !read_bool(data, &offset, &parsed.planner_has_reference) ||
        !read_bool(data, &offset, &parsed.stall_boost_active) ||
        !read_pod(data, &offset, &parsed.distance_to_goal) ||
        !read_pod(data, &offset, &parsed.min_lidar_distance) ||
        !read_pod(data, &offset, &parsed.front_lidar_distance) ||
        !read_pod(data, &offset, &parsed.chosen_gate_distance) ||
        !read_pod(data, &offset, &parsed.last_j) ||
        !read_pod(data, &offset, &parsed.last_r) ||
        !read_pod(data, &offset, &parsed.planner_speed_ref) ||
        !read_pod(data, &offset, &parsed.tracker_cross_track_error) ||
        !read_pod(data, &offset, &parsed.tracker_heading_error_deg) ||
        !read_pod(data, &offset, &parsed.valid_lidar_samples) ||
        !read_pod(data, &offset, &parsed.close_lidar_samples) ||
        !read_pod(data, &offset, &parsed.front_close_lidar_samples) ||
        !read_pod(data, &offset, &parsed.candidate_gates) ||
        !read_pod(data, &offset, &parsed.accumulated_lidar_points) ||
        !read_pod(data, &offset, &parsed.no_motion_command_cycles) ||
        !read_pod(data, &offset, &parsed.chosen_gate_index) ||
        !read_pod(data, &offset, &parsed.passed_gates) ||
        !read_pod(data, &offset, &map_source) ||
        !read_pod(data, &offset, &exploration_state) ||
        !read_pod(data, &offset, &control_source) ||
        !read_bool(data, &offset, &parsed.selected_frontier_valid) ||
        !read_vec2(data, &offset, &parsed.selected_frontier) ||
        !read_bool(data, &offset, &parsed.start_matching_enabled) ||
        !read_bool(data, &offset, &parsed.start_matching_complete) ||
        !read_bool(data, &offset, &parsed.start_matching_accepted) ||
        !read_pod(data, &offset, &parsed.start_matching_confidence) ||
        !read_string(data, &offset, &parsed.start_matching_status) ||
        !read_bool(data, &offset, &parsed.slam_enabled) ||
        !read_bool(data, &offset, &parsed.slam_connected) ||
        !read_bool(data, &offset, &parsed.slam_pose_valid) ||
        !read_pod(data, &offset, &parsed.slam_map_updates) ||
        !read_pod(data, &offset, &parsed.slam_graph_nodes) ||
        !read_pod(data, &offset, &parsed.slam_loop_edges) ||
        !read_pod(data, &offset, &parsed.slam_map_resolution_m) ||
        !read_pod(data, &offset, &parsed.slam_map_age_s) ||
        !read_string(data, &offset, &parsed.slam_session_id) ||
        !read_string(data, &offset, &parsed.slam_reset_reason) ||
        !read_string(data, &offset, &parsed.slam_status) ||
        !read_string(data, &offset, &parsed.reference_invalidation_reason) ||
        !read_vehicle_state(data, &offset, &parsed.vehicle) ||
        !read_vec2(data, &offset, &parsed.navigation_position) ||
        !read_pod(data, &offset, &parsed.navigation_yaw) ||
        !read_pod(data, &offset, &parsed.navigation_yaw_rate) ||
        !read_pod(data, &offset, &parsed.navigation_curvature) ||
        !read_pod(data, &offset, &parsed.navigation_speed) ||
        !read_pod(data, &offset, &parsed.navigation_accel) ||
        !read_vector(data, &offset, &parsed.gates, read_gate_frame) ||
        !read_vector(data, &offset, &parsed.visible_gate_indices, [](const std::vector<std::uint8_t>& bytes, std::size_t* pos, int* value) {
            return read_pod(bytes, pos, value);
        }) ||
        !read_vector(data, &offset, &parsed.trail, read_vec2) ||
        !read_vector(data, &offset, &parsed.planned_trajectory, read_vec2) ||
        !read_vector(data, &offset, &parsed.local_safety_occupied_points, read_vec2) ||
        !read_vector(data, &offset, &parsed.global_free_points, read_vec2) ||
        !read_vector(data, &offset, &parsed.global_occupied_points, read_vec2) ||
        !read_vector(data, &offset, &parsed.lidar_hits, read_lidar_hit) ||
        !read_bool(data, &offset, &parsed.has_last_mpc_command)) {
        return false;
    }

    if (parsed.has_last_mpc_command && !read_mpc_command(data, &offset, &parsed.last_mpc_command)) {
        return false;
    }
    if (!read_bool(data, &offset, &parsed.has_latest_sample)) {
        return false;
    }
    if (parsed.has_latest_sample && !read_hardware_sample(data, &offset, &parsed.latest_sample)) {
        return false;
    }
    if (offset != data.size()) {
        return false;
    }

    parsed.map_source = static_cast<ExplorationMapSource>(map_source);
    parsed.exploration_state = static_cast<UnstructuredExplorationState>(exploration_state);
    parsed.control_source = static_cast<HardwareControlSource>(control_source);
    *frame = std::move(parsed);
    return true;
}
