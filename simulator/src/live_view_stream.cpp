#include "live_view_stream.h"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstring>
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
constexpr std::uint16_t kPacketVersion = 5U;
constexpr std::uint16_t kPacketHello = 0U;
constexpr std::uint16_t kPacketScene = 1U;
constexpr std::uint16_t kPacketFrame = 2U;
constexpr std::size_t kPacketHeaderSize = sizeof(std::uint32_t) + sizeof(std::uint16_t) + sizeof(std::uint16_t) + sizeof(std::uint32_t);
constexpr std::uint32_t kMaxPacketBytes = 32U * 1024U * 1024U;
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
    if (!read_pod(data, offset, &size) || value == nullptr || *offset + size > data.size()) {
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
    values->clear();
    values->reserve(count);
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
        !read_vector(data, offset, &gates, read_gate_spec) ||
        !read_vector(data, offset, &road_centerline, read_vec2)) {
        return false;
    }

    const EnvironmentMode env = static_cast<EnvironmentMode>(environment_mode);
    WorldMap restored = env == EnvironmentMode::StructuredRoad
                            ? WorldMap::structured_demo(static_cast<StructuredMapPreset>(structured_preset))
                            : WorldMap::unstructured_demo(
                                  static_cast<UnstructuredMapPreset>(unstructured_preset),
                                  static_cast<GateBehaviorMode>(gate_behavior),
                                  gate_seed);

    restored.set_start(start);
    restored.set_goal(goal);
    restored.set_start_heading(start_heading);
    restored.editable_obstacles() = std::move(obstacles);
    restored.editable_gates() = std::move(gates);
    restored.editable_road_centerline() = std::move(road_centerline);
    restored.finalize_editor_changes();
    if (env == EnvironmentMode::UnstructuredGates) {
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
    write_pod(out, sample.controller_safety_flags);
    write_pod(out, sample.controller_motor_flags);
    write_pod(out, sample.controller_status_flags);
    write_pod(out, sample.controller_error_code);
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
           read_pod(data, offset, &sample->controller_safety_flags) &&
           read_pod(data, offset, &sample->controller_motor_flags) &&
           read_pod(data, offset, &sample->controller_status_flags) &&
           read_pod(data, offset, &sample->controller_error_code);
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
    write_vector(&out, frame.slam_points, write_vec2);
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
        !read_vector(data, &offset, &parsed.slam_points, read_vec2) ||
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

    *frame = std::move(parsed);
    return true;
}

bool set_nonblocking(int fd) {
    if (fd < 0) {
        return false;
    }
    const int flags = fcntl(fd, F_GETFL, 0);
    if (flags < 0) {
        return false;
    }
    return fcntl(fd, F_SETFL, flags | O_NONBLOCK) == 0;
}

void close_socket(int* fd) {
    if (fd != nullptr && *fd >= 0) {
        ::close(*fd);
        *fd = -1;
    }
}

std::string socket_error_message(const std::string& prefix) {
    return prefix + ": " + std::strerror(errno);
}

std::string endpoint_to_string(const sockaddr_storage& addr) {
    char host[NI_MAXHOST] = {};
    char service[NI_MAXSERV] = {};
    if (getnameinfo(reinterpret_cast<const sockaddr*>(&addr),
                    sizeof(addr),
                    host,
                    sizeof(host),
                    service,
                    sizeof(service),
                    NI_NUMERICHOST | NI_NUMERICSERV) == 0) {
        return std::string(host) + ":" + service;
    }
    return "unknown";
}

bool send_all(int fd, const std::uint8_t* data, std::size_t size) {
    std::size_t sent = 0;
    while (sent < size) {
        const ssize_t bytes = ::send(fd, data + sent, size - sent, 0);
        if (bytes <= 0) {
            if (errno == EINTR) {
                continue;
            }
            return false;
        }
        sent += static_cast<std::size_t>(bytes);
    }
    return true;
}

std::vector<std::uint8_t> make_packet(std::uint16_t type, const std::vector<std::uint8_t>& payload) {
    std::vector<std::uint8_t> packet;
    packet.reserve(kPacketHeaderSize + payload.size());
    write_pod(&packet, kPacketMagic);
    write_pod(&packet, kPacketVersion);
    write_pod(&packet, type);
    write_pod<std::uint32_t>(&packet, static_cast<std::uint32_t>(payload.size()));
    packet.insert(packet.end(), payload.begin(), payload.end());
    return packet;
}

bool recv_all_with_timeout(int fd, std::uint8_t* data, std::size_t size, int timeout_ms) {
    if (fd < 0 || data == nullptr) {
        return false;
    }

    std::size_t received = 0;
    while (received < size) {
        pollfd pfd{};
        pfd.fd = fd;
        pfd.events = POLLIN;
        const int rc = ::poll(&pfd, 1, timeout_ms);
        if (rc == 0) {
            errno = ETIMEDOUT;
            return false;
        }
        if (rc < 0) {
            if (errno == EINTR) {
                continue;
            }
            return false;
        }
        if ((pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
            errno = ECONNRESET;
            return false;
        }

        const ssize_t bytes = ::recv(fd, data + received, size - received, 0);
        if (bytes <= 0) {
            if (bytes < 0 && errno == EINTR) {
                continue;
            }
            if (bytes == 0) {
                errno = ECONNRESET;
            }
            return false;
        }
        received += static_cast<std::size_t>(bytes);
    }
    return true;
}

bool receive_server_hello(int fd, std::string* error) {
    std::array<std::uint8_t, kPacketHeaderSize> header{};
    if (!recv_all_with_timeout(fd, header.data(), header.size(), kHelloTimeoutMs)) {
        if (error != nullptr) {
            if (errno == ETIMEDOUT) {
                *error = "live viewer handshake failed: timeout waiting for viewer hello";
            } else {
                *error = socket_error_message("live viewer handshake failed");
            }
        }
        return false;
    }

    std::vector<std::uint8_t> header_vec(header.begin(), header.end());
    std::size_t offset = 0;
    std::uint32_t magic = 0U;
    std::uint16_t version = 0U;
    std::uint16_t type = 0U;
    std::uint32_t size = 0U;
    if (!read_pod(header_vec, &offset, &magic) ||
        !read_pod(header_vec, &offset, &version) ||
        !read_pod(header_vec, &offset, &type) ||
        !read_pod(header_vec, &offset, &size)) {
        if (error != nullptr) {
            *error = "live viewer handshake failed: malformed header";
        }
        return false;
    }
    if (magic != kPacketMagic || version != kPacketVersion || type != kPacketHello) {
        if (error != nullptr) {
            *error = "live viewer handshake failed: protocol mismatch or wrong service on port";
        }
        return false;
    }
    if (size > kMaxPacketBytes) {
        if (error != nullptr) {
            *error = "live viewer handshake failed: packet too large";
        }
        return false;
    }
    if (size == 0U) {
        return true;
    }

    std::vector<std::uint8_t> payload(size);
    if (!recv_all_with_timeout(fd, payload.data(), payload.size(), kHelloTimeoutMs)) {
        if (error != nullptr) {
            *error = socket_error_message("live viewer handshake payload failed");
        }
        return false;
    }
    return true;
}

bool points_form_closed_loop(const std::vector<Vec2>& points, double threshold = 0.45) {
    return points.size() >= 3 && distance(points.front(), points.back()) <= threshold;
}

double wrap_angle(double angle) {
    const double kPi = 0.5 * kTwoPi;
    while (angle > kPi) {
        angle -= kTwoPi;
    }
    while (angle < -kPi) {
        angle += kTwoPi;
    }
    return angle;
}

struct PolylineLayout {
    std::vector<Vec2> points;
    std::vector<double> cumulative_s;
    double total_length = 0.0;
    bool closed_loop = false;
};

struct PolylineProjection {
    bool valid = false;
    Vec2 point;
    double s = 0.0;
    double heading = 0.0;
    double lateral = 0.0;
};

struct StructuredDisplayRemap {
    bool active = false;
    WorldMap display_world;
    PolylineLayout source_path;
    PolylineLayout display_path;
};

double clamp_value(double value, double lo, double hi) {
    return std::max(lo, std::min(value, hi));
}

PolylineLayout make_polyline_layout(const std::vector<Vec2>& points) {
    PolylineLayout layout;
    layout.points = points;
    layout.closed_loop = points_form_closed_loop(points);
    layout.cumulative_s.reserve(points.size());
    layout.cumulative_s.push_back(0.0);
    for (std::size_t i = 1; i < points.size(); ++i) {
        layout.total_length += distance(points[i - 1], points[i]);
        layout.cumulative_s.push_back(layout.total_length);
    }
    return layout;
}

PolylineProjection project_point_onto_polyline(const PolylineLayout& layout, const Vec2& point) {
    PolylineProjection best;
    double best_distance_sq = 0.0;
    bool have_best = false;

    if (layout.points.size() < 2) {
        return best;
    }

    for (std::size_t i = 0; i + 1 < layout.points.size(); ++i) {
        const Vec2& a = layout.points[i];
        const Vec2& b = layout.points[i + 1];
        const double dx = b.x - a.x;
        const double dy = b.y - a.y;
        const double segment_length_sq = dx * dx + dy * dy;
        if (segment_length_sq <= 1e-12) {
            continue;
        }

        const double t = clamp_value(((point.x - a.x) * dx + (point.y - a.y) * dy) / segment_length_sq, 0.0, 1.0);
        const Vec2 projected{a.x + dx * t, a.y + dy * t};
        const double error_x = point.x - projected.x;
        const double error_y = point.y - projected.y;
        const double distance_sq = error_x * error_x + error_y * error_y;
        if (!have_best || distance_sq < best_distance_sq) {
            const double heading = std::atan2(dy, dx);
            best.valid = true;
            best.point = projected;
            best.s = layout.cumulative_s[i] + std::sqrt(segment_length_sq) * t;
            best.heading = heading;
            best.lateral = -std::sin(heading) * error_x + std::cos(heading) * error_y;
            best_distance_sq = distance_sq;
            have_best = true;
        }
    }

    return best;
}

PolylineProjection sample_polyline_at_s(const PolylineLayout& layout, double s) {
    PolylineProjection sample;
    if (layout.points.empty()) {
        return sample;
    }
    if (layout.points.size() == 1 || layout.total_length <= 1e-9) {
        sample.valid = true;
        sample.point = layout.points.front();
        sample.heading = 0.0;
        sample.s = 0.0;
        return sample;
    }

    double wrapped_s = s;
    if (layout.closed_loop && layout.total_length > 1e-9) {
        wrapped_s = std::fmod(wrapped_s, layout.total_length);
        if (wrapped_s < 0.0) {
            wrapped_s += layout.total_length;
        }
    } else {
        wrapped_s = clamp_value(wrapped_s, 0.0, layout.total_length);
    }

    for (std::size_t i = 0; i + 1 < layout.points.size(); ++i) {
        const double start_s = layout.cumulative_s[i];
        const double end_s = layout.cumulative_s[i + 1];
        const double segment_length = end_s - start_s;
        if (wrapped_s > end_s && i + 2 < layout.points.size()) {
            continue;
        }

        const Vec2& a = layout.points[i];
        const Vec2& b = layout.points[i + 1];
        const double heading = std::atan2(b.y - a.y, b.x - a.x);
        const double alpha = segment_length > 1e-9 ? clamp_value((wrapped_s - start_s) / segment_length, 0.0, 1.0) : 0.0;
        sample.valid = true;
        sample.point = {a.x + (b.x - a.x) * alpha, a.y + (b.y - a.y) * alpha};
        sample.heading = heading;
        sample.s = wrapped_s;
        return sample;
    }

    sample.valid = true;
    sample.point = layout.points.back();
    sample.heading = std::atan2(
        layout.points.back().y - layout.points[layout.points.size() - 2].y,
        layout.points.back().x - layout.points[layout.points.size() - 2].x);
    sample.s = wrapped_s;
    return sample;
}

bool needs_structured_display_normalization(const WorldMap& world) {
    if (world.environment_mode() != EnvironmentMode::StructuredRoad) {
        return false;
    }
    if (points_form_closed_loop(world.road_centerline())) {
        return false;
    }
    const Rect& bounds = world.bounds();
    const double world_span = std::max(bounds.max_x - bounds.min_x, bounds.max_y - bounds.min_y);
    return world.structured_preset() == StructuredMapPreset::HardwareTrack || world_span <= 5.0;
}

StructuredDisplayRemap make_structured_display_remap(const WorldMap& source_world) {
    StructuredDisplayRemap remap;
    remap.display_world = source_world;
    if (!needs_structured_display_normalization(source_world)) {
        return remap;
    }

    remap.source_path = make_polyline_layout(source_world.road_centerline());
    remap.display_world = WorldMap::structured_demo(StructuredMapPreset::ValidationRoad);
    remap.display_path = make_polyline_layout(remap.display_world.road_centerline());
    remap.active = remap.source_path.total_length > 1e-6 && remap.display_path.total_length > 1e-6;
    if (!remap.active) {
        remap.display_world = source_world;
    }
    return remap;
}

Vec2 remap_point_to_display(const StructuredDisplayRemap& remap, const Vec2& point) {
    if (!remap.active) {
        return point;
    }
    const PolylineProjection source_projection = project_point_onto_polyline(remap.source_path, point);
    if (!source_projection.valid || remap.source_path.total_length <= 1e-9) {
        return point;
    }

    const double progress = source_projection.s / remap.source_path.total_length;
    const PolylineProjection display_projection =
        sample_polyline_at_s(remap.display_path, progress * remap.display_path.total_length);
    if (!display_projection.valid) {
        return point;
    }

    return {
        display_projection.point.x - std::sin(display_projection.heading) * source_projection.lateral,
        display_projection.point.y + std::cos(display_projection.heading) * source_projection.lateral,
    };
}

double remap_yaw_to_display(const StructuredDisplayRemap& remap, const Vec2& source_point, double yaw) {
    if (!remap.active) {
        return yaw;
    }
    const PolylineProjection source_projection = project_point_onto_polyline(remap.source_path, source_point);
    if (!source_projection.valid || remap.source_path.total_length <= 1e-9) {
        return yaw;
    }

    const double progress = source_projection.s / remap.source_path.total_length;
    const PolylineProjection display_projection =
        sample_polyline_at_s(remap.display_path, progress * remap.display_path.total_length);
    if (!display_projection.valid) {
        return yaw;
    }
    return wrap_angle(yaw - source_projection.heading + display_projection.heading);
}

std::vector<Vec2> remap_points_to_display(const StructuredDisplayRemap& remap,
                                          const std::vector<Vec2>& points) {
    if (!remap.active || points.empty()) {
        return points;
    }

    std::vector<Vec2> out;
    out.reserve(points.size());
    for (const Vec2& point : points) {
        out.push_back(remap_point_to_display(remap, point));
    }
    return out;
}

std::vector<LidarHit> remap_hits_to_display(const StructuredDisplayRemap& remap,
                                            const std::vector<LidarHit>& hits,
                                            const Vec2& source_origin) {
    if (!remap.active || hits.empty()) {
        return hits;
    }

    const Vec2 display_origin = remap_point_to_display(remap, source_origin);
    std::vector<LidarHit> out;
    out.reserve(hits.size());
    for (const LidarHit& hit : hits) {
        LidarHit mapped = hit;
        mapped.point = remap_point_to_display(remap, hit.point);
        mapped.distance = distance(display_origin, mapped.point);
        mapped.angle = std::atan2(mapped.point.y - display_origin.y, mapped.point.x - display_origin.x);
        out.push_back(std::move(mapped));
    }
    return out;
}

void apply_structured_display_remap(const StructuredDisplayRemap& remap, LiveFrameSnapshot* frame) {
    if (!remap.active || frame == nullptr) {
        return;
    }

    const Vec2 source_vehicle_position = frame->vehicle.position;
    const Vec2 source_navigation_position = frame->navigation_position;
    frame->vehicle.position = remap_point_to_display(remap, source_vehicle_position);
    frame->vehicle.yaw = remap_yaw_to_display(remap, source_vehicle_position, frame->vehicle.yaw);
    frame->navigation_position = remap_point_to_display(remap, source_navigation_position);
    frame->navigation_yaw = remap_yaw_to_display(remap, source_navigation_position, frame->navigation_yaw);
    frame->trail = remap_points_to_display(remap, frame->trail);
    frame->planned_trajectory = remap_points_to_display(remap, frame->planned_trajectory);
    frame->slam_points = remap_points_to_display(remap, frame->slam_points);
    frame->lidar_hits = remap_hits_to_display(remap, frame->lidar_hits, source_navigation_position);
}

}  // namespace

LiveSceneSnapshot make_live_scene_snapshot(const HardwarePlannerRunner& runner) {
    const StructuredDisplayRemap remap = make_structured_display_remap(runner.world());
    const bool lidar_enabled = runner.lidar_enabled_for_current_mode();
    LiveSceneSnapshot scene;
    scene.stream_label = "Hardware live stream";
    scene.stream_profile = "planner";
    scene.world = remap.display_world;
    scene.geometry = runner.geometry();
    scene.imu_enabled = true;
    scene.lidar_enabled = lidar_enabled;
    scene.localization_mode = lidar_enabled ? "Encoders + IMU + LiDAR" : "Encoders + IMU";
    scene.heading_source = "IMU";
    scene.range_sensor_name = lidar_enabled ? "RPLidar A1" : "LiDAR disabled for structured planner";
    scene.vehicle_model_name = "Car-like bicycle";
    scene.tracking_controller_name = "MPC path follower";
    scene.active_lidar_beams = lidar_enabled ? 360 : 0;
    scene.active_lidar_fov_rad = lidar_enabled ? kTwoPi : 0.0;
    scene.active_lidar_range = lidar_enabled ? runner.config().localization.max_range_m : 0.0;
    return scene;
}

LiveFrameSnapshot make_live_frame_snapshot(const HardwarePlannerRunner& runner) {
    const StructuredDisplayRemap remap = make_structured_display_remap(runner.world());
    LiveFrameSnapshot frame;
    frame.sim_time = runner.sim_time();
    frame.step_count = runner.step_count();
    frame.connected = runner.connected();
    frame.telemetry_ready = runner.telemetry_ready();
    frame.goal_reached = runner.goal_reached();
    frame.safety_stop_active = runner.safety_stop_active();
    frame.dynamic_gap_gates = runner.diagnostics().dynamic_gap_gates;
    frame.planner_has_reference = runner.diagnostics().planner_has_reference;
    frame.stall_boost_active = runner.diagnostics().stall_boost_active;
    frame.distance_to_goal = runner.distance_to_goal();
    frame.min_lidar_distance = runner.estimate().min_lidar_distance;
    frame.front_lidar_distance = runner.estimate().front_lidar_distance;
    frame.chosen_gate_distance =
        std::isfinite(runner.diagnostics().chosen_gate_distance) ? runner.diagnostics().chosen_gate_distance : -1.0;
    frame.last_j = runner.last_j();
    frame.last_r = runner.last_r();
    frame.planner_speed_ref = runner.planner_speed_reference();
    frame.tracker_cross_track_error = runner.tracker_cross_track_error();
    frame.tracker_heading_error_deg = runner.tracker_heading_error_deg();
    frame.valid_lidar_samples = runner.diagnostics().valid_lidar_points;
    frame.close_lidar_samples = runner.diagnostics().close_lidar_points;
    frame.front_close_lidar_samples = runner.diagnostics().front_close_lidar_points;
    frame.candidate_gates = runner.diagnostics().candidate_gates;
    frame.accumulated_lidar_points = runner.diagnostics().accumulated_lidar_points;
    frame.no_motion_command_cycles = runner.diagnostics().no_motion_command_cycles;
    frame.chosen_gate_index = runner.chosen_gate_index();
    frame.navigation_position = runner.estimate().position;
    frame.navigation_yaw = runner.estimate().yaw;
    frame.navigation_yaw_rate = runner.estimate().yaw_rate;
    frame.navigation_curvature = runner.estimate().curvature;
    frame.navigation_speed = runner.estimate().speed;
    frame.navigation_accel = runner.estimate().accel;
    frame.visible_gate_indices = runner.visible_gate_indices();
    frame.trail = runner.trail();
    frame.planned_trajectory = runner.planned_trajectory();
    frame.slam_points = runner.lidar_map_points();
    frame.lidar_hits = runner.lidar_hits();

    const HardwarePlannerEstimate& estimate = runner.estimate();
    const HardwareControlCommand& command = runner.last_command();
    const VehicleGeometry& geometry = runner.geometry();
    const double half_track = geometry.track * 0.5;
    frame.vehicle.position = estimate.position;
    frame.vehicle.yaw = estimate.yaw;
    frame.vehicle.speed = estimate.speed;
    frame.vehicle.accel = estimate.accel;
    frame.vehicle.curvature = estimate.curvature;
    frame.vehicle.steer_angle = std::clamp(
        std::atan(geometry.wheelbase * estimate.curvature),
        -geometry.max_steer_angle,
        geometry.max_steer_angle);
    frame.vehicle.yaw_rate = estimate.yaw_rate;
    frame.vehicle.sideslip = 0.0;
    frame.vehicle.left_wheel_speed = estimate.speed - estimate.yaw_rate * half_track;
    frame.vehicle.right_wheel_speed = estimate.speed + estimate.yaw_rate * half_track;
    frame.vehicle.target_speed = command.target_speed;
    frame.vehicle.target_yaw_rate = command.target_yaw_rate;
    frame.vehicle.target_steer_angle = std::clamp(
        std::atan(geometry.wheelbase * command.target_curvature),
        -geometry.max_steer_angle,
        geometry.max_steer_angle);
    frame.vehicle.left_pwm = command.pwm_left;
    frame.vehicle.right_pwm = command.pwm_right;

    const auto& runtime_gate_specs = runner.gate_specs();
    const auto& planner_gates = runner.gates();
    frame.gates.reserve(runtime_gate_specs.size());
    for (std::size_t i = 0; i < runtime_gate_specs.size(); ++i) {
        LiveGateFrame gate_frame;
        gate_frame.spec = runtime_gate_specs[i];
        if (i < planner_gates.size()) {
            gate_frame.passed = planner_gates[i].passed;
        }
        frame.gates.push_back(std::move(gate_frame));
    }

    if (runner.last_mpc_command().has_value()) {
        frame.has_last_mpc_command = true;
        frame.last_mpc_command.accel_cmd = runner.last_mpc_command()->accel_cmd;
        frame.last_mpc_command.steer_rate_cmd = runner.last_mpc_command()->steer_rate_cmd;
    }

    if (!runner.history().empty()) {
        frame.has_latest_sample = true;
        frame.latest_sample = runner.history().back();
    }

    apply_structured_display_remap(remap, &frame);
    return frame;
}

LiveViewStreamClient::~LiveViewStreamClient() {
    disconnect();
}

bool LiveViewStreamClient::connect_to(const std::string& host, std::uint16_t port) {
    disconnect();
    last_error_.clear();

    addrinfo hints{};
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;

    addrinfo* results = nullptr;
    const std::string port_str = std::to_string(port);
    const int rc = getaddrinfo(host.c_str(), port_str.c_str(), &hints, &results);
    if (rc != 0) {
        last_error_ = "getaddrinfo failed: " + std::string(gai_strerror(rc));
        return false;
    }

    bool connected = false;
    for (addrinfo* it = results; it != nullptr && !connected; it = it->ai_next) {
        socket_fd_ = ::socket(it->ai_family, it->ai_socktype, it->ai_protocol);
        if (socket_fd_ < 0) {
            continue;
        }
        if (::connect(socket_fd_, it->ai_addr, it->ai_addrlen) == 0) {
            connected = true;
            break;
        }
        close_socket(&socket_fd_);
    }
    freeaddrinfo(results);

    if (!connected) {
        last_error_ = socket_error_message("connect failed");
        return false;
    }
    if (!receive_server_hello(socket_fd_, &last_error_)) {
        disconnect();
        return false;
    }
    return true;
}

void LiveViewStreamClient::disconnect() {
    close_socket(&socket_fd_);
}

bool LiveViewStreamClient::send_packet(std::uint16_t type, const std::vector<std::uint8_t>& payload) {
    if (socket_fd_ < 0) {
        last_error_ = "stream client is not connected";
        return false;
    }

    const std::vector<std::uint8_t> packet = make_packet(type, payload);

    if (!send_all(socket_fd_, packet.data(), packet.size())) {
        last_error_ = socket_error_message("send failed");
        disconnect();
        return false;
    }
    return true;
}

bool LiveViewStreamClient::send_scene(const LiveSceneSnapshot& scene) {
    return send_packet(kPacketScene, serialize_scene(scene));
}

bool LiveViewStreamClient::send_frame(const LiveFrameSnapshot& frame) {
    return send_packet(kPacketFrame, serialize_frame(frame));
}

LiveViewStreamServer::~LiveViewStreamServer() {
    stop();
}

bool LiveViewStreamServer::start(std::uint16_t port) {
    stop();
    last_error_.clear();

    listen_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd_ < 0) {
        last_error_ = socket_error_message("socket failed");
        return false;
    }

    int reuse_addr = 1;
    if (setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse_addr, sizeof(reuse_addr)) != 0) {
        last_error_ = socket_error_message("setsockopt failed");
        stop();
        return false;
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(port);

    if (bind(listen_fd_, reinterpret_cast<const sockaddr*>(&addr), sizeof(addr)) != 0) {
        last_error_ = socket_error_message("bind failed");
        stop();
        return false;
    }
    if (listen(listen_fd_, 1) != 0) {
        last_error_ = socket_error_message("listen failed");
        stop();
        return false;
    }
    if (!set_nonblocking(listen_fd_)) {
        last_error_ = socket_error_message("failed to set listener non-blocking");
        stop();
        return false;
    }

    port_ = port;
    return true;
}

void LiveViewStreamServer::close_client() {
    close_socket(&client_fd_);
    remote_endpoint_.clear();
    recv_buffer_.clear();
}

void LiveViewStreamServer::stop() {
    close_client();
    close_socket(&listen_fd_);
    port_ = 0;
}

bool LiveViewStreamServer::accept_client() {
    if (listen_fd_ < 0) {
        return false;
    }

    sockaddr_storage addr{};
    socklen_t addr_len = sizeof(addr);
    const int fd = ::accept(listen_fd_, reinterpret_cast<sockaddr*>(&addr), &addr_len);
    if (fd < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
            last_error_ = socket_error_message("accept failed");
        }
        return false;
    }

    close_client();
    client_fd_ = fd;
    const std::vector<std::uint8_t> hello_packet = make_packet(kPacketHello, {});
    if (!send_all(client_fd_, hello_packet.data(), hello_packet.size())) {
        last_error_ = socket_error_message("failed to send live viewer handshake");
        close_client();
        return false;
    }
    if (!set_nonblocking(client_fd_)) {
        last_error_ = socket_error_message("failed to set client non-blocking");
        close_client();
        return false;
    }
    remote_endpoint_ = endpoint_to_string(addr);
    last_error_.clear();
    return true;
}

void LiveViewStreamServer::read_client_data() {
    if (client_fd_ < 0) {
        return;
    }

    std::array<std::uint8_t, 8192> chunk{};
    while (true) {
        const ssize_t bytes = ::recv(client_fd_, chunk.data(), chunk.size(), 0);
        if (bytes > 0) {
            recv_buffer_.insert(recv_buffer_.end(), chunk.begin(), chunk.begin() + bytes);
            continue;
        }
        if (bytes == 0) {
            close_client();
            return;
        }
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return;
        }
        if (errno == EINTR) {
            continue;
        }
        last_error_ = socket_error_message("recv failed");
        close_client();
        return;
    }
}

bool LiveViewStreamServer::parse_next_packet(PollResult* result) {
    if (recv_buffer_.size() < kPacketHeaderSize) {
        return false;
    }

    std::size_t offset = 0;
    std::uint32_t magic = 0U;
    std::uint16_t version = 0U;
    std::uint16_t type = 0U;
    std::uint32_t size = 0U;
    if (!read_pod(recv_buffer_, &offset, &magic) ||
        !read_pod(recv_buffer_, &offset, &version) ||
        !read_pod(recv_buffer_, &offset, &type) ||
        !read_pod(recv_buffer_, &offset, &size)) {
        return false;
    }

    if (magic != kPacketMagic || version != kPacketVersion) {
        last_error_ = "live stream protocol mismatch";
        close_client();
        return false;
    }
    if (size > kMaxPacketBytes) {
        last_error_ = "live stream packet too large";
        close_client();
        return false;
    }
    if (recv_buffer_.size() < kPacketHeaderSize + size) {
        return false;
    }

    std::vector<std::uint8_t> payload(size);
    if (size > 0) {
        std::memcpy(payload.data(), recv_buffer_.data() + kPacketHeaderSize, size);
    }
    recv_buffer_.erase(recv_buffer_.begin(), recv_buffer_.begin() + static_cast<std::ptrdiff_t>(kPacketHeaderSize + size));

    if (result == nullptr) {
        return true;
    }

    if (type == kPacketScene) {
        LiveSceneSnapshot scene;
        if (!deserialize_scene(payload, &scene)) {
            last_error_ = "failed to decode live scene snapshot";
            close_client();
            return false;
        }
        result->scene_received = true;
        result->scene = std::move(scene);
        return true;
    }
    if (type == kPacketFrame) {
        LiveFrameSnapshot frame;
        if (!deserialize_frame(payload, &frame)) {
            last_error_ = "failed to decode live frame snapshot";
            close_client();
            return false;
        }
        result->frame_received = true;
        result->frame = std::move(frame);
        return true;
    }

    last_error_ = "unknown live stream packet type";
    close_client();
    return false;
}

LiveViewStreamServer::PollResult LiveViewStreamServer::poll() {
    PollResult result;
    if (listen_fd_ < 0) {
        return result;
    }

    if (client_fd_ < 0) {
        accept_client();
    }
    read_client_data();
    while (parse_next_packet(&result)) {
    }
    return result;
}

}  // namespace thesis_sim
