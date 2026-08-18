#!/usr/bin/env python3
"""UDP adapter between the C++ viewer and the real ROS 2 slam_toolbox node."""

from __future__ import annotations

import math
import socket
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from slam_toolbox.srv import Reset
from tf2_ros import Buffer, TransformBroadcaster, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from sequence_policy import classify_packet


MIN_SLAM_RETURNS = 8


@dataclass
class ScanPacket:
    session: str
    sequence: int
    timestamp: float
    x: float
    y: float
    yaw: float
    min_range: float
    max_range: float
    lidar_offset_x: float
    lidar_offset_y: float
    beams: list[tuple[float, float, bool]]
    source: tuple[str, int]


def yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    return 0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw)


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class UdpSlamBridge(Node):
    def __init__(self) -> None:
        super().__init__("thesis_slam_toolbox_udp_bridge")
        self.scan_pub = self.create_publisher(LaserScan, "/scan", 10)
        self.tf = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(OccupancyGrid, "/map", self.on_map, 2)
        self.create_subscription(
            MarkerArray,
            "/slam_toolbox/graph_visualization",
            self.on_graph_visualization,
            2,
        )
        self.reset_client = self.create_client(Reset, "/slam_toolbox/reset")
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        self.sock.bind(("0.0.0.0", 9760))
        self.latest_map: OccupancyGrid | None = None
        self.latest_corrected_pose: tuple[float, float, float] | None = None
        self.latest_packet: ScanPacket | None = None
        self.map_updates = 0
        self.graph_nodes = 0
        self.loop_edges = 0
        self.active_session = ""
        self.last_sequence = -1
        self.reset_future = None
        self.pending_reset_packet: ScanPacket | None = None
        self.last_reset_reason = "startup"
        self.create_timer(0.01, self.poll_udp)
        self.get_logger().info("UDP bridge listening on 0.0.0.0:9760")

    def parse_packet(self, raw: bytes, source: tuple[str, int]) -> ScanPacket | None:
        try:
            fields = raw.decode("ascii").split("|")
            if len(fields) not in (10, 12) or fields[0] != "SCAN":
                return None
            beam_field = 11 if len(fields) == 12 else 9
            beams = []
            for token in fields[beam_field].split(";"):
                angle, distance, hit = token.split(",")
                parsed_angle = float(angle)
                parsed_distance = float(distance)
                parsed_hit = int(hit)
                if (
                    not math.isfinite(parsed_angle)
                    or not math.isfinite(parsed_distance)
                    or parsed_distance <= 0.0
                    or parsed_hit not in (0, 1)
                ):
                    continue
                beams.append((parsed_angle, parsed_distance, parsed_hit != 0))
            lidar_offset_x = float(fields[9]) if len(fields) == 12 else 0.0
            lidar_offset_y = float(fields[10]) if len(fields) == 12 else 0.0
            sequence = int(fields[2])
            numeric_metadata = (
                float(fields[3]), float(fields[4]), float(fields[5]),
                float(fields[6]), float(fields[7]), float(fields[8]),
                lidar_offset_x, lidar_offset_y,
            )
            if (
                not fields[1]
                or len(fields[1]) > 96
                or sequence < 0
                or not all(math.isfinite(value) for value in numeric_metadata)
                or numeric_metadata[5] <= numeric_metadata[4]
                or numeric_metadata[4] < 0.0
                or len(beams) < MIN_SLAM_RETURNS
            ):
                return None
            return ScanPacket(
                fields[1], sequence,
                numeric_metadata[0], numeric_metadata[1], numeric_metadata[2],
                numeric_metadata[3], numeric_metadata[4], numeric_metadata[5],
                numeric_metadata[6], numeric_metadata[7], beams, source,
            )
        except (UnicodeDecodeError, ValueError, OverflowError):
            return None

    def poll_udp(self) -> None:
        self.complete_pending_reset()
        while True:
            try:
                raw, source = self.sock.recvfrom(65507)
            except BlockingIOError:
                return
            packet = self.parse_packet(raw, source)
            if packet is None:
                continue
            sequence_action = classify_packet(
                self.active_session,
                self.last_sequence,
                packet.session,
                packet.sequence,
            )
            if sequence_action == "new_session":
                self.last_reset_reason = "session_changed"
                self.active_session = packet.session
                self.last_sequence = -1
                self.map_updates = 0
                self.graph_nodes = 0
                self.loop_edges = 0
                self.latest_map = None
                self.latest_corrected_pose = None
                self.pending_reset_packet = packet
                self.begin_reset_if_ready()
                continue
            # UDP may duplicate or reorder datagrams. A stale sequence is not
            # a mapping restart and must never erase the pose graph. The C++
            # runner gives every process/run a unique session ID, so an actual
            # restart always arrives through the session-change branch above.
            if sequence_action == "stale":
                continue
            self.last_sequence = packet.sequence
            if self.pending_reset_packet is not None:
                self.pending_reset_packet = packet
                continue
            self.latest_packet = packet
            self.publish_scan(packet)
            self.send_response()

    def begin_reset_if_ready(self) -> None:
        if self.reset_future is not None or not self.reset_client.service_is_ready():
            return
        request = Reset.Request()
        request.pause_new_measurements = False
        self.reset_future = self.reset_client.call_async(request)

    def complete_pending_reset(self) -> None:
        if self.pending_reset_packet is None:
            return
        self.begin_reset_if_ready()
        if self.reset_future is None or not self.reset_future.done():
            return
        try:
            response = self.reset_future.result()
            if response.result != Reset.Response.RESULT_SUCCESS:
                self.get_logger().warning(
                    f"slam_toolbox reset returned result={response.result}"
                )
        except Exception as error:
            self.get_logger().warning(f"slam_toolbox reset failed: {error}")
        self.reset_future = None
        packet = self.pending_reset_packet
        self.pending_reset_packet = None
        self.last_sequence = packet.sequence
        self.latest_packet = packet
        self.publish_scan(packet)
        self.send_response()

    def publish_scan(self, packet: ScanPacket) -> None:
        stamp = self.get_clock().now().to_msg()
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_link"
        transform.transform.translation.x = packet.x
        transform.transform.translation.y = packet.y
        qx, qy, qz, qw = yaw_to_quaternion(packet.yaw)
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self.tf.sendTransform(transform)

        laser_transform = TransformStamped()
        laser_transform.header.stamp = stamp
        laser_transform.header.frame_id = "base_link"
        laser_transform.child_frame_id = "laser"
        laser_transform.transform.translation.x = packet.lidar_offset_x
        laser_transform.transform.translation.y = packet.lidar_offset_y
        laser_transform.transform.rotation.w = 1.0
        self.tf.sendTransform(laser_transform)

        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = "laser"
        scan.range_min = max(packet.min_range, 0.01)
        scan.range_max = max(packet.max_range, scan.range_min + 0.01)
        if len(packet.beams) < 2:
            return
        # LidarHit contains only received returns, so its angular samples are
        # not guaranteed to be uniform. LaserScan requires a uniform lattice:
        # project the returns into one-degree bins and leave missing beams inf.
        beam_count = 360
        scan.angle_min = -math.pi
        scan.angle_increment = 2.0 * math.pi / beam_count
        scan.angle_max = scan.angle_min + (beam_count - 1) * scan.angle_increment
        scan.scan_time = 0.10
        scan.time_increment = scan.scan_time / beam_count
        ranges = [math.inf] * beam_count
        for world_angle, distance, hit in packet.beams:
            relative_angle = math.atan2(
                math.sin(world_angle - packet.yaw),
                math.cos(world_angle - packet.yaw),
            )
            index = round((relative_angle - scan.angle_min) / scan.angle_increment)
            index = min(max(index, 0), beam_count - 1)
            if hit and scan.range_min <= distance <= scan.range_max:
                ranges[index] = min(ranges[index], distance)
        if sum(math.isfinite(distance) for distance in ranges) < MIN_SLAM_RETURNS:
            return
        scan.ranges = ranges
        self.scan_pub.publish(scan)
        self.update_corrected_pose(packet)

    def on_map(self, message: OccupancyGrid) -> None:
        self.latest_map = message
        self.map_updates += 1
        self.send_response()

    def on_graph_visualization(self, message: MarkerArray) -> None:
        """Extract graph diagnostics exposed by the Jazzy 2.8.x package.

        Mapping nodes are individual red spheres in the ``slam_toolbox``
        namespace. Mapping constraints are published as a LINE_LIST; a
        connected odometry chain has N-1 constraints, therefore any additional
        constraints are loop-closure edges.
        """
        node_ids = {
            marker.id
            for marker in message.markers
            if marker.action == Marker.ADD
            and marker.ns == "slam_toolbox"
            and marker.type == Marker.SPHERE
        }
        edge_count = sum(
            len(marker.points) // 2
            for marker in message.markers
            if marker.action == Marker.ADD
            and marker.ns == "slam_toolbox_edges"
            and marker.id == 0
            and marker.type == Marker.LINE_LIST
        )
        self.graph_nodes = len(node_ids)
        self.loop_edges = max(0, edge_count - max(0, self.graph_nodes - 1))
        self.send_response()

    def update_corrected_pose(self, packet: ScanPacket) -> None:
        """Compose slam_toolbox's map->odom transform with current odometry."""
        try:
            transform = self.tf_buffer.lookup_transform("map", "odom", Time())
        except Exception:  # tf2 raises several lookup/connectivity subclasses
            self.latest_corrected_pose = None
            return
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        map_odom_yaw = quaternion_to_yaw(
            rotation.x, rotation.y, rotation.z, rotation.w,
        )
        cosine = math.cos(map_odom_yaw)
        sine = math.sin(map_odom_yaw)
        self.latest_corrected_pose = (
            translation.x + cosine * packet.x - sine * packet.y,
            translation.y + sine * packet.x + cosine * packet.y,
            math.atan2(
                math.sin(map_odom_yaw + packet.yaw),
                math.cos(map_odom_yaw + packet.yaw),
            ),
        )

    def send_response(self) -> None:
        packet = self.latest_packet
        if packet is None:
            return
        self.update_corrected_pose(packet)
        pose_valid = self.latest_corrected_pose is not None
        if pose_valid:
            pose_x, pose_y, pose_yaw = self.latest_corrected_pose
        else:
            pose_x, pose_y, pose_yaw = packet.x, packet.y, packet.yaw

        if self.latest_map is None:
            resolution, origin_x, origin_y, width, height = 0.025, 0.0, 0.0, 1, 1
            occupied = ""
        else:
            info = self.latest_map.info
            resolution = info.resolution
            origin_x = info.origin.position.x
            origin_y = info.origin.position.y
            width, height = info.width, info.height
            occupied = ",".join(
                str(index) for index, value in enumerate(self.latest_map.data) if value >= 50
            )
            free_indices = [
                str(index) for index, value in enumerate(self.latest_map.data) if value == 0
            ]
            free = ",".join(free_indices)
        if self.latest_map is None:
            free = ""
        response = (
            f"MAP|{packet.session}|{packet.sequence}|{int(pose_valid)}|"
            f"{pose_x:.9f}|{pose_y:.9f}|{pose_yaw:.9f}|{self.map_updates}|"
            f"{self.graph_nodes}|{self.loop_edges}|{resolution:.9f}|"
            f"{origin_x:.9f}|{origin_y:.9f}|{width}|{height}|{occupied}|{free}|"
            f"{self.last_reset_reason}"
        ).encode("ascii")
        # Small thesis arenas fit in one datagram. For a larger map retain all
        # occupied cells and progressively thin only the free-space display
        # cells, keeping scan matching and the ROS occupancy grid untouched.
        stride = 1
        while len(response) > 65507 and self.latest_map is not None and stride < 64:
            stride *= 2
            free = ",".join(free_indices[::stride])
            response = (
                f"MAP|{packet.session}|{packet.sequence}|{int(pose_valid)}|"
                f"{pose_x:.9f}|{pose_y:.9f}|{pose_yaw:.9f}|{self.map_updates}|"
                f"{self.graph_nodes}|{self.loop_edges}|{resolution:.9f}|"
                f"{origin_x:.9f}|{origin_y:.9f}|{width}|{height}|{occupied}|{free}|"
                f"{self.last_reset_reason}"
            ).encode("ascii")
        if len(response) <= 65507:
            self.sock.sendto(response, packet.source)


def main() -> None:
    rclpy.init()
    node = UdpSlamBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
