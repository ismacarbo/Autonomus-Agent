#!/usr/bin/env python3
"""UDP adapter between the C++ viewer and the real ROS 2 slam_toolbox node."""

from __future__ import annotations

import math
import socket
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster


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
        self.create_subscription(OccupancyGrid, "/map", self.on_map, 2)
        self.create_subscription(PoseWithCovarianceStamped, "/pose", self.on_pose, 10)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        self.sock.bind(("0.0.0.0", 9760))
        self.latest_map: OccupancyGrid | None = None
        self.latest_pose: PoseWithCovarianceStamped | None = None
        self.latest_packet: ScanPacket | None = None
        self.map_updates = 0
        self.graph_nodes = 0
        self.loop_edges = 0
        self.active_session = ""
        self.create_timer(0.01, self.poll_udp)
        self.get_logger().info("UDP bridge listening on 0.0.0.0:9760")

    def parse_packet(self, raw: bytes, source: tuple[str, int]) -> ScanPacket | None:
        try:
            fields = raw.decode("ascii").split("|")
            if len(fields) != 10 or fields[0] != "SCAN":
                return None
            beams = []
            for token in fields[9].split(";"):
                angle, distance, hit = token.split(",")
                beams.append((float(angle), float(distance), int(hit) != 0))
            return ScanPacket(
                fields[1], int(fields[2]), float(fields[3]),
                float(fields[4]), float(fields[5]), float(fields[6]),
                float(fields[7]), float(fields[8]), beams, source,
            )
        except (UnicodeDecodeError, ValueError):
            return None

    def poll_udp(self) -> None:
        while True:
            try:
                raw, source = self.sock.recvfrom(65507)
            except BlockingIOError:
                return
            packet = self.parse_packet(raw, source)
            if packet is None:
                continue
            if packet.session != self.active_session:
                self.active_session = packet.session
                self.map_updates = 0
                self.graph_nodes = 0
                self.loop_edges = 0
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

        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = "base_link"
        scan.range_min = max(packet.min_range, 0.01)
        scan.range_max = max(packet.max_range, scan.range_min + 0.01)
        ordered = sorted(packet.beams, key=lambda beam: beam[0])
        if len(ordered) < 2:
            return
        scan.angle_min = ordered[0][0] - packet.yaw
        scan.angle_max = ordered[-1][0] - packet.yaw
        scan.angle_increment = (scan.angle_max - scan.angle_min) / (len(ordered) - 1)
        scan.scan_time = 0.10
        scan.time_increment = scan.scan_time / len(ordered)
        scan.ranges = [
            distance if hit and scan.range_min <= distance <= scan.range_max else math.inf
            for _, distance, hit in ordered
        ]
        self.scan_pub.publish(scan)

    def on_map(self, message: OccupancyGrid) -> None:
        self.latest_map = message
        self.map_updates += 1
        self.send_response()

    def on_pose(self, message: PoseWithCovarianceStamped) -> None:
        self.latest_pose = message
        self.send_response()

    def send_response(self) -> None:
        packet = self.latest_packet
        if packet is None:
            return
        pose_valid = self.latest_pose is not None
        if pose_valid:
            pose = self.latest_pose.pose.pose
            pose_x = pose.position.x
            pose_y = pose.position.y
            pose_yaw = quaternion_to_yaw(
                pose.orientation.x, pose.orientation.y,
                pose.orientation.z, pose.orientation.w,
            )
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
        response = (
            f"MAP|{packet.session}|{packet.sequence}|{int(pose_valid)}|"
            f"{pose_x:.9f}|{pose_y:.9f}|{pose_yaw:.9f}|{self.map_updates}|"
            f"{self.graph_nodes}|{self.loop_edges}|{resolution:.9f}|"
            f"{origin_x:.9f}|{origin_y:.9f}|{width}|{height}|{occupied}"
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
