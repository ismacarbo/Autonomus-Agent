import argparse
import math
import struct
import time
from dataclasses import dataclass

from lidarLib import Lidar, LidarError, serial as pyserial

try:
    import numpy as np
except Exception:
    np = None

try:
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection
    from matplotlib.patches import FancyArrowPatch, Rectangle
except Exception:
    plt = None
    LineCollection = None
    FancyArrowPatch = None
    Rectangle = None


def wrap_angle(theta):
    return (theta + math.pi) % (2.0 * math.pi) - math.pi


@dataclass
class CalibrationConfig:
    lidar_x_offset_m: float = 0.075
    lidar_y_offset_m: float = 0.0
    lidar_yaw_offset_rad: float = 2.83
    min_range_m: float = 0.12
    max_range_m: float = 3.0
    front_half_angle_rad: float = 0.26
    body_length_m: float = 0.34
    body_width_m: float = 0.24


@dataclass
class ImuSample:
    mcu_time_ms: int
    yaw_rad: float
    yaw_rate_rad_s: float
    yaw_rel_rad: float


class ImuBridgeError(Exception):
    pass


class RspImuBridge:
    SOF1 = 0xAA
    SOF2 = 0x55
    VERSION = 0x01
    HEADER_LEN = 6
    MSG_ACK = 0x02
    MSG_ERROR = 0x03
    MSG_GYRO_ZERO_CMD = 0x13
    MSG_CONFIG_SET = 0x14
    MSG_HEARTBEAT_CMD = 0x15
    MSG_IMU_TELEMETRY = 0x20
    FLAG_ACK_REQ = 0x01
    PARAM_IMU_TELEMETRY_MS = 0x02
    VALUE_TYPE_UINT16 = 0x03

    def __init__(self, port, baudrate=115200, timeout=0.15, heartbeat_period_s=0.35):
        self.port = port
        self.baudrate = int(baudrate)
        self.timeout = float(timeout)
        self.heartbeat_period_s = float(heartbeat_period_s)
        self._serial = None
        self._rx = bytearray()
        self._seq = 0
        self._last_heartbeat_s = 0.0
        self._yaw_origin = None
        self.latest_sample = None

    def connect(self):
        if pyserial is None:
            raise ImuBridgeError("pyserial is not installed")
        try:
            self._serial = pyserial.Serial(self.port, self.baudrate, timeout=self.timeout)
            self._serial.reset_input_buffer()
            self._serial.reset_output_buffer()
        except Exception as exc:
            raise ImuBridgeError(f"Cannot open IMU port {self.port}: {exc}") from exc

    def disconnect(self):
        if self._serial is not None and self._serial.is_open:
            self._serial.close()
        self._serial = None
        self._rx.clear()

    def _require_serial(self):
        if self._serial is None or not self._serial.is_open:
            raise ImuBridgeError("IMU bridge not connected")

    @staticmethod
    def _crc16_update(crc, data_byte):
        crc ^= data_byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
        return crc

    @classmethod
    def _crc16_compute(cls, payload):
        crc = 0xFFFF
        for b in payload:
            crc = cls._crc16_update(crc, b)
        return crc

    def _send_frame(self, msg_type, payload=b"", flags=0):
        self._require_serial()
        header = bytes([
            self.VERSION,
            msg_type & 0xFF,
            flags & 0xFF,
            self._seq & 0xFF,
            len(payload) & 0xFF,
            (len(payload) >> 8) & 0xFF,
        ])
        crc = self._crc16_compute(header + payload)
        frame = bytes([self.SOF1, self.SOF2]) + header + payload + struct.pack("<H", crc)
        self._serial.write(frame)
        self._seq = (self._seq + 1) & 0xFF

    def send_heartbeat(self):
        host_ms = int(time.time() * 1000.0) & 0xFFFFFFFF
        payload = struct.pack("<IH", host_ms, 0)
        self._send_frame(self.MSG_HEARTBEAT_CMD, payload)
        self._last_heartbeat_s = time.time()

    def send_gyro_zero(self):
        self._send_frame(self.MSG_GYRO_ZERO_CMD, b"", flags=self.FLAG_ACK_REQ)

    def set_imu_period_ms(self, period_ms):
        period_ms = max(10, min(1000, int(period_ms)))
        payload = bytes([
            self.PARAM_IMU_TELEMETRY_MS,
            self.VALUE_TYPE_UINT16,
            period_ms & 0xFF,
            (period_ms >> 8) & 0xFF,
        ])
        self._send_frame(self.MSG_CONFIG_SET, payload, flags=self.FLAG_ACK_REQ)

    def _try_parse_frame(self):
        while len(self._rx) >= 2 and not (self._rx[0] == self.SOF1 and self._rx[1] == self.SOF2):
            self._rx.pop(0)
        if len(self._rx) < 2 + self.HEADER_LEN + 2:
            return None

        version = self._rx[2]
        if version != self.VERSION:
            self._rx.pop(0)
            return None
        payload_len = self._rx[6] | (self._rx[7] << 8)
        frame_len = 2 + self.HEADER_LEN + payload_len + 2
        if len(self._rx) < frame_len:
            return None

        frame = bytes(self._rx[:frame_len])
        del self._rx[:frame_len]
        header = frame[2:2 + self.HEADER_LEN]
        payload = frame[2 + self.HEADER_LEN:-2]
        crc_recv = struct.unpack("<H", frame[-2:])[0]
        crc_calc = self._crc16_compute(header + payload)
        if crc_recv != crc_calc:
            return None

        return {
            "msg_type": header[1],
            "flags": header[2],
            "seq": header[3],
            "payload": payload,
        }

    def _handle_frame(self, frame):
        if frame["msg_type"] != self.MSG_IMU_TELEMETRY:
            return
        payload = frame["payload"]
        if len(payload) != 20:
            return
        mcu_time_ms, yaw_mrad, yaw_rate_mrad_s, acc_x, acc_y, acc_z, gyro_z = struct.unpack(
            "<Iiihhhh",
            payload,
        )
        yaw_rad = yaw_mrad / 1000.0
        yaw_rate_rad_s = yaw_rate_mrad_s / 1000.0
        if self._yaw_origin is None:
            self._yaw_origin = yaw_rad
        yaw_rel_rad = wrap_angle(yaw_rad - self._yaw_origin)
        self.latest_sample = ImuSample(
            mcu_time_ms=int(mcu_time_ms),
            yaw_rad=float(yaw_rad),
            yaw_rate_rad_s=float(yaw_rate_rad_s),
            yaw_rel_rad=float(yaw_rel_rad),
        )

    def pump(self):
        self._require_serial()
        now = time.time()
        if now - self._last_heartbeat_s >= self.heartbeat_period_s:
            self.send_heartbeat()

        available = getattr(self._serial, "in_waiting", 0)
        if available:
            self._rx.extend(self._serial.read(available))
        else:
            chunk = self._serial.read(1)
            if chunk:
                self._rx.extend(chunk)

        while True:
            frame = self._try_parse_frame()
            if frame is None:
                break
            self._handle_frame(frame)

        return self.latest_sample


class LocalOccupancyGrid:
    def __init__(self, x_min, x_max, y_min, y_max, resolution):
        self.x_min = float(x_min)
        self.x_max = float(x_max)
        self.y_min = float(y_min)
        self.y_max = float(y_max)
        self.resolution = float(resolution)

        self.width = int(math.ceil((self.x_max - self.x_min) / self.resolution))
        self.height = int(math.ceil((self.y_max - self.y_min) / self.resolution))
        self.log_odds = np.zeros((self.height, self.width), dtype=np.float32)

        self.free_log_odds = -0.18
        self.hit_log_odds = 0.55
        self.log_odds_min = -3.0
        self.log_odds_max = 3.0

    def decay(self, factor):
        self.log_odds *= float(factor)

    def _world_to_cell(self, x, y):
        col = int((x - self.x_min) / self.resolution)
        row = int((y - self.y_min) / self.resolution)
        if col < 0 or col >= self.width or row < 0 or row >= self.height:
            return None
        return row, col

    def _sample_ray(self, start_xy, end_xy, include_endpoint):
        sx, sy = start_xy
        ex, ey = end_xy
        length = math.hypot(ex - sx, ey - sy)
        if length <= 1e-9:
            return [(ex, ey)] if include_endpoint else []
        step_m = max(self.resolution * 0.7, 0.02)
        steps = max(1, int(math.ceil(length / step_m)))
        last = steps if include_endpoint else steps - 1
        samples = []
        for i in range(max(0, last + 1)):
            alpha = min(1.0, max(0.0, i / steps))
            samples.append((sx + (ex - sx) * alpha, sy + (ey - sy) * alpha))
        return samples

    def inverse_sensor_update(self, origin_xy, hit_points_xy):
        ox, oy = origin_xy
        for hx, hy in hit_points_xy:
            free_samples = self._sample_ray((ox, oy), (hx, hy), include_endpoint=False)
            for x, y in free_samples:
                cell = self._world_to_cell(x, y)
                if cell is None:
                    continue
                self.log_odds[cell] = np.clip(
                    self.log_odds[cell] + self.free_log_odds,
                    self.log_odds_min,
                    self.log_odds_max,
                )

            cell = self._world_to_cell(hx, hy)
            if cell is not None:
                self.log_odds[cell] = np.clip(
                    self.log_odds[cell] + self.hit_log_odds,
                    self.log_odds_min,
                    self.log_odds_max,
                )

    def probability_map(self):
        return 1.0 / (1.0 + np.exp(-self.log_odds))

    @property
    def extent(self):
        return (self.x_min, self.x_max, self.y_min, self.y_max)


def scan_to_body_frame(scan, cfg, decimation):
    kept = []
    local_hits = []
    lidar_origin = np.array([cfg.lidar_x_offset_m, cfg.lidar_y_offset_m], dtype=float)
    ct = math.cos(cfg.lidar_yaw_offset_rad)
    st = math.sin(cfg.lidar_yaw_offset_rad)
    rot = np.array([[ct, -st], [st, ct]], dtype=float)

    step = max(1, int(decimation))
    for idx, (_, angle_deg, distance_mm) in enumerate(scan):
        if idx % step != 0:
            continue
        distance_m = distance_mm / 1000.0
        if distance_m < cfg.min_range_m or distance_m > cfg.max_range_m:
            continue

        beam_angle = math.radians(angle_deg)
        raw_xy = np.array(
            [distance_m * math.cos(beam_angle), distance_m * math.sin(beam_angle)],
            dtype=float,
        )
        body_xy = lidar_origin + rot @ raw_xy
        kept.append((beam_angle, distance_m, body_xy[0], body_xy[1]))
        local_hits.append((body_xy[0], body_xy[1]))

    return kept, np.asarray(local_hits, dtype=float) if local_hits else np.empty((0, 2), dtype=float)


def compute_front_distance(local_hits, half_angle_rad):
    if local_hits.size == 0:
        return float("nan"), 0, float("nan")
    angles = np.arctan2(local_hits[:, 1], local_hits[:, 0])
    dists = np.hypot(local_hits[:, 0], local_hits[:, 1])
    mask = np.abs(angles) <= half_angle_rad
    if np.any(mask):
        return float(np.min(dists[mask])), int(np.count_nonzero(mask)), float("nan")

    soft_half_angle = min(max(half_angle_rad * 1.8, half_angle_rad + 0.08), 0.45)
    soft_mask = np.abs(angles) <= soft_half_angle
    if np.any(soft_mask):
        return float("nan"), 0, float(np.min(dists[soft_mask]))

    return float("nan"), 0, float("nan")


def build_ray_segments(origin_xy, hit_points):
    if hit_points.size == 0:
        return []
    origin = np.repeat(np.asarray(origin_xy, dtype=float)[None, :], hit_points.shape[0], axis=0)
    return np.stack([origin, hit_points], axis=1)


def rotate_points(points, angle_rad):
    if points.size == 0:
        return np.empty((0, 2), dtype=float)
    ct = math.cos(angle_rad)
    st = math.sin(angle_rad)
    rot = np.array([[ct, -st], [st, ct]], dtype=float)
    return points @ rot.T


def rotate_point(point_xy, angle_rad):
    ct = math.cos(angle_rad)
    st = math.sin(angle_rad)
    x, y = point_xy
    return np.array([ct * x - st * y, st * x + ct * y], dtype=float)


class CalibrationViewer:
    def __init__(self, grid, cfg):
        if plt is None or LineCollection is None or FancyArrowPatch is None or Rectangle is None:
            raise RuntimeError("matplotlib is required for the calibration GUI")
        self.grid = grid
        self.cfg = cfg
        self.fig, (self.ax_grid, self.ax_hits) = plt.subplots(1, 2, figsize=(15, 7))

        self.grid_img = self.ax_grid.imshow(
            np.full((grid.height, grid.width), 0.5, dtype=float),
            origin="lower",
            extent=grid.extent,
            cmap="gray_r",
            vmin=0.0,
            vmax=1.0,
            interpolation="nearest",
        )
        self.grid_rays = LineCollection([], colors="#61d48c", linewidths=0.35, alpha=0.35)
        self.ax_grid.add_collection(self.grid_rays)
        self.ax_grid.set_title("Occupancy Grid")
        self.ax_grid.set_xlabel("X [m]")
        self.ax_grid.set_ylabel("Y [m]")
        self.ax_grid.set_aspect("equal", "box")
        self.ax_grid.grid(True, alpha=0.18)

        self.grid_heading = FancyArrowPatch(
            posA=(0.0, 0.0),
            posB=(0.35, 0.0),
            arrowstyle="-|>",
            mutation_scale=16,
            linewidth=2.2,
            color="#ff5555",
        )
        self.ax_grid.add_patch(self.grid_heading)

        self.ax_hits.set_title("Live Hits + Headings")
        self.ax_hits.set_xlabel("Forward X [m]")
        self.ax_hits.set_ylabel("Left Y [m]")
        self.ax_hits.set_aspect("equal", "box")
        self.ax_hits.grid(True, alpha=0.18)
        self.hit_scatter = self.ax_hits.scatter([], [], s=10, c="#4fb3ff", alpha=0.85)
        self.local_rays = LineCollection([], colors="#61d48c", linewidths=0.45, alpha=0.55)
        self.ax_hits.add_collection(self.local_rays)

        half_len = cfg.body_length_m * 0.5
        half_wid = cfg.body_width_m * 0.5
        robot_rect = Rectangle(
            (-half_len, -half_wid),
            cfg.body_length_m,
            cfg.body_width_m,
            linewidth=1.5,
            edgecolor="#ffb86c",
            facecolor="none",
        )
        self.ax_hits.add_patch(robot_rect)

        lidar_rect = Rectangle(
            (cfg.lidar_x_offset_m - 0.025, cfg.lidar_y_offset_m - 0.025),
            0.05,
            0.05,
            linewidth=1.2,
            edgecolor="#ff5555",
            facecolor="none",
        )
        self.ax_hits.add_patch(lidar_rect)

        self.robot_heading = FancyArrowPatch(
            posA=(0.0, 0.0),
            posB=(0.35, 0.0),
            arrowstyle="-|>",
            mutation_scale=16,
            linewidth=2.2,
            color="#ff5555",
        )
        self.ax_hits.add_patch(self.robot_heading)

        lidar_heading_x = cfg.lidar_x_offset_m + 0.30 * math.cos(cfg.lidar_yaw_offset_rad)
        lidar_heading_y = cfg.lidar_y_offset_m + 0.30 * math.sin(cfg.lidar_yaw_offset_rad)
        self.lidar_heading = FancyArrowPatch(
            posA=(cfg.lidar_x_offset_m, cfg.lidar_y_offset_m),
            posB=(lidar_heading_x, lidar_heading_y),
            arrowstyle="-|>",
            mutation_scale=16,
            linewidth=2.0,
            color="#ffd166",
        )
        self.ax_hits.add_patch(self.lidar_heading)

        self.status_text = self.ax_hits.text(
            0.02,
            0.98,
            "",
            transform=self.ax_hits.transAxes,
            ha="left",
            va="top",
            fontsize=10,
            bbox={"facecolor": "#101418", "alpha": 0.78, "edgecolor": "#2a3a45"},
            color="white",
        )

        x_min, x_max, y_min, y_max = grid.extent
        self.ax_grid.set_xlim(x_min, x_max)
        self.ax_grid.set_ylim(y_min, y_max)
        self.ax_hits.set_xlim(x_min, x_max)
        self.ax_hits.set_ylim(y_min, y_max)

        self.fig.suptitle("LiDAR Calibration Viewer", fontsize=14)
        self.fig.tight_layout()
        plt.ion()
        plt.show(block=False)

    def is_open(self):
        return plt.fignum_exists(self.fig.number)

    def update(self,
               prob_map,
               map_ray_segments,
               local_hit_points,
               local_ray_segments,
               front_distance_m,
               front_count,
               front_soft_distance_m,
               scan_idx,
               imu_sample):
        self.grid_img.set_data(prob_map)
        self.grid_rays.set_segments(map_ray_segments)
        self.local_rays.set_segments(local_ray_segments)

        if local_hit_points.size > 0:
            self.hit_scatter.set_offsets(local_hit_points)
        else:
            self.hit_scatter.set_offsets(np.empty((0, 2), dtype=float))

        imu_str = "n/a"
        if imu_sample is not None:
            imu_str = f"{math.degrees(imu_sample.yaw_rel_rad):.1f} deg"
            grid_head = np.array([0.35 * math.cos(imu_sample.yaw_rel_rad), 0.35 * math.sin(imu_sample.yaw_rel_rad)])
            self.grid_heading.set_positions((0.0, 0.0), (grid_head[0], grid_head[1]))
            self.ax_grid.set_title("Occupancy Grid (IMU Frame)")
        else:
            self.grid_heading.set_positions((0.0, 0.0), (0.35, 0.0))
            self.ax_grid.set_title("Occupancy Grid (Body Frame)")

        yaw_deg = math.degrees(self.cfg.lidar_yaw_offset_rad)
        front_str = f"{front_distance_m:.3f} m" if math.isfinite(front_distance_m) else "n/a"
        front_soft_str = f"{front_soft_distance_m:.3f} m" if math.isfinite(front_soft_distance_m) else "n/a"
        self.status_text.set_text(
            f"scan: {scan_idx}\n"
            f"front core: {front_str}\n"
            f"front beams: {front_count}\n"
            f"front soft: {front_soft_str}\n"
            f"lidar_x_offset: {self.cfg.lidar_x_offset_m:.3f} m\n"
            f"lidar_y_offset: {self.cfg.lidar_y_offset_m:.3f} m\n"
            f"lidar_yaw_offset: {yaw_deg:.1f} deg\n"
            f"imu_yaw_rel: {imu_str}\n"
            f"red = robot heading\n"
            f"yellow = lidar heading"
        )

        self.fig.canvas.draw_idle()
        plt.pause(0.001)


def parse_args():
    parser = argparse.ArgumentParser(description="Quick LiDAR calibration viewer with occupancy grid")
    parser.add_argument("--lidar-port", default="/dev/ttyUSB0", help="RPLidar serial port")
    parser.add_argument("--baudrate", type=int, default=115200, help="LiDAR baudrate")
    parser.add_argument("--timeout", type=float, default=1.0, help="Serial timeout [s]")
    parser.add_argument("--motor-pwm", type=int, default=660, help="Motor PWM [0..1023]")
    parser.add_argument("--grid", nargs=4, type=float, default=[-1.5, 1.5, -1.5, 1.5],
                        metavar=("XMIN", "XMAX", "YMIN", "YMAX"))
    parser.add_argument("--resolution", type=float, default=0.03, help="Grid resolution [m]")
    parser.add_argument("--min-range", type=float, default=0.12, help="Min valid range [m]")
    parser.add_argument("--max-range", type=float, default=3.0, help="Max valid range [m]")
    parser.add_argument("--scan-decimation", type=int, default=1, help="Keep 1 sample every N")
    parser.add_argument("--min-scan-points", type=int, default=80, help="Minimum points per revolution")
    parser.add_argument("--grid-decay", type=float, default=0.985, help="Occupancy decay per scan")
    parser.add_argument("--max-scans", type=int, default=0, help="Stop after N scans (0=infinite)")
    parser.add_argument("--no-gui", action="store_true", help="Disable live GUI")
    parser.add_argument("--save-map", default="occupancy_grid.png", help="Output occupancy image")
    parser.add_argument("--save-points", default="trajectory.csv", help="Output latest points CSV")
    parser.add_argument("--imu-port", default="", help="Optional Arduino/RSP serial port for IMU yaw")
    parser.add_argument("--imu-baudrate", type=int, default=115200, help="Arduino IMU baudrate")
    parser.add_argument("--imu-timeout", type=float, default=0.15, help="Arduino IMU serial timeout [s]")
    parser.add_argument("--imu-heartbeat-ms", type=int, default=350, help="Heartbeat period for RSP IMU link [ms]")
    parser.add_argument("--imu-telemetry-ms", type=int, default=40, help="Requested IMU telemetry period [ms]")
    parser.add_argument("--no-imu-zero", action="store_true", help="Do not send GYRO_ZERO when connecting to IMU")

    parser.add_argument("--lidar-x-offset", type=float, default=0.075, help="LiDAR X offset in body frame [m]")
    parser.add_argument(
        "--lidar-y-offset",
        type=float,
        default=-0.04,
        help="LiDAR Y offset in body frame [m]. Negative means the LiDAR is mounted slightly on the robot right side",
    )
    parser.add_argument(
        "--lidar-yaw-deg",
        type=float,
        default=162.0,
        help="LiDAR yaw offset in body frame [deg]. Default is a calibration guess for reversed + left-rotated mount",
    )
    parser.add_argument("--body-length", type=float, default=0.34, help="Robot body length [m]")
    parser.add_argument("--body-width", type=float, default=0.24, help="Robot body width [m]")
    parser.add_argument("--front-half-angle-deg", type=float, default=15.0, help="Front core half-angle [deg]")
    return parser.parse_args()


def save_outputs(grid, latest_hits, out_map_path, out_points_path):
    if plt is not None:
        fig, ax = plt.subplots(figsize=(7, 7))
        ax.imshow(
            grid.probability_map(),
            origin="lower",
            extent=grid.extent,
            cmap="gray_r",
            vmin=0.0,
            vmax=1.0,
            interpolation="nearest",
        )
        ax.set_title("LiDAR Calibration Occupancy Grid")
        ax.set_xlabel("Forward X [m]")
        ax.set_ylabel("Left Y [m]")
        ax.set_aspect("equal", "box")
        fig.tight_layout()
        fig.savefig(out_map_path, dpi=220, bbox_inches="tight")
        plt.close(fig)

    header = "x_m,y_m"
    points = latest_hits if latest_hits.size > 0 else np.empty((0, 2), dtype=float)
    np.savetxt(out_points_path, points, delimiter=",", header=header, comments="")


def run():
    args = parse_args()
    if np is None:
        raise RuntimeError("numpy is not installed. Install numpy to run the calibration viewer.")
    if plt is None and not args.no_gui:
        raise RuntimeError("matplotlib is not installed. Re-run with --no-gui or install matplotlib.")

    cfg = CalibrationConfig(
        lidar_x_offset_m=args.lidar_x_offset,
        lidar_y_offset_m=args.lidar_y_offset,
        lidar_yaw_offset_rad=math.radians(args.lidar_yaw_deg),
        min_range_m=args.min_range,
        max_range_m=args.max_range,
        front_half_angle_rad=math.radians(args.front_half_angle_deg),
        body_length_m=args.body_length,
        body_width_m=args.body_width,
    )

    grid = LocalOccupancyGrid(*args.grid, resolution=args.resolution)
    viewer = None if args.no_gui else CalibrationViewer(grid, cfg)

    lidar = Lidar(
        args.lidar_port,
        baudrate=args.baudrate,
        timeout=args.timeout,
        motor_pwm=args.motor_pwm,
    )

    latest_hits = np.empty((0, 2), dtype=float)
    scan_idx = 0
    last_log_ts = 0.0
    imu_bridge = None
    imu_sample = None

    print(f"[INFO] starting LiDAR calibration on {args.lidar_port} @ {args.baudrate} baud")
    print(
        f"[INFO] offsets: x={cfg.lidar_x_offset_m:.3f} m, y={cfg.lidar_y_offset_m:.3f} m, "
        f"yaw={math.degrees(cfg.lidar_yaw_offset_rad):.1f} deg"
    )

    try:
        lidar.connect()
        print("[LIDAR] INFO:", lidar.getInfo())
        print("[LIDAR] HEALTH:", lidar.getHealth())
        lidar.startScan()

        if args.imu_port:
            imu_bridge = RspImuBridge(
                args.imu_port,
                baudrate=args.imu_baudrate,
                timeout=args.imu_timeout,
                heartbeat_period_s=max(0.10, args.imu_heartbeat_ms / 1000.0),
            )
            imu_bridge.connect()
            imu_bridge.send_heartbeat()
            imu_bridge.set_imu_period_ms(args.imu_telemetry_ms)
            if not args.no_imu_zero:
                imu_bridge.send_gyro_zero()
            print(f"[IMU] connected on {args.imu_port} @ {args.imu_baudrate} baud")

        while True:
            for raw_scan in lidar.iterScan(min_points=args.min_scan_points):
                scan_idx += 1
                if imu_bridge is not None:
                    imu_sample = imu_bridge.pump()
                _, local_hits = scan_to_body_frame(raw_scan, cfg, args.scan_decimation)
                latest_hits = local_hits

                grid.decay(args.grid_decay)
                map_origin = np.array([cfg.lidar_x_offset_m, cfg.lidar_y_offset_m], dtype=float)
                map_hits = local_hits
                if imu_sample is not None:
                    map_origin = rotate_point(map_origin, imu_sample.yaw_rel_rad)
                    map_hits = rotate_points(local_hits, imu_sample.yaw_rel_rad)

                if map_hits.size > 0:
                    grid.inverse_sensor_update((map_origin[0], map_origin[1]), map_hits)

                front_distance, front_count, front_soft_distance = compute_front_distance(
                    local_hits,
                    cfg.front_half_angle_rad,
                )
                map_ray_segments = build_ray_segments((map_origin[0], map_origin[1]), map_hits)
                local_ray_segments = build_ray_segments((cfg.lidar_x_offset_m, cfg.lidar_y_offset_m), local_hits)

                if viewer is not None:
                    viewer.update(
                        prob_map=grid.probability_map(),
                        map_ray_segments=map_ray_segments,
                        local_hit_points=local_hits,
                        local_ray_segments=local_ray_segments,
                        front_distance_m=front_distance,
                        front_count=front_count,
                        front_soft_distance_m=front_soft_distance,
                        scan_idx=scan_idx,
                        imu_sample=imu_sample,
                    )
                    if not viewer.is_open():
                        print("[INFO] GUI closed, stopping")
                        raise KeyboardInterrupt

                now = time.time()
                if now - last_log_ts > 0.7:
                    last_log_ts = now
                    front_str = f"{front_distance:.3f}" if math.isfinite(front_distance) else "n/a"
                    front_soft_str = (
                        f"{front_soft_distance:.3f}" if math.isfinite(front_soft_distance) else "n/a"
                    )
                    imu_str = (
                        f"{math.degrees(imu_sample.yaw_rel_rad):.1f}" if imu_sample is not None else "n/a"
                    )
                    print(
                        f"[CAL] scan={scan_idx:5d} hits={local_hits.shape[0]:4d} "
                        f"front={front_str}m beams={front_count:2d} soft={front_soft_str}m "
                        f"yaw_off={math.degrees(cfg.lidar_yaw_offset_rad):.1f}deg "
                        f"imu={imu_str}deg"
                    )

                if args.max_scans > 0 and scan_idx >= args.max_scans:
                    print(f"[INFO] reached --max-scans={args.max_scans}")
                    raise KeyboardInterrupt
    except KeyboardInterrupt:
        print("\n[INFO] calibration stopped")
    except LidarError as exc:
        print(f"[ERROR] lidar failure: {exc}")
    finally:
        try:
            lidar.stopScan()
        except Exception:
            pass
        try:
            lidar.disconnect()
        except Exception:
            pass
        if imu_bridge is not None:
            try:
                imu_bridge.disconnect()
            except Exception:
                pass

    save_outputs(grid, latest_hits, args.save_map, args.save_points)
    print(f"[INFO] map saved to: {args.save_map}")
    print(f"[INFO] latest hits saved to: {args.save_points}")

    if viewer is not None and viewer.is_open():
        plt.ioff()
        plt.show()


if __name__ == "__main__":
    run()
