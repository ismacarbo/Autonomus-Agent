import struct
import time

try:
    import serial
except Exception:
    serial = None


class LidarError(Exception):
    pass


class Lidar:
    SYNC_A = 0xA5
    SYNC_B = 0x5A
    DESCRIPTOR_LEN = 7

    CMD_STOP = 0x25
    CMD_SCAN = 0x20
    CMD_FORCE_SCAN = 0x21
    CMD_RESET = 0x40
    CMD_GET_INFO = 0x50
    CMD_GET_HEALTH = 0x52
    CMD_SET_PWM = 0xF0

    DEFAULT_MOTOR_PWM = 660
    MAX_MOTOR_PWM = 1023

    def __init__(self, port, baudrate=115200, timeout=1.0, motor_pwm=DEFAULT_MOTOR_PWM):
        self.port = port
        self.baudrate = int(baudrate)
        self.timeout = float(timeout)
        self._motor_speed = int(max(0, min(self.MAX_MOTOR_PWM, motor_pwm)))
        self._serial = None
        self.motor_running = False
        self.scanning = False

    def _require_serial(self):
        if serial is None:
            raise LidarError("pyserial is not installed")
        if self._serial is None or not self._serial.is_open:
            raise LidarError("LiDAR not connected")

    def connect(self):
        if serial is None:
            raise LidarError("Cannot open LiDAR because pyserial is not installed")
        try:
            self._serial = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
            self._serial.reset_input_buffer()
            self._serial.reset_output_buffer()
        except serial.SerialException as exc:
            raise LidarError(f"Cannot open {self.port}: {exc}") from exc

    def disconnect(self):
        if self._serial is not None and self._serial.is_open:
            self._serial.close()
        self._serial = None
        self.motor_running = False
        self.scanning = False

    def _write(self, payload):
        self._require_serial()
        self._serial.write(payload)

    def _read_exact(self, size, timeout=None):
        self._require_serial()
        timeout_s = self.timeout if timeout is None else float(timeout)
        deadline = time.time() + max(0.05, timeout_s)
        out = bytearray()
        while len(out) < size and time.time() < deadline:
            chunk = self._serial.read(size - len(out))
            if chunk:
                out.extend(chunk)
        if len(out) != size:
            raise LidarError(f"Timeout reading {size} bytes (got {len(out)})")
        return bytes(out)

    def send_command(self, cmd, payload=None):
        if payload:
            pkt = bytes([self.SYNC_A, cmd, len(payload)]) + payload
            chk = 0
            for b in pkt:
                chk ^= b
            pkt += bytes([chk])
        else:
            pkt = bytes([self.SYNC_A, cmd])
        self._write(pkt)

    def sendCmd(self, cmd):
        self.send_command(cmd)

    def read_descriptor(self, timeout=None):
        self._require_serial()
        timeout_s = self.timeout if timeout is None else float(timeout)
        deadline = time.time() + max(0.05, timeout_s)
        state = 0

        while time.time() < deadline:
            b = self._serial.read(1)
            if not b:
                continue
            value = b[0]
            if state == 0:
                state = 1 if value == self.SYNC_A else 0
            elif state == 1:
                if value == self.SYNC_B:
                    rest = self._read_exact(self.DESCRIPTOR_LEN - 2, timeout_s)
                    desc = bytes([self.SYNC_A, self.SYNC_B]) + rest
                    size_mode = int.from_bytes(desc[2:6], byteorder="little", signed=False)
                    return {
                        "size": size_mode & 0x3FFFFFFF,
                        "mode": (size_mode >> 30) & 0x3,
                        "dtype": desc[6],
                        "raw": desc,
                    }
                state = 1 if value == self.SYNC_A else 0

        raise LidarError("Timeout waiting for descriptor")

    def readDescriptor(self):
        return self.read_descriptor()["size"]

    def read_response(self, size, timeout=None):
        return self._read_exact(size, timeout=timeout)

    def readResponse(self, size):
        return self.read_response(size)

    def get_info(self):
        self._require_serial()
        self._serial.reset_input_buffer()
        self.send_command(self.CMD_GET_INFO)
        desc = self.read_descriptor(timeout=1.5)
        raw = self.read_response(desc["size"], timeout=1.5)
        if len(raw) < 20:
            raise LidarError("GET_INFO response too short")
        return {
            "model": raw[0],
            "firmware": (raw[2], raw[1]),
            "hardware": raw[3],
            "serial": raw[4:20].hex().upper(),
        }

    def getInfo(self):
        return self.get_info()

    def get_health(self):
        self._require_serial()
        self._serial.reset_input_buffer()
        self.send_command(self.CMD_GET_HEALTH)
        desc = self.read_descriptor(timeout=1.5)
        raw = self.read_response(desc["size"], timeout=1.5)
        if len(raw) < 3:
            raise LidarError("GET_HEALTH response too short")
        status = {0: "Good", 1: "Warning", 2: "Error"}.get(raw[0], "Unknown")
        error_code = (raw[2] << 8) | raw[1]
        return status, error_code

    def getHealth(self):
        return self.get_health()

    def set_motor_pwm(self, pwm):
        self._require_serial()
        pwm = int(max(0, min(self.MAX_MOTOR_PWM, pwm)))
        self.send_command(self.CMD_SET_PWM, payload=struct.pack("<H", pwm))
        self._motor_speed = pwm

    def start_motor(self):
        self._require_serial()
        self._serial.setDTR(False)
        self.set_motor_pwm(self._motor_speed)
        self.motor_running = True
        time.sleep(0.08)

    def stop_motor(self):
        self._require_serial()
        self.set_motor_pwm(0)
        self._serial.setDTR(True)
        self.motor_running = False
        time.sleep(0.08)

    def start_scan(self, force=False):
        self._require_serial()
        if not self.motor_running:
            self.start_motor()

        self._serial.reset_input_buffer()
        self.send_command(self.CMD_FORCE_SCAN if force else self.CMD_SCAN)
        desc = self.read_descriptor(timeout=2.0)
        if desc["size"] != 5:
            raise LidarError(f"Unexpected scan descriptor size: {desc['size']}")
        self.scanning = True

    def startScan(self):
        self.start_scan(force=False)

    def stop_scan(self, stop_motor=True):
        if self._serial is None or not self._serial.is_open:
            return
        try:
            self.send_command(self.CMD_STOP)
            time.sleep(0.05)
        except Exception:
            pass
        self.scanning = False
        if stop_motor and self.motor_running:
            try:
                self.stop_motor()
            except Exception:
                pass

    def stopScan(self):
        self.stop_scan(stop_motor=True)

    @staticmethod
    def process_scan_packet(raw):
        if len(raw) != 5:
            raise LidarError("Scan packet must be 5 bytes")

        b0, b1, b2, b3, b4 = raw
        start_flag = (b0 & 0x01) != 0
        inv_start = ((b0 >> 1) & 0x01) != 0
        if start_flag == inv_start:
            raise LidarError("Invalid start/inverse-start flags")
        if (b1 & 0x01) != 0x01:
            raise LidarError("Invalid check bit in scan packet")

        quality = b0 >> 2
        angle_deg = (((b1 >> 1) | (b2 << 7)) / 64.0) % 360.0
        distance_mm = (b3 | (b4 << 8)) / 4.0
        return start_flag, quality, angle_deg, distance_mm

    @staticmethod
    def procesScan(raw):
        return Lidar.process_scan_packet(raw)

    def _read_scan_node(self):
        self._require_serial()
        if not self.scanning:
            raise LidarError("Scan not started")

        deadline = time.time() + max(0.05, self.timeout)
        buf = bytearray()
        invalid_packets = 0

        while time.time() < deadline:
            chunk = self._serial.read(1)
            if not chunk:
                continue
            buf += chunk
            if len(buf) > 5:
                buf = buf[-5:]
            if len(buf) < 5:
                continue

            b0, b1, _, _, _ = buf
            start_flag = (b0 & 0x01) != 0
            inv_start = ((b0 >> 1) & 0x01) != 0
            check_bit_ok = (b1 & 0x01) == 0x01
            if start_flag == inv_start or not check_bit_ok:
                invalid_packets += 1
                if invalid_packets > 250:
                    raise LidarError("Too many invalid scan packets while resynchronizing")
                continue

            try:
                return self.process_scan_packet(bytes(buf))
            except LidarError:
                invalid_packets += 1
                if invalid_packets > 250:
                    raise

        raise LidarError("Timeout reading scan node")

    def iter_measurements(self, max_bad_packets=400):
        bad_packets = 0
        while True:
            try:
                yield self._read_scan_node()
                bad_packets = 0
            except LidarError:
                bad_packets += 1
                if bad_packets > max_bad_packets:
                    raise

    def iter_scans(self, min_points=60):
        min_points = max(1, int(min_points))
        scan_buf = []
        for new_scan, quality, angle_deg, dist_mm in self.iter_measurements():
            if new_scan and scan_buf:
                if len(scan_buf) >= min_points:
                    yield scan_buf
                scan_buf = []
            if dist_mm > 0.0:
                scan_buf.append((quality, angle_deg, dist_mm))

    def iterScan(self, min_points=60):
        yield from self.iter_scans(min_points=min_points)
