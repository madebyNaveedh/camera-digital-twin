"""
LiDAR Interface for Real RPLIDAR Hardware and High-Fidelity Synthetic Simulation.
Supports real serial communication over /dev/ttyUSB* with automatic fallback.
"""

from dataclasses import dataclass
from typing import Optional, Tuple, List, Dict
import time
import math
import threading
import logging
import numpy as np

try:
    import serial
except ImportError:
    serial = None

logger = logging.getLogger("LidarInterface")


@dataclass
class LidarScan:
    timestamp: float
    angles_deg: np.ndarray        # (N,) array of angles [0, 360)
    ranges_m: np.ndarray          # (N,) array of ranges in meters
    qualities: np.ndarray         # (N,) signal qualities
    is_hardware: bool             # True if from physical serial device
    source_info: str


class RPLidarHardwareDriver:
    """
    Native pure-Python driver for RPLIDAR A1/A2 series over USB serial.
    Implements the standard Slamtec RPLidar communication protocol.
    """
    SYNC_BYTE = b"\xA5"
    SYNC_BYTE2 = b"\x5A"
    CMD_STOP = b"\xA5\x25"
    CMD_RESET = b"\xA5\x40"
    CMD_SCAN = b"\xA5\x60"
    CMD_GET_INFO = b"\xA5\x50"
    CMD_GET_HEALTH = b"\xA5\x52"

    def __init__(self, port: str = "/dev/ttyUSB0", baudrate: int = 115200, timeout: float = 1.0):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser: Optional[serial.Serial] = None
        self.is_scanning = False

    def connect(self) -> bool:
        if serial is None:
            logger.error("pyserial is not available.")
            return False
        try:
            logger.info(f"Connecting to RPLIDAR on {self.port} at {self.baudrate} baud...")
            self.ser = serial.Serial(
                self.port,
                baudrate=self.baudrate,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=self.timeout
            )
            self.stop_motor()
            time.sleep(0.1)
            self.ser.reset_input_buffer()
            logger.info("Connected to RPLIDAR hardware.")
            return True
        except Exception as e:
            logger.warning(f"Could not open LiDAR serial port {self.port}: {e}")
            self.ser = None
            return False

    def start_scan(self) -> bool:
        if not self.ser or not self.ser.is_open:
            return False
        try:
            self.ser.write(self.CMD_SCAN)
            # Expect 7-byte response descriptor
            resp = self.ser.read(7)
            if len(resp) == 7 and resp[0:2] == b"\xA5\x5A":
                self.is_scanning = True
                logger.info("RPLIDAR scan started.")
                return True
            else:
                logger.warning("RPLIDAR scan response descriptor mismatch.")
                return False
        except Exception as e:
            logger.error(f"Error starting RPLIDAR scan: {e}")
            return False

    def stop_motor(self):
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(self.CMD_STOP)
                time.sleep(0.05)
                self.ser.reset_input_buffer()
            except Exception:
                pass
        self.is_scanning = False

    def read_scan(self, max_read_timeout: float = 0.5) -> Optional[LidarScan]:
        """
        Reads one complete 360-degree rotation of 5-byte scan packets.
        """
        if not self.ser or not self.is_scanning:
            return None

        angles = []
        ranges = []
        qualities = []
        start_time = time.time()
        saw_first_rotation = False

        while (time.time() - start_time) < max_read_timeout:
            raw = self.ser.read(5)
            if len(raw) < 5:
                continue

            b0, b1, b2, b3, b4 = raw
            sync_quality = b0
            s_flag = bool(sync_quality & 0x01)
            inv_s_flag = bool(sync_quality & 0x02)

            # Check packet validity: s_flag and inv_s_flag must be complementary
            if s_flag == inv_s_flag:
                # Out of sync, skip 1 byte to realign
                self.ser.read(1)
                continue

            if s_flag:
                if saw_first_rotation and len(angles) > 30:
                    # Completed one 360 deg sweep!
                    return LidarScan(
                        timestamp=time.time(),
                        angles_deg=np.array(angles, dtype=np.float64),
                        ranges_m=np.array(ranges, dtype=np.float64),
                        qualities=np.array(qualities, dtype=np.float64),
                        is_hardware=True,
                        source_info=f"Hardware RPLidar ({self.port})"
                    )
                saw_first_rotation = True
                angles.clear()
                ranges.clear()
                qualities.clear()

            # Parse angle and distance
            # Check bit 0 of b1 must be 1
            if not (b1 & 0x01):
                continue

            angle_q6 = ((b2 << 7) | (b1 >> 1))
            angle_deg = angle_q6 / 64.0

            distance_q2 = (b4 << 8) | b3
            distance_m = distance_q2 / 4000.0
            quality = sync_quality >> 2

            angles.append(angle_deg)
            ranges.append(distance_m)
            qualities.append(quality)

        return None

    def close(self):
        self.stop_motor()
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.ser = None


class SyntheticLidarGenerator:
    """
    High-fidelity 2D LiDAR raycaster against room boundaries and simulated physical obstacles.
    Simulates real sensor noise, beam divergence, and beam occlusions.
    """
    def __init__(self,
                 room_width: float = 5.0,
                 room_length: float = 4.0,
                 lidar_x: float = 0.0,
                 lidar_y: float = 0.0,
                 num_rays: int = 360,
                 noise_std: float = 0.012):
        self.half_w = room_width / 2.0
        self.half_l = room_length / 2.0
        self.lidar_x = lidar_x
        self.lidar_y = lidar_y
        self.num_rays = num_rays
        self.noise_std = noise_std
        
        # Interactive obstacles in physical room: name -> (x, y, radius)
        self.physical_obstacles: Dict[str, Tuple[float, float, float]] = {}
        self.rng = np.random.default_rng(123)

    def set_obstacle(self, name: str, x: float, y: float, radius: float = 0.20):
        self.physical_obstacles[name] = (float(x), float(y), float(radius))

    def remove_obstacle(self, name: str):
        self.physical_obstacles.pop(name, None)

    def clear_obstacles(self):
        self.physical_obstacles.clear()

    def generate_scan(self) -> LidarScan:
        angles_deg = np.linspace(0.0, 360.0, self.num_rays, endpoint=False)
        angles_rad = np.radians(angles_deg)
        ranges = np.zeros(self.num_rays, dtype=np.float64)

        cos_a = np.cos(angles_rad)
        sin_a = np.sin(angles_rad)

        for i in range(self.num_rays):
            ca = cos_a[i]
            sa = sin_a[i]
            d_hit = 20.0  # max range

            # 1. Raycast against 4 planar room walls
            # East wall: x = +half_w
            if ca > 1e-4:
                d = (self.half_w - self.lidar_x) / ca
                y_hit = self.lidar_y + d * sa
                if -self.half_l <= y_hit <= self.half_l:
                    d_hit = min(d_hit, d)
            # West wall: x = -half_w
            elif ca < -1e-4:
                d = (-self.half_w - self.lidar_x) / ca
                y_hit = self.lidar_y + d * sa
                if -self.half_l <= y_hit <= self.half_l:
                    d_hit = min(d_hit, d)

            # North wall: y = +half_l
            if sa > 1e-4:
                d = (self.half_l - self.lidar_y) / sa
                x_hit = self.lidar_x + d * ca
                if -self.half_w <= x_hit <= self.half_w:
                    d_hit = min(d_hit, d)
            # South wall: y = -half_l
            elif sa < -1e-4:
                d = (-self.half_l - self.lidar_y) / sa
                x_hit = self.lidar_x + d * ca
                if -self.half_w <= x_hit <= self.half_w:
                    d_hit = min(d_hit, d)

            # 2. Raycast against physical obstacle cylinders
            for ox, oy, r in self.physical_obstacles.values():
                # Line-circle intersection
                dx = self.lidar_x - ox
                dy = self.lidar_y - oy
                b = 2.0 * (ca * dx + sa * dy)
                c = dx * dx + dy * dy - r * r
                disc = b * b - 4.0 * c
                if disc >= 0:
                    sqrt_disc = math.sqrt(disc)
                    t1 = (-b - sqrt_disc) / 2.0
                    if t1 > 0:
                        d_hit = min(d_hit, t1)

            # Add Gaussian noise
            if self.noise_std > 0:
                d_hit += self.rng.normal(0, self.noise_std)
            ranges[i] = max(0.10, d_hit)

        qualities = np.full(self.num_rays, 47.0, dtype=np.float64)

        return LidarScan(
            timestamp=time.time(),
            angles_deg=angles_deg,
            ranges_m=ranges,
            qualities=qualities,
            is_hardware=False,
            source_info="Synthetic Physical Twin Emulator"
        )


class LidarInterface:
    """
    Unified LiDAR Interface supporting hardware RPLidar with automatic synthetic fallback.
    """
    def __init__(self,
                 port: str = "/dev/ttyUSB0",
                 baudrate: int = 115200,
                 fallback_synthetic: bool = True,
                 room_width: float = 5.0,
                 room_length: float = 4.0):
        self.port = port
        self.baudrate = baudrate
        self.fallback_synthetic = fallback_synthetic
        self.hw_driver = RPLidarHardwareDriver(port=port, baudrate=baudrate)
        self.synthetic_gen = SyntheticLidarGenerator(room_width=room_width, room_length=room_length)

        self.is_hardware_active = False
        self._is_running = False
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()
        self.latest_scan: Optional[LidarScan] = None

    def start(self) -> bool:
        """Start acquisition. Attempts hardware first; falls back to synthetic if unavailable."""
        self._is_running = True
        hw_ok = self.hw_driver.connect()
        if hw_ok and self.hw_driver.start_scan():
            self.is_hardware_active = True
            logger.info("Using physical RPLIDAR hardware interface.")
        else:
            self.is_hardware_active = False
            if self.fallback_synthetic:
                logger.info(f"LiDAR hardware port {self.port} unavailable. Running in High-Fidelity Synthetic Emulation mode.")
            else:
                logger.error("LiDAR hardware connection failed and synthetic fallback disabled.")
                self._is_running = False
                return False

        self._thread = threading.Thread(target=self._acquisition_loop, daemon=True)
        self._thread.start()
        return True

    def _acquisition_loop(self):
        while self._is_running:
            if self.is_hardware_active:
                scan = self.hw_driver.read_scan(max_read_timeout=0.3)
                if scan is not None:
                    with self._lock:
                        self.latest_scan = scan
                else:
                    time.sleep(0.02)
            else:
                # 10 Hz synthetic scan rate
                scan = self.synthetic_gen.generate_scan()
                with self._lock:
                    self.latest_scan = scan
                time.sleep(0.10)

    def get_latest_scan(self) -> Optional[LidarScan]:
        with self._lock:
            return self.latest_scan

    def set_synthetic_obstacle(self, name: str, x: float, y: float, radius: float = 0.20):
        """Allows injecting or moving physical obstacles for live demonstrations/tests."""
        self.synthetic_gen.set_obstacle(name, x, y, radius)

    def remove_synthetic_obstacle(self, name: str):
        """Allows removing physical obstacle."""
        self.synthetic_gen.remove_obstacle(name)

    def stop(self):
        self._is_running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        if self.is_hardware_active:
            self.hw_driver.close()
        logger.info("LiDAR interface stopped.")
