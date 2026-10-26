"""
Core Digital Twin Orchestration Engine.
Coordinates sensor acquisition, perception, spatial transformations,
simulation synchronisation, and autonomous robot navigation.
"""

import time
import math
import logging
import threading
from typing import Optional, Dict, Any, List, Tuple
import numpy as np

from calibration.coordinate_transforms import CoordinateTransformer
from sensors.camera_interface import CameraInterface
from sensors.lidar_interface import LidarInterface, LidarScan
from perception.lidar_processing import LidarProcessor
from perception.obstacle_detection import ObstacleDetector, ObstacleCandidate
from digital_twin.environment_state import EnvironmentState, TrackedObstacle
from digital_twin.synchronizer import DigitalTwinSynchronizer
from simulation.coppelia_client import CoppeliaClient
from simulation.scene_manager import SceneManager
from navigation.robot_controller import RobotController, NavigationState

logger = logging.getLogger("DigitalTwinEngine")


class DigitalTwinEngine:
    def __init__(self,
                 room_width: float = 5.0,
                 room_length: float = 4.0,
                 lidar_port: str = "/dev/ttyUSB0",
                 lidar_baudrate: int = 115200,
                 camera_index: int = 0,
                 robot_start: Tuple[float, float] = (-1.5, -1.0),
                 robot_goal: Tuple[float, float] = (1.5, 1.0)):
        
        self.room_width = room_width
        self.room_length = room_length
        self.robot_start = robot_start
        self.robot_goal = robot_goal

        # 1. Calibration and Coordinate Transforms
        # Sensor mounting at room center, 0.25m above floor
        self.transformer = CoordinateTransformer(
            lidar_x=0.0,
            lidar_y=0.0,
            lidar_z=0.25,
            lidar_yaw_deg=0.0
        )

        # 2. Physical Sensors
        self.camera = CameraInterface(device_index=camera_index, width=640, height=480, fps=30)
        self.lidar = LidarInterface(
            port=lidar_port,
            baudrate=lidar_baudrate,
            fallback_synthetic=True,
            room_width=room_width,
            room_length=room_length
        )

        # 3. Perception
        # Boundaries to clip out points that are near or outside walls (e.g. within 0.15m of wall)
        safe_margin = 0.20
        room_bounds = (
            -room_width / 2.0 + safe_margin,
            room_width / 2.0 - safe_margin,
            -room_length / 2.0 + safe_margin,
            room_length / 2.0 - safe_margin
        )
        self.lidar_processor = LidarProcessor(range_min=0.15, range_max=10.0, room_bounds=room_bounds)
        self.obstacle_detector = ObstacleDetector(
            distance_threshold=0.25,
            min_cluster_size=4,
            max_cluster_size=150,
            min_obstacle_radius=0.06,
            max_obstacle_radius=0.70
        )

        # 4. Digital Twin State & Synchronizer
        self.state = EnvironmentState(room_width=room_width, room_length=room_length)
        self.coppelia_client = CoppeliaClient()
        self.scene_manager: Optional[SceneManager] = None
        self.synchronizer: Optional[DigitalTwinSynchronizer] = None

        # 5. Robot Navigation Controller
        self.controller = RobotController(
            wheel_radius=0.0975,
            wheel_base=0.381,
            max_linear_speed=0.32,
            max_angular_speed=1.0,
            goal_tolerance=0.20,
            safety_distance=0.65,
            stop_distance=0.38
        )
        self.controller.set_goal(robot_goal[0], robot_goal[1])

        # State tracking
        self.is_running = False
        self._loop_thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()
        
        # Telemetry & Performance
        self.last_robot_pose = (robot_start[0], robot_start[1], 0.0)
        self.last_wheel_speeds = (0.0, 0.0)
        self.last_nav_state = NavigationState.IDLE
        self.last_candidates: List[ObstacleCandidate] = []
        self.pipeline_latency_ms: float = 0.0
        self.loop_hz: float = 0.0

    def start(self, connect_simulation: bool = True) -> bool:
        """Initialize all subsystems and launch real-time control loop."""
        logger.info("Starting Digital Twin Engine...")
        
        # Start sensors
        self.camera.start()
        self.lidar.start()

        # Connect CoppeliaSim if requested
        if connect_simulation:
            if self.coppelia_client.connect():
                self.scene_manager = SceneManager(
                    sim=self.coppelia_client.sim,
                    room_width=self.room_width,
                    room_length=self.room_length
                )
                self.scene_manager.setup_scene(
                    robot_start_pos=self.robot_start,
                    goal_pos=self.robot_goal
                )
                self.coppelia_client.start_simulation()
            else:
                logger.warning("CoppeliaSim could not be connected. Running in simulation-detached mode.")

        # Setup synchronizer with scene manager
        self.synchronizer = DigitalTwinSynchronizer(
            state=self.state,
            association_distance_threshold=0.45,
            confirmation_frames=2,
            removal_timeout_seconds=2.0,
            smoothing_alpha=0.65,
            scene_manager=self.scene_manager
        )

        self.is_running = True
        self._loop_thread = threading.Thread(target=self._run_loop, daemon=True)
        self._loop_thread.start()
        logger.info("Digital Twin Engine running.")
        return True

    def _run_loop(self):
        rate_hz = 15.0
        dt = 1.0 / rate_hz
        t_fps_start = time.time()
        fps_count = 0

        while self.is_running:
            t0 = time.perf_counter()
            self._step_pipeline()
            t1 = time.perf_counter()

            elapsed = t1 - t0
            self.pipeline_latency_ms = elapsed * 1000.0

            fps_count += 1
            if (time.time() - t_fps_start) >= 2.0:
                self.loop_hz = fps_count / (time.time() - t_fps_start)
                fps_count = 0
                t_fps_start = time.time()

            sleep_time = max(0.005, dt - elapsed)
            time.sleep(sleep_time)

    def _step_pipeline(self):
        """Single iteration of the end-to-end perception, sync, and navigation pipeline."""
        now = time.time()

        # 1. Acquire LiDAR scan
        scan = self.lidar.get_latest_scan()
        if scan is None:
            return

        # 2. Filter raw scan
        valid_angles, valid_ranges, _ = self.lidar_processor.filter_raw_scan(
            scan.angles_deg, scan.ranges_m, scan.qualities
        )

        # 3. Convert to 2D Cartesian in LiDAR frame & Room frame
        pts_lidar = self.transformer.polar_to_lidar_cartesian(valid_angles, valid_ranges)
        pts_room = self.transformer.lidar_to_room_2d(pts_lidar)

        # 4. Filter room boundary perimeter points (isolate obstacles inside room)
        obstacle_pts = self.lidar_processor.filter_room_boundaries(pts_room)

        # 5. Perception: Detect obstacle candidates
        candidates = self.obstacle_detector.detect_obstacles(obstacle_pts)

        # 6. Digital Twin Synchronisation
        if self.synchronizer:
            confirmed_obstacles = self.synchronizer.process_observations(candidates, current_time=now)
        else:
            confirmed_obstacles = []

        # 7. Navigation: Extract obstacle centroids for avoidance
        obstacle_centroids = [obs.centroid[:2] for obs in confirmed_obstacles]

        # 8. Robot Control (if scene manager connected)
        if self.scene_manager and self.coppelia_client.is_connected:
            rx, ry, ryaw = self.scene_manager.get_robot_pose()
            vl, vr, nav_state = self.controller.compute_control(rx, ry, ryaw, obstacle_centroids)
            self.scene_manager.set_robot_wheel_speeds(vl, vr)

            with self._lock:
                self.last_robot_pose = (rx, ry, ryaw)
                self.last_wheel_speeds = (vl, vr)
                self.last_nav_state = nav_state
                self.last_candidates = candidates
        else:
            with self._lock:
                self.last_candidates = candidates

    def get_telemetry(self) -> Dict[str, Any]:
        """Provides complete real-time telemetry snapshot for dashboard and logging."""
        with self._lock:
            frame, frame_ts, cam_valid = self.camera.get_latest_frame()
            scan = self.lidar.get_latest_scan()
            confirmed_obs = self.state.get_confirmed_obstacles()

            return {
                "timestamp": time.time(),
                "camera": {
                    "is_active": cam_valid,
                    "last_timestamp": frame_ts,
                    "has_frame": frame is not None,
                    "actual_fps": getattr(self.camera, "fps_actual", 0.0)
                },
                "lidar": {
                    "is_hardware": scan.is_hardware if scan else False,
                    "source": scan.source_info if scan else "None",
                    "num_points": len(scan.ranges_m) if scan else 0,
                    "timestamp": scan.timestamp if scan else 0.0
                },
                "simulation": {
                    "is_connected": self.coppelia_client.is_connected,
                    "sim_time": self.coppelia_client.get_simulation_time()
                },
                "robot": {
                    "pose": self.last_robot_pose,
                    "wheel_speeds": self.last_wheel_speeds,
                    "nav_state": self.last_nav_state.value,
                    "goal": self.robot_goal
                },
                "digital_twin": {
                    "confirmed_obstacle_count": len(confirmed_obs),
                    "obstacles": [
                        {
                            "id": o.id,
                            "centroid": o.centroid.tolist(),
                            "radius": o.radius,
                            "confidence": o.confidence,
                            "age_s": o.age(time.time()),
                            "sim_handle": o.sim_handle
                        }
                        for o in confirmed_obs
                    ],
                    "sync_duration_ms": self.synchronizer.sync_duration_ms if self.synchronizer else 0.0,
                    "pipeline_latency_ms": self.pipeline_latency_ms,
                    "loop_rate_hz": self.loop_hz
                }
            }

    def stop(self):
        """Clean shutdown of all interfaces."""
        logger.info("Stopping Digital Twin Engine...")
        self.is_running = False
        if self._loop_thread and self._loop_thread.is_alive():
            self._loop_thread.join(timeout=1.5)

        if self.scene_manager:
            try:
                self.scene_manager.set_robot_wheel_speeds(0.0, 0.0)
                self.scene_manager.clear_all_obstacles()
            except Exception:
                pass

        if self.coppelia_client.is_connected:
            self.coppelia_client.stop_simulation()

        self.camera.stop()
        self.lidar.stop()
        logger.info("Digital Twin Engine stopped.")
