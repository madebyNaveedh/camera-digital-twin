"""
CoppeliaSim ZeroMQ Remote API interface for the Pioneer P3DX mobile robot.
Handles obstacle synchronization and perimeter navigation.
"""

import os
import time
import math
import logging
from typing import Tuple, Optional, List

try:
    from coppeliasim_zmqremoteapi_client import RemoteAPIClient
except ImportError:
    RemoteAPIClient = None

logger = logging.getLogger("CoppeliaInterface")

COPPELIASIM_ROOT = os.environ.get("COPPELIASIM_ROOT", "/home/mohammed/Downloads/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu22_04")
ROBOT_MODEL_PATH = os.path.join(COPPELIASIM_ROOT, "models/robots/mobile/pioneer p3dx.ttm")


class CoppeliaInterface:
    def __init__(self,
                 host: str = "localhost",
                 port: int = 23000,
                 enable_robot: bool = True,
                 safety_zone: Tuple[float, float, float, float] = (-0.5, 0.5, -0.5, 0.5),
                 patrol_speed: float = 0.16):
        
        self.host = host
        self.port = port
        self.enable_robot = enable_robot
        self.safety_zone = safety_zone
        self.patrol_speed = patrol_speed

        self.client: Optional[RemoteAPIClient] = None
        self.sim = None
        self.is_connected = False

        # Handles
        self.obstacle_handle = None
        self.robot_handle = None
        self.left_motor = None
        self.right_motor = None
        self.ultrasonic_sensors: List[int] = []

        # 4 Corner Waypoints for Continuous Square Loop Patrol (Counter-Clockwise)
        # Inside the 3.0m x 3.0m arena, waypoints at +/- 0.95m give generous 0.55m wall clearance
        self.square_waypoints = [
            ( 0.95,  0.95, "North-East Corner"),
            (-0.95,  0.95, "North-West Corner"),
            (-0.95, -0.95, "South-West Corner"),
            ( 0.95, -0.95, "South-East Corner")
        ]
        self.current_wp_idx = 0
        self.wp_reach_threshold = 0.28  # meters

        # State
        self.robot_nav_status = "IDLE"
        self.current_obs_pos = [0.0, 0.0]
        self.is_obstacle_active = False

    def connect(self) -> bool:
        """Establish connection to CoppeliaSim Remote API with socket probe."""
        if RemoteAPIClient is None:
            logger.error("coppeliasim_zmqremoteapi_client is not installed.")
            return False

        import socket
        try:
            with socket.create_connection((self.host, self.port), timeout=1.5):
                pass
        except (socket.timeout, ConnectionRefusedError, OSError):
            logger.warning(f"CoppeliaSim ZeroMQ server not running at {self.host}:{self.port}")
            self.is_connected = False
            return False

        try:
            self.client = RemoteAPIClient(host=self.host, port=self.port)
            self.sim = self.client.require("sim")
            ver = self.sim.getInt32Param(self.sim.intparam_program_version)
            self.is_connected = True
            logger.info(f"Connected to CoppeliaSim Remote API (Version: {ver})")
            return True
        except Exception as e:
            logger.warning(f"CoppeliaSim connection failed: {e}")
            self.is_connected = False
            return False

    def setup_scene(self) -> bool:
        """Verify and obtain handles for obstacle, robot motors, and sensors."""
        if not self.is_connected:
            return False

        sim = self.sim
        try:
            # 1. Clean up any stray test objects
            for stray in ["/LBRiiwa7R800", "/diningChair", "/CameraTwin_Table", "/CameraTwin_SafetyZone"]:
                try:
                    sh = sim.getObject(stray)
                    sim.removeObject(sh)
                except Exception:
                    pass

            # 2. Tracked Virtual Obstacle Cylinder
            try:
                self.obstacle_handle = sim.getObject("/Corridor_TrackedObstacle")
            except Exception:
                self.obstacle_handle = sim.createPrimitiveShape(sim.primitiveshape_cylinder, [0.30, 0.30, 0.50])
                sim.setObjectAlias(self.obstacle_handle, "Corridor_TrackedObstacle")
                sim.setObjectPosition(self.obstacle_handle, [0.0, 0.0, 0.25])
                sim.setObjectColor(self.obstacle_handle, 0, sim.colorcomponent_ambient_diffuse, [0.95, 0.45, 0.15])
                sim.setObjectInt32Param(self.obstacle_handle, sim.shapeintparam_static, 1)
                sim.setObjectInt32Param(self.obstacle_handle, sim.shapeintparam_respondable, 1)

            # 3. Pioneer P3DX Mobile Robot Setup
            if self.enable_robot:
                try:
                    self.robot_handle = sim.getObject("/PioneerP3DX")
                except Exception:
                    self.robot_handle = sim.loadModel(ROBOT_MODEL_PATH)
                    sim.setObjectAlias(self.robot_handle, "PioneerP3DX")

                # Set starting position firmly on floor at South-East corner (1.0, -1.0, 0.15), facing North
                sim.setObjectPosition(self.robot_handle, [1.0, -1.0, 0.15])
                sim.setObjectOrientation(self.robot_handle, [0.0, 0.0, math.pi / 2.0])

                self.left_motor = sim.getObject("/PioneerP3DX/leftMotor")
                self.right_motor = sim.getObject("/PioneerP3DX/rightMotor")

                # Disable Pioneer internal script so Python has exclusive control
                ps = sim.getScript(sim.scripttype_childscript, self.robot_handle)
                if ps != -1:
                    sim.setObjectInt32Param(ps, sim.scriptintparam_enabled, 0)

                # Cache Pioneer ultrasonic proximity sensors
                children = sim.getObjectsInTree(self.robot_handle)
                self.ultrasonic_sensors = [c for c in children if "ultrasonic" in sim.getObjectAlias(c).lower()]

            # Start simulation cleanly
            state = sim.getSimulationState()
            if state == sim.simulation_stopped:
                sim.startSimulation()
            elif state in (sim.simulation_paused, 8):
                sim.stopSimulation()
                time.sleep(0.2)
                sim.startSimulation()

            time.sleep(0.3)
            logger.info("Square arena simulation initialized successfully.")
            return True

        except Exception as e:
            logger.error(f"Scene setup failed: {e}")
            return False

    def update_virtual_obstacle(self, sim_x: float, sim_y: float, is_active: bool):
        """Update position and visual state of the obstacle inside the square arena."""
        self.is_obstacle_active = is_active
        if not self.is_connected or not self.obstacle_handle:
            return

        sim = self.sim
        try:
            # Clamp obstacle strictly inside the arena walls
            clamped_x = float(max(-1.4, min(1.4, sim_x)))
            clamped_y = float(max(-1.4, min(1.4, sim_y)))

            self.current_obs_pos = [clamped_x, clamped_y]
            sim.setObjectPosition(self.obstacle_handle, [clamped_x, clamped_y, 0.25])

            if is_active:
                sim.setObjectColor(self.obstacle_handle, 0, sim.colorcomponent_ambient_diffuse, [0.95, 0.45, 0.15])
            else:
                sim.setObjectColor(self.obstacle_handle, 0, sim.colorcomponent_ambient_diffuse, [0.55, 0.50, 0.45])
        except Exception:
            pass

    def check_ultrasonic_clearance(self) -> Tuple[bool, float]:
        """Poll strictly forward-facing proximity sensors (indices 2, 3, 4, 5) for emergency wall clearance."""
        if not self.sim or not self.ultrasonic_sensors:
            return False, 999.0

        min_dist = 999.0
        # Sensors 2, 3, 4, 5 are the frontal heading sensors on Pioneer P3DX
        for idx in (2, 3, 4, 5):
            if idx < len(self.ultrasonic_sensors):
                s_handle = self.ultrasonic_sensors[idx]
                try:
                    res, dist, _, _, _ = self.sim.readProximitySensor(s_handle)
                    if res == 1 and 0.02 < dist < min_dist:
                        min_dist = dist
                except Exception:
                    pass

        return (min_dist < 0.18), min_dist

    def update_robot_interaction(self, in_safety_zone: bool = False, is_active: Optional[bool] = None) -> str:
        """
        Controls Pioneer P3DX mobile robot:
        - Navigates continuous square loop around the arena.
        - Turns cleanly in place at corners without swinging into walls.
        - Halts smoothly when an active physical obstacle blocks its perimeter path.
        - Watchdog: automatically rescues robot if it ever falls or drifts.
        """
        if not self.is_connected or not self.left_motor or not self.right_motor or not self.robot_handle:
            self.robot_nav_status = "DISABLED"
            return self.robot_nav_status

        if is_active is not None:
            self.is_obstacle_active = is_active

        sim = self.sim
        try:
            pos = sim.getObjectPosition(self.robot_handle, sim.handle_world)
            ori = sim.getObjectOrientation(self.robot_handle, sim.handle_world)
            rx, ry, rz = pos[0], pos[1], pos[2]
            yaw = ori[2]

            # Anti-Fall Watchdog: if robot ever leaves floor (Z < 0.05 or out of bounds), rescue it!
            if rz < 0.05 or abs(rx) > 1.45 or abs(ry) > 1.45:
                sim.setObjectPosition(self.robot_handle, [1.0, -1.0, 0.15])
                sim.setObjectOrientation(self.robot_handle, [0.0, 0.0, math.pi / 2.0])
                rx, ry, rz = 1.0, -1.0, 0.15
                yaw = math.pi / 2.0
                self.current_wp_idx = 0

            R = 0.0975  # Wheel radius (m)
            L = 0.381   # Wheelbase width (m)

            # 1. Ultrasonic Wall Clearance
            wall_blocked, wall_dist = self.check_ultrasonic_clearance()

            # 2. Dynamic Physical Obstacle Distance Check
            dx_obs = self.current_obs_pos[0] - rx
            dy_obs = self.current_obs_pos[1] - ry
            dist_to_obs = math.hypot(dx_obs, dy_obs)
            angle_to_obs = math.atan2(dy_obs, dx_obs)
            obs_rel_angle = math.atan2(math.sin(angle_to_obs - yaw), math.cos(angle_to_obs - yaw))

            # Obstacle in path ahead: ONLY if actively tracked, close (< 0.65m), and in frontal +/- 50 deg heading cone
            obstacle_in_path = (self.is_obstacle_active and dist_to_obs < 0.65 and abs(obs_rel_angle) < math.radians(50))

            if obstacle_in_path or wall_blocked:
                sim.setJointTargetVelocity(self.left_motor, 0.0)
                sim.setJointTargetVelocity(self.right_motor, 0.0)
                if wall_blocked:
                    self.robot_nav_status = f"WALL CLEARANCE HALT ({wall_dist:.2f}m)"
                else:
                    self.robot_nav_status = f"OBSTACLE BLOCKING PATH ({dist_to_obs:.2f}m)"
                return self.robot_nav_status

            # 3. Square Loop Navigation
            tx, ty, corner_name = self.square_waypoints[self.current_wp_idx]
            dist_to_wp = math.hypot(tx - rx, ty - ry)

            # Waypoint reached -> advance to next corner of the square!
            if dist_to_wp < self.wp_reach_threshold:
                self.current_wp_idx = (self.current_wp_idx + 1) % len(self.square_waypoints)
                tx, ty, corner_name = self.square_waypoints[self.current_wp_idx]

            target_heading = math.atan2(ty - ry, tx - rx)
            yaw_err = math.atan2(math.sin(target_heading - yaw), math.cos(target_heading - yaw))

            # Zero-radius in-place rotation at corners:
            # Spins cleanly on its axis until facing the next side of the square!
            if abs(yaw_err) > math.radians(16):
                w = math.copysign(0.60, yaw_err)
                v = 0.0
                self.robot_nav_status = f"TURNING CORNER -> {corner_name}"
            else:
                # Drive straight along the current side of the square
                w = 1.0 * yaw_err
                v = self.patrol_speed
                self.robot_nav_status = f"CIRCLING SQUARE -> {corner_name} ({v:.2f} m/s)"

            # Differential drive kinematics
            vl = (v - w * L / 2.0) / R
            vr = (v + w * L / 2.0) / R

            sim.setJointTargetVelocity(self.left_motor, float(vl))
            sim.setJointTargetVelocity(self.right_motor, float(vr))

        except Exception as e:
            self.robot_nav_status = "NAV ERROR"

        return self.robot_nav_status

    def stop(self):
        """Clean shutdown of CoppeliaSim simulation."""
        if self.is_connected and self.sim:
            try:
                if self.left_motor and self.right_motor:
                    self.sim.setJointTargetVelocity(self.left_motor, 0.0)
                    self.sim.setJointTargetVelocity(self.right_motor, 0.0)
                self.sim.stopSimulation()
            except Exception:
                pass
        self.is_connected = False
