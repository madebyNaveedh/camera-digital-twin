"""
CoppeliaSim Scene Manager.
Constructs the virtual room, instantiates the mobile robot,
manages obstacle representations, and provides robot control interfaces.
"""

import math
import logging
from typing import Tuple, List, Optional, Dict
import numpy as np

logger = logging.getLogger("SceneManager")

DEFAULT_ROBOT_MODEL_PATH = "/home/mohammed/Downloads/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu22_04/models/robots/mobile/pioneer p3dx.ttm"


class SceneManager:
    def __init__(self,
                 sim,
                 room_width: float = 5.0,
                 room_length: float = 4.0,
                 wall_height: float = 0.8,
                 robot_model_path: str = DEFAULT_ROBOT_MODEL_PATH):
        self.sim = sim
        self.room_width = room_width
        self.room_length = room_length
        self.wall_height = wall_height
        self.robot_model_path = robot_model_path

        # Handles
        self.floor_handle = None
        self.wall_handles: List[int] = []
        self.robot_handle = None
        self.left_motor_handle = None
        self.right_motor_handle = None
        self.goal_marker_handle = None

        # Track created obstacles: handle -> name
        self.active_obstacle_handles: Dict[int, str] = {}

    def setup_scene(self,
                    robot_start_pos: Tuple[float, float] = (-1.5, -1.0),
                    goal_pos: Tuple[float, float] = (1.5, 1.0)) -> bool:
        """
        Build or verify the 3D room, load mobile robot, and place goal marker.
        """
        try:
            logger.info("Setting up CoppeliaSim digital twin scene...")
            self._create_room_geometry()
            self._load_or_find_robot(robot_start_pos)
            self._create_or_update_goal_marker(goal_pos)
            logger.info("Scene setup completed successfully.")
            return True
        except Exception as e:
            logger.error(f"Failed to setup scene: {e}", exc_info=True)
            return False

    def _create_room_geometry(self):
        """Construct floor and perimeter walls."""
        sim = self.sim

        # 1. Floor plane
        try:
            floor = sim.getObject("/DigitalTwin_Floor")
            self.floor_handle = floor
        except Exception:
            # Create cuboid floor: width (X) x length (Y) x thickness 0.05 (Z)
            self.floor_handle = sim.createPrimitiveShape(
                sim.primitiveshape_cuboid,
                [self.room_width, self.room_length, 0.05]
            )
            sim.setObjectPosition(self.floor_handle, [0.0, 0.0, -0.025])
            sim.setObjectAlias(self.floor_handle, "DigitalTwin_Floor")
            sim.setObjectColor(self.floor_handle, 0, sim.colorcomponent_ambient_diffuse, [0.85, 0.85, 0.88])
            # Set static & respondable
            sim.setObjectInt32Param(self.floor_handle, sim.shapeintparam_static, 1)
            sim.setObjectInt32Param(self.floor_handle, sim.shapeintparam_respondable, 1)

        # 2. Four Boundary Walls: North, South, East, West
        wall_thick = 0.10
        half_w = self.room_width / 2.0
        half_l = self.room_length / 2.0
        half_h = self.wall_height / 2.0

        wall_specs = [
            ("NorthWall", [self.room_width, wall_thick, self.wall_height], [0.0, half_l + wall_thick / 2.0, half_h]),
            ("SouthWall", [self.room_width, wall_thick, self.wall_height], [0.0, -half_l - wall_thick / 2.0, half_h]),
            ("EastWall", [wall_thick, self.room_length, self.wall_height], [half_w + wall_thick / 2.0, 0.0, half_h]),
            ("WestWall", [wall_thick, self.room_length, self.wall_height], [-half_w - wall_thick / 2.0, 0.0, half_h]),
        ]

        for alias, dims, pos in wall_specs:
            try:
                wh = sim.getObject(f"/DigitalTwin_{alias}")
                self.wall_handles.append(wh)
            except Exception:
                wh = sim.createPrimitiveShape(sim.primitiveshape_cuboid, dims)
                sim.setObjectPosition(wh, pos)
                sim.setObjectAlias(wh, f"DigitalTwin_{alias}")
                sim.setObjectColor(wh, 0, sim.colorcomponent_ambient_diffuse, [0.35, 0.45, 0.55])
                sim.setObjectInt32Param(wh, sim.shapeintparam_static, 1)
                sim.setObjectInt32Param(wh, sim.shapeintparam_respondable, 1)
                self.wall_handles.append(wh)

    def _load_or_find_robot(self, start_pos: Tuple[float, float]):
        """Load or locate Pioneer P3DX mobile robot."""
        sim = self.sim
        try:
            self.robot_handle = sim.getObject("/PioneerP3DX")
        except Exception:
            logger.info(f"Loading Pioneer P3DX model from {self.robot_model_path}...")
            self.robot_handle = sim.loadModel(self.robot_model_path)
            sim.setObjectAlias(self.robot_handle, "PioneerP3DX")

        # Set robot starting position
        sim.setObjectPosition(self.robot_handle, [start_pos[0], start_pos[1], 0.15])
        # Face +X / slightly toward goal
        sim.setObjectOrientation(self.robot_handle, [0.0, 0.0, math.atan2(start_pos[1], start_pos[0]) + math.pi])

        # Get motor handles
        self.left_motor_handle = sim.getObject("/PioneerP3DX/leftMotor")
        self.right_motor_handle = sim.getObject("/PioneerP3DX/rightMotor")
        logger.info(f"Robot configured. Base: {self.robot_handle}, Left: {self.left_motor_handle}, Right: {self.right_motor_handle}")

    def _create_or_update_goal_marker(self, goal_pos: Tuple[float, float]):
        """Create or update visual goal disc marker in the scene."""
        sim = self.sim
        try:
            self.goal_marker_handle = sim.getObject("/DigitalTwin_GoalMarker")
        except Exception:
            self.goal_marker_handle = sim.createPrimitiveShape(
                sim.primitiveshape_cylinder,
                [0.40, 0.40, 0.02]
            )
            sim.setObjectAlias(self.goal_marker_handle, "DigitalTwin_GoalMarker")
            sim.setObjectColor(self.goal_marker_handle, 0, sim.colorcomponent_ambient_diffuse, [0.15, 0.85, 0.35])
            # Set static non-respondable so robot drives smoothly over it
            sim.setObjectInt32Param(self.goal_marker_handle, sim.shapeintparam_static, 1)
            sim.setObjectInt32Param(self.goal_marker_handle, sim.shapeintparam_respondable, 0)

        sim.setObjectPosition(self.goal_marker_handle, [goal_pos[0], goal_pos[1], 0.01])

    def get_robot_pose(self) -> Tuple[float, float, float]:
        """
        Get current simulated robot pose: (x, y, yaw in radians).
        """
        if not self.robot_handle:
            return 0.0, 0.0, 0.0
        pos = self.sim.getObjectPosition(self.robot_handle, self.sim.handle_world)
        ori = self.sim.getObjectOrientation(self.robot_handle, self.sim.handle_world)
        return float(pos[0]), float(pos[1]), float(ori[2])

    def set_robot_wheel_speeds(self, v_left: float, v_right: float):
        """
        Command angular velocities (rad/s) to the differential drive motors.
        """
        if self.left_motor_handle and self.right_motor_handle:
            self.sim.setJointTargetVelocity(self.left_motor_handle, float(v_left))
            self.sim.setJointTargetVelocity(self.right_motor_handle, float(v_right))

    def update_or_create_obstacle(self,
                                  sim_handle: Optional[int],
                                  name: str,
                                  position: np.ndarray,
                                  radius: float = 0.20,
                                  height: float = 0.50) -> int:
        """
        Create or update an obstacle representation in CoppeliaSim.
        If sim_handle is valid and exists, updates position.
        Otherwise creates a new cylinder shape.
        """
        sim = self.sim
        pos_z = height / 2.0
        target_pos = [float(position[0]), float(position[1]), float(pos_z)]

        # Check if handle is still valid
        if sim_handle is not None:
            try:
                sim.setObjectPosition(sim_handle, target_pos)
                return sim_handle
            except Exception:
                # Handle expired or invalid, recreate below
                pass

        # Create new visual and respondable obstacle cylinder
        diameter = max(0.12, radius * 2.0)
        h = max(0.20, height)
        handle = sim.createPrimitiveShape(sim.primitiveshape_cylinder, [diameter, diameter, h])
        sim.setObjectAlias(handle, f"Obstacle_{name}")
        sim.setObjectPosition(handle, target_pos)
        # Vibrant orange-red color for physical synchronised obstacles
        sim.setObjectColor(handle, 0, sim.colorcomponent_ambient_diffuse, [0.95, 0.30, 0.15])
        # Make static and respondable so physics and raycasts detect it
        sim.setObjectInt32Param(handle, sim.shapeintparam_static, 1)
        sim.setObjectInt32Param(handle, sim.shapeintparam_respondable, 1)

        self.active_obstacle_handles[handle] = name
        return handle

    def remove_obstacle(self, sim_handle: int):
        """Remove obstacle object from CoppeliaSim."""
        try:
            self.sim.removeObject(sim_handle)
            self.active_obstacle_handles.pop(sim_handle, None)
        except Exception:
            pass

    def clear_all_obstacles(self):
        """Remove all created obstacle shapes."""
        for handle in list(self.active_obstacle_handles.keys()):
            self.remove_obstacle(handle)
        self.active_obstacle_handles.clear()
