"""
Coordinate transformation utilities for the Real-Time Digital Twin.

Transforms between:
- Polar sensor measurements (r, theta) -> Cartesian LiDAR frame (x_s, y_s)
- LiDAR frame -> Physical Room frame: p_room = R(yaw) * p_sensor + t_sensor
- Physical Room frame -> CoppeliaSim world frame: p_sim = R_sim * p_room + t_sim
- Physical Room frame -> Robot frame: p_robot = R_robot^T * (p_room - t_robot)

Frame Conventions:
- LiDAR Frame:
    Origin: Center of the LiDAR rotating mirror
    +X: Forward (0 degrees angle)
    +Y: Left (90 degrees angle)
    +Z: Upward along rotation axis
- Physical Room Frame:
    Origin: Center of physical room floor (configurable)
    +X: Lengthwise axis (East / Forward)
    +Y: Crosswise axis (North / Left)
    +Z: Upward
- CoppeliaSim World Frame:
    Standard right-handed XYZ system, aligned with Physical Room Frame.
- Robot Frame (Pioneer P3DX):
    Origin: Midpoint between drive wheels at ground contact
    +X: Forward driving direction
    +Y: Left wheel direction
    +Z: Upward
"""

import math
from typing import Tuple, List, Optional, Union
import numpy as np


class CoordinateTransformer:
    def __init__(self,
                 lidar_x: float = 0.0,
                 lidar_y: float = 0.0,
                 lidar_z: float = 0.25,
                 lidar_yaw_deg: float = 0.0,
                 sim_offset_x: float = 0.0,
                 sim_offset_y: float = 0.0,
                 sim_offset_z: float = 0.0,
                 sim_yaw_deg: float = 0.0):
        """
        Initialize the coordinate transformer with mounting and simulation offsets.
        """
        self.lidar_t = np.array([lidar_x, lidar_y, lidar_z], dtype=np.float64)
        self.lidar_yaw = math.radians(lidar_yaw_deg)
        
        # 2D rotation matrix for LiDAR mounting yaw
        c = math.cos(self.lidar_yaw)
        s = math.sin(self.lidar_yaw)
        self.R_lidar_to_room_2d = np.array([[c, -s],
                                            [s,  c]], dtype=np.float64)
        
        # Simulation world offset relative to Room Frame
        self.sim_t = np.array([sim_offset_x, sim_offset_y, sim_offset_z], dtype=np.float64)
        self.sim_yaw = math.radians(sim_yaw_deg)
        cs = math.cos(self.sim_yaw)
        ss = math.sin(self.sim_yaw)
        self.R_room_to_sim_2d = np.array([[cs, -ss],
                                          [ss,  cs]], dtype=np.float64)

    def polar_to_lidar_cartesian(self,
                                 angles_deg: np.ndarray,
                                 ranges_m: np.ndarray) -> np.ndarray:
        """
        Convert polar LiDAR readings (angle in degrees, range in meters)
        to 2D Cartesian coordinates in the LiDAR frame.
        
        Returns:
            np.ndarray: Shape (N, 2) where columns are [x_lidar, y_lidar]
        """
        angles_rad = np.radians(angles_deg)
        x = ranges_m * np.cos(angles_rad)
        y = ranges_m * np.sin(angles_rad)
        return np.column_stack((x, y))

    def lidar_to_room_2d(self, points_lidar_2d: np.ndarray) -> np.ndarray:
        """
        Transform 2D points from LiDAR frame to Physical Room frame.
        p_room = R(yaw) * p_lidar + t_lidar[:2]
        
        Args:
            points_lidar_2d: Shape (N, 2)
        Returns:
            np.ndarray: Shape (N, 2) in Room coordinates
        """
        if points_lidar_2d.size == 0:
            return np.empty((0, 2), dtype=np.float64)
            
        pts = np.asarray(points_lidar_2d, dtype=np.float64)
        # R @ pts.T + t -> (pts @ R.T) + t
        transformed = pts @ self.R_lidar_to_room_2d.T + self.lidar_t[:2]
        return transformed

    def lidar_to_room_3d(self, points_lidar_2d: np.ndarray) -> np.ndarray:
        """
        Transform 2D LiDAR points into 3D Physical Room coordinates (z = lidar_z).
        Returns:
            np.ndarray: Shape (N, 3) in Room coordinates
        """
        if points_lidar_2d.size == 0:
            return np.empty((0, 3), dtype=np.float64)
        pts_2d = self.lidar_to_room_2d(points_lidar_2d)
        z = np.full((len(pts_2d), 1), self.lidar_t[2], dtype=np.float64)
        return np.column_stack((pts_2d, z))

    def room_to_sim(self, points_room: np.ndarray) -> np.ndarray:
        """
        Transform coordinates from Physical Room frame to CoppeliaSim world frame.
        """
        if points_room.size == 0:
            return points_room.copy()
            
        pts = np.asarray(points_room, dtype=np.float64)
        if pts.shape[1] == 2:
            return pts @ self.R_room_to_sim_2d.T + self.sim_t[:2]
        elif pts.shape[1] == 3:
            pts_2d = pts[:, :2] @ self.R_room_to_sim_2d.T + self.sim_t[:2]
            z = pts[:, 2:3] + self.sim_t[2]
            return np.column_stack((pts_2d, z))
        else:
            raise ValueError(f"Expected Nx2 or Nx3 array, got shape {pts.shape}")

    def room_to_robot(self,
                      points_room_2d: np.ndarray,
                      robot_x: float,
                      robot_y: float,
                      robot_yaw_rad: float) -> np.ndarray:
        """
        Transform 2D points from Room Frame to Robot Ego Frame.
        p_robot = R(-yaw) * (p_room - t_robot)
        """
        if points_room_2d.size == 0:
            return np.empty((0, 2), dtype=np.float64)
            
        pts = np.asarray(points_room_2d, dtype=np.float64)
        rel = pts - np.array([robot_x, robot_y])
        c = math.cos(robot_yaw_rad)
        s = math.sin(robot_yaw_rad)
        R_inv = np.array([[c,  s],
                          [-s, c]], dtype=np.float64)
        return rel @ R_inv.T
