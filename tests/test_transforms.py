"""
Unit tests for coordinate transformation pipeline.
"""

import unittest
import math
import numpy as np
import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from calibration.coordinate_transforms import CoordinateTransformer


class TestCoordinateTransforms(unittest.TestCase):
    def setUp(self):
        # LiDAR at (0.5, -0.5, 0.2), rotated +90 degrees counter-clockwise
        self.transformer = CoordinateTransformer(
            lidar_x=0.5,
            lidar_y=-0.5,
            lidar_z=0.2,
            lidar_yaw_deg=90.0,
            sim_offset_x=1.0,
            sim_offset_y=2.0,
            sim_offset_z=0.0,
            sim_yaw_deg=0.0
        )

    def test_polar_to_cartesian_cardinals(self):
        angles = np.array([0.0, 90.0, 180.0, 270.0])
        ranges = np.array([2.0, 3.0, 1.0, 4.0])
        pts = self.transformer.polar_to_lidar_cartesian(angles, ranges)
        
        # 0 deg: (2, 0)
        self.assertAlmostEqual(pts[0, 0], 2.0, places=5)
        self.assertAlmostEqual(pts[0, 1], 0.0, places=5)
        # 90 deg: (0, 3)
        self.assertAlmostEqual(pts[1, 0], 0.0, places=5)
        self.assertAlmostEqual(pts[1, 1], 3.0, places=5)
        # 180 deg: (-1, 0)
        self.assertAlmostEqual(pts[2, 0], -1.0, places=5)
        self.assertAlmostEqual(pts[2, 1], 0.0, places=5)
        # 270 deg: (0, -4)
        self.assertAlmostEqual(pts[3, 0], 0.0, places=5)
        self.assertAlmostEqual(pts[3, 1], -4.0, places=5)

    def test_lidar_to_room_rotation_and_translation(self):
        # Point in sensor frame: 1.0 m straight ahead (+X_sensor)
        # LiDAR is rotated 90 deg CCW, so +X_sensor maps to +Y_room
        # LiDAR is located at (0.5, -0.5)
        # Expected room coordinate: (0.5 + 0, -0.5 + 1.0) = (0.5, 0.5)
        p_sensor = np.array([[1.0, 0.0]])
        p_room = self.transformer.lidar_to_room_2d(p_sensor)
        
        self.assertAlmostEqual(p_room[0, 0], 0.5, places=5)
        self.assertAlmostEqual(p_room[0, 1], 0.5, places=5)

    def test_room_to_sim_transform(self):
        p_room = np.array([[0.5, 0.5, 0.2]])
        p_sim = self.transformer.room_to_sim(p_room)
        # sim offset is (1.0, 2.0, 0.0)
        self.assertAlmostEqual(p_sim[0, 0], 1.5, places=5)
        self.assertAlmostEqual(p_sim[0, 1], 2.5, places=5)
        self.assertAlmostEqual(p_sim[0, 2], 0.2, places=5)

    def test_room_to_robot_ego_frame(self):
        # Robot at (1.0, 1.0), heading = 0 rad (+X)
        # Point in room at (3.0, 1.0) is 2.0 m directly in front of robot
        p_room = np.array([[3.0, 1.0]])
        p_robot = self.transformer.room_to_robot(p_room, robot_x=1.0, robot_y=1.0, robot_yaw_rad=0.0)
        self.assertAlmostEqual(p_robot[0, 0], 2.0, places=5)
        self.assertAlmostEqual(p_robot[0, 1], 0.0, places=5)

        # Rotate robot 90 deg CCW (facing +Y)
        # Point at (3.0, 1.0) is now to the robot's right (-Y_robot)
        p_robot_rot = self.transformer.room_to_robot(p_room, robot_x=1.0, robot_y=1.0, robot_yaw_rad=math.pi / 2)
        self.assertAlmostEqual(p_robot_rot[0, 0], 0.0, places=5)
        self.assertAlmostEqual(p_robot_rot[0, 1], -2.0, places=5)


if __name__ == "__main__":
    unittest.main()
