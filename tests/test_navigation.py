"""
Unit tests for differential-drive robot controller and obstacle avoidance.
"""

import unittest
import math
import numpy as np
import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from navigation.robot_controller import RobotController, NavigationState


class TestRobotController(unittest.TestCase):
    def setUp(self):
        self.controller = RobotController(
            wheel_radius=0.0975,
            wheel_base=0.381,
            max_linear_speed=0.4,
            max_angular_speed=1.0,
            goal_tolerance=0.15,
            safety_distance=0.60,
            stop_distance=0.35
        )

    def test_diff_drive_ik_pure_forward(self):
        v_left, v_right = self.controller.diff_drive_ik(v=0.195, w=0.0)
        # With r=0.0975, 0.195 m/s gives 2.0 rad/s on both wheels
        self.assertAlmostEqual(v_left, 2.0, places=3)
        self.assertAlmostEqual(v_right, 2.0, places=3)

    def test_goal_reached(self):
        self.controller.set_goal(1.0, 1.0)
        # Robot already at (1.0, 1.0)
        vl, vr, state = self.controller.compute_control(
            robot_x=1.0, robot_y=1.0, robot_yaw=0.0, obstacle_centroids_2d=[]
        )
        self.assertEqual(state, NavigationState.GOAL_REACHED)
        self.assertEqual(vl, 0.0)
        self.assertEqual(vr, 0.0)

    def test_critical_frontal_obstacle_stops_robot(self):
        self.controller.set_goal(2.0, 0.0)
        # Robot at (0.0, 0.0) facing +X (yaw = 0)
        # Obstacle at (0.30, 0.0) directly in front (inside stop_distance=0.35)
        obstacles = [np.array([0.30, 0.0])]
        vl, vr, state = self.controller.compute_control(
            robot_x=0.0, robot_y=0.0, robot_yaw=0.0, obstacle_centroids_2d=obstacles
        )
        self.assertEqual(state, NavigationState.BLOCKED)
        self.assertEqual(vl, 0.0)
        self.assertEqual(vr, 0.0)

    def test_obstacle_avoidance_deflection(self):
        self.controller.set_goal(2.0, 0.0)
        # Robot at (0.0, 0.0), facing +X
        # Obstacle at (0.50, 0.10) (inside safety_distance=0.60, but outside stop_distance)
        obstacles = [np.array([0.50, 0.10])]
        vl, vr, state = self.controller.compute_control(
            robot_x=0.0, robot_y=0.0, robot_yaw=0.0, obstacle_centroids_2d=obstacles
        )
        self.assertEqual(state, NavigationState.AVOIDING)
        # Since obstacle is slightly to the left (+Y), robot should deflect to the right (vr < vl)
        self.assertGreater(vl, vr)


if __name__ == "__main__":
    unittest.main()
