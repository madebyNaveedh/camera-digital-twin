"""
Autonomous differential-drive robot controller with reactive obstacle avoidance.
Uses Artificial Potential Field (APF) with safety zones.
"""

import math
from enum import Enum
from typing import Tuple, List, Optional
import numpy as np


class NavigationState(Enum):
    IDLE = "IDLE"
    NAVIGATING = "NAVIGATING"
    AVOIDING = "AVOIDING"
    BLOCKED = "BLOCKED"
    GOAL_REACHED = "GOAL_REACHED"


class RobotController:
    def __init__(self,
                 wheel_radius: float = 0.0975,
                 wheel_base: float = 0.381,
                 max_linear_speed: float = 0.35,
                 max_angular_speed: float = 1.0,
                 goal_tolerance: float = 0.18,
                 safety_distance: float = 0.65,
                 stop_distance: float = 0.38,
                 k_att: float = 1.2,
                 k_rep: float = 0.9):
        """
        Args:
            wheel_radius: Radius of Pioneer P3DX drive wheels (m)
            wheel_base: Distance between drive wheels (m)
            max_linear_speed: Maximum forward velocity (m/s)
            max_angular_speed: Maximum rotational velocity (rad/s)
            goal_tolerance: Distance to goal to consider reached (m)
            safety_distance: Distance to obstacle to trigger avoidance deflection (m)
            stop_distance: Critical distance directly in front to trigger hard stop (m)
        """
        self.r = wheel_radius
        self.L = wheel_base
        self.max_v = max_linear_speed
        self.max_w = max_angular_speed
        self.goal_tolerance = goal_tolerance
        self.safety_distance = safety_distance
        self.stop_distance = stop_distance
        self.k_att = k_att
        self.k_rep = k_rep

        self.current_state = NavigationState.IDLE
        self.active_goal: Optional[Tuple[float, float]] = None

    def set_goal(self, gx: float, gy: float):
        self.active_goal = (float(gx), float(gy))
        self.current_state = NavigationState.NAVIGATING

    def diff_drive_ik(self, v: float, w: float) -> Tuple[float, float]:
        """
        Compute wheel rotational velocities (rad/s) from (linear_v, angular_w).
        """
        # Clamp inputs
        v = np.clip(v, -self.max_v, self.max_v)
        w = np.clip(w, -self.max_w, self.max_w)

        v_left = (v - (w * self.L / 2.0)) / self.r
        v_right = (v + (w * self.L / 2.0)) / self.r
        return float(v_left), float(v_right)

    def compute_control(self,
                        robot_x: float,
                        robot_y: float,
                        robot_yaw: float,
                        obstacle_centroids_2d: List[np.ndarray]) -> Tuple[float, float, NavigationState]:
        """
        Calculate target wheel speeds (v_left, v_right) and updated NavigationState.
        
        Args:
            robot_x, robot_y: Current robot position in Room frame (m)
            robot_yaw: Current robot heading in Room frame (rad)
            obstacle_centroids_2d: List of [x, y] coordinates of detected obstacles
            
        Returns:
            (v_left, v_right, current_state)
        """
        if self.active_goal is None:
            self.current_state = NavigationState.IDLE
            return 0.0, 0.0, self.current_state

        gx, gy = self.active_goal
        dx = gx - robot_x
        dy = gy - robot_y
        dist_to_goal = math.hypot(dx, dy)

        # 1. Goal Reached Check
        if dist_to_goal <= self.goal_tolerance:
            self.current_state = NavigationState.GOAL_REACHED
            return 0.0, 0.0, self.current_state

        # 2. Check for Immediate Obstacles Ahead
        # Transform obstacles into robot frame to inspect frontal sector
        c = math.cos(robot_yaw)
        s = math.sin(robot_yaw)
        critical_block = False
        min_frontal_dist = float("inf")

        for obs in obstacle_centroids_2d:
            ox = float(obs[0]) - robot_x
            oy = float(obs[1]) - robot_y
            # Ego coordinates: X_ego forward, Y_ego left
            ego_x = c * ox + s * oy
            ego_y = -s * ox + c * oy

            # Check if obstacle is in front and within robot width corridor
            if ego_x > 0.0 and abs(ego_y) < (self.L * 0.85):
                dist_ego = math.hypot(ego_x, ego_y)
                if dist_ego < min_frontal_dist:
                    min_frontal_dist = dist_ego
                if dist_ego <= self.stop_distance:
                    critical_block = True

        if critical_block:
            self.current_state = NavigationState.BLOCKED
            # Stop immediately to avoid collision
            return 0.0, 0.0, self.current_state

        # 3. Artificial Potential Field Calculation
        # Attractive force towards goal
        F_att_x = self.k_att * (dx / dist_to_goal)
        F_att_y = self.k_att * (dy / dist_to_goal)

        # Repulsive forces from obstacles
        F_rep_x = 0.0
        F_rep_y = 0.0
        active_avoidance = False

        for obs in obstacle_centroids_2d:
            d_obs_x = robot_x - float(obs[0])
            d_obs_y = robot_y - float(obs[1])
            d_obs = math.hypot(d_obs_x, d_obs_y)

            if 0.01 < d_obs < self.safety_distance:
                active_avoidance = True
                rep_magnitude = self.k_rep * (1.0 / d_obs - 1.0 / self.safety_distance) * (1.0 / (d_obs ** 2))
                rep_magnitude = min(rep_magnitude, 4.0)  # Bound repulsion
                F_rep_x += rep_magnitude * (d_obs_x / d_obs)
                F_rep_y += rep_magnitude * (d_obs_y / d_obs)

        F_total_x = F_att_x + F_rep_x
        F_total_y = F_att_y + F_rep_y

        desired_heading = math.atan2(F_total_y, F_total_x)
        heading_error = desired_heading - robot_yaw

        # Normalize heading error to [-pi, pi]
        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))

        # 4. Velocity Commands
        # Proportional heading control
        angular_gain = 1.8
        target_w = angular_gain * heading_error

        # Linear speed slows down when turning or near obstacles
        heading_alignment = max(0.0, math.cos(heading_error))
        target_v = self.max_v * (heading_alignment ** 2)

        # Scale down forward speed when in close avoidance
        if active_avoidance:
            target_v *= 0.65
            self.current_state = NavigationState.AVOIDING
        else:
            self.current_state = NavigationState.NAVIGATING

        # Slow down near goal
        if dist_to_goal < 0.60:
            target_v *= (dist_to_goal / 0.60)
            target_v = max(0.05, target_v)

        v_left, v_right = self.diff_drive_ik(target_v, target_w)
        return v_left, v_right, self.current_state
