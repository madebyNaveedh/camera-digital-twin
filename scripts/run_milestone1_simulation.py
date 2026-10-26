#!/usr/bin/env python3
"""
Milestone 1: Working Simulation Demonstration.
Tests CoppeliaSim room creation, Pioneer P3DX mobile robot loading,
goal navigation, and reactive obstacle response.
"""

import time
import math
import logging
import sys
import os
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from simulation.coppelia_client import CoppeliaClient
from simulation.scene_manager import SceneManager
from navigation.robot_controller import RobotController, NavigationState

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
logger = logging.getLogger("Milestone1")


def run_milestone1(headless: bool = False, duration_seconds: float = 20.0):
    client = CoppeliaClient()
    if not client.connect():
        logger.error("Could not connect to CoppeliaSim. Make sure CoppeliaSim is running!")
        return False

    sim = client.sim
    scene_mgr = SceneManager(sim=sim, room_width=5.0, room_length=4.0)

    # Robot start and mission goal
    start_pos = (-1.5, -1.0)
    goal_pos = (1.5, 1.0)

    # Setup room, robot, and goal marker
    if not scene_mgr.setup_scene(robot_start_pos=start_pos, goal_pos=goal_pos):
        logger.error("Scene setup failed.")
        return False

    # Add a test obstacle in the path (e.g. at (0.0, 0.0) with radius 0.20m)
    obs_pos = np.array([0.0, 0.0, 0.25])
    test_obs_handle = scene_mgr.update_or_create_obstacle(
        sim_handle=None,
        name="TestObstacle",
        position=obs_pos,
        radius=0.22,
        height=0.50
    )
    logger.info(f"Test obstacle placed at {obs_pos[:2]} with handle {test_obs_handle}")

    # Initialize controller
    controller = RobotController(
        wheel_radius=0.0975,
        wheel_base=0.381,
        max_linear_speed=0.35,
        max_angular_speed=1.0,
        goal_tolerance=0.20,
        safety_distance=0.65,
        stop_distance=0.35
    )
    controller.set_goal(goal_pos[0], goal_pos[1])

    # Start simulation
    client.start_simulation()
    time.sleep(0.5)

    logger.info("Autonomous navigation loop started...")
    start_time = time.time()
    dt = 0.05
    last_state = None

    try:
        while (time.time() - start_time) < duration_seconds:
            # 1. Read robot pose
            rx, ry, ryaw = scene_mgr.get_robot_pose()

            # 2. Obstacles list in Room frame
            obstacles = [obs_pos[:2]]

            # 3. Compute control
            vl, vr, state = controller.compute_control(rx, ry, ryaw, obstacles)

            if state != last_state:
                logger.info(f"Navigation State Changed: {state.value} | Robot at ({rx:.2f}, {ry:.2f})")
                last_state = state

            # 4. Command wheels
            scene_mgr.set_robot_wheel_speeds(vl, vr)

            dist_to_goal = math.hypot(goal_pos[0] - rx, goal_pos[1] - ry)
            dist_to_obs = math.hypot(obs_pos[0] - rx, obs_pos[1] - ry)

            if state == NavigationState.GOAL_REACHED:
                logger.info(f"SUCCESS: Goal reached at ({rx:.2f}, {ry:.2f})! Distance={dist_to_goal:.3f}m")
                break

            time.sleep(dt)

    finally:
        logger.info("Stopping robot and stopping simulation...")
        scene_mgr.set_robot_wheel_speeds(0.0, 0.0)
        scene_mgr.remove_obstacle(test_obs_handle)
        client.stop_simulation()

    logger.info("Milestone 1 test completed successfully.")
    return True


if __name__ == "__main__":
    run_milestone1()
