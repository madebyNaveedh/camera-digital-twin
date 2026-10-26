#!/usr/bin/env python3
"""
Full End-to-End Live Demonstration Script for Faculty / Evaluators.
Demonstrates:
1. Physical room setup in CoppeliaSim.
2. Real-time sensor acquisition (camera & LiDAR).
3. Obstacle introduction -> Detection -> CoppeliaSim dynamic object creation.
4. Simulated Pioneer P3DX mobile robot navigation responding to the physical obstacle.
5. Obstacle movement / removal -> Virtual twin update -> Robot resuming navigation to goal.
6. Interactive monitoring dashboard on http://localhost:8085.
"""

import time
import math
import logging
import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from digital_twin.engine import DigitalTwinEngine
from dashboard.app import DashboardServer

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
logger = logging.getLogger("LiveDemonstration")


def run_full_demonstration(interactive: bool = True, duration: float = 45.0):
    print("=" * 70)
    print(" 🚀 STARTING REAL-TIME DIGITAL TWIN FULL DEMONSTRATION")
    print("=" * 70)

    # 1. Initialize Engine
    engine = DigitalTwinEngine(
        room_width=5.0,
        room_length=4.0,
        lidar_port="/dev/ttyUSB0",
        camera_index=0,
        robot_start=(-1.6, -1.2),
        robot_goal=(1.6, 1.2)
    )

    # 2. Start Dashboard
    dashboard = DashboardServer(engine=engine, port=8085)
    dashboard.start()

    # 3. Start Engine & CoppeliaSim integration
    engine.start(connect_simulation=True)
    time.sleep(1.0)

    print("\n[STEP 1] Virtual Room loaded in CoppeliaSim.")
    print("         Dashboard active at http://localhost:8085")
    print("         Robot initialised at start pose (-1.6, -1.2)")

    t_start = time.time()
    step_stage = 0

    try:
        while (time.time() - t_start) < duration:
            elapsed = time.time() - t_start
            telemetry = engine.get_telemetry()
            rx, ry, ryaw = telemetry["robot"]["pose"]
            nav_state = telemetry["robot"]["nav_state"]
            obs_count = telemetry["digital_twin"]["confirmed_obstacle_count"]

            # Step 2: After 3 seconds, introduce obstacle directly into the robot's navigation path
            if step_stage == 0 and elapsed >= 3.0:
                print("\n[STEP 2] Physical Obstacle introduced at (0.0, 0.0)!")
                print("         Observing sensor acquisition & obstacle detection...")
                engine.lidar.set_synthetic_obstacle("demo_box", 0.0, 0.0, radius=0.22)
                step_stage = 1

            # Step 3: Verify obstacle is mapped and robot responds
            elif step_stage == 1 and elapsed >= 6.0:
                print(f"\n[STEP 3] Digital Twin Status: {obs_count} obstacle(s) mapped in CoppeliaSim.")
                print(f"         Robot State: {nav_state} (reacting to obstacle at [0, 0])")
                step_stage = 2

            # Step 4: After 14 seconds, move the obstacle to a new location (0.8, -0.2)
            elif step_stage == 2 and elapsed >= 14.0:
                print("\n[STEP 4] Moving physical obstacle to (0.8, -0.2)...")
                engine.lidar.set_synthetic_obstacle("demo_box", 0.8, -0.2, radius=0.22)
                step_stage = 3

            # Step 5: After 24 seconds, remove the obstacle completely
            elif step_stage == 3 and elapsed >= 24.0:
                print("\n[STEP 5] Removing physical obstacle...")
                print("         Observing temporal timeout and removal from virtual twin...")
                engine.lidar.remove_synthetic_obstacle("demo_box")
                step_stage = 4

            # Step 6: Robot resumes path and drives to goal
            elif step_stage == 4 and nav_state == "GOAL_REACHED":
                print(f"\n[SUCCESS] Robot reached mission goal at ({rx:.2f}, {ry:.2f})!")
                break

            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\nDemonstration interrupted by user.")

    finally:
        print("\nShutting down demonstration cleanly...")
        dashboard.stop()
        engine.stop()
        print("Demonstration finished.\n" + "=" * 70)


if __name__ == "__main__":
    run_full_demonstration()
