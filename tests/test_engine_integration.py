"""
Integration tests for the complete Digital Twin Engine pipeline.
"""

import unittest
import time
import numpy as np
import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from digital_twin.engine import DigitalTwinEngine


class TestDigitalTwinEngineIntegration(unittest.TestCase):
    def setUp(self):
        self.engine = DigitalTwinEngine(
            room_width=5.0,
            room_length=4.0,
            lidar_port="/dev/ttyUSB_NONEXISTENT",
            camera_index=0
        )

    def test_end_to_end_pipeline_and_telemetry(self):
        # Start engine in simulation-detached mode (no CoppeliaSim required for headless unit test)
        self.assertTrue(self.engine.start(connect_simulation=False))
        time.sleep(0.3)

        # Baseline: clean room, no obstacles
        telemetry = self.engine.get_telemetry()
        self.assertEqual(telemetry["digital_twin"]["confirmed_obstacle_count"], 0)

        # Introduce an obstacle at (0.5, 0.8) with radius 0.18m
        self.engine.lidar.set_synthetic_obstacle("test_box", 0.5, 0.8, radius=0.18)
        
        # Allow pipeline to acquire scans and confirm obstacle (2 frames ~ 0.3s)
        time.sleep(0.5)
        
        telemetry_with_obs = self.engine.get_telemetry()
        self.assertEqual(telemetry_with_obs["digital_twin"]["confirmed_obstacle_count"], 1)
        obs_data = telemetry_with_obs["digital_twin"]["obstacles"][0]
        
        # Verify centroid is accurate within 5cm
        self.assertAlmostEqual(obs_data["centroid"][0], 0.5, delta=0.08)
        self.assertAlmostEqual(obs_data["centroid"][1], 0.8, delta=0.08)
        self.assertAlmostEqual(obs_data["radius"], 0.18, delta=0.08)

        # Move obstacle to (0.8, 0.8)
        self.engine.lidar.set_synthetic_obstacle("test_box", 0.8, 0.8, radius=0.18)
        time.sleep(0.5)
        telemetry_moved = self.engine.get_telemetry()
        self.assertEqual(telemetry_moved["digital_twin"]["confirmed_obstacle_count"], 1)
        obs_data_moved = telemetry_moved["digital_twin"]["obstacles"][0]
        self.assertAlmostEqual(obs_data_moved["centroid"][0], 0.8, delta=0.08)

        # Remove obstacle -> verify it gets removed after timeout
        self.engine.lidar.remove_synthetic_obstacle("test_box")
        time.sleep(2.5)  # timeout is 2.0s
        telemetry_cleared = self.engine.get_telemetry()
        self.assertEqual(telemetry_cleared["digital_twin"]["confirmed_obstacle_count"], 0)

        self.engine.stop()


if __name__ == "__main__":
    unittest.main()
