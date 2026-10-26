"""
Unit and functional tests for sensor acquisition interfaces.
"""

import unittest
import time
import numpy as np
import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from sensors.camera_interface import CameraInterface
from sensors.lidar_interface import LidarInterface, SyntheticLidarGenerator


class TestSensorInterfaces(unittest.TestCase):
    def test_synthetic_lidar_raycasting(self):
        gen = SyntheticLidarGenerator(room_width=4.0, room_length=4.0, noise_std=0.0)
        scan = gen.generate_scan()
        self.assertEqual(len(scan.angles_deg), 360)
        self.assertEqual(len(scan.ranges_m), 360)

        # 0 deg (+X East wall at x=2.0) should hit wall at ~2.0m
        idx_0 = 0
        self.assertAlmostEqual(scan.ranges_m[idx_0], 2.0, delta=0.05)

        # 90 deg (+Y North wall at y=2.0) should hit wall at ~2.0m
        idx_90 = 90
        self.assertAlmostEqual(scan.ranges_m[idx_90], 2.0, delta=0.05)

        # Now place an obstacle at (1.0, 0.0) with radius 0.2m
        # Ray at 0 deg should hit the obstacle front at x = 1.0 - 0.2 = 0.8m!
        gen.set_obstacle("box1", 1.0, 0.0, radius=0.20)
        scan_with_obs = gen.generate_scan()
        self.assertAlmostEqual(scan_with_obs.ranges_m[idx_0], 0.80, delta=0.05)

        # Ray at 90 deg should still hit wall at 2.0m
        self.assertAlmostEqual(scan_with_obs.ranges_m[idx_90], 2.0, delta=0.05)

    def test_lidar_interface_lifecycle(self):
        lidar = LidarInterface(port="/dev/ttyUSB_NONEXISTENT", fallback_synthetic=True)
        self.assertTrue(lidar.start())
        time.sleep(0.25)
        scan = lidar.get_latest_scan()
        self.assertIsNotNone(scan)
        self.assertFalse(scan.is_hardware)
        self.assertGreater(len(scan.ranges_m), 100)
        lidar.stop()

    def test_camera_interface_lifecycle(self):
        # Test real webcam at /dev/video0
        cam = CameraInterface(device_index=0, width=640, height=480, fps=30)
        started = cam.start()
        if started:
            time.sleep(0.3)
            frame, ts, valid = cam.get_latest_frame()
            self.assertTrue(valid)
            self.assertIsNotNone(frame)
            self.assertEqual(frame.shape[0], 480)
            self.assertEqual(frame.shape[1], 640)
            self.assertEqual(frame.shape[2], 3)
            self.assertGreater(ts, 0.0)
            cam.stop()
        else:
            self.skipTest("Physical camera not accessible in current environment")


if __name__ == "__main__":
    unittest.main()
