"""
Unit and integration tests for Camera-Driven Digital Twin.
Tests ArUco detection, multi-color presets, eyedropper, and sensitivity coordinate mapping.
"""

import unittest
import numpy as np
import cv2
import os
import sys

from camera_tracker import CameraTracker
from coordinate_mapping import CoordinateMapper
from generate_marker import generate_marker


class TestCameraDigitalTwin(unittest.TestCase):
    def setUp(self):
        self.tmp_dir = "/tmp/test_camera_twin"
        os.makedirs(self.tmp_dir, exist_ok=True)

    def test_marker_generation_and_detection(self):
        # 1. Generate marker with quiet border
        marker_path = generate_marker(marker_id=0, dict_name="DICT_4X4_50", size_px=300, output_dir=self.tmp_dir)
        self.assertTrue(os.path.exists(marker_path))

        # 2. Load generated image
        img = cv2.imread(marker_path)
        self.assertIsNotNone(img)

        # 3. Test tracker detection on this image
        tracker = CameraTracker(mode="aruco", target_marker_id=0)
        res = tracker.track(img)

        self.assertTrue(res["tracked"])
        self.assertEqual(res["confidence"], 1.0)
        self.assertIsNotNone(res["center"])
        h, w = img.shape[:2]
        self.assertAlmostEqual(res["center"][0], w / 2.0, delta=10.0)
        self.assertAlmostEqual(res["center"][1], h / 2.0, delta=10.0)

    def test_multi_color_tracking(self):
        # 1. Test Blue Preset
        img_blue = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.circle(img_blue, (320, 240), 40, (255, 120, 20), -1)  # BGR Blue

        tracker_blue = CameraTracker(mode="color", color_preset="blue")
        res_blue = tracker_blue.track(img_blue)
        self.assertTrue(res_blue["tracked"])
        self.assertAlmostEqual(res_blue["center"][0], 320.0, delta=5.0)

        # 2. Test Green Preset
        img_green = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.circle(img_green, (200, 150), 40, (20, 220, 30), -1)  # BGR Green

        tracker_green = CameraTracker(mode="color", color_preset="green")
        res_green = tracker_green.track(img_green)
        self.assertTrue(res_green["tracked"])
        self.assertAlmostEqual(res_green["center"][0], 200.0, delta=5.0)

        # 3. Test Red Preset
        img_red = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.circle(img_red, (450, 300), 40, (20, 20, 240), -1)  # BGR Red

        tracker_red = CameraTracker(mode="color", color_preset="red")
        res_red = tracker_red.track(img_red)
        self.assertTrue(res_red["tracked"])
        self.assertAlmostEqual(res_red["center"][0], 450.0, delta=5.0)

    def test_eyedropper_click_sampling(self):
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        # Custom purple patch: BGR (200, 30, 150)
        cv2.circle(img, (250, 200), 45, (200, 30, 150), -1)

        tracker = CameraTracker(mode="color")
        tracker.last_frame = img
        tracker.pick_color_at_pixel(250, 200)

        res = tracker.track(img)
        self.assertTrue(res["tracked"])
        self.assertAlmostEqual(res["center"][0], 250.0, delta=6.0)

    def test_coordinate_mapping_and_sensitivity_gain(self):
        mapper = CoordinateMapper(
            u_min=140, u_max=500,
            v_min=100, v_max=380,
            sim_x_min=-3.0, sim_x_max=3.0,
            sim_y_min=-2.0, sim_y_max=2.0,
            sensitivity_gain=2.0,
            smoothing_alpha=1.0,  # No EMA lag for unit test assertion
            safety_zone=(-0.5, 0.5, -0.5, 0.5)
        )

        # Center: (320, 240) -> (0.0, 0.0)
        p1, active1, in_zone1 = mapper.update((320, 240))
        self.assertTrue(active1)
        self.assertAlmostEqual(p1[0], 0.0, delta=0.01)
        self.assertAlmostEqual(p1[1], 0.0, delta=0.01)
        self.assertTrue(in_zone1)

        # Moderate 90px movement right (320 + 90 = 410) with 2x gain reaches full right (3.0m)!
        # half_w = 180, so 90 / 180 = 0.5. With gain 2.0 -> scaled_u = 1.0 -> sim_x = +3.0m!
        p2, active2, _ = mapper.update((410, 240))
        self.assertTrue(active2)
        self.assertAlmostEqual(p2[0], 3.0, delta=0.01)

        # When object leaves camera frame (None observation), position is held!
        p_held, active_held, _ = mapper.update(None)
        self.assertAlmostEqual(p_held[0], 3.0, delta=0.01)


if __name__ == "__main__":
    unittest.main()
