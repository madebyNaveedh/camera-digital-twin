"""
Unit tests for obstacle detection and clustering.
"""

import unittest
import numpy as np
import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from perception.obstacle_detection import ObstacleDetector, ObstacleCandidate


class TestObstacleDetection(unittest.TestCase):
    def setUp(self):
        self.detector = ObstacleDetector(
            distance_threshold=0.25,
            min_cluster_size=4,
            max_cluster_size=100,
            min_obstacle_radius=0.05,
            max_obstacle_radius=0.60
        )

    def test_single_cluster_detection(self):
        # Generate points around centroid (1.0, 1.5) with radius ~0.15m
        rng = np.random.default_rng(42)
        angles = np.linspace(0, 2 * np.pi, 12, endpoint=False)
        pts = np.column_stack((
            1.0 + 0.15 * np.cos(angles) + rng.normal(0, 0.01, 12),
            1.5 + 0.15 * np.sin(angles) + rng.normal(0, 0.01, 12)
        ))

        candidates = self.detector.detect_obstacles(pts)
        self.assertEqual(len(candidates), 1)
        self.assertAlmostEqual(candidates[0].centroid[0], 1.0, delta=0.10)
        self.assertAlmostEqual(candidates[0].centroid[1], 1.5, delta=0.10)
        self.assertAlmostEqual(candidates[0].radius, 0.15, delta=0.05)
        self.assertGreaterEqual(candidates[0].confidence, 0.5)

    def test_two_distinct_clusters(self):
        # Cluster A around (-1.0, 0.5)
        cA = np.array([
            [-1.0, 0.5],
            [-1.05, 0.52],
            [-0.95, 0.48],
            [-1.02, 0.55],
            [-0.98, 0.45]
        ])
        # Cluster B around (1.2, -0.8)
        cB = np.array([
            [1.2, -0.8],
            [1.25, -0.78],
            [1.18, -0.82],
            [1.22, -0.75],
            [1.15, -0.85]
        ])
        # Noise points far away
        noise = np.array([
            [0.0, 0.0],
            [0.5, 0.5]
        ])

        all_pts = np.vstack([cA, cB, noise])
        candidates = self.detector.detect_obstacles(all_pts)

        # Should detect exactly 2 clusters; isolated noise points should be filtered out
        self.assertEqual(len(candidates), 2)

        centroids = [c.centroid for c in candidates]
        # Check that one is near (-1, 0.5) and one near (1.2, -0.8)
        found_A = any(np.linalg.norm(c - np.array([-1.0, 0.5])) < 0.1 for c in centroids)
        found_B = any(np.linalg.norm(c - np.array([1.2, -0.8])) < 0.1 for c in centroids)
        self.assertTrue(found_A)
        self.assertTrue(found_B)

    def test_noise_suppression(self):
        # Only isolated noise points
        noise = np.array([
            [-2.0, 1.0],
            [0.0, -1.0],
            [1.5, 0.2]
        ])
        candidates = self.detector.detect_obstacles(noise)
        self.assertEqual(len(candidates), 0)


if __name__ == "__main__":
    unittest.main()
