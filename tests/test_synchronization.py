"""
Unit tests for Digital-Twin state and synchronisation.
"""

import unittest
import numpy as np
import time
import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from digital_twin.environment_state import EnvironmentState, TrackedObstacle
from digital_twin.synchronizer import DigitalTwinSynchronizer
from perception.obstacle_detection import ObstacleCandidate


class TestDigitalTwinSynchronizer(unittest.TestCase):
    def setUp(self):
        self.state = EnvironmentState(room_width=5.0, room_length=4.0)
        self.sync = DigitalTwinSynchronizer(
            state=self.state,
            association_distance_threshold=0.40,
            confirmation_frames=2,
            removal_timeout_seconds=2.0,
            smoothing_alpha=0.5
        )

    def test_confirmation_frames(self):
        t0 = 1000.0
        cand = ObstacleCandidate(
            id=0,
            centroid=np.array([1.0, 1.0]),
            radius=0.15,
            point_count=10,
            confidence=0.8,
            points=np.array([[1.0, 1.0]]),
            bbox_extent=np.array([0.3, 0.3])
        )

        # Frame 1: Candidate should be tracked but NOT yet confirmed
        confirmed = self.sync.process_observations([cand], current_time=t0)
        self.assertEqual(len(confirmed), 0)
        self.assertEqual(len(self.state.obstacles), 1)

        # Frame 2: Same candidate observed again -> should become confirmed
        confirmed = self.sync.process_observations([cand], current_time=t0 + 0.1)
        self.assertEqual(len(confirmed), 1)
        self.assertEqual(confirmed[0].id, "Obstacle_1")
        self.assertTrue(confirmed[0].is_confirmed)

    def test_moving_obstacle_association(self):
        t0 = 1000.0
        # Frame 1: Obstacle at (0.0, 0.0)
        cand1 = ObstacleCandidate(0, np.array([0.0, 0.0]), 0.2, 10, 0.9, np.zeros((1, 2)), np.zeros(2))
        self.sync.process_observations([cand1], current_time=t0)
        
        # Frame 2: Obstacle moves slightly to (0.1, 0.0)
        cand2 = ObstacleCandidate(0, np.array([0.1, 0.0]), 0.2, 10, 0.9, np.zeros((1, 2)), np.zeros(2))
        confirmed = self.sync.process_observations([cand2], current_time=t0 + 0.1)

        # Should retain same ID Obstacle_1
        self.assertEqual(len(confirmed), 1)
        self.assertEqual(confirmed[0].id, "Obstacle_1")
        # Position smoothed: 0.5 * 0.1 + 0.5 * 0.0 = 0.05
        self.assertAlmostEqual(confirmed[0].centroid[0], 0.05, places=3)

    def test_obstacle_timeout_removal(self):
        t0 = 1000.0
        cand = ObstacleCandidate(0, np.array([0.5, 0.5]), 0.2, 10, 0.9, np.zeros((1, 2)), np.zeros(2))
        self.sync.process_observations([cand], current_time=t0)
        self.sync.process_observations([cand], current_time=t0 + 0.1)
        self.assertEqual(len(self.state.get_confirmed_obstacles()), 1)

        # No observations for 1.0 second (timeout is 2.0s) -> should still exist
        self.sync.process_observations([], current_time=t0 + 1.1)
        self.assertEqual(len(self.state.get_confirmed_obstacles()), 1)

        # No observations after 2.5 seconds -> timed out and pruned
        self.sync.process_observations([], current_time=t0 + 2.6)
        self.assertEqual(len(self.state.get_confirmed_obstacles()), 0)
        self.assertEqual(len(self.state.obstacles), 0)


if __name__ == "__main__":
    unittest.main()
