"""
Digital-Twin Synchronizer.
Associates physical sensor detections, performs temporal filtering,
and synchronizes obstacle objects with CoppeliaSim.
"""

from typing import List, Dict, Optional, Tuple
import time
import numpy as np
from digital_twin.environment_state import EnvironmentState, TrackedObstacle
from perception.obstacle_detection import ObstacleCandidate


class DigitalTwinSynchronizer:
    def __init__(self,
                 state: EnvironmentState,
                 association_distance_threshold: float = 0.45,
                 confirmation_frames: int = 2,
                 removal_timeout_seconds: float = 2.5,
                 smoothing_alpha: float = 0.65,
                 scene_manager=None):
        """
        Args:
            state: The shared EnvironmentState
            association_distance_threshold: Max distance (m) to match an observation with existing obstacle
            confirmation_frames: Number of observation frames before confirming an obstacle
            removal_timeout_seconds: Time (s) without observation before removing an obstacle
            smoothing_alpha: Position smoothing factor (1.0 = no smoothing, 0.5 = 50% blend)
            scene_manager: Optional CoppeliaSim SceneManager instance to mirror objects
        """
        self.state = state
        self.assoc_dist_thresh = association_distance_threshold
        self.confirmation_frames = confirmation_frames
        self.removal_timeout = removal_timeout_seconds
        self.alpha = smoothing_alpha
        self.scene_manager = scene_manager
        
        # Performance metrics
        self.last_sync_time: float = 0.0
        self.sync_duration_ms: float = 0.0
        self.total_sync_count: int = 0

    def process_observations(self,
                             candidates: List[ObstacleCandidate],
                             current_time: Optional[float] = None) -> List[TrackedObstacle]:
        """
        Main update step:
        1. Match incoming candidates with existing tracked obstacles
        2. Update matched obstacles (smooth position, increment count)
        3. Create new unconfirmed obstacles for unmatched candidates
        4. Age unmatched existing obstacles and prune timed-out ones
        5. Push updates to CoppeliaSim simulation if scene_manager is attached
        """
        t0 = time.perf_counter()
        if current_time is None:
            current_time = time.time()

        existing_obstacles = list(self.state.obstacles.values())
        unmatched_candidates = list(range(len(candidates)))
        matched_obstacle_ids = set()

        # Step 1 & 2: Nearest neighbor association
        for obs in existing_obstacles:
            best_cand_idx = -1
            best_dist = float("inf")

            for c_idx in unmatched_candidates:
                cand = candidates[c_idx]
                dist = float(np.linalg.norm(obs.centroid[:2] - cand.centroid[:2]))
                if dist < self.assoc_dist_thresh and dist < best_dist:
                    best_dist = dist
                    best_cand_idx = c_idx

            if best_cand_idx >= 0:
                # Match found! Update tracked obstacle
                cand = candidates[best_cand_idx]
                unmatched_candidates.remove(best_cand_idx)
                matched_obstacle_ids.add(obs.id)

                # Exponential smoothing of position
                new_pos_2d = self.alpha * cand.centroid[:2] + (1.0 - self.alpha) * obs.centroid[:2]
                obs.centroid = np.array([new_pos_2d[0], new_pos_2d[1], cand.radius], dtype=np.float64)
                obs.radius = self.alpha * cand.radius + (1.0 - self.alpha) * obs.radius
                obs.confidence = self.alpha * cand.confidence + (1.0 - self.alpha) * obs.confidence
                obs.last_observed_time = current_time
                obs.observation_count += 1
                obs.missed_count = 0

                if obs.observation_count >= self.confirmation_frames:
                    obs.is_confirmed = True

        # Step 3: Instantiate candidates that were not matched
        for c_idx in unmatched_candidates:
            cand = candidates[c_idx]
            new_id = self.state.generate_obstacle_id()
            centroid_3d = np.array([cand.centroid[0], cand.centroid[1], cand.radius], dtype=np.float64)
            is_confirmed = (self.confirmation_frames <= 1)
            
            new_obs = TrackedObstacle(
                id=new_id,
                centroid=centroid_3d,
                radius=cand.radius,
                confidence=cand.confidence,
                first_observed_time=current_time,
                last_observed_time=current_time,
                observation_count=1,
                missed_count=0,
                is_confirmed=is_confirmed
            )
            self.state.obstacles[new_id] = new_obs
            if is_confirmed:
                matched_obstacle_ids.add(new_id)

        # Step 4: Handle unmatched existing obstacles (decay / timeout)
        expired_ids = []
        for obs_id, obs in self.state.obstacles.items():
            if obs_id not in matched_obstacle_ids:
                obs.missed_count += 1
                if obs.is_stale(current_time, self.removal_timeout):
                    expired_ids.append(obs_id)

        # Prune expired obstacles
        for exp_id in expired_ids:
            exp_obs = self.state.remove_obstacle(exp_id)
            if self.scene_manager and exp_obs and exp_obs.sim_handle is not None:
                try:
                    self.scene_manager.remove_obstacle(exp_obs.sim_handle)
                except Exception as e:
                    pass

        # Step 5: Mirror confirmed obstacles to CoppeliaSim
        confirmed_obstacles = self.state.get_confirmed_obstacles()
        if self.scene_manager:
            for obs in confirmed_obstacles:
                try:
                    handle = self.scene_manager.update_or_create_obstacle(
                        sim_handle=obs.sim_handle,
                        name=obs.id,
                        position=obs.centroid,
                        radius=obs.radius,
                        height=obs.radius * 2.0
                    )
                    obs.sim_handle = handle
                except Exception as e:
                    pass

        t1 = time.perf_counter()
        self.sync_duration_ms = (t1 - t0) * 1000.0
        self.last_sync_time = current_time
        self.state.last_sync_timestamp = current_time
        self.total_sync_count += 1

        return confirmed_obstacles
