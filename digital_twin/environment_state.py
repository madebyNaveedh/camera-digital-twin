"""
Digital-Twin State representation of the physical environment.
"""

from dataclasses import dataclass, field
from typing import Dict, List, Optional
import time
import numpy as np


@dataclass
class TrackedObstacle:
    id: str
    centroid: np.ndarray          # [x, y, z] in Physical Room coordinates
    radius: float                 # Estimated radius (m)
    confidence: float             # In [0.0, 1.0]
    first_observed_time: float
    last_observed_time: float
    observation_count: int = 1
    missed_count: int = 0
    is_confirmed: bool = False
    sim_handle: Optional[int] = None

    def age(self, current_time: float) -> float:
        return current_time - self.last_observed_time

    def is_stale(self, current_time: float, timeout_seconds: float) -> bool:
        return self.age(current_time) > timeout_seconds


class EnvironmentState:
    def __init__(self, room_width: float = 5.0, room_length: float = 4.0):
        self.room_width = room_width
        self.room_length = room_length
        self.obstacles: Dict[str, TrackedObstacle] = {}
        self.last_sensor_timestamp: float = 0.0
        self.last_sync_timestamp: float = 0.0
        self.next_obstacle_id: int = 1

    def generate_obstacle_id(self) -> str:
        oid = f"Obstacle_{self.next_obstacle_id}"
        self.next_obstacle_id += 1
        return oid

    def get_confirmed_obstacles(self) -> List[TrackedObstacle]:
        """Return list of confirmed obstacles (active and validated)."""
        return [obs for obs in self.obstacles.values() if obs.is_confirmed]

    def remove_obstacle(self, obstacle_id: str) -> Optional[TrackedObstacle]:
        return self.obstacles.pop(obstacle_id, None)

    def clear(self):
        self.obstacles.clear()
        self.next_obstacle_id = 1
