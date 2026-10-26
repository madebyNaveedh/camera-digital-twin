"""
LiDAR measurement filtering and preprocessing.
"""

from typing import Tuple, Optional
import numpy as np


class LidarProcessor:
    def __init__(self,
                 range_min: float = 0.15,
                 range_max: float = 10.0,
                 room_bounds: Optional[Tuple[float, float, float, float]] = None):
        """
        Args:
            range_min: Minimum valid range (m)
            range_max: Maximum valid range (m)
            room_bounds: Optional (x_min, x_max, y_min, y_max) to clip points to the physical room
        """
        self.range_min = range_min
        self.range_max = range_max
        self.room_bounds = room_bounds

    def filter_raw_scan(self,
                        angles_deg: np.ndarray,
                        ranges_m: np.ndarray,
                        qualities: Optional[np.ndarray] = None) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Filter invalid, infinite, zero, or out-of-bounds range measurements.
        """
        angles = np.asarray(angles_deg, dtype=np.float64)
        ranges = np.asarray(ranges_m, dtype=np.float64)
        if qualities is not None:
            quals = np.asarray(qualities, dtype=np.float64)
        else:
            quals = np.ones_like(ranges)

        valid_mask = (
            np.isfinite(ranges) &
            (ranges >= self.range_min) &
            (ranges <= self.range_max) &
            np.isfinite(angles)
        )

        return angles[valid_mask], ranges[valid_mask], quals[valid_mask]

    def filter_room_boundaries(self, points_room_2d: np.ndarray) -> np.ndarray:
        """
        Filter out points that fall outside the defined physical room boundaries.
        """
        if self.room_bounds is None or points_room_2d.size == 0:
            return points_room_2d

        x_min, x_max, y_min, y_max = self.room_bounds
        x = points_room_2d[:, 0]
        y = points_room_2d[:, 1]

        in_bounds_mask = (x >= x_min) & (x <= x_max) & (y >= y_min) & (y <= y_max)
        return points_room_2d[in_bounds_mask]
