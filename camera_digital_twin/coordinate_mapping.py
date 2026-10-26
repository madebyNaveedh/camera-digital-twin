"""
Maps 2D camera pixels to 3D simulation coordinates.
Includes desk-to-arena scaling and exponential moving average (EMA) smoothing.
"""

from typing import Tuple, Optional
import time
import numpy as np


class CoordinateMapper:
    def __init__(self,
                 u_min: int = 140,
                 u_max: int = 500,
                 v_min: int = 100,
                 v_max: int = 380,
                 sim_x_min: float = -3.5,
                 sim_x_max: float = 3.5,
                 sim_y_min: float = -2.5,
                 sim_y_max: float = 2.5,
                 sensitivity_gain: float = 2.2,
                 smoothing_alpha: float = 0.70,
                 loss_timeout: float = 2.0,
                 safety_zone: Optional[Tuple[float, float, float, float]] = (-0.6, 0.6, -0.6, 0.6)):
        
        # Central Desk Control Zone (pixels)
        self.u_min = float(u_min)
        self.u_max = float(u_max)
        self.v_min = float(v_min)
        self.v_max = float(v_max)

        # Simulation world bounds (covers corridors and rooms)
        self.sim_x_min = sim_x_min
        self.sim_x_max = sim_x_max
        self.sim_y_min = sim_y_min
        self.sim_y_max = sim_y_max

        self.sensitivity_gain = float(sensitivity_gain)
        self.alpha = smoothing_alpha
        self.loss_timeout = loss_timeout
        self.safety_zone = safety_zone

        # State
        self.smoothed_sim_pos: Optional[np.ndarray] = None
        self.last_valid_sim_pos: np.ndarray = np.array([0.0, 0.0], dtype=np.float64)
        self.last_seen_time: float = 0.0
        self.is_tracking_active: bool = False

    def pixel_to_sim(self, u: float, v: float) -> Tuple[float, float]:
        """
        Map camera pixel coordinates (u, v) to CoppeliaSim workspace (x, y)
        using center-referenced sensitivity gain scaling.
        
        A small, comfortable physical movement in the center of the camera
        smoothly traverses the full simulation world without hitting camera edges.
        """
        center_u = (self.u_min + self.u_max) / 2.0
        center_v = (self.v_min + self.v_max) / 2.0
        half_w = max(10.0, (self.u_max - self.u_min) / 2.0)
        half_h = max(10.0, (self.v_max - self.v_min) / 2.0)

        # Normalized coordinates centered at (0, 0), from -1.0 to +1.0
        norm_u = (u - center_u) / half_w
        norm_v = (v - center_v) / half_h

        # Apply sensitivity gain multiplier
        scaled_u = np.clip(norm_u * self.sensitivity_gain, -1.0, 1.0)
        scaled_v = np.clip(norm_v * self.sensitivity_gain, -1.0, 1.0)

        # Map to simulation workspace
        sim_center_x = (self.sim_x_min + self.sim_x_max) / 2.0
        sim_half_x = (self.sim_x_max - self.sim_x_min) / 2.0
        sim_center_y = (self.sim_y_min + self.sim_y_max) / 2.0
        sim_half_y = (self.sim_y_max - self.sim_y_min) / 2.0

        sim_x = sim_center_x + scaled_u * sim_half_x
        # Invert vertical axis: top of frame (+norm_v) corresponds to +Y in simulation
        sim_y = sim_center_y - scaled_v * sim_half_y

        # Clamp to world limits
        sim_x = np.clip(sim_x, self.sim_x_min, self.sim_x_max)
        sim_y = np.clip(sim_y, self.sim_y_min, self.sim_y_max)

        return float(sim_x), float(sim_y)

    def set_sensitivity(self, delta: float):
        """Dynamically adjust sensitivity multiplier."""
        self.sensitivity_gain = np.clip(self.sensitivity_gain + delta, 0.8, 5.0)

    def update(self, center_px: Optional[Tuple[float, float]]) -> Tuple[Optional[Tuple[float, float]], bool, bool]:
        """
        Update tracking with new pixel observation.
        Maintains last valid position when object leaves camera frame.
        
        Returns:
            (sim_coords, is_active, in_safety_zone)
        """
        now = time.time()

        if center_px is not None:
            raw_x, raw_y = self.pixel_to_sim(center_px[0], center_px[1])
            new_pos = np.array([raw_x, raw_y], dtype=np.float64)

            if self.smoothed_sim_pos is None:
                self.smoothed_sim_pos = new_pos
            else:
                # Exponential moving average filter for jitter reduction
                self.smoothed_sim_pos = self.alpha * new_pos + (1.0 - self.alpha) * self.smoothed_sim_pos

            self.last_valid_sim_pos = self.smoothed_sim_pos.copy()
            self.last_seen_time = now
            self.is_tracking_active = True
        else:
            # Check timeout
            if (now - self.last_seen_time) > self.loss_timeout:
                self.is_tracking_active = False

        # When tracking is temporarily lost, hold last valid position in simulation
        if self.smoothed_sim_pos is None:
            self.smoothed_sim_pos = self.last_valid_sim_pos.copy()

        curr_x = float(self.smoothed_sim_pos[0])
        curr_y = float(self.smoothed_sim_pos[1])
        in_zone = self.is_tracking_active and self.check_safety_zone(curr_x, curr_y)

        return (curr_x, curr_y), self.is_tracking_active, in_zone

    def check_safety_zone(self, x: float, y: float) -> bool:
        """Check if coordinates fall inside the designated safety corridor."""
        if not self.safety_zone:
            return False
        x_min, x_max, y_min, y_max = self.safety_zone
        return (x_min <= x <= x_max) and (y_min <= y <= y_max)
