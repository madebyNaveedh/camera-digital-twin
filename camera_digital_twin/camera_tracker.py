"""
Camera tracker for ArUco markers and HSV color tracking (Blue, Green, Red).
"""

from typing import Tuple, Optional, Dict, Any, List
import time
import math
import cv2
import numpy as np


# Standard Clean Color Presets (HSV ranges): Blue, Green, Red
COLOR_PRESETS = {
    "blue": {
        "name": "Blue",
        "ranges": [([100, 100, 60], [135, 255, 255])],
        "display_color": (255, 120, 0)
    },
    "green": {
        "name": "Green",
        "ranges": [([35, 70, 50], [85, 255, 255])],
        "display_color": (0, 255, 0)
    },
    "red": {
        "name": "Red",
        "ranges": [
            ([0, 100, 70], [10, 255, 255]),
            ([168, 100, 70], [180, 255, 255])
        ],
        "display_color": (0, 0, 255)
    }
}


class CameraTracker:
    def __init__(self,
                 device_index: int = 0,
                 width: int = 640,
                 height: int = 480,
                 fps: int = 30,
                 mode: str = "color",
                 aruco_dict_name: str = "DICT_4X4_50",
                 target_marker_id: int = 0,
                 color_preset: str = "blue",
                 color_min_area: int = 300):
        
        self.device_index = device_index
        self.width = width
        self.height = height
        self.fps = fps
        self.mode = mode.lower()  # 'aruco' or 'color'
        self.target_marker_id = target_marker_id
        self.color_preset_key = color_preset if color_preset in COLOR_PRESETS else "blue"
        self.color_min_area = color_min_area

        # Custom picked color (via mouse click)
        self.custom_color_hsv: Optional[Tuple[int, int, int]] = None
        self.custom_color_ranges: Optional[List[Tuple[List[int], List[int]]]] = None

        # Initialize ArUco detector
        aruco_dict_id = getattr(cv2.aruco, aruco_dict_name, cv2.aruco.DICT_4X4_50)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_id)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        self.cap: Optional[cv2.VideoCapture] = None
        self.fps_actual = 0.0
        self._fps_count = 0
        self._fps_timer = time.time()
        self.last_frame: Optional[np.ndarray] = None

    def start(self) -> bool:
        """Open camera device with automatic fallback."""
        candidates = [self.device_index]
        if 2 not in candidates:
            candidates.append(2)
        if 0 not in candidates:
            candidates.append(0)

        for dev_idx in candidates:
            self.cap = cv2.VideoCapture(dev_idx, cv2.CAP_V4L2)
            if not self.cap.isOpened():
                self.cap = cv2.VideoCapture(dev_idx)

            if self.cap.isOpened():
                ret, _ = self.cap.read()
                if ret:
                    self.device_index = dev_idx
                    self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                    self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                    self.cap.set(cv2.CAP_PROP_FPS, self.fps)
                    print(f"Connected to camera on /dev/video{dev_idx}")
                    return True
                else:
                    self.cap.release()

        return False

    def read_frame(self) -> Tuple[bool, Optional[np.ndarray]]:
        """Read fresh frame and calculate FPS."""
        if not self.cap or not self.cap.isOpened():
            return False, None

        ret, frame = self.cap.read()
        if not ret or frame is None:
            return False, None

        self.last_frame = frame
        self._fps_count += 1
        elapsed = time.time() - self._fps_timer
        if elapsed >= 1.0:
            self.fps_actual = self._fps_count / elapsed
            self._fps_count = 0
            self._fps_timer = time.time()

        return True, frame

    def pick_color_at_pixel(self, x: int, y: int):
        """
        Eyedropper tool: samples HSV at clicked coordinate (x, y)
        and configures an adaptive band for immediate tracking.
        """
        if self.last_frame is None:
            return

        h, w = self.last_frame.shape[:2]
        x = np.clip(x, 2, w - 3)
        y = np.clip(y, 2, h - 3)

        # 5x5 patch around click
        patch = self.last_frame[y - 2:y + 3, x - 2:x + 3]
        hsv_patch = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)
        med_h = int(np.median(hsv_patch[:, :, 0]))
        med_s = int(np.median(hsv_patch[:, :, 1]))
        med_v = int(np.median(hsv_patch[:, :, 2]))

        self.custom_color_hsv = (med_h, med_s, med_v)

        # Handle Hue wrap-around near 0 or 180
        h_margin = 16
        s_low = max(40, med_s - 65)
        s_high = min(255, med_s + 75)
        v_low = max(45, med_v - 70)
        v_high = min(255, med_v + 75)

        if med_h - h_margin < 0:
            self.custom_color_ranges = [
                ([0, s_low, v_low], [med_h + h_margin, s_high, v_high]),
                ([180 + (med_h - h_margin), s_low, v_low], [180, s_high, v_high])
            ]
        elif med_h + h_margin > 180:
            self.custom_color_ranges = [
                ([med_h - h_margin, s_low, v_low], [180, s_high, v_high]),
                ([0, s_low, v_low], [(med_h + h_margin) - 180, s_high, v_high])
            ]
        else:
            self.custom_color_ranges = [
                ([med_h - h_margin, s_low, v_low], [med_h + h_margin, s_high, v_high])
            ]

        self.color_preset_key = "custom"
        self.mode = "color"
        print(f"🎯 Picked Custom Color: H={med_h}, S={med_s}, V={med_v} -> Active Tracking!")

    def set_preset(self, preset_key: str):
        """Switch active color preset."""
        if preset_key in COLOR_PRESETS:
            self.color_preset_key = preset_key
            self.mode = "color"
            print(f"Switched color preset to: {COLOR_PRESETS[preset_key]['name']}")

    def track(self, frame: np.ndarray) -> Dict[str, Any]:
        """
        Track object in frame based on selected mode.
        Returns dictionary containing tracking results.
        """
        result = {
            "tracked": False,
            "center": None,
            "confidence": 0.0,
            "method": self.mode,
            "corners": None,
            "bbox": None,
            "color_name": self.color_preset_key
        }

        # 1. ArUco Marker Tracking
        if self.mode in ("aruco", "auto"):
            corners, ids, _ = self.aruco_detector.detectMarkers(frame)
            if ids is not None and len(ids) > 0:
                flat_ids = ids.ravel()
                target_idx = None
                for idx, mid in enumerate(flat_ids):
                    if mid == self.target_marker_id:
                        target_idx = idx
                        break

                if target_idx is None:
                    target_idx = 0

                marker_corners = corners[target_idx][0]
                center_u = float(np.mean(marker_corners[:, 0]))
                center_v = float(np.mean(marker_corners[:, 1]))

                x_min = int(np.min(marker_corners[:, 0]))
                y_min = int(np.min(marker_corners[:, 1]))
                x_max = int(np.max(marker_corners[:, 0]))
                y_max = int(np.max(marker_corners[:, 1]))

                result.update({
                    "tracked": True,
                    "center": (center_u, center_v),
                    "confidence": 1.0,
                    "method": f"ArUco Marker #{flat_ids[target_idx]}",
                    "corners": marker_corners,
                    "bbox": (x_min, y_min, x_max - x_min, y_max - y_min)
                })
                return result

        # 2. Multi-Color / Custom Eyedropper Tracking
        if self.mode in ("color", "auto"):
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
            # Determine active ranges
            if self.color_preset_key == "custom" and self.custom_color_ranges:
                active_ranges = self.custom_color_ranges
                preset_name = "Custom Picked Color"
            else:
                preset_info = COLOR_PRESETS.get(self.color_preset_key, COLOR_PRESETS["blue"])
                active_ranges = preset_info["ranges"]
                preset_name = preset_info["name"]

            # Combine multi-range masks (e.g. for Red or Custom wrapped hues)
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for lower_b, upper_b in active_ranges:
                sub_mask = cv2.inRange(hsv, np.array(lower_b, dtype=np.uint8), np.array(upper_b, dtype=np.uint8))
                mask = cv2.bitwise_or(mask, sub_mask)

            # Morphological noise filtering
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                largest = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(largest)
                if area >= self.color_min_area:
                    M = cv2.moments(largest)
                    if M["m00"] > 0:
                        cx = float(M["m10"] / M["m00"])
                        cy = float(M["m01"] / M["m00"])
                        x, y, w, h = cv2.boundingRect(largest)

                        result.update({
                            "tracked": True,
                            "center": (cx, cy),
                            "confidence": min(1.0, area / 2000.0),
                            "method": f"Color: {preset_name}",
                            "corners": None,
                            "bbox": (x, y, w, h),
                            "color_name": preset_name
                        })
                        return result

        return result

    def draw_hud(self,
                 frame: np.ndarray,
                 track_res: Dict[str, Any],
                 roi_bounds: Tuple[int, int, int, int],
                 sim_coords: Optional[Tuple[float, float]] = None,
                 sensitivity: float = 2.2,
                 in_safety_zone: bool = False,
                 nav_status: str = "IDLE") -> np.ndarray:
        """Draw interactive HUD overlays onto the display frame."""
        vis = frame.copy()
        h, w = vis.shape[:2]

        # 1. Draw Active Central Desk Tracking Zone
        u_min, u_max, v_min, v_max = roi_bounds
        cv2.rectangle(vis, (u_min, v_min), (u_max, v_max), (0, 220, 220), 2, cv2.LINE_AA)
        cv2.putText(vis, "DESK TRACKING ZONE (Move object inside here)", (u_min + 8, v_min - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 220, 220), 1, cv2.LINE_AA)

        # 2. Draw Target Tracking Indicators
        if track_res["tracked"] and track_res["center"]:
            cu, cv_pt = int(track_res["center"][0]), int(track_res["center"][1])

            # Draw ArUco polygon or Color bounding box
            if track_res["corners"] is not None:
                pts = track_res["corners"].astype(np.int32).reshape((-1, 1, 2))
                cv2.polylines(vis, [pts], isClosed=True, color=(0, 255, 0), thickness=2)
            elif track_res["bbox"] is not None:
                bx, by, bw, bh = track_res["bbox"]
                cv2.rectangle(vis, (bx, by), (bx + bw, by + bh), (0, 255, 0), 2)

            # Center crosshair
            cv2.drawMarker(vis, (cu, cv_pt), (0, 0, 255), cv2.MARKER_CROSS, 20, 2)
            cv2.circle(vis, (cu, cv_pt), 6, (0, 255, 0), -1)

            # Coordinates tag
            coord_str = f"Sim: X={sim_coords[0]:.2f}m, Y={sim_coords[1]:.2f}m" if sim_coords else f"Px: ({cu}, {cv_pt})"
            cv2.putText(vis, coord_str, (cu + 12, cv_pt - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 2, cv2.LINE_AA)

        # 3. Top Status Banner (Glassmorphism Dark HUD)
        cv2.rectangle(vis, (0, 0), (w, 55), (20, 20, 20), -1)
        cv2.line(vis, (0, 55), (w, 55), (60, 60, 60), 1)

        # Status indicator
        status_color = (0, 255, 0) if track_res["tracked"] else (0, 0, 255)
        cv2.circle(vis, (20, 26), 7, status_color, -1)

        method_text = track_res["method"] if track_res["tracked"] else "Searching Object..."
        cv2.putText(vis, f"TRACKER: {method_text}", (35, 32),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.58, (255, 255, 255), 2, cv2.LINE_AA)

        fps_text = f"FPS: {self.fps_actual:.1f} | Sens: {sensitivity:.1f}x"
        cv2.putText(vis, fps_text, (w - 210, 32),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 220, 220), 1, cv2.LINE_AA)

        # 4. Robot Navigation Status Bar (Bottom)
        cv2.rectangle(vis, (0, h - 50), (w, h), (25, 25, 25), -1)
        cv2.line(vis, (0, h - 50), (w, h - 50), (60, 60, 60), 1)

        nav_color = (0, 0, 255) if in_safety_zone or "HALTED" in nav_status else (0, 255, 120)
        cv2.putText(vis, f"ROBOT: {nav_status}", (20, h - 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.52, nav_color, 2, cv2.LINE_AA)

        # Helpful Shortcuts Helper on Bottom
        help_text = "Presets: [1: Blue]  [2: Green]  [3: Red]  |  A: ArUco  |  +/-: Sensitivity  |  Q: Quit"
        cv2.putText(vis, help_text, (20, h - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.40, (210, 230, 245), 1, cv2.LINE_AA)

        return vis

    def release(self):
        """Release camera hardware."""
        if self.cap and self.cap.isOpened():
            self.cap.release()
