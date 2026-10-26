"""
Physical Camera Interface using OpenCV.
Captures live timestamped video frames with non-blocking threaded acquisition.
"""

import cv2
import time
import threading
import logging
from typing import Optional, Tuple
import numpy as np

logger = logging.getLogger("CameraInterface")


class CameraInterface:
    def __init__(self,
                 device_index: int = 0,
                 width: int = 640,
                 height: int = 480,
                 fps: int = 30):
        self.device_index = device_index
        self.width = width
        self.height = height
        self.target_fps = fps

        self.cap: Optional[cv2.VideoCapture] = None
        self._is_running: bool = False
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()

        # State
        self.current_frame: Optional[np.ndarray] = None
        self.last_frame_timestamp: float = 0.0
        self.is_connected: bool = False
        self.fps_actual: float = 0.0
        self._frame_count: int = 0
        self._fps_start_time: float = 0.0

    def start(self) -> bool:
        """Initialize camera and begin background frame grabbing thread."""
        logger.info(f"Opening camera index {self.device_index} ({self.width}x{self.height} @ {self.target_fps}fps)...")
        self.cap = cv2.VideoCapture(self.device_index, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            logger.warning(f"V4L2 backend failed, attempting default backend for camera {self.device_index}...")
            self.cap = cv2.VideoCapture(self.device_index)

        if not self.cap.isOpened():
            logger.error(f"Failed to open camera device {self.device_index}")
            self.is_connected = False
            return False

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.target_fps)

        # Verify initial frame capture
        ret, frame = self.cap.read()
        if not ret or frame is None:
            logger.error(f"Camera opened but failed to read initial frame.")
            self.cap.release()
            self.is_connected = False
            return False

        with self._lock:
            self.current_frame = frame
            self.last_frame_timestamp = time.time()
            self.is_connected = True

        self._is_running = True
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()
        logger.info(f"Camera interface started successfully. Native resolution: {frame.shape[1]}x{frame.shape[0]}")
        return True

    def _capture_loop(self):
        """Dedicated thread to continuously grab latest frame and prevent buffer lag."""
        self._fps_start_time = time.time()
        self._frame_count = 0

        while self._is_running:
            if self.cap is None or not self.cap.isOpened():
                time.sleep(0.1)
                continue

            ret, frame = self.cap.read()
            now = time.time()

            if ret and frame is not None:
                with self._lock:
                    self.current_frame = frame
                    self.last_frame_timestamp = now
                    self.is_connected = True

                self._frame_count += 1
                elapsed = now - self._fps_start_time
                if elapsed >= 2.0:
                    self.fps_actual = self._frame_count / elapsed
                    self._frame_count = 0
                    self._fps_start_time = now
            else:
                with self._lock:
                    self.is_connected = False
                logger.warning("Camera frame read dropped.")
                time.sleep(0.05)

    def get_latest_frame(self) -> Tuple[Optional[np.ndarray], float, bool]:
        """
        Returns:
            (frame_bgr, timestamp, is_valid)
        """
        with self._lock:
            if self.current_frame is not None:
                # Check for staleness (> 1.0 second without update)
                stale = (time.time() - self.last_frame_timestamp) > 1.0
                return self.current_frame.copy(), self.last_frame_timestamp, (self.is_connected and not stale)
            return None, 0.0, False

    def stop(self):
        """Release camera and terminate acquisition thread."""
        self._is_running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        if self.cap and self.cap.isOpened():
            self.cap.release()
        self.is_connected = False
        logger.info("Camera interface stopped.")
