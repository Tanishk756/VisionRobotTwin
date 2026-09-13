"""Camera capture abstraction and synthetic frame generator.

Handles physical webcam capture with configurable parameters, backend selection,
frame validation, and seamless fallback to synthetic ArUco marker animation
when physical cameras are unavailable or when running automated CI/tests.
"""

import time
import math
from typing import Optional, Tuple
import cv2
import numpy as np

from config.settings import CameraConfig, ArUcoConfig
from utils.logger import get_logger

logger = get_logger("Vision.Camera")


class SyntheticFrameGenerator:
    """Generates synthetic video frames containing animated ArUco markers for simulation & testing."""

    def __init__(
        self,
        width: int = 1280,
        height: int = 720,
        aruco_config: Optional[ArUcoConfig] = None,
    ):
        self.width = width
        self.height = height
        self.aruco_config = aruco_config or ArUcoConfig()
        self._start_time = time.time()

        # Cache generated marker images
        dict_id = getattr(cv2.aruco, self.aruco_config.dictionary_name, cv2.aruco.DICT_4X4_50)
        self.dictionary = cv2.aruco.getPredefinedDictionary(dict_id)
        self.marker_imgs = {}
        for mid in [0, 1, 2]:
            marker_px = 160
            img = np.zeros((marker_px, marker_px), dtype=np.uint8)
            cv2.aruco.generateImageMarker(self.dictionary, mid, marker_px, img, 1)
            self.marker_imgs[mid] = img

    def get_frame(self, marker_id: int = 0, state: str = "MANUAL") -> np.ndarray:
        """Renders a 3-channel background frame with perspective-warped ArUco marker."""
        t = time.time() - self._start_time
        frame = np.full((self.height, self.width, 3), 32, dtype=np.uint8)

        # Draw a textured workspace background (table grid)
        grid_size = 40
        for y in range(0, self.height, grid_size):
            cv2.line(frame, (0, y), (self.width, y), (42, 42, 48), 1)
        for x in range(0, self.width, grid_size):
            cv2.line(frame, (x, 0), (x, self.height), (42, 42, 48), 1)

        # Synthetic motion trajectory (smooth Lissajous or circular path)
        if state == "MANUAL":
            center_x = self.width / 2.0 + 220.0 * math.sin(t * 0.8)
            center_y = self.height / 2.0 + 120.0 * math.cos(t * 1.2)
            scale = 1.0 + 0.25 * math.sin(t * 0.5)  # Simulates depth (Z) movement
            angle = math.degrees(math.sin(t * 0.4) * 0.35)
            active_id = marker_id
        elif state in ("APPROACH", "PICK"):
            # Move towards pick target (ID 1)
            center_x = self.width / 2.0 - 180.0
            center_y = self.height / 2.0 + 60.0
            scale = 1.1
            angle = 0.0
            active_id = 1
        elif state in ("MOVE_TO_PLACE", "PLACE"):
            # Move towards place target (ID 2)
            center_x = self.width / 2.0 + 180.0
            center_y = self.height / 2.0 + 60.0
            scale = 1.1
            angle = 0.0
            active_id = 2
        else:
            center_x = self.width / 2.0
            center_y = self.height / 2.0
            scale = 1.0
            angle = 0.0
            active_id = marker_id

        # Overlay active marker
        marker_img = self.marker_imgs.get(active_id, self.marker_imgs[0])
        m_h, m_w = marker_img.shape
        scaled_w = int(m_w * scale)
        scaled_h = int(m_h * scale)
        resized_m = cv2.resize(marker_img, (scaled_w, scaled_h), interpolation=cv2.INTER_NEAREST)

        # Apply rotation and place on frame
        rot_mat = cv2.getRotationMatrix2D((scaled_w / 2.0, scaled_h / 2.0), angle, 1.0)
        rotated_m = cv2.warpAffine(
            resized_m,
            rot_mat,
            (scaled_w, scaled_h),
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=255,
        )

        # Paste into canvas
        x1 = int(center_x - scaled_w / 2)
        y1 = int(center_y - scaled_h / 2)
        x2 = x1 + scaled_w
        y2 = y1 + scaled_h

        if x1 >= 0 and y1 >= 0 and x2 < self.width and y2 < self.height:
            # White border around marker
            border_pad = 12
            cv2.rectangle(
                frame,
                (x1 - border_pad, y1 - border_pad),
                (x2 + border_pad, y2 + border_pad),
                (255, 255, 255),
                -1,
            )
            frame[y1:y2, x1:x2] = cv2.cvtColor(rotated_m, cv2.COLOR_GRAY2BGR)

        # Synthetic mode banner
        cv2.putText(
            frame,
            "[SYNTHETIC CAMERA STREAM - LIVE SIMULATION]",
            (self.width // 2 - 210, self.height - 55),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.50,
            (100, 200, 255),
            1,
            cv2.LINE_AA,
        )
        return frame


class Camera:
    """Robust camera interface with automatic physical capture and synthetic fallback."""

    def __init__(self, config: CameraConfig, aruco_config: Optional[ArUcoConfig] = None):
        self.config = config
        self.aruco_config = aruco_config or ArUcoConfig()
        self.cap: Optional[cv2.VideoCapture] = None
        self.is_synthetic = config.synthetic_mode
        self._synthetic_gen: Optional[SyntheticFrameGenerator] = None

        if not self.is_synthetic:
            self._open_physical_camera()

        if self.cap is None or not self.cap.isOpened():
            logger.warning(
                f"Webcam at index {self.config.camera_index} is unavailable. "
                "Switching automatically to Synthetic Frame Generator for real-time simulation."
            )
            self.is_synthetic = True
            self._synthetic_gen = SyntheticFrameGenerator(
                width=self.config.width,
                height=self.config.height,
                aruco_config=self.aruco_config,
            )

    def _open_physical_camera(self) -> None:
        """Attempts to open physical webcam device using DirectShow or MSMF backends."""
        backends = [cv2.CAP_DSHOW, cv2.CAP_MSMF, cv2.CAP_ANY]
        for backend in backends:
            try:
                cap = cv2.VideoCapture(self.config.camera_index, backend)
                if cap.isOpened():
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.config.width)
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config.height)
                    cap.set(cv2.CAP_PROP_FPS, self.config.fps)
                    cap.set(cv2.CAP_PROP_BUFFERSIZE, self.config.buffer_size)

                    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                    logger.info(
                        f"Physical webcam opened successfully (Index: {self.config.camera_index}, "
                        f"Backend: {backend}, Resolution: {actual_w}x{actual_h})"
                    )
                    self.cap = cap
                    return
            except Exception as e:
                logger.debug(f"Backend {backend} failed: {e}")

    def read(self, active_marker_id: int = 0, state: str = "MANUAL") -> Tuple[bool, Optional[np.ndarray]]:
        """Captures a new frame or generates a synthetic one."""
        if self.is_synthetic:
            if self._synthetic_gen is None:
                self._synthetic_gen = SyntheticFrameGenerator(
                    width=self.config.width,
                    height=self.config.height,
                    aruco_config=self.aruco_config,
                )
            frame = self._synthetic_gen.get_frame(marker_id=active_marker_id, state=state)
            return True, frame

        if self.cap is not None and self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret and frame is not None and frame.size > 0:
                return True, frame

            logger.warning("Frame capture failed from physical camera. Falling back to synthetic generator.")
            self.is_synthetic = True
            if self._synthetic_gen is None:
                self._synthetic_gen = SyntheticFrameGenerator(
                    width=self.config.width,
                    height=self.config.height,
                    aruco_config=self.aruco_config,
                )
            return True, self._synthetic_gen.get_frame(marker_id=active_marker_id, state=state)

        return False, None

    def release(self) -> None:
        """Safely releases the camera hardware."""
        if self.cap is not None:
            try:
                self.cap.release()
                logger.info("Physical camera stream released.")
            except Exception as e:
                logger.warning(f"Error releasing camera: {e}")
            self.cap = None
