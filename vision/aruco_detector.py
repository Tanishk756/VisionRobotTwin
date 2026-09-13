"""ArUco Marker Detection Module.

Utilizes modern OpenCV ArUco detector API (cv2.aruco.ArucoDetector) to reliably detect
and locate square fiducial markers in RGB/BGR frames.
"""

from dataclasses import dataclass
from typing import List, Optional, Tuple
import cv2
import numpy as np

from config.settings import ArUcoConfig
from utils.logger import get_logger

logger = get_logger("Vision.ArUcoDetector")


@dataclass
class MarkerDetection:
    """Represents a detected ArUco marker."""
    id: int
    corners: np.ndarray  # Shape: (4, 2) in image pixel coordinates
    center: Tuple[float, float]  # (cx, cy) in pixels
    area: float


class ArUcoDetector:
    """Encapsulates OpenCV ArUco dictionary, parameters, and detection pipeline."""

    def __init__(self, config: ArUcoConfig):
        self.config = config

        # Retrieve predefined ArUco dictionary
        dict_attr = getattr(cv2.aruco, config.dictionary_name, None)
        if dict_attr is None:
            logger.warning(
                f"Dictionary '{config.dictionary_name}' not found. Defaulting to DICT_4X4_50."
            )
            dict_attr = cv2.aruco.DICT_4X4_50

        self.dictionary = cv2.aruco.getPredefinedDictionary(dict_attr)
        self.detector_params = cv2.aruco.DetectorParameters()
        self.detector_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

        # Initialize detector instance
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.detector_params)
        logger.info(f"Initialized ArUco Detector with dictionary '{config.dictionary_name}'")

    def detect(self, frame: np.ndarray) -> List[MarkerDetection]:
        """Detects all ArUco markers in the supplied frame.

        Args:
            frame: Input BGR or grayscale image frame.

        Returns:
            List of MarkerDetection objects sorted by marker ID.
        """
        if frame is None or frame.size == 0:
            return []

        # Convert to grayscale if BGR
        if len(frame.shape) == 3:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        else:
            gray = frame

        corners_list, ids, _ = self.detector.detectMarkers(gray)

        detections: List[MarkerDetection] = []
        if ids is not None and len(ids) > 0:
            flat_ids = np.asarray(ids).ravel()
            for idx, raw_id in enumerate(flat_ids):
                marker_id = int(raw_id)
                raw_corners = corners_list[idx].reshape((4, 2))
                cx = float(np.mean(raw_corners[:, 0]))
                cy = float(np.mean(raw_corners[:, 1]))
                area = float(cv2.contourArea(raw_corners))

                detections.append(
                    MarkerDetection(
                        id=marker_id,
                        corners=raw_corners,
                        center=(cx, cy),
                        area=area,
                    )
                )

        detections.sort(key=lambda d: d.id)
        return detections

    def draw_detections(
        self,
        frame: np.ndarray,
        detections: List[MarkerDetection],
        highlight_id: Optional[int] = None,
    ) -> np.ndarray:
        """Draws bounding boxes, IDs, and center points for detected markers."""
        for d in detections:
            pts = d.corners.astype(np.int32)
            is_target = (highlight_id is not None and d.id == highlight_id)
            color = (0, 255, 0) if is_target else (255, 180, 0)
            thickness = 3 if is_target else 2

            # Draw polygon perimeter
            cv2.polylines(frame, [pts], isClosed=True, color=color, thickness=thickness, lineType=cv2.LINE_AA)

            # Draw center point
            cx_int, cy_int = int(d.center[0]), int(d.center[1])
            cv2.circle(frame, (cx_int, cy_int), 5, (0, 0, 255), -1, cv2.LINE_AA)

            # Draw ID label with backdrop
            label = f"ID: {d.id}"
            (text_w, text_h), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
            top_left = (int(d.corners[0, 0]), int(d.corners[0, 1]) - 10)
            bg_pt1 = (top_left[0], top_left[1] - text_h - 4)
            bg_pt2 = (top_left[0] + text_w + 4, top_left[1] + baseline)

            cv2.rectangle(frame, bg_pt1, bg_pt2, (20, 20, 20), -1)
            cv2.putText(
                frame,
                label,
                (top_left[0] + 2, top_left[1]),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 255) if is_target else (255, 255, 255),
                2,
                cv2.LINE_AA,
            )

        return frame
