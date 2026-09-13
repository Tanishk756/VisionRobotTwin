"""Live Visual Telemetry and Heads-Up Display (HUD) for OpenCV Window.

Renders high-contrast, structured telemetry panels presenting perception,
robotics, kinematics, and safety metrics clearly without cluttering the main view.
"""

from dataclasses import dataclass
from typing import Optional, Tuple
import cv2
import numpy as np


@dataclass
class TelemetryData:
    """Snapshot of current system state for telemetry rendering."""
    mode: str = "MANUAL"
    state: str = "HOME"
    marker_id: Optional[int] = None
    tracking_active: bool = False
    raw_pos_cam: Optional[Tuple[float, float, float]] = None
    filtered_pos_cam: Optional[Tuple[float, float, float]] = None
    marker_distance_m: Optional[float] = None
    robot_target_pos: Optional[Tuple[float, float, float]] = None
    robot_ee_pos: Optional[Tuple[float, float, float]] = None
    tracking_error_m: Optional[float] = None
    ik_status: str = "IDLE"
    fps: float = 0.0
    sim_fps: float = 0.0
    physics_target_hz: float = 240.0
    physics_substeps: int = 1
    physics_actual_step_rate: float = 240.0
    lost_tracking_time_s: float = 0.0
    is_calibrated: bool = False
    is_calibrated_extrinsics: bool = False
    transform_mode: str = "relative"
    workspace_clamped: bool = False
    debug_mode: bool = False


class TelemetryOverlay:
    """Draws sleek, semi-transparent telemetry panels on video frames."""

    # Color Palette (BGR)
    COLOR_BG = (20, 20, 24)
    COLOR_BORDER = (55, 65, 81)
    COLOR_ACCENT = (245, 158, 11)   # Amber
    COLOR_CYAN = (230, 180, 50)
    COLOR_TEXT = (240, 240, 245)
    COLOR_MUTED = (160, 160, 175)
    COLOR_GREEN = (46, 204, 113)
    COLOR_RED = (60, 70, 235)
    COLOR_YELLOW = (40, 200, 240)
    COLOR_BLUE = (220, 140, 30)

    def __init__(self, show_help: bool = True):
        self.show_help = show_help

    def _draw_panel(
        self,
        img: np.ndarray,
        x: int,
        y: int,
        w: int,
        h: int,
        alpha: float = 0.75,
        border_color: Optional[Tuple[int, int, int]] = None,
    ) -> None:
        """Draws a rounded semi-transparent dark rectangle."""
        overlay = img.copy()
        cv2.rectangle(overlay, (x, y), (x + w, y + h), self.COLOR_BG, -1)
        cv2.addWeighted(overlay, alpha, img, 1.0 - alpha, 0, img)
        border = border_color if border_color is not None else self.COLOR_BORDER
        cv2.rectangle(img, (x, y), (x + w, y + h), border, 1, cv2.LINE_AA)

    def render(self, frame: np.ndarray, data: TelemetryData) -> np.ndarray:
        """Draws complete HUD overlay on frame in-place."""
        h, w = frame.shape[:2]

        # Top Bar: Title, Mode, State, FPS
        self._draw_header(frame, data, w)

        # Left Panel: Perception & Vision Tracking
        self._draw_vision_panel(frame, data, 15, 65)

        # Right Panel: Robotics & Kinematics
        self._draw_robot_panel(frame, data, w - 325, 65)

        # Bottom Bar: Controls & Shortcuts
        if self.show_help:
            self._draw_bottom_bar(frame, data, w, h)

        return frame

    def _draw_header(self, frame: np.ndarray, data: TelemetryData, width: int) -> None:
        self._draw_panel(frame, 15, 10, width - 30, 42, alpha=0.85)

        # App Title
        cv2.putText(
            frame,
            "VISION ROBOT TWIN",
            (30, 38),
            cv2.FONT_HERSHEY_DUPLEX,
            0.65,
            self.COLOR_ACCENT,
            1,
            cv2.LINE_AA,
        )

        # Mode Badge
        mode_text = f"MODE: {data.mode}"
        mode_color = self.COLOR_CYAN if data.mode == "MANUAL" else self.COLOR_GREEN
        cv2.putText(
            frame,
            mode_text,
            (240, 37),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.50,
            mode_color,
            1,
            cv2.LINE_AA,
        )

        # State Badge
        state_color = self.COLOR_GREEN if data.state in ("TRACK", "PICK", "PLACE") else (
            self.COLOR_YELLOW if data.state in ("HOLD", "APPROACH", "LIFT") else self.COLOR_MUTED
        )
        if data.state == "ERROR":
            state_color = self.COLOR_RED

        cv2.putText(
            frame,
            f"STATE: {data.state}",
            (410, 37),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.50,
            state_color,
            1,
            cv2.LINE_AA,
        )

        # FPS / Physics Stats
        fps_text = f"CTRL: {data.fps:.1f} FPS | SIM: {data.physics_actual_step_rate:.0f} Hz ({data.physics_substeps} steps/f)"
        cv2.putText(
            frame,
            fps_text,
            (width - 320, 37),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.44,
            self.COLOR_TEXT,
            1,
            cv2.LINE_AA,
        )

    def _draw_vision_panel(self, frame: np.ndarray, data: TelemetryData, x: int, y: int) -> None:
        pw, ph = 295, 235
        border = self.COLOR_GREEN if data.tracking_active else self.COLOR_BORDER
        self._draw_panel(frame, x, y, pw, ph, alpha=0.80, border_color=border)

        cv2.putText(
            frame,
            "PERCEPTION (CAMERA FRAME)",
            (x + 12, y + 22),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            self.COLOR_ACCENT,
            1,
            cv2.LINE_AA,
        )

        # Marker Status
        marker_str = f"ID: {data.marker_id}" if data.marker_id is not None else "ID: None"
        status_str = "TRACKING" if data.tracking_active else (
            f"LOST ({data.lost_tracking_time_s:.1f}s)" if data.lost_tracking_time_s > 0 else "NO TARGET"
        )
        status_col = self.COLOR_GREEN if data.tracking_active else self.COLOR_RED

        cv2.putText(frame, marker_str, (x + 12, y + 48), cv2.FONT_HERSHEY_SIMPLEX, 0.45, self.COLOR_TEXT, 1, cv2.LINE_AA)
        cv2.putText(frame, f"Status: {status_str}", (x + 110, y + 48), cv2.FONT_HERSHEY_SIMPLEX, 0.45, status_col, 1, cv2.LINE_AA)

        # Raw Pose
        if data.raw_pos_cam:
            rx, ry, rz = data.raw_pos_cam
            raw_str = f"Raw:    [{rx:+6.3f}, {ry:+6.3f}, {rz:+6.3f}] m"
        else:
            raw_str = "Raw:    [  --- ,   --- ,   --- ] m"
        cv2.putText(frame, raw_str, (x + 12, y + 74), cv2.FONT_HERSHEY_SIMPLEX, 0.42, self.COLOR_MUTED, 1, cv2.LINE_AA)

        # Filtered Pose
        if data.filtered_pos_cam:
            fx, fy, fz = data.filtered_pos_cam
            filt_str = f"Filt:   [{fx:+6.3f}, {fy:+6.3f}, {fz:+6.3f}] m"
        else:
            filt_str = "Filt:   [  --- ,   --- ,   --- ] m"
        cv2.putText(frame, filt_str, (x + 12, y + 98), cv2.FONT_HERSHEY_SIMPLEX, 0.42, self.COLOR_CYAN, 1, cv2.LINE_AA)

        # Distance
        dist_str = f"Distance: {data.marker_distance_m * 100:.1f} cm" if data.marker_distance_m is not None else "Distance: ---"
        cv2.putText(frame, dist_str, (x + 12, y + 122), cv2.FONT_HERSHEY_SIMPLEX, 0.42, self.COLOR_TEXT, 1, cv2.LINE_AA)

        # Intrinsics status
        cal_str = "INTRINSICS: CALIBRATED" if data.is_calibrated else "INTRINSICS: FALLBACK PINHOLE"
        cal_col = self.COLOR_GREEN if data.is_calibrated else self.COLOR_YELLOW
        cv2.putText(frame, cal_str, (x + 12, y + 148), cv2.FONT_HERSHEY_SIMPLEX, 0.40, cal_col, 1, cv2.LINE_AA)

        # Extrinsics status
        if data.transform_mode == "se3":
            ext_str = "EXTRINSICS: CALIBRATED" if data.is_calibrated_extrinsics else "EXTRINSICS: NOMINAL"
            ext_col = self.COLOR_GREEN if data.is_calibrated_extrinsics else self.COLOR_YELLOW
        else:
            ext_str = "TRANSFORM: RELATIVE TELEOP"
            ext_col = self.COLOR_CYAN
        cv2.putText(frame, ext_str, (x + 12, y + 172), cv2.FONT_HERSHEY_SIMPLEX, 0.40, ext_col, 1, cv2.LINE_AA)

        # Guidance note
        if not data.is_calibrated:
            cv2.putText(frame, "Run tools/calibrate_camera.py", (x + 12, y + 200), cv2.FONT_HERSHEY_SIMPLEX, 0.36, self.COLOR_MUTED, 1, cv2.LINE_AA)
        elif data.transform_mode == "se3" and not data.is_calibrated_extrinsics:
            cv2.putText(frame, "Run tools/calibrate_extrinsics.py", (x + 12, y + 200), cv2.FONT_HERSHEY_SIMPLEX, 0.36, self.COLOR_MUTED, 1, cv2.LINE_AA)

    def _draw_robot_panel(self, frame: np.ndarray, data: TelemetryData, x: int, y: int) -> None:
        pw, ph = 310, 235
        self._draw_panel(frame, x, y, pw, ph, alpha=0.80)

        cv2.putText(
            frame,
            "ROBOT DIGITAL TWIN (BASE FRAME)",
            (x + 12, y + 22),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            self.COLOR_ACCENT,
            1,
            cv2.LINE_AA,
        )

        # IK Status Badge
        ik_color = self.COLOR_GREEN if data.ik_status in ("OK", "SOLUTION_RETURNED") else (
            self.COLOR_YELLOW if data.ik_status == "IDLE" else self.COLOR_RED
        )
        cv2.putText(frame, f"IK Status: {data.ik_status}", (x + 12, y + 48), cv2.FONT_HERSHEY_SIMPLEX, 0.45, ik_color, 1, cv2.LINE_AA)

        # Clamped Alert
        if data.workspace_clamped:
            cv2.putText(frame, "[WORKSPACE LIMIT]", (x + 160, y + 48), cv2.FONT_HERSHEY_SIMPLEX, 0.42, self.COLOR_YELLOW, 1, cv2.LINE_AA)

        # Target Pose
        if data.robot_target_pos:
            tx, ty, tz = data.robot_target_pos
            tgt_str = f"Target: [{tx:+6.3f}, {ty:+6.3f}, {tz:+6.3f}] m"
        else:
            tgt_str = "Target: [  --- ,   --- ,   --- ] m"
        cv2.putText(frame, tgt_str, (x + 12, y + 78), cv2.FONT_HERSHEY_SIMPLEX, 0.42, self.COLOR_CYAN, 1, cv2.LINE_AA)

        # Actual End-Effector Pose
        if data.robot_ee_pos:
            ex, ey, ez = data.robot_ee_pos
            ee_str = f"EE Pos: [{ex:+6.3f}, {ey:+6.3f}, {ez:+6.3f}] m"
        else:
            ee_str = "EE Pos: [  --- ,   --- ,   --- ] m"
        cv2.putText(frame, ee_str, (x + 12, y + 104), cv2.FONT_HERSHEY_SIMPLEX, 0.42, self.COLOR_GREEN, 1, cv2.LINE_AA)

        # Tracking Error (Cartesian Norm)
        if data.tracking_error_m is not None:
            err_mm = data.tracking_error_m * 1000.0
            err_col = self.COLOR_GREEN if err_mm < 25.0 else (self.COLOR_YELLOW if err_mm < 60.0 else self.COLOR_RED)
            err_str = f"Pos Error: {err_mm:5.1f} mm"
        else:
            err_str = "Pos Error:  --- mm"
            err_col = self.COLOR_MUTED
        cv2.putText(frame, err_str, (x + 12, y + 130), cv2.FONT_HERSHEY_SIMPLEX, 0.44, err_col, 1, cv2.LINE_AA)

        # Manipulator Info
        cv2.putText(frame, "Model: Franka Emika Panda (7-DoF)", (x + 12, y + 158), cv2.FONT_HERSHEY_SIMPLEX, 0.40, self.COLOR_TEXT, 1, cv2.LINE_AA)
        cv2.putText(frame, "Control: Joint Velocity / Position", (x + 12, y + 180), cv2.FONT_HERSHEY_SIMPLEX, 0.40, self.COLOR_MUTED, 1, cv2.LINE_AA)
        cv2.putText(frame, "Solver: Damped Least-Squares IK", (x + 12, y + 202), cv2.FONT_HERSHEY_SIMPLEX, 0.40, self.COLOR_MUTED, 1, cv2.LINE_AA)

    def _draw_bottom_bar(self, frame: np.ndarray, data: TelemetryData, width: int, height: int) -> None:
        self._draw_panel(frame, 15, height - 38, width - 30, 28, alpha=0.85)
        help_text = (
            "[Q] Quit   [H] Home   [SPACE] Hold/Resume   [M] Manual   [A] Auto   "
            "[R] Reset   [T] Trajectory   [S] Screenshot   [D] Debug"
        )
        cv2.putText(
            frame,
            help_text,
            (25, height - 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.38,
            self.COLOR_TEXT,
            1,
            cv2.LINE_AA,
        )
