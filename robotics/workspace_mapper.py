"""Workspace Mapping, Coordinate Transformation Pipeline, and Cartesian Safety Layer.

Translates optical perception coordinates into safe Franka Panda Cartesian targets.
Supports:
- Explicit SE(3) transformation mode (T_base_marker = T_base_camera @ T_camera_marker)
- Relative intuitive teleoperation mapping mode
- Time-based Cartesian and Angular slew-rate limiting (independent of frame rate)
- Strict workspace boundary clamping and NaN/Inf validation
"""

from dataclasses import dataclass
from typing import Optional, Tuple, Union
import time
import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from config.settings import WorkspaceConfig, TransformConfig
from robotics.coordinate_transform import (
    SE3Transform,
    create_homogeneous_matrix,
    multiply_quaternions,
    compute_angular_distance,
)
from utils.logger import get_logger

logger = get_logger("Robotics.WorkspaceMapper")


@dataclass
class MappedTarget:
    """Represents the validated, clamped Cartesian and Orientation robot target."""
    position: np.ndarray             # (3,) [x, y, z] target in robot base frame (m)
    orientation: np.ndarray          # (4,) [x, y, z, w] target unit quaternion
    raw_mapped_pos: np.ndarray       # Position before boundary clamping (m)
    is_clamped: bool                 # True if target was bounded by workspace limits
    is_valid: bool                   # True if mathematically valid and safe
    error_message: Optional[str] = None

    @property
    def x(self) -> float:
        return float(self.position[0])

    @property
    def y(self) -> float:
        return float(self.position[1])

    @property
    def z(self) -> float:
        return float(self.position[2])


class WorkspaceMapper:
    """Manages rigid frame transformations, workspace scaling, and velocity limiting."""

    def __init__(self, ws_config: WorkspaceConfig, tf_config: Optional[TransformConfig] = None):
        self.ws_config = ws_config
        self.tf_config = tf_config or TransformConfig()

        # Build T_base_camera from nominal/calibrated extrinsics
        self._T_base_camera = create_homogeneous_matrix(
            rotation=self.tf_config.camera_euler_rpy_rad,
            translation=self.tf_config.camera_position_in_robot_base,
        )

        self._last_commanded_position: Optional[np.ndarray] = None
        self._last_commanded_orientation: Optional[np.ndarray] = None
        self._last_update_time: Optional[float] = None

    def map_camera_to_robot(
        self,
        camera_pos: np.ndarray,
        camera_quat_xyzw: Optional[np.ndarray] = None,
        enforce_slew_rate: bool = True,
        timestamp: Optional[float] = None,
    ) -> MappedTarget:
        """Transforms camera-frame 6-DoF pose into robot base Cartesian workspace.

        Args:
            camera_pos: 3D point in camera frame [x_cam, y_cam, z_cam] (meters).
            camera_quat_xyzw: Optional marker orientation quaternion [x, y, z, w].
            enforce_slew_rate: If True, applies time-based velocity limits.
            timestamp: Current monotonic time in seconds.

        Returns:
            MappedTarget with bounded position and orientation.
        """
        pos = np.asarray(camera_pos, dtype=np.float64).flatten()
        now = timestamp if timestamp is not None else time.time()

        # Validate finite values
        if not np.all(np.isfinite(pos)) or len(pos) != 3:
            return MappedTarget(
                position=self._get_fallback_position(),
                orientation=self.tf_config.tool_orientation_offset,
                raw_mapped_pos=pos if len(pos) == 3 else np.zeros(3),
                is_clamped=False,
                is_valid=False,
                error_message="Invalid camera position containing NaN or Inf",
            )

        # 1. Transform Position
        if self.tf_config.transform_mode == "se3":
            # Direct rigid transformation: p_base = R_base_cam @ p_cam + t_base_cam
            R_base_cam = self._T_base_camera[:3, :3]
            t_base_cam = self._T_base_camera[:3, 3]
            raw_mapped_pos = R_base_cam @ pos + t_base_cam
        else:
            # Intuitive teleoperation mapping
            dx_cam = pos[0] - self.ws_config.cam_center_x
            dy_cam = pos[1] - self.ws_config.cam_center_y
            dz_cam = pos[2] - self.ws_config.cam_center_z

            mapped_x = self.ws_config.robot_center_x + (dz_cam * self.ws_config.scale_z)
            mapped_y = self.ws_config.robot_center_y - (dx_cam * self.ws_config.scale_x)
            mapped_z = self.ws_config.robot_center_z - (dy_cam * self.ws_config.scale_y)
            raw_mapped_pos = np.array([mapped_x, mapped_y, mapped_z], dtype=np.float64)

        # 2. Transform Orientation
        if camera_quat_xyzw is not None and np.all(np.isfinite(camera_quat_xyzw)):
            q_norm = np.linalg.norm(camera_quat_xyzw)
            if q_norm > 1e-6:
                q_cam_marker = np.asarray(camera_quat_xyzw) / q_norm
                if self.tf_config.transform_mode == "se3":
                    R_base_cam = self._T_base_camera[:3, :3]
                    q_base_cam = Rotation.from_matrix(R_base_cam).as_quat()
                    q_base_marker = multiply_quaternions(q_base_cam, q_cam_marker)
                    target_orn = multiply_quaternions(q_base_marker, self.tf_config.tool_orientation_offset)
                else:
                    # In relative mode, apply orientation delta around default downward orientation
                    target_orn = multiply_quaternions(q_cam_marker, self.tf_config.tool_orientation_offset)
            else:
                target_orn = np.asarray(self.tf_config.tool_orientation_offset, dtype=np.float64)
        else:
            target_orn = np.asarray(self.tf_config.tool_orientation_offset, dtype=np.float64)

        # 3. Enforce Cartesian Workspace Bounds
        clamped_x = np.clip(raw_mapped_pos[0], self.ws_config.x_min, self.ws_config.x_max)
        clamped_y = np.clip(raw_mapped_pos[1], self.ws_config.y_min, self.ws_config.y_max)
        clamped_z = np.clip(raw_mapped_pos[2], self.ws_config.z_min, self.ws_config.z_max)

        clamped_pos = np.array([clamped_x, clamped_y, clamped_z], dtype=np.float64)
        was_clamped = not np.allclose(raw_mapped_pos, clamped_pos, atol=1e-4)

        # 4. Enforce Time-Based Slew-Rate Velocity Limiting
        final_pos = clamped_pos.copy()
        final_orn = target_orn.copy()

        if enforce_slew_rate and self._last_commanded_position is not None:
            dt = now - self._last_update_time if self._last_update_time is not None else 0.033
            dt = np.clip(dt, 0.001, 0.20)  # Bound time step

            # Position Slew Limiter (m/s)
            max_pos_step = min(
                self.ws_config.max_cartesian_velocity_mps * dt,
                self.ws_config.max_cartesian_step_m,
            )
            pos_delta = clamped_pos - self._last_commanded_position
            pos_dist = np.linalg.norm(pos_delta)

            if pos_dist > max_pos_step and pos_dist > 1e-6:
                final_pos = self._last_commanded_position + (pos_delta / pos_dist) * max_pos_step

            # Orientation Slew Limiter (rad/s via SLERP)
            if self._last_commanded_orientation is not None:
                max_ang_step = self.ws_config.max_angular_velocity_radps * dt
                ang_dist = compute_angular_distance(self._last_commanded_orientation, target_orn)
                if ang_dist > max_ang_step and ang_dist > 1e-4:
                    fraction = max_ang_step / ang_dist
                    key_rots = Rotation.from_quat([self._last_commanded_orientation, target_orn])
                    slerp = Slerp([0.0, 1.0], key_rots)
                    final_orn = slerp([fraction]).as_quat()[0]

        self._last_commanded_position = final_pos.copy()
        self._last_commanded_orientation = final_orn.copy()
        self._last_update_time = now

        return MappedTarget(
            position=final_pos,
            orientation=final_orn,
            raw_mapped_pos=raw_mapped_pos,
            is_clamped=was_clamped,
            is_valid=True,
            error_message=None,
        )

    def _get_fallback_position(self) -> np.ndarray:
        if self._last_commanded_position is not None:
            return self._last_commanded_position.copy()
        return np.array([
            self.ws_config.robot_center_x,
            self.ws_config.robot_center_y,
            self.ws_config.robot_center_z,
        ], dtype=np.float64)

    def reset(self) -> None:
        self._last_commanded_position = None
        self._last_commanded_orientation = None
        self._last_update_time = None
