"""Workspace Mapping and Cartesian Safety Layer.

Maps physical camera-tracked motion into the reachable Cartesian workspace volume
of the Franka Emika Panda manipulator. Handles:
- Intuitive natural coordinate mapping (Camera to Robot axes)
- Workspace boundary clamping
- NaN/Inf protection and invalid pose rejection
- Slew-rate velocity limiting to eliminate sudden jumps
"""

from dataclasses import dataclass
from typing import Optional, Tuple
import numpy as np

from config.settings import WorkspaceConfig
from utils.logger import get_logger

logger = get_logger("Robotics.WorkspaceMapper")


@dataclass
class MappedTarget:
    """Represents the validated, clamped Cartesian robot target."""
    position: np.ndarray        # (3,) [x, y, z] target in robot base frame (m)
    raw_mapped_pos: np.ndarray  # Position before boundary clamping (m)
    is_clamped: bool            # True if target was bounded by workspace limits
    is_valid: bool              # True if mathematically valid and safe
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
    """Translates filtered camera 3D poses into safe Franka Panda Cartesian targets."""

    def __init__(self, config: WorkspaceConfig):
        self.config = config
        self._last_commanded_position: Optional[np.ndarray] = None

    def map_camera_to_robot(
        self,
        camera_pos: np.ndarray,
        enforce_slew_rate: bool = True,
    ) -> MappedTarget:
        """Transforms camera-frame 3D coordinates into robot base Cartesian workspace.

        Mapping convention:
          - Camera +X (right)   -> Robot +Y (left/right motion)
          - Camera -Y (up)      -> Robot +Z (vertical height)
          - Camera +Z (away)    -> Robot +X (reach depth)

        Args:
            camera_pos: 3D point in camera frame [x_cam, y_cam, z_cam] (meters).
            enforce_slew_rate: If True, limits maximum displacement from previous step.

        Returns:
            MappedTarget containing bounded, rate-limited robot target.
        """
        pos = np.asarray(camera_pos, dtype=np.float64).flatten()

        # Check for NaN / Inf
        if not np.all(np.isfinite(pos)) or len(pos) != 3:
            return MappedTarget(
                position=self._get_fallback_position(),
                raw_mapped_pos=pos if len(pos) == 3 else np.zeros(3),
                is_clamped=False,
                is_valid=False,
                error_message="Invalid camera position containing NaN or Inf",
            )

        # Compute delta from camera interaction center
        dx_cam = pos[0] - self.config.cam_center_x
        dy_cam = pos[1] - self.config.cam_center_y
        dz_cam = pos[2] - self.config.cam_center_z

        # Apply intuitive axis mapping and scaling
        mapped_x = self.config.robot_center_x + (dz_cam * self.config.scale_z)
        mapped_y = self.config.robot_center_y - (dx_cam * self.config.scale_x)  # Inverted for mirror intuition
        mapped_z = self.config.robot_center_z - (dy_cam * self.config.scale_y)  # Inverted because Cam -Y is up

        raw_mapped = np.array([mapped_x, mapped_y, mapped_z], dtype=np.float64)

        # Enforce Workspace Cartesian Bounds
        clamped_x = np.clip(mapped_x, self.config.x_min, self.config.x_max)
        clamped_y = np.clip(mapped_y, self.config.y_min, self.config.y_max)
        clamped_z = np.clip(mapped_z, self.config.z_min, self.config.z_max)

        clamped_pos = np.array([clamped_x, clamped_y, clamped_z], dtype=np.float64)
        was_clamped = not np.allclose(raw_mapped, clamped_pos, atol=1e-4)

        # Apply Cartesian Slew-Rate Limiting (prevents jerk/teleportation)
        final_pos = clamped_pos.copy()
        if enforce_slew_rate and self._last_commanded_position is not None:
            delta = clamped_pos - self._last_commanded_position
            dist = np.linalg.norm(delta)
            if dist > self.config.max_cartesian_step_m:
                step_vector = (delta / dist) * self.config.max_cartesian_step_m
                final_pos = self._last_commanded_position + step_vector

        self._last_commanded_position = final_pos.copy()

        return MappedTarget(
            position=final_pos,
            raw_mapped_pos=raw_mapped,
            is_clamped=was_clamped,
            is_valid=True,
            error_message=None,
        )

    def _get_fallback_position(self) -> np.ndarray:
        """Returns safe default center position."""
        if self._last_commanded_position is not None:
            return self._last_commanded_position.copy()
        return np.array([
            self.config.robot_center_x,
            self.config.robot_center_y,
            self.config.robot_center_z,
        ], dtype=np.float64)

    def reset(self) -> None:
        """Resets the slew rate history."""
        self._last_commanded_position = None
