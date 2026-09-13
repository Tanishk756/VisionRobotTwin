"""Rigid Body Kinematics and SE(3) Homogeneous Coordinate Transformations.

Implements rigorous Lie group SE(3) representations for coordinate frame transformations:
- Camera Optical Frame (C)
- ArUco Marker Frame (M)
- Robot Base Frame (B)
- Robot End-Effector Frame (E)

Ensures exact mathematical correctness for composition, inversion, translation extraction,
and SO(3) rotations via Euler angles (RPY) and unit quaternions [x, y, z, w].
"""

import math
from typing import Optional, Tuple, Union
import numpy as np
from scipy.spatial.transform import Rotation


def euler_to_rotation_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Computes a 3x3 SO(3) rotation matrix from Roll-Pitch-Yaw angles (radians).
    
    Convention: Intrinsic Z-Y-X (or Extrinsic X-Y-Z) Euler rotation:
      R = R_z(yaw) @ R_y(pitch) @ R_x(roll)
    """
    rot = Rotation.from_euler("xyz", [roll, pitch, yaw], degrees=False)
    return rot.as_matrix()


def rotation_matrix_to_euler(rot_mat: np.ndarray) -> Tuple[float, float, float]:
    """Extracts (roll, pitch, yaw) angles in radians from a 3x3 rotation matrix."""
    rot = Rotation.from_matrix(rot_mat)
    rpy = rot.as_euler("xyz", degrees=False)
    return float(rpy[0]), float(rpy[1]), float(rpy[2])


def quaternion_to_rotation_matrix(quat_xyzw: Union[np.ndarray, list, tuple]) -> np.ndarray:
    """Converts a unit quaternion [x, y, z, w] to a 3x3 rotation matrix."""
    q = np.asarray(quat_xyzw, dtype=np.float64)
    norm = np.linalg.norm(q)
    if norm < 1e-8:
        raise ValueError("Quaternion cannot have zero norm")
    q = q / norm
    rot = Rotation.from_quat(q)
    return rot.as_matrix()


def rotation_matrix_to_quaternion(rot_mat: np.ndarray) -> np.ndarray:
    """Converts a 3x3 rotation matrix to a normalized unit quaternion [x, y, z, w]."""
    rot = Rotation.from_matrix(rot_mat)
    return rot.as_quat()


def create_homogeneous_matrix(
    rotation: Union[np.ndarray, Tuple[float, float, float], Tuple[float, float, float, float]],
    translation: Union[np.ndarray, Tuple[float, float, float], list],
) -> np.ndarray:
    """Constructs a 4x4 SE(3) homogeneous transformation matrix.

    Args:
        rotation: 3x3 rotation matrix, 3-element Euler RPY tuple, or 4-element quaternion [x,y,z,w].
        translation: 3-element translation vector [x, y, z].

    Returns:
        (4, 4) homogeneous transformation matrix.
    """
    T = np.eye(4, dtype=np.float64)
    t = np.asarray(translation, dtype=np.float64).reshape(3)

    if isinstance(rotation, np.ndarray) and rotation.shape == (3, 3):
        R = rotation
    elif len(rotation) == 3:
        R = euler_to_rotation_matrix(rotation[0], rotation[1], rotation[2])
    elif len(rotation) == 4:
        R = quaternion_to_rotation_matrix(rotation)
    else:
        raise ValueError(f"Invalid rotation input: {rotation}")

    T[:3, :3] = R
    T[:3, 3] = t
    return T


def invert_homogeneous_matrix(T: np.ndarray) -> np.ndarray:
    """Computes analytical inverse of a 4x4 SE(3) transformation matrix:
    
      T = [ R | t ]   =>   T^-1 = [ R^T | -R^T @ t ]
          [ 0 | 1 ]               [ 0   |    1    ]
    """
    if T.shape != (4, 4):
        raise ValueError(f"Matrix must be (4, 4), got {T.shape}")

    R = T[:3, :3]
    t = T[:3, 3]

    T_inv = np.eye(4, dtype=np.float64)
    R_T = R.T
    T_inv[:3, :3] = R_T
    T_inv[:3, 3] = -R_T @ t
    return T_inv


def compose_transforms(*transforms: np.ndarray) -> np.ndarray:
    """Composes an ordered sequence of 4x4 SE(3) transformation matrices:
       T_result = T_1 @ T_2 @ ... @ T_n
    """
    if not transforms:
        return np.eye(4, dtype=np.float64)

    result = np.eye(4, dtype=np.float64)
    for T in transforms:
        if T.shape != (4, 4):
            raise ValueError(f"Each matrix must be (4, 4), got {T.shape}")
        result = result @ T
    return result


def is_valid_se3(T: np.ndarray, tol: float = 1e-4) -> bool:
    """Validates whether a 4x4 matrix is a mathematically valid SE(3) transformation:
    1. Shape is (4, 4)
    2. Bottom row is [0, 0, 0, 1]
    3. Rotation submatrix R is orthogonal (R @ R^T = I)
    4. det(R) = +1 (proper rotation, no reflection)
    """
    if T.shape != (4, 4):
        return False

    # Check bottom row
    if not np.allclose(T[3, :], [0.0, 0.0, 0.0, 1.0], atol=tol):
        return False

    # Check SO(3) rotation submatrix
    R = T[:3, :3]
    if not np.allclose(R @ R.T, np.eye(3), atol=tol):
        return False
    if not np.isclose(np.linalg.det(R), 1.0, atol=tol):
        return False

    return True


class SE3Transform:
    """High-level object-oriented SE(3) transformation wrapper."""

    def __init__(self, matrix: Optional[np.ndarray] = None):
        if matrix is None:
            self._matrix = np.eye(4, dtype=np.float64)
        else:
            m = np.asarray(matrix, dtype=np.float64)
            if m.shape != (4, 4):
                raise ValueError(f"SE3Transform requires (4, 4) matrix, got {m.shape}")
            self._matrix = m

    @classmethod
    def from_translation_and_rotation(
        cls,
        translation: Union[np.ndarray, Tuple[float, float, float], list],
        rotation: Union[np.ndarray, Tuple[float, float, float], Tuple[float, float, float, float]],
    ) -> "SE3Transform":
        """Factory method to construct transform from translation and rotation."""
        return cls(create_homogeneous_matrix(rotation, translation))

    @classmethod
    def from_rpy(
        cls,
        x: float, y: float, z: float,
        roll: float, pitch: float, yaw: float
    ) -> "SE3Transform":
        """Constructs transform from XYZ translation and Roll-Pitch-Yaw angles (radians)."""
        return cls(create_homogeneous_matrix((roll, pitch, yaw), (x, y, z)))

    @property
    def matrix(self) -> np.ndarray:
        return self._matrix.copy()

    @property
    def translation(self) -> np.ndarray:
        return self._matrix[:3, 3].copy()

    @property
    def rotation_matrix(self) -> np.ndarray:
        return self._matrix[:3, :3].copy()

    @property
    def quaternion_xyzw(self) -> np.ndarray:
        return rotation_matrix_to_quaternion(self._matrix[:3, :3])

    @property
    def euler_rpy(self) -> Tuple[float, float, float]:
        return rotation_matrix_to_euler(self._matrix[:3, :3])

    def inverse(self) -> "SE3Transform":
        """Returns the inverse SE(3) transformation."""
        return SE3Transform(invert_homogeneous_matrix(self._matrix))

    def __matmul__(self, other: "SE3Transform") -> "SE3Transform":
        """Composes two transforms: self @ other."""
        return SE3Transform(self._matrix @ other.matrix)

    def transform_point(self, point: Union[np.ndarray, list, tuple]) -> np.ndarray:
        """Transforms a 3D point: p_new = R @ p + t."""
        p = np.asarray(point, dtype=np.float64).reshape(3)
        p_homo = np.array([p[0], p[1], p[2], 1.0], dtype=np.float64)
        p_trans = self._matrix @ p_homo
        return p_trans[:3]

    def transform_vector(self, vector: Union[np.ndarray, list, tuple]) -> np.ndarray:
        """Transforms a 3D direction vector (rotation only): v_new = R @ v."""
        v = np.asarray(vector, dtype=np.float64).reshape(3)
        return self._matrix[:3, :3] @ v

    def is_valid(self, tol: float = 1e-4) -> bool:
        return is_valid_se3(self._matrix, tol)
