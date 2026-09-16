"""Robot Data Models and Capability Definitions.

Provides strongly typed schemas for multi-manipulator digital twins,
including kinematic configuration, physical joint pattern matching, and
hardware capabilities.
"""

from dataclasses import dataclass, field
from enum import Enum, auto
import math
from typing import List, Tuple, Optional, Dict, Any


class JointRole(Enum):
    """Categorized role of a robot joint in manipulator control."""
    ARM = auto()
    GRIPPER = auto()
    OTHER = auto()


class JointMotionType(Enum):
    """Generic kinematic motion type of a joint."""
    REVOLUTE = auto()
    PRISMATIC = auto()
    FIXED = auto()
    OTHER = auto()


@dataclass(frozen=True)
class RobotCapabilities:
    """Explicit hardware capability flags for a manipulator."""
    has_gripper: bool
    supports_pick_place: bool
    supports_velocity_control: bool = True
    supports_self_collision: bool = True
    max_payload_kg: float = 3.0


@dataclass
class RobotModelSpec:
    """Comprehensive specification and URDF metadata for a robot manipulator."""
    robot_id: str
    display_name: str
    urdf_path: str
    base_position: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    base_orientation: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    fixed_base: bool = True
    home_joint_positions: List[float] = field(default_factory=list)
    arm_joint_name_patterns: List[str] = field(default_factory=lambda: ["joint", "link"])
    end_effector_link_name: str = "panda_grasptarget"
    gripper_joint_name_patterns: List[str] = field(default_factory=lambda: ["finger", "gripper"])
    capabilities: RobotCapabilities = field(
        default_factory=lambda: RobotCapabilities(has_gripper=False, supports_pick_place=False)
    )
    max_joint_force: float = 87.0
    max_joint_velocity_radps: float = 2.175
    position_gain: float = 0.05
    velocity_gain: float = 1.0
    default_ee_orientation: Tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
    spherical_reach_m: float = 0.855
    min_reach_m: float = 0.10
    allowed_self_collision_pairs: List[Tuple[int, int]] = field(default_factory=list)


@dataclass(frozen=True)
class ResolvedJointMetadata:
    """Immutable metadata for an individual robot joint.

    Attributes:
        model_index: Stable 0-based index in the overall model's joint list.
        canonical_index: 0-based index within its specific role (e.g. 0..6 for 7-DoF arm).
        name: Joint identifier name (e.g. 'panda_joint1').
        role: Functional role (ARM, GRIPPER, or OTHER).
        motion_type: Kinematic motion type (REVOLUTE, PRISMATIC, FIXED, OTHER).
        lower_limit: Minimum joint position limit (rad or m).
        upper_limit: Maximum joint position limit (rad or m).
        max_force: Maximum allowable effort/torque (N or Nm).
        max_velocity: Maximum allowable angular or linear velocity (rad/s or m/s).
        link_name: Name of the child link attached to this joint.
        native_index: Optional engine-native index (e.g. PyBullet joint integer ID).
        native_joint_type: Optional engine-native joint type identifier.
    """
    model_index: int
    canonical_index: int
    name: str
    role: JointRole
    motion_type: JointMotionType
    lower_limit: float
    upper_limit: float
    max_force: float
    max_velocity: float
    link_name: str
    native_index: Optional[int] = None
    native_joint_type: Optional[int] = None

    def __post_init__(self) -> None:
        if self.model_index < 0:
            raise ValueError(f"model_index must be >= 0, got {self.model_index}")
        if self.canonical_index < 0:
            raise ValueError(f"canonical_index must be >= 0, got {self.canonical_index}")
        for field_name, val in [
            ("lower_limit", self.lower_limit),
            ("upper_limit", self.upper_limit),
            ("max_force", self.max_force),
            ("max_velocity", self.max_velocity),
        ]:
            if not math.isfinite(val):
                raise ValueError(f"Non-finite {field_name} value for joint '{self.name}': {val}")
        if self.max_force < 0:
            raise ValueError(f"max_force must be >= 0, got {self.max_force}")
        if self.max_velocity < 0:
            raise ValueError(f"max_velocity must be >= 0, got {self.max_velocity}")


@dataclass(frozen=True)
class ResolvedRobotModel:
    """Immutable, fully resolved kinematic and dynamic metadata for a robotic manipulator.

    Attributes:
        robot_id: String identifier (e.g. 'panda', 'kuka_iiwa').
        display_name: Human-readable model name.
        all_joints: Tuple of all parsed joints in topological/URDF order.
        arm_joints: Tuple of controllable arm joints in canonical order.
        gripper_joints: Tuple of gripper/finger joints in canonical order.
        ee_link_name: Name of the active end-effector link.
        ee_link_native_index: Optional engine-native link index.
        home_joint_positions: Canonical home joint angles (rad).
        capabilities: Hardware capability flags.
    """
    robot_id: str
    display_name: str
    all_joints: Tuple[ResolvedJointMetadata, ...]
    arm_joints: Tuple[ResolvedJointMetadata, ...]
    gripper_joints: Tuple[ResolvedJointMetadata, ...]
    ee_link_name: str
    ee_link_native_index: Optional[int] = None
    home_joint_positions: Tuple[float, ...] = ()
    capabilities: RobotCapabilities = field(
        default_factory=lambda: RobotCapabilities(has_gripper=False, supports_pick_place=False)
    )

    def __post_init__(self) -> None:
        # Validate unique arm joint names
        arm_names = [j.name for j in self.arm_joints]
        if len(arm_names) != len(set(arm_names)):
            raise ValueError(f"Duplicate arm joint name detected in {arm_names}")
        # Validate unique gripper joint names
        grip_names = [j.name for j in self.gripper_joints]
        if len(grip_names) != len(set(grip_names)):
            raise ValueError(f"Duplicate gripper joint name detected in {grip_names}")
        # Validate roles
        for j in self.arm_joints:
            if j.role != JointRole.ARM:
                raise ValueError(f"Joint '{j.name}' in arm_joints must have role ARM, got {j.role}")
        for j in self.gripper_joints:
            if j.role != JointRole.GRIPPER:
                raise ValueError(f"Joint '{j.name}' in gripper_joints must have role GRIPPER, got {j.role}")

    @property
    def dof(self) -> int:
        """Controllable arm degrees of freedom."""
        return len(self.arm_joints)

    @property
    def arm_joint_names(self) -> Tuple[str, ...]:
        return tuple(j.name for j in self.arm_joints)

    @property
    def arm_lower_limits(self) -> Tuple[float, ...]:
        return tuple(j.lower_limit for j in self.arm_joints)

    @property
    def arm_upper_limits(self) -> Tuple[float, ...]:
        return tuple(j.upper_limit for j in self.arm_joints)

    @property
    def arm_joint_ranges(self) -> Tuple[float, ...]:
        return tuple(j.upper_limit - j.lower_limit for j in self.arm_joints)

    @property
    def arm_max_velocities(self) -> Tuple[float, ...]:
        return tuple(j.max_velocity for j in self.arm_joints)

    @property
    def arm_max_forces(self) -> Tuple[float, ...]:
        return tuple(j.max_force for j in self.arm_joints)

    def require_arm_native_indices(self) -> Tuple[int, ...]:
        """Returns tuple of native indices for all arm joints or raises ValueError."""
        indices = []
        for j in self.arm_joints:
            if j.native_index is None:
                raise ValueError(f"Arm joint '{j.name}' lacks a native_index")
            indices.append(j.native_index)
        return tuple(indices)

    def require_gripper_native_indices(self) -> Tuple[int, ...]:
        """Returns tuple of native indices for all gripper joints or raises ValueError."""
        indices = []
        for j in self.gripper_joints:
            if j.native_index is None:
                raise ValueError(f"Gripper joint '{j.name}' lacks a native_index")
            indices.append(j.native_index)
        return tuple(indices)
