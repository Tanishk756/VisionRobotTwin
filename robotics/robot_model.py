"""Robot Data Models and Capability Definitions.

Provides strongly typed schemas for multi-manipulator digital twins,
including kinematic configuration, physical joint pattern matching, and
hardware capabilities.
"""

from dataclasses import dataclass, field
from typing import List, Tuple, Optional, Dict, Any


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
