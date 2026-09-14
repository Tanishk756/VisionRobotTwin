"""Base Robot Adapter Interface.

Defines the abstract interface and standard behaviors for manipulator adapters.
"""

from abc import ABC, abstractmethod
from typing import List, Tuple, Optional, Dict, Any
import numpy as np

from robotics.robot_model import RobotModelSpec, RobotCapabilities


class RobotAdapter(ABC):
    """Abstract interface defining manipulator-specific URDF parsing and kinematics overrides."""

    def __init__(self, spec: RobotModelSpec):
        self.spec = spec

    @property
    def robot_id(self) -> str:
        return self.spec.robot_id

    @property
    def display_name(self) -> str:
        return self.spec.display_name

    @property
    def capabilities(self) -> RobotCapabilities:
        return self.spec.capabilities

    @abstractmethod
    def fix_joint_limits(self, joint_name: str, lower: float, upper: float) -> Tuple[float, float]:
        """Adjusts uninitialized or zero joint limits in the URDF."""
        pass

    @abstractmethod
    def identify_ee_link_index(self, joint_info_map: Dict[int, Any], default_index: int) -> int:
        """Determines the primary end-effector link index."""
        pass
