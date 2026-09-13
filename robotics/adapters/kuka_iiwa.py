"""KUKA LBR iiwa 7-DoF Manipulator Adapter.

Defines URDF inspection, joint limits, end-effector identification, and
tool flange management for the 7-DoF KUKA LBR iiwa.
"""

from typing import Dict, Tuple, Any, List
from robotics.adapters.base import RobotAdapter
from robotics.robot_model import RobotModelSpec, RobotCapabilities


class KukaIiwaAdapter(RobotAdapter):
    """Adapter for KUKA LBR iiwa 7-DoF robotic manipulator (bare tool flange)."""

    def __init__(self, spec: RobotModelSpec = None):
        if spec is None:
            spec = RobotModelSpec(
                robot_id="kuka_iiwa",
                display_name="KUKA LBR iiwa",
                urdf_path="kuka_iiwa/model.urdf",
                base_position=(0.0, 0.0, 0.0),
                base_orientation=(0.0, 0.0, 0.0, 1.0),
                fixed_base=True,
                home_joint_positions=[0.0, 0.0, 0.0, -1.5708, 0.0, 1.5708, 0.0],
                arm_joint_name_patterns=["lbr_iiwa_joint", "joint"],
                end_effector_link_name="lbr_iiwa_link_7",
                gripper_joint_name_patterns=[],
                capabilities=RobotCapabilities(
                    has_gripper=False,
                    supports_pick_place=False,
                    supports_velocity_control=True,
                    supports_self_collision=True,
                    max_payload_kg=7.0,
                ),
                max_joint_force=200.0,
                max_joint_velocity_radps=1.71,
                position_gain=0.75,
                velocity_gain=1.0,
                default_ee_orientation=(0.0, 1.0, 0.0, 0.0),
                spherical_reach_m=0.820,
                min_reach_m=0.10,
            )
        super().__init__(spec)

    def fix_joint_limits(self, joint_name: str, lower: float, upper: float) -> Tuple[float, float]:
        """Ensures valid joint limits for KUKA iiwa revolute joints."""
        if lower >= upper:
            # Standard KUKA iiwa joint limits
            if "joint_1" in joint_name or "joint_3" in joint_name or "joint_5" in joint_name:
                return -2.967, 2.967
            elif "joint_2" in joint_name or "joint_4" in joint_name or "joint_6" in joint_name:
                return -2.094, 2.094
            elif "joint_7" in joint_name:
                return -3.054, 3.054
        return lower, upper

    def identify_ee_link_index(self, joint_info_map: Dict[int, Any], default_index: int = 6) -> int:
        """Finds the tool flange link on the KUKA iiwa (typically link 6 / lbr_iiwa_link_7)."""
        for idx, jinfo in joint_info_map.items():
            link_name = jinfo.link_name.lower()
            if "link_7" in link_name or "ee" in link_name or "flange" in link_name:
                return idx
        return default_index
