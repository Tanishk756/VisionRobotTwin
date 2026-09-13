"""Franka Emika Panda Manipulator Adapter.

Defines URDF inspection, joint limits, end-effector identification, and
gripper management for the 7-DoF Franka Emika Panda.
"""

from typing import Dict, Tuple, Any, List
from robotics.adapters.base import RobotAdapter
from robotics.robot_model import RobotModelSpec, RobotCapabilities


class PandaAdapter(RobotAdapter):
    """Adapter for Franka Emika Panda 7-DoF arm with parallel-jaw gripper."""

    def __init__(self, spec: RobotModelSpec = None):
        if spec is None:
            spec = RobotModelSpec(
                robot_id="panda",
                display_name="Franka Emika Panda",
                urdf_path="franka_panda/panda.urdf",
                base_position=(0.0, 0.0, 0.0),
                base_orientation=(0.0, 0.0, 0.0, 1.0),
                fixed_base=True,
                home_joint_positions=[0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785],
                arm_joint_name_patterns=["panda_joint"],
                end_effector_link_name="panda_grasptarget",
                gripper_joint_name_patterns=["panda_finger"],
                capabilities=RobotCapabilities(
                    has_gripper=True,
                    supports_pick_place=True,
                    supports_velocity_control=True,
                    supports_self_collision=True,
                    max_payload_kg=3.0,
                ),
                max_joint_force=200.0,
                max_joint_velocity_radps=2.0,
                position_gain=0.75,
                velocity_gain=1.0,
                default_ee_orientation=(1.0, 0.0, 0.0, 0.0),
                spherical_reach_m=0.855,
                min_reach_m=0.10,
            )
        super().__init__(spec)

    def fix_joint_limits(self, joint_name: str, lower: float, upper: float) -> Tuple[float, float]:
        """Adjusts default zero limits in pybullet_data panda.urdf."""
        if lower >= upper:
            if "joint1" in joint_name or "joint3" in joint_name or "joint5" in joint_name or "joint7" in joint_name:
                return -2.8973, 2.8973
            elif "joint2" in joint_name:
                return -1.7628, 1.7628
            elif "joint4" in joint_name:
                return -3.0718, -0.0698
            elif "joint6" in joint_name:
                return -0.0175, 3.7525
        return lower, upper

    def identify_ee_link_index(self, joint_info_map: Dict[int, Any], default_index: int = 11) -> int:
        """Finds the link index corresponding to the panda_grasptarget or panda_hand."""
        # Prioritize grasptarget link (link 11) at finger tips
        for idx, jinfo in joint_info_map.items():
            link_name = jinfo.link_name.lower()
            if "grasptarget" in link_name:
                return idx
        for idx, jinfo in joint_info_map.items():
            link_name = jinfo.link_name.lower()
            if "hand" in link_name or "ee" in link_name:
                return idx
        return default_index
