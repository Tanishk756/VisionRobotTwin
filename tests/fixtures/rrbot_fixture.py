"""Test fixture providing ResolvedRobotModel for the 2-DoF RRBot.

Used exclusively for test and controller-level validation against ros2_control
demo environments (e.g. ros2_control_demo_example_3).
"""

from typing import Tuple
from robotics.robot_model import (
    JointMotionType,
    JointRole,
    ResolvedJointMetadata,
    ResolvedRobotModel,
    RobotCapabilities,
)

RRBOT_CANONICAL_JOINTS: Tuple[str, ...] = ("joint1", "joint2")


def create_rrbot_model() -> ResolvedRobotModel:
    """Creates a minimal ResolvedRobotModel for 2-DoF RRBot."""
    joint1 = ResolvedJointMetadata(
        model_index=0,
        canonical_index=0,
        name="joint1",
        role=JointRole.ARM,
        motion_type=JointMotionType.REVOLUTE,
        lower_limit=-3.14159,
        upper_limit=3.14159,
        max_force=100.0,
        max_velocity=2.0,
        link_name="link2",
    )
    joint2 = ResolvedJointMetadata(
        model_index=1,
        canonical_index=1,
        name="joint2",
        role=JointRole.ARM,
        motion_type=JointMotionType.REVOLUTE,
        lower_limit=-3.14159,
        upper_limit=3.14159,
        max_force=100.0,
        max_velocity=2.0,
        link_name="link3",
    )
    joints = (joint1, joint2)
    return ResolvedRobotModel(
        robot_id="rrbot",
        display_name="RRBot 2-DoF",
        all_joints=joints,
        arm_joints=joints,
        gripper_joints=(),
        ee_link_name="link3",
        home_joint_positions=(0.0, 0.0),
        capabilities=RobotCapabilities(
            has_gripper=False,
            supports_pick_place=False,
            supports_velocity_control=True,
            supports_self_collision=False,
            max_payload_kg=1.0,
        ),
    )
