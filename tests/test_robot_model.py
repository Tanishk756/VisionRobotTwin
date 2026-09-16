"""Unit tests for ResolvedRobotModel and ResolvedJointMetadata schemas."""

import pytest
import math

from robotics.robot_model import (
    RobotCapabilities,
    JointRole,
    JointMotionType,
    ResolvedJointMetadata,
    ResolvedRobotModel,
)


def _make_joint(
    model_index=0,
    canonical_index=0,
    name="joint1",
    role=JointRole.ARM,
    motion_type=JointMotionType.REVOLUTE,
    lower=-2.0,
    upper=2.0,
    force=100.0,
    vel=2.0,
    link_name="link1",
    native_index=0,
    native_joint_type=0,
):
    return ResolvedJointMetadata(
        model_index=model_index,
        canonical_index=canonical_index,
        name=name,
        role=role,
        motion_type=motion_type,
        lower_limit=lower,
        upper_limit=upper,
        max_force=force,
        max_velocity=vel,
        link_name=link_name,
        native_index=native_index,
        native_joint_type=native_joint_type,
    )


def test_resolved_joint_metadata_immutability_and_validation():
    """Verifies that ResolvedJointMetadata is frozen and enforces valid parameters."""
    j = _make_joint()
    with pytest.raises(Exception):
        j.lower_limit = -1.0  # dataclass is frozen

    # Invalid negative model_index
    with pytest.raises(ValueError, match="model_index"):
        _make_joint(model_index=-1)

    # Invalid negative canonical_index
    with pytest.raises(ValueError, match="canonical_index"):
        _make_joint(canonical_index=-1)

    # Non-finite limit
    with pytest.raises(ValueError, match="Non-finite"):
        _make_joint(lower=float("nan"))


def test_resolved_robot_model_helpers_and_immutability():
    """Verifies properties, DoF, range calculations, and native index retrieval."""
    arm_j0 = _make_joint(model_index=0, canonical_index=0, name="arm_j0", role=JointRole.ARM, lower=-1.0, upper=1.0, force=50.0, vel=1.5, native_index=0)
    arm_j1 = _make_joint(model_index=1, canonical_index=1, name="arm_j1", role=JointRole.ARM, lower=-2.0, upper=2.0, force=40.0, vel=2.0, native_index=1)
    grip_j = _make_joint(model_index=2, canonical_index=0, name="finger_j", role=JointRole.GRIPPER, motion_type=JointMotionType.PRISMATIC, lower=0.0, upper=0.04, force=10.0, vel=0.1, native_index=2)

    model = ResolvedRobotModel(
        robot_id="test_bot",
        display_name="Test Robot",
        all_joints=(arm_j0, arm_j1, grip_j),
        arm_joints=(arm_j0, arm_j1),
        gripper_joints=(grip_j,),
        ee_link_name="ee_link",
        ee_link_native_index=3,
        home_joint_positions=(0.0, 0.5),
        capabilities=RobotCapabilities(has_gripper=True, supports_pick_place=True),
    )

    assert model.dof == 2
    assert model.arm_joint_names == ("arm_j0", "arm_j1")
    assert model.arm_lower_limits == (-1.0, -2.0)
    assert model.arm_upper_limits == (1.0, 2.0)
    assert model.arm_joint_ranges == (2.0, 4.0)
    assert model.arm_max_forces == (50.0, 40.0)
    assert model.arm_max_velocities == (1.5, 2.0)
    assert model.require_arm_native_indices() == (0, 1)
    assert model.require_gripper_native_indices() == (2,)


def test_resolved_robot_model_validation_invariants():
    """Verifies that duplicate names, role mismatches, and missing native indices raise ValueError."""
    arm_j0 = _make_joint(model_index=0, canonical_index=0, name="j0", role=JointRole.ARM)
    arm_j1_dup = _make_joint(model_index=1, canonical_index=1, name="j0", role=JointRole.ARM)  # duplicate name

    with pytest.raises(ValueError, match="Duplicate arm joint name"):
        ResolvedRobotModel(
            robot_id="bot",
            display_name="Bot",
            all_joints=(arm_j0, arm_j1_dup),
            arm_joints=(arm_j0, arm_j1_dup),
            gripper_joints=(),
            ee_link_name="ee",
            home_joint_positions=(0.0, 0.0),
        )

    # Role mismatch in arm_joints
    grip_in_arm = _make_joint(model_index=0, canonical_index=0, name="grip", role=JointRole.GRIPPER)
    with pytest.raises(ValueError, match="role ARM"):
        ResolvedRobotModel(
            robot_id="bot",
            display_name="Bot",
            all_joints=(grip_in_arm,),
            arm_joints=(grip_in_arm,),
            gripper_joints=(),
            ee_link_name="ee",
            home_joint_positions=(0.0,),
        )

    # require_arm_native_indices raises when native_index is None
    arm_no_native = _make_joint(model_index=0, canonical_index=0, name="j0", role=JointRole.ARM, native_index=None)
    model_no_native = ResolvedRobotModel(
        robot_id="bot",
        display_name="Bot",
        all_joints=(arm_no_native,),
        arm_joints=(arm_no_native,),
        gripper_joints=(),
        ee_link_name="ee",
        home_joint_positions=(0.0,),
    )
    with pytest.raises(ValueError, match="lacks a native_index"):
        model_no_native.require_arm_native_indices()


def test_rrbot_fixture_model():
    """Verifies that the test-only RRBot model fixture adheres to all schema contracts."""
    from tests.fixtures.rrbot_fixture import create_rrbot_model, RRBOT_CANONICAL_JOINTS

    rrbot = create_rrbot_model()
    assert rrbot.robot_id == "rrbot"
    assert rrbot.display_name == "RRBot 2-DoF"
    assert len(rrbot.arm_joints) == 2
    assert tuple(j.name for j in rrbot.arm_joints) == RRBOT_CANONICAL_JOINTS
    assert tuple(j.name for j in rrbot.arm_joints) == ("joint1", "joint2")
    assert rrbot.arm_joints[0].lower_limit < rrbot.arm_joints[0].upper_limit
    assert rrbot.arm_joints[1].lower_limit < rrbot.arm_joints[1].upper_limit
    assert rrbot.capabilities.supports_velocity_control is True
