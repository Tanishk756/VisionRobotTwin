"""Unit tests for PyBullet VirtualGripper physics constraints and distance gating."""

import pybullet as p
import numpy as np
import pytest

from config.settings import AppConfig
from robotics.simulator import PyBulletSimulator


def test_gripper_distance_gated_attachment():
    """Verify that PyBullet constraint only attaches when EE is physically near the object."""
    config = AppConfig()
    config.simulation.gui = False
    sim = PyBulletSimulator(config, headless=True)

    try:
        gripper = sim.gripper
        pick_cube_id = sim.pick_cube_id
        assert gripper is not None

        # 1. Arm is at home posture, far from cube at [0.45, -0.20, 0.02]
        ee_pos, _ = sim.controller.get_end_effector_pose()
        cube_pos, _ = p.getBasePositionAndOrientation(pick_cube_id, physicsClientId=sim.client_id)
        dist_far = np.linalg.norm(ee_pos - np.array(cube_pos))
        assert dist_far > 0.15  # Home pose is > 15 cm away

        # Attempt grasp -> must be REJECTED
        res_far = gripper.attach_object(pick_cube_id)
        assert not res_far.success
        assert "DISTANCE_EXCEEDED" in res_far.reason
        assert not gripper.is_grasping

        # 2. Command arm to descend onto pick cube
        target_near = np.array([cube_pos[0], cube_pos[1], cube_pos[2] + 0.02])
        ik_res = sim.ik_solver.solve(target_near)
        assert ik_res.success
        sim.controller.set_arm_joint_positions(ik_res.joint_positions)
        for _ in range(240):
            sim.step()

        # Attempt grasp -> must SUCCEED
        res_near = gripper.attach_object(pick_cube_id)
        assert res_near.success
        assert gripper.is_grasping

        # 3. Detach -> must free constraint
        gripper.detach_object()
        assert not gripper.is_grasping

    finally:
        sim.close()
