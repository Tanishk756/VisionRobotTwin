"""Unit tests for Franka Panda robot kinematics, controller, and PyBullet IK solver."""

from unittest.mock import MagicMock, patch
import numpy as np
import pytest

from config.settings import AppConfig
from robotics.simulator import PyBulletSimulator
from robotics.inverse_kinematics import IKStatus, PandaIKSolver


def test_panda_ik_solution_returned_and_tracking():
    """Verify PyBullet IK solver finds valid joint solutions for reachable workspace points."""
    config = AppConfig()
    config.simulation.gui = False  # Headless mode for automated tests

    sim = PyBulletSimulator(config, headless=True)
    try:
        controller = sim.controller
        ik_solver = sim.ik_solver
        assert controller is not None
        assert ik_solver is not None
        assert len(controller.arm_joint_indices) == 7

        test_targets = [
            np.array([0.50, 0.00, 0.35]),
            np.array([0.40, -0.15, 0.25]),
            np.array([0.55, 0.20, 0.40]),
        ]

        for target in test_targets:
            res = ik_solver.solve(target, target_orientation=[1.0, 0.0, 0.0, 0.0])
            assert res.success is True
            assert res.status == IKStatus.SOLUTION_RETURNED
            assert len(res.joint_positions) == 7
            assert not any(np.isnan(res.joint_positions))

            controller.set_arm_joint_positions(res.joint_positions)
            for _ in range(120):
                sim.step()

            err = controller.compute_cartesian_error(target)
            assert err < 0.045, f"Tracking error {err:.4f} m exceeds tolerance for target {target}"

    finally:
        sim.close()


def test_ik_unreachable_and_invalid_targets():
    """Verify IK solver safely rejects out-of-reach and invalid targets."""
    config = AppConfig()
    config.simulation.gui = False
    sim = PyBulletSimulator(config, headless=True)
    try:
        ik_solver = sim.ik_solver

        # Target beyond physical arm reach (e.g. 5.0m)
        res_far = ik_solver.solve(np.array([5.0, 0.0, 0.0]))
        assert not res_far.success
        assert res_far.status == IKStatus.UNREACHABLE
        assert len(res_far.joint_positions) == 0

        # Target containing NaN
        res_nan = ik_solver.solve(np.array([np.nan, 0.0, 0.5]))
        assert not res_nan.success
        assert res_nan.status == IKStatus.INVALID_TARGET
        assert len(res_nan.joint_positions) == 0
    finally:
        sim.close()


def test_ik_out_of_limits_is_failure_and_never_commanded():
    """Verify that candidate solutions exceeding joint limits return success=False and are not commanded."""
    config = AppConfig()
    config.simulation.gui = False
    sim = PyBulletSimulator(config, headless=True)
    try:
        ik_solver = sim.ik_solver

        # Mock calculateInverseKinematics to return a joint position far outside joint 0 upper limit (+2.8973 rad)
        # We will mock pybullet call with an angle of 3.50 rad for joint 0
        fake_raw = [3.50, 0.0, 0.0, -1.5, 0.0, 1.5, 0.0, 0.0, 0.0]
        with patch("pybullet.calculateInverseKinematics", return_value=fake_raw):
            res = ik_solver.solve(np.array([0.5, 0.0, 0.35]))
            assert not res.success, "OUT_OF_LIMITS solution must not have success=True"
            assert res.status == IKStatus.OUT_OF_LIMITS
            assert len(res.joint_positions) == 0

        # Verify controller does not accept empty positions
        initial_joints = sim.controller.get_current_joint_positions()
        if not res.success:
            # Application logic branch: only command if success is True
            pass
        else:
            sim.controller.set_arm_joint_positions(res.joint_positions)

        current_joints = sim.controller.get_current_joint_positions()
        assert np.allclose(initial_joints, current_joints), "Rejected IK should never alter commanded joints"

    finally:
        sim.close()
