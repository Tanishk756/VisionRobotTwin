"""Unit tests for Franka Panda robot kinematics, controller, and PyBullet IK solver."""

import numpy as np
import pytest

from config.settings import AppConfig
from robotics.simulator import PyBulletSimulator
from robotics.inverse_kinematics import IKStatus


def test_panda_ik_convergence_and_tracking():
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
            assert res.status in (IKStatus.SOLUTION_RETURNED, IKStatus.CONVERGED)
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

        # Target containing NaN
        res_nan = ik_solver.solve(np.array([np.nan, 0.0, 0.5]))
        assert not res_nan.success
        assert res_nan.status == IKStatus.INVALID_TARGET
    finally:
        sim.close()
