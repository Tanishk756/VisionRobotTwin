"""Unit tests for Franka Panda robot kinematics, controller, and PyBullet IK solver."""

import numpy as np
import pytest

from config.settings import AppConfig
from robotics.simulator import PyBulletSimulator


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

        # Test reachable target points in front of robot
        test_targets = [
            np.array([0.50, 0.00, 0.35]),
            np.array([0.40, -0.15, 0.25]),
            np.array([0.55, 0.20, 0.40]),
        ]

        for target in test_targets:
            res = ik_solver.solve(target)
            assert res.success is True
            assert len(res.joint_positions) == 7
            assert not any(np.isnan(res.joint_positions))

            # Apply joint positions to controller and step simulation
            controller.set_arm_joint_positions(res.joint_positions)
            for _ in range(120):
                sim.step()

            # Verify end-effector arrived within reasonable tolerance (< 4.5 cm dynamically)
            err = controller.compute_cartesian_error(target)
            assert err < 0.045, f"Tracking error {err:.4f} m exceeds tolerance for target {target}"

    finally:
        sim.close()
