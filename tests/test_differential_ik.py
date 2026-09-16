"""Tests for Resolved-Rate Cartesian Velocity Controller and Null-Space Joint Centering."""

import pytest
import pybullet as p
import pybullet_data
import numpy as np

from robotics.robot_registry import get_robot_registry
from robotics.robot_controller import GenericRobotController
from robotics.differential_ik import ResolvedRateController
from robotics.kinematics_provider import PyBulletKinematicsProvider


@pytest.fixture
def pybullet_direct():
    """Initializes a direct PyBullet physics simulation fixture."""
    client_id = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=client_id)
    p.setGravity(0, 0, -9.81, physicsClientId=client_id)
    yield client_id
    p.disconnect(physicsClientId=client_id)


def test_resolved_rate_cartesian_tracking_convergence(pybullet_direct):
    """Verifies resolved-rate controller drives end-effector toward target position."""
    client_id = pybullet_direct
    registry = get_robot_registry()
    spec = registry.get_robot_spec("panda")
    
    body_id = p.loadURDF(spec.urdf_path, useFixedBase=True, physicsClientId=client_id)
    controller = GenericRobotController(client_id, body_id, spec)
    controller.reset_to_home()
    
    rr_controller = ResolvedRateController(
        physics_client_id=client_id,
        robot_controller=controller,
        kp_pos=5.0,
        kp_orn=3.0,
        enable_nullspace=True,
    )
    
    target_pos = np.array([0.45, 0.0, 0.35])
    target_orn = np.array([1.0, 0.0, 0.0, 0.0])
    
    init_err = controller.compute_cartesian_error(target_pos)
    assert init_err > 0.05
    
    # Run closed-loop resolved-rate steps using genuine velocity control
    dt = 1.0 / 240.0
    for _ in range(480):
        q_dot, metrics = rr_controller.compute_step(target_pos, target_orn, dt=dt)
        assert len(q_dot) == 7
        assert np.all(np.isfinite(q_dot))

        controller.set_arm_joint_velocities(q_dot)
        p.stepSimulation(physicsClientId=client_id)

    final_err = controller.compute_cartesian_error(target_pos)
    assert final_err < init_err * 0.1  # Over 90% error reduction (converges to sub-millimeter)


def test_resolved_rate_velocity_clamping_and_nan_rejection(pybullet_direct):
    """Verifies joint velocities are bounded by max velocity and NaNs are rejected."""
    client_id = pybullet_direct
    registry = get_robot_registry()
    spec = registry.get_robot_spec("panda")
    
    body_id = p.loadURDF(spec.urdf_path, useFixedBase=True, physicsClientId=client_id)
    controller = GenericRobotController(client_id, body_id, spec)
    
    rr_controller = ResolvedRateController(
        physics_client_id=client_id,
        robot_controller=controller,
        max_joint_velocity_radps=1.5,
    )
    
    # Far target (large error)
    far_pos = np.array([0.90, 0.50, 0.80])
    q_dot, _ = rr_controller.compute_step(far_pos, dt=1.0 / 240.0)
    assert np.all(np.abs(q_dot) <= 1.5001)
    
    # NaN target
    nan_pos = np.array([np.nan, 0.0, 0.5])
    q_dot_nan, _ = rr_controller.compute_step(nan_pos, dt=1.0 / 240.0)
    assert np.all(q_dot_nan == 0.0)


def test_resolved_rate_controller_with_injected_provider(pybullet_direct):
    """Verifies ResolvedRateController operates when injected with an explicit KinematicsProvider."""
    client_id = pybullet_direct
    spec = get_robot_registry().get_robot_spec("panda")
    body_id = p.loadURDF(spec.urdf_path, useFixedBase=True, physicsClientId=client_id)
    controller = GenericRobotController(client_id, body_id, spec)
    controller.reset_to_home()

    provider = PyBulletKinematicsProvider(
        physics_client_id=client_id,
        robot_body_id=body_id,
        arm_joint_indices=controller.arm_joint_indices,
        end_effector_link_index=controller.ee_link_index,
    )

    rr_controller = ResolvedRateController(
        physics_client_id=client_id,
        robot_controller=controller,
        kinematics_provider=provider,
    )

    assert rr_controller.kinematics_provider is provider
    q_dot, metrics = rr_controller.compute_step([0.45, 0.0, 0.35], [1.0, 0.0, 0.0, 0.0])
    assert len(q_dot) == 7
    assert np.all(np.isfinite(q_dot))
    assert metrics.manipulability >= 0.0


def test_resolved_rate_controller_has_zero_direct_pybullet_calls():
    """Verifies that ResolvedRateController module contains zero direct pybullet references or calls."""
    import inspect
    import robotics.differential_ik as diff_module

    source = inspect.getsource(diff_module)
    assert "p.calculateJacobian" not in source
    assert "p.getLinkState" not in source
    assert "p.getNumJoints" not in source
    assert "p.getJointInfo" not in source
    assert "import pybullet as p" not in source


def test_resolved_rate_controller_with_ros2_simulation_backend():
    """Verifies ResolvedRateController computes q_dot and GenericRobotController publishes it via ROS2SimulationBackend."""
    import time
    from unittest.mock import MagicMock, patch
    import robotics.backends.ros2_simulation_backend as sim_module
    import robotics.backends.ros2_joint_state_backend as state_module
    from robotics.backends.ros2_simulation_backend import ROS2SimulationBackend
    from robotics.backends.ros2_state_mapping import (
        ROS2JointStateBackendConfig,
        ROS2SimulationBackendConfig,
    )
    from robotics.backends.base import TimestampedJointState
    from robotics.kinematics_provider import KinematicsProvider
    from robotics.robot_model import (
        JointRole,
        JointMotionType,
        ResolvedJointMetadata,
        ResolvedRobotModel,
    )

    class MockFloat64MultiArray:
        def __init__(self):
            self.data = []

    with patch.object(sim_module, "_HAS_RCLPY", True), \
         patch.object(state_module, "_HAS_RCLPY", True), \
         patch.object(sim_module, "Float64MultiArray", MockFloat64MultiArray):

        joints = tuple(
            ResolvedJointMetadata(
                model_index=i,
                canonical_index=i,
                name=f"panda_joint{i+1}",
                role=JointRole.ARM,
                motion_type=JointMotionType.REVOLUTE,
                lower_limit=-2.8973,
                upper_limit=2.8973,
                max_force=87.0,
                max_velocity=2.175,
                link_name=f"panda_link{i+1}",
            )
            for i in range(7)
        )
        model = ResolvedRobotModel(
            robot_id="panda",
            display_name="Panda",
            all_joints=joints,
            arm_joints=joints,
            gripper_joints=(),
            ee_link_name="panda_link7",
            home_joint_positions=(0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785),
        )

        state_cfg = ROS2JointStateBackendConfig(expected_joint_names=tuple(f"panda_joint{i+1}" for i in range(7)))
        sim_cfg = ROS2SimulationBackendConfig(
            state_config=state_cfg,
            command_mode="velocity",
            velocity_command_topic="/panda/velocity_commands",
            require_subscriber_ready=False,
        )

        sim_backend = ROS2SimulationBackend(sim_cfg)
        sim_backend._is_connected = True
        sim_backend._commands_enabled = True
        sim_backend._cmd_publisher = MagicMock()
        sim_backend._state_backend = MagicMock()
        sim_backend._state_backend.is_connected.return_value = True

        current_state = TimestampedJointState(
            source_timestamp_s=1.0,
            receive_timestamp_s=time.monotonic(),
            joint_names=tuple(f"panda_joint{i+1}" for i in range(7)),
            positions=(0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785),
            velocities=(0.0,) * 7,
        )
        sim_backend._state_backend.get_joint_state.return_value = current_state

        mock_kinematics = MagicMock(spec=KinematicsProvider)
        mock_kinematics.compute_fk.return_value = (np.array([0.4, 0.0, 0.4]), np.array([1.0, 0.0, 0.0, 0.0]))
        # 6x7 dummy Jacobian
        J = np.zeros((6, 7))
        for i in range(6):
            J[i, i] = 1.0
        mock_kinematics.compute_jacobian.return_value = (J[:3, :], J[3:, :], J)

        controller = GenericRobotController(
            resolved_model=model,
            backend=sim_backend,
            kinematics_provider=mock_kinematics,
        )

        rr_controller = ResolvedRateController(
            robot_controller=controller,
            kinematics_provider=mock_kinematics,
            kp_pos=2.0,
            enable_nullspace=False,
        )

        target_pos = np.array([0.45, 0.0, 0.40])
        target_orn = np.array([1.0, 0.0, 0.0, 0.0])

        q_dot, metrics = rr_controller.compute_step(target_pos, target_orn, dt=0.01)
        assert len(q_dot) == 7
        assert np.all(np.isfinite(q_dot))

        # Command velocities to GenericRobotController, which dispatches to ROS2SimulationBackend
        controller.set_arm_joint_velocities(q_dot)
        sim_backend._cmd_publisher.publish.assert_called_once()
        published_msg = sim_backend._cmd_publisher.publish.call_args[0][0]
        for val, qd in zip(published_msg.data, q_dot):
            assert val == pytest.approx(qd, abs=1e-5)
