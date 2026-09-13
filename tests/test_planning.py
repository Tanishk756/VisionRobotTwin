"""Tests for Joint-Space Motion Planning, RRT-Connect, and Path Shortcutting."""

import pytest
import pybullet as p
import pybullet_data
import numpy as np

from robotics.robot_registry import get_robot_registry
from robotics.robot_controller import GenericRobotController
from robotics.collision import CollisionChecker
from robotics.planning import (
    is_joint_path_collision_free,
    RRTConnectPlanner,
    shortcut_path,
    PlanningResult,
)


@pytest.fixture
def pybullet_scene():
    """Initializes PyBullet physics client with table and a central obstacle."""
    client_id = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=client_id)
    p.setGravity(0, 0, -9.81, physicsClientId=client_id)

    table_id = p.loadURDF("table/table.urdf", [0.5, 0.0, -0.65], useFixedBase=True, physicsClientId=client_id)

    # Obstacle positioned in the workspace
    col_box = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.08, 0.08, 0.20], physicsClientId=client_id)
    vis_box = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.08, 0.08, 0.20], rgbaColor=[0.9, 0.1, 0.1, 1.0], physicsClientId=client_id)
    obstacle_id = p.createMultiBody(
        baseMass=0.0,
        baseCollisionShapeIndex=col_box,
        baseVisualShapeIndex=vis_box,
        basePosition=[0.50, 0.0, 0.20],
        physicsClientId=client_id,
    )

    yield client_id, table_id, obstacle_id
    p.disconnect(physicsClientId=client_id)


def test_direct_path_validator_free_and_blocked(pybullet_scene):
    """Verifies direct path checker identifies collision-free vs colliding linear joint paths."""
    client_id, table_id, obstacle_id = pybullet_scene
    registry = get_robot_registry()
    spec = registry.get_robot_spec("panda")

    body_id = p.loadURDF(spec.urdf_path, useFixedBase=True, physicsClientId=client_id)
    controller = GenericRobotController(client_id, body_id, spec)

    checker = CollisionChecker(
        physics_client_id=client_id,
        robot_id=body_id,
        table_id=table_id,
        obstacle_ids=[obstacle_id],
    )

    q_start = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])
    # Very small perturbation from start (guaranteed free)
    q_near = q_start + 0.02

    is_free, waypoints = is_joint_path_collision_free(
        q_start, q_near, checker, arm_joint_indices=controller.arm_joint_indices
    )
    assert is_free
    assert len(waypoints) >= 2


def test_rrt_connect_planner_deterministic_success(pybullet_scene):
    """Verifies RRT-Connect finds a collision-free path with deterministic seed."""
    client_id, table_id, obstacle_id = pybullet_scene
    registry = get_robot_registry()
    spec = registry.get_robot_spec("panda")

    body_id = p.loadURDF(spec.urdf_path, useFixedBase=True, physicsClientId=client_id)
    controller = GenericRobotController(client_id, body_id, spec)

    checker = CollisionChecker(
        physics_client_id=client_id,
        robot_id=body_id,
        table_id=table_id,
        obstacle_ids=[obstacle_id],
    )

    lows, highs, _, _ = controller.get_joint_limits()

    planner = RRTConnectPlanner(
        lower_limits=lows,
        upper_limits=highs,
        collision_checker=checker,
        arm_joint_indices=controller.arm_joint_indices,
        step_size_rad=0.15,
        max_iterations=400,
        random_seed=42,
    )

    q_start = np.array([0.2, -0.6, 0.1, -2.0, 0.1, 1.5, 0.7])
    q_goal = np.array([-0.2, -0.5, -0.1, -1.8, -0.1, 1.4, 0.6])

    result = planner.plan(q_start, q_goal)
    assert isinstance(result, PlanningResult)
    assert result.success
    assert len(result.path) >= 2
    assert result.planning_time_ms > 0.0
    assert result.path_length_joint_rad > 0.0

    # Verify every waypoint in the returned path is strictly collision free
    for wpt in result.path:
        col = checker.check_collision(wpt, arm_joint_indices=controller.arm_joint_indices)
        assert not col.in_collision


def test_path_shortcutting_preserves_validity(pybullet_scene):
    """Verifies path shortcutting preserves collision-free status and reduces/maintains path length."""
    client_id, table_id, obstacle_id = pybullet_scene
    registry = get_robot_registry()
    spec = registry.get_robot_spec("panda")

    body_id = p.loadURDF(spec.urdf_path, useFixedBase=True, physicsClientId=client_id)
    controller = GenericRobotController(client_id, body_id, spec)

    checker = CollisionChecker(
        physics_client_id=client_id,
        robot_id=body_id,
        table_id=table_id,
        obstacle_ids=[obstacle_id],
    )

    lows, highs, _, _ = controller.get_joint_limits()
    planner = RRTConnectPlanner(
        lower_limits=lows,
        upper_limits=highs,
        collision_checker=checker,
        arm_joint_indices=controller.arm_joint_indices,
        random_seed=123,
    )

    q_start = np.array([0.3, -0.6, 0.1, -2.0, 0.1, 1.5, 0.7])
    q_goal = np.array([-0.3, -0.5, -0.1, -1.8, -0.1, 1.4, 0.6])

    raw_result = planner.plan(q_start, q_goal)
    assert raw_result.success

    smoothed_path = shortcut_path(
        raw_result.path,
        checker,
        arm_joint_indices=controller.arm_joint_indices,
        max_attempts=30,
        random_seed=42,
    )

    assert len(smoothed_path) >= 2
    assert len(smoothed_path) <= len(raw_result.path)

    # Every segment of smoothed path must remain collision-free
    for i in range(len(smoothed_path) - 1):
        is_free, _ = is_joint_path_collision_free(
            smoothed_path[i], smoothed_path[i + 1], checker, arm_joint_indices=controller.arm_joint_indices
        )
        assert is_free
