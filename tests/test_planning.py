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
    controller.reset_to_home()

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

    # A path that drives right into the obstacle
    q_blocked_target = np.array([0.0, 0.04, 0.0, -2.38, 0.0, 2.41, 0.785])
    is_blocked_free, _ = is_joint_path_collision_free(
        q_start, q_blocked_target, checker, arm_joint_indices=controller.arm_joint_indices
    )
    assert not is_blocked_free


def test_rrt_connect_planner_deterministic_success(pybullet_scene):
    """Verifies RRT-Connect finds a collision-free path with deterministic seed."""
    client_id, table_id, obstacle_id = pybullet_scene
    registry = get_robot_registry()
    spec = registry.get_robot_spec("panda")

    body_id = p.loadURDF(spec.urdf_path, useFixedBase=True, physicsClientId=client_id)
    controller = GenericRobotController(client_id, body_id, spec)
    controller.reset_to_home()

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

    # Guarantee endpoint matching regardless of tree swapping
    assert np.allclose(result.path[0], q_start, atol=1e-5)
    assert np.allclose(result.path[-1], q_goal, atol=1e-5)

    # Verify every waypoint and segment is strictly collision free
    for wpt in result.path:
        col = checker.check_collision(wpt, arm_joint_indices=controller.arm_joint_indices)
        assert not col.in_collision

    for i in range(len(result.path) - 1):
        seg_free, _ = is_joint_path_collision_free(
            result.path[i], result.path[i + 1], checker, arm_joint_indices=controller.arm_joint_indices
        )
        assert seg_free


def test_path_shortcutting_preserves_validity(pybullet_scene):
    """Verifies path shortcutting preserves collision-free status and reduces/maintains path length."""
    client_id, table_id, obstacle_id = pybullet_scene
    registry = get_robot_registry()
    spec = registry.get_robot_spec("panda")

    body_id = p.loadURDF(spec.urdf_path, useFixedBase=True, physicsClientId=client_id)
    controller = GenericRobotController(client_id, body_id, spec)
    controller.reset_to_home()

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
    assert np.allclose(smoothed_path[0], q_start, atol=1e-5)
    assert np.allclose(smoothed_path[-1], q_goal, atol=1e-5)

    # Every segment of smoothed path must remain collision-free
    for i in range(len(smoothed_path) - 1):
        is_free, _ = is_joint_path_collision_free(
            smoothed_path[i], smoothed_path[i + 1], checker, arm_joint_indices=controller.arm_joint_indices
        )
        assert is_free


def test_impossible_scene_failure_and_state_restoration(pybullet_scene):
    """Verifies planner cleanly returns failure with bounded iterations when goal is blocked by impossible obstacle."""
    client_id, table_id, obstacle_id = pybullet_scene
    registry = get_robot_registry()
    spec = registry.get_robot_spec("panda")

    body_id = p.loadURDF(spec.urdf_path, useFixedBase=True, physicsClientId=client_id)
    controller = GenericRobotController(client_id, body_id, spec)
    controller.reset_to_home()

    # Encase goal position inside an impenetrable obstacle
    giant_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.5, 0.5, 0.5], physicsClientId=client_id)
    giant_obs = p.createMultiBody(baseMass=0.0, baseCollisionShapeIndex=giant_col, basePosition=[0.0, 0.5, 0.5], physicsClientId=client_id)

    checker = CollisionChecker(
        physics_client_id=client_id,
        robot_id=body_id,
        table_id=table_id,
        obstacle_ids=[obstacle_id, giant_obs],
    )

    lows, highs, _, _ = controller.get_joint_limits()
    max_iter = 20
    planner = RRTConnectPlanner(
        lower_limits=lows,
        upper_limits=highs,
        collision_checker=checker,
        arm_joint_indices=controller.arm_joint_indices,
        max_iterations=max_iter,
        random_seed=42,
    )

    initial_q = controller.get_current_joint_positions()
    q_start = np.array([0.2, -0.6, 0.1, -2.0, 0.1, 1.5, 0.7])
    # Goal inside the giant obstacle
    q_goal_in_obs = np.array([0.0, 0.5, 0.0, -1.0, 0.0, 1.5, 0.0])

    res = planner.plan(q_start, q_goal_in_obs)
    assert not res.success
    assert res.iterations <= max_iter
    assert "collision" in res.reason.lower() or "iteration" in res.reason.lower() or "timeout" in res.reason.lower()

    # Verify robot state was restored
    restored_q = controller.get_current_joint_positions()
    assert np.allclose(restored_q, initial_q, atol=1e-5)


class MockCollisionProvider:
    """Mock collision provider for testing planning algorithms without PyBullet."""

    def __init__(self, blocked_predicate=None):
        self.blocked_predicate = blocked_predicate
        self.queries = []

    def check_collision(self, joint_positions=None):
        from robotics.collision_provider import CollisionResult
        if joint_positions is None:
            return CollisionResult(in_collision=False)
        q = list(joint_positions)
        self.queries.append(q)
        is_blocked = self.blocked_predicate(q) if self.blocked_predicate else False
        return CollisionResult(
            in_collision=is_blocked,
            self_collision=False,
            env_collision=is_blocked,
            min_distance_m=0.0 if is_blocked else 0.5,
        )


def test_pure_rrt_planning_without_pybullet():
    """Verifies RRTConnectPlanner functions with a MockCollisionProvider without any PyBullet dependency."""
    # Define a 3-DoF robot with a spherical obstacle in joint space around [0.5, 0.5, 0.5]
    def in_obstacle(q):
        return np.linalg.norm(np.array(q) - np.array([0.5, 0.5, 0.5])) < 0.25

    mock_provider = MockCollisionProvider(blocked_predicate=in_obstacle)

    planner = RRTConnectPlanner(
        lower_limits=[0.0, 0.0, 0.0],
        upper_limits=[1.0, 1.0, 1.0],
        collision_checker=mock_provider,
        step_size_rad=0.10,
        max_iterations=300,
        random_seed=42,
    )

    q_start = [0.1, 0.1, 0.1]
    q_goal = [0.9, 0.9, 0.9]

    res = planner.plan(q_start, q_goal)
    assert res.success
    assert len(res.path) >= 2
    assert np.allclose(res.path[0], q_start, atol=1e-5)
    assert np.allclose(res.path[-1], q_goal, atol=1e-5)

    # Every waypoint must be collision free
    for wpt in res.path:
        assert not in_obstacle(wpt)

    # Test shortcutting
    smoothed = shortcut_path(res.path, mock_provider, max_attempts=20, random_seed=42)
    assert len(smoothed) >= 2
    assert len(smoothed) <= len(res.path)
    assert np.allclose(smoothed[0], q_start, atol=1e-5)
    assert np.allclose(smoothed[-1], q_goal, atol=1e-5)
    for wpt in smoothed:
        assert not in_obstacle(wpt)
