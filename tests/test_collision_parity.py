"""Golden Parity Tests for Collision Detection and Motion Planning.

Validates that collision evaluation, clearance metrics, and planning algorithms match
pre-A3 baseline behavior (Commit: 26a7c187cf9d44c75c89b02ed3e7ef7e64ec17a7).
"""

import pytest
import numpy as np
import pybullet as p
import pybullet_data

from robotics.robot_registry import get_robot_registry
from robotics.robot_controller import GenericRobotController
from robotics.collision import CollisionChecker, CollisionResult
from robotics.planning import (
    is_joint_path_collision_free,
    RRTConnectPlanner,
    shortcut_path,
    PlanningResult,
)
from tests.fixtures.collision_golden import (
    SOURCE_RUNTIME_SHA,
    PANDA_HOME_Q,
    PANDA_OBSTACLE_COLLISION_Q,
    PANDA_SELF_COLLISION_Q,
    KUKA_HOME_Q,
    KUKA_SELF_COLLISION_Q,
    PLANNING_PANDA_START_Q,
    PLANNING_PANDA_GOAL_Q,
    PLANNING_RRT_SEED,
    PLANNING_SHORTCUT_SEED,
    PLANNING_SHORTCUT_ATTEMPTS,
)


@pytest.fixture
def scene_env():
    """Initializes a DIRECT PyBullet simulation client with table and central obstacle."""
    client_id = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=client_id)
    p.setGravity(0, 0, -9.81, physicsClientId=client_id)

    table_id = p.loadURDF("table/table.urdf", [0.5, 0.0, -0.65], useFixedBase=True, physicsClientId=client_id)

    # Obstacle
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


def test_provenance_metadata():
    """Verifies provenance metadata is recorded."""
    assert len(SOURCE_RUNTIME_SHA) == 40


def test_panda_collision_golden_parity(scene_env):
    """Verifies Panda collision classifications match pre-A3 golden baselines."""
    client_id, table_id, obstacle_id = scene_env
    registry = get_robot_registry()
    spec = registry.get_robot_spec("panda")

    body_id = p.loadURDF(spec.urdf_path, useFixedBase=True, physicsClientId=client_id)
    controller = GenericRobotController(client_id, body_id, spec)

    checker = CollisionChecker(
        physics_client_id=client_id,
        robot_id=body_id,
        table_id=table_id,
        obstacle_ids=[obstacle_id],
        allowed_self_link_pairs=spec.allowed_self_collision_pairs,
    )

    # 1. Home configuration: clear
    res_home = checker.check_collision(PANDA_HOME_Q)
    assert isinstance(res_home, CollisionResult)
    assert not res_home.in_collision
    assert not res_home.self_collision
    assert not res_home.env_collision
    assert res_home.min_distance_m > 0.0

    # 2. Obstacle collision
    res_obs = checker.check_collision(PANDA_OBSTACLE_COLLISION_Q)
    assert res_obs.in_collision
    assert res_obs.env_collision
    assert (body_id, obstacle_id) in res_obs.colliding_bodies

    # 3. Self-collision
    res_self = checker.check_collision(PANDA_SELF_COLLISION_Q)
    assert res_self.in_collision
    assert res_self.self_collision
    assert (body_id, body_id) in res_self.colliding_bodies


def test_kuka_collision_golden_parity(scene_env):
    """Verifies KUKA collision classifications match pre-A3 golden baselines."""
    client_id, table_id, obstacle_id = scene_env
    registry = get_robot_registry()
    spec = registry.get_robot_spec("kuka_iiwa")

    body_id = p.loadURDF(spec.urdf_path, useFixedBase=True, physicsClientId=client_id)
    controller = GenericRobotController(client_id, body_id, spec)

    checker = CollisionChecker(
        physics_client_id=client_id,
        robot_id=body_id,
        table_id=table_id,
        obstacle_ids=[obstacle_id],
        allowed_self_link_pairs=spec.allowed_self_collision_pairs,
    )

    # 1. Home configuration: clear
    res_home = checker.check_collision(KUKA_HOME_Q)
    assert not res_home.in_collision
    assert not res_home.self_collision

    # 2. Folded self-collision
    res_self = checker.check_collision(KUKA_SELF_COLLISION_Q)
    assert res_self.in_collision
    assert res_self.self_collision
    assert (body_id, body_id) in res_self.colliding_bodies


def test_planning_golden_parity(scene_env):
    """Verifies planning direct-path, RRT-Connect, and shortcutting match pre-A3 baselines."""
    client_id, table_id, obstacle_id = scene_env
    registry = get_robot_registry()
    spec = registry.get_robot_spec("panda")

    body_id = p.loadURDF(spec.urdf_path, useFixedBase=True, physicsClientId=client_id)
    controller = GenericRobotController(client_id, body_id, spec)

    checker = CollisionChecker(
        physics_client_id=client_id,
        robot_id=body_id,
        table_id=table_id,
        obstacle_ids=[obstacle_id],
        allowed_self_link_pairs=spec.allowed_self_collision_pairs,
    )

    # 1. Direct path check
    q_start = np.array(PANDA_HOME_Q)
    q_near = q_start + 0.02
    is_free, waypoints = is_joint_path_collision_free(q_start, q_near, checker, arm_joint_indices=controller.arm_joint_indices)
    assert is_free
    assert len(waypoints) >= 2

    # Blocked direct path check
    q_blocked = np.array(PANDA_OBSTACLE_COLLISION_Q)
    is_blocked_free, _ = is_joint_path_collision_free(q_start, q_blocked, checker, arm_joint_indices=controller.arm_joint_indices)
    assert not is_blocked_free

    # 2. Seeded RRT-Connect
    lows, highs, _, _ = controller.get_joint_limits()
    planner = RRTConnectPlanner(
        lower_limits=lows,
        upper_limits=highs,
        collision_checker=checker,
        arm_joint_indices=controller.arm_joint_indices,
        step_size_rad=0.15,
        max_iterations=400,
        random_seed=PLANNING_RRT_SEED,
    )

    plan_res = planner.plan(PLANNING_PANDA_START_Q, PLANNING_PANDA_GOAL_Q)
    assert isinstance(plan_res, PlanningResult)
    assert plan_res.success
    assert len(plan_res.path) >= 2
    assert np.allclose(plan_res.path[0], PLANNING_PANDA_START_Q, atol=1e-5)
    assert np.allclose(plan_res.path[-1], PLANNING_PANDA_GOAL_Q, atol=1e-5)
    assert plan_res.path_length_joint_rad > 0.0

    # Verify all waypoints in path are collision-free
    for wpt in plan_res.path:
        col = checker.check_collision(wpt, arm_joint_indices=controller.arm_joint_indices)
        assert not col.in_collision

    # 3. Path Shortcutting
    smoothed = shortcut_path(
        plan_res.path,
        checker,
        arm_joint_indices=controller.arm_joint_indices,
        max_attempts=PLANNING_SHORTCUT_ATTEMPTS,
        random_seed=PLANNING_SHORTCUT_SEED,
    )
    assert len(smoothed) >= 2
    assert len(smoothed) <= len(plan_res.path)
    assert np.allclose(smoothed[0], PLANNING_PANDA_START_Q, atol=1e-5)
    assert np.allclose(smoothed[-1], PLANNING_PANDA_GOAL_Q, atol=1e-5)
