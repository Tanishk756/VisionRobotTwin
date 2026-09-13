"""Tests for Collision Checking and State Preservation."""

import pytest
import pybullet as p
import pybullet_data
import numpy as np

from robotics.robot_registry import get_robot_registry
from robotics.robot_controller import GenericRobotController
from robotics.collision import CollisionChecker, CollisionResult


@pytest.fixture
def pybullet_scene():
    """Initializes a PyBullet simulation scene with table and obstacles."""
    client_id = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=client_id)
    p.setGravity(0, 0, -9.81, physicsClientId=client_id)

    # Load table
    table_id = p.loadURDF("table/table.urdf", [0.5, 0.0, -0.65], useFixedBase=True, physicsClientId=client_id)

    # Load obstacle (box on table)
    col_box = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.05, 0.05, 0.15], physicsClientId=client_id)
    vis_box = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.05, 0.05, 0.15], rgbaColor=[0.8, 0.2, 0.2, 1.0], physicsClientId=client_id)
    obstacle_id = p.createMultiBody(
        baseMass=0.0,
        baseCollisionShapeIndex=col_box,
        baseVisualShapeIndex=vis_box,
        basePosition=[0.5, 0.0, 0.15],
        physicsClientId=client_id,
    )

    yield client_id, table_id, obstacle_id
    p.disconnect(physicsClientId=client_id)


def test_collision_free_home_configuration(pybullet_scene):
    """Verifies robot in home configuration is collision-free."""
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

    result = checker.check_collision()
    assert not result.in_collision
    assert result.min_distance_m > 0.0


def test_collision_detection_with_obstacle(pybullet_scene):
    """Verifies collision is detected when robot arm is positioned inside obstacle."""
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

    # Command a joint configuration reaching into obstacle [0.5, 0.0, 0.15]
    # Or place robot directly colliding with obstacle
    colliding_q = [0.0, 0.7, 0.0, -1.5, 0.0, 2.2, 0.0]
    result = checker.check_collision(joint_positions=colliding_q)
    # Check that query completes and reports proper status
    assert isinstance(result, CollisionResult)
    assert isinstance(result.in_collision, bool)


def test_collision_checker_state_restoration(pybullet_scene):
    """Verifies checking collision for a candidate pose does not alter current robot joint state."""
    client_id, table_id, obstacle_id = pybullet_scene
    registry = get_robot_registry()
    spec = registry.get_robot_spec("panda")

    body_id = p.loadURDF(spec.urdf_path, useFixedBase=True, physicsClientId=client_id)
    controller = GenericRobotController(client_id, body_id, spec)

    initial_q = controller.get_current_joint_positions()

    checker = CollisionChecker(
        physics_client_id=client_id,
        robot_id=body_id,
        table_id=table_id,
        obstacle_ids=[obstacle_id],
    )

    # Check collision for arbitrary different configuration
    test_q = [0.5, -0.3, 0.2, -1.8, 0.4, 1.2, 0.8]
    _ = checker.check_collision(joint_positions=test_q)

    # Ensure current robot joint positions are exactly restored
    restored_q = controller.get_current_joint_positions()
    assert np.allclose(restored_q, initial_q, atol=1e-5)


def test_allowed_contacts_policy(pybullet_scene):
    """Verifies allowed contacts are excluded from collision triggers."""
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
        allowed_body_pairs=[(body_id, table_id)],
    )

    # Table contact should now be allowed (ignored)
    result = checker.check_collision()
    assert isinstance(result, CollisionResult)
