"""Tests for CollisionProvider Abstract Interface and Core Models."""

import pytest
from typing import Optional, Sequence
import numpy as np

from robotics.collision_provider import (
    CollisionResult,
    CollisionProvider,
)


class _DummyConcreteCollisionProvider(CollisionProvider):
    """Minimal concrete implementation for testing ABC contract."""

    def __init__(self, in_collision: bool = False, min_distance: float = 0.25):
        self._in_collision = in_collision
        self._min_distance = min_distance
        self.last_queried_q = None

    def check_collision(
        self,
        joint_positions: Optional[Sequence[float]] = None,
    ) -> CollisionResult:
        self.last_queried_q = list(joint_positions) if joint_positions is not None else None
        return CollisionResult(
            in_collision=self._in_collision,
            self_collision=False,
            env_collision=self._in_collision,
            min_distance_m=self._min_distance,
            min_env_clearance_m=self._min_distance,
            min_self_clearance_m=float("inf"),
        )


def test_collision_provider_cannot_be_instantiated_directly():
    """Verifies CollisionProvider is an abstract base class that cannot be instantiated."""
    with pytest.raises(TypeError, match="Can't instantiate abstract class"):
        CollisionProvider()  # type: ignore


def test_collision_provider_subclass_contract():
    """Verifies that a concrete subclass can be instantiated and fulfills check_collision."""
    provider = _DummyConcreteCollisionProvider(in_collision=False, min_distance=0.15)
    assert isinstance(provider, CollisionProvider)

    res = provider.check_collision([0.1, 0.2, 0.3])
    assert isinstance(res, CollisionResult)
    assert not res.in_collision
    assert res.min_distance_m == 0.15
    assert provider.last_queried_q == [0.1, 0.2, 0.3]


def test_collision_result_dataclass_defaults():
    """Verifies CollisionResult default attributes match expected specifications."""
    res = CollisionResult(in_collision=False)
    assert not res.in_collision
    assert not res.self_collision
    assert not res.env_collision
    assert res.min_distance_m == float("inf")
    assert res.min_env_clearance_m == float("inf")
    assert res.min_self_clearance_m == float("inf")
    assert res.colliding_bodies == []
    assert res.colliding_links == []
    assert res.details == ""


@pytest.fixture
def pybullet_scene():
    """Initializes a DIRECT PyBullet simulation client with table and an obstacle."""
    import pybullet as p
    import pybullet_data

    client_id = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=client_id)
    p.setGravity(0, 0, -9.81, physicsClientId=client_id)

    table_id = p.loadURDF("table/table.urdf", [0.5, 0.0, -0.65], useFixedBase=True, physicsClientId=client_id)

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


def test_pybullet_collision_provider_parity_panda(pybullet_scene):
    """Verifies PyBulletCollisionProvider detects home clear, obstacle collision, and self-collision on Panda."""
    import pybullet as p
    from robotics.robot_registry import get_robot_registry
    from robotics.robot_controller import GenericRobotController
    from robotics.collision_provider import PyBulletCollisionProvider

    client_id, table_id, obstacle_id = pybullet_scene
    registry = get_robot_registry()
    spec = registry.get_robot_spec("panda")

    body_id = p.loadURDF(spec.urdf_path, useFixedBase=True, physicsClientId=client_id)
    controller = GenericRobotController(client_id, body_id, spec)

    provider = PyBulletCollisionProvider(
        physics_client_id=client_id,
        robot_body_id=body_id,
        arm_joint_indices=controller.arm_joint_indices,
        table_id=table_id,
        obstacle_ids=[obstacle_id],
        allowed_self_link_pairs=spec.allowed_self_collision_pairs,
    )
    assert isinstance(provider, CollisionProvider)

    # 1. Home configuration is collision-free
    res_home = provider.check_collision()
    assert not res_home.in_collision
    assert not res_home.self_collision
    assert not res_home.env_collision
    assert res_home.min_distance_m > 0.0

    # 2. Obstacle collision
    colliding_q = [0.0, 0.04, 0.0, -2.38, 0.0, 2.41, 0.785]
    res_obs = provider.check_collision(colliding_q)
    assert res_obs.in_collision
    assert res_obs.env_collision
    assert (body_id, obstacle_id) in res_obs.colliding_bodies

    # 3. Self-collision
    folded_q = [0.0, 1.5, 0.0, -3.0, 0.0, 3.5, 0.0]
    res_self = provider.check_collision(folded_q)
    assert res_self.in_collision
    assert res_self.self_collision
    assert (body_id, body_id) in res_self.colliding_bodies


def test_pybullet_collision_provider_parity_kuka(pybullet_scene):
    """Verifies PyBulletCollisionProvider detects home clear and self-collision on KUKA."""
    import pybullet as p
    from robotics.robot_registry import get_robot_registry
    from robotics.robot_controller import GenericRobotController
    from robotics.collision_provider import PyBulletCollisionProvider

    client_id, table_id, obstacle_id = pybullet_scene
    registry = get_robot_registry()
    spec = registry.get_robot_spec("kuka_iiwa")

    body_id = p.loadURDF(spec.urdf_path, useFixedBase=True, physicsClientId=client_id)
    controller = GenericRobotController(client_id, body_id, spec)

    provider = PyBulletCollisionProvider(
        physics_client_id=client_id,
        robot_body_id=body_id,
        arm_joint_indices=controller.arm_joint_indices,
        table_id=table_id,
        obstacle_ids=[obstacle_id],
        allowed_self_link_pairs=spec.allowed_self_collision_pairs,
    )

    # 1. Home configuration is collision-free
    res_home = provider.check_collision()
    assert not res_home.in_collision
    assert not res_home.self_collision

    # 2. Self-collision
    folded_kuka_q = [0.0, 2.0, 0.0, -2.0, 0.0, 2.0, 0.0]
    res_self = provider.check_collision(folded_kuka_q)
    assert res_self.in_collision
    assert res_self.self_collision
    assert (body_id, body_id) in res_self.colliding_bodies


def test_pybullet_collision_provider_state_preservation(pybullet_scene):
    """Verifies PyBulletCollisionProvider saves and restores q and dq during candidate checks."""
    import pybullet as p
    from robotics.robot_registry import get_robot_registry
    from robotics.robot_controller import GenericRobotController
    from robotics.collision_provider import PyBulletCollisionProvider

    client_id, table_id, obstacle_id = pybullet_scene
    registry = get_robot_registry()
    spec = registry.get_robot_spec("panda")

    body_id = p.loadURDF(spec.urdf_path, useFixedBase=True, physicsClientId=client_id)
    controller = GenericRobotController(client_id, body_id, spec)

    initial_q = controller.get_current_joint_positions()

    provider = PyBulletCollisionProvider(
        physics_client_id=client_id,
        robot_body_id=body_id,
        arm_joint_indices=controller.arm_joint_indices,
        table_id=table_id,
        obstacle_ids=[obstacle_id],
    )

    test_q = [0.5, -0.3, 0.2, -1.8, 0.4, 1.2, 0.8]
    _ = provider.check_collision(test_q)

    restored_q = controller.get_current_joint_positions()
    assert np.allclose(restored_q, initial_q, atol=1e-5)


def test_pybullet_collision_provider_input_validation(pybullet_scene):
    """Verifies PyBulletCollisionProvider rejects invalid input dimensions, NaN, and Inf."""
    import pybullet as p
    from robotics.robot_registry import get_robot_registry
    from robotics.robot_controller import GenericRobotController
    from robotics.collision_provider import PyBulletCollisionProvider

    client_id, table_id, obstacle_id = pybullet_scene
    registry = get_robot_registry()
    spec = registry.get_robot_spec("panda")

    body_id = p.loadURDF(spec.urdf_path, useFixedBase=True, physicsClientId=client_id)
    controller = GenericRobotController(client_id, body_id, spec)

    provider = PyBulletCollisionProvider(
        physics_client_id=client_id,
        robot_body_id=body_id,
        arm_joint_indices=controller.arm_joint_indices,
        table_id=table_id,
        obstacle_ids=[obstacle_id],
    )

    # Wrong length
    with pytest.raises(ValueError, match="Expected 7 joint positions"):
        provider.check_collision([0.0] * 6)

    # NaN
    with pytest.raises(ValueError, match="finite"):
        provider.check_collision([float("nan")] * 7)

    # Inf
    with pytest.raises(ValueError, match="finite"):
        provider.check_collision([float("inf")] * 7)


def test_pybullet_collision_provider_shared_lock(pybullet_scene):
    """Verifies PyBulletCollisionProvider supports injected shared lock."""
    import threading
    import pybullet as p
    from robotics.robot_registry import get_robot_registry
    from robotics.robot_controller import GenericRobotController
    from robotics.collision_provider import PyBulletCollisionProvider

    client_id, table_id, obstacle_id = pybullet_scene
    registry = get_robot_registry()
    spec = registry.get_robot_spec("panda")

    body_id = p.loadURDF(spec.urdf_path, useFixedBase=True, physicsClientId=client_id)
    controller = GenericRobotController(client_id, body_id, spec)

    shared_lock = threading.RLock()
    provider = PyBulletCollisionProvider(
        physics_client_id=client_id,
        robot_body_id=body_id,
        arm_joint_indices=controller.arm_joint_indices,
        query_lock=shared_lock,
    )
    assert provider.query_lock is shared_lock
