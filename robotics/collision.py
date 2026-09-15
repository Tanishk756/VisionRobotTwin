"""Collision Checking and Safe State Querying for Manipulators.

Backward-compatibility module re-exporting CollisionResult, CollisionProvider,
and PyBulletCollisionProvider, with CollisionChecker defined as a legacy subclass.
"""

import threading
from typing import List, Optional, Sequence, Tuple
from utils.logger import get_logger
from robotics.collision_provider import (
    CollisionResult,
    CollisionProvider,
    PyBulletCollisionProvider,
)

logger = get_logger("Robotics.Collision")


class CollisionChecker(PyBulletCollisionProvider):
    """Backward-compatible wrapper / subclass of PyBulletCollisionProvider.

    Maintains the legacy constructor and check_collision API while delegating
    to the canonical PyBulletCollisionProvider model implementation.
    """

    def __init__(
        self,
        physics_client_id: int,
        robot_id: int,
        table_id: Optional[int] = None,
        obstacle_ids: Optional[List[int]] = None,
        allowed_body_pairs: Optional[List[Tuple[int, int]]] = None,
        allowed_link_pairs: Optional[List[Tuple[int, int, int, int]]] = None,
        allowed_self_link_pairs: Optional[List[Tuple[int, int]]] = None,
        arm_joint_indices: Optional[Sequence[int]] = None,
        query_lock: Optional[threading.RLock] = None,
    ):
        """Initializes collision checker.

        Args:
            physics_client_id: PyBullet client ID.
            robot_id: Robot multi-body ID.
            table_id: Optional table multi-body ID.
            obstacle_ids: Optional list of obstacle body IDs.
            allowed_body_pairs: Pairs of (body_id_a, body_id_b) permitted to contact.
            allowed_link_pairs: Quadruples of (body_a, link_a, body_b, link_b) permitted to contact.
            allowed_self_link_pairs: Pairs of (link_a, link_b) permitted to contact on the robot.
            arm_joint_indices: Optional controllable arm joint indices.
            query_lock: Optional shared model-query re-entrant lock.
        """
        super().__init__(
            physics_client_id=physics_client_id,
            robot_body_id=robot_id,
            arm_joint_indices=arm_joint_indices,
            table_id=table_id,
            obstacle_ids=obstacle_ids,
            allowed_body_pairs=allowed_body_pairs,
            allowed_link_pairs=allowed_link_pairs,
            allowed_self_link_pairs=allowed_self_link_pairs,
            query_lock=query_lock,
        )


__all__ = [
    "CollisionResult",
    "CollisionProvider",
    "PyBulletCollisionProvider",
    "CollisionChecker",
]
