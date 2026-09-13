"""Collision Checking and Safe State Querying for Manipulators in PyBullet."""

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Set, Tuple, Union
import numpy as np
import pybullet as p

from utils.logger import get_logger

logger = get_logger("Robotics.Collision")


@dataclass
class CollisionResult:
    """Represents the collision evaluation outcome."""
    in_collision: bool
    min_distance_m: float
    colliding_bodies: List[Tuple[int, int]] = field(default_factory=list)
    colliding_links: List[Tuple[int, int]] = field(default_factory=list)
    details: str = ""


class CollisionChecker:
    """Manages robot self-collision, environment, and obstacle contact queries."""

    def __init__(
        self,
        physics_client_id: int,
        robot_id: int,
        table_id: Optional[int] = None,
        obstacle_ids: Optional[List[int]] = None,
        allowed_body_pairs: Optional[List[Tuple[int, int]]] = None,
        allowed_link_pairs: Optional[List[Tuple[int, int, int, int]]] = None,
    ):
        """Initializes collision checker.

        Args:
            physics_client_id: PyBullet client ID.
            robot_id: Robot multi-body ID.
            table_id: Optional table multi-body ID.
            obstacle_ids: Optional list of obstacle body IDs.
            allowed_body_pairs: Pairs of (body_id_a, body_id_b) permitted to contact.
            allowed_link_pairs: Quadruples of (body_a, link_a, body_b, link_b) permitted to contact.
        """
        self.client_id = physics_client_id
        self.robot_id = robot_id
        self.table_id = table_id
        self.obstacle_ids = list(obstacle_ids) if obstacle_ids else []
        self.allowed_body_pairs = set(allowed_body_pairs) if allowed_body_pairs else set()
        self.allowed_link_pairs = set(allowed_link_pairs) if allowed_link_pairs else set()

    def add_obstacle(self, obstacle_id: int) -> None:
        """Adds an obstacle body to monitored collision objects."""
        if obstacle_id not in self.obstacle_ids:
            self.obstacle_ids.append(obstacle_id)

    def remove_obstacle(self, obstacle_id: int) -> None:
        """Removes an obstacle body from monitored collision objects."""
        if obstacle_id in self.obstacle_ids:
            self.obstacle_ids.remove(obstacle_id)

    def _is_allowed(self, body_a: int, link_a: int, body_b: int, link_b: int) -> bool:
        """Checks whether contact between the two bodies/links is whitelisted."""
        if (body_a, body_b) in self.allowed_body_pairs or (body_b, body_a) in self.allowed_body_pairs:
            return True
        if (body_a, link_a, body_b, link_b) in self.allowed_link_pairs or (
            body_b,
            link_b,
            body_a,
            link_a,
        ) in self.allowed_link_pairs:
            return True
        return False

    def check_collision(
        self,
        joint_positions: Optional[Union[np.ndarray, List[float]]] = None,
        arm_joint_indices: Optional[List[int]] = None,
    ) -> CollisionResult:
        """Checks if the robot is in collision in its current or proposed joint configuration.

        Ensures full state preservation when evaluating proposed joint configurations.

        Args:
            joint_positions: Optional candidate joint configuration.
            arm_joint_indices: Controllable arm joint indices if joint_positions is given.

        Returns:
            CollisionResult detailing collision state and minimum clearance.
        """
        saved_positions = []
        if joint_positions is not None:
            # If indices not provided, query movable joints
            if arm_joint_indices is None:
                num_joints = p.getNumJoints(self.robot_id, physicsClientId=self.client_id)
                arm_joint_indices = [
                    i
                    for i in range(num_joints)
                    if p.getJointInfo(self.robot_id, i, physicsClientId=self.client_id)[2]
                    != p.JOINT_FIXED
                ][: len(joint_positions)]

            # Save existing state
            for idx in arm_joint_indices:
                state = p.getJointState(self.robot_id, idx, physicsClientId=self.client_id)
                saved_positions.append((idx, state[0], state[1]))

            # Set candidate positions
            for idx, pos in zip(arm_joint_indices, joint_positions):
                p.resetJointState(self.robot_id, idx, float(pos), physicsClientId=self.client_id)

            # Perform collision update
            p.performCollisionDetection(physicsClientId=self.client_id)

        try:
            colliding_bodies = []
            colliding_links = []
            min_dist = float("inf")

            # 1. Environment bodies (table + obstacles)
            env_bodies = []
            if self.table_id is not None:
                env_bodies.append(self.table_id)
            env_bodies.extend(self.obstacle_ids)

            for env_id in env_bodies:
                # Query contact points (overlap / penetration)
                contacts = p.getContactPoints(
                    bodyA=self.robot_id,
                    bodyB=env_id,
                    physicsClientId=self.client_id,
                )
                for c in contacts:
                    link_robot = c[3]
                    link_env = c[4]
                    if not self._is_allowed(self.robot_id, link_robot, env_id, link_env):
                        colliding_bodies.append((self.robot_id, env_id))
                        colliding_links.append((link_robot, link_env))

                # Query distance to obstacle
                closest = p.getClosestPoints(
                    bodyA=self.robot_id,
                    bodyB=env_id,
                    distance=1.0,
                    physicsClientId=self.client_id,
                )
                for cp in closest:
                    link_robot = cp[3]
                    link_env = cp[4]
                    if not self._is_allowed(self.robot_id, link_robot, env_id, link_env):
                        dist = cp[8]
                        if dist < min_dist:
                            min_dist = dist

            in_collision = len(colliding_bodies) > 0 or (min_dist <= 0.0)
            if min_dist == float("inf"):
                min_dist = 1.0  # Safe default if no obstacles nearby

            return CollisionResult(
                in_collision=in_collision,
                min_distance_m=float(min_dist),
                colliding_bodies=colliding_bodies,
                colliding_links=colliding_links,
                details=f"Collisions: {len(colliding_bodies)}, Min Clearance: {min_dist:.4f}m",
            )

        finally:
            # Restore saved state if temporary pose was checked
            if saved_positions:
                for idx, pos, vel in saved_positions:
                    p.resetJointState(self.robot_id, idx, pos, vel, physicsClientId=self.client_id)
                p.performCollisionDetection(physicsClientId=self.client_id)
