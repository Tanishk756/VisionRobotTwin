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
    self_collision: bool = False
    env_collision: bool = False
    min_distance_m: float = float("inf")
    min_env_clearance_m: float = float("inf")
    min_self_clearance_m: float = float("inf")
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
        allowed_self_link_pairs: Optional[List[Tuple[int, int]]] = None,
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
        """
        self.client_id = physics_client_id
        self.robot_id = robot_id
        self.table_id = table_id
        self.obstacle_ids = list(obstacle_ids) if obstacle_ids else []
        self.allowed_body_pairs = set(allowed_body_pairs) if allowed_body_pairs else set()
        self.allowed_link_pairs = set(allowed_link_pairs) if allowed_link_pairs else set()
        self.allowed_self_links = set(allowed_self_link_pairs) if allowed_self_link_pairs else set()

        # Build adjacent / allowed self-link pairs from URDF kinematics tree
        self.adjacent_links: Set[Tuple[int, int]] = set()
        num_joints = p.getNumJoints(self.robot_id, physicsClientId=self.client_id)

        # Build link tree adjacency graph (including base link -1)
        from collections import deque
        adj: Dict[int, Set[int]] = {i: set() for i in range(-1, num_joints)}
        for i in range(num_joints):
            info = p.getJointInfo(self.robot_id, i, physicsClientId=self.client_id)
            parent_link = int(info[16])
            child_link = i
            adj[parent_link].add(child_link)
            adj[child_link].add(parent_link)

        # Allow self contact for links within topological tree distance <= 4
        # (accounts for intermediate fixed joints and adjacent mechanical collars)
        for start in adj:
            dist = {start: 0}
            q = deque([start])
            while q:
                u = q.popleft()
                if dist[u] <= 4:
                    self.adjacent_links.add((start, u))
                    self.adjacent_links.add((u, start))
                    for v in adj[u]:
                        if v not in dist:
                            dist[v] = dist[u] + 1
                            q.append(v)

    def add_obstacle(self, obstacle_id: int) -> None:
        """Adds an obstacle body to monitored collision objects."""
        if obstacle_id not in self.obstacle_ids:
            self.obstacle_ids.append(obstacle_id)

    def remove_obstacle(self, obstacle_id: int) -> None:
        """Removes an obstacle body from monitored collision objects."""
        if obstacle_id in self.obstacle_ids:
            self.obstacle_ids.remove(obstacle_id)

    def _is_allowed(self, body_a: int, link_a: int, body_b: int, link_b: int) -> bool:
        """Checks whether contact between two distinct bodies/links is whitelisted."""
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

    def _is_allowed_self_contact(self, link_a: int, link_b: int) -> bool:
        """Checks whether contact between two links on the same robot is permitted."""
        if link_a == link_b:
            return True
        if (link_a, link_b) in self.adjacent_links or (link_b, link_a) in self.adjacent_links:
            return True
        if (link_a, link_b) in self.allowed_self_links or (link_b, link_a) in self.allowed_self_links:
            return True
        if (self.robot_id, link_a, self.robot_id, link_b) in self.allowed_link_pairs or (
            self.robot_id,
            link_b,
            self.robot_id,
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
                p.resetJointState(self.robot_id, idx, float(pos), targetVelocity=0.0, physicsClientId=self.client_id)

            p.performCollisionDetection(physicsClientId=self.client_id)

        try:
            colliding_bodies = []
            colliding_links = []
            min_env_dist = float("inf")
            min_self_dist = float("inf")
            has_self_collision = False
            has_env_collision = False

            # 1. Robot Self-Collision Query
            self_contacts = p.getContactPoints(
                bodyA=self.robot_id,
                bodyB=self.robot_id,
                physicsClientId=self.client_id,
            )
            for sc in self_contacts:
                la = sc[3]
                lb = sc[4]
                if not self._is_allowed_self_contact(la, lb):
                    has_self_collision = True
                    colliding_bodies.append((self.robot_id, self.robot_id))
                    colliding_links.append((la, lb))

            # Query self-distance if feasible
            self_closest = p.getClosestPoints(
                bodyA=self.robot_id,
                bodyB=self.robot_id,
                distance=0.5,
                physicsClientId=self.client_id,
            )
            for scp in self_closest:
                la = scp[3]
                lb = scp[4]
                if not self._is_allowed_self_contact(la, lb):
                    d = scp[8]
                    if d < min_self_dist:
                        min_self_dist = d
                    if d <= 0.0:
                        has_self_collision = True
                        if (self.robot_id, self.robot_id) not in colliding_bodies:
                            colliding_bodies.append((self.robot_id, self.robot_id))
                        if (la, lb) not in colliding_links:
                            colliding_links.append((la, lb))

            # 2. Environment bodies (table + obstacles)
            env_bodies = []
            if self.table_id is not None:
                env_bodies.append(self.table_id)
            env_bodies.extend(self.obstacle_ids)

            for env_id in env_bodies:
                contacts = p.getContactPoints(
                    bodyA=self.robot_id,
                    bodyB=env_id,
                    physicsClientId=self.client_id,
                )
                for c in contacts:
                    link_robot = c[3]
                    link_env = c[4]
                    if not self._is_allowed(self.robot_id, link_robot, env_id, link_env):
                        has_env_collision = True
                        colliding_bodies.append((self.robot_id, env_id))
                        colliding_links.append((link_robot, link_env))

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
                        if dist < min_env_dist:
                            min_env_dist = dist
                        if dist <= 0.0:
                            has_env_collision = True
                            if (self.robot_id, env_id) not in colliding_bodies:
                                colliding_bodies.append((self.robot_id, env_id))
                            if (link_robot, link_env) not in colliding_links:
                                colliding_links.append((link_robot, link_env))

            in_collision = has_self_collision or has_env_collision
            overall_min_dist = min(min_env_dist, min_self_dist)

            return CollisionResult(
                in_collision=in_collision,
                self_collision=has_self_collision,
                env_collision=has_env_collision,
                min_distance_m=float(overall_min_dist),
                min_env_clearance_m=float(min_env_dist),
                min_self_clearance_m=float(min_self_dist),
                colliding_bodies=colliding_bodies,
                colliding_links=colliding_links,
                details=f"Self-Col: {has_self_collision}, Env-Col: {has_env_collision}, Min Clear: {overall_min_dist:.4f}m",
            )

        finally:
            if saved_positions:
                for idx, pos, vel in saved_positions:
                    p.resetJointState(self.robot_id, idx, pos, targetVelocity=vel, physicsClientId=self.client_id)
                p.performCollisionDetection(physicsClientId=self.client_id)
