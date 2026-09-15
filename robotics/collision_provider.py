"""Abstract Collision Provider Boundary and Core Result Models.

Defines the abstract interface for collision detection, self-collision querying,
and clearance evaluation across interchangeable robot backends and simulation environments.
"""

from abc import ABC, abstractmethod
from collections import deque
from dataclasses import dataclass, field
import threading
from typing import Dict, List, Optional, Sequence, Set, Tuple, Union
import numpy as np
import pybullet as p

from utils.logger import get_logger

logger = get_logger("Robotics.CollisionProvider")


@dataclass
class CollisionResult:
    """Represents the outcome of a collision evaluation query."""
    in_collision: bool
    self_collision: bool = False
    env_collision: bool = False
    min_distance_m: float = float("inf")
    min_env_clearance_m: float = float("inf")
    min_self_clearance_m: float = float("inf")
    colliding_bodies: List[Tuple[int, int]] = field(default_factory=list)
    colliding_links: List[Tuple[int, int]] = field(default_factory=list)
    details: str = ""


class CollisionProvider(ABC):
    """Abstract interface for robot collision detection and proximity queries."""

    @abstractmethod
    def check_collision(
        self,
        joint_positions: Optional[Sequence[float]] = None,
    ) -> CollisionResult:
        """Evaluates collision status for the current or candidate joint configuration.

        Args:
            joint_positions: Optional candidate joint configuration vector for controllable
                arm joints. If None, queries collision state at current robot configuration.

        Returns:
            CollisionResult containing collision flags, minimum clearances, and diagnostics.
        """
        pass


class PyBulletCollisionProvider(CollisionProvider):
    """PyBullet implementation of CollisionProvider.

    Manages robot self-collision, environment obstacles, allowed-contact policies,
    and state-preserving candidate configuration collision checks.
    """

    def __init__(
        self,
        physics_client_id: int,
        robot_body_id: int,
        arm_joint_indices: Optional[Sequence[int]] = None,
        table_id: Optional[int] = None,
        obstacle_ids: Optional[Sequence[int]] = None,
        allowed_body_pairs: Optional[Sequence[Tuple[int, int]]] = None,
        allowed_link_pairs: Optional[Sequence[Tuple[int, int, int, int]]] = None,
        allowed_self_link_pairs: Optional[Sequence[Tuple[int, int]]] = None,
        query_lock: Optional[threading.RLock] = None,
    ):
        self.client_id = int(physics_client_id)
        self.robot_id = int(robot_body_id)
        self.arm_joint_indices = tuple(int(j) for j in arm_joint_indices) if arm_joint_indices is not None else None
        self.table_id = int(table_id) if table_id is not None else None
        self.obstacle_ids: List[int] = list(int(o) for o in obstacle_ids) if obstacle_ids else []
        self.allowed_body_pairs: Set[Tuple[int, int]] = set(allowed_body_pairs) if allowed_body_pairs else set()
        self.allowed_link_pairs: Set[Tuple[int, int, int, int]] = set(allowed_link_pairs) if allowed_link_pairs else set()
        self.allowed_self_links: Set[Tuple[int, int]] = set(allowed_self_link_pairs) if allowed_self_link_pairs else set()
        self._lock = query_lock if query_lock is not None else threading.RLock()

        # Build adjacent / allowed self-link pairs from kinematics tree
        self.adjacent_links: Set[Tuple[int, int]] = set()
        self._build_adjacency_sets()

    @property
    def query_lock(self) -> threading.RLock:
        """Returns the model-query re-entrant lock used for candidate configuration serialization."""
        return self._lock

    def _build_adjacency_sets(self) -> None:
        """Constructs link adjacency closures over direct parent-child connections and fixed joints."""
        try:
            num_joints = p.getNumJoints(self.robot_id, physicsClientId=self.client_id)
        except Exception as e:
            logger.debug(f"Could not read joint info during adjacency construction: {e}")
            return

        direct_adj: Dict[int, Set[int]] = {i: set() for i in range(-1, num_joints)}
        fixed_adj: Dict[int, Set[int]] = {i: set() for i in range(-1, num_joints)}

        for i in range(num_joints):
            info = p.getJointInfo(self.robot_id, i, physicsClientId=self.client_id)
            parent_link = int(info[16])
            child_link = i
            direct_adj[parent_link].add(child_link)
            direct_adj[child_link].add(parent_link)
            if info[2] == p.JOINT_FIXED:
                fixed_adj[parent_link].add(child_link)
                fixed_adj[child_link].add(parent_link)

        # 1. Direct parent-child links
        for u in direct_adj:
            for v in direct_adj[u]:
                self.adjacent_links.add((u, v))
                self.adjacent_links.add((v, u))

        # 2. Fixed-joint connected rigid subassemblies (transitive closure over fixed joints)
        for start in fixed_adj:
            visited = {start}
            q = deque([start])
            while q:
                curr = q.popleft()
                for neighbor in fixed_adj[curr]:
                    if neighbor not in visited:
                        visited.add(neighbor)
                        q.append(neighbor)
            for node in visited:
                self.adjacent_links.add((start, node))
                self.adjacent_links.add((node, start))

        # 3. Connections between direct parent-child neighbors and fixed subassemblies
        for fixed_node in fixed_adj:
            for fixed_connected in fixed_adj[fixed_node]:
                for u in direct_adj[fixed_node]:
                    self.adjacent_links.add((u, fixed_connected))
                    self.adjacent_links.add((fixed_connected, u))

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
        joint_positions: Optional[Union[np.ndarray, Sequence[float]]] = None,
        arm_joint_indices: Optional[Sequence[int]] = None,
    ) -> CollisionResult:
        """Checks if the robot is in collision in its current or proposed joint configuration.

        Ensures full state preservation when evaluating proposed joint configurations.

        Args:
            joint_positions: Optional candidate joint configuration vector.
            arm_joint_indices: Optional controllable arm joint indices override.

        Returns:
            CollisionResult detailing collision state and minimum clearance.
        """
        saved_positions = []
        if joint_positions is not None:
            q_arr = np.asarray(joint_positions, dtype=np.float64)
            if not np.all(np.isfinite(q_arr)):
                raise ValueError(f"Candidate joint positions must be finite, got: {joint_positions}")

            if arm_joint_indices is not None:
                active_indices = [int(i) for i in arm_joint_indices]
            elif self.arm_joint_indices is not None:
                active_indices = list(self.arm_joint_indices)
            else:
                # Legacy dynamic fallback for omitted arm indices (Ruling C)
                num_joints = p.getNumJoints(self.robot_id, physicsClientId=self.client_id)
                active_indices = [
                    i
                    for i in range(num_joints)
                    if p.getJointInfo(self.robot_id, i, physicsClientId=self.client_id)[2]
                    != p.JOINT_FIXED
                ][: len(joint_positions)]

            if len(q_arr) != len(active_indices):
                raise ValueError(
                    f"Expected {len(active_indices)} joint positions, got {len(q_arr)}"
                )

            with self._lock:
                # Save existing state
                for idx in active_indices:
                    state = p.getJointState(self.robot_id, idx, physicsClientId=self.client_id)
                    saved_positions.append((idx, state[0], state[1]))

                # Set candidate positions
                for idx, pos in zip(active_indices, q_arr):
                    p.resetJointState(self.robot_id, idx, float(pos), targetVelocity=0.0, physicsClientId=self.client_id)

                p.performCollisionDetection(physicsClientId=self.client_id)
                return self._evaluate_collision_state(saved_positions)
        else:
            with self._lock:
                return self._evaluate_collision_state(saved_positions)

    def _evaluate_collision_state(self, saved_positions: List[Tuple[int, float, float]]) -> CollisionResult:
        """Evaluates collision contacts and closest points against active simulation state."""
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
