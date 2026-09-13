"""Collision-Aware Joint-Space Motion Planning using RRT-Connect and Shortcutting."""

from dataclasses import dataclass, field
import time
from typing import List, Optional, Sequence, Tuple, Union
import numpy as np

from robotics.collision import CollisionChecker
from utils.logger import get_logger

logger = get_logger("Robotics.Planning")


@dataclass
class PlanningResult:
    """Result of a joint-space motion planning query."""
    success: bool
    path: List[np.ndarray] = field(default_factory=list)
    iterations: int = 0
    planning_time_ms: float = 0.0
    path_length_joint_rad: float = 0.0
    reason: str = ""


def compute_path_length(path: Sequence[np.ndarray]) -> float:
    """Computes total Euclidean joint-space travel along waypoints in radians."""
    if len(path) < 2:
        return 0.0
    length = 0.0
    for i in range(len(path) - 1):
        length += float(np.linalg.norm(path[i + 1] - path[i]))
    return length


def is_joint_path_collision_free(
    q_start: Union[np.ndarray, List[float]],
    q_goal: Union[np.ndarray, List[float]],
    collision_checker: CollisionChecker,
    resolution_rad: float = 0.05,
    arm_joint_indices: Optional[List[int]] = None,
) -> Tuple[bool, List[np.ndarray]]:
    """Checks whether a straight linear joint-space path from q_start to q_goal is collision-free.

    Args:
        q_start: Starting joint configuration vector.
        q_goal: Goal joint configuration vector.
        collision_checker: CollisionChecker instance.
        resolution_rad: Maximum joint distance between intermediate collision checks.
        arm_joint_indices: Joint indices corresponding to the configuration vector.

    Returns:
        (is_collision_free, checked_waypoints)
    """
    start = np.asarray(q_start, dtype=np.float64).flatten()
    goal = np.asarray(q_goal, dtype=np.float64).flatten()

    dist = np.linalg.norm(goal - start)
    if dist < 1e-6:
        col = collision_checker.check_collision(start, arm_joint_indices=arm_joint_indices)
        return not col.in_collision, [start]

    num_steps = max(2, int(np.ceil(dist / resolution_rad)) + 1)
    alphas = np.linspace(0.0, 1.0, num_steps)
    waypoints = [start + a * (goal - start) for a in alphas]

    for wpt in waypoints:
        col = collision_checker.check_collision(wpt, arm_joint_indices=arm_joint_indices)
        if col.in_collision:
            return False, waypoints

    return True, waypoints


class _RRTNode:
    """Internal search tree node for RRT-Connect."""

    def __init__(self, config: np.ndarray, parent: Optional["_RRTNode"] = None):
        self.config = np.asarray(config, dtype=np.float64)
        self.parent = parent


class RRTConnectPlanner:
    """Bidirectional RRT-Connect motion planner in joint space."""

    def __init__(
        self,
        lower_limits: Sequence[float],
        upper_limits: Sequence[float],
        collision_checker: CollisionChecker,
        arm_joint_indices: Optional[List[int]] = None,
        step_size_rad: float = 0.10,
        goal_bias: float = 0.05,
        max_iterations: int = 500,
        collision_resolution_rad: float = 0.05,
        random_seed: Optional[int] = None,
    ):
        self.lower_limits = np.asarray(lower_limits, dtype=np.float64)
        self.upper_limits = np.asarray(upper_limits, dtype=np.float64)
        self.dof = len(self.lower_limits)
        self.checker = collision_checker
        self.arm_joint_indices = arm_joint_indices
        self.step_size = float(step_size_rad)
        self.goal_bias = float(goal_bias)
        self.max_iterations = int(max_iterations)
        self.collision_resolution = float(collision_resolution_rad)
        self.rng = np.random.RandomState(random_seed)

    def _sample_random_config(self, goal: np.ndarray) -> np.ndarray:
        """Samples random configuration within joint limits with optional goal bias."""
        if self.rng.uniform(0.0, 1.0) < self.goal_bias:
            return goal.copy()
        return self.rng.uniform(self.lower_limits, self.upper_limits)

    def _find_nearest(self, tree: List[_RRTNode], target: np.ndarray) -> _RRTNode:
        """Finds nearest node in the tree to target configuration."""
        dists = [np.linalg.norm(node.config - target) for node in tree]
        return tree[int(np.argmin(dists))]

    def _extend(
        self,
        tree: List[_RRTNode],
        target: np.ndarray,
    ) -> Tuple[str, Optional[_RRTNode]]:
        """Extends tree one step towards target."""
        nearest = self._find_nearest(tree, target)
        diff = target - nearest.config
        dist = np.linalg.norm(diff)

        if dist < 1e-6:
            return "REACHED", nearest

        step_dist = min(self.step_size, dist)
        new_config = nearest.config + (diff / dist) * step_dist

        # Check collision for segment from nearest to new_config
        is_free, _ = is_joint_path_collision_free(
            nearest.config,
            new_config,
            self.checker,
            resolution_rad=self.collision_resolution,
            arm_joint_indices=self.arm_joint_indices,
        )

        if not is_free:
            return "TRAPPED", None

        new_node = _RRTNode(new_config, parent=nearest)
        tree.append(new_node)

        if np.linalg.norm(new_config - target) < 1e-4:
            return "REACHED", new_node
        return "ADVANCED", new_node

    def _connect(
        self,
        tree: List[_RRTNode],
        target: np.ndarray,
    ) -> Tuple[str, Optional[_RRTNode]]:
        """Repeatedly extends tree towards target until reached or trapped."""
        status = "ADVANCED"
        last_node = None
        while status == "ADVANCED":
            status, last_node = self._extend(tree, target)
        return status, last_node

    def plan(
        self,
        q_start: Union[np.ndarray, List[float]],
        q_goal: Union[np.ndarray, List[float]],
    ) -> PlanningResult:
        """Plans a collision-free path between q_start and q_goal.

        Args:
            q_start: Initial joint configuration.
            q_goal: Goal joint configuration.

        Returns:
            PlanningResult containing path, solve time, and metadata.
        """
        start_time = time.perf_counter()
        start = np.asarray(q_start, dtype=np.float64).flatten()
        goal = np.asarray(q_goal, dtype=np.float64).flatten()

        # 1. Validate start and goal limits and collisions
        col_start = self.checker.check_collision(start, arm_joint_indices=self.arm_joint_indices)
        if col_start.in_collision:
            return PlanningResult(
                success=False,
                reason="Start configuration is in collision",
                planning_time_ms=(time.perf_counter() - start_time) * 1000.0,
            )

        col_goal = self.checker.check_collision(goal, arm_joint_indices=self.arm_joint_indices)
        if col_goal.in_collision:
            return PlanningResult(
                success=False,
                reason="Goal configuration is in collision",
                planning_time_ms=(time.perf_counter() - start_time) * 1000.0,
            )

        # 2. Check direct path first (fast-path optimization)
        direct_free, direct_wpts = is_joint_path_collision_free(
            start, goal, self.checker, resolution_rad=self.collision_resolution, arm_joint_indices=self.arm_joint_indices
        )
        if direct_free:
            solve_ms = (time.perf_counter() - start_time) * 1000.0
            return PlanningResult(
                success=True,
                path=[start, goal],
                iterations=0,
                planning_time_ms=solve_ms,
                path_length_joint_rad=compute_path_length([start, goal]),
                reason="Direct collision-free path found",
            )

        # 3. Bidirectional RRT-Connect
        tree_start = [_RRTNode(start)]
        tree_goal = [_RRTNode(goal)]

        for iteration in range(1, self.max_iterations + 1):
            q_rand = self._sample_random_config(goal)

            status_a, new_node_a = self._extend(tree_start, q_rand)
            if status_a != "TRAPPED" and new_node_a is not None:
                status_b, new_node_b = self._connect(tree_goal, new_node_a.config)
                if status_b == "REACHED" and new_node_b is not None:
                    # Trees connected! Reconstruct path
                    path_a = []
                    curr = new_node_a
                    while curr is not None:
                        path_a.append(curr.config)
                        curr = curr.parent
                    path_a.reverse()

                    path_b = []
                    curr = new_node_b
                    while curr is not None:
                        path_b.append(curr.config)
                        curr = curr.parent

                    full_path = path_a + path_b
                    solve_ms = (time.perf_counter() - start_time) * 1000.0
                    return PlanningResult(
                        success=True,
                        path=full_path,
                        iterations=iteration,
                        planning_time_ms=solve_ms,
                        path_length_joint_rad=compute_path_length(full_path),
                        reason="RRT-Connect connected successfully",
                    )

            # Swap trees
            tree_start, tree_goal = tree_goal, tree_start

        solve_ms = (time.perf_counter() - start_time) * 1000.0
        return PlanningResult(
            success=False,
            iterations=self.max_iterations,
            planning_time_ms=solve_ms,
            reason="Max iterations reached without connecting trees",
        )


def shortcut_path(
    path: List[np.ndarray],
    collision_checker: CollisionChecker,
    arm_joint_indices: Optional[List[int]] = None,
    max_attempts: int = 40,
    step_size_rad: float = 0.05,
    random_seed: Optional[int] = None,
) -> List[np.ndarray]:
    """Applies randomized straight-line shortcutting to smooth and shorten an RRT path.

    Args:
        path: List of joint configuration waypoints.
        collision_checker: CollisionChecker instance.
        arm_joint_indices: Controllable joint indices.
        max_attempts: Number of random shortcut attempts.
        step_size_rad: Collision checking resolution.
        random_seed: Optional random seed for reproducible smoothing.

    Returns:
        Smoothed and shortened waypoint list.
    """
    if len(path) <= 2:
        return list(path)

    smoothed = [np.array(p, copy=True) for p in path]
    rng = np.random.RandomState(random_seed)

    for _ in range(max_attempts):
        if len(smoothed) <= 2:
            break

        idx1 = rng.randint(0, len(smoothed) - 1)
        idx2 = rng.randint(0, len(smoothed))

        if idx1 > idx2:
            idx1, idx2 = idx2, idx1

        if idx2 - idx1 <= 1:
            continue

        # Check if direct line between smoothed[idx1] and smoothed[idx2] is collision-free
        is_free, _ = is_joint_path_collision_free(
            smoothed[idx1],
            smoothed[idx2],
            collision_checker,
            resolution_rad=step_size_rad,
            arm_joint_indices=arm_joint_indices,
        )

        if is_free:
            # Cut intermediate vertices
            smoothed = smoothed[: idx1 + 1] + smoothed[idx2 :]

    return smoothed
