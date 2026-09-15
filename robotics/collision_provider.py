"""Abstract Collision Provider Boundary and Core Result Models.

Defines the abstract interface for collision detection, self-collision querying,
and clearance evaluation across interchangeable robot backends and simulation environments.
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple
import numpy as np

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
