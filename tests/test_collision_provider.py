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
