"""Limit provenance and interval intersection engine for physical robot commissioning.

Phase B4.1 vendor-neutral framework for gathering, intersecting, and validating
position and velocity limits across multiple authoritative sources (manufacturer,
active controller drivers, URDF digital twin, commissioning profile).
"""

from dataclasses import dataclass
from enum import Enum, auto
import math
from typing import Dict, List, Optional, Sequence, Tuple


class LimitSourceKind(Enum):
    """Categorized source of robot physical joint limits."""

    MANUFACTURER_SPEC = auto()
    DRIVER_ACTIVE = auto()
    COMMISSIONING_PROFILE = auto()
    RESOLVED_ROBOT_MODEL = auto()
    OTHER = auto()


@dataclass(frozen=True)
class JointLimitSource:
    """Individual declaration of joint limits from an identifiable source."""

    source_name: str
    source_kind: LimitSourceKind
    position_lower: float
    position_upper: float
    max_velocity: float
    version_or_reference: Optional[str] = None
    trusted_for_command: bool = True


@dataclass(frozen=True)
class ResolvedJointLimit:
    """Consolidated joint limit envelope computed via interval intersection.

    Attributes:
        joint_name: Name of the evaluated joint.
        effective_lower: Maximum of valid lower position bounds.
        effective_upper: Minimum of valid upper position bounds.
        effective_max_velocity: Minimum of valid positive max velocity limits.
        is_valid: True if bounds are finite, non-inverted, and non-empty.
        sources_used: Tuple of source names included in resolution.
        rejection_reasons: Tuple of error or diagnostic reasons if invalid.
    """

    joint_name: str
    effective_lower: float
    effective_upper: float
    effective_max_velocity: float
    is_valid: bool
    sources_used: Tuple[str, ...]
    rejection_reasons: Tuple[str, ...]


class LimitProvenanceEngine:
    """Computes effective safe motion envelopes using interval intersection."""

    def resolve_joint_limits(
        self,
        joint_name: str,
        sources: Sequence[JointLimitSource],
    ) -> ResolvedJointLimit:
        """Computes effective position and velocity limits across supplied sources.

        Position rule:
            effective_lower = max(lower_bounds)
            effective_upper = min(upper_bounds)
            Valid only if effective_lower <= effective_upper.
        Velocity rule:
            effective_max_velocity = min(max_velocities)
            Valid only if effective_max_velocity > 0.
        """
        reasons: List[str] = []
        sources_used: List[str] = []

        if not sources:
            return ResolvedJointLimit(
                joint_name=joint_name,
                effective_lower=float("nan"),
                effective_upper=float("nan"),
                effective_max_velocity=float("nan"),
                is_valid=False,
                sources_used=(),
                rejection_reasons=("No limit sources provided.",),
            )

        lower_bounds: List[float] = []
        upper_bounds: List[float] = []
        vel_limits: List[float] = []

        for src in sources:
            sources_used.append(src.source_name)

            if not math.isfinite(src.position_lower) or not math.isfinite(src.position_upper):
                reasons.append(
                    f"Non-finite position limits in source '{src.source_name}': [{src.position_lower}, {src.position_upper}]"
                )
                continue

            if src.position_lower > src.position_upper:
                reasons.append(
                    f"Inverted position limits in source '{src.source_name}': lower {src.position_lower} > upper {src.position_upper}"
                )
                continue

            if not math.isfinite(src.max_velocity) or src.max_velocity <= 0.0:
                reasons.append(
                    f"max_velocity must be positive finite in source '{src.source_name}', got {src.max_velocity}"
                )
                continue

            lower_bounds.append(src.position_lower)
            upper_bounds.append(src.position_upper)
            vel_limits.append(src.max_velocity)

        if len(lower_bounds) == 0:
            return ResolvedJointLimit(
                joint_name=joint_name,
                effective_lower=float("nan"),
                effective_upper=float("nan"),
                effective_max_velocity=float("nan"),
                is_valid=False,
                sources_used=tuple(sources_used),
                rejection_reasons=tuple(reasons) if reasons else ("No valid limit sources.",),
            )

        eff_lower = max(lower_bounds)
        eff_upper = min(upper_bounds)
        eff_vel = min(vel_limits)

        if eff_lower > eff_upper:
            reasons.append(
                f"Inconsistent position limits across sources: effective lower {eff_lower:.4f} > effective upper {eff_upper:.4f}"
            )

        is_valid = len(reasons) == 0

        return ResolvedJointLimit(
            joint_name=joint_name,
            effective_lower=eff_lower if is_valid else float("nan"),
            effective_upper=eff_upper if is_valid else float("nan"),
            effective_max_velocity=eff_vel if is_valid else float("nan"),
            is_valid=is_valid,
            sources_used=tuple(sources_used),
            rejection_reasons=tuple(reasons),
        )

    def resolve_robot_limits(
        self,
        sources_by_joint: Dict[str, Sequence[JointLimitSource]],
    ) -> Tuple[bool, Dict[str, ResolvedJointLimit], Tuple[str, ...]]:
        """Resolves limits for all joints of a robot manipulator."""
        all_valid = True
        resolved: Dict[str, ResolvedJointLimit] = {}
        all_reasons: List[str] = []

        for j_name, j_sources in sources_by_joint.items():
            res = self.resolve_joint_limits(j_name, j_sources)
            resolved[j_name] = res
            if not res.is_valid:
                all_valid = False
                for r in res.rejection_reasons:
                    all_reasons.append(f"Joint '{j_name}': {r}")

        return all_valid, resolved, tuple(all_reasons)
