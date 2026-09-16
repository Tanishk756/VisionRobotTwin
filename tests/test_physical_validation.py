"""Unit tests for Limit Provenance Engine and interval intersection logic."""

import pytest

from robotics.backends.physical_validation import (
    JointLimitSource,
    LimitProvenanceEngine,
    LimitSourceKind,
    ResolvedJointLimit,
)


class TestLimitProvenanceEngine:
    """Tests for interval intersection and velocity bounds resolution across limit sources."""

    def test_nominal_interval_intersection(self):
        engine = LimitProvenanceEngine()
        src_mfg = JointLimitSource(
            source_name="mfg_datasheet",
            source_kind=LimitSourceKind.MANUFACTURER_SPEC,
            position_lower=-2.0,
            position_upper=2.0,
            max_velocity=2.5,
        )
        src_driver = JointLimitSource(
            source_name="driver_param",
            source_kind=LimitSourceKind.DRIVER_ACTIVE,
            position_lower=-1.5,
            position_upper=1.8,
            max_velocity=1.5,
        )
        src_urdf = JointLimitSource(
            source_name="urdf_model",
            source_kind=LimitSourceKind.RESOLVED_ROBOT_MODEL,
            position_lower=-1.8,
            position_upper=1.9,
            max_velocity=2.0,
        )

        res = engine.resolve_joint_limits("joint1", [src_mfg, src_driver, src_urdf])
        assert res.is_valid is True
        # max(-2.0, -1.5, -1.8) = -1.5
        assert res.effective_lower == pytest.approx(-1.5)
        # min(2.0, 1.8, 1.9) = 1.8
        assert res.effective_upper == pytest.approx(1.8)
        # min(2.5, 1.5, 2.0) = 1.5
        assert res.effective_max_velocity == pytest.approx(1.5)
        assert len(res.rejection_reasons) == 0

    def test_inconsistent_disjoint_limits_marked_invalid(self):
        engine = LimitProvenanceEngine()
        src1 = JointLimitSource(
            source_name="source_a",
            source_kind=LimitSourceKind.MANUFACTURER_SPEC,
            position_lower=-1.0,
            position_upper=0.0,
            max_velocity=1.0,
        )
        src2 = JointLimitSource(
            source_name="source_b",
            source_kind=LimitSourceKind.DRIVER_ACTIVE,
            position_lower=0.5,
            position_upper=1.0,
            max_velocity=1.0,
        )

        res = engine.resolve_joint_limits("joint1", [src1, src2])
        assert res.is_valid is False
        assert any("Inconsistent position limits" in r for r in res.rejection_reasons)

    def test_inverted_source_limits_rejected(self):
        engine = LimitProvenanceEngine()
        src = JointLimitSource(
            source_name="bad_source",
            source_kind=LimitSourceKind.DRIVER_ACTIVE,
            position_lower=2.0,
            position_upper=-2.0,  # inverted
            max_velocity=1.0,
        )
        res = engine.resolve_joint_limits("joint1", [src])
        assert res.is_valid is False
        assert any("inverted" in r.lower() for r in res.rejection_reasons)

    def test_nan_or_inf_limits_rejected(self):
        engine = LimitProvenanceEngine()
        src_nan = JointLimitSource(
            source_name="nan_source",
            source_kind=LimitSourceKind.DRIVER_ACTIVE,
            position_lower=float("nan"),
            position_upper=1.0,
            max_velocity=1.0,
        )
        res = engine.resolve_joint_limits("joint1", [src_nan])
        assert res.is_valid is False
        assert any("Non-finite" in r for r in res.rejection_reasons)

    def test_non_positive_velocity_rejected(self):
        engine = LimitProvenanceEngine()
        src_zero_vel = JointLimitSource(
            source_name="zero_vel_source",
            source_kind=LimitSourceKind.DRIVER_ACTIVE,
            position_lower=-1.0,
            position_upper=1.0,
            max_velocity=0.0,
        )
        res = engine.resolve_joint_limits("joint1", [src_zero_vel])
        assert res.is_valid is False
        assert any("max_velocity must be positive" in r for r in res.rejection_reasons)

    def test_empty_sources_marked_invalid(self):
        engine = LimitProvenanceEngine()
        res = engine.resolve_joint_limits("joint1", [])
        assert res.is_valid is False
        assert any("No limit sources provided" in r for r in res.rejection_reasons)

    def test_resolve_multi_joint_robot_limits(self):
        engine = LimitProvenanceEngine()
        sources = {
            "joint1": [
                JointLimitSource("mfg", LimitSourceKind.MANUFACTURER_SPEC, -1.0, 1.0, 2.0),
                JointLimitSource("driver", LimitSourceKind.DRIVER_ACTIVE, -0.8, 0.8, 1.5),
            ],
            "joint2": [
                JointLimitSource("mfg", LimitSourceKind.MANUFACTURER_SPEC, -2.0, 2.0, 3.0),
                JointLimitSource("driver", LimitSourceKind.DRIVER_ACTIVE, -1.5, 1.5, 2.0),
            ],
        }
        all_valid, resolved_dict, reasons = engine.resolve_robot_limits(sources)
        assert all_valid is True
        assert len(resolved_dict) == 2
        assert resolved_dict["joint1"].effective_upper == pytest.approx(0.8)
        assert resolved_dict["joint2"].effective_upper == pytest.approx(1.5)
        assert len(reasons) == 0
