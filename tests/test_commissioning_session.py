"""Unit tests for CommissioningSessionRecord traceability schemas."""

import pytest

from robotics.backends.commissioning_session import CommissioningSessionRecord
from robotics.backends.physical_identity import PhysicalRobotIdentity


class TestCommissioningSessionRecord:
    """Tests for CommissioningSessionRecord immutability and traceability."""

    def test_valid_session_record_creation(self):
        ident = PhysicalRobotIdentity(
            vendor="Franka",
            model="Panda",
            joint_names=("panda_j1",),
            joint_count=1,
            receive_timestamp_s=1.0,
        )
        rec = CommissioningSessionRecord(
            session_id="COMM-2026-001",
            software_commit_sha="0b91ce0491352f73ccc2492d629717be1b6e5ddd",
            working_tree_clean=True,
            started_at_iso="2026-09-16T12:00:00Z",
            target_identity=ident,
            provider_name="MockPhysicalTelemetryAdapter",
            provider_version="1.0.0",
            resolved_model_id="panda",
            commissioning_phase="B4.1_OBSERVATION",
            reasons_or_faults=(),
        )
        assert rec.session_id == "COMM-2026-001"
        assert rec.software_commit_sha == "0b91ce0491352f73ccc2492d629717be1b6e5ddd"
        assert rec.working_tree_clean is True
        assert rec.target_identity.vendor == "Franka"

    def test_empty_commit_sha_raises_value_error(self):
        with pytest.raises(ValueError, match="software_commit_sha cannot be empty"):
            CommissioningSessionRecord(
                session_id="COMM-001",
                software_commit_sha="",
                working_tree_clean=True,
                started_at_iso="2026-09-16T12:00:00Z",
                target_identity=None,
                provider_name="MockAdapter",
                provider_version=None,
                resolved_model_id="test",
                commissioning_phase="B4.1_OBSERVATION",
                reasons_or_faults=(),
            )

    def test_empty_session_id_raises_value_error(self):
        with pytest.raises(ValueError, match="session_id cannot be empty"):
            CommissioningSessionRecord(
                session_id="",
                software_commit_sha="abcdef",
                working_tree_clean=True,
                started_at_iso="2026-09-16T12:00:00Z",
                target_identity=None,
                provider_name="MockAdapter",
                provider_version=None,
                resolved_model_id="test",
                commissioning_phase="B4.1_OBSERVATION",
                reasons_or_faults=(),
            )

    def test_secret_detection_in_notes_raises_value_error(self):
        with pytest.raises(ValueError, match="Potential secret or credential detected"):
            CommissioningSessionRecord(
                session_id="COMM-001",
                software_commit_sha="abcdef",
                working_tree_clean=True,
                started_at_iso="2026-09-16T12:00:00Z",
                target_identity=None,
                provider_name="MockAdapter",
                provider_version=None,
                resolved_model_id="test",
                commissioning_phase="B4.1_OBSERVATION",
                reasons_or_faults=(),
                operator_note="Using robot password=SuperSecretPassword123!",
            )

    def test_to_dict_and_serialization(self):
        rec = CommissioningSessionRecord(
            session_id="COMM-002",
            software_commit_sha="1234567890abcdef",
            working_tree_clean=True,
            started_at_iso="2026-09-16T12:00:00Z",
            target_identity=None,
            provider_name="MockAdapter",
            provider_version="0.1.0",
            resolved_model_id="kuka_iiwa",
            commissioning_phase="B4.1_OBSERVATION",
            reasons_or_faults=("Readiness check nominal",),
        )
        d = rec.to_dict()
        assert d["session_id"] == "COMM-002"
        assert d["software_commit_sha"] == "1234567890abcdef"
        assert d["reasons_or_faults"] == ["Readiness check nominal"]
