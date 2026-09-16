"""Traceability records and metadata logging for physical commissioning sessions.

Phase B4.1 schema recording immutable commissioning events, software commit hashes,
working tree integrity, hardware identity, and safety evaluation results.
"""

from dataclasses import asdict, dataclass
import re
from typing import Any, Dict, Optional, Tuple

from robotics.backends.physical_identity import PhysicalRobotIdentity

_SUSPICIOUS_SECRET_PATTERNS = [
    re.compile(r"password\s*=", re.IGNORECASE),
    re.compile(r"passwd\s*=", re.IGNORECASE),
    re.compile(r"secret\s*=", re.IGNORECASE),
    re.compile(r"api_key\s*=", re.IGNORECASE),
    re.compile(r"bearer\s+[a-zA-Z0-9_\-\.]+", re.IGNORECASE),
    re.compile(r"ssh-rsa\s+", re.IGNORECASE),
]


@dataclass(frozen=True)
class CommissioningSessionRecord:
    """Immutable audit record documenting a physical robot commissioning session.

    Attributes:
        session_id: Unique string session identifier.
        software_commit_sha: Exact git commit SHA under which session was run.
        working_tree_clean: True if git working tree was clean at session start.
        started_at_iso: ISO-8601 formatted session start timestamp.
        target_identity: Optional reported PhysicalRobotIdentity.
        provider_name: Class or driver identifier of the telemetry provider.
        resolved_model_id: Identifier of the ResolvedRobotModel used.
        commissioning_phase: Phase identifier (e.g. "B4.1_OBSERVATION").
        reasons_or_faults: Tuple of logged observations, reasons, or fault details.
        finished_at_iso: Optional ISO-8601 session finish timestamp.
        provider_version: Optional provider/driver version string.
        observation_summary: Optional dictionary of observation soak statistics.
        limit_provenance_summary: Optional dictionary of resolved limit envelopes.
        readiness_summary: Optional dictionary of readiness report metrics.
        operator_note: Optional human operator note (sanitized against secrets).
    """

    session_id: str
    software_commit_sha: str
    working_tree_clean: bool
    started_at_iso: str
    target_identity: Optional[PhysicalRobotIdentity]
    provider_name: str
    resolved_model_id: str
    commissioning_phase: str
    reasons_or_faults: Tuple[str, ...]
    finished_at_iso: Optional[str] = None
    provider_version: Optional[str] = None
    observation_summary: Optional[Dict[str, Any]] = None
    limit_provenance_summary: Optional[Dict[str, Any]] = None
    readiness_summary: Optional[Dict[str, Any]] = None
    operator_note: Optional[str] = None

    def __post_init__(self) -> None:
        if not self.session_id or not self.session_id.strip():
            raise ValueError("session_id cannot be empty.")
        if not self.software_commit_sha or not self.software_commit_sha.strip():
            raise ValueError("software_commit_sha cannot be empty.")

        if self.operator_note:
            for pat in _SUSPICIOUS_SECRET_PATTERNS:
                if pat.search(self.operator_note):
                    raise ValueError(
                        f"Potential secret or credential detected in operator_note: '{self.operator_note}'"
                    )

    def to_dict(self) -> Dict[str, Any]:
        """Serializes session record to a plain dictionary suitable for JSON export."""
        d = asdict(self)
        # Convert tuple to list for clean JSON serialization
        d["reasons_or_faults"] = list(self.reasons_or_faults)
        return d
