"""Unit tests for CommandSafety schemas, state models, and configuration."""

import pytest
from robotics.safety import (
    CommandSafetyConfig,
    CommandSafetyFault,
    CommandSafetyFaultCode,
    CommandSafetyViolationError,
    CommandWatchdogTimeoutError,
    SafetyGuardState,
    SoftwareCommandReadinessReport,
)


def test_safety_guard_states():
    """Verify SafetyGuardState members."""
    assert SafetyGuardState.DISARMED.value == "disarmed"
    assert SafetyGuardState.ARMED.value == "armed"
    assert SafetyGuardState.FAULT_LATCHED.value == "fault_latched"


def test_command_safety_fault_codes():
    """Verify CommandSafetyFaultCode members."""
    expected = {
        "NONE",
        "STATE_UNAVAILABLE",
        "STATE_STALE",
        "STATE_INVALID",
        "MODEL_MISMATCH",
        "BACKEND_UNHEALTHY",
        "COMMAND_TRANSPORT_UNAVAILABLE",
        "POSITION_LIMIT_VIOLATION",
        "POSITION_STEP_VIOLATION",
        "VELOCITY_LIMIT_VIOLATION",
        "COMMAND_DISPATCH_FAILED",
        "COMMAND_WATCHDOG_TIMEOUT",
        "COMMAND_WATCHDOG_HALT_FAILED",
        "SOFTWARE_STOP_FAILED",
        "READINESS_FAILED",
    }
    actual = {code.name for code in CommandSafetyFaultCode}
    assert actual == expected


def test_command_safety_config_defaults_and_immutability():
    """Verify CommandSafetyConfig default values and frozen immutability."""
    config = CommandSafetyConfig()
    assert config.state_timeout_s == 0.5
    assert config.command_watchdog_timeout_s == 0.2
    assert config.position_limit_margin_rad == 0.0
    assert config.velocity_limit_scale == 1.0
    assert config.max_position_step_by_joint is None
    assert config.require_velocity_feedback_for_velocity_commands is False
    assert config.watchdog_enabled is True

    with pytest.raises(Exception):
        config.state_timeout_s = 1.0  # dataclass frozen


def test_software_command_readiness_report():
    """Verify SoftwareCommandReadinessReport structure and fields."""
    report = SoftwareCommandReadinessReport(
        backend_connected=True,
        backend_health_ok=True,
        state_available=True,
        state_fresh=True,
        model_match=True,
        command_mode_supported=True,
        halt_supported=True,
        transport_ready=True,
        external_readiness_ok=True,
        fault_clear=True,
        ready_to_arm=True,
        reasons=(),
    )
    assert report.ready_to_arm is True
    assert report.backend_connected is True
