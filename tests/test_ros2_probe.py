"""Tests for the read-only ROS2 joint state diagnostic probe tool."""

import pytest
from unittest.mock import MagicMock, patch

from tools.ros2_joint_state_probe import build_parser, run_probe


def test_probe_argument_parser():
    """Verify probe CLI parser options, defaults, and absence of command flags."""
    parser = build_parser()

    args = parser.parse_args(["--robot", "panda", "--topic", "/my_joint_states", "--timeout", "2.0"])
    assert args.robot == "panda"
    assert args.topic == "/my_joint_states"
    assert args.timeout == 2.0
    assert args.qos == "best_effort"
    assert args.depth == 5
    assert args.domain_id is None
    assert args.max_samples is None

    # Verify no command-capable flags exist
    option_strings = [opt for action in parser._actions for opt in action.option_strings]
    assert "--command" not in option_strings
    assert "--target" not in option_strings
    assert "--velocity" not in option_strings
    assert "--halt" not in option_strings
    assert "--move" not in option_strings
    assert "--enable" not in option_strings


def test_probe_run_with_max_samples():
    """Verify run_probe connects, queries diagnostics and states, and exits cleanly after max_samples."""
    mock_backend = MagicMock()
    mock_backend.connect.return_value = True

    mock_diag = MagicMock()
    mock_diag.messages_received = 10
    mock_diag.valid_messages = 10
    mock_diag.invalid_messages = 0
    mock_diag.is_stale = False
    mock_diag.last_receive_monotonic_s = 100.0
    mock_backend.diagnostics.return_value = mock_diag

    mock_state = MagicMock()
    mock_state.sequence_id = 1
    mock_state.positions = (0.1, 0.2, 0.3)
    mock_state.velocities = (0.01, 0.02, 0.03)
    mock_state.efforts = None
    mock_state.age_s.return_value = 0.01
    mock_backend.get_joint_state.return_value = mock_state

    with patch("tools.ros2_joint_state_probe.create_ros2_backend", return_value=mock_backend):
        exit_code = run_probe(
            robot_id="panda",
            topic="/joint_states",
            timeout_s=1.0,
            qos_reliability="best_effort",
            qos_depth=5,
            domain_id=None,
            max_samples=2,
            interval_s=0.001,
        )
        assert exit_code == 0
        mock_backend.connect.assert_called_once()
        mock_backend.disconnect.assert_called_once()
        assert mock_backend.get_joint_state.call_count == 2
