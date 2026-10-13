"""Release test lifecycle cleanup for automatically started watchdogs."""
import pytest

from robotics.safety import GuardedRobotBackend


@pytest.fixture(autouse=True)
def stop_test_watchdog_threads(monkeypatch):
    guards = []
    original = GuardedRobotBackend.start_watchdog_monitor

    def start(guard, *args, **kwargs):
        if guard not in guards:
            guards.append(guard)
        return original(guard, *args, **kwargs)

    monkeypatch.setattr(GuardedRobotBackend, "start_watchdog_monitor", start)
    yield
    for guard in guards:
        guard.stop_watchdog_monitor()
