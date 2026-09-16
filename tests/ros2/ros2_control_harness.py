"""Deterministic test harness for launching and lifecycle-managing ros2_control demo environments.

Used for Phase B3.5 real controller-level validation on ROS2 Humble.
"""

import os
import signal
import subprocess
import tempfile
import time
from typing import List, Optional, Tuple

_HAS_ROS2: bool = False
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState as ROSJointState
    from controller_manager_msgs.srv import ListControllers
    _HAS_ROS2 = True
except ImportError:
    pass


class ROS2ControlHarness:
    """Manages the subprocess lifecycle and readiness probing of a real ros2_control simulation node."""

    def __init__(
        self,
        robot_controller: str = "forward_position_controller",
        domain_id: Optional[int] = None,
        launch_timeout_s: float = 30.0,
    ) -> None:
        self.robot_controller = robot_controller
        self.domain_id = domain_id
        self.launch_timeout_s = launch_timeout_s
        self.process: Optional[subprocess.Popen] = None
        self._stdout_file = None
        self._stderr_file = None
        self._prev_domain_id: Optional[str] = None
        self._observed_controller_type: Optional[str] = None

    @property
    def observed_controller_type(self) -> Optional[str]:
        """Returns the exact type reported by ListControllers for the spawned controller."""
        return self._observed_controller_type

    def start(self) -> None:
        """Launches the ros2_control headless node and waits deterministically for full readiness."""
        if not _HAS_ROS2:
            raise RuntimeError("ROS2 Humble (rclpy) is required to run ROS2ControlHarness.")

        current_dir = os.path.dirname(os.path.abspath(__file__))
        launch_file = os.path.join(current_dir, "launch_rrbot_headless.py")

        cmd = [
            "ros2",
            "launch",
            launch_file,
            f"robot_controller:={self.robot_controller}",
        ]

        # Configure environment and align DOMAIN_ID
        if self.domain_id is not None:
            self._prev_domain_id = os.environ.get("ROS_DOMAIN_ID")
            os.environ["ROS_DOMAIN_ID"] = str(self.domain_id)

        env = os.environ.copy()
        env["PYTHONUNBUFFERED"] = "1"
        env["QT_QPA_PLATFORM"] = "offscreen"
        if self.domain_id is not None:
            env["ROS_DOMAIN_ID"] = str(self.domain_id)

        # Temporary files for process stdout/stderr to avoid 64KB pipe buffer deadlocks
        self._stdout_file = tempfile.NamedTemporaryFile(mode="w+", delete=False, prefix="ros2_ctrl_stdout_")
        self._stderr_file = tempfile.NamedTemporaryFile(mode="w+", delete=False, prefix="ros2_ctrl_stderr_")

        kwargs = {
            "stdout": self._stdout_file,
            "stderr": self._stderr_file,
            "env": env,
        }
        if os.name != "nt":
            kwargs["preexec_fn"] = os.setsid
        else:
            kwargs["creationflags"] = subprocess.CREATE_NEW_PROCESS_GROUP

        self.process = subprocess.Popen(cmd, **kwargs)

        # Wait for full deterministic readiness
        try:
            self._wait_for_readiness()
        except Exception:
            self.stop()
            raise

    def _wait_for_readiness(self) -> None:
        """Polls controller manager services and joint_states until all readiness conditions pass."""
        t_start = time.monotonic()

        context = rclpy.Context()
        rclpy.init(context=context)
        node = Node("harness_readiness_probe", context=context)

        received_states: List[ROSJointState] = []

        def _state_cb(msg: ROSJointState) -> None:
            received_states.append(msg)

        sub = node.create_subscription(ROSJointState, "/joint_states", _state_cb, 10)
        client = node.create_client(ListControllers, "/controller_manager/list_controllers")

        try:
            # 1. Wait for list_controllers service
            service_ready = False
            while time.monotonic() - t_start < self.launch_timeout_s:
                if self.process.poll() is not None:
                    logs = self.get_logs()
                    raise RuntimeError(
                        f"ros2_control process exited prematurely with code {self.process.returncode}.\n{logs}"
                    )
                if client.wait_for_service(timeout_sec=0.2):
                    service_ready = True
                    break
                time.sleep(0.1)

            if not service_ready:
                logs = self.get_logs()
                raise TimeoutError(
                    f"controller_manager/list_controllers service not available within {self.launch_timeout_s}s.\n{logs}"
                )

            # 2. Wait for joint_state_broadcaster and robot_controller to be active
            controllers_active = False
            while time.monotonic() - t_start < self.launch_timeout_s:
                req = ListControllers.Request()
                future = client.call_async(req)
                rclpy.spin_until_future_complete(node, future, timeout_sec=1.0)

                if future.done() and future.exception() is None:
                    resp = future.result()
                    jsb_active = False
                    target_active = False
                    for c in resp.controller:
                        if c.name == "joint_state_broadcaster" and c.state.lower() == "active":
                            jsb_active = True
                        if c.name == self.robot_controller and c.state.lower() == "active":
                            target_active = True
                            self._observed_controller_type = c.type

                    if jsb_active and target_active:
                        controllers_active = True
                        break

                time.sleep(0.2)

            if not controllers_active:
                logs = self.get_logs()
                raise TimeoutError(
                    f"Controllers joint_state_broadcaster and '{self.robot_controller}' did not become 'active' "
                    f"within {self.launch_timeout_s}s.\n{logs}"
                )

            # 3. Wait for initial valid /joint_states message
            state_received = False
            while time.monotonic() - t_start < self.launch_timeout_s:
                rclpy.spin_once(node, timeout_sec=0.1)
                if received_states:
                    last_msg = received_states[-1]
                    if "joint1" in last_msg.name and "joint2" in last_msg.name:
                        state_received = True
                        break
                time.sleep(0.05)

            if not state_received:
                logs = self.get_logs()
                raise TimeoutError(
                    f"Initial valid /joint_states not received within {self.launch_timeout_s}s.\n{logs}"
                )

        finally:
            node.destroy_subscription(sub)
            node.destroy_client(client)
            node.destroy_node()
            if context.ok():
                rclpy.shutdown(context=context)

    def get_logs(self) -> str:
        """Reads captured stdout and stderr log files."""
        stdout_txt, stderr_txt = "", ""
        if self._stdout_file:
            try:
                self._stdout_file.flush()
                with open(self._stdout_file.name, "r", errors="replace") as f:
                    stdout_txt = f.read()
            except Exception:
                pass
        if self._stderr_file:
            try:
                self._stderr_file.flush()
                with open(self._stderr_file.name, "r", errors="replace") as f:
                    stderr_txt = f.read()
            except Exception:
                pass
        return f"=== STDOUT ===\n{stdout_txt}\n=== STDERR ===\n{stderr_txt}"

    def stop(self, timeout_s: float = 5.0) -> Tuple[str, str]:
        """Gracefully terminates the ros2_control process group with SIGTERM, falling back to SIGKILL."""
        if self.process is not None and self.process.poll() is None:
            try:
                if os.name != "nt":
                    os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
                else:
                    self.process.terminate()
            except Exception:
                pass

            t_term = time.monotonic()
            while time.monotonic() - t_term < timeout_s:
                if self.process.poll() is not None:
                    break
                time.sleep(0.1)

            if self.process.poll() is None:
                try:
                    if os.name != "nt":
                        os.killpg(os.getpgid(self.process.pid), signal.SIGKILL)
                    else:
                        self.process.kill()
                except Exception:
                    pass

        # Restore previous DOMAIN_ID
        if self.domain_id is not None:
            if self._prev_domain_id is not None:
                os.environ["ROS_DOMAIN_ID"] = self._prev_domain_id
            else:
                os.environ.pop("ROS_DOMAIN_ID", None)

        logs = self.get_logs()
        # Clean up temporary log files
        for tmp_file in [self._stdout_file, self._stderr_file]:
            if tmp_file:
                try:
                    tmp_file.close()
                    os.remove(tmp_file.name)
                except Exception:
                    pass
        self._stdout_file = None
        self._stderr_file = None

        return logs, ""

    def __enter__(self) -> "ROS2ControlHarness":
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.stop()
