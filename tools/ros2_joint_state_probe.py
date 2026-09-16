#!/usr/bin/env python3
"""Read-only CLI diagnostic probe for monitoring live ROS2 joint state telemetry.

This tool attaches ROS2JointStateBackend to a specified ROS topic and displays
real-time joint positions, velocities, efforts, message frequencies, freshness,
and health status.

SAFETY GUARANTEE:
This tool is strictly read-only. It has zero capability to transmit robot commands,
activate controllers, move joints, or manipulate hardware.
"""

import argparse
import sys
import time
from typing import Optional

from robotics.backends.base import (
    BackendError,
    BackendHealthStatus,
    BackendStateFieldUnavailableError,
    BackendStateStaleError,
    BackendStateUnavailableError,
    OptionalDependencyError,
)
from robotics.backends.ros2_state_mapping import ROS2JointStateBackendConfig
from robotics.robot_registry import get_robot_registry


def create_ros2_backend(config: ROS2JointStateBackendConfig):
    """Factory helper to construct ROS2JointStateBackend."""
    from robotics.backends.ros2_joint_state_backend import ROS2JointStateBackend
    return ROS2JointStateBackend(config=config)


def build_parser() -> argparse.ArgumentParser:
    """Builds argument parser for read-only joint state probe."""
    parser = argparse.ArgumentParser(
        description="Read-only diagnostic probe for ROS2 joint state telemetry.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--robot",
        type=str,
        default="panda",
        help="Robot identifier registered in robot registry (e.g. 'panda', 'kuka_iiwa').",
    )
    parser.add_argument(
        "--topic",
        type=str,
        default="/joint_states",
        help="ROS2 JointState topic to subscribe to.",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=1.0,
        help="Telemetry freshness timeout window in seconds.",
    )
    parser.add_argument(
        "--qos",
        type=str,
        choices=["best_effort", "reliable"],
        default="best_effort",
        help="ROS2 subscriber QoS reliability profile.",
    )
    parser.add_argument(
        "--depth",
        type=int,
        default=5,
        help="ROS2 subscriber QoS history depth.",
    )
    parser.add_argument(
        "--domain-id",
        type=int,
        default=None,
        help="Optional ROS_DOMAIN_ID override.",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=0.2,
        help="Diagnostic print refresh interval in seconds.",
    )
    parser.add_argument(
        "--max-samples",
        type=int,
        default=None,
        help="Optional maximum number of samples to display before exiting.",
    )
    return parser


def run_probe(
    robot_id: str = "panda",
    topic: str = "/joint_states",
    timeout_s: float = 1.0,
    qos_reliability: str = "best_effort",
    qos_depth: int = 5,
    domain_id: Optional[int] = None,
    interval_s: float = 0.2,
    max_samples: Optional[int] = None,
) -> int:
    """Runs the live joint state diagnostic probe loop."""
    registry = get_robot_registry()
    try:
        spec = registry.get_robot_spec(robot_id)
    except KeyError as err:
        print(f"[ERROR] {err}", file=sys.stderr)
        return 1

    # Extract expected arm joint names from spec
    expected_names = tuple(spec.arm_joint_name_patterns)
    # If patterns are generic, attempt resolved model or arm names
    if hasattr(spec, "arm_joint_names") and spec.arm_joint_names:
        expected_names = tuple(spec.arm_joint_names)
    elif robot_id == "panda":
        expected_names = tuple(f"panda_joint{i+1}" for i in range(7))
    elif robot_id == "kuka_iiwa":
        expected_names = tuple(f"lbr_iiwa_joint_{i+1}" for i in range(7))

    config = ROS2JointStateBackendConfig(
        expected_joint_names=expected_names,
        joint_state_topic=topic,
        state_timeout_s=timeout_s,
        qos_reliability=qos_reliability,
        qos_depth=qos_depth,
        domain_id=domain_id,
    )

    try:
        backend = create_ros2_backend(config)
    except OptionalDependencyError as err:
        print(f"[ERROR] ROS2 backend unavailable: {err}", file=sys.stderr)
        return 1

    print(f"======================================================================")
    print(f"  VISION-ROBOT-TWIN READ-ONLY ROS2 JOINT STATE PROBE")
    print(f"  Robot        : {spec.display_name} ({robot_id})")
    print(f"  Topic        : {topic}")
    print(f"  QoS          : {qos_reliability} (depth={qos_depth})")
    print(f"  Timeout      : {timeout_s:.2f}s")
    print(f"  Expected DoF : {len(expected_names)} {expected_names}")
    print(f"======================================================================")
    print(f"Connecting to ROS2 graph...")

    if not backend.connect():
        print("[ERROR] Failed to initialize ROS2 context and node.", file=sys.stderr)
        return 1

    sample_count = 0
    try:
        while True:
            health = backend.health_status()
            diag = backend.diagnostics()

            try:
                state = backend.get_joint_state()
                pos_str = ", ".join(f"{p:+.3f}" for p in state.positions)
                if state.velocities is not None:
                    vel_str = ", ".join(f"{v:+.3f}" for v in state.velocities)
                else:
                    vel_str = "N/A"
                age_ms = state.age_s(time.monotonic()) * 1000.0
                status_line = (
                    f"[{health.name}] seq={state.sequence_id:04d} age={age_ms:5.1f}ms "
                    f"rx={diag.messages_received} valid={diag.valid_messages} "
                    f"pos=[{pos_str}] vel=[{vel_str}]"
                )
            except BackendStateUnavailableError:
                status_line = f"[{health.name}] rx={diag.messages_received} err={diag.last_validation_error or 'Waiting for first message...'}"
            except BackendStateStaleError as stale_err:
                status_line = f"[{health.name}] STALE: {stale_err}"

            print(status_line)
            sample_count += 1
            if max_samples is not None and sample_count >= max_samples:
                break

            time.sleep(interval_s)
    except KeyboardInterrupt:
        print("\nProbe interrupted by user. Disconnecting...")
    finally:
        backend.disconnect()
        print("Backend disconnected cleanly.")

    return 0


def main():
    parser = build_parser()
    args = parser.parse_args()
    sys.exit(
        run_probe(
            robot_id=args.robot,
            topic=args.topic,
            timeout_s=args.timeout,
            qos_reliability=args.qos,
            qos_depth=args.depth,
            domain_id=args.domain_id,
            interval_s=args.interval,
            max_samples=args.max_samples,
        )
    )


if __name__ == "__main__":
    main()
