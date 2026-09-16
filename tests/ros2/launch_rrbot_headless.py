"""Headless test launch script for ros2_control RRBot multi-interface simulation.

Uses ros2_control_node, robot_state_publisher, joint_state_broadcaster,
and forward command controllers with the official mock_components/GenericSystem
plugin from ros2_control.
"""

import os
from typing import List


def generate_launch_description():
    """Generates ROS2 LaunchDescription for headless RRBot multi-interface."""
    import xacro
    from launch import LaunchDescription
    from launch.actions import DeclareLaunchArgument
    from launch.substitutions import LaunchConfiguration
    from launch_ros.actions import Node

    current_dir = os.path.dirname(os.path.abspath(__file__))

    # 1. Locate URDF
    urdf_file = os.path.join(current_dir, "rrbot_system_multi_interface.urdf")
    if not os.path.exists(urdf_file):
        raise FileNotFoundError(f"RRBot URDF file not found at {urdf_file}")

    with open(urdf_file, "r", encoding="utf-8") as f:
        robot_desc_str = f.read()
    robot_description = {"robot_description": robot_desc_str}

    # 2. Locate Controller Config YAML
    robot_controllers = os.path.join(current_dir, "rrbot_controllers.yaml")
    if not os.path.exists(robot_controllers):
        raise FileNotFoundError(f"RRBot controllers YAML not found at {robot_controllers}")

    # Launch arguments
    declared_arguments: List[DeclareLaunchArgument] = [
        DeclareLaunchArgument(
            "robot_controller",
            default_value="forward_position_controller",
            description="Robot controller to spawn and activate (forward_position_controller or forward_velocity_controller)",
        ),
    ]

    robot_controller = LaunchConfiguration("robot_controller")

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="screen",
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[robot_controller, "--controller-manager", "/controller_manager"],
        output="screen",
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)
