"""Headless test launch script for ros2_control_demo_example_3 RRBot.

Uses exclusively official installed ros2_control_demo_example_3 resources
(Xacro, controller YAML) to launch ros2_control_node, robot_state_publisher,
joint_state_broadcaster, and the selected forward command controller in CI
without GUI/RViz dependencies.
"""

import os
from typing import List

def generate_launch_description():
    """Generates ROS2 LaunchDescription for headless RRBot multi-interface."""
    import xacro
    from ament_index_python.packages import get_package_share_directory
    from launch import LaunchDescription
    from launch.actions import DeclareLaunchArgument
    from launch.substitutions import LaunchConfiguration
    from launch_ros.actions import Node

    # Locate installed example_3 package
    pkg_share = get_package_share_directory("ros2_control_demo_example_3")

    # Locate URDF / Xacro
    xacro_file = os.path.join(pkg_share, "urdf", "rrbot_system_multi_interface.urdf.xacro")
    if not os.path.exists(xacro_file):
        # Fallback search if named differently in specific package version
        urdf_dir = os.path.join(pkg_share, "urdf")
        candidates = [os.path.join(urdf_dir, f) for f in os.listdir(urdf_dir) if f.endswith(".xacro")]
        if candidates:
            xacro_file = candidates[0]
        else:
            raise FileNotFoundError(f"No xacro file found in {urdf_dir}")

    doc = xacro.process_file(xacro_file)
    robot_description = {"robot_description": doc.toxml()}

    # Locate Controller Config YAML
    config_dir = os.path.join(pkg_share, "config")
    yaml_candidates = [
        os.path.join(config_dir, "rrbot_multi_interface_forward_controllers.yaml"),
        os.path.join(config_dir, "rrbot_controllers.yaml"),
    ]
    robot_controllers = None
    for candidate in yaml_candidates:
        if os.path.exists(candidate):
            robot_controllers = candidate
            break
    if robot_controllers is None:
        yaml_files = [os.path.join(config_dir, f) for f in os.listdir(config_dir) if f.endswith(".yaml")]
        if yaml_files:
            robot_controllers = yaml_files[0]
        else:
            raise FileNotFoundError(f"No controller YAML found in {config_dir}")

    # Launch arguments
    declared_arguments: List[DeclareLaunchArgument] = [
        DeclareLaunchArgument(
            "robot_controller",
            default_value="forward_position_controller",
            description="Robot controller to spawn and activate (e.g. forward_position_controller or forward_velocity_controller)",
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
