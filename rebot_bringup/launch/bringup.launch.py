#!/usr/bin/env python3
# ==============================================================================
# reBot Mock-mode bringup
#
# Launches the full ros2_control pipeline with a virtual hardware interface,
# joint state broadcasting, trajectory controller, and RViz visualisation.
#
# Usage:
#   ros2 launch rebot_bringup bringup.launch.py
# ==============================================================================

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ---- Launch arguments ----
    use_rviz = LaunchConfiguration("use_rviz", default="true")

    declare_use_rviz = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Launch RViz2 for visualisation",
    )

    # ---- Package paths ----
    pkg_bringup = FindPackageShare("rebot_bringup")
    pkg_description = FindPackageShare("rebot_description")

    urdf_path = PathJoinSubstitution([pkg_description, "urdf", "rebot_b601_rs.urdf"])
    rviz_config_path = PathJoinSubstitution(
        [pkg_description, "rviz", "rebot.rviz"]
    )
    controllers_path = PathJoinSubstitution(
        [pkg_bringup, "config", "ros2_control_controllers.yaml"]
    )

    # ---- Robot description (plain URDF, xacro passthrough-safe) ----
    robot_description_content = Command([FindExecutable(name="xacro"), " ", urdf_path])
    robot_description_param = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # ==========================================================================
    # 1. ros2_control_node
    #    - robot_description passed via remap (matching official demo pattern)
    #    - controllers YAML passed as parameter file
    # ==========================================================================
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controllers_path],
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
        output="both",
    )

    # ==========================================================================
    # 2. robot_state_publisher — publishes URDF to /robot_description topic,
    #    then publishes TF tree from /joint_states
    # ==========================================================================
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description_param],
        output="both",
    )

    # ==========================================================================
    # 3. Controller spawners
    # ==========================================================================

    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="both",
    )

    joint_trajectory_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--activate"],
        output="both",
    )

    gripper_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--activate"],
        output="both",
    )

    # ==========================================================================
    # 4. RViz2
    # ==========================================================================
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            rviz_config_path,
            "-f",
            "world",
        ],
        condition=IfCondition(use_rviz),
        output="both",
    )

    # ==========================================================================
    # Assembly
    # ==========================================================================
    return LaunchDescription(
        [
            declare_use_rviz,
            controller_manager_node,
            robot_state_publisher_node,
            joint_state_spawner,
            joint_trajectory_spawner,
            gripper_spawner,
            rviz_node,
        ]
    )
