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
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, LogInfo
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import Node
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
    controllers_path = PathJoinSubstitution(
        [pkg_bringup, "config", "ros2_control_controllers.yaml"]
    )

    # ---- Robot description (plain URDF, xacro passthrough-safe) ----
    robot_description = {
        "robot_description": Command([FindExecutable(name="xacro"), " ", urdf_path])
    }

    # ==========================================================================
    # 1. ros2_control_node — controller_manager + hardware interface host
    # ==========================================================================
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers_path],
        output="both",
        # The node name doubles as the controller_manager service namespace.
        # Spawners below use `--controller-manager controller_manager` (the default).
        name="controller_manager",
    )

    # ==========================================================================
    # 2. robot_state_publisher — URDF → TF tree (reads /joint_states)
    # ==========================================================================
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="both",
    )

    # ==========================================================================
    # 3. Controller spawners — load + configure + activate via controller_manager
    #
    #    joint_state_broadcaster:  reads hardware → /joint_states  (auto_start)
    #    joint_trajectory_controller: MoveIt2 / action goal entry  (--activate)
    #    gripper_controller:          group position for parallel gripper
    #
    #    Spawners wait for the controller_manager service before proceeding.
    # ==========================================================================

    # joint_state_broadcaster — publish-only, no goal interface needed
    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "controller_manager",
        ],
        output="both",
    )

    # joint_trajectory_controller — main arm motion (position + velocity FF)
    joint_trajectory_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_trajectory_controller",
            "--controller-manager", "controller_manager",
            "--activate",
        ],
        output="both",
    )

    # gripper_controller — single-driving-joint for parallel gripper
    gripper_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_controller",
            "--controller-manager", "controller_manager",
            "--activate",
        ],
        output="both",
    )

    # ==========================================================================
    # 4. RViz2 — visual feedback (RobotModel + TF)
    # ==========================================================================
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        condition=IfCondition(use_rviz),
        output="both",
    )

    # ==========================================================================
    # Assembly — all nodes launch concurrently; spawners auto-wait on services.
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
