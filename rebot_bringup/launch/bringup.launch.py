#!/usr/bin/env python3
# Copyright 2026 Eaglewzw
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==============================================================================
# reBot B601-DM bringup (mock or real hardware)
#
# Launches the full ros2_control pipeline: controller_manager with the
# rebot hardware plugin (or mock components), joint state broadcasting,
# trajectory + gripper controllers, and RViz visualisation.
#
# Usage:
#   Mock (default):  ros2 launch rebot_bringup bringup.launch.py
#   Real hardware:   ros2 launch rebot_bringup bringup.launch.py \
#                        use_mock_hardware:=false serial_port:=/dev/ttyACM0
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
    use_rviz = LaunchConfiguration("use_rviz")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    serial_port = LaunchConfiguration("serial_port")

    declared_arguments = [
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Launch RViz2 for visualisation",
        ),
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="true",
            description="true: mock_components simulation; "
            "false: real arm via Damiao USB-CAN bridge",
        ),
        DeclareLaunchArgument(
            "serial_port",
            default_value="/dev/ttyACM0",
            description="Damiao USB-CAN serial bridge device (real hardware only)",
        ),
    ]

    # ---- Package paths ----
    pkg_bringup = FindPackageShare("rebot_bringup")
    pkg_description = FindPackageShare("rebot_description")

    xacro_path = PathJoinSubstitution(
        [pkg_description, "urdf", "rebot_b601_dm.urdf.xacro"]
    )
    rviz_config_path = PathJoinSubstitution([pkg_description, "rviz", "rebot.rviz"])
    controllers_path = PathJoinSubstitution(
        [pkg_bringup, "config", "ros2_control_controllers.yaml"]
    )

    # ---- Robot description ----
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            xacro_path,
            " use_mock_hardware:=",
            use_mock_hardware,
            " serial_port:=",
            serial_port,
        ]
    )
    robot_description_param = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # ==========================================================================
    # 1. ros2_control_node
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
    # 2. robot_state_publisher
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

    # Custom MIT-mode controllers, loaded inactive. Activate exactly one arm
    # controller at a time, e.g.:
    #   ros2 control switch_controllers \
    #       --deactivate joint_trajectory_controller \
    #       --activate gravity_compensation_controller
    custom_controllers_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "mit_joint_controller",
            "gravity_compensation_controller",
            "mit_trajectory_controller",
            "joint_impedance_controller",
            "teleop_stream_controller",
            "--inactive",
        ],
        output="both",
    )

    # ==========================================================================
    # 4. RViz2
    # ==========================================================================
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config_path, "-f", "world"],
        condition=IfCondition(use_rviz),
        output="both",
    )

    return LaunchDescription(
        declared_arguments
        + [
            controller_manager_node,
            robot_state_publisher_node,
            joint_state_spawner,
            joint_trajectory_spawner,
            gripper_spawner,
            custom_controllers_spawner,
            rviz_node,
        ]
    )
