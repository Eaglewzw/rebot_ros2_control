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
# reBot MoveIt2 demo launch (standalone, includes ros2_control + move_group)
#
# Usage:
#   Mock standalone:  ros2 launch rebot_moveit_config demo.launch.py
#   Real hardware:    ros2 launch rebot_moveit_config demo.launch.py use_mock_hardware:=false
# ==============================================================================

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    LogInfo,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    # ---- Launch arguments ----
    use_rviz = LaunchConfiguration("use_rviz")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    serial_port = LaunchConfiguration("serial_port")

    declared_arguments = [
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("use_mock_hardware", default_value="true"),
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyACM0"),
    ]

    # ---- URDF via xacro (single source of truth) ----
    xacro_path = PathJoinSubstitution([
        FindPackageShare("rebot_description"),
        "urdf", "rebot_b601_dm.urdf.xacro",
    ])
    robot_description_content = Command([
        FindExecutable(name="xacro"), " ",
        xacro_path,
        " use_mock_hardware:=", use_mock_hardware,
        " serial_port:=", serial_port,
    ])
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # ---- MoveIt2 configs (all except URDF, which is generated above) ----
    moveit_config = (
        MoveItConfigsBuilder("rebot_b601_dm", package_name="rebot_moveit_config")
        .robot_description_semantic()
        .robot_description_kinematics()
        .planning_pipelines()
        .trajectory_execution()
        .planning_scene_monitor()
        .joint_limits()
        .to_moveit_configs()
    )

    # Merge URDF into moveit_config for move_group
    move_group_params = moveit_config.to_dict()
    move_group_params["robot_description"] = robot_description["robot_description"]

    # ---- ros2_control_node (URDF as parameter, not topic, to avoid conflicts) ----
    controllers_path = PathJoinSubstitution([
        FindPackageShare("rebot_bringup"), "config", "ros2_control_controllers.yaml"
    ])
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controllers_path, robot_description],
        output="both",
    )
    control_node_exit_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=control_node,
            on_exit=[
                LogInfo(
                    msg="ERROR: ros2_control_node exited; shutting down MoveIt because "
                    "current robot state and trajectory execution are unavailable."
                ),
                EmitEvent(
                    event=Shutdown(reason="ros2_control_node exited")
                ),
            ],
        )
    )

    # ---- Controller spawners (delayed so ros2_control_node is ready first) ----
    spawner_delay = 3.0
    joint_state_spawner = TimerAction(
        period=spawner_delay,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster"],
                output="both",
            )
        ],
    )
    mit_trajectory_spawner = TimerAction(
        period=spawner_delay,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["mit_trajectory_controller", "--activate"],
                output="both",
            )
        ],
    )
    gripper_spawner = TimerAction(
        period=spawner_delay,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["gripper_controller", "--activate"],
                output="both",
            )
        ],
    )

    # ---- Robot State Publisher ----
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="both",
    )

    # ---- move_group ----
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="both",
        parameters=[move_group_params],
    )

    # ---- RViz ----
    rviz_config = PathJoinSubstitution([
        FindPackageShare("rebot_moveit_config"), "rviz", "moveit.rviz"
    ])
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        parameters=[move_group_params],
        condition=IfCondition(use_rviz),
        output="both",
    )
    moveit_delay = TimerAction(
        period=5.0,
        actions=[move_group_node, rviz_node],
    )

    return LaunchDescription(
        declared_arguments
        + [
            control_node_exit_handler,
            control_node,
            rsp_node,
            moveit_delay,
            joint_state_spawner,
            mit_trajectory_spawner,
            gripper_spawner,
        ]
    )
