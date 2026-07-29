#!/usr/bin/env python3
# Copyright 2026 reBot ros2_control contributors
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
# Joy-Con teleoperation demo.
#
#   ros2 launch rebot_teleop_joy joy_teleop.launch.py \
#       teleop_mode:=joint|cartesian use_mock_hardware:=true|false \
#       joy_config:=<params yaml>
#
# Includes the rebot bringup, switches the arm to teleop_stream_controller,
# then starts the minimal servo pipeline and the teleop node.
# ==============================================================================

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    teleop_mode = LaunchConfiguration('teleop_mode')
    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    joy_config = LaunchConfiguration('joy_config')

    default_config = PathJoinSubstitution(
        [FindPackageShare('rebot_teleop_joy'), 'config', 'teleop_joy.yaml'])

    declared_arguments = [
        DeclareLaunchArgument(
            'teleop_mode', default_value='joint',
            description="Teleop mode fixed for this session: 'joint' or 'cartesian'"),
        DeclareLaunchArgument(
            'use_mock_hardware', default_value='true',
            description='true: mock simulation; false: real arm'),
        DeclareLaunchArgument(
            'joy_config', default_value=default_config,
            description='Teleop parameter file'),
    ]

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('rebot_bringup'), 'launch', 'bringup.launch.py'])),
        launch_arguments={'use_mock_hardware': use_mock_hardware}.items(),
    )

    # Give the controller spawners time to finish, then hand the arm to the
    # teleop stream controller.
    switch_to_teleop = TimerAction(
        period=6.0,
        actions=[ExecuteProcess(
            cmd=['ros2', 'control', 'switch_controllers',
                 '--deactivate', 'joint_trajectory_controller',
                 '--activate', 'teleop_stream_controller'],
            output='screen')],
    )

    servo_node = Node(
        package='rebot_teleop_joy',
        executable='servo_minimal',
        parameters=[joy_config],
        output='both',
    )

    teleop_node = Node(
        package='rebot_teleop_joy',
        executable='teleop_node',
        parameters=[joy_config, {'teleop_mode': teleop_mode}],
        output='both',
    )

    return LaunchDescription(
        declared_arguments + [bringup, switch_to_teleop, servo_node, teleop_node])
