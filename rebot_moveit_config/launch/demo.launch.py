#!/usr/bin/env python3
# ==============================================================================
# reBot MoveIt2 demo launch (standalone, includes ros2_control + move_group)
#
# Usage:
#   Mock standalone:  ros2 launch rebot_moveit_config demo.launch.py
#   Real hardware:    ros2 launch rebot_moveit_config demo.launch.py use_mock_hardware:=false
# ==============================================================================

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
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

    # ---- URDF via xacro ----
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

    # ---- MoveIt2 configs ----
    moveit_config = (
        MoveItConfigsBuilder("rebot_b601_dm", package_name="rebot_moveit_config")
        .robot_description(mappings={"use_mock_hardware": use_mock_hardware})
        .robot_description_semantic()
        .robot_description_kinematics()
        .planning_pipelines()
        .trajectory_execution()
        .planning_scene_monitor()
        .joint_limits()
        .to_moveit_configs()
    )
    move_group_params = moveit_config.to_dict()
    move_group_params["robot_description"] = robot_description["robot_description"]

    # ---- ros2_control_node ----
    controllers_path = PathJoinSubstitution([
        FindPackageShare("rebot_bringup"), "config", "ros2_control_controllers.yaml"
    ])
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controllers_path],
        remappings=[("~/robot_description", "/robot_description")],
        output="both",
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
    jtc_spawner = TimerAction(
        period=spawner_delay,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_trajectory_controller", "--activate"],
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
        condition=IfCondition(use_rviz),
        output="both",
    )

    return LaunchDescription(
        declared_arguments
        + [
            control_node,
            rsp_node,
            move_group_node,
            rviz_node,
            joint_state_spawner,
            jtc_spawner,
            gripper_spawner,
        ]
    )
