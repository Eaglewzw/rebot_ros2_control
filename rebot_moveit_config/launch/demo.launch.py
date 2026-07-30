#!/usr/bin/env python3
# ==============================================================================
# reBot MoveIt2 demo launch
#
# Usage:
#   Standalone (mock):  ros2 launch rebot_moveit_config demo.launch.py
#   With real bringup:  ros2 launch rebot_bringup bringup.launch.py use_mock_hardware:=false &
#                        ros2 launch rebot_moveit_config demo.launch.py use_mock_hardware:=false
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
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    # ---- Launch arguments ----
    use_rviz = LaunchConfiguration("use_rviz")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")

    declared_arguments = [
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Launch RViz2 with MotionPlanning panel",
        ),
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="true",
            description="Passed through to the URDF xacro; true = mock simulation",
        ),
    ]

    # ---- URDF via xacro ----
    xacro_path = PathJoinSubstitution([
        FindPackageShare("rebot_description"),
        "urdf",
        "rebot_b601_dm.urdf.xacro",
    ])
    robot_description_content = Command([
        FindExecutable(name="xacro"),
        " ",
        xacro_path,
        " use_mock_hardware:=",
        use_mock_hardware,
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

    # ---- Robot State Publisher ----
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="both",
    )

    # ---- move_group node ----
    move_group_params = moveit_config.to_dict()
    # Ensure the URDF is also in move_group params (overwrite the xacro-generated one
    # from MoveItConfigsBuilder with our Command-based one for consistency)
    move_group_params["robot_description"] = robot_description["robot_description"]

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
            rsp_node,
            move_group_node,
            rviz_node,
        ]
    )
