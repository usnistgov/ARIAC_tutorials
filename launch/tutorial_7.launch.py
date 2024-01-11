import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)

from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def launch_setup(context, *args, **kwargs):

    urdf = os.path.join(get_package_share_directory("ariac_description"), "urdf/ariac_robots/ariac_robots.urdf.xacro")

    moveit_config = (
        MoveItConfigsBuilder("ariac_robots", package_name="ariac_moveit_config")
        .robot_description(urdf)
        .robot_description_semantic(file_path="config/ariac_robots.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .joint_limits(file_path="config/joint_limits.yaml")
        .moveit_cpp(
            file_path=get_package_share_directory("ariac_tutorials")
            + "/config/moveit_config.yaml"
        )
        .to_moveit_configs()
    )
    
    parameters_dict = moveit_config.to_dict()
    parameters_dict["use_sim_time"] = True
    tutorial_node = Node(
        package="ariac_tutorials",
        executable="tutorial_7.py",
        output="screen",
        parameters=[
            parameters_dict
        ],
    )
    start_rviz = LaunchConfiguration("rviz")

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("test_competitor"), "rviz", "test_competitor.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
        ],
        condition=IfCondition(start_rviz)
    )

    nodes_to_start = [
        tutorial_node,
        rviz_node
    ]

    return nodes_to_start

def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument("rviz", default_value="false", description="start rviz node?")
    )
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])