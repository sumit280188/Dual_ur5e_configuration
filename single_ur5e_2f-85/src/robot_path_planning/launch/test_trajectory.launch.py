import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("ur5e", package_name="robot_moveit_config").to_moveit_configs()

    # MoveItCpp demo executable
    moveit_cpp_node = Node(
        name="test_trajectory",
        package="robot_moveit_config",
        executable="test_trajectory",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {'use_sim_time': True},
            {"qos_overrides./clock.subscription.reliability": "reliable"},
            {"qos_overrides./clock.subscription.durability": "transient_local"},
        ],
    )

    return LaunchDescription(
        [moveit_cpp_node]
    )
