from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():

    moveit_config = (
        MoveItConfigsBuilder(
            "ur5e", package_name="robot_moveit_config")
        .robot_description(file_path=os.path.join(get_package_share_directory("robot_moveit_config"), "config", "ur5e.urdf"))
        .robot_description_semantic(file_path=os.path.join(get_package_share_directory("robot_moveit_config"), "config", "ur5e.srdf"))
        .trajectory_execution(file_path=os.path.join(get_package_share_directory("robot_moveit_config"), "config", "moveit_controllers.yaml"))
        .robot_description_kinematics(file_path=os.path.join(get_package_share_directory("robot_moveit_config"), "config", "kinematics.yaml"))  # Ensure this is loaded
        .to_moveit_configs()
    )

    ur5e_complete_pick_and_place_node = Node(
        package="robot_path_planning",
        executable="ur5e_complete_pick_and_place",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.trajectory_execution,
            moveit_config.robot_description_kinematics,
            {'use_sim_time': True},
        ],
    )

    return LaunchDescription([ur5e_complete_pick_and_place_node])