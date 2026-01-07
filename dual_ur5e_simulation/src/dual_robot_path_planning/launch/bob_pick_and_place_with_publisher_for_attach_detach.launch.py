from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def load_yaml(package_name, file_path):
    """Load a YAML file from a package share directory and return its contents as a dict."""
    yaml_path = os.path.join(get_package_share_directory(package_name), file_path)
    with open(yaml_path, "r") as f:
        return yaml.safe_load(f)

def generate_launch_description():
    # Build MoveIt config for the dual robot system
    moveit_config = (
        MoveItConfigsBuilder("dual_robots_config", package_name="dual_robot_combined_moveit_config")
        .robot_description(
            file_path=os.path.join(
                get_package_share_directory("dual_robot_combined_description"), 
                "urdf", 
                "dual_robots_config.urdf.xacro"
            )
        )
        .robot_description_semantic(
            file_path=os.path.join(
                get_package_share_directory("dual_robot_combined_moveit_config"), 
                "config", 
                "dual_robots_config.srdf"
            )
        )
        .robot_description_kinematics(
            file_path=os.path.join(
                get_package_share_directory("dual_robot_combined_moveit_config"), 
                "config", 
                "kinematics.yaml"
            )
        )
        .trajectory_execution(
            file_path=os.path.join(
                get_package_share_directory("dual_robot_combined_moveit_config"), 
                "config", 
                "moveit_controllers.yaml"
            )
        )
        .planning_pipelines(
            pipelines=["ompl"]
        )
        .to_moveit_configs()
    )
    
    bob_pick_and_place_with_publisher_for_attach_detach_node = Node(
        package="dual_robot_path_planning",
        executable="bob_pick_and_place_with_publisher_for_attach_detach",
        output="screen",
        emulate_tty=True,
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.trajectory_execution,
            moveit_config.robot_description_kinematics,
            {'use_sim_time': True},
        ],
    )

    return LaunchDescription([bob_pick_and_place_with_publisher_for_attach_detach_node,
                              
    ])