import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    robot_path_planning_pkg = get_package_share_directory('robot_path_planning')
    collision_object_manager_pkg = get_package_share_directory('collision_object_manager')

    # robot_path_planning_launch= IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(robot_path_planning_pkg, 'launch', 'pick_and_place_with_publisher.launch.py'),
    #     )
    # )

    robot_path_planning_launch = TimerAction(
        period=5.0,  # Increased from 10 to allow box to load first
        actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_path_planning_pkg, 'launch', 'pick_and_place_with_publisher_for_attach_detach.launch.py'),
        )
        )
        ]
    )
    
    collision_object_manager_launch = TimerAction(
        period=0.0,  # Increased from 10 to allow box to load first
        actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(collision_object_manager_pkg, 'launch', 'collision_pipe_attach_detach.launch.py'),
        )
        )
        ]
    )

    return LaunchDescription([
        robot_path_planning_launch,
        collision_object_manager_launch,
    ])