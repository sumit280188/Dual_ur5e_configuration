import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    robot_description_pkg = get_package_share_directory('dual_robot_combined_description')
    robot_moveit_pkg = get_package_share_directory('dual_robot_combined_moveit_config')

    launch_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_description_pkg, 'launch', 'combined_gazebo.launch.py'),
        )
    )

    # 5-second delay before launching MoveIt
    launch_moveit = TimerAction(
        period=20.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(robot_moveit_pkg, 'launch', 'move_group.launch.py'),
                )
            )
        ]
    )

    return LaunchDescription([
        launch_gazebo,
        launch_moveit
    ])
