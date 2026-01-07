from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='collision_object_manager',
            executable='attach_detach',  
            name='collision_object_manager',
            output='screen',
            emulate_tty=True,
        )
    ])

