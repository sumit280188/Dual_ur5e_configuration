from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='add_collision_objects',
            executable='attach_detach',  
            name='add_collision_objects',
            output='screen',
            emulate_tty=True,
        )
    ])

