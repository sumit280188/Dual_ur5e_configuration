from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='add_collision_objects',
            executable='collision_pipe_attach_detach',
            name='collision_pipe_attach_detach',
            output='screen',
            emulate_tty=True,
        )
    ])