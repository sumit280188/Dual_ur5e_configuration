from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='collision_object_manager',
            executable='collision_pipe_attach_detach',
            name='collision_pipe_attach_detach',
            output='screen',
            emulate_tty=True,
        )
    ])