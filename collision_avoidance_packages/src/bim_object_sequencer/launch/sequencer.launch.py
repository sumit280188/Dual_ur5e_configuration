from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    # 1) Start C++ attach/detach manager immediately
    attach_detach = Node(
        package='add_collision_objects',
        executable='attach_detach',
        name='attach_detach',
        output='screen'
    )

    # 2) Start spawner shortly after (ensures it comes up after attach_detach)
    spawner = Node(
        package='bim_object_sequencer',
        executable='spawner',
        name='bim_object_spawner',
        output='screen',
    )
    start_spawner = TimerAction(period=1.0, actions=[spawner])

    # 3) Start attached_base 2s after spawner
    attached_base = Node(
        package='bim_object_sequencer',
        executable='attached_base',
        name='attached_base',
        output='screen',
    )
    start_attached_base = TimerAction(period=2.0, actions=[attached_base])

    # 4) Start sequencer ~1s after attached_base
    sequencer = Node(
        package='bim_object_sequencer',
        executable='sequencer',
        name='bim_object_sequencer',
        output='screen'
    )
    start_sequencer = TimerAction(period=5.0, actions=[sequencer])  # 2s (base) + 1s = 3s

    return LaunchDescription([
        attach_detach,
        start_spawner,
        start_attached_base,
        start_sequencer,
    ])

