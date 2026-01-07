from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    SetEnvironmentVariable,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from pathlib import Path
import os
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    declared_arguments = []

    # UR specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            description="IP address of used UR robot.",
            default_value="0.0.0.0",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            description="Type/series of used UR robot.",
            default_value="ur5e",
            choices=["ur3", "ur3e", "ur5", "ur5e", "ur10", "ur10e", "ur16e", "ur20", "ur30"],
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_limits",
            default_value="true",
            description="Enables the safety limits controller if true.",
        )
    )

    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="robot_ur5e_hande_description",
            description="Description package with combined robot URDF/XACRO files.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="ur5e_hande.urdf.xacro",
            description="Combined URDF/XACRO description file with both robots.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation time.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gz_verbosity",
            default_value="4",
            description="Gazebo verbosity level (1-4).",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "world_file",
            default_value="empty.sdf",
            description="Gazebo world file to load.",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config",
            default_value=PathJoinSubstitution([
                FindPackageShare("robot_ur5e_hande_description"),
                "rviz",
                "ur5e_hande.rviz"
            ]),
            description="Path to the RViz config file."
        )
    )

    # Initialize Arguments
    robot_ip = LaunchConfiguration("robot_ip")
    ur_type = LaunchConfiguration("ur_type")
    safety_limits = LaunchConfiguration("safety_limits")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    gz_verbosity = LaunchConfiguration("gz_verbosity")
    world_file = LaunchConfiguration("world_file")
    rviz_config = LaunchConfiguration("rviz_config")

    # ========================================
    # COMBINED ROBOT DESCRIPTION
    # ========================================
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([
                FindPackageShare(description_package),
                "urdf",
                description_file,
            ]),
            " ",
            "safety_limits:=",
            safety_limits,
            " ",
            "ur_type:=",
            ur_type,
            " ",
            "robot_ip:=",
            robot_ip,
            " ",
            "use_fake_hardware:=false",
            " ",
            "sim_ignition:=true",
        ]
    )

    robot_description = {
    "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # ========================================
    # ROBOT STATE PUBLISHER (Single for combined URDF)
    # ========================================
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            {
                'publish_frequency': 50.0,  # Publish at 50 Hz
                'use_sim_time': True,
                'ignore_timestamp': False,  # Important for Gazebo
                'frame_prefix': '',
            }
        ],
    )

    # ========================================
    # GAZEBO RESOURCE PATH
    # ========================================
    robot_description_path = get_package_share_directory("ur_description")
    gripper_description_path = get_package_share_directory("robotiq_hande_description")

    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            str(Path(robot_description_path).parent.resolve()),
            ":",
            str(Path(gripper_description_path).parent.resolve()),
        ],
    )

    # ========================================
    # GAZEBO SIMULATION
    # ========================================
    # gazebo = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         [
    #             os.path.join(get_package_share_directory("ros_gz_sim"), "launch"),
    #             "/gz_sim.launch.py",
    #         ]
    #     ),
    #     launch_arguments=[
    #         ("gz_args", [" -v ", gz_verbosity, " -r ", world_file]),
    #         ("on_exit_shutdown", "true"),
    #     ],
    # )
    
    # Get world path
    custom_world_path = os.path.join(
        get_package_share_directory("robot_ur5e_hande_controller"),
        "worlds",
        "custom_world.sdf"
    )
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("ros_gz_sim"), "launch"),
                "/gz_sim.launch.py",
            ]
        ),
        #launch_arguments=[("gz_args", " -v 4 -r empty.sdf ")],
        launch_arguments=[
        ("gz_args", f" -v 4 -r {custom_world_path} ")
        ],
    )

    # ========================================
    # SPAWN COMBINED ROBOT IN GAZEBO
    # ========================================
    gz_spawn_robots = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="ros_gz_sim",
                executable="create",
                output="screen",
                arguments=[
                    "-topic",
                    "/robot_description",
                    "-name",
                    "ur5e_hande_config",
                ],
                parameters=[{"use_sim_time": use_sim_time}],
            )
        ],
    )

    # ========================================
    # ROS-GAZEBO BRIDGE
    # ========================================
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        parameters=[
            {"use_sim_time": use_sim_time},
            # {"qos_overrides./clock.publisher.reliability": "reliable"},
            # {"qos_overrides./clock.publisher.durability": "transient_local"},
        ],
        output="screen",
    )

    # ========================================
    # CONTROLLER SPAWNERS (Added here!)
    # ========================================
    joint_state_broadcaster_spawner = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            )
        ],
    )

    arm_controller_spawner = TimerAction(
        period=10.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["arm_controller", "--controller-manager", "/controller_manager"],
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            )
        ],
    )

    gripper_action_controller_spawner = TimerAction(
        period=15.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["gripper_action_controller", "--controller-manager", "/controller_manager"],
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            )
        ],
    )

    # === RViz NODE ===
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
        parameters=[robot_description],
    )

    nodes_to_start = [
        gazebo_resource_path,
        gazebo,
        gz_ros2_bridge,
        robot_state_publisher_node,
        gz_spawn_robots,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        gripper_action_controller_spawner,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)