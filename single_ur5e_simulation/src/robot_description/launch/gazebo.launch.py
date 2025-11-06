from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from pathlib import Path
import os
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.actions import TimerAction

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
            default_value="robot_description",
            description="Description package with robot URDF/XACRO files. Usually the argument "
            "is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="robot.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value='""',
            description="Prefix of the joint names, useful for "
            "multi-robot setup. If changed than also joint names in the controllers' configuration "
            "have to be updated.",
        )
    )
    #important parameter for simulation
    declared_arguments.append(
        DeclareLaunchArgument(
        "sim_ignition",
        default_value="true",
        description="Set to 'true' to use Gazebo Ignition (Gazebo Sim) for simulation.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Make MoveIt to use simulation time. This is needed for the trajectory planing in simulation.",
        )
    )

    # Initialize Arguments
    robot_ip = LaunchConfiguration("robot_ip")
    ur_type = LaunchConfiguration("ur_type")
    safety_limits = LaunchConfiguration("safety_limits")
    # General arguments
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    tf_prefix = LaunchConfiguration("tf_prefix")
    #important parameter for simulation
    sim_ignition = LaunchConfiguration("sim_ignition")
    use_sim_time = LaunchConfiguration("use_sim_time")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
            " ",
            "safety_limits:=",
            safety_limits,
            " ",
            "name:=",
            "ur5e",
            " ",
            "ur_type:=",
            ur_type,
            " ",
            "robot_ip:=",
            robot_ip,
            " ",
            "sim_ignition:=",
            sim_ignition,
            " ",
            "tf_prefix:=",
            tf_prefix
        ]
    )
    
    robot_description = {"robot_description": robot_description_content
                        }

    
    robot_description_path = get_package_share_directory("ur_description")
    gripper_description_path = get_package_share_directory("robotiq_description")
    
    # Environment variable for Gazebo resource path
    gazebo_resource_path = SetEnvironmentVariable(
    name="GZ_SIM_RESOURCE_PATH",
        value=[
            str(Path(robot_description_path).parent.resolve()),
            ":",
            str(Path(gripper_description_path).parent.resolve()),
        ]
    )

    # Get world path
    custom_world_path = os.path.join(
        get_package_share_directory("robot_simulation_launch"),
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
    
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "/robot_description", "-name", "ur5e"],
        parameters=[{"use_sim_time": use_sim_time}],  # Added parameter
        #robot_description is the topic name, ur5e is the name of the robot but what about the gripper topic name?        
        #gripper_topic is also robot_description.
    )
    
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        parameters=[
        {"use_sim_time": use_sim_time},
        {"qos_overrides./clock.publisher.reliability": "reliable"},
        {"qos_overrides./clock.publisher.durability": "transient_local"}
        ]
    )
    
    # your_pipe_pkg_node = Node(
    #         package='ros_gz_sim',
    #         executable='create',
    #         name='spawn_pipe_assembly',
    #         output='screen',
    #         arguments=[
    #             '-name', 'pipe_system',
    #             '-file', PathJoinSubstitution([
    #                 FindPackageShare('your_pipe_pkg'),
    #                 'models',
    #                 'pipe_system.sdf'
    #             ]),
    #             '-x', '0.0',
    #             '-y', '0.2',
    #             '-z', '0.0'
    #         ]
    #     )

    
    nodes_to_start = [
        gazebo_resource_path,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge,
        #your_pipe_pkg_node,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)