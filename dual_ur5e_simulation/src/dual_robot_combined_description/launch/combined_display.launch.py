from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, FindExecutable, Command
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = []

    # Common arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="dual_robot_combined_description",
            description="Package with robot URDF/XACRO files."
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="dual_robots_config.urdf.xacro",
            description="Combined URDF/XACRO file with both robots."
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config",
            default_value=PathJoinSubstitution([
                FindPackageShare("dual_robot_combined_description"),
                "rviz",
                "dual_robots_combined.rviz"
            ]),
            description="Path to the RViz config file."
        )
    )

    # Get launch configurations
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    rviz_config = LaunchConfiguration("rviz_config")

    # === COMBINED ROBOT DESCRIPTION ===
    # Single URDF containing both Alice and Bob
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([
                FindPackageShare(description_package), 
                "urdf", 
                description_file
            ]),
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    # === ROBOT STATE PUBLISHER (Single for combined URDF) ===
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # This will just republish joint states without GUI
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
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

    return LaunchDescription(declared_arguments + [
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node,
    ])