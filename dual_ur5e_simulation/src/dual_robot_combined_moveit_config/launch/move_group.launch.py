from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
import os
from ur_moveit_config.launch_common import load_yaml

def generate_launch_description():
    # Build MoveIt config for the dual robot system
    moveit_config = (
        MoveItConfigsBuilder("dual_robots_config", package_name="dual_robot_combined_moveit_config")
        .robot_description(
            file_path=os.path.join(
                get_package_share_directory("dual_robot_combined_description"), 
                "urdf", 
                "dual_robots_config.urdf.xacro"
            )
        )
        .robot_description_semantic(
            file_path=os.path.join(
                get_package_share_directory("dual_robot_combined_moveit_config"), 
                "config", 
                "dual_robots_config.srdf"
            )
        )
        .robot_description_kinematics(
            file_path=os.path.join(
                get_package_share_directory("dual_robot_combined_moveit_config"), 
                "config", 
                "kinematics.yaml"
            )
        )
        .trajectory_execution(
            file_path=os.path.join(
                get_package_share_directory("dual_robot_combined_moveit_config"), 
                "config", 
                "moveit_controllers.yaml"
            )
        )
        .planning_pipelines(
            pipelines=["ompl"]
        )
        .to_moveit_configs()
    )

    # Planning scene monitor parameters
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "monitor_dynamics": False,
    }        
    
    # Planning Functionality
    planning_pipelines_config = {
        "default_planning_pipeline": "ompl",
        "planning_pipelines": ["pilz", "ompl"],
        "pilz": {
            "planning_plugin": "pilz_industrial_motion_planner/CommandPlanner",
            "request_adapters": "",
            "start_state_max_bounds_error": 0.1,
        },
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        },
    }
    ompl_planning_yaml = load_yaml(
        "moveit_resources_prbt_moveit_config", "config/ompl_planning.yaml"
    )
    planning_pipelines_config["ompl"].update(ompl_planning_yaml)




    # Single move_group node for both robots
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        emulate_tty=True,
        parameters=[
            moveit_config.to_dict(),
            {
                "publish_robot_description_semantic": True,
                "use_sim_time": True,
                "current_state_monitor_timeout": 10.0,
                #"trajectory_execution.allowed_execution_duration_scaling": 2.0,
                #"trajectory_execution.allowed_goal_duration_margin": 0.5,
            },
            planning_scene_monitor_parameters,
            planning_pipelines_config,
            
        ],
        # For a unified robot description, you need to aggregate joint states from both robots
        # remappings=[
        #     # The move_group will subscribe to a single /joint_states topic
        #     # You'll need a joint_state_publisher node to combine alice and bob joint states
        #     ("/joint_states", "/joint_states"),  # combined joint states
        # ],
    )

    # # Joint state aggregator - combines joint states from alice and bob
    # joint_state_publisher = Node(
    #     package="joint_state_publisher",
    #     executable="joint_state_publisher",
    #     name="joint_state_publisher",
    #     output="screen",
    #     parameters=[
    #         {
    #             "source_list": ["/alice/joint_states", "/bob/joint_states"],
    #             "use_sim_time": True,
    #         }
    #     ],
    # )

    # RViz setup (uncomment when ready)
    rviz_config = os.path.join(
        get_package_share_directory("dual_robot_combined_moveit_config"), 
        "config", 
        "moveit.rviz"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        emulate_tty=True,
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([
        #joint_state_publisher,  # Must start before move_group
        move_group_node,
        rviz_node,
    ])