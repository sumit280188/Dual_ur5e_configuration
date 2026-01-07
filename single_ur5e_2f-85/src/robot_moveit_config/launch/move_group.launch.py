# from moveit_configs_utils import MoveItConfigsBuilder
# from moveit_configs_utils.launches import generate_move_group_launch


# def generate_launch_description():
#     moveit_config = MoveItConfigsBuilder("ur5e", package_name="robot_moveit_config").to_moveit_configs()
#     return generate_move_group_launch(moveit_config)

# from moveit_configs_utils import MoveItConfigsBuilder
# from moveit_configs_utils.launches import generate_move_group_launch
# from launch import LaunchDescription
# from launch_ros.actions import Node

# def generate_launch_description():
#     moveit_config = MoveItConfigsBuilder("ur5e", package_name="robot_moveit_config").to_moveit_configs()
    
#     # Move Group Node
#     move_group_node = Node(
#         package="moveit_ros_move_group",
#         executable="move_group",
#         output="screen",
#         parameters=[
#             moveit_config.to_dict(),
#             {"trajectory_execution.allowed_execution_duration_scaling": 2.0,},
#             {"publish_robot_description_semantic": True},
#             {"use_sim_time": True},
#         ],
#     )

#     return LaunchDescription(
#         [move_group_node]
#     )

from launch import LaunchDescription
from moveit_configs_utils import MoveItConfigsBuilder
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from ur_moveit_config.launch_common import load_yaml


def generate_launch_description():

    moveit_config = (
        MoveItConfigsBuilder(
            "ur5e", package_name="robot_moveit_config")
        .robot_description(file_path=os.path.join(get_package_share_directory("robot_moveit_config"), "config", "ur5e.urdf"))
        .robot_description_semantic(file_path=os.path.join(get_package_share_directory("robot_moveit_config"), "config", "ur5e.srdf"))
        .trajectory_execution(file_path=os.path.join(get_package_share_directory("robot_moveit_config"), "config", "moveit_controllers.yaml"))
        .robot_description_kinematics(file_path=os.path.join(get_package_share_directory("robot_moveit_config"), "config", "kinematics.yaml"))  # Ensure this is loaded
        .to_moveit_configs()
    )
    
    
    # Planning Configuration
    # ompl_planning_pipeline_config = {
    #     "move_group": {
    #         "planning_plugin": "ompl_interface/OMPLPlanner",
    #         "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
    #         "start_state_max_bounds_error": 10.0, #changed from 0.1
    #     }
    # }
    
    # ompl_planning_yaml = load_yaml("robot_moveit_config", "config/ompl_planning.yaml")
    # if ompl_planning_yaml:
    #     ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)
    # else:
    #     print("WARNING: Could not load ompl_planning.yaml")
    
    
    # controllers_yaml = load_yaml("robot_moveit_config", "config/moveit_controllers.yaml")
    
    # moveit_controllers = {
    # "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    # "moveit_simple_controller_manager": controllers_yaml["moveit_simple_controller_manager"],
    # }

    #Correctly set the default for joint_trajectory_controller within moveit_simple_controller_manager
    # controllers_yaml["moveit_simple_controller_manager"]["arm_controller"]["default"] = True
    # controllers_yaml["moveit_simple_controller_manager"]["robotiq_gripper_controller"]["default"] = True

    # moveit_controllers = {
    #     "moveit_simple_controller_manager": controllers_yaml["moveit_simple_controller_manager"],
    #     "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    # }

    # trajectory_execution = {
    #     "moveit_manage_controllers": True, #changed from False
    #     "trajectory_execution.allowed_execution_duration_scaling": 10.0, #chnaged from 1.2
    #     "trajectory_execution.allowed_goal_duration_margin": 10.0,
    #     "trajectory_execution.allowed_start_tolerance": 10.0,
    #     # Execution time monitoring can be incompatible with the scaled JTC
    #     "trajectory_execution.execution_duration_monitoring": True, #changed from False
    # }
    
    workspace_boundaries = {
        "workspace_parameters.min_corner.x": -2.0,
        "workspace_parameters.min_corner.y": -2.0,
        "workspace_parameters.min_corner.z": 0.0,  # Floor level so that robot does not move below floor
        "workspace_parameters.max_corner.x": 2.0,
        "workspace_parameters.max_corner.y": 2.0,
        "workspace_parameters.max_corner.z": 2.0,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
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

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        emulate_tty=True,
        parameters=[
            moveit_config.to_dict(),
            #{"trajectory_execution.allowed_execution_duration_scaling": 10.0,}, #very important for path planning in simulation, may be in real robot also
            {"publish_robot_description_semantic": True},
            {"use_sim_time": True},
            {"current_state_monitor_timeout": 10.0},
            workspace_boundaries,
            planning_scene_monitor_parameters,
            #ompl_planning_pipeline_config,
            #moveit_controllers,
            #trajectory_execution,
            planning_pipelines_config,
        ],
    )
    
    # # Joint state publisher
    # joint_state_publisher_node = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher',
    #     output='screen'
    # )  
      
    rviz_config = os.path.join(get_package_share_directory("robot_moveit_config"), "config", "moveit.rviz")
    
    if not os.path.exists(rviz_config):
        print(f"WARNING: RViz config file not found at {rviz_config}")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        emulate_tty=True,
        parameters=[moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.joint_limits,
                    {"use_sim_time": True}, # Add use_sim_time parameter later
                    ]
    )
    
    return LaunchDescription([move_group_node,
                              rviz_node,
                              #joint_state_publisher_node                              
    ])