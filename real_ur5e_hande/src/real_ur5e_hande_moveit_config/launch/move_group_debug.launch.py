from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def load_yaml(package_name, file_path):
    """Load a YAML file from a package share directory and return its contents as a dict."""
    try:
        yaml_path = os.path.join(get_package_share_directory(package_name), file_path)
        print(f"Attempting to load: {yaml_path}")
        if not os.path.exists(yaml_path):
            print(f"WARNING: File does not exist: {yaml_path}")
            return {}
        with open(yaml_path, "r") as f:
            content = yaml.safe_load(f)
            if content is None:
                print(f"WARNING: YAML file is empty or returned None: {yaml_path}")
                return {}
            return content
    except Exception as e:
        print(f"ERROR loading {file_path} from {package_name}: {e}")
        return {}

def validate_dict(d, name):
    """Validate that a dictionary doesn't contain None values."""
    if d is None:
        print(f"ERROR: {name} is None!")
        return False
    for key, value in d.items():
        if value is None:
            print(f"ERROR: {name}['{key}'] is None!")
            return False
        if isinstance(value, dict):
            if not validate_dict(value, f"{name}.{key}"):
                return False
    return True

def generate_launch_description():
    print("\n=== Starting launch file generation ===\n")
    
    # Build MoveIt configuration
    print("Building MoveIt configuration...")
    try:
        moveit_config = (
            MoveItConfigsBuilder("real_ur5e_hande", package_name="real_ur5e_hande_moveit_config")
            .robot_description(
                file_path=os.path.join(
                    get_package_share_directory("real_ur5e_hande_description"), 
                    "urdf", 
                    "real_ur5e_hande_config.urdf.xacro"
                )
            )
            .robot_description_semantic(
                file_path=os.path.join(
                    get_package_share_directory("real_ur5e_hande_moveit_config"), 
                    "config", 
                    "real_ur5e_hande.srdf"
                )
            )
            .robot_description_kinematics(
                file_path=os.path.join(
                    get_package_share_directory("real_ur5e_hande_moveit_config"), 
                    "config", 
                    "kinematics.yaml"
                )
            )
            .trajectory_execution(
                file_path=os.path.join(
                    get_package_share_directory("real_ur5e_hande_moveit_config"), 
                    "config", 
                    "moveit_controllers.yaml"
                )
            )
            .planning_pipelines(
                pipelines=["ompl"]
            )
            .to_moveit_configs()
        )
        print("MoveIt configuration built successfully")
    except Exception as e:
        print(f"ERROR building MoveIt configuration: {e}")
        raise

    # Convert to dict and validate
    print("\nConverting MoveIt config to dict...")
    moveit_dict = moveit_config.to_dict()
    print(f"MoveIt dict has {len(moveit_dict)} keys")
    
    # Check for None values in moveit_dict
    print("\nValidating MoveIt configuration dictionary...")
    validate_dict(moveit_dict, "moveit_config")

    # Workspace boundaries
    workspace_boundaries = {
        "workspace_parameters.min_corner.x": -2.0,
        "workspace_parameters.min_corner.y": -2.0,
        "workspace_parameters.min_corner.z": 0.0,
        "workspace_parameters.max_corner.x": 2.0,
        "workspace_parameters.max_corner.y": 2.0,
        "workspace_parameters.max_corner.z": 2.0,
    }
    print("\nWorkspace boundaries configured")
    validate_dict(workspace_boundaries, "workspace_boundaries")
    
    # Planning Functionality - Start with basic configuration
    planning_pipelines_config = {
        "default_planning_pipeline": "ompl",
        "planning_pipelines": ["ompl"],
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        },
    }
    print("\nPlanning pipelines configured")
    validate_dict(planning_pipelines_config, "planning_pipelines_config")
    
    # Planning scene monitor parameters
    planning_scene_params = {
        "planning_scene_monitor.publish_planning_scene": True,
        "planning_scene_monitor.publish_planning_scene_frequency": 1.0,
        "planning_scene_monitor.publish_geometry_updates": True,
        "planning_scene_monitor.publish_state_updates": True,
        "planning_scene_monitor.publish_transforms_updates": True,
        "planning_scene_monitor.publish_robot_description": True,
        "planning_scene_monitor.publish_robot_description_semantic": True,
    }
    print("\nPlanning scene monitor parameters configured")
    validate_dict(planning_scene_params, "planning_scene_params")
    
    # Other parameters
    other_params = {
        "publish_robot_description_semantic": True,
        "use_sim_time": True,
        "current_state_monitor_timeout": 10.0,
    }
    print("\nOther parameters configured")
    validate_dict(other_params, "other_params")

    # Combine all parameters
    print("\nCombining all parameters for move_group node...")
    all_parameters = [
        moveit_dict,
        other_params,
        workspace_boundaries,
        planning_scene_params,
        planning_pipelines_config,
    ]
    
    # Validate each parameter dict
    for i, param_dict in enumerate(all_parameters):
        print(f"Validating parameter dict {i}: {type(param_dict)}")
        if not validate_dict(param_dict, f"param_dict_{i}"):
            raise ValueError(f"Parameter dict {i} contains None values!")

    # Single move_group node
    print("\nCreating move_group node...")
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        emulate_tty=True,
        parameters=all_parameters,
    )

    # RViz setup
    print("\nConfiguring RViz...")
    rviz_config = os.path.join(
        get_package_share_directory("real_ur5e_hande_moveit_config"), 
        "config", 
        "moveit.rviz"
    )
    
    if not os.path.exists(rviz_config):
        print(f"WARNING: RViz config file does not exist: {rviz_config}")
    
    rviz_params = [
        moveit_config.robot_description,
        moveit_config.robot_description_semantic,
        moveit_config.robot_description_kinematics,
        moveit_config.planning_pipelines,
        moveit_config.joint_limits,
        {"use_sim_time": True},
    ]
    
    # Validate RViz parameters
    print("\nValidating RViz parameters...")
    for i, param in enumerate(rviz_params):
        if param is None:
            print(f"ERROR: RViz parameter {i} is None!")
        elif isinstance(param, dict):
            validate_dict(param, f"rviz_param_{i}")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        emulate_tty=True,
        arguments=["-d", rviz_config],
        parameters=rviz_params,
    )

    print("\n=== Launch description generated successfully ===\n")
    
    return LaunchDescription([
        move_group_node,
        rviz_node,
    ])