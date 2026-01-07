cmake_minimum_required(VERSION 3.22)
project(robot_moveit_config)

find_package(ament_cmake REQUIRED)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Boost REQUIRED) # Add Boost dependency
find_package(interactive_markers REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)

# Common dependencies for all targets
set(COMMON_DEPS
  rclcpp
  moveit_ros_planning_interface
  moveit_core
  moveit_msgs
  geometry_msgs
  Boost
)

# Cartesian path planner
add_executable(cartesian_path_planner_node src/cartesian_path_planner_node.cpp)
ament_target_dependencies(cartesian_path_planner_node
  ${COMMON_DEPS}
)
target_include_directories(cartesian_path_planner_node PRIVATE
  ${moveit_core_INCLUDE_DIRS}
)

# Generate the executable
add_executable(test_trajectory
  src/test_trajectory.cpp)
target_include_directories(test_trajectory
  PRIVATE include)
ament_target_dependencies(test_trajectory
  ament_cmake
  rclcpp
  rclcpp_action
  moveit_core
  moveit_ros_planning_interface
  interactive_markers
  moveit_ros_planning
  control_msgs)

# Install the executable
install(TARGETS
  test_trajectory
  DESTINATION lib/${PROJECT_NAME}
)

# Install the launch file
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)


# Install targets
install(TARGETS
  cartesian_path_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
