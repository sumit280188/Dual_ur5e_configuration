cmake_minimum_required(VERSION 3.8)
project(robot_path_planning)

# Default to C++17
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Boost REQUIRED)  # Add Boost dependency
find_package(control_msgs REQUIRED) # Add control_msgs dependency
find_package(trajectory_msgs REQUIRED) # Add trajectory_msgs dependency
find_package(std_msgs REQUIRED) # for std_msgs::msg::String


# Common dependencies for all targets
set(COMMON_DEPS
  rclcpp
  moveit_ros_planning_interface
  moveit_core
  moveit_msgs
  control_msgs
  std_msgs
  trajectory_msgs
  geometry_msgs
  Boost
)

# Static_Single_thread_executor
add_executable(Static_Single_thread_executor src/Static_Single_thread_executor.cpp)
ament_target_dependencies(Static_Single_thread_executor
  ${COMMON_DEPS}
)

add_executable(ur5e_complete_pick_and_place src/ur5e_complete_pick_and_place.cpp)
ament_target_dependencies(ur5e_complete_pick_and_place
  ${COMMON_DEPS}
)


set(MOVEIT_INCLUDE_DIRS "/opt/ros/humble/include")
include_directories(${MOVEIT_INCLUDE_DIRS})

target_include_directories(ur5e_complete_pick_and_place PRIVATE
  ${moveit_ros_planning_interface_INCLUDE_DIRS}
)

# Cartesian path planner
add_executable(Robot_with_gripper src/Robot_with_gripper.cpp)
ament_target_dependencies(Robot_with_gripper
  ${COMMON_DEPS}
)
target_include_directories(Robot_with_gripper PRIVATE
  ${moveit_core_INCLUDE_DIRS}
)

add_executable(robot_with_gripper_feedback src/robot_with_gripper_feedback.cpp)
ament_target_dependencies(robot_with_gripper_feedback
  ${COMMON_DEPS}
)
target_include_directories(robot_with_gripper_feedback PRIVATE
  ${moveit_core_INCLUDE_DIRS}
)

add_executable(pick_and_place src/pick_and_place.cpp)
ament_target_dependencies(pick_and_place
  ${COMMON_DEPS}
)
target_include_directories(pick_and_place PRIVATE
  ${moveit_core_INCLUDE_DIRS}
)

add_executable(pick_and_place_with_publisher src/pick_and_place_with_publisher.cpp)
ament_target_dependencies(pick_and_place_with_publisher
  ${COMMON_DEPS}
)
target_include_directories(pick_and_place_with_publisher PRIVATE
  ${moveit_core_INCLUDE_DIRS}
)

add_executable(pick_and_place_with_publisher_for_attach_detach src/pick_and_place_with_publisher_for_attach_detach.cpp)
ament_target_dependencies(pick_and_place_with_publisher_for_attach_detach
  ${COMMON_DEPS}
)
target_include_directories(pick_and_place_with_publisher_for_attach_detach PRIVATE
  ${moveit_core_INCLUDE_DIRS}
)

add_executable(hello_moveit src/hello_moveit.cpp)
ament_target_dependencies(hello_moveit
  rclcpp
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
)

# Install targets
install(TARGETS
  Static_Single_thread_executor
  ur5e_complete_pick_and_place
  Robot_with_gripper
  robot_with_gripper_feedback
  pick_and_place
  pick_and_place_with_publisher
  pick_and_place_with_publisher_for_attach_detach
  hello_moveit
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()