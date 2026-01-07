cmake_minimum_required(VERSION 3.8)
project(dual_robot_path_planning)

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
find_package(nlohmann_json REQUIRED) # for JSON handling


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

include_directories(include)

add_executable(bob_pick_and_place_with_publisher_for_attach_detach src/bob_pick_and_place_with_publisher_for_attach_detach.cpp
  src/pick_place_node.cpp
  src/guid_monitor.cpp
)
ament_target_dependencies(bob_pick_and_place_with_publisher_for_attach_detach
  ${COMMON_DEPS}
)
target_include_directories(bob_pick_and_place_with_publisher_for_attach_detach PRIVATE
  ${moveit_core_INCLUDE_DIRS}
)

ament_target_dependencies(bob_pick_and_place_with_publisher_for_attach_detach
  ${COMMON_DEPS}
  nlohmann_json
)

# Install targets
install(TARGETS
  bob_pick_and_place_with_publisher_for_attach_detach
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch resource
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()