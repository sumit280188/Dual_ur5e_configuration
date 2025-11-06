cmake_minimum_required(VERSION 3.8)
project(collision_object_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

include_directories(include)

add_executable(collision_object_manager src/collision_object_manager.cpp)
ament_target_dependencies(collision_object_manager
  rclcpp
  std_msgs
  moveit_msgs
  geometry_msgs
  shape_msgs
  moveit_ros_planning_interface
)

add_executable(collision_pipe_attach_detach src/collision_pipe_attach_detach.cpp)
ament_target_dependencies(collision_pipe_attach_detach
  rclcpp
  std_msgs
  moveit_msgs
  geometry_msgs
  shape_msgs
  moveit_ros_planning_interface
)

install(TARGETS
  collision_object_manager
  collision_pipe_attach_detach
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()