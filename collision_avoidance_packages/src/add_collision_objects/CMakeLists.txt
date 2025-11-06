cmake_minimum_required(VERSION 3.8)
project(add_collision_objects)

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

add_executable(add_collision_objects src/add_collision_objects.cpp)
ament_target_dependencies(add_collision_objects
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

# Added: attach_detach node
add_executable(attach_detach src/attach_detach.cpp)
ament_target_dependencies(attach_detach
  rclcpp
  std_msgs
  moveit_msgs
  geometry_msgs
  shape_msgs
  moveit_ros_planning_interface
)

install(TARGETS
  add_collision_objects
  collision_pipe_attach_detach
  attach_detach
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

