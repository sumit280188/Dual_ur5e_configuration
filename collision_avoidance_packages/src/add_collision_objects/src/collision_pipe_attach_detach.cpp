#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

using namespace std::chrono_literals;

class AddCollisionObjects : public rclcpp::Node
{
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

public:
  AddCollisionObjects()
      : Node("collision_object_manager")
  {
    // Add floor and collision objects immediately
    addFloorToScene();
    addCollisionPipe();
    addCollisionPipe2();
    addObstacleBox3();

    // Keep gripper status subscription for attachment/detachment
    gripper_status_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/gripper/status", 10,
        std::bind(&AddCollisionObjects::gripperStatusCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Collision Object Manager initialized");
  }

private:
  void gripperStatusCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    const std::string status = msg->data;

    if (status.find("[Gripper] Gripper closed successfully") != std::string::npos)
    {
      attachCollisionPipe();
    }
    else if (status.find("[Gripper] Gripper fully opened successfully - object released") != std::string::npos)
    {
      detachCollisionPipe();
    }
  }

  void addCollisionPipe()
  {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = "base_link";
    collision_object.id = "object_pipe";
  
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.CYLINDER;
    primitive.dimensions.resize(2);
    primitive.dimensions[0] = 0.1;  // height (length of the pipe)
    primitive.dimensions[1] = 0.02; // radius
  
    geometry_msgs::msg::Pose pipe_pose;
    // Rotate -90 degrees around Y-axis to make the cylinder lie along X-axis
    pipe_pose.orientation.x = 0.0;
    pipe_pose.orientation.y = 0.7071;
    pipe_pose.orientation.z = 0.0;
    pipe_pose.orientation.w = 0.7071;
  
    // Position the center of the cylinder
    pipe_pose.position.x = 0.695;
    pipe_pose.position.y = 0.168;
    pipe_pose.position.z = 0.01;  // Keep it close to the ground level if needed
  
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(pipe_pose);
    collision_object.operation = collision_object.ADD;
  
    planning_scene_interface_.applyCollisionObject(collision_object);
    RCLCPP_INFO(this->get_logger(), "Added horizontal pipe (cylinder tilted along X-axis)");
  }

  void addCollisionPipe2()
  {
    moveit_msgs::msg::CollisionObject collision_object2;
    collision_object2.header.frame_id = "base_link";
    collision_object2.id = "object_pipe2";
  
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.CYLINDER;
    primitive.dimensions.resize(2);
    primitive.dimensions[0] = 0.1;  // height (along Z-axis)
    primitive.dimensions[1] = 0.02; // radius
  
    geometry_msgs::msg::Pose pipe_pose;
    pipe_pose.orientation.w = 1.0;
    pipe_pose.position.x = -0.613;
    pipe_pose.position.y = -0.139;
    pipe_pose.position.z = 0.25; // Half of height to place it on the ground
  
    collision_object2.primitives.push_back(primitive);
    collision_object2.primitive_poses.push_back(pipe_pose);
    collision_object2.operation = collision_object2.ADD;
  
    planning_scene_interface_.applyCollisionObject(collision_object2);
    RCLCPP_INFO(this->get_logger(), "Added collision pipe2 (vertical cylinder) on startup");
  }

  void addObstacleBox3()
  {
    moveit_msgs::msg::CollisionObject obstacle_object3;
    obstacle_object3.header.frame_id = "base_link";
    obstacle_object3.id = "obstacle_box3";

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions = {0.2, 0.2, 0.2};

    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = -0.613;
    box_pose.position.y = -0.139;
    box_pose.position.z = 0.10; // Match grasp height

    obstacle_object3.primitives.push_back(primitive);
    obstacle_object3.primitive_poses.push_back(box_pose);
    obstacle_object3.operation = obstacle_object3.ADD;

    planning_scene_interface_.applyCollisionObject(obstacle_object3);
    RCLCPP_INFO(this->get_logger(), "Added obstacle box immediately on startup");
  }

  void attachCollisionPipe()
  {
    moveit_msgs::msg::AttachedCollisionObject attached_object;
    attached_object.link_name = "robotiq_85_right_finger_tip_link";
    attached_object.object.id = "object_pipe";
    attached_object.object.operation = attached_object.object.ADD;

    // Add touch links for proper collision checking
    attached_object.touch_links = {
        "robotiq_85_left_finger_tip_link",
        "robotiq_85_right_finger_tip_link"};

    planning_scene_interface_.applyAttachedCollisionObject(attached_object);
    RCLCPP_INFO(this->get_logger(), "Attached collision pipe to gripper");
  }

  void detachCollisionPipe()
  {
    moveit_msgs::msg::AttachedCollisionObject detach_object;
    detach_object.link_name = "robotiq_85_right_finger_tip_link";
    detach_object.object.id = "object_pipe";
    detach_object.object.operation = detach_object.object.REMOVE;

    planning_scene_interface_.applyAttachedCollisionObject(detach_object);
    
    // Remove the original collision pipe
    planning_scene_interface_.removeCollisionObjects({"object_pipe"});
    RCLCPP_INFO(this->get_logger(), "Detached collision pipe from gripper");

    // Create a timer to delay stacking by 1 second
    auto stack_timer = this->create_wall_timer(
        std::chrono::seconds(1),
        [this]() {
          stackPipeOnPipe2();
          // This is a one-shot timer, so cancel it after it fires
          stack_timer_->cancel();
        });
    
    // Store the timer to cancel it later
    stack_timer_ = stack_timer;
    
    RCLCPP_INFO(this->get_logger(), "Will stack pipe on pipe2 after 1 second delay");
  }

  void stackPipeOnPipe2()
  {
    // Create the first pipe again but position it on top of the second pipe
    moveit_msgs::msg::CollisionObject stacked_pipe;
    stacked_pipe.header.frame_id = "base_link";
    stacked_pipe.id = "object_pipe";

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.CYLINDER;
    primitive.dimensions.resize(2);
    primitive.dimensions[0] = 0.1;  // height (length of the pipe)
    primitive.dimensions[1] = 0.02; // radius
  
    geometry_msgs::msg::Pose pipe_pose;
    // Set vertical orientation (same as pipe2)
    pipe_pose.orientation.x = 0.0;
    pipe_pose.orientation.y = 0.0;
    pipe_pose.orientation.z = 0.0;
    pipe_pose.orientation.w = 1.0;
    
    // Position on top of pipe2
    // pipe2 is at z=0.25 and has height 0.5, so its top is at z=0.5
    pipe_pose.position.x = -0.613;
    pipe_pose.position.y = -0.139;
    pipe_pose.position.z = 0.2 + 0.1 + 0.05;  // Top of pipe2 + half of this pipe's height
  
    stacked_pipe.primitives.push_back(primitive);
    stacked_pipe.primitive_poses.push_back(pipe_pose);
    stacked_pipe.operation = stacked_pipe.ADD;
  
    planning_scene_interface_.applyCollisionObject(stacked_pipe);
    RCLCPP_INFO(this->get_logger(), "Stacked collision pipe vertically on top of pipe2");
  }

  void addFloorToScene(double floor_height = 0.0)
  {
    moveit_msgs::msg::CollisionObject floor_object;
    floor_object.header.frame_id = "world";
    floor_object.id = "floor";

    shape_msgs::msg::SolidPrimitive box;
    box.type = box.BOX;
    box.dimensions = {4.0, 4.0, 0.01};

    geometry_msgs::msg::Pose floor_pose;
    floor_pose.position.z = floor_height - 0.02;
    floor_pose.orientation.w = 1.0;

    floor_object.primitives.push_back(box);
    floor_object.primitive_poses.push_back(floor_pose);
    floor_object.operation = floor_object.ADD;

    planning_scene_interface_.applyCollisionObject(floor_object);
  }

  // Member variables
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gripper_status_sub_;
  rclcpp::TimerBase::SharedPtr stack_timer_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AddCollisionObjects>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}