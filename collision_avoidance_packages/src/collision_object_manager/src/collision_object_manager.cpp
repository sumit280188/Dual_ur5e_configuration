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

using namespace std::chrono_literals;

class CollisionObjectManager : public rclcpp::Node
{
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

public:
  CollisionObjectManager()
      : Node("collision_object_manager")
  {
    // Add floor and collision box immediately
    addFloorToScene();
    addCollisionBox();
    addObstacleBox();
    addObstacleBox2();
    addObstacleBox3();

    // Keep gripper status subscription for attachment/detachment
    gripper_status_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/gripper/status", 10,
        std::bind(&CollisionObjectManager::gripperStatusCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Collision Object Manager initialized");
  }

private:
  void gripperStatusCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    const std::string status = msg->data;

    if (status.find("[Gripper] Gripper closed successfully") != std::string::npos)
    {
      attachCollisionBox();
    }
    else if (status.find("[Gripper] Gripper fully opened successfully - object released") != std::string::npos)
    {
      detachCollisionBox();
    }
  }

  void addCollisionBox()
  {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = "base_link";
    collision_object.id = "object_box";

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions = {0.04, 0.04, 0.04};

    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.695;
    box_pose.position.y = 0.168;
    box_pose.position.z = 0.01; // Match grasp height

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    planning_scene_interface_.applyCollisionObject(collision_object);
    RCLCPP_INFO(this->get_logger(), "Added collision box immediately on startup");
  }

  void addObstacleBox()
  {
    moveit_msgs::msg::CollisionObject obstacle_object;
    obstacle_object.header.frame_id = "base_link";
    obstacle_object.id = "obstacle_box";

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions = {0.4, 0.4, 1.0};

    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.0;
    box_pose.position.y = 0.5;
    box_pose.position.z = 0.01; // Match grasp height

    obstacle_object.primitives.push_back(primitive);
    obstacle_object.primitive_poses.push_back(box_pose);
    obstacle_object.operation = obstacle_object.ADD;

    planning_scene_interface_.applyCollisionObject(obstacle_object);
    RCLCPP_INFO(this->get_logger(), "Added obstacle box immediately on startup");
  }

  void addObstacleBox2()
  {
    moveit_msgs::msg::CollisionObject obstacle_object2;
    obstacle_object2.header.frame_id = "base_link";
    obstacle_object2.id = "obstacle_box2";

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions = {0.4, 0.4, 1.0};

    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.0;
    box_pose.position.y = -0.5;
    box_pose.position.z = 0.01; // Match grasp height

    obstacle_object2.primitives.push_back(primitive);
    obstacle_object2.primitive_poses.push_back(box_pose);
    obstacle_object2.operation = obstacle_object2.ADD;

    planning_scene_interface_.applyCollisionObject(obstacle_object2);
    RCLCPP_INFO(this->get_logger(), "Added obstacle box immediately on startup");
  }

  void addObstacleBox3()
  {
    moveit_msgs::msg::CollisionObject obstacle_object3;
    obstacle_object3.header.frame_id = "base_link";
    obstacle_object3.id = "obstacle_box3";

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions = {0.4, 0.4, 0.1};

    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.0;
    box_pose.position.y = 0.0;
    box_pose.position.z = 1.2; // Match grasp height

    obstacle_object3.primitives.push_back(primitive);
    obstacle_object3.primitive_poses.push_back(box_pose);
    obstacle_object3.operation = obstacle_object3.ADD;

    planning_scene_interface_.applyCollisionObject(obstacle_object3);
    RCLCPP_INFO(this->get_logger(), "Added obstacle box immediately on startup");
  }

  void attachCollisionBox()
  {
    moveit_msgs::msg::AttachedCollisionObject attached_object;
    attached_object.link_name = "robotiq_85_right_finger_tip_link";
    attached_object.object.id = "object_box";
    attached_object.object.operation = attached_object.object.ADD;

    // Add touch links for proper collision checking
    attached_object.touch_links = {
        "robotiq_85_left_finger_tip_link",
        "robotiq_85_right_finger_tip_link"};

    planning_scene_interface_.applyAttachedCollisionObject(attached_object);
    RCLCPP_INFO(this->get_logger(), "Attached collision box to gripper");
  }

  void detachCollisionBox()
  {
    moveit_msgs::msg::AttachedCollisionObject detach_object;
    detach_object.object.id = "object_box";
    detach_object.object.operation = detach_object.object.REMOVE;

    planning_scene_interface_.applyAttachedCollisionObject(detach_object);
    planning_scene_interface_.removeCollisionObjects({"object_box"});
    RCLCPP_INFO(this->get_logger(), "Detached and removed collision box");

    // Re-add the collision box at the new position
    moveit_msgs::msg::CollisionObject new_object;
    new_object.header.frame_id = "base_link";
    new_object.id = "object_box";

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions = {0.04, 0.04, 0.04};

    geometry_msgs::msg::Pose new_pose;
    new_pose.orientation.w = 1.0;
    new_pose.position.x = -0.707;
    new_pose.position.y = -0.110;
    new_pose.position.z = 0.02;

    new_object.primitives.push_back(primitive);
    new_object.primitive_poses.push_back(new_pose);
    new_object.operation = new_object.ADD;

    planning_scene_interface_.applyCollisionObject(new_object);
    RCLCPP_INFO(this->get_logger(), "Re-added collision box at new position (-0.707, -0.110, 0.02)");
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
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CollisionObjectManager>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}