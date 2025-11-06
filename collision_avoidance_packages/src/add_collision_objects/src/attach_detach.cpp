#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/attached_collision_object.hpp>

#include <memory>
#include <string>
#include <vector>

class AddCollisionObjects : public rclcpp::Node
{
public:
  AddCollisionObjects()
  : Node("collision_object_manager"),
    default_object_id_(declare_parameter<std::string>("object_id", "object_pipe")),
    attach_link_(declare_parameter<std::string>("attach_link", "robotiq_85_right_finger_tip_link"))
  {
    // Allow configuring touch links via params
    touch_links_ = declare_parameter<std::vector<std::string>>(
        "touch_links",
        std::vector<std::string>{
          "robotiq_85_left_finger_tip_link",
          "robotiq_85_right_finger_tip_link"
        });

    // Subscribe to the last spawned collision object ID (latched)
    auto latched_qos = rclcpp::QoS(1).reliable().transient_local();
    spawned_id_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/spawned_collision_object_id", latched_qos,
        [this](const std_msgs::msg::String::SharedPtr msg){
          latest_spawned_id_ = msg->data;
          RCLCPP_INFO(this->get_logger(),
            "Updated last spawned object id: '%s'", latest_spawned_id_.c_str());
        });

    // Gripper status → attach/detach
    gripper_status_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/gripper/status", 10,
        std::bind(&AddCollisionObjects::gripperStatusCallback, this, std::placeholders::_1));

    // Publishers for sequencer triggers
    aco_attach_pub_ = this->create_publisher<moveit_msgs::msg::AttachedCollisionObject>("/attached_collision_object", 10);
    aco_detach_pub_ = this->create_publisher<moveit_msgs::msg::AttachedCollisionObject>("/detached_collision_object", 10);

    RCLCPP_INFO(this->get_logger(),
                "AddCollisionObjects ready (default_object_id='%s', attach_link='%s')",
                default_object_id_.c_str(), attach_link_.c_str());
  }

private:
  void gripperStatusCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    const std::string& status = msg->data;

    if (status.find("[Gripper] Gripper closed successfully") != std::string::npos)
    {
      attachObject();
    }
    else if (status.find("[Gripper] Gripper fully opened successfully - object released") != std::string::npos)
    {
      detachObject();
    }
  }

  // Decide which object ID to use: prefer the last spawned one, otherwise fallback to default param
  std::string current_target_id() const
  {
    return latest_spawned_id_.empty() ? default_object_id_ : latest_spawned_id_;
  }

  void attachObject()
  {
    const auto id = current_target_id();
    if (latest_spawned_id_.empty()) {
      RCLCPP_WARN(this->get_logger(),
        "No recently spawned object ID received; falling back to default '%s'", id.c_str());
    }

    moveit_msgs::msg::AttachedCollisionObject aco;
    aco.link_name = attach_link_;
    aco.object.id = id;
    aco.object.operation = aco.object.ADD;  // Attach existing world object with same ID
    aco.touch_links = touch_links_;

    planning_scene_interface_.applyAttachedCollisionObject(aco);
    RCLCPP_INFO(this->get_logger(), "Attached '%s' to '%s'", id.c_str(), attach_link_.c_str());

    // Still publish attach (harmless; useful for debugging/other nodes)
    aco_attach_pub_->publish(aco);
  }

  void detachObject()
  {
    const auto id = current_target_id();

    moveit_msgs::msg::AttachedCollisionObject aco;
    aco.link_name = attach_link_;
    aco.object.id = id;
    aco.object.operation = aco.object.REMOVE;  // Detach; object returns to world

    planning_scene_interface_.applyAttachedCollisionObject(aco);
    RCLCPP_INFO(this->get_logger(), "Detached '%s' from '%s'", id.c_str(), attach_link_.c_str());

    // NEW: publish detach event the sequencer will listen to
    aco_detach_pub_->publish(aco);
  }

  // ROS / MoveIt
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gripper_status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr spawned_id_sub_;
  rclcpp::Publisher<moveit_msgs::msg::AttachedCollisionObject>::SharedPtr aco_attach_pub_;
  rclcpp::Publisher<moveit_msgs::msg::AttachedCollisionObject>::SharedPtr aco_detach_pub_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

  // Config + state
  std::string default_object_id_;
  std::string attach_link_;
  std::vector<std::string> touch_links_;
  std::string latest_spawned_id_;  // updated by /spawned_collision_object_id
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AddCollisionObjects>());
  rclcpp::shutdown();
  return 0;
}

