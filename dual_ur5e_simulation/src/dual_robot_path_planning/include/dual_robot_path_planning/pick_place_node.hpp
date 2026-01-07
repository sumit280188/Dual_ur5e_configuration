#ifndef PICK_PLACE_NODE_HPP
#define PICK_PLACE_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/action/execute_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <memory>

class GuidMonitor;  // forward declaration

class PickPlaceNode
{
public:
    PickPlaceNode(rclcpp::Node::SharedPtr node);
    void executePickAndPlaceTask();
    
    // Public method for GUID Monitor to call
    bool movingToDropLocation(geometry_msgs::msg::Pose& target_pose1);

private:
    rclcpp::Node::SharedPtr node_;
    std::string arm_group_name_;
    std::string bob_gripper_group_name_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_arm_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_bob_gripper_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr arm_status_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr bob_gripper_status_publisher_;

    rclcpp_action::Client<moveit_msgs::action::ExecuteTrajectory>::SharedPtr trajectory_action_client_;

    // Synchronization variables for action feedback
    std::atomic<bool> motion_completed_{false};
    std::atomic<bool> motion_success_{false};
    std::mutex mutex_;
    std::condition_variable cv_;

    // GUID Monitor instance
    std::unique_ptr<GuidMonitor> guid_monitor_;

    // Helper methods
    void publishArmStatus(const std::string &status);
    void publishbob_gripperStatus(const std::string &status);
    bool moveToPregraspPosition();
    bool openbob_gripper();
    bool moveDownToGraspPosition();
    bool closebob_gripperPartially();
    bool liftToPregraspPosition();
    bool movingToDropLocation();  // Original without parameters
    bool moveDownToDropPosition();
    bool openbob_gripperFully();
    bool moveUpFromDropPosition();
    bool executeTrajectoryWithFeedback(const moveit_msgs::msg::RobotTrajectory &trajectory);
};

#endif // PICK_PLACE_NODE_HPP