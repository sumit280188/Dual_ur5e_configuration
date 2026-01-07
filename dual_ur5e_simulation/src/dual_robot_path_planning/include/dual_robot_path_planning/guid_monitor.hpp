#ifndef GUID_MONITOR_HPP
#define GUID_MONITOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <vector>
#include <mutex>
#include <memory>
#include <string>

class PickPlaceNode;  // forward declaration

class GuidMonitor
{
public:
    GuidMonitor(rclcpp::Node::SharedPtr node, PickPlaceNode* pick_place_node);
    
private:
    void checkAndProcessGuids();
    void spawnedGuidCallback(const std_msgs::msg::String::SharedPtr msg);
    void preAttachedGuidCallback(const std_msgs::msg::String::SharedPtr msg);
    void loadJsonSequencing();
    
    // Struct to hold parsed JSON data
    struct SequenceItem
    {
        std::string guid;
        double x, y, z;
        int montage_index;
    };
    
    // Members
    rclcpp::Node::SharedPtr node_;
    PickPlaceNode* pick_place_node_;
    std::mutex guid_mutex_;
    std::string spawned_guid_;
    std::string pre_attached_guid_;
    std::string last_processed_guid_;
    std::vector<SequenceItem> sequence_items_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr spawned_collision_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr pre_attached_sub_;
};

#endif // GUID_MONITOR_HPP