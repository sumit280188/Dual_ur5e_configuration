#include "dual_robot_path_planning/guid_monitor.hpp"
#include "dual_robot_path_planning/pick_place_node.hpp"
#include <fstream>
#include <nlohmann/json.hpp>
#include <algorithm>

using json = nlohmann::json;

GuidMonitor::GuidMonitor(rclcpp::Node::SharedPtr node, PickPlaceNode* pick_place_node)
    : node_(node)
    , pick_place_node_(pick_place_node)
    , spawned_guid_("")
    , pre_attached_guid_("")
    , last_processed_guid_("")
{
    // Load and parse JSON file at startup
    loadJsonSequencing();

    // Create subscribers
    spawned_collision_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/spawned_collision_object_id",
        10,
        std::bind(&GuidMonitor::spawnedGuidCallback, this, std::placeholders::_1));

    pre_attached_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/bim/pre_attached_id",
        10,
        std::bind(&GuidMonitor::preAttachedGuidCallback, this, std::placeholders::_1));

    RCLCPP_INFO(node_->get_logger(), "GUID Monitor initialized successfully");
}

void GuidMonitor::loadJsonSequencing()
{
    const std::string json_path = 
        "/home/robotik/workspace/src/dual_ur5e_simulation/src/"
        "dual_robot_path_planning/resource/JSON_Sequencing_File/Sequencing.json";

    try
    {
        std::ifstream file(json_path);
        if (!file.is_open())
        {
            RCLCPP_ERROR(node_->get_logger(), 
                "Failed to open JSON file: %s", json_path.c_str());
            return;
        }

        json j;
        file >> j;

        // Parse items array
        if (j.contains("items") && j["items"].is_array())
        {
            for (const auto& item : j["items"])
            {
                SequenceItem seq_item;
                seq_item.guid = item["guid"].get<std::string>();
                seq_item.x = item["point"]["x"].get<double>();
                seq_item.y = item["point"]["y"].get<double>();
                seq_item.z = item["point"]["z"].get<double>();
                seq_item.montage_index = item["montage_index"].get<int>();
                
                sequence_items_.push_back(seq_item);
            }

            RCLCPP_INFO(node_->get_logger(), 
                "Loaded %zu items from JSON sequencing file", 
                sequence_items_.size());
        }
        else
        {
            RCLCPP_WARN(node_->get_logger(), 
                "JSON file does not contain 'items' array");
        }
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(node_->get_logger(), 
            "Exception while loading JSON: %s", e.what());
    }
}

void GuidMonitor::spawnedGuidCallback(const std_msgs::msg::String::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(guid_mutex_);
    spawned_guid_ = msg->data;
    
    RCLCPP_INFO(node_->get_logger(), 
        "Received spawned GUID: %s", spawned_guid_.c_str());
    
    // Check and process
    checkAndProcessGuids();
}

void GuidMonitor::preAttachedGuidCallback(const std_msgs::msg::String::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(guid_mutex_);
    pre_attached_guid_ = msg->data;
    
    RCLCPP_INFO(node_->get_logger(), 
        "Received pre-attached GUID: %s", pre_attached_guid_.c_str());
    
    // Check and process
    checkAndProcessGuids();
}

void GuidMonitor::checkAndProcessGuids()
{
    // Note: This function is called with guid_mutex_ already locked
    
    if (spawned_guid_.empty() || pre_attached_guid_.empty())
        return;

    if (spawned_guid_ == pre_attached_guid_)
    {
        RCLCPP_INFO(node_->get_logger(),
            "GUIDs match (%s). Doing nothing.", spawned_guid_.c_str());
        return;
    }

    if (spawned_guid_ == last_processed_guid_)
        return;  // already handled

    last_processed_guid_ = spawned_guid_;

    auto it = std::find_if(sequence_items_.begin(), sequence_items_.end(),
        [this](const SequenceItem& item)
        { return item.guid == spawned_guid_; });

    if (it == sequence_items_.end())
    {
        RCLCPP_WARN(node_->get_logger(),
            "GUID '%s' not found in JSON database.", spawned_guid_.c_str());
        return;
    }

    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = it->x;
    target_pose.position.y = it->y;
    target_pose.position.z = it->z;
    target_pose.orientation.x = 0.509;
    target_pose.orientation.y = -0.491;
    target_pose.orientation.z = -0.494;
    target_pose.orientation.w = 0.506;

    RCLCPP_INFO(node_->get_logger(),
        "GUID changed: %s → Moving robot to (%.3f, %.3f, %.3f)",
        spawned_guid_.c_str(), it->x, it->y, it->z);

    if (pick_place_node_)
        pick_place_node_->movingToDropLocation(target_pose);
}