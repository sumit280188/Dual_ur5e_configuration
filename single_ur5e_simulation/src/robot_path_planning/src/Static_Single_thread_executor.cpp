#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <thread>
#include <chrono>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);

    // Spin the node in a separate thread using StaticSingleThreadedExecutor
    rclcpp::executors::StaticSingleThreadedExecutor executor;
    executor.add_node(move_group_node);
    std::thread([&executor]()
                { executor.spin(); })
        .detach();

    static const std::string PLANNING_GROUP_ARM = "ur_manipulator";
    moveit::planning_interface::MoveGroupInterface move_group_arm(move_group_node, PLANNING_GROUP_ARM);

    // ----------------- Pregrasp Position -----------------
    RCLCPP_INFO(LOGGER, "Moving to Pregrasp Position...");
    geometry_msgs::msg::Pose target_pose1;
    target_pose1.orientation.x = 0.708;
    target_pose1.orientation.y = -0.706;
    target_pose1.orientation.z = 0.0;
    target_pose1.orientation.w = 0.0;
    target_pose1.position.x = 0.491;
    target_pose1.position.y = 0.133;
    target_pose1.position.z = 0.488;

    move_group_arm.setPlanningTime(5.0);
    move_group_arm.allowReplanning(true);
    move_group_arm.setPoseTarget(target_pose1);

    if (move_group_arm.move() != moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_ERROR(LOGGER, "Failed to reach pregrasp position!");
        rclcpp::shutdown();
        return 1;
    }

    // ----------------- Approach Object -----------------
    RCLCPP_INFO(LOGGER, "Approaching Object...");
    std::vector<geometry_msgs::msg::Pose> approach_waypoints;
    geometry_msgs::msg::Pose start_pose = move_group_arm.getCurrentPose().pose;

    for (int i = 0; i < 6; ++i) 
    {
        geometry_msgs::msg::Pose new_pose = start_pose;
        new_pose.position.z -= 0.01 * i;
        approach_waypoints.push_back(new_pose);
    }

    moveit_msgs::msg::RobotTrajectory trajectory_approach;
    const double eef_step = 0.001;
    const double jump_threshold = 0.0;
    double fraction = move_group_arm.computeCartesianPath(
        approach_waypoints, eef_step, jump_threshold, trajectory_approach);

    if (fraction >= 0.9)
    {
        move_group_arm.execute(trajectory_approach);
    }
    else
    {
        RCLCPP_ERROR(LOGGER, "Approach path planning failed! Fraction: %.2f", fraction);
        rclcpp::shutdown();
        return 1;
    }

    // ----------------- Retreat -----------------
    RCLCPP_INFO(LOGGER, "Retreating...");
    std::vector<geometry_msgs::msg::Pose> retreat_waypoints;
    geometry_msgs::msg::Pose current_pose = move_group_arm.getCurrentPose().pose;

    for (int i = 0; i < 6; ++i)
    {
        geometry_msgs::msg::Pose new_pose = current_pose;
        new_pose.position.z += 0.01 * i;
        retreat_waypoints.push_back(new_pose);
    }

    moveit_msgs::msg::RobotTrajectory trajectory_retreat;
    fraction = move_group_arm.computeCartesianPath(
        retreat_waypoints, eef_step, jump_threshold, trajectory_retreat);

    if (fraction >= 0.9)
    {
        move_group_arm.execute(trajectory_retreat);
    }
    else
    {
        RCLCPP_ERROR(LOGGER, "Retreat path planning failed! Fraction: %.2f", fraction);
    }

    rclcpp::shutdown();
    return 0;
}
