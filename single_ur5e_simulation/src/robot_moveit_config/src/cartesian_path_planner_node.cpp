#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_parameterization.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

class CartesianPathPlanner : public rclcpp::Node
{
public:
    CartesianPathPlanner() : Node("cartesian_path_planner_node"),
                             move_group_(std::shared_ptr<rclcpp::Node>(this), "ur_manipulator")
    {
        // Wait until valid robot state is received
        auto current_state = move_group_.getCurrentState(10); // Wait up to 10 seconds
        if (!current_state)
        {
            RCLCPP_FATAL(this->get_logger(), "Failed to get current robot state");
            return;
        }

        planCartesianPath();
    }

    void planCartesianPath()
    {
        RCLCPP_INFO(this->get_logger(), "Planning Cartesian path...");

        // Get the current pose of the end effector
        geometry_msgs::msg::Pose start_pose = move_group_.getCurrentPose().pose;

        // Define waypoints
        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(start_pose);

        // Define target pose
        geometry_msgs::msg::Pose target_pose = start_pose;
        target_pose.position.x += 0.10; // 10 cm forward
        waypoints.push_back(target_pose);

        // Compute Cartesian Path
        moveit_msgs::msg::RobotTrajectory trajectory;
        double fraction = move_group_.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

        if (fraction < 0.95)
        {
            RCLCPP_WARN(this->get_logger(), "Only %.2f%% of the path was planned", fraction * 100.0);
            return;
        }

        // Apply time parameterization to the trajectory
        robot_trajectory::RobotTrajectory rt(move_group_.getCurrentState()->getRobotModel(), "ur_manipulator");
        rt.setRobotTrajectoryMsg(*move_group_.getCurrentState(), trajectory);

        trajectory_processing::IterativeParabolicTimeParameterization time_param;
        time_param.computeTimeStamps(rt);
        rt.getRobotTrajectoryMsg(trajectory);

        RCLCPP_INFO(this->get_logger(), "Cartesian path planning completed successfully");
    }

private:
    moveit::planning_interface::MoveGroupInterface move_group_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CartesianPathPlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}