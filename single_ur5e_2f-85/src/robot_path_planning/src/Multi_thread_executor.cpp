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
        // Use a timer to delay planning until after executor starts spinning
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),  // Short delay to ensure full initialization
            [this]() {
                this->timer_->cancel();  // One-shot timer
                this->planCartesianPath();
            });
    }

    void planCartesianPath()
    {
        RCLCPP_INFO(this->get_logger(), "Starting Cartesian path planning...");

        // Wait for valid robot state (asynchronous)
        moveit::core::RobotStatePtr current_state;
        int retry_count = 0;
        const int max_retries = 5;
        
        while (retry_count++ < max_retries) {
            current_state = move_group_.getCurrentState(30.0);
            if (current_state) break;
            RCLCPP_WARN(this->get_logger(), "Waiting for current robot state... (attempt %d/%d)", 
                        retry_count, max_retries);
        }

        if (!current_state) {
            RCLCPP_FATAL(this->get_logger(), "Failed to get current robot state after %d attempts", max_retries);
            rclcpp::shutdown();
            return;
        }

        // Get current pose
        geometry_msgs::msg::Pose start_pose = move_group_.getCurrentPose().pose;

        // Define waypoints
        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(start_pose);

        geometry_msgs::msg::Pose target_pose = start_pose;
        target_pose.position.x += 0.10;
        waypoints.push_back(target_pose);

        // Compute Cartesian path
        moveit_msgs::msg::RobotTrajectory trajectory;
        const double step_size = 0.001;
        const double jump_threshold = 2.0;
        
        double fraction = move_group_.computeCartesianPath(
            waypoints, step_size, jump_threshold, trajectory);

        if (fraction < 0.95) {
            RCLCPP_ERROR(this->get_logger(), 
                        "Failed to compute complete path (%.2f%% achieved)", 
                        fraction * 100.0);
            rclcpp::shutdown();
            return;
        }

        // Time parameterization
        robot_trajectory::RobotTrajectory rt(
            move_group_.getCurrentState()->getRobotModel(), 
            move_group_.getName());
        rt.setRobotTrajectoryMsg(*current_state, trajectory);

        trajectory_processing::IterativeParabolicTimeParameterization time_param;
        if (!time_param.computeTimeStamps(rt)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to compute time stamps");
            rclcpp::shutdown();
            return;
        }

        rt.getRobotTrajectoryMsg(trajectory);

        RCLCPP_INFO(this->get_logger(), 
                   "Successfully planned trajectory with %zu points",
                   trajectory.joint_trajectory.points.size());

        // Execute the trajectory (add your execution logic here)
        // move_group_.execute(trajectory);

        rclcpp::shutdown();
    }

private:
    moveit::planning_interface::MoveGroupInterface move_group_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<CartesianPathPlanner>();
    
    // Configure multi-threaded executor
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    
    // Run executor in separate thread
    std::thread executor_thread([&executor]() {
        executor.spin();
    });
    
    // Wait for planning to complete
    executor_thread.join();
    
    return 0;
}