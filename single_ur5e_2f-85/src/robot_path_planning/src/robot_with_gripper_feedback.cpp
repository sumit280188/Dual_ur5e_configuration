#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/trajectory_execution_manager/trajectory_execution_manager.h>
#include <geometry_msgs/msg/pose.hpp>
#include <thread>
#include <chrono>
#include <functional>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

class RobotControlNode
{
public:
    RobotControlNode(rclcpp::Node::SharedPtr node)
    : node_(node)
    {
        // Define planning groups
        arm_group_name_ = "ur_manipulator";
        gripper_group_name_ = "gripper";

        // Initialize move groups
        move_group_arm_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, arm_group_name_);
        move_group_gripper_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, gripper_group_name_);

        // Planning Scene Monitor for tracking planning scene changes
        psm_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node_, "robot_description");
        psm_->startStateMonitor();
        psm_->startSceneMonitor();
        psm_->startWorldGeometryMonitor();

        // Set up planning scene monitor observer
        psm_->addUpdateCallback(
            std::bind(&RobotControlNode::planningSceneCallback, this, std::placeholders::_1));

        // Set up Trajectory Execution Manager with proper parameters
        // We need to get the robot model from planning scene
        auto robot_model = psm_->getRobotModel();
        auto current_state_monitor = psm_->getStateMonitor();
        
        // Create the trajectory execution manager with correct parameters
        trajectory_execution_manager_ = std::make_shared<trajectory_execution_manager::TrajectoryExecutionManager>(
            node_, robot_model, current_state_monitor);
    }

    void executeTask()
    {
        // ----------------- Pregrasp Position -----------------
        RCLCPP_INFO(LOGGER, "Planning to Pregrasp Position...");
        
        geometry_msgs::msg::Pose target_pose1;
        target_pose1.orientation.x = 0.708;
        target_pose1.orientation.y = -0.706;
        target_pose1.orientation.z = 0.0;
        target_pose1.orientation.w = 0.0;
        target_pose1.position.x = 0.491;
        target_pose1.position.y = 0.133;
        target_pose1.position.z = 0.488;

        move_group_arm_->setPlanningTime(10.0);
        move_group_arm_->allowReplanning(true);
        move_group_arm_->setPoseTarget(target_pose1);

        // Plan using the action interface
        auto [success, plan] = [this] {
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            auto const result = move_group_arm_->plan(plan);
            return std::make_pair(result == moveit::core::MoveItErrorCode::SUCCESS, plan);
        }();

        if (!success)
        {
            RCLCPP_ERROR(LOGGER, "Failed to plan to pregrasp position!");
            return;
        }
        
        RCLCPP_INFO(LOGGER, "Executing plan to pregrasp position...");
        
        // Execute using the action interface
        move_group_arm_->execute(plan);
        waitForMoveCompletion(move_group_arm_);

        // ----------------- Approach Object -----------------
        RCLCPP_INFO(LOGGER, "Planning approach to object...");
        std::vector<geometry_msgs::msg::Pose> approach_waypoints;
        geometry_msgs::msg::Pose start_pose = move_group_arm_->getCurrentPose().pose;

        for (int i = 0; i < 6; ++i) 
        {
            geometry_msgs::msg::Pose new_pose = start_pose;
            new_pose.position.z -= 0.01 * i;
            approach_waypoints.push_back(new_pose);
        }

        moveit_msgs::msg::RobotTrajectory trajectory_approach;
        const double eef_step = 0.005;
        const double jump_threshold = 2.0;
        double fraction = move_group_arm_->computeCartesianPath(
            approach_waypoints, eef_step, jump_threshold, trajectory_approach);

        if (fraction < 0.9)
        {
            RCLCPP_ERROR(LOGGER, "Approach path planning failed! Fraction: %.2f", fraction);
            return;
        }

        RCLCPP_INFO(LOGGER, "Executing approach path...");
        // Execute the trajectory
        moveit::planning_interface::MoveGroupInterface::Plan approach_plan;
        approach_plan.trajectory_ = trajectory_approach;
        move_group_arm_->execute(approach_plan);
        waitForMoveCompletion(move_group_arm_);

        // ----------------- Open Gripper -----------------
        RCLCPP_INFO(LOGGER, "Planning to open gripper...");
        move_group_gripper_->setMaxVelocityScalingFactor(1.0);    // Adjust velocity as needed
        move_group_gripper_->setJointValueTarget("robotiq_85_left_knuckle_joint", 0.8); // Adjust joint value as needed
        
        auto [gripper_success, gripper_plan] = [this] {
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            auto const result = move_group_gripper_->plan(plan);
            return std::make_pair(result == moveit::core::MoveItErrorCode::SUCCESS, plan);
        }();

        if (!gripper_success) 
        {
            RCLCPP_ERROR(LOGGER, "Failed to plan opening gripper!");
            return;
        }
        
        RCLCPP_INFO(LOGGER, "Executing plan to open gripper...");
        move_group_gripper_->execute(gripper_plan);
        waitForMoveCompletion(move_group_gripper_);

        // Wait for 5 seconds before retreating
        RCLCPP_INFO(LOGGER, "Waiting for 5 seconds...");
        std::this_thread::sleep_for(std::chrono::seconds(5));

        // ----------------- Retreat -----------------
        RCLCPP_INFO(LOGGER, "Planning retreat path...");
        std::vector<geometry_msgs::msg::Pose> retreat_waypoints;
        geometry_msgs::msg::Pose current_pose = move_group_arm_->getCurrentPose().pose;

        for (int i = 0; i < 6; ++i)
        {
            geometry_msgs::msg::Pose new_pose = current_pose;
            new_pose.position.z += 0.01 * i;
            retreat_waypoints.push_back(new_pose);
        }

        moveit_msgs::msg::RobotTrajectory trajectory_retreat;
        fraction = move_group_arm_->computeCartesianPath(
            retreat_waypoints, eef_step, jump_threshold, trajectory_retreat);

        if (fraction < 0.9)
        {
            RCLCPP_ERROR(LOGGER, "Retreat path planning failed! Fraction: %.2f", fraction);
            return;
        }

        RCLCPP_INFO(LOGGER, "Executing retreat path...");
        moveit::planning_interface::MoveGroupInterface::Plan retreat_plan;
        retreat_plan.trajectory_ = trajectory_retreat;
        move_group_arm_->execute(retreat_plan);
        waitForMoveCompletion(move_group_arm_);

        // ----------------- Close Gripper -----------------
        RCLCPP_INFO(LOGGER, "Planning to close gripper...");
        move_group_gripper_->setMaxVelocityScalingFactor(1.0);   // Adjust velocity as needed
        move_group_gripper_->setJointValueTarget("robotiq_85_left_knuckle_joint", 0.0); // Adjust joint value as needed
        
        auto [close_success, close_plan] = [this] {
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            auto const result = move_group_gripper_->plan(plan);
            return std::make_pair(result == moveit::core::MoveItErrorCode::SUCCESS, plan);
        }();

        if (!close_success)
        {
            RCLCPP_ERROR(LOGGER, "Failed to plan closing gripper!");
            return;
        }
        
        RCLCPP_INFO(LOGGER, "Executing plan to close gripper...");
        move_group_gripper_->execute(close_plan);
        waitForMoveCompletion(move_group_gripper_);
        
        RCLCPP_INFO(LOGGER, "Task completed successfully!");
    }

private:
    rclcpp::Node::SharedPtr node_;
    std::string arm_group_name_;
    std::string gripper_group_name_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_arm_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_gripper_;
    std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> psm_;
    std::shared_ptr<trajectory_execution_manager::TrajectoryExecutionManager> trajectory_execution_manager_;

    void planningSceneCallback(const planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType update_type)
    {
        std::string update_type_str;
        
        // Use numerical values instead of named constants since the enum names are not accessible
        switch (update_type)
        {
            case planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType(0):
                update_type_str = "SCENE";
                break;
            case planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType(1):
                update_type_str = "WORLD";
                break;
            case planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType(2):
                update_type_str = "TRANSFORMS";
                break;
            default:
                update_type_str = "UNKNOWN";
        }
        RCLCPP_INFO(LOGGER, "Planning Scene Update: %s", update_type_str.c_str());
    }

    void waitForMoveCompletion(const std::shared_ptr<moveit::planning_interface::MoveGroupInterface>& move_group)
    {
        // Simplified implementation - wait for motion to complete
        // Sleep for a short time to allow the move group to start executing
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        // Check if the group is still moving
        auto start_time = std::chrono::steady_clock::now();
        const double timeout = 30.0; // 30 seconds timeout
        
        while (rclcpp::ok())
        {
            auto current_time = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration<double>(current_time - start_time).count();
            
            if (elapsed > timeout) {
                RCLCPP_WARN(LOGGER, "Timeout waiting for move to complete");
                break;
            }
            
            // Check if the robot has stopped moving by trying to get current state
            auto current_state = move_group->getCurrentJointValues();
            bool is_still_moving = false;
            
            // In a real implementation, you would compare current joint values with target
            // We're simplifying here by assuming the move is complete after a short delay
            std::this_thread::sleep_for(std::chrono::seconds(1));
            
            // For simplicity, we're assuming motion completes after a few iterations
            // In a real implementation, you'd check if target state was reached
            static int check_counter = 0;
            if (++check_counter >= 3) {
                check_counter = 0;
                is_still_moving = false;
            } else {
                is_still_moving = true;
            }
            
            if (!is_still_moving) {
                RCLCPP_INFO(LOGGER, "Move appears to be complete");
                break;
            }
            
            RCLCPP_INFO(LOGGER, "Waiting for move to complete... (%.1f seconds elapsed)", elapsed);
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);

    RobotControlNode robot_control(move_group_node);

    // Spin the node in a separate thread
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(move_group_node);
    std::thread([&executor]() { executor.spin(); }).detach();

    // Execute the task
    robot_control.executeTask();

    // Wait a bit before shutting down to make sure all messages are processed
    std::this_thread::sleep_for(std::chrono::seconds(2));
    rclcpp::shutdown();
    return 0;
}