#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/action/execute_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <thread>
#include <chrono>
#include <functional>
#include <atomic>
#include <condition_variable>
#include <mutex>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("pick_place_demo");

class PickPlaceNode
{
public:
    PickPlaceNode(rclcpp::Node::SharedPtr node)
    : node_(node)
    {
        // Define planning groups
        arm_group_name_ = "ur_manipulator";
        gripper_group_name_ = "gripper";

        // Initialize move groups
        move_group_arm_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, arm_group_name_);
        move_group_gripper_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, gripper_group_name_);

        // Set up planning parameters
        move_group_arm_->setPlanningTime(15.0);
        move_group_arm_->allowReplanning(true);
        move_group_gripper_->setMaxVelocityScalingFactor(1.0);

        // Initialize execution feedback handler
        trajectory_action_client_ = rclcpp_action::create_client<moveit_msgs::action::ExecuteTrajectory>(
            node_, "/execute_trajectory");

        // Wait for action server to be available
        if (!trajectory_action_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(LOGGER, "Execute trajectory action server not available after 5 seconds");
            throw std::runtime_error("Action server not available");
        }
        RCLCPP_INFO(LOGGER, "Execute trajectory action server is available");
    }

    void executePickAndPlaceTask()
    {
        // Reset motion completion status
        motion_completed_ = false;
        motion_success_ = false;

        // 1. Move to Pregrasp Position
        RCLCPP_INFO(LOGGER, "Step 1: Moving to Pregrasp Position...");
        if (!moveToPregraspPosition()) {
            RCLCPP_ERROR(LOGGER, "Failed to move to pregrasp position. Aborting task.");
            return;
        }
        
        // 2. Open Gripper
        RCLCPP_INFO(LOGGER, "Step 2: Opening Gripper...");
        if (!openGripper()) {
            RCLCPP_ERROR(LOGGER, "Failed to open gripper. Aborting task.");
            return;
        }
        
        // 3. Move to Grasp Position (30 cm down)
        RCLCPP_INFO(LOGGER, "Step 3: Moving to Grasp Position (30 cm down)...");
        if (!moveDownToGraspPosition()) {
            RCLCPP_ERROR(LOGGER, "Failed to move to grasp position. Aborting task.");
            return;
        }
        
        // 4. Close Gripper to 50%
        RCLCPP_INFO(LOGGER, "Step 4: Closing Gripper to half...");
        if (!closeGripperPartially()) {
            RCLCPP_ERROR(LOGGER, "Failed to close gripper. Aborting task.");
            return;
        }
        
        // 5. Lift to Pregrasp Position (30 cm up)
        RCLCPP_INFO(LOGGER, "Step 5: Lifting to Pregrasp Position...");
        if (!liftToPregraspPosition()) {
            RCLCPP_ERROR(LOGGER, "Failed to lift to pregrasp position. Aborting task.");
            return;
        }
        
        RCLCPP_INFO(LOGGER, "Pick and Place task completed successfully!");
    }

private:
    rclcpp::Node::SharedPtr node_;
    std::string arm_group_name_;
    std::string gripper_group_name_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_arm_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_gripper_;
    rclcpp_action::Client<moveit_msgs::action::ExecuteTrajectory>::SharedPtr trajectory_action_client_;
    
    // Synchronization variables for action feedback
    std::atomic<bool> motion_completed_{false};
    std::atomic<bool> motion_success_{false};
    std::mutex mutex_;
    std::condition_variable cv_;

    bool moveToPregraspPosition()
    {
        // Define pregrasp pose
        geometry_msgs::msg::Pose target_pose;
        target_pose.orientation.x = 0.707;
        target_pose.orientation.y = -0.707;
        target_pose.orientation.z = 0.0;
        target_pose.orientation.w = 0.0;
        target_pose.position.x = 0.5;
        target_pose.position.y = 0.0;
        target_pose.position.z = 0.5;

        move_group_arm_->setPoseTarget(target_pose);

        // Plan the motion
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_arm_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        
        if (!success) {
            RCLCPP_ERROR(LOGGER, "Failed to plan motion to pregrasp position");
            return false;
        }
        
        RCLCPP_INFO(LOGGER, "Executing planned path to pregrasp position");
        
        // Use direct execution first for more stability
        return (move_group_arm_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    }

    bool openGripper()
    {
        // Set gripper to fully open position
        move_group_gripper_->setJointValueTarget("robotiq_85_left_knuckle_joint", 0.0);
        
        moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
        bool success = (move_group_gripper_->plan(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        
        if (!success) {
            RCLCPP_ERROR(LOGGER, "Failed to plan gripper opening");
            return false;
        }

        RCLCPP_INFO(LOGGER, "Executing gripper opening");
        return (move_group_gripper_->execute(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    }

    bool moveDownToGraspPosition()
    {
        // Get current pose
        geometry_msgs::msg::Pose start_pose = move_group_arm_->getCurrentPose().pose;
        
        // Create waypoints for cartesian path (30 cm down)
        std::vector<geometry_msgs::msg::Pose> waypoints;
        geometry_msgs::msg::Pose target_pose = start_pose;
        target_pose.position.z -= 0.3;  // 30 cm down
        waypoints.push_back(target_pose);
        
        // Compute cartesian path
        moveit_msgs::msg::RobotTrajectory trajectory;
        const double eef_step = 0.01;  // 1 cm resolution
        const double jump_threshold = 0.0;  // disable jump threshold checking
        double fraction = move_group_arm_->computeCartesianPath(
            waypoints, eef_step, jump_threshold, trajectory);
            
        if (fraction < 0.9) {
            RCLCPP_ERROR(LOGGER, "Cartesian path planning failed (%.2f%% achieved)", fraction * 100.0);
            return false;
        }
        
        // Check if trajectory is valid
        if (trajectory.joint_trajectory.points.empty() && trajectory.multi_dof_joint_trajectory.points.empty()) {
            RCLCPP_ERROR(LOGGER, "Computed trajectory is empty");
            return false;
        }
        
        RCLCPP_INFO(LOGGER, "Executing downward movement");
        return (move_group_arm_->execute(trajectory) == moveit::core::MoveItErrorCode::SUCCESS);
    }

    bool closeGripperPartially()
    {
        // Set gripper to 50% closed
        // Assuming the gripper joint range is 0.0 (closed) to 0.8 (open)
        double half_closed_value = 0.4;  // 50% of 0.8
        
        move_group_gripper_->setJointValueTarget("robotiq_85_left_knuckle_joint", half_closed_value);
        
        moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
        bool success = (move_group_gripper_->plan(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        
        if (!success) {
            RCLCPP_ERROR(LOGGER, "Failed to plan gripper closing");
            return false;
        }

        RCLCPP_INFO(LOGGER, "Executing gripper closing");
        return (move_group_gripper_->execute(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    }

    bool liftToPregraspPosition()
    {
        // Get current pose
        geometry_msgs::msg::Pose start_pose = move_group_arm_->getCurrentPose().pose;
        
        // Create waypoints for cartesian path (30 cm up)
        std::vector<geometry_msgs::msg::Pose> waypoints;
        geometry_msgs::msg::Pose target_pose = start_pose;
        target_pose.position.z += 0.3;  // 30 cm up
        waypoints.push_back(target_pose);
        
        // Compute cartesian path
        moveit_msgs::msg::RobotTrajectory trajectory;
        const double eef_step = 0.01;  // 1 cm resolution
        const double jump_threshold = 0.0;  // disable jump threshold checking
        double fraction = move_group_arm_->computeCartesianPath(
            waypoints, eef_step, jump_threshold, trajectory);
            
        if (fraction < 0.9) {
            RCLCPP_ERROR(LOGGER, "Cartesian path planning failed (%.2f%% achieved)", fraction * 100.0);
            return false;
        }
        
        // Check if trajectory is valid
        if (trajectory.joint_trajectory.points.empty() && trajectory.multi_dof_joint_trajectory.points.empty()) {
            RCLCPP_ERROR(LOGGER, "Computed trajectory is empty");
            return false;
        }
        
        RCLCPP_INFO(LOGGER, "Executing lifting movement");
        return (move_group_arm_->execute(trajectory) == moveit::core::MoveItErrorCode::SUCCESS);
    }

    // Advanced execution with action client feedback
    // This method can be used later once the basic execution works
    bool executeTrajectoryWithFeedback(const moveit_msgs::msg::RobotTrajectory& trajectory)
    {
        // Check if trajectory is valid
        if (trajectory.joint_trajectory.points.empty() && trajectory.multi_dof_joint_trajectory.points.empty()) {
            RCLCPP_ERROR(LOGGER, "Trajectory is empty, cannot execute");
            return false;
        }
        
        // Reset status
        motion_completed_ = false;
        motion_success_ = false;
        
        // Create goal
        auto goal_msg = moveit_msgs::action::ExecuteTrajectory::Goal();
        goal_msg.trajectory = trajectory;
        
        // Send goal and setup callbacks
        auto send_goal_options = rclcpp_action::Client<moveit_msgs::action::ExecuteTrajectory>::SendGoalOptions();
        
        // Feedback callback
        send_goal_options.feedback_callback = 
        [this](rclcpp_action::ClientGoalHandle<moveit_msgs::action::ExecuteTrajectory>::SharedPtr,
               const std::shared_ptr<const moveit_msgs::action::ExecuteTrajectory::Feedback> feedback) {
            // Process feedback safely
            RCLCPP_INFO(LOGGER, "Received feedback: %s", feedback->state.c_str());
        };
        
        // Goal response callback
        send_goal_options.goal_response_callback =
            [this](const rclcpp_action::ClientGoalHandle<moveit_msgs::action::ExecuteTrajectory>::SharedPtr& goal_handle) {
                if (!goal_handle) {
                    RCLCPP_ERROR(LOGGER, "Goal was rejected by server");
                    {
                        std::lock_guard<std::mutex> lock(mutex_);
                        motion_completed_ = true;
                        motion_success_ = false;
                    }
                    cv_.notify_one();
                } else {
                    RCLCPP_INFO(LOGGER, "Goal accepted by server");
                }
            };
        
        // Result callback
        send_goal_options.result_callback =
            [this](const rclcpp_action::ClientGoalHandle<moveit_msgs::action::ExecuteTrajectory>::WrappedResult& result) {
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    motion_completed_ = true;
                    
                    switch (result.code) {
                        case rclcpp_action::ResultCode::SUCCEEDED:
                            RCLCPP_INFO(LOGGER, "Motion completed successfully!");
                            motion_success_ = true;
                            break;
                        case rclcpp_action::ResultCode::ABORTED:
                            RCLCPP_ERROR(LOGGER, "Motion aborted");
                            motion_success_ = false;
                            break;
                        case rclcpp_action::ResultCode::CANCELED:
                            RCLCPP_WARN(LOGGER, "Motion canceled");
                            motion_success_ = false;
                            break;
                        default:
                            RCLCPP_ERROR(LOGGER, "Unknown result code");
                            motion_success_ = false;
                            break;
                    }
                }
                cv_.notify_one();
            };
            
        try {
            // Send the goal
            auto goal_handle_future = trajectory_action_client_->async_send_goal(goal_msg, send_goal_options);
            
            // Wait for the future to complete
            if (rclcpp::spin_until_future_complete(node_, goal_handle_future) !=
                rclcpp::FutureReturnCode::SUCCESS) {
                RCLCPP_ERROR(LOGGER, "Failed to send goal");
                return false;
            }
            
            // Wait for the motion to complete (with timeout)
            {
                std::unique_lock<std::mutex> lock(mutex_);
                if (!cv_.wait_for(lock, std::chrono::seconds(30), [this] { return motion_completed_.load(); })) {
                    RCLCPP_ERROR(LOGGER, "Timeout waiting for motion to complete");
                    return false;
                }
            }
            
            return motion_success_;
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(LOGGER, "Exception during trajectory execution: %s", e.what());
            return false;
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("pick_place_demo", node_options);

    try {
        // Create and set up a multi-threaded executor for improved performance
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(node);
        
        // Start the executor in a separate thread
        std::thread executor_thread([&executor]() {
            executor.spin();
        });
        
        // Wait a moment for the executor to start properly
        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        RCLCPP_INFO(LOGGER, "Initializing pick and place node...");
        PickPlaceNode pick_place_node(node);

        // Execute the pick and place task
        pick_place_node.executePickAndPlaceTask();

        // Clean up
        executor.cancel();
        executor_thread.join();
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(LOGGER, "Exception during pick and place execution: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}

//The code does not use ROS2 topics or subscriptions to track motion completion; 
//it relies entirely on the blocking calls to MoveIt's execute functions, which return only after the motion is completed (or fails).