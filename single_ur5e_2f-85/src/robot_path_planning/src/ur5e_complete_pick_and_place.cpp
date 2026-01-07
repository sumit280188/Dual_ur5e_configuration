#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <memory>
#include <chrono>
#include <vector>
#include <functional>

using namespace std::chrono_literals;

class UR5eWithGripperMoveGroup : public rclcpp::Node
{
public:
    UR5eWithGripperMoveGroup()
        : Node("ur5e_with_gripper_movegroup")
    {
        RCLCPP_INFO(get_logger(), "Initializing UR5e with Gripper MoveGroup Interface");
    }

    void initialize()
    {
        // Create MoveGroup interface for the arm and gripper
        move_group_arm_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(this->shared_from_this(), "ur_manipulator");
        move_group_gripper_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(this->shared_from_this(), "gripper");
        
        // Set higher planning time for better success rate
        move_group_arm_->allowReplanning(true);
        
        // Set planning speed to be slower for safety
        move_group_arm_->setMaxVelocityScalingFactor(0.5);
        move_group_arm_->setMaxAccelerationScalingFactor(0.5);

        // Set gripper planning parameters
        move_group_gripper_->setMaxVelocityScalingFactor(0.5);
        
        RCLCPP_INFO(get_logger(), "MoveGroup interfaces initialized");
        
        // Create a timer to start the sequence
        sequence_timer_ = this->create_wall_timer(3s, [this]() {
            static bool first_run = true;
            if (first_run) {
                start_sequence();
                first_run = false;
            }
            // Cancel the timer after first execution
            sequence_timer_->cancel();
        });
    }

    void start_sequence()
    {
        RCLCPP_INFO(get_logger(), "Starting pick-and-place sequence");
        current_step_ = 0;
        move_ur5e_to_initial_position();
    }

private:
    std::vector<rclcpp::TimerBase::SharedPtr> timers_;
    rclcpp::TimerBase::SharedPtr sequence_timer_;
    int current_step_ = 0;
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_arm_;
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_gripper_;
    int retry_count_ = 0;
    const int max_retries_ = 3;

    template <typename CallbackT>
    void create_timer_callback(std::chrono::milliseconds duration, CallbackT callback)
    {
        auto timer = this->create_wall_timer(duration, [callback, this](){ 
            callback();
            // Cancel the timer from within its own callback
            this->timers_.back()->cancel();
        });
        
        // Store the timer so it doesn't get destroyed
        timers_.push_back(timer);
    }

    void handle_error(const std::string& operation)
    {
        retry_count_++;
        
        if (retry_count_ <= max_retries_) {
            RCLCPP_WARN(get_logger(), "Retry attempt %d of %d for operation: %s", 
                retry_count_, max_retries_, operation.c_str());
            
            // Retry the current step after a delay
            create_timer_callback(2s, [this]() {
                switch (current_step_) {
                    case 0:
                    case 4:
                    case 8:
                        move_ur5e_to_initial_position();
                        break;
                    case 1:
                    case 5:
                        open_gripper();
                        break;
                    case 2:
                        move_ur5e_down();
                        break;
                    case 3:
                        close_gripper();
                        break;
                    case 6:
                        move_ur5e_down_after_new_position();
                        break;
                    case 7:
                        open_gripper();
                        break;
                    default:
                        RCLCPP_ERROR(get_logger(), "Unknown step to retry: %d", current_step_);
                }
            });
        } else {
            RCLCPP_ERROR(get_logger(), "Exceeded maximum retry attempts (%d). Aborting sequence.", max_retries_);
            RCLCPP_ERROR(get_logger(), "Failed operation: %s", operation.c_str());
            rclcpp::shutdown();
        }
    }

    void sequence_complete()
    {
        RCLCPP_INFO(get_logger(), "Pick-and-place sequence completed successfully!");
        rclcpp::shutdown();
    }

    void move_ur5e_to_initial_position()
    {
        RCLCPP_INFO(get_logger(), "Moving to initial position");
        
        // Set named target if available
        try {
            move_group_arm_->setNamedTarget("home");
        } catch (const std::exception& e) {
            RCLCPP_WARN(get_logger(), "Named target 'home' not available, using joint positions");
            
            // Set joint positions if named target not available
            std::vector<double> joint_positions = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            move_group_arm_->setJointValueTarget(joint_positions);
        }
        
        // Plan and execute
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_arm_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        
        if (success) {
            moveit::core::MoveItErrorCode execute_result = move_group_arm_->execute(plan);
            if (execute_result == moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_INFO(get_logger(), "Initial position reached");
                retry_count_ = 0; // Reset retry counter
                
                // Next step: open gripper
                create_timer_callback(500ms, [this]() {
                    open_gripper();
                    current_step_++;
                });
            } else {
                RCLCPP_ERROR(get_logger(), "Failed to execute plan to initial position");
                handle_error("Moving to initial position");
            }
        } else {
            RCLCPP_ERROR(get_logger(), "Failed to plan movement to initial position");
            handle_error("Planning to initial position");
        }
    }

    void open_gripper()
    {
        RCLCPP_INFO(get_logger(), "Opening gripper");
        
        // Get the current gripper joint values
        std::vector<double> gripper_joint_values = move_group_gripper_->getCurrentJointValues();
        
        // Target values for open gripper (0.8 seems to be open position from original code)
        std::map<std::string, double> target_joint_values;
        
        if (!gripper_joint_values.empty()) {
            target_joint_values["robotiq_85_left_knuckle_joint"] = 0.8;
        } else {
            RCLCPP_WARN(get_logger(), "Could not get current gripper joint values, using default");
            target_joint_values["robotiq_85_left_knuckle_joint"] = 0.8;
        }
        
        move_group_gripper_->setJointValueTarget(target_joint_values);
        
        // Plan and execute
        moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
        bool success = (move_group_gripper_->plan(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        
        if (success) {
            moveit::core::MoveItErrorCode execute_result = move_group_gripper_->execute(gripper_plan);
            if (execute_result == moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_INFO(get_logger(), "Gripper opened successfully");
                retry_count_ = 0; // Reset retry counter
                
                // Determine next step based on current step
                switch(current_step_) {
                    case 1:
                        create_timer_callback(500ms, [this]() {
                            move_ur5e_down();
                            current_step_++;
                        });
                        break;
                    case 5:
                        create_timer_callback(500ms, [this]() {
                            move_ur5e_down_after_new_position();
                            current_step_++;
                        });
                        break;
                    case 7:
                        create_timer_callback(500ms, [this]() {
                            move_ur5e_up_after_gripper();
                            current_step_++;
                        });
                        break;
                    default:
                        RCLCPP_ERROR(get_logger(), "Unexpected step after opening gripper: %d", current_step_);
                }
            } else {
                RCLCPP_ERROR(get_logger(), "Failed to execute plan to open gripper");
                handle_error("Executing open gripper");
            }
        } else {
            RCLCPP_ERROR(get_logger(), "Failed to plan movement to open gripper");
            handle_error("Planning open gripper");
        }
    }

    void close_gripper()
    {
        RCLCPP_INFO(get_logger(), "Closing gripper");
        
        // Target values for closed gripper (0.0 seems to be closed position from original code)
        std::map<std::string, double> target_joint_values;
        target_joint_values["robotiq_85_left_knuckle_joint"] = 0.0;
        
        move_group_gripper_->setJointValueTarget(target_joint_values);
        
        // Plan and execute
        moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
        bool success = (move_group_gripper_->plan(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        
        if (success) {
            moveit::core::MoveItErrorCode execute_result = move_group_gripper_->execute(gripper_plan);
            if (execute_result == moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_INFO(get_logger(), "Gripper closed successfully");
                retry_count_ = 0; // Reset retry counter
                
                // Move up after closing gripper
                create_timer_callback(500ms, [this]() {
                    move_ur5e_up();
                    current_step_++;
                });
            } else {
                RCLCPP_ERROR(get_logger(), "Failed to execute plan to close gripper");
                handle_error("Executing close gripper");
            }
        } else {
            RCLCPP_ERROR(get_logger(), "Failed to plan movement to close gripper");
            handle_error("Planning close gripper");
        }
    }

    void move_ur5e_down()
    {
        RCLCPP_INFO(get_logger(), "Moving UR5e down");
        execute_cartesian_motion(-0.05); // 5cm steps downward, made it larger for clarity
    }

    void move_ur5e_up()
    {
        RCLCPP_INFO(get_logger(), "Moving UR5e up");
        execute_cartesian_motion(0.05); // 5cm steps upward, made it larger for clarity
    }

    void execute_cartesian_motion(double delta_z)
    {
        // Wait for a valid current state
        int max_attempts = 5;
        bool valid_state = false;
        
        for (int attempt = 0; attempt < max_attempts; attempt++) {
            rclcpp::sleep_for(std::chrono::milliseconds(200));
            try {
                move_group_arm_->getCurrentState();
                valid_state = true;
                break;
            } catch (const std::exception& e) {
                RCLCPP_WARN(get_logger(), "Attempt %d: Waiting for valid robot state...", attempt + 1);
            }
        }
        
        if (!valid_state) {
            RCLCPP_ERROR(get_logger(), "Could not get valid robot state after %d attempts", max_attempts);
            handle_error("Getting robot state");
            return;
        }
        
        std::vector<geometry_msgs::msg::Pose> waypoints;
        geometry_msgs::msg::Pose current_pose = move_group_arm_->getCurrentPose().pose;
        
        // Create incremental waypoints
        geometry_msgs::msg::Pose target_pose = current_pose;
        target_pose.position.z += delta_z;
        waypoints.push_back(target_pose);
        
        move_ur5e_cartesian(waypoints);
    }

    void move_ur5e_cartesian(const std::vector<geometry_msgs::msg::Pose> &waypoints)
    {
        move_group_arm_->allowReplanning(true);
        
        // Try to set the start state to current state
        try {
            move_group_arm_->setStartStateToCurrentState();
        } catch (const std::exception& e) {
            RCLCPP_WARN(get_logger(), "Failed to set start state: %s", e.what());
            // Continue anyway - it might use the last known state
        }
        
        moveit_msgs::msg::RobotTrajectory trajectory;

        double eef_step = 0.005;     // Small steps for smooth motion
        double jump_threshold = 2.0; // Allow some jumping between joint configurations

        RCLCPP_INFO(get_logger(), "Computing Cartesian path...");
        double fraction = move_group_arm_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

        if (fraction > 0.95)
        {
            RCLCPP_INFO(get_logger(), "Executing Cartesian path (%.2f%% coverage)", fraction * 100);
            moveit::core::MoveItErrorCode execute_result = move_group_arm_->execute(trajectory);
            
            if (execute_result == moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_INFO(get_logger(), "Cartesian motion completed successfully");
                retry_count_ = 0; // Reset retry counter
                
                // Process next step based on current_step
                create_timer_callback(500ms, [this]() {
                    switch (current_step_) {
                        case 2:
                            close_gripper();
                            current_step_++;
                            break;
                        case 4:
                            move_ur5e_to_new_position();
                            current_step_++;
                            break;
                        case 6:
                            open_gripper();
                            current_step_++;
                            break;
                        case 8:
                            sequence_complete();
                            break;
                    }
                });
            } else {
                RCLCPP_ERROR(get_logger(), "Failed to execute Cartesian motion");
                handle_error("Executing Cartesian motion");
            }
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "Cartesian path failed (%.2f%% completion)", fraction * 100);
            // Fall back to joint-space planning instead
            if (fraction > 0.5) {
                RCLCPP_WARN(get_logger(), "Executing partial path");
                move_group_arm_->execute(trajectory);
                
                // Try to plan the rest with joint-space planning
                RCLCPP_INFO(get_logger(), "Planning remaining path with joint-space planner");
                fallback_to_joint_space_move(waypoints.back());
            } else {
                RCLCPP_WARN(get_logger(), "Falling back to joint space planning");
                fallback_to_joint_space_move(waypoints.back());
            }
        }
    }

    void fallback_to_joint_space_move(const geometry_msgs::msg::Pose& target_pose)
    {
        try {
            move_group_arm_->setPoseTarget(target_pose);
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            bool success = (move_group_arm_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
            if (success) {
                RCLCPP_INFO(get_logger(), "Joint space plan successful, executing...");
                moveit::core::MoveItErrorCode execute_result = move_group_arm_->execute(plan);
                
                if (execute_result == moveit::core::MoveItErrorCode::SUCCESS) {
                    RCLCPP_INFO(get_logger(), "Joint space motion completed successfully");
                    retry_count_ = 0; // Reset retry counter
                    
                    // Process next step based on current_step
                    create_timer_callback(500ms, [this]() {
                        switch (current_step_) {
                            case 2:
                                close_gripper();
                                current_step_++;
                                break;
                            case 4:
                                move_ur5e_to_new_position();
                                current_step_++;
                                break;
                            case 6:
                                open_gripper();
                                current_step_++;
                                break;
                            case 8:
                                sequence_complete();
                                break;
                        }
                    });
                } else {
                    RCLCPP_ERROR(get_logger(), "Failed to execute joint space plan");
                    handle_error("Executing joint space motion");
                }
            } else {
                RCLCPP_ERROR(get_logger(), "Joint space planning failed");
                try_simplified_movement();
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Exception in joint space planning: %s", e.what());
            try_simplified_movement();
        }
    }
    
    void try_simplified_movement()
    {
        RCLCPP_WARN(get_logger(), "Trying simplified movement");
        
        // Just move a single joint slightly as a last resort
        std::vector<double> joint_values;
        
        try {
            // Get current joint values if possible
            joint_values = move_group_arm_->getCurrentJointValues();
            
            // Slightly modify elbow joint
            if (current_step_ < 4) {
                joint_values[2] += 0.05; // Slightly bend elbow
            } else {
                joint_values[2] -= 0.05; // Slightly straighten elbow
            }
        } catch (const std::exception& e) {
            // Fallback to default positions with slight modification
            joint_values = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            
            if (current_step_ < 4) {
                joint_values[2] = 0.05; // Slightly bend elbow
            } else {
                joint_values[2] = -0.05; // Slightly straighten elbow
            }
        }
        
        move_group_arm_->setJointValueTarget(joint_values);
        
        moveit::planning_interface::MoveGroupInterface::Plan simple_plan;
        bool success = (move_group_arm_->plan(simple_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        
        if (success) {
            RCLCPP_INFO(get_logger(), "Simplified joint movement planned successfully");
            moveit::core::MoveItErrorCode execute_result = move_group_arm_->execute(simple_plan);
            
            if (execute_result == moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_INFO(get_logger(), "Simplified movement completed successfully");
                retry_count_ = 0; // Reset retry counter
                
                // Process next step
                create_timer_callback(500ms, [this]() {
                    switch (current_step_) {
                        case 2:
                            close_gripper();
                            current_step_++;
                            break;
                        case 4:
                            move_ur5e_to_new_position();
                            current_step_++;
                            break;
                        case 6:
                            open_gripper();
                            current_step_++;
                            break;
                        case 8:
                            sequence_complete();
                            break;
                    }
                });
            } else {
                RCLCPP_ERROR(get_logger(), "Failed to execute simplified movement");
                handle_error("Executing simplified movement");
            }
        } else {
            RCLCPP_ERROR(get_logger(), "Failed to plan simplified movement");
            handle_error("Planning simplified movement");
        }
    }

    void move_ur5e_to_new_position()
    {
        RCLCPP_INFO(get_logger(), "Moving to new position");
        
        // Set joint positions from original code
        std::vector<double> joint_positions = {0.0, -1.57, -1.57, 0.0, 0.0, 0.0};
        move_group_arm_->setJointValueTarget(joint_positions);
        
        // Plan and execute
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_arm_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        
        if (success) {
            moveit::core::MoveItErrorCode execute_result = move_group_arm_->execute(plan);
            if (execute_result == moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_INFO(get_logger(), "New position reached");
                retry_count_ = 0; // Reset retry counter
                
                // Next step: open gripper
                create_timer_callback(500ms, [this]() {
                    open_gripper();
                    current_step_++;
                });
            } else {
                RCLCPP_ERROR(get_logger(), "Failed to execute plan to new position");
                handle_error("Executing movement to new position");
            }
        } else {
            RCLCPP_ERROR(get_logger(), "Failed to plan movement to new position");
            handle_error("Planning movement to new position");
        }
    }

    void move_ur5e_down_after_new_position()
    {
        move_ur5e_down();
    }

    void move_ur5e_up_after_gripper()
    {
        move_ur5e_up();
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    // Create the node
    auto node = std::make_shared<UR5eWithGripperMoveGroup>();
    
    // Create a single executor
    auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    
    // Add the node to the executor
    executor->add_node(node);
    
    // Do a short spin to initialize ROS communications before MoveIt initialization
    std::thread([&executor]() { 
        for (int i = 0; i < 10; i++) {
            executor->spin_some(100ms);
            std::this_thread::sleep_for(100ms);
        }
    }).join();
    
    // Initialize the MoveGroup interfaces
    node->initialize();
    
    // Spin the executor
    executor->spin();
    
    rclcpp::shutdown();
    return 0;
}