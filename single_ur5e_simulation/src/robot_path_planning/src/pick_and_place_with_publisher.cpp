#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/action/execute_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp> // Added missing header
#include <thread>
#include <chrono>
#include <functional>
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("pick_place_publisher_demo");

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
        move_group_arm_->setPlanningTime(20.0);
        move_group_arm_->allowReplanning(true);
        move_group_arm_->setMaxVelocityScalingFactor(0.5);
        move_group_arm_->setNumPlanningAttempts(10);
        move_group_arm_->setMaxAccelerationScalingFactor(0.5);
        move_group_arm_->setGoalTolerance(0.01);
        move_group_arm_->setGoalJointTolerance(0.01);

        move_group_gripper_->setMaxVelocityScalingFactor(0.5);

        // Initialize execution feedback handler
        trajectory_action_client_ = rclcpp_action::create_client<moveit_msgs::action::ExecuteTrajectory>(
            node_, "/execute_trajectory");

        // Wait for action server to be available
        if (!trajectory_action_client_->wait_for_action_server(std::chrono::seconds(5)))
        {
            RCLCPP_ERROR(LOGGER, "Execute trajectory action server not available after 5 seconds");
            throw std::runtime_error("Action server not available");
        }
        RCLCPP_INFO(LOGGER, "Execute trajectory action server is available");

        arm_status_publisher_ = node_->create_publisher<std_msgs::msg::String>(
            "/ur_manipulator/status", 10);
        gripper_status_publisher_ = node_->create_publisher<std_msgs::msg::String>(
            "/gripper/status", 10);
    }

    void executePickAndPlaceTask()
    {
        // Reset motion completion status
        motion_completed_ = false;
        motion_success_ = false;

        // 1. Move to Pregrasp Position
        RCLCPP_INFO(LOGGER, "Step 1: Moving to Pregrasp Position");
        if (!moveToPregraspPosition())
        {
            RCLCPP_ERROR(LOGGER, "Failed to move to pregrasp position. Aborting task.");
            return;
        }

        // 2. Open Gripper
        RCLCPP_INFO(LOGGER, "Step 2: Opening Gripper");
        if (!openGripper())
        {
            RCLCPP_ERROR(LOGGER, "Failed to open gripper. Aborting task.");
            return;
        }

        // 3. Move to Grasp Position (30 cm down)
        RCLCPP_INFO(LOGGER, "Step 3: Moving to Grasp Position (30 cm down)");
        if (!moveDownToGraspPosition())
        {
            RCLCPP_ERROR(LOGGER, "Failed to move to grasp position. Aborting task.");
            return;
        }

        // 4. Close Gripper to 50%
        RCLCPP_INFO(LOGGER, "Step 4: Closing Gripper to half");
        if (!closeGripperPartially())
        {
            RCLCPP_ERROR(LOGGER, "Failed to close gripper. Aborting task.");
            return;
        }

        // 5. Lift to Pregrasp Position (30 cm up)
        RCLCPP_INFO(LOGGER, "Step 5: Lifting to Pregrasp Position");
        if (!liftToPregraspPosition())
        {
            RCLCPP_ERROR(LOGGER, "Failed to lift to pregrasp position. Aborting task.");
            return;
        }

        // 6. Move to Drop Location
        RCLCPP_INFO(LOGGER, "Step 6: Moving to Drop Location");
        if (!movingToDropLocation())
        {
            RCLCPP_ERROR(LOGGER, "Failed to move to drop location. Aborting task.");
            return;
        }

        // 7. Move Down to Drop Position
        RCLCPP_INFO(LOGGER, "Step 7: Moving Down to Drop Position");
        if (!moveDownToDropPosition())
        {
            RCLCPP_ERROR(LOGGER, "Failed to move down to drop position. Aborting task.");
            return;
        }

        // 8. Open Gripper FULLY to release object
        RCLCPP_INFO(LOGGER, "Step 8: Opening Gripper Fully to Release Object");
        if (!openGripperFully())
        {
            RCLCPP_ERROR(LOGGER, "Failed to open gripper fully. Aborting task.");
            return;
        }

        // 9. Move Up from Drop Position
        RCLCPP_INFO(LOGGER, "Step 9: Moving Up from Drop Position");
        if (!moveUpFromDropPosition())
        {
            RCLCPP_ERROR(LOGGER, "Failed to move up from drop position. Aborting task.");
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

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr arm_status_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gripper_status_publisher_;

    void publishArmStatus(const std::string &status)
    {
        auto msg = std_msgs::msg::String();
        msg.data = "[UR Manipulator] " + status;
        arm_status_publisher_->publish(msg);
        RCLCPP_INFO(LOGGER, "%s", msg.data.c_str());
    }

    void publishGripperStatus(const std::string &status)
    {
        auto msg = std_msgs::msg::String();
        msg.data = "[Gripper] " + status;
        gripper_status_publisher_->publish(msg);
        RCLCPP_INFO(LOGGER, "%s", msg.data.c_str());
    }

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
        target_pose.orientation.x = 0.721;
        target_pose.orientation.y = -0.693;
        target_pose.orientation.z = -0.006;
        target_pose.orientation.w = 0.006;
        target_pose.position.x = 0.695;
        target_pose.position.y = 0.168;
        target_pose.position.z = 0.500; //changed from 520 to 525

        move_group_arm_->setPoseTarget(target_pose);

        publishArmStatus("Planning motion to pregrasp position");

        // Plan the motion
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_arm_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (!success)
        {
            publishArmStatus("Failed to plan motion to pregrasp position");
            return false;
        }

        publishArmStatus("Executing trajectory to pregrasp position");
        bool execution_success = (move_group_arm_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (execution_success)
        {
            publishArmStatus("Trajectory execution completed successfully");
        }
        else
        {
            publishArmStatus("Trajectory execution failed");
        }

        return execution_success;
    }

    bool openGripper()
    {
        publishGripperStatus("Planning gripper opening");

        // Set gripper to fully open position
        move_group_gripper_->setJointValueTarget("robotiq_85_left_knuckle_joint", 0.0);

        moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
        bool success = (move_group_gripper_->plan(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (!success)
        {
            publishGripperStatus("Failed to plan gripper opening");
            return false;
        }

        publishGripperStatus("Executing gripper opening");
        bool execution_success = (move_group_gripper_->execute(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (execution_success)
        {
            publishGripperStatus("Gripper opened successfully");
        }
        else
        {
            publishGripperStatus("Gripper opening failed");
        }

        return execution_success;
    }

    bool moveDownToGraspPosition()
    {
        publishArmStatus("Moving down to grasp position");
        // Get current pose
        geometry_msgs::msg::Pose start_pose = move_group_arm_->getCurrentPose().pose;

        // Create waypoints for cartesian path (30 cm down)
        std::vector<geometry_msgs::msg::Pose> waypoints;

        // Instead of a single 30cm motion, create multiple smaller waypoints
        const int num_segments = 10;
        const double step_size = 0.3 / num_segments; // 30cm divided into smaller steps

        for (int i = 1; i <= num_segments; i++)
        {
            geometry_msgs::msg::Pose intermediate_pose = start_pose;
            intermediate_pose.position.z -= (step_size * i);
            waypoints.push_back(intermediate_pose);
        }

        // Compute cartesian path with finer resolution
        moveit_msgs::msg::RobotTrajectory trajectory;
        const double eef_step = 0.01;      // 1cm resolution (finer)
        const double jump_threshold = 0.0; // disable jump threshold checking

        double fraction = move_group_arm_->computeCartesianPath(
            waypoints, eef_step, jump_threshold, trajectory);

        if (fraction < 0.9)
        {
            RCLCPP_ERROR(LOGGER, "Cartesian path planning failed (%.2f%% achieved)", fraction * 100.0);
            publishArmStatus("Cartesian path planning failed");

            // If cartesian planning fails, try joint-space planning as a fallback
            publishArmStatus("Attempting joint-space planning instead");

            geometry_msgs::msg::Pose target_pose = start_pose;
            target_pose.position.z -= 0.3; // 30cm down

            move_group_arm_->setPoseTarget(target_pose);

            // Slow down for better planning
            move_group_arm_->setMaxVelocityScalingFactor(0.3);
            moveit::planning_interface::MoveGroupInterface::Plan joint_plan;

            bool success = (move_group_arm_->plan(joint_plan) == moveit::core::MoveItErrorCode::SUCCESS);
            if (!success)
            {
                publishArmStatus("Joint-space planning also failed");
                move_group_arm_->setMaxVelocityScalingFactor(0.5); // restore original value
                return false;
            }

            bool execution_success = (move_group_arm_->execute(joint_plan) == moveit::core::MoveItErrorCode::SUCCESS);
            move_group_arm_->setMaxVelocityScalingFactor(0.5); // restore original value

            if (execution_success)
            {
                publishArmStatus("Joint-space motion executed successfully");
                return true;
            }
            else
            {
                publishArmStatus("Joint-space motion execution failed");
                return false;
            }
        }

        // Check if trajectory is valid
        if (trajectory.joint_trajectory.points.empty() && trajectory.multi_dof_joint_trajectory.points.empty())
        {
            RCLCPP_ERROR(LOGGER, "Computed trajectory is empty");
            publishArmStatus("Computed trajectory is empty");
            return false;
        }

        // Scale down velocity for smoother execution
        robot_trajectory::RobotTrajectory rt(move_group_arm_->getRobotModel(), arm_group_name_);
        rt.setRobotTrajectoryMsg(*move_group_arm_->getCurrentState(), trajectory);

        trajectory_processing::IterativeParabolicTimeParameterization iptp;
        iptp.computeTimeStamps(rt, 0.3, 0.3); // 30% of max velocity and acceleration for smoother motion
        rt.getRobotTrajectoryMsg(trajectory);

        publishArmStatus("Executing trajectory to grasp position");
        bool execution_success = (move_group_arm_->execute(trajectory) == moveit::core::MoveItErrorCode::SUCCESS);

        if (execution_success)
        {
            publishArmStatus("Trajectory execution completed successfully");
        }
        else
        {
            publishArmStatus("Trajectory execution failed");
        }

        return execution_success;
    }

    bool closeGripperPartially()
    {
        publishGripperStatus("Planning gripper closing");
        // Set gripper to 50% closed
        // Assuming the gripper joint range is 0.0 (open) to 0.8 (closed)
        double half_closed_value = 0.4551; // 50% of 0.8

        move_group_gripper_->setJointValueTarget("robotiq_85_left_knuckle_joint", half_closed_value);

        moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
        bool success = (move_group_gripper_->plan(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (!success)
        {
            publishGripperStatus("Failed to plan gripper closing");
            return false;
        }

        publishGripperStatus("Executing gripper closing");
        bool execution_success = (move_group_gripper_->execute(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (execution_success)
        {
            publishGripperStatus("Gripper closed successfully");
        }
        else
        {
            publishGripperStatus("Gripper closing failed");
        }

        return execution_success;
    }

    bool liftToPregraspPosition()
    {
        publishArmStatus("Planning motion back to pregrasp position");
        // Get current pose
        geometry_msgs::msg::Pose start_pose = move_group_arm_->getCurrentPose().pose;

        // Create waypoints for cartesian path (30 cm up)
        std::vector<geometry_msgs::msg::Pose> waypoints;

        // Instead of a single 30cm motion, create multiple smaller waypoints
        const int num_segments = 10;
        const double step_size = 0.3 / num_segments; // 30cm divided into smaller steps

        for (int i = 1; i <= num_segments; i++)
        {
            geometry_msgs::msg::Pose intermediate_pose = start_pose;
            intermediate_pose.position.z += (step_size * i);
            waypoints.push_back(intermediate_pose);
        }

        // Compute cartesian path with finer resolution
        moveit_msgs::msg::RobotTrajectory trajectory;
        const double eef_step = 0.01;      // 1cm resolution (finer)
        const double jump_threshold = 0.0; // disable jump threshold checking

        double fraction = move_group_arm_->computeCartesianPath(
            waypoints, eef_step, jump_threshold, trajectory);

        if (fraction < 0.9)
        {
            RCLCPP_ERROR(LOGGER, "Cartesian path planning failed (%.2f%% achieved)", fraction * 100.0);
            publishArmStatus("Cartesian path planning failed");

            // If cartesian planning fails, try joint-space planning as a fallback
            publishArmStatus("Attempting joint-space planning instead");

            geometry_msgs::msg::Pose target_pose = start_pose;
            target_pose.position.z += 0.3; // 30cm up

            move_group_arm_->setPoseTarget(target_pose);

            // Slow down for better planning
            move_group_arm_->setMaxVelocityScalingFactor(0.3);
            moveit::planning_interface::MoveGroupInterface::Plan joint_plan;

            bool success = (move_group_arm_->plan(joint_plan) == moveit::core::MoveItErrorCode::SUCCESS);
            if (!success)
            {
                publishArmStatus("Joint-space planning also failed");
                move_group_arm_->setMaxVelocityScalingFactor(0.5); // restore original value
                return false;
            }

            bool execution_success = (move_group_arm_->execute(joint_plan) == moveit::core::MoveItErrorCode::SUCCESS);
            move_group_arm_->setMaxVelocityScalingFactor(0.5); // restore original value

            if (execution_success)
            {
                publishArmStatus("Joint-space motion executed successfully");
                return true;
            }
            else
            {
                publishArmStatus("Joint-space motion execution failed");
                return false;
            }
        }
        // Check if trajectory is valid
        if (trajectory.joint_trajectory.points.empty() && trajectory.multi_dof_joint_trajectory.points.empty())
        {
            RCLCPP_ERROR(LOGGER, "Computed trajectory is empty");
            publishArmStatus("Computed trajectory is empty");
            return false;
        }

        // Scale down velocity for smoother execution
        robot_trajectory::RobotTrajectory rt(move_group_arm_->getRobotModel(), arm_group_name_);
        rt.setRobotTrajectoryMsg(*move_group_arm_->getCurrentState(), trajectory);

        trajectory_processing::IterativeParabolicTimeParameterization iptp;
        iptp.computeTimeStamps(rt, 0.3, 0.3); // 30% of max velocity and acceleration for smoother motion
        rt.getRobotTrajectoryMsg(trajectory);

        publishArmStatus("Executing trajectory back to pregrasp position");
        bool execution_success = (move_group_arm_->execute(trajectory) == moveit::core::MoveItErrorCode::SUCCESS);

        if (execution_success)
        {
            publishArmStatus("Trajectory execution completed successfully");
        }
        else
        {
            publishArmStatus("Trajectory execution failed");
        }

        return execution_success;
    }

    bool movingToDropLocation()
    {
        publishArmStatus("Planning motion to drop location");

        // Define target pose for drop location
        geometry_msgs::msg::Pose target_pose1;
        target_pose1.orientation.x = 0.722;
        target_pose1.orientation.y = 0.692;
        target_pose1.orientation.z = 0.006;
        target_pose1.orientation.w = 0.006;
        target_pose1.position.x = -0.707;
        target_pose1.position.y = -0.110;
        target_pose1.position.z = 0.520;

        // Get current pose
        geometry_msgs::msg::Pose current_pose = move_group_arm_->getCurrentPose().pose;

        // Create waypoints for a smoother path (intermediate points)
        std::vector<geometry_msgs::msg::Pose> waypoints;

        // Add current pose as starting point
        waypoints.push_back(current_pose);

        // Calculate intermediate poses (3 points)
        for (int i = 1; i <= 3; i++)
        {
            geometry_msgs::msg::Pose intermediate_pose;

            // Interpolate position linearly
            double factor = static_cast<double>(i) / 4.0; // 1/4, 2/4, 3/4
            intermediate_pose.position.x = current_pose.position.x +
                                           factor * (target_pose1.position.x - current_pose.position.x);
            intermediate_pose.position.y = current_pose.position.y +
                                           factor * (target_pose1.position.y - current_pose.position.y);
            intermediate_pose.position.z = current_pose.position.z +
                                           factor * (target_pose1.position.z - current_pose.position.z);

            // For simplicity, use target orientation for intermediates
            // In a more advanced implementation, you would do SLERP for orientation
            intermediate_pose.orientation = target_pose1.orientation;

            waypoints.push_back(intermediate_pose);
        }

        // Add final target pose
        waypoints.push_back(target_pose1);

        // Try Cartesian path planning for smoother motion
        publishArmStatus("Computing Cartesian path to drop location");

        moveit_msgs::msg::RobotTrajectory trajectory;
        const double eef_step = 0.05;      // 5cm resolution
        const double jump_threshold = 0.0; // disable jump threshold checking

        double fraction = move_group_arm_->computeCartesianPath(
            waypoints, eef_step, jump_threshold, trajectory);

        if (fraction >= 0.9)
        {
            // Cartesian path planning succeeded
            publishArmStatus("Cartesian path computed successfully (" +
                             std::to_string(fraction * 100.0) + "% achieved)");

            // Scale down the velocity and acceleration
            robot_trajectory::RobotTrajectory rt(move_group_arm_->getRobotModel(), arm_group_name_);
            rt.setRobotTrajectoryMsg(*move_group_arm_->getCurrentState(), trajectory);

            trajectory_processing::IterativeParabolicTimeParameterization iptp;
            iptp.computeTimeStamps(rt, 0.3, 0.3); // 30% of max velocity and acceleration

            rt.getRobotTrajectoryMsg(trajectory);

            publishArmStatus("Executing smooth trajectory to drop location");
            bool execution_success = (move_group_arm_->execute(trajectory) == moveit::core::MoveItErrorCode::SUCCESS);

            if (execution_success)
            {
                publishArmStatus("Trajectory execution to drop location completed successfully");
            }
            else
            {
                publishArmStatus("Trajectory execution to drop location failed");
            }

            return execution_success;
        }
        else
        {
            // Cartesian planning failed, fall back to regular joint-space planning
            publishArmStatus("Cartesian planning failed (" +
                             std::to_string(fraction * 100.0) + "% achieved), using joint-space planning");

            move_group_arm_->setPoseTarget(target_pose1);

            // Slow down the movement speed
            move_group_arm_->setMaxVelocityScalingFactor(0.3);
            move_group_arm_->setMaxAccelerationScalingFactor(0.3);

            // Plan the motion
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            bool success = (move_group_arm_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

            if (!success)
            {
                publishArmStatus("Failed to plan motion to drop location");
                // Restore original velocity scaling
                move_group_arm_->setMaxVelocityScalingFactor(0.5);
                move_group_arm_->setMaxAccelerationScalingFactor(0.5);
                return false;
            }

            publishArmStatus("Executing trajectory to drop location");
            bool execution_success = (move_group_arm_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);

            // Restore original velocity scaling
            move_group_arm_->setMaxVelocityScalingFactor(0.5);
            move_group_arm_->setMaxAccelerationScalingFactor(0.5);

            if (execution_success)
            {
                publishArmStatus("Trajectory execution to drop location completed successfully");
            }
            else
            {
                publishArmStatus("Trajectory execution to drop location failed");
            }

            return execution_success;
        }
    }

    bool moveDownToDropPosition()
    {
        publishArmStatus("Moving down to drop position");

        // Get current pose
        geometry_msgs::msg::Pose start_pose = move_group_arm_->getCurrentPose().pose;

        // Create waypoints for cartesian path (30 cm down)
        std::vector<geometry_msgs::msg::Pose> waypoints;

        // Instead of a single 30cm motion, create multiple smaller waypoints
        const int num_segments = 10;
        const double step_size = 0.3 / num_segments; // 30cm divided into smaller steps

        for (int i = 1; i <= num_segments; i++)
        {
            geometry_msgs::msg::Pose intermediate_pose = start_pose;
            intermediate_pose.position.z -= (step_size * i);
            waypoints.push_back(intermediate_pose);
        }

        // Compute cartesian path with finer resolution
        moveit_msgs::msg::RobotTrajectory trajectory;
        const double eef_step = 0.01;      // 1cm resolution (finer)
        const double jump_threshold = 0.0; // disable jump threshold checking

        double fraction = move_group_arm_->computeCartesianPath(
            waypoints, eef_step, jump_threshold, trajectory);

        if (fraction < 0.9)
        {
            RCLCPP_ERROR(LOGGER, "Cartesian path planning failed (%.2f%% achieved)", fraction * 100.0);
            publishArmStatus("Cartesian path planning failed");

            // If cartesian planning fails, try joint-space planning as a fallback
            publishArmStatus("Attempting joint-space planning instead");

            geometry_msgs::msg::Pose target_pose = start_pose;
            target_pose.position.z -= 0.3; // 30cm down

            move_group_arm_->setPoseTarget(target_pose);

            // Slow down for better planning
            move_group_arm_->setMaxVelocityScalingFactor(0.3);
            moveit::planning_interface::MoveGroupInterface::Plan joint_plan;

            bool success = (move_group_arm_->plan(joint_plan) == moveit::core::MoveItErrorCode::SUCCESS);
            if (!success)
            {
                publishArmStatus("Joint-space planning also failed");
                move_group_arm_->setMaxVelocityScalingFactor(0.5); // restore original value
                return false;
            }

            bool execution_success = (move_group_arm_->execute(joint_plan) == moveit::core::MoveItErrorCode::SUCCESS);
            move_group_arm_->setMaxVelocityScalingFactor(0.5); // restore original value

            if (execution_success)
            {
                publishArmStatus("Joint-space motion executed successfully");
                return true;
            }
            else
            {
                publishArmStatus("Joint-space motion execution failed");
                return false;
            }
        }

        // Check if trajectory is valid
        if (trajectory.joint_trajectory.points.empty() && trajectory.multi_dof_joint_trajectory.points.empty())
        {
            RCLCPP_ERROR(LOGGER, "Computed trajectory is empty");
            publishArmStatus("Computed trajectory is empty");
            return false;
        }

        // Scale down velocity for smoother execution
        robot_trajectory::RobotTrajectory rt(move_group_arm_->getRobotModel(), arm_group_name_);
        rt.setRobotTrajectoryMsg(*move_group_arm_->getCurrentState(), trajectory);

        trajectory_processing::IterativeParabolicTimeParameterization iptp;
        iptp.computeTimeStamps(rt, 0.3, 0.3); // 30% of max velocity and acceleration for smoother motion
        rt.getRobotTrajectoryMsg(trajectory);

        publishArmStatus("Executing trajectory down to drop position");
        bool execution_success = (move_group_arm_->execute(trajectory) == moveit::core::MoveItErrorCode::SUCCESS);

        if (execution_success)
        {
            publishArmStatus("Trajectory execution down to drop position completed successfully");
        }
        else
        {
            publishArmStatus("Trajectory execution down to drop position failed");
        }

        return execution_success;
    }

    // New function to open gripper fully to release object
    bool openGripperFully()
    {
        publishGripperStatus("Planning gripper opening to fully release object");

        // Set gripper to fully open position (0.0 represents fully open)
        move_group_gripper_->setJointValueTarget("robotiq_85_left_knuckle_joint", 0.0);

        moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
        bool success = (move_group_gripper_->plan(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (!success)
        {
            publishGripperStatus("Failed to plan full gripper opening for object release");
            return false;
        }

        publishGripperStatus("Executing full gripper opening to release object");
        bool execution_success = (move_group_gripper_->execute(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (execution_success)
        {
            publishGripperStatus("Gripper fully opened successfully - object released");
        }
        else
        {
            publishGripperStatus("Failed to fully open gripper for object release");
        }

        return execution_success;
    }

    bool moveUpFromDropPosition()
    {
        publishArmStatus("Moving up from drop position");
        // Get current pose
        geometry_msgs::msg::Pose start_pose = move_group_arm_->getCurrentPose().pose;

        // Create waypoints for cartesian path (30 cm up)
        std::vector<geometry_msgs::msg::Pose> waypoints;

        // Instead of a single 30cm motion, create multiple smaller waypoints
        const int num_segments = 10;
        const double step_size = 0.3 / num_segments; // 30cm divided into smaller steps

        for (int i = 1; i <= num_segments; i++)
        {
            geometry_msgs::msg::Pose intermediate_pose = start_pose;
            intermediate_pose.position.z += (step_size * i);
            waypoints.push_back(intermediate_pose);
        }

        // Compute cartesian path with finer resolution
        moveit_msgs::msg::RobotTrajectory trajectory;
        const double eef_step = 0.01;      // 1cm resolution (finer)
        const double jump_threshold = 0.0; // disable jump threshold checking

        double fraction = move_group_arm_->computeCartesianPath(
            waypoints, eef_step, jump_threshold, trajectory);

        if (fraction < 0.9)
        {
            RCLCPP_ERROR(LOGGER, "Cartesian path planning failed (%.2f%% achieved)", fraction * 100.0);
            publishArmStatus("Cartesian path planning failed");

            // If cartesian planning fails, try joint-space planning as a fallback
            publishArmStatus("Attempting joint-space planning instead");

            geometry_msgs::msg::Pose target_pose = start_pose;
            target_pose.position.z += 0.3; // 30cm up

            move_group_arm_->setPoseTarget(target_pose);

            // Slow down for better planning
            move_group_arm_->setMaxVelocityScalingFactor(0.3);
            moveit::planning_interface::MoveGroupInterface::Plan joint_plan;

            bool success = (move_group_arm_->plan(joint_plan) == moveit::core::MoveItErrorCode::SUCCESS);
            if (!success)
            {
                publishArmStatus("Joint-space planning also failed");
                move_group_arm_->setMaxVelocityScalingFactor(0.5); // restore original value
                return false;
            }

            bool execution_success = (move_group_arm_->execute(joint_plan) == moveit::core::MoveItErrorCode::SUCCESS);
            move_group_arm_->setMaxVelocityScalingFactor(0.5); // restore original value

            if (execution_success)
            {
                publishArmStatus("Joint-space motion executed successfully");
                return true;
            }
            else
            {
                publishArmStatus("Joint-space motion execution failed");
                return false;
            }
        }

        // Check if trajectory is valid
        if (trajectory.joint_trajectory.points.empty() && trajectory.multi_dof_joint_trajectory.points.empty())
        {
            RCLCPP_ERROR(LOGGER, "Computed trajectory is empty");
            publishArmStatus("Computed trajectory is empty");
            return false;
        }

        // Scale down velocity for smoother execution
        robot_trajectory::RobotTrajectory rt(move_group_arm_->getRobotModel(), arm_group_name_);
        rt.setRobotTrajectoryMsg(*move_group_arm_->getCurrentState(), trajectory);

        trajectory_processing::IterativeParabolicTimeParameterization iptp;
        iptp.computeTimeStamps(rt, 0.3, 0.3); // 30% of max velocity and acceleration for smoother motion
        rt.getRobotTrajectoryMsg(trajectory);

        publishArmStatus("Executing trajectory up from drop position");
        bool execution_success = (move_group_arm_->execute(trajectory) == moveit::core::MoveItErrorCode::SUCCESS);

        if (execution_success)
        {
            publishArmStatus("Trajectory execution up from drop position completed successfully");
        }
        else
        {
            publishArmStatus("Trajectory execution up from drop position failed");
        }

        return execution_success;
    }

    // Advanced execution with action client feedback
    // This method can be used later once the basic execution works
    bool executeTrajectoryWithFeedback(const moveit_msgs::msg::RobotTrajectory &trajectory)
    {
        // Check if trajectory is valid
        if (trajectory.joint_trajectory.points.empty() && trajectory.multi_dof_joint_trajectory.points.empty())
        {
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
                   const std::shared_ptr<const moveit_msgs::action::ExecuteTrajectory::Feedback> feedback)
        {
            // Process feedback safely
            RCLCPP_INFO(LOGGER, "Received feedback: %s", feedback->state.c_str());
        };

        // Goal response callback
        send_goal_options.goal_response_callback =
            [this](const rclcpp_action::ClientGoalHandle<moveit_msgs::action::ExecuteTrajectory>::SharedPtr &goal_handle)
        {
            if (!goal_handle)
            {
                RCLCPP_ERROR(LOGGER, "Goal was rejected by server");
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    motion_completed_ = true;
                    motion_success_ = false;
                }
                cv_.notify_one();
            }
            else
            {
                RCLCPP_INFO(LOGGER, "Goal accepted by server");
            }
        };

        // Result callback
        send_goal_options.result_callback =
            [this](const rclcpp_action::ClientGoalHandle<moveit_msgs::action::ExecuteTrajectory>::WrappedResult &result)
        {
            {
                std::lock_guard<std::mutex> lock(mutex_);
                motion_completed_ = true;

                switch (result.code)
                {
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

        try
        {
            // Send the goal
            auto goal_handle_future = trajectory_action_client_->async_send_goal(goal_msg, send_goal_options);

            // Wait for the future to complete
            if (rclcpp::spin_until_future_complete(node_, goal_handle_future) !=
                rclcpp::FutureReturnCode::SUCCESS)
            {
                RCLCPP_ERROR(LOGGER, "Failed to send goal");
                return false;
            }

            // Wait for the motion to complete (with timeout)
            {
                std::unique_lock<std::mutex> lock(mutex_);
                if (!cv_.wait_for(lock, std::chrono::seconds(30), [this]
                                  { return motion_completed_.load(); }))
                {
                    RCLCPP_ERROR(LOGGER, "Timeout waiting for motion to complete");
                    return false;
                }
            }

            return motion_success_;
        }
        catch (const std::exception &e)
        {
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
    auto node = rclcpp::Node::make_shared("pick_place_publisher_demo", node_options);

    try
    {
        // Create and set up a multi-threaded executor for improved performance
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(node);

        // Start the executor in a separate thread
        std::thread executor_thread([&executor]()
                                    { executor.spin(); });

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
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(LOGGER, "Exception during pick and place execution: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}