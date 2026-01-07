#include "dual_robot_path_planning/pick_place_node.hpp"
#include "dual_robot_path_planning/guid_monitor.hpp"
#include <thread>
#include <chrono>
#include <functional>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("pick_place_node");

PickPlaceNode::PickPlaceNode(rclcpp::Node::SharedPtr node)
    : node_(node)
{
    // Define planning groups
    arm_group_name_ = "bob_arm";
    bob_gripper_group_name_ = "bob_gripper";

    // Initialize move groups
    move_group_arm_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, arm_group_name_);
    move_group_bob_gripper_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, bob_gripper_group_name_);

    // Set up planning parameters
    move_group_arm_->setPlanningTime(20.0);
    move_group_arm_->allowReplanning(true);
    move_group_arm_->setMaxVelocityScalingFactor(0.5);
    move_group_arm_->setNumPlanningAttempts(10);
    move_group_arm_->setMaxAccelerationScalingFactor(0.5);
    move_group_arm_->setGoalTolerance(0.01);
    move_group_arm_->setGoalJointTolerance(0.01);

    move_group_bob_gripper_->setMaxVelocityScalingFactor(0.5);

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
        "/bob_arm/status", 10);
    bob_gripper_status_publisher_ = node_->create_publisher<std_msgs::msg::String>(
        "/bob_gripper/status", 10);

    // Initialize GUID Monitor
    guid_monitor_ = std::make_unique<GuidMonitor>(node_, this);
    RCLCPP_INFO(LOGGER, "GUID Monitor integrated successfully");
}

void PickPlaceNode::executePickAndPlaceTask()
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

    // 2. Open bob_gripper
    RCLCPP_INFO(LOGGER, "Step 2: Opening bob_gripper");
    if (!openbob_gripper())
    {
        RCLCPP_ERROR(LOGGER, "Failed to open bob_gripper. Aborting task.");
        return;
    }

    // 3. Move to Grasp Position (30 cm down)
    RCLCPP_INFO(LOGGER, "Step 3: Moving to Grasp Position (30 cm down)");
    if (!moveDownToGraspPosition())
    {
        RCLCPP_ERROR(LOGGER, "Failed to move to grasp position. Aborting task.");
        return;
    }

    // 4. Close bob_gripper to 50%
    RCLCPP_INFO(LOGGER, "Step 4: Closing bob_gripper to half");
    if (!closebob_gripperPartially())
    {
        RCLCPP_ERROR(LOGGER, "Failed to close bob_gripper. Aborting task.");
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

    // 8. Open bob_gripper FULLY to release object
    RCLCPP_INFO(LOGGER, "Step 8: Opening bob_gripper Fully to Release Object");
    if (!openbob_gripperFully())
    {
        RCLCPP_ERROR(LOGGER, "Failed to open bob_gripper fully. Aborting task.");
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

bool PickPlaceNode::movingToDropLocation(geometry_msgs::msg::Pose& target_pose1)
{
    publishArmStatus("Planning motion to drop location (from GUID)");

    // Get current pose
    geometry_msgs::msg::Pose current_pose = move_group_arm_->getCurrentPose().pose;

    // Create waypoints for a smoother path (intermediate points)
    std::vector<geometry_msgs::msg::Pose> waypoints;
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

        intermediate_pose.orientation = target_pose1.orientation;
        waypoints.push_back(intermediate_pose);
    }

    waypoints.push_back(target_pose1);

    // Try Cartesian path planning for smoother motion
    publishArmStatus("Computing Cartesian path to drop location");

    moveit_msgs::msg::RobotTrajectory trajectory;
    const double eef_step = 0.05;
    const double jump_threshold = 0.0;

    double fraction = move_group_arm_->computeCartesianPath(
        waypoints, eef_step, jump_threshold, trajectory);

    if (fraction >= 0.9)
    {
        publishArmStatus("Cartesian path computed successfully (" +
                         std::to_string(fraction * 100.0) + "% achieved)");

        // Scale down the velocity and acceleration
        robot_trajectory::RobotTrajectory rt(move_group_arm_->getRobotModel(), arm_group_name_);
        rt.setRobotTrajectoryMsg(*move_group_arm_->getCurrentState(), trajectory);

        trajectory_processing::IterativeParabolicTimeParameterization iptp;
        iptp.computeTimeStamps(rt, 0.3, 0.3);
        rt.getRobotTrajectoryMsg(trajectory);

        publishArmStatus("Executing smooth trajectory to drop location");
        bool execution_success = (move_group_arm_->execute(trajectory) == 
                                 moveit::core::MoveItErrorCode::SUCCESS);

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
        // Cartesian planning failed, fall back to joint-space planning
        publishArmStatus("Cartesian planning failed (" +
                         std::to_string(fraction * 100.0) + "% achieved), using joint-space planning");

        move_group_arm_->setPoseTarget(target_pose1);
        move_group_arm_->setMaxVelocityScalingFactor(0.3);
        move_group_arm_->setMaxAccelerationScalingFactor(0.3);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_arm_->plan(plan) == 
                       moveit::core::MoveItErrorCode::SUCCESS);

        if (!success)
        {
            publishArmStatus("Failed to plan motion to drop location");
            move_group_arm_->setMaxVelocityScalingFactor(0.5);
            move_group_arm_->setMaxAccelerationScalingFactor(0.5);
            return false;
        }

        publishArmStatus("Executing trajectory to drop location");
        bool execution_success = (move_group_arm_->execute(plan) == 
                                 moveit::core::MoveItErrorCode::SUCCESS);

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

bool PickPlaceNode::executeTrajectoryWithFeedback(const moveit_msgs::msg::RobotTrajectory &trajectory)
{
    if (trajectory.joint_trajectory.points.empty() && trajectory.multi_dof_joint_trajectory.points.empty())
    {
        RCLCPP_ERROR(LOGGER, "Trajectory is empty, cannot execute");
        return false;
    }

    motion_completed_ = false;
    motion_success_ = false;

    auto goal_msg = moveit_msgs::action::ExecuteTrajectory::Goal();
    goal_msg.trajectory = trajectory;

    auto send_goal_options = rclcpp_action::Client<moveit_msgs::action::ExecuteTrajectory>::SendGoalOptions();

    send_goal_options.feedback_callback =
        [this](rclcpp_action::ClientGoalHandle<moveit_msgs::action::ExecuteTrajectory>::SharedPtr,
               const std::shared_ptr<const moveit_msgs::action::ExecuteTrajectory::Feedback> feedback)
    {
        RCLCPP_INFO(LOGGER, "Received feedback: %s", feedback->state.c_str());
    };

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
        auto goal_handle_future = trajectory_action_client_->async_send_goal(goal_msg, send_goal_options);

        if (rclcpp::spin_until_future_complete(node_, goal_handle_future) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(LOGGER, "Failed to send goal");
            return false;
        }

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

void PickPlaceNode::publishArmStatus(const std::string &status)
{
    auto msg = std_msgs::msg::String();
    msg.data = "[BOB arm] " + status;
    arm_status_publisher_->publish(msg);
    RCLCPP_INFO(LOGGER, "%s", msg.data.c_str());
}

void PickPlaceNode::publishbob_gripperStatus(const std::string &status)
{
    auto msg = std_msgs::msg::String();
    msg.data = "[BOB gripper] " + status;
    bob_gripper_status_publisher_->publish(msg);
    RCLCPP_INFO(LOGGER, "%s", msg.data.c_str());
}

bool PickPlaceNode::moveToPregraspPosition()
{
    // Define pregrasp pose
    geometry_msgs::msg::Pose target_pose;
    target_pose.orientation.x = -0.063;
    target_pose.orientation.y = 0.998;
    target_pose.orientation.z = -0.028;
    target_pose.orientation.w = 0.000;
    target_pose.position.x = -1.030;
    target_pose.position.y = 0.135;
    target_pose.position.z = 0.545;

    move_group_arm_->setPoseTarget(target_pose);
    publishArmStatus("Planning motion to pregrasp position");

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

bool PickPlaceNode::openbob_gripper()
{
    publishbob_gripperStatus("Planning bob_gripper opening");
    move_group_bob_gripper_->setJointValueTarget("bob_robotiq_85_left_knuckle_joint", 0.0);

    moveit::planning_interface::MoveGroupInterface::Plan bob_gripper_plan;
    bool success = (move_group_bob_gripper_->plan(bob_gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (!success)
    {
        publishbob_gripperStatus("Failed to plan bob_gripper opening");
        return false;
    }

    publishbob_gripperStatus("Executing bob_gripper opening");
    bool execution_success = (move_group_bob_gripper_->execute(bob_gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (execution_success)
    {
        publishbob_gripperStatus("bob_gripper opened successfully");
    }
    else
    {
        publishbob_gripperStatus("bob_gripper opening failed");
    }

    return execution_success;
}

bool PickPlaceNode::moveDownToGraspPosition()
{
    publishArmStatus("Moving down to grasp position");
    geometry_msgs::msg::Pose start_pose = move_group_arm_->getCurrentPose().pose;

    std::vector<geometry_msgs::msg::Pose> waypoints;
    const int num_segments = 10;
    const double step_size = 0.3 / num_segments;

    for (int i = 1; i <= num_segments; i++)
    {
        geometry_msgs::msg::Pose intermediate_pose = start_pose;
        intermediate_pose.position.z -= (step_size * i);
        waypoints.push_back(intermediate_pose);
    }

    moveit_msgs::msg::RobotTrajectory trajectory;
    const double eef_step = 0.01;
    const double jump_threshold = 0.0;

    double fraction = move_group_arm_->computeCartesianPath(
        waypoints, eef_step, jump_threshold, trajectory);

    if (fraction < 0.9)
    {
        RCLCPP_ERROR(LOGGER, "Cartesian path planning failed (%.2f%% achieved)", fraction * 100.0);
        publishArmStatus("Cartesian path planning failed");
        publishArmStatus("Attempting joint-space planning instead");

        geometry_msgs::msg::Pose target_pose = start_pose;
        target_pose.position.z -= 0.3;
        move_group_arm_->setPoseTarget(target_pose);
        move_group_arm_->setMaxVelocityScalingFactor(0.3);
        
        moveit::planning_interface::MoveGroupInterface::Plan joint_plan;
        bool success = (move_group_arm_->plan(joint_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        
        if (!success)
        {
            publishArmStatus("Joint-space planning also failed");
            move_group_arm_->setMaxVelocityScalingFactor(0.5);
            return false;
        }

        bool execution_success = (move_group_arm_->execute(joint_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        move_group_arm_->setMaxVelocityScalingFactor(0.5);

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

    if (trajectory.joint_trajectory.points.empty() && trajectory.multi_dof_joint_trajectory.points.empty())
    {
        RCLCPP_ERROR(LOGGER, "Computed trajectory is empty");
        publishArmStatus("Computed trajectory is empty");
        return false;
    }

    robot_trajectory::RobotTrajectory rt(move_group_arm_->getRobotModel(), arm_group_name_);
    rt.setRobotTrajectoryMsg(*move_group_arm_->getCurrentState(), trajectory);

    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    iptp.computeTimeStamps(rt, 0.3, 0.3);
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

bool PickPlaceNode::closebob_gripperPartially()
{
    publishbob_gripperStatus("Planning bob_gripper closing");
    double half_closed_value = 0.4551;
    move_group_bob_gripper_->setJointValueTarget("bob_robotiq_85_left_knuckle_joint", half_closed_value);

    moveit::planning_interface::MoveGroupInterface::Plan bob_gripper_plan;
    bool success = (move_group_bob_gripper_->plan(bob_gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (!success)
    {
        publishbob_gripperStatus("Failed to plan bob_gripper closing");
        return false;
    }

    publishbob_gripperStatus("Executing bob_gripper closing");
    bool execution_success = (move_group_bob_gripper_->execute(bob_gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (execution_success)
    {
        publishbob_gripperStatus("bob_gripper closed successfully");
    }
    else
    {
        publishbob_gripperStatus("bob_gripper closing failed");
    }

    return execution_success;
}

bool PickPlaceNode::liftToPregraspPosition()
{
    publishArmStatus("Planning motion back to pregrasp position");
    geometry_msgs::msg::Pose start_pose = move_group_arm_->getCurrentPose().pose;

    std::vector<geometry_msgs::msg::Pose> waypoints;
    const int num_segments = 10;
    const double step_size = 0.3 / num_segments;

    for (int i = 1; i <= num_segments; i++)
    {
        geometry_msgs::msg::Pose intermediate_pose = start_pose;
        intermediate_pose.position.z += (step_size * i);
        waypoints.push_back(intermediate_pose);
    }

    moveit_msgs::msg::RobotTrajectory trajectory;
    const double eef_step = 0.01;
    const double jump_threshold = 0.0;

    double fraction = move_group_arm_->computeCartesianPath(
        waypoints, eef_step, jump_threshold, trajectory);

    if (fraction < 0.9)
    {
        RCLCPP_ERROR(LOGGER, "Cartesian path planning failed (%.2f%% achieved)", fraction * 100.0);
        publishArmStatus("Cartesian path planning failed");
        publishArmStatus("Attempting joint-space planning instead");

        geometry_msgs::msg::Pose target_pose = start_pose;
        target_pose.position.z += 0.3;
        move_group_arm_->setPoseTarget(target_pose);
        move_group_arm_->setMaxVelocityScalingFactor(0.3);
        
        moveit::planning_interface::MoveGroupInterface::Plan joint_plan;
        bool success = (move_group_arm_->plan(joint_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        
        if (!success)
        {
            publishArmStatus("Joint-space planning also failed");
            move_group_arm_->setMaxVelocityScalingFactor(0.5);
            return false;
        }

        bool execution_success = (move_group_arm_->execute(joint_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        move_group_arm_->setMaxVelocityScalingFactor(0.5);

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

    if (trajectory.joint_trajectory.points.empty() && trajectory.multi_dof_joint_trajectory.points.empty())
    {
        RCLCPP_ERROR(LOGGER, "Computed trajectory is empty");
        publishArmStatus("Computed trajectory is empty");
        return false;
    }

    robot_trajectory::RobotTrajectory rt(move_group_arm_->getRobotModel(), arm_group_name_);
    rt.setRobotTrajectoryMsg(*move_group_arm_->getCurrentState(), trajectory);

    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    iptp.computeTimeStamps(rt, 0.3, 0.3);
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

bool PickPlaceNode::movingToDropLocation()
{
    publishArmStatus("Planning motion to drop location");

    geometry_msgs::msg::Pose target_pose1;
    target_pose1.orientation.x = 0.509;
    target_pose1.orientation.y = -0.491;
    target_pose1.orientation.z = -0.494;
    target_pose1.orientation.w = 0.506;
    target_pose1.position.x = -0.603;
    target_pose1.position.y = -0.139;
    target_pose1.position.z = 0.650;

    return movingToDropLocation(target_pose1);
}

bool PickPlaceNode::moveDownToDropPosition()
{
    publishArmStatus("Moving down to drop position");
    geometry_msgs::msg::Pose start_pose = move_group_arm_->getCurrentPose().pose;

    std::vector<geometry_msgs::msg::Pose> waypoints;
    const int num_segments = 10;
    const double step_size = 0.3 / num_segments;

    for (int i = 1; i <= num_segments; i++)
    {
        geometry_msgs::msg::Pose intermediate_pose = start_pose;
        intermediate_pose.position.z -= (step_size * i);
        waypoints.push_back(intermediate_pose);
    }

    moveit_msgs::msg::RobotTrajectory trajectory;
    const double eef_step = 0.01;
    const double jump_threshold = 0.0;

    double fraction = move_group_arm_->computeCartesianPath(
        waypoints, eef_step, jump_threshold, trajectory);

    if (fraction < 0.9)
    {
        RCLCPP_ERROR(LOGGER, "Cartesian path planning failed (%.2f%% achieved)", fraction * 100.0);
        publishArmStatus("Cartesian path planning failed");
        publishArmStatus("Attempting joint-space planning instead");

        geometry_msgs::msg::Pose target_pose = start_pose;
        target_pose.position.z -= 0.3;
        move_group_arm_->setPoseTarget(target_pose);
        move_group_arm_->setMaxVelocityScalingFactor(0.3);
        
        moveit::planning_interface::MoveGroupInterface::Plan joint_plan;
        bool success = (move_group_arm_->plan(joint_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        
        if (!success)
        {
            publishArmStatus("Joint-space planning also failed");
            move_group_arm_->setMaxVelocityScalingFactor(0.5);
            return false;
        }

        bool execution_success = (move_group_arm_->execute(joint_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        move_group_arm_->setMaxVelocityScalingFactor(0.5);

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

    if (trajectory.joint_trajectory.points.empty() && trajectory.multi_dof_joint_trajectory.points.empty())
    {
        RCLCPP_ERROR(LOGGER, "Computed trajectory is empty");
        publishArmStatus("Computed trajectory is empty");
        return false;
    }

    robot_trajectory::RobotTrajectory rt(move_group_arm_->getRobotModel(), arm_group_name_);
    rt.setRobotTrajectoryMsg(*move_group_arm_->getCurrentState(), trajectory);

    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    iptp.computeTimeStamps(rt, 0.3, 0.3);
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

bool PickPlaceNode::openbob_gripperFully()
{
    publishbob_gripperStatus("Planning bob_gripper opening to fully release object");
    move_group_bob_gripper_->setJointValueTarget("bob_robotiq_85_left_knuckle_joint", 0.0);

    moveit::planning_interface::MoveGroupInterface::Plan bob_gripper_plan;
    bool success = (move_group_bob_gripper_->plan(bob_gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (!success)
    {
        publishbob_gripperStatus("Failed to plan full bob_gripper opening for object release");
        return false;
    }

    publishbob_gripperStatus("Executing full bob_gripper opening to release object");
    bool execution_success = (move_group_bob_gripper_->execute(bob_gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (execution_success)
    {
        publishbob_gripperStatus("bob_gripper fully opened successfully - object released");
    }
    else
    {
        publishbob_gripperStatus("Failed to fully open bob_gripper for object release");
    }

    return execution_success;
}

bool PickPlaceNode::moveUpFromDropPosition()
{
    publishArmStatus("Moving up from drop position");
    geometry_msgs::msg::Pose start_pose = move_group_arm_->getCurrentPose().pose;

    std::vector<geometry_msgs::msg::Pose> waypoints;
    const int num_segments = 10;
    const double step_size = 0.3 / num_segments;

    for (int i = 1; i <= num_segments; i++)
    {
        geometry_msgs::msg::Pose intermediate_pose = start_pose;
        intermediate_pose.position.z += (step_size * i);
        waypoints.push_back(intermediate_pose);
    }

    moveit_msgs::msg::RobotTrajectory trajectory;
    const double eef_step = 0.01;
    const double jump_threshold = 0.0;

    double fraction = move_group_arm_->computeCartesianPath(
        waypoints, eef_step, jump_threshold, trajectory);

    if (fraction < 0.9)
    {
        RCLCPP_ERROR(LOGGER, "Cartesian path planning failed (%.2f%% achieved)", fraction * 100.0);
        publishArmStatus("Cartesian path planning failed");
        publishArmStatus("Attempting joint-space planning instead");

        geometry_msgs::msg::Pose target_pose = start_pose;
        target_pose.position.z += 0.3;
        move_group_arm_->setPoseTarget(target_pose);
        move_group_arm_->setMaxVelocityScalingFactor(0.3);
        
        moveit::planning_interface::MoveGroupInterface::Plan joint_plan;
        bool success = (move_group_arm_->plan(joint_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        
        if (!success)
        {
            publishArmStatus("Joint-space planning also failed");
            move_group_arm_->setMaxVelocityScalingFactor(0.5);
            return false;
        }

        bool execution_success = (move_group_arm_->execute(joint_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        move_group_arm_->setMaxVelocityScalingFactor(0.5);

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

    if (trajectory.joint_trajectory.points.empty() && trajectory.multi_dof_joint_trajectory.points.empty())
    {
        RCLCPP_ERROR(LOGGER, "Computed trajectory is empty");
        publishArmStatus("Computed trajectory is empty");
        return false;
    }

    robot_trajectory::RobotTrajectory rt(move_group_arm_->getRobotModel(), move_group_arm_->getRobotModel()->getModelFrame());
    rt.setRobotTrajectoryMsg(*move_group_arm_->getCurrentState(), trajectory);    

    trajectory_processing::IterativeParabolicTimeParameterization iptp;        
    iptp.computeTimeStamps(rt);                                  
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