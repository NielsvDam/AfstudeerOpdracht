#include "TrajectoryExecutorNode.hpp"
#include <algorithm>
#include <thread>
#include "JointVectorUtils.hpp"

TrajectoryExecutorNode::~TrajectoryExecutorNode() {}

TrajectoryExecutorNode::TrajectoryExecutorNode()
    : Node("execute_trajectory_node", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)),
      // Create the action server for executing trajectories
      executeTrajectoryAction(rclcpp_action::create_server<custom_msgs::action::ExecuteTrajectory>(
          this,
          get_parameter("action_name").as_string(),
          std::bind(&TrajectoryExecutorNode::handleActionGoal, this, std::placeholders::_1, std::placeholders::_2),
          std::bind(&TrajectoryExecutorNode::handleActionCancel, this, std::placeholders::_1),
          std::bind(&TrajectoryExecutorNode::handleActionAccepted, this, std::placeholders::_1))),
      // Create a subscription on the joint states (positions), to monitor the robot's state

      // The action/state that the standalone trajectory executor listens to is called ""

      jointStatesSubscription(this->create_subscription<sensor_msgs::msg::JointState>(
          "/joint_states",
          10,
          std::bind(&TrajectoryExecutorNode::jointStateCallback, this, std::placeholders::_1))),
      // Create the publisher of the trajectory
      trajectoryToControllerPublisher(
          this->create_publisher<trajectory_msgs::msg::JointTrajectory>(get_parameter("controller").as_string(), 10)) // <<! The current controller might be wrong in exec params, check this. >>
{
    RCLCPP_INFO(get_logger(), "Stand-alone trajectory executor initialized.");
}

rclcpp_action::GoalResponse TrajectoryExecutorNode::handleActionGoal(
    const rclcpp_action::GoalUUID& uuid,
    // NOLINTNEXTLINE (performance-unnecessary-value-param) // can't change definition in ros sourcecode
    const std::shared_ptr<const custom_msgs::action::ExecuteTrajectory::Goal> goal)
{
    (void)uuid; // suppress unused parameter warning
    RCLCPP_INFO(this->get_logger(), "Received new goal");
    // Get the trajectory and tolerance from the goal
    trajectory_msgs::msg::JointTrajectory trajectory = goal->trajectory;
    double epsilon = goal->joint_tollerance;
    // Check if trajectory is not empty
    if (trajectory.points.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "Trajectory is empty. Rejecting goal.");
        return rclcpp_action::GoalResponse::REJECT;
    }
    // Check if tolerance is set
    if (epsilon <= 0.0)
    {
        RCLCPP_ERROR(this->get_logger(), "Tolerance must be greater than zero. Rejecting goal.");
        return rclcpp_action::GoalResponse::REJECT;
    }
    // Check if start state matches current state
    std::unique_lock<std::mutex> lock(currentStateMutex);
    // Create vectors for the start and current joint states
    std::vector<double> startPositionVector = trajectory.points.front().positions;
    std::vector<double> currentPositionVector = currentJointState.position;
    std::vector<std::string> startNameVector = trajectory.joint_names;
    std::vector<std::string> currentNameVector = currentJointState.name;
    // Order the joint names and positions, so that they are in the same order
    JointVectorUtils::order(startNameVector, startPositionVector);
    JointVectorUtils::order(currentNameVector, currentPositionVector);
    // Check if the start state matches the current state
    if (!JointVectorUtils::equals(startPositionVector, currentPositionVector, epsilon))
    {
        RCLCPP_ERROR(this->get_logger(), "Start state does not match current state. Rejecting goal.");
        return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TrajectoryExecutorNode::handleActionCancel(
    // NOLINTNEXTLINE (performance-unnecessary-value-param) // can't change definition in ros sourcecode
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<custom_msgs::action::ExecuteTrajectory>> goal_handle)
{
    (void)goal_handle; // suppress unused parameter warning
    RCLCPP_INFO(this->get_logger(), "Cancel requested is not yet implemented. Rejecting request.");
    // TODO: Implement cancel logic, however there currently is no demand for this.
    return rclcpp_action::CancelResponse::REJECT;
}

void TrajectoryExecutorNode::handleActionAccepted(
    // NOLINTNEXTLINE (performance-unnecessary-value-param) // can't change definition in ros sourcecode
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<custom_msgs::action::ExecuteTrajectory>> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Goal accepted.");
    // Execute the goal in a separate thread
    std::thread{std::bind(&TrajectoryExecutorNode::executeAction, this, std::placeholders::_1), goal_handle}.detach();
}

void TrajectoryExecutorNode::executeAction(
    // NOLINTNEXTLINE (performance-unnecessary-value-param) // can't change definition in ros sourcecode
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<custom_msgs::action::ExecuteTrajectory>> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Executing trajectory...");
    // Create result message
    custom_msgs::action::ExecuteTrajectory::Result::SharedPtr result =
        std::make_shared<custom_msgs::action::ExecuteTrajectory::Result>();
    // Get trajectory and tolerance
    trajectory_msgs::msg::JointTrajectory trajectory = goal_handle->get_goal()->trajectory;
    double epsilon = goal_handle->get_goal()->joint_tollerance;
    // Publish trajectory
    trajectoryToControllerPublisher->publish(trajectory);
    // Wait for final joint state to match
    std::unique_lock<std::mutex> lock(currentStateMutex);
    bool reached = false;
    // Wait for the trajectory to reach the target
    while (!reached)
    {
        // Wait for the joint state to be updated, to avoid unnecessary computation
        jointStateCv.wait(lock);
        // Create vectors for the target and current joint states
        std::vector<double> targetJointPositionVector = trajectory.points.back().positions;
        std::vector<double> currentJointPositionVector = currentJointState.position;
        std::vector<std::string> targetJointNameVector = trajectory.joint_names;
        std::vector<std::string> currentJointNameVector = currentJointState.name;
        // Order the joint names and positions, so that they are in the same order
        JointVectorUtils::order(targetJointNameVector, targetJointPositionVector);
        JointVectorUtils::order(currentJointNameVector, currentJointPositionVector);
        // Check if the target joint state is reached
        reached = JointVectorUtils::equals(targetJointPositionVector, currentJointPositionVector, epsilon);
    }
    // Publish result
    result->success = true;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Trajectory execution completed.");
}

// NOLINTNEXTLINE (performance-unnecessary-value-param) // can't change definition in ros sourcecode
void TrajectoryExecutorNode::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    // lock the current joint state
    std::lock_guard<std::mutex> lock(currentStateMutex);
    // update the current joint state
    currentJointState = *msg;
    // notify any waiting threads about the update
    jointStateCv.notify_all();
}