#include "StateEngine/Instruction/MovementInstruction.hpp"

#include <vector>            // std::vector
#include <rclcpp/rclcpp.hpp> // RCLCPP_INFO, RCLCPP_ERROR

#include "MachineStateControlNode.hpp"

namespace instruction
{
    MovementInstruction::~MovementInstruction() {}

    MovementInstruction::MovementInstruction(
        moveit_msgs::msg::RobotTrajectory& trajectory,
        double goalJointTolerance,
        const std::string& description)
        : AbstractInstruction(LOGGER_NAME, description),
          trajectory(trajectory),
          goalJointTolerance(goalJointTolerance),
          executeTrajectoryDone(false)
    {}

    void MovementInstruction::execute()
    {
        auto action_client = MachineStateControlNode::getInstance()->getExecuteTrajectoryClient();

        // Create the goal message
        auto goal_msg = custom_msgs::action::ExecuteTrajectory::Goal();
        goal_msg.trajectory = trajectory.joint_trajectory;
        goal_msg.joint_tollerance = goalJointTolerance;

        // Create SendGoalOptions object
        auto options = rclcpp_action::Client<custom_msgs::action::ExecuteTrajectory>::SendGoalOptions();
        options.result_callback = std::bind(&MovementInstruction::resultCallback, this, std::placeholders::_1);

        RCLCPP_INFO(logger, "Sending trajectory with %zu points", goal_msg.trajectory.points.size());

        // Send goal asynchronously with the options
        action_client->async_send_goal(goal_msg, options);

        // Wait for the result.
        std::unique_lock<std::mutex> lock(executeTrajectoryMtx);
        executeTrajectoryCv.wait(lock, [this] { return executeTrajectoryDone; });
    }

    // NOLINTNEXTLINE (readability-const-return-type) // in fact, the returned value should never be modified
    const moveit_msgs::msg::RobotState MovementInstruction::getFinalJointValues() const
    {
        if (trajectory.joint_trajectory.points.empty())
        {
            throw std::logic_error("Final state of instruction requested but the trajectory is empty.");
        }

        // Initialize the joint values with the final positions from the trajectory
        const std::vector<double> joints = trajectory.joint_trajectory.points.back().positions;
        const std::vector<std::string>& jointNames = trajectory.joint_trajectory.joint_names;

        moveit_msgs::msg::RobotState finalJointState;
        finalJointState.joint_state.name = jointNames;
        finalJointState.joint_state.position = joints;

        return finalJointState;
    }

    moveit_msgs::msg::RobotTrajectory MovementInstruction::getTrajectory() const
    {
        return trajectory;
    }

    void MovementInstruction::resultCallback(
        const rclcpp_action::ClientGoalHandle<custom_msgs::action::ExecuteTrajectory>::WrappedResult& result)
    {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
        {
            RCLCPP_INFO(logger, "Trajectory executed successfully!");
            std::unique_lock<std::mutex> lock(executeTrajectoryMtx);
            executeTrajectoryDone = true;
            executeTrajectoryCv.notify_one();
        }
        else
        {
            RCLCPP_ERROR(logger, "Trajectory execution failed with code: %d", static_cast<int>(result.code));
            throw std::runtime_error("Trajectory execution failed");
        }
    }

} // namespace instruction