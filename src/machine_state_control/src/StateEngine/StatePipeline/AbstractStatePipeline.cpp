#include "StateEngine/StatePipeline/AbstractStatePipeline.hpp"

#include <chrono>                           // std::chrono::steady_clock
#include <moveit/robot_state/conversions.h> // moveit::core::robotStateMsgToRobotState
#include <moveit/robot_state/robot_state.h> // moveit::core::RobotState
#include <nav_msgs/msg/path.hpp>            // nav_msgs::msg::Path

#include "StateEngine/Instruction/MovementInstruction.hpp" // instruction::MovementInstruction
#include "MachineStateControlNode.hpp"

namespace state_pipeline
{
    AbstractStatePipeline::~AbstractStatePipeline() {}

    AbstractStatePipeline::AbstractStatePipeline(
        const std::string& loggerName,
        const moveit_msgs::msg::RobotState& startJointValues,
        const std::shared_ptr<moveit::planning_interface::MoveGroupInterface>& moveGroup,
        const std::shared_ptr<state::AbstractState>& state)
        : logger(rclcpp::get_logger(loggerName)),
          startJointValues(startJointValues),
          state(state),
          moveGroup(moveGroup),
          curInstructionNumber(0),
          completed(false)
    {}

    // NOLINTNEXTLINE (readability-const-return-type) // pointer should never be modified
    const std::shared_ptr<state::AbstractState> AbstractStatePipeline::getState()
    {
        return state;
    }

    // Note: overloaded function to work with Pose instead of PoseStamped
    moveit_msgs::msg::RobotTrajectory AbstractStatePipeline::createTrajectory(
        const geometry_msgs::msg::Pose& targetPose, // Pose
        const std::string& endEffectorLink,
        const std::string& preferredPilzPlannerId,
        double velocityScaling)
    {
        // Convert geometry_msgs::msg::Pose to geometry_msgs::msg::PoseStamped
        geometry_msgs::msg::PoseStamped targetPoseStamped;
        targetPoseStamped.pose = targetPose;
        targetPoseStamped.header.frame_id = "world";
        return createTrajectory(targetPoseStamped, endEffectorLink, preferredPilzPlannerId, velocityScaling);
    }

    moveit_msgs::msg::RobotTrajectory AbstractStatePipeline::createTrajectory(
        const geometry_msgs::msg::PoseStamped& targetPose, // PoseStamped
        const std::string& endEffectorLink,
        const std::string& preferredPilzPlannerId,
        double velocityScaling)
    {
        // Define the order of pilz planner IDs to try
        std::vector<std::string> pilzIds = {"LIN", "PTP", "CIRC"};

        // Check if preferredPilzPlannerId is correct
        if (std::find(pilzIds.begin(), pilzIds.end(), preferredPilzPlannerId) == pilzIds.end())
        {
            RCLCPP_ERROR(logger, "Preferred Pilz Planner ID %s is not valid", preferredPilzPlannerId.c_str());
            throw std::invalid_argument("Atempted to use an invalid Pilz Planner ID");
        }
        // Move the preferredPilzPlannerId to the front
        pilzIds.erase(std::remove(pilzIds.begin(), pilzIds.end(), preferredPilzPlannerId), pilzIds.end());
        pilzIds.insert(pilzIds.begin(), preferredPilzPlannerId);

        // set default velocity scaling
        if (velocityScaling < 0.0)
        {
            velocityScaling = MachineStateControlNode::getInstance()->get_parameter("velocity_scaling").as_double();
        }

        // Try each pilz planner ID until one succeeds. (pilz is very quick, so we try it first before heavy OMPL)
        for (const auto& plannerId : pilzIds)
        {
            auto trajectory =
                planTrajectory(targetPose, endEffectorLink, "pilz_industrial_motion_planner", plannerId, velocityScaling);
            if (trajectory)
            {
                return *trajectory;
            }
        }
        // When all pilz planners fail, use OMPL
        auto trajectory = planTrajectory(targetPose, endEffectorLink, "ompl", "RRTstarkConfigDefault", velocityScaling);
        if (trajectory)
        {
            RCLCPP_INFO(logger, "OMPL succeeded");
            return *trajectory;
        }
        RCLCPP_ERROR(logger, "OMPL failed. No trajectory found");
        throw std::runtime_error("Failed to create trajectory");
    }

    bool AbstractStatePipeline::createNextInstruction()
    {
        // Log start time for debugging
        auto start = std::chrono::steady_clock::now();
        auto startDuration = std::chrono::duration_cast<std::chrono::milliseconds>(start.time_since_epoch()).count();
        RCLCPP_INFO(logger, "Create instruction begin: %ld ms", startDuration);

        // call abstract pipeline function to create instruction
        std::shared_ptr<instruction::AbstractInstruction> instruction = createInstruction(curInstructionNumber);
        if (!instruction)
        {
            RCLCPP_ERROR(
                logger,
                "createInstruction() returned nullptr as an instruction. Is markCompleted() called? Did planning fail?");
            throw std::runtime_error("State pipeline returned nullptr as an instruction.");
        }

        // Log end time for debugging
        auto end = std::chrono::steady_clock::now();
        auto endDuration = std::chrono::duration_cast<std::chrono::milliseconds>(end.time_since_epoch()).count();
        RCLCPP_INFO(
            logger,
            "Create instruction end:   %ld ms. duration was %ld ms",
            endDuration,
            std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());

        // Publish trajectory for visualization
        if (auto movementInstruction = std::dynamic_pointer_cast<instruction::MovementInstruction>(instruction))
        {
            auto trajectory = movementInstruction->getTrajectory();
            publishTrajectoryPath(trajectory);
        }

        curInstructionNumber++;                        // increment instruction number
        state->addInstruction(instruction, completed); // add instruction to the state
        return completed;
    }

    geometry_msgs::msg::Pose AbstractStatePipeline::getStartPose(const std::string& endEffectorLink)
    {
        // Get the robot state object from the startJointValues (to get access to getGlobalLinkTransform())
        moveit::core::RobotState robotState(moveGroup->getRobotModel());
        moveit::core::robotStateMsgToRobotState(state->getLastJointValues(), robotState, true);

        // Get the transform
        auto transform = robotState.getGlobalLinkTransform(endEffectorLink);

        // Convert Eigen::Isometry3d to geometry_msgs::msg::Pose
        geometry_msgs::msg::Pose pose;
        pose.position.x = transform.translation().x();
        pose.position.y = transform.translation().y();
        pose.position.z = transform.translation().z();
        Eigen::Quaterniond quat(transform.rotation());
        pose.orientation.x = quat.x();
        pose.orientation.y = quat.y();
        pose.orientation.z = quat.z();
        pose.orientation.w = quat.w();
        return pose;
    }

    moveit_msgs::msg::RobotTrajectory::SharedPtr AbstractStatePipeline::planTrajectory(
        const geometry_msgs::msg::PoseStamped& targetPose,
        const std::string& endEffectorLink,
        const std::string& planningPipelineId,
        const std::string& plannerId,
        double velocityScaling)
    {
        moveGroup->clearPoseTargets();
        moveGroup->setStartState(state->getLastJointValues());
        moveGroup->setPoseTarget(targetPose, endEffectorLink);
        moveGroup->setPlanningPipelineId(planningPipelineId);
        moveGroup->setPlannerId(plannerId);
        moveGroup->setMaxVelocityScalingFactor(velocityScaling);
        moveGroup->setMaxAccelerationScalingFactor(
            MachineStateControlNode::getInstance()->get_parameter("acceleration_scaling").as_double());
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (moveGroup->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (success)
        {
            return std::make_shared<moveit_msgs::msg::RobotTrajectory>(plan.trajectory_);
        }
        else
        {
            RCLCPP_INFO(logger, "%s failed with planner %s", planningPipelineId.c_str(), plannerId.c_str());
        }
        return nullptr;
    }

    void AbstractStatePipeline::publishTrajectoryPath(moveit_msgs::msg::RobotTrajectory& trajectory)
    {
        // Create a new Path message
        nav_msgs::msg::Path path;
        path.header.frame_id = "world"; // Set the frame ID to the appropriate frame: world.
        path.header.stamp = rclcpp::Clock().now();

        for (const auto& point : trajectory.joint_trajectory.points)
        {
            // Create a RobotState object and set the joint positions
            moveit::core::RobotState robotState(moveGroup->getRobotModel());
            robotState.setVariablePositions(point.positions);
            robotState.update();

            // Get the TCP position
            auto tcpTransform = robotState.getGlobalLinkTransform("tool_tcp"); // HARDCODED : TODO | WAS "rv5as_default_tcp"
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "world";
            pose.header.stamp = rclcpp::Clock().now();
            pose.pose.position.x = tcpTransform.translation().x();
            pose.pose.position.y = tcpTransform.translation().y();
            pose.pose.position.z = tcpTransform.translation().z();
            Eigen::Quaterniond quat(tcpTransform.rotation());
            pose.pose.orientation.x = quat.x();
            pose.pose.orientation.y = quat.y();
            pose.pose.orientation.z = quat.z();
            pose.pose.orientation.w = quat.w();

            path.poses.push_back(pose);
        }

        auto pathPublisher = MachineStateControlNode::getInstance()->getPathPublisher();
        // Publish the Path message
        pathPublisher->publish(path);
    }

    double AbstractStatePipeline::getGoalJointTolerance()
    {
        return moveGroup->getGoalJointTolerance();
    }

    void AbstractStatePipeline::markCompleted()
    {
        completed = true;
    }
} // namespace state_pipeline