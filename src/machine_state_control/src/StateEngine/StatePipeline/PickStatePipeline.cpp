#include "StateEngine/StatePipeline/PickStatePipeline.hpp"

#include "StateEngine/Instruction/MovementInstruction.hpp" // instruction::MovementInstruction
#include "StateEngine/Instruction/GripperInstruction.hpp"  // instruction::GripperInstruction
#include "StateEngine/Instruction/SleepInstruction.hpp"    // instruction::SleepInstruction

namespace state_pipeline
{
    PickStatePipeline::~PickStatePipeline() {}

    PickStatePipeline::PickStatePipeline(
        const moveit_msgs::msg::RobotState& startJointValues,
        const std::shared_ptr<moveit::planning_interface::MoveGroupInterface>& moveGroup)
        : AbstractStatePipeline(
              LOGGER_NAME,
              startJointValues,
              moveGroup,
              std::make_shared<state::PickState>(startJointValues)),
          pickSolution(nullptr)
    {
        RCLCPP_INFO(logger, "PickStatePipeline created.");
    }

    void PickStatePipeline::setPickSolution(const std::shared_ptr<pick_solution_finder::PickSolution>& pickSolution)
    {
        this->pickSolution = pickSolution;
        const geometry_msgs::msg::Pose& pickPose = pickSolution->getPickPose();
        RCLCPP_INFO(
            logger,
            "Pick solution set to object at x: %f, y: %f, z: %f. rotation: %f, %f, %f, %f",
            pickPose.position.x,
            pickPose.position.y,
            pickPose.position.z,
            pickPose.orientation.x,
            pickPose.orientation.y,
            pickPose.orientation.z,
            pickPose.orientation.w);
    }

    std::shared_ptr<instruction::AbstractInstruction> PickStatePipeline::createInstruction(uint8_t instructionNumber)
    {
        if (!pickSolution)
        {
            RCLCPP_ERROR(logger, "Generation started but no pick-solution set");
            throw std::logic_error("no pick-solution set");
        }

        switch (instructionNumber)
        {
            case 0: // ensure gripper is open.
            {
                return std::make_shared<instruction::GripperInstruction>(false, "open gripper");
            }
            case 1: // move to aboveCratePose.
            {
                geometry_msgs::msg::Pose pose = pickSolution->getAboveCratePose();
                auto trajectory = createTrajectory(pose, "tool_tcp", "PTP"); // HARDCODED : TODO
                RCLCPP_INFO(logger,"Move to aboveCratePose.");
                return std::make_shared<instruction::MovementInstruction>(
                    trajectory,
                    getGoalJointTolerance(),
                    "move to aboveCratePose");
            }
            case 2: // move to retract pose.
            {
                geometry_msgs::msg::Pose pose = pickSolution->getRetractPose();
                auto trajectory = createTrajectory(pose, "tool_tcp", "PTP"); // HARDCODED : TODO
                RCLCPP_INFO(logger,"Move to retractpose.");
                return std::make_shared<instruction::MovementInstruction>(
                    trajectory,
                    getGoalJointTolerance(),
                    "move to RetractPose");
            }
            case 3: // move to pick pose.
            {
                geometry_msgs::msg::Pose pose = pickSolution->getPickPose();
                auto trajectory = createTrajectory(pose, "tool_tcp", "LIN", NEAR_OBJECT_VELOCITY_SCALING); // HARDCODED : TODO
                RCLCPP_INFO(logger,"Move to pickpose.");
                return std::make_shared<instruction::MovementInstruction>(
                    trajectory,
                    getGoalJointTolerance(),
                    "move to PickPose");
            }
            case 4: // close gripper.
            {
                return std::make_shared<instruction::GripperInstruction>(true, "close gripper");
            }
            case 5: // wait for gripper to close.
            {
                return std::make_shared<instruction::SleepInstruction>(125, "wait for gripper to close");
            }
            case 6: // move to retract pose.
            {
                geometry_msgs::msg::Pose pose = pickSolution->getRetractPose();
                auto trajectory = createTrajectory(pose, "tool_tcp", "LIN", NEAR_OBJECT_VELOCITY_SCALING); // HARDCODED : TODO
                markCompleted();
                RCLCPP_INFO(logger, "Move to retractionpose."); // DEBUG prints.
                return std::make_shared<instruction::MovementInstruction>(
                    trajectory,
                    getGoalJointTolerance(),
                    "move to PickPose");
            }
            default:
            {
                RCLCPP_ERROR(logger, "Unknown instruction number %d", instructionNumber);
                throw std::invalid_argument("Unknown instruction number");
            }
        }
        return nullptr;
    }
} // namespace state_pipeline