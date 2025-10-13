#include "StateEngine/StatePipeline/KnockOverStatePipeline.hpp"

namespace state_pipeline
{
    KnockOverStatePipeline::~KnockOverStatePipeline() {}

    KnockOverStatePipeline::KnockOverStatePipeline(
        const moveit_msgs::msg::RobotState& startJointValues,
        const std::shared_ptr<moveit::planning_interface::MoveGroupInterface>& moveGroup)
        : AbstractStatePipeline(
              LOGGER_NAME,
              startJointValues,
              moveGroup,
              std::make_shared<state::KnockOverState>(startJointValues)),
          stackPose(nullptr)
    {}

    void KnockOverStatePipeline::setStackPose(geometry_msgs::msg::Pose pose)
    {
        stackPose = std::make_shared<geometry_msgs::msg::Pose>(pose);
    }

    void KnockOverStatePipeline::setDetectedObjects(const std::vector<geometry_msgs::msg::Pose>& objects)
    {
        detectedObjects = objects;
    }

    std::shared_ptr<instruction::AbstractInstruction> KnockOverStatePipeline::createInstruction(uint8_t instructionNumber)
    {
        (void)instructionNumber; // suppress unused parameter warning
        return nullptr;
    }
} // namespace state_pipeline