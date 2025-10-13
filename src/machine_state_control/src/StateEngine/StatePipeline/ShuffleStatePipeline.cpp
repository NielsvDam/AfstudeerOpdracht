#include "StateEngine/StatePipeline/ShuffleStatePipeline.hpp"

namespace state_pipeline
{
    ShuffleStatePipeline::~ShuffleStatePipeline() {}

    ShuffleStatePipeline::ShuffleStatePipeline(
        const moveit_msgs::msg::RobotState& startJointValues,
        const std::shared_ptr<moveit::planning_interface::MoveGroupInterface>& moveGroup)
        : AbstractStatePipeline(
              LOGGER_NAME,
              startJointValues,
              moveGroup,
              std::make_shared<state::ShuffleState>(startJointValues)),
          shuffleCenter(nullptr),
          shuffleRadius(0)
    {}

    void ShuffleStatePipeline::setShuffleCenter(geometry_msgs::msg::Pose pose)
    {
        shuffleCenter = std::make_shared<geometry_msgs::msg::Pose>(pose);
    }

    void ShuffleStatePipeline::setShuffleRadius(unsigned int radius)
    {
        shuffleRadius = radius;
    }

    std::shared_ptr<instruction::AbstractInstruction> ShuffleStatePipeline::createInstruction(uint8_t instructionNumber)
    {
        (void)instructionNumber; // suppress unused parameter warning
        return nullptr;
    }
} // namespace state_pipeline