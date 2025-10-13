#include "StateEngine/State/AbstractCrateState.hpp"

namespace state
{
    AbstractCrateState::~AbstractCrateState() {}

    AbstractCrateState::AbstractCrateState(const moveit_msgs::msg::RobotState& startJointValues)
        : AbstractState(startJointValues)
    {}

    const std::string& AbstractCrateState::name() const
    {
        static const std::string name = "AbstractCrateState";
        return name;
    }
} // namespace state