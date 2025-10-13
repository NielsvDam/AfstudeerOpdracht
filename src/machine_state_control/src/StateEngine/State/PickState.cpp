#include "StateEngine/State/PickState.hpp"

namespace state
{
    PickState::~PickState() {}

    PickState::PickState(const moveit_msgs::msg::RobotState& startJointValues) : AbstractCrateState(startJointValues)
    {}

    const std::string& PickState::name() const
    {
        static const std::string name = "PickState";
        return name;
    }
} // namespace state