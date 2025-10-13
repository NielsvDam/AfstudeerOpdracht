#include "StateEngine/State/ShuffleState.hpp"

namespace state
{
    ShuffleState::~ShuffleState() {}

    ShuffleState::ShuffleState(const moveit_msgs::msg::RobotState& startJointValues)
        : AbstractDisplaceState(startJointValues)
    {}

    const std::string& ShuffleState::name() const
    {
        static const std::string name = "ShuffleState";
        return name;
    }
} // namespace state