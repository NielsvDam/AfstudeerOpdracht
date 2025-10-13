#include "StateEngine/State/KnockOverState.hpp"

namespace state
{
    KnockOverState::KnockOverState(const moveit_msgs::msg::RobotState& startJointValues)
        : AbstractDisplaceState(startJointValues)
    {}

    KnockOverState::~KnockOverState() {}

    const std::string& KnockOverState::name() const
    {
        static const std::string name = "KnockOverState";
        return name;
    }
} // namespace state