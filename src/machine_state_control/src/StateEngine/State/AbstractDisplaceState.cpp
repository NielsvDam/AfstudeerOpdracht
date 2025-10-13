#include "StateEngine/State/AbstractDisplaceState.hpp"

namespace state
{
    AbstractDisplaceState::~AbstractDisplaceState() {}

    AbstractDisplaceState::AbstractDisplaceState(const moveit_msgs::msg::RobotState& startJointValues)
        : AbstractState(startJointValues)
    {}

    const std::string& AbstractDisplaceState::name() const
    {
        static const std::string name = "AbstractDisplaceState";
        return name;
    }
} // namespace state