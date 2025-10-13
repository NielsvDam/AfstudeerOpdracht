#include "StateEngine/State/PlaceState.hpp"

namespace state
{
    PlaceState::~PlaceState() {}

    PlaceState::PlaceState(const moveit_msgs::msg::RobotState& startJointValues) : AbstractState(startJointValues) {}

    const std::string& PlaceState::name() const
    {
        static const std::string name = "PlaceState";
        return name;
    }
} // namespace state