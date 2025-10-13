#include "StateEngine/State/PictureState.hpp"

namespace state
{
    PictureState::~PictureState() {}

    PictureState::PictureState(const moveit_msgs::msg::RobotState& startJointValues) : AbstractState(startJointValues)
    {}

    const std::string& PictureState::name() const
    {
        static const std::string name = "PictureState";
        return name;
    }
} // namespace state