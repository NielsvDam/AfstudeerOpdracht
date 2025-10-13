#ifndef PLACESTATE_HPP
#define PLACESTATE_HPP

#include "AbstractState.hpp" // state::AbstractState

/**
 * @brief The state namespace contains classes representing the different states of the robot.
 *
 * Each state contains a set of executable instructions.
 */
namespace state
{
    /**
     * @class PlaceState
     * @brief A class representing the state where the robot places an object.
     */
    class PlaceState : public AbstractState
    {
    public:
        virtual ~PlaceState();
        /**
         * @brief Constructor for the PlaceState class.
         *
         * @param startJointValues The initial joint values of the robot.
         *
         * @note More info in AbstractState::AbstractState().
         */
        explicit PlaceState(const moveit_msgs::msg::RobotState& startJointValues);
        /**
         * @brief Get the name of the state (the class name).
         *
         * @return const std::string& The name of the state.
         */
        const std::string& name() const override;
    };
} // namespace state

#endif // PLACESTATE_HPP
