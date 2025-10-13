#ifndef PICKSTATE_HPP
#define PICKSTATE_HPP

#include "AbstractCrateState.hpp" // state::AbstractCrateState

/**
 * @brief The state namespace contains classes representing the different states of the robot.
 *
 * Each state contains a set of executable instructions.
 */
namespace state
{
    /**
     * @class PickState
     * @brief A class representing the state where the robot picks an object from the crate.
     */
    class PickState : public AbstractCrateState
    {
    public:
        virtual ~PickState();
        /**
         * @brief Constructor for the PickState class.
         *
         * @param startJointValues The initial joint values of the robot.
         *
         * @note More info in AbstractState::AbstractState().
         */
        explicit PickState(const moveit_msgs::msg::RobotState& startJointValues);
        /**
         * @brief Get the name of the state (the class name).
         *
         * @return const std::string& The name of the state.
         */
        const std::string& name() const override;
    };
} // namespace state

#endif // PICKSTATE_HPP
