#ifndef SHUFFLESTATE_HPP
#define SHUFFLESTATE_HPP

#include "AbstractDisplaceState.hpp" // state::AbstractDisplaceState

/**
 * @brief The state namespace contains classes representing the different states of the robot.
 *
 * Each state contains a set of executable instructions.
 */
namespace state
{
    /**
     * @class ShuffleState
     * @brief A class representing the state where the robot will shuffle objects around in the crate.
     */
    class ShuffleState : public AbstractDisplaceState
    {
    public:
        virtual ~ShuffleState();
        /**
         * @brief Constructor for the ShuffleState class.
         *
         * @param startJointValues The initial joint values of the robot.
         *
         * @note More info in AbstractState::AbstractState().
         */
        explicit ShuffleState(const moveit_msgs::msg::RobotState& startJointValues);
        /**
         * @brief Get the name of the state (the class name).
         *
         * @return const std::string& The name of the state.
         */
        const std::string& name() const override;
    };
} // namespace state

#endif // SHUFFLESTATE_HPP
