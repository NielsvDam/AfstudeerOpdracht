#ifndef KNOCKOVERSTATE_HPP
#define KNOCKOVERSTATE_HPP

#include "AbstractDisplaceState.hpp" // state::AbstractDisplaceState

/**
 * @brief The state namespace contains classes representing the different states of the robot.
 *
 * Each state contains a set of executable instructions.
 */
namespace state
{
    /**
     * @class KnockOverState
     * @brief A class representing the state where the robot will knock over a stack of objects.
     */
    class KnockOverState : public AbstractDisplaceState
    {
    public:
        virtual ~KnockOverState();
        /**
         * @brief Constructor for the KnockOverState class.
         *
         * @param startJointValues The initial joint values of the robot.
         *
         * @note More info in AbstractState::AbstractState().
         */
        explicit KnockOverState(const moveit_msgs::msg::RobotState& startJointValues);
        /**
         * @brief Get the name of the state (the class name).
         *
         * @return const std::string& The name of the state.
         */
        const std::string& name() const override;
    };
} // namespace state

#endif // KNOCKOVERSTATE_HPP
