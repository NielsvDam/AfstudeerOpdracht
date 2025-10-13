#ifndef ABSTRACTCRATESTATE_HPP
#define ABSTRACTCRATESTATE_HPP

#include "AbstractState.hpp" // state::AbstractState

/**
 * @brief The state namespace contains classes representing the different states of the robot.
 * 
 * Each state contains a set of executable instructions.
 */
namespace state
{

    /**
     * @class AbstractCrateState
     * @brief An abstract base class representing a state in which the robot interacts with the crate.
     *
     * @note This class is intended to be subclassed and cannot be instantiated directly.
     */
    class AbstractCrateState : public AbstractState
    {
    public:
        virtual ~AbstractCrateState();
        /**
         * @brief Get the name of the state (the class name).
         *
         * @return const std::string& The name of the state.
         */
        const std::string& name() const override;
    protected:
        /**
         * @brief Protected Constructor for the AbstractCrateState class.
         *
         * @param startJointValues The initial joint values of the robot.
         *
         * @note More info in AbstractState::AbstractState().
         */
        explicit AbstractCrateState(const moveit_msgs::msg::RobotState& startJointValues);
    };
} // namespace state

#endif // ABSTRACTCRATESTATE_HPP
