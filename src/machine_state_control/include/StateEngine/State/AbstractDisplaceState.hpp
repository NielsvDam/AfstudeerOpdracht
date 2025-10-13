#ifndef ABSTRACTDISPLACESTATE_HPP
#define ABSTRACTDISPLACESTATE_HPP

#include "AbstractState.hpp" // state::AbstractState
/**
 * @brief The state namespace contains classes representing the different states of the robot.
 *
 * Each state contains a set of executable instructions.
 */
namespace state
{
    /**
     * @class AbstractDisplaceState
     * @brief An abstract base class representing a state in which the robot displaces objects.
     *
     * @note This class is intended to be subclassed and cannot be instantiated directly.
     */
    class AbstractDisplaceState : public AbstractState
    {
    public:
        virtual ~AbstractDisplaceState();
        /**
         * @brief Get the name of the state (the class name).
         *
         * @return const std::string& The name of the state.
         */
        const std::string& name() const override;
    protected:
        /**
         * @brief Protected Constructor for the AbstractDisplaceState class.
         *
         * @param startJointValues The initial joint values of the robot.
         *
         * @note More info in AbstractState::AbstractState().
         */
        explicit AbstractDisplaceState(const moveit_msgs::msg::RobotState& startJointValues);
    };
} // namespace state

#endif // ABSTRACTDISPLACESTATE_HPP
