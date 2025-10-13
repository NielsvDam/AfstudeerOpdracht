#ifndef PICTURESTATE_HPP
#define PICTURESTATE_HPP

#include "AbstractState.hpp" // state::AbstractState

/**
 * @brief The state namespace contains classes representing the different states of the robot.
 *
 * Each state contains a set of executable instructions.
 */
namespace state
{
    /**
     * @class PictureState
     * @brief A class representing the state where the robot will move to a position to take a picture, and take it.
     */
    class PictureState : public AbstractState
    {
    public:
        virtual ~PictureState();
        /**
         * @brief Constructor for the PictureState class.
         *
         * @param startJointValues The initial joint values of the robot.
         *
         * @note More info in AbstractState::AbstractState().
         */
        explicit PictureState(const moveit_msgs::msg::RobotState& startJointValues);
        /**
         * @brief Get the name of the state (the class name).
         *
         * @return const std::string& The name of the state.
         */
        const std::string& name() const override;
    };
} // namespace state

#endif // PICTURESTATE_HPP
