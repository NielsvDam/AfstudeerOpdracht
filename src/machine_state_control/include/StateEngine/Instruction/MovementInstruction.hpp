#ifndef MOVEMENTINSTRUCTION_HPP
#define MOVEMENTINSTRUCTION_HPP

#include <mutex>                                // std::mutex
#include <condition_variable>                   // std::condition_variable
#include <rclcpp_action/rclcpp_action.hpp>      // rclcpp_action::ClientGoalHandle
#include <moveit_msgs/msg/robot_trajectory.hpp> // moveit_msgs::msg::RobotTrajectory
#include <moveit_msgs/msg/robot_state.hpp>      // moveit_msgs::msg::RobotState

#include <custom_msgs/action/execute_trajectory.hpp> // custom_msgs::action::ExecuteTrajectory
#include "AbstractInstruction.hpp"                   // instruction::AbstractInstruction

/**
 * @namespace instruction
 * @brief Contains classes of executable instructions.
 *
 * The instruction namespace contains classes which can be executed. It includes the AbstractInstruction class, which serves
 * as a base class for executable instructions. Each inheriting class is responsible for implementing the execute() function,
 * so only the base class has to be called by the executor.
 */
namespace instruction
{
    /**
     * @class MovementInstruction
     * @brief Represents an instruction for robot movement.
     */
    class MovementInstruction : public AbstractInstruction
    {
    public:
        virtual ~MovementInstruction();
        /**
         * @brief Constructs a new MovementInstruction object.
         *
         * @param trajectory A planned trajectory.
         * @param goalJointTolerance The tolerance to consider the goal joint values being reached.
         * @param description A string describing the instruction. Defaults to "unnamed movement instruction".
         */
        explicit MovementInstruction(
            moveit_msgs::msg::RobotTrajectory& trajectory,
            double goalJointTolerance,
            const std::string& description = "unnamed movement instruction");
        /**
         * @brief Executes the movement instruction.
         *
         * This method overrides the execute function from the AbstractInstruction class.
         */
        void execute() override;
        /**
         * @brief Get the joint values in which the robot would be after excecution.
         *
         * @return moveit_msgs::msg::RobotState The final joint values.
         */
        const moveit_msgs::msg::RobotState getFinalJointValues() const;
        /**
         * @brief Get the planned trajectory.
         *
         * @return moveit_msgs::msg::RobotTrajectory The planned trajectory.
         */
        moveit_msgs::msg::RobotTrajectory getTrajectory() const;
    private:
        /**
         * @brief Callback function to handle the result of the called ExecuteTrajectory action in the execute() method.
         *
         * @param result The result of the action.
         */
        void resultCallback(
            const rclcpp_action::ClientGoalHandle<custom_msgs::action::ExecuteTrajectory>::WrappedResult& result);

        moveit_msgs::msg::RobotTrajectory trajectory; /* The planned trajectory. */
        double goalJointTolerance;                    /* The tolerance to consider the goal joint values being reached.*/
        bool executeTrajectoryDone;                   /* A boolean indicating if the trajectory execution is done. */
        std::mutex executeTrajectoryMtx;              /* A mutex to protect the executeTrajectoryDone variable. */
        std::condition_variable executeTrajectoryCv;  /* A condition variable to wait for the trajectory execution to
                                                         finish. */
        inline static const std::string LOGGER_NAME = "MovementInstruction"; /* The name of the logger. */

        // const std::string& description; /* Private replication of the description, this way it can be easily accessed from within the movementinstruction itself. */
    };
} // namespace instruction

#endif // MOVEMENTINSTRUCTION_HPP
