#ifndef ABSTRACTSTATE_HPP
#define ABSTRACTSTATE_HPP

#include <string>                          // std::string
#include <memory>                          // std::shared_ptr
#include <queue>                           // std::queue
#include <mutex>                           // std::mutex
#include <condition_variable>              // std::condition_variable
#include <moveit_msgs/msg/robot_state.hpp> // moveit_msgs::msg::RobotState

#include "StateEngine/Instruction/AbstractInstruction.hpp" // instruction::AbstractInstruction

/**
 * @brief The state namespace contains classes representing the different states of the robot.
 *
 * Each state contains a set of executable instructions.
 */
namespace state
{
    /**
     * @class AbstractState
     * @brief An abstract base class representing a state of the robot arm.
     *
     * The AbstractState class provides administration about instructions of a state. It ensures
     * thread safety and provides a mechanisms for adding and retrieving.
     *
     * Once a instruction is added, it can be directly used due to the thread-safe mechanism.
     *
     * @note This class is intended to be subclassed and cannot be instantiated directly.
     */
    class AbstractState
    {
    public:
        virtual ~AbstractState();
        /**
         * @brief Adds an instruction to the state.
         *
         * This function adds a new instruction to the queue of instructions.
         *
         * @param instruction A shared pointer to the instruction to be added.
         * @param final A boolean indicating if the instruction is the final instruction of the state.
         *
         * @pre \p instruction must not be nullptr.
         * @pre a instruction marked state-final has not been added to the state already.
         *
         * @throws std::runtime_error if \p instruction is nullptr
         * @throws std::runtime_error if a instruction marked final has already been added to the state
         *
         * @note The function uses a mutex to ensure thread safety.
         */
        void addInstruction(const std::shared_ptr<instruction::AbstractInstruction>& instruction, bool isFinal = false);
        /**
         * @brief Get the (final) joint values of the last added MovementInstruction.
         *
         * @return const moveit_msgs::msg::RobotState& A constant reference to the current goal joint values.
         */
        const moveit_msgs::msg::RobotState& getLastJointValues();
        /**
         * @brief Waits for and retrieves the next instruction of the state.
         *
         * @note This function blocks until the a instruction is available in the queue, there may be instructions available
         * directly but this is not guaranteed.
         *
         * @return std::shared_ptr<AbstractInstruction> A shared pointer to the next instruction.
         */
        std::shared_ptr<instruction::AbstractInstruction> awaitNextInstruction();
        /**
         * @brief Aborts the state.
         *
         * This function clears the instruction queue and sets the current goal joint values to the last dispatched joint.
         */
        void abort();
        /**
         * @brief Get the name of the state (the class name).
         *
         * @return const std::string& The name of the state.
         */
        virtual const std::string& name() const;
        /**
         * @brief Converts the current state to a string representation.
         *
         * This function creates a string that includes the name of the state and the number of remaining instructions.
         * It then iterates over the instructions and appends each description, like a list of tasks
         *
         * @return std::string The string representation of the state.
         */
        std::string toString() const;
    protected:
        /**
         * @brief Protected constructor for the AbstractState class.
         *
         * This constructor initializes the AbstractState object with the given start joint values. It sets the
         * currentGoalJointPositions to the startJointValues, which get overwritten when adding a MovementInstruction.
         *
         * @param startJointValues The initial joint values of the robot.
         */
        explicit AbstractState(const moveit_msgs::msg::RobotState& startJointValues);
    private:
        moveit_msgs::msg::RobotState lastJointValues;         /* The goal joint values (of the last added
                                                                           MovementInstruction) */
        moveit_msgs::msg::RobotState lastExecutedJointValues; /* The last dispatched joint values */
        std::mutex instructionsMutex;                         /* Mutex for the instructions queue */
        std::condition_variable instructionsCv;               /* Condition variable for the instructions queue */
        std::queue<std::shared_ptr<instruction::AbstractInstruction>> instructions; /* The queue of instructions */
        bool allInstructionsAdded; /* Flag to indicate if all instructions have been added to the state */
    };
} // namespace state

#endif // ABSTRACTSTATE_HPP
