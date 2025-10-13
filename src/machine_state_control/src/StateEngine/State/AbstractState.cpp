#include "StateEngine/State/AbstractState.hpp"

#include "StateEngine/Instruction/MovementInstruction.hpp" // instruction::MovementInstruction

namespace state
{
    AbstractState::~AbstractState() {}

    AbstractState::AbstractState(const moveit_msgs::msg::RobotState& startJointValues)
        : lastJointValues(startJointValues), lastExecutedJointValues(startJointValues), allInstructionsAdded(false)
    {}

    void AbstractState::addInstruction(const std::shared_ptr<instruction::AbstractInstruction>& instruction, bool isFinal)
    {
        if (!instruction)
        {
            RCLCPP_ERROR(rclcpp::get_logger(name()), "Error: Cannot add a nullptr instruction to a state.");
            throw std::runtime_error("Attempted to add a nullptr instruction to a state");
        }

        std::unique_lock<std::mutex> lock(instructionsMutex);

        if (allInstructionsAdded)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger(name()),
                "Error: Cannot add instruction when a previous instruction was already marked as final.");
            std::string desc1 = instruction->getDescription();
            std::string desc2 = instructions.back()->getDescription();
            RCLCPP_ERROR(rclcpp::get_logger(name()), "Attempted to add instruction:");
            RCLCPP_ERROR(rclcpp::get_logger(name()), "  '%s'", desc1.c_str());
            RCLCPP_ERROR(rclcpp::get_logger(name()), " after:");
            RCLCPP_ERROR(rclcpp::get_logger(name()), "  '%s' (state-final).", desc2.c_str());

            throw std::runtime_error("Attempted to add instruction after final instruction");
        }
        instructions.push(instruction);

        // Update the current goal joint values if the instruction is a MovementInstruction
        std::shared_ptr<instruction::MovementInstruction> movementInstruction =
            std::dynamic_pointer_cast<instruction::MovementInstruction>(instruction);
        if (movementInstruction)
        {
            try
            {
                lastJointValues = movementInstruction->getFinalJointValues();
            }
            catch (const std::logic_error& e)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger(name()),
                    "Error: Exception 'logic_error' caught while getting goal joint values. currentGoalJointPositions "
                    "will not be updated. message = %s",
                    e.what());
            }
        }

        allInstructionsAdded = isFinal;
        instructionsCv.notify_one(); // Notify that one instruction was added
    }

    void AbstractState::abort()
    {
        std::unique_lock<std::mutex> lock(instructionsMutex);
        while (!instructions.empty())
        {
            instructions.pop();
        }
        // Set the lastJointValues to lastDispatchedJointValues, so getLastJointValues() returns the last executed.
        lastJointValues = lastExecutedJointValues;
        allInstructionsAdded = true;
        instructionsCv.notify_one(); // Notify that all instructions have been added
    }

    const moveit_msgs::msg::RobotState& AbstractState::getLastJointValues()
    {
        std::unique_lock<std::mutex> lock(instructionsMutex);
        return lastJointValues;
    }

    std::shared_ptr<instruction::AbstractInstruction> AbstractState::awaitNextInstruction()
    {
        std::unique_lock<std::mutex> lock(instructionsMutex);
        while (instructions.empty() && !allInstructionsAdded)
        {
            instructionsCv.wait(lock);
        }
        if (instructions.empty())
        {
            return nullptr; // Completed
        }
        std::shared_ptr<instruction::AbstractInstruction> instruction = std::move(instructions.front());
        instructions.pop();
        // Update the last dispatched joint values
        if (std::shared_ptr<instruction::MovementInstruction> movementInstruction =
                std::dynamic_pointer_cast<instruction::MovementInstruction>(instruction))
        {
            lastExecutedJointValues = movementInstruction->getFinalJointValues();
        }
        return instruction;
    }

    const std::string& AbstractState::name() const
    {
        static const std::string name = "AbstractState";
        return name;
    }

    std::string AbstractState::toString() const
    {
        // create a string with the name of the state and the number of instructions
        std::string result = name() + " with " + std::to_string(instructions.size()) + " remaining instructions:\n";
        // create a temporary queue to iterate over the instructions
        std::queue<std::shared_ptr<instruction::AbstractInstruction>> tempQueue = instructions;
        // iterate over the instructions and add the description to the result string
        while (!tempQueue.empty())
        {
            result += " - " + tempQueue.front()->getDescription() + "\n";
            tempQueue.pop();
        }
        return result;
    }
} // namespace state