#ifndef GRIPPERINSTRUCTION_HPP
#define GRIPPERINSTRUCTION_HPP

#include "AbstractInstruction.hpp" // instruction::AbstractInstruction

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
     * @class GripperInstruction
     * @brief Represents an instruction to control the gripper.
     */
    class GripperInstruction : public AbstractInstruction
    {
    public:
        virtual ~GripperInstruction();
        /**
         * @brief Constructs a new GripperInstruction object.
         *
         * @param open A boolean indicating whether the gripper should open (true) or close (false).
         * @param description A string describing the instruction. Defaults to "unnamed gripper instruction".
         */
        explicit GripperInstruction(bool open, const std::string& description = "unnamed gripper instruction");
        /**
         * @brief Executes the gripper instruction.
         *
         * This method overrides the execute function from the AbstractInstruction class.
         */
        void execute() override;
    private:
        bool open; /* A boolean indicating whether the gripper should open (true) or close (false). */
        inline static const std::string LOGGER_NAME = "GripperInstruction"; /* The name of the logger. */
    };
} // namespace instruction

#endif // GRIPPERINSTRUCTION_HPP
