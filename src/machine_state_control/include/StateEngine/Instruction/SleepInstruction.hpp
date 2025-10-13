#ifndef SLEEPINSTRUCTION_HPP
#define SLEEPINSTRUCTION_HPP

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
     * @class SleepInstruction
     * @brief Represents an instruction to sleep for a certain period.
     */
    class SleepInstruction : public AbstractInstruction
    {
    public:
        virtual ~SleepInstruction();
        /**
         * @brief Constructs a new SleepInstruction object.
         *
         * @param duration The duration in milliseconds to sleep.
         * @param description A string describing the instruction. Defaults to "unnamed sleep instruction".
         */
        explicit SleepInstruction(unsigned int duration, const std::string& description = "unnamed sleep instruction");
        /**
         * @brief Executes the sleep instruction.
         *
         * This method overrides the execute function from the AbstractInstruction class.
         */
        void execute() override;
    private:
        unsigned int duration;                                            /* The duration in milliseconds to sleep. */
        inline static const std::string LOGGER_NAME = "SleepInstruction"; /* The name of the logger. */
    };
} // namespace instruction

#endif // SLEEPINSTRUCTION_HPP
