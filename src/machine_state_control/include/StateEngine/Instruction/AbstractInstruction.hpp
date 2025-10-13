#ifndef ABSTRACTINSTRUCTION_HPP
#define ABSTRACTINSTRUCTION_HPP

#include <string>            // std::string
#include <rclcpp/rclcpp.hpp> // rclcpp::Logger

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
     * @class AbstractInstruction
     * @brief An abstract base class for executable instructions.
     *
     * @details
     * - execute() needs to be implemented by the derived class, and needs to be blocking untill its complete.
     * - the logger member should be used by the derived class to log messages.
     */
    class AbstractInstruction
    {
    public:
        virtual ~AbstractInstruction();
        /**
         * @brief Pure virtual function to execute the instruction.
         *
         * This function is responsible for executing the instruction. It should be blocking untill the instruction is
         * completed.
         */
        virtual void execute() = 0;
        /**
         * @brief Retrieves the description of the instruction.
         *
         * @return A string containing a description of the instruction.
         */
        const std::string& getDescription() const;
    protected:
        /**
         * @brief Constructs an AbstractInstruction object.
         *
         * @param LOGGER_NAME A name for the logger.
         * @param description A brief description of the instruction. Defaults to "unnamed instruction".
         */
        explicit AbstractInstruction(const std::string& LOGGER_NAME, const std::string& description = "unnamed instruction");
        rclcpp::Logger logger; /* A logger object for logging messages. */
    private:
        const std::string description; /* A brief description of the instruction. */
    };
} // namespace instruction

#endif // ABSTRACTINSTRUCTION_HPP
