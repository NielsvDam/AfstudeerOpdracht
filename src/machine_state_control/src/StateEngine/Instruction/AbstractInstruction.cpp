#include "StateEngine/Instruction/AbstractInstruction.hpp"

namespace instruction
{
    AbstractInstruction::~AbstractInstruction() {}

    AbstractInstruction::AbstractInstruction(const std::string& LOGGER_NAME, const std::string& description)
        : logger(rclcpp::get_logger(LOGGER_NAME)), description(description)
    {}

    const std::string& AbstractInstruction::getDescription() const
    {
        return description;
    }
} // namespace instruction