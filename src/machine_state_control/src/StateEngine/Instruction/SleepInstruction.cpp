#include "StateEngine/Instruction/SleepInstruction.hpp"

#include <thread>            // std::this_thread::sleep_for
#include <chrono>            // std::chrono::milliseconds
#include <rclcpp/rclcpp.hpp> // RCLCPP_INFO

namespace instruction
{
    SleepInstruction::~SleepInstruction() {}

    SleepInstruction::SleepInstruction(unsigned int duration, const std::string& description)
        : AbstractInstruction(LOGGER_NAME, description), duration(duration)
    {}

    void SleepInstruction::execute()
    {
        RCLCPP_INFO(logger, "Sleeping for %d milliseconds", duration);
        std::this_thread::sleep_for(std::chrono::milliseconds(duration));
    }
} // namespace instruction