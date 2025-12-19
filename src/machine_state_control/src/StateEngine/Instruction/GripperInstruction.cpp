#include "StateEngine/Instruction/GripperInstruction.hpp"

#include <melfa_msgs/srv/gpio_configure.hpp> // melfa_msgs::srv::GpioConfigure
#include <rclcpp/rclcpp.hpp>                 // RCLCPP_INFO

#include "MachineStateControlNode.hpp"

namespace instruction
{
    GripperInstruction::~GripperInstruction() {}

    GripperInstruction::GripperInstruction(bool open, const std::string& description)
        : AbstractInstruction(LOGGER_NAME, description), open(open)
    {}

    void GripperInstruction::execute()
    {
        // Alter this to reflect the new gripper system.
        // Simply put: Replace request into open/close function to send out service calls.
        // Beforehand: Check if this should be done here, or if it should be handled like the Mitsubishi IO, with a client from the MachineStateControl Node.

        const uint16_t GRIPPER_OPEN = 0b10;
        const uint16_t GRIPPER_CLOSE = 0b01;

        auto request = std::make_shared<melfa_msgs::srv::GpioConfigure::Request>();
        request->bitid = 900;
        request->mode = "WRITE_OUT";
        request->bitmask = 0xFFFF;
        request->bitdata = open ? GRIPPER_OPEN : GRIPPER_CLOSE;

        auto gripperServiceClient = MachineStateControlNode::getInstance()->getGripperServiceClient();
        RCLCPP_INFO(logger, "Sending gripper command: %s", open ? "open" : "close");
        gripperServiceClient->async_send_request(request);
    }
} // namespace instruction