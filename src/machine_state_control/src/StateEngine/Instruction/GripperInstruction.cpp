#include "StateEngine/Instruction/GripperInstruction.hpp"

#include <rclcpp/rclcpp.hpp>                 // RCLCPP_INFO

#include "MachineStateControlNode.hpp"
#include "GripperController.hpp"

namespace instruction
{
    GripperInstruction::~GripperInstruction() {}

    GripperInstruction::GripperInstruction(bool open, const std::string& description)
        : AbstractInstruction(LOGGER_NAME, description), open(open)
    {}

    void GripperInstruction::execute()
    {
        int8_t output;

        auto& gripperController = MachineStateControlNode::getInstance()->getGripperController();

        int8_t currentState = gripperController.getGripperState();
        if(currentState==-1){
            RCLCPP_INFO(logger, "Service call failed, check prior logger output.");
            return;
        }
        
        // For performance reasons, ignore request if gripper already at requested state.
        if (open == currentState) {
            RCLCPP_INFO(logger,"Already %s, ignoring command to speed things up.", open ? "opened" : "closed");
            return;
        }

        // Carry out the open/close.
        output = gripperController.setGripperState(open);

        // Check if output was a -1 to cut the further proces short.
        if (!(output == 1 || output == 0)) {
            RCLCPP_INFO(logger, "Service call failed, check prior logger output.");
            return;
        }

        // Report what the output was if valid (0/1).
        if (!output) {
            RCLCPP_INFO(logger, "Gripper %s failed, got false from controller.", open ? "open" : "close");
        } else {
            RCLCPP_INFO(logger, "Gripper %s.", open ? "opened" : "closed");
        }
        
        return;
    }
} // namespace instruction