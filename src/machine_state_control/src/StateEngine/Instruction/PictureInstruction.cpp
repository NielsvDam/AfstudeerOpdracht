#include "StateEngine/Instruction/PictureInstruction.hpp"

#include <rclcpp/rclcpp.hpp> // RCLCPP_INFO

#include "MachineStateControlNode.hpp"

namespace instruction
{
    PictureInstruction::~PictureInstruction() {}

    PictureInstruction::PictureInstruction(const std::string& description)
        : AbstractInstruction(LOGGER_NAME, description), executePictureDone(false)
    {}

    void PictureInstruction::execute()
    {
        std::shared_ptr<MachineStateControlNode> controlNode = MachineStateControlNode::getInstance();
        auto client = controlNode->getPictureClient();
        // Create the goal message
        auto goal_msg = custom_msgs::action::Picture::Goal();

        // Create SendGoalOptions object
        auto options = rclcpp_action::Client<custom_msgs::action::Picture>::SendGoalOptions();

        // assign the callbacks
        options.result_callback =
            std::bind(&MachineStateControlNode::pictureResultCallback, controlNode, std::placeholders::_1);
        options.feedback_callback =
            std::bind(&PictureInstruction::pictureFeedbackCallback, this, std::placeholders::_1, std::placeholders::_2);

        RCLCPP_INFO(logger, "Calling picture action.");

        // Send goal asynchronously with the options
        client->async_send_goal(goal_msg, options);

        // Wait for the picture to complete.
        std::unique_lock<std::mutex> lock(executePictureMtx);
        executePictureCv.wait(lock, [this] { return executePictureDone; });
    }

    void PictureInstruction::pictureFeedbackCallback(
        const rclcpp_action::ClientGoalHandle<custom_msgs::action::Picture>::SharedPtr& goalHandle,
        const std::shared_ptr<const custom_msgs::action::Picture::Feedback>& feedback)
    {
        (void)goalHandle; // suppress unused parameter warning
        if (feedback->picture_taken)
        {
            RCLCPP_INFO(logger, "Received action feedback. Picture is taken!");
            std::lock_guard<std::mutex> lock(executePictureMtx);
            executePictureDone = true;
            executePictureCv.notify_all();
        }
        else
        {
            RCLCPP_ERROR(logger, "Received feedback but picture was not taken.");
            throw std::logic_error("Received feedback but picture was not taken.");
        }
    }
} // namespace instruction