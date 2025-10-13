#ifndef PICTUREINSTRUCTION_HPP
#define PICTUREINSTRUCTION_HPP

#include <mutex>                           // std::mutex
#include <condition_variable>              // std::condition_variable
#include <rclcpp_action/rclcpp_action.hpp> // rclcpp_action::ClientGoalHandle

#include <custom_msgs/action/picture.hpp> // custom_msgs::action::Picture
#include "AbstractInstruction.hpp"        // instruction::AbstractInstruction

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
     * @class PictureInstruction
     * @brief Represents an instruction to take a picture.
     */
    class PictureInstruction : public AbstractInstruction
    {
    public:
        virtual ~PictureInstruction();
        /**
         * @brief Constructs a new PictureInstruction object.
         *
         * @param description A string describing the instruction. Defaults to "unnamed picture instruction".
         */
        explicit PictureInstruction(const std::string& description = "unnamed picture instruction");
        /**
         * @brief Executes the picture instruction.
         *
         * This method overrides the execute function from the AbstractInstruction class.
         */
        void execute() override;
    private:
        /**
         * @brief Callback function to handle the feedback of the called Picture action in the execute() method.
         *
         * @param goalHandle The goal handle of the action.
         * @param feedback The feedback of the action.
         */
        void pictureFeedbackCallback(
            const rclcpp_action::ClientGoalHandle<custom_msgs::action::Picture>::SharedPtr& goalHandle,
            const std::shared_ptr<const custom_msgs::action::Picture::Feedback>& feedback);

        inline static const std::string LOGGER_NAME = "PictureInstruction"; /* The name of the logger. */

        bool executePictureDone;                  /* A boolean indicating if the picture is taken. */
        std::mutex executePictureMtx;             /* A mutex to protect the executePictureDone variable. */
        std::condition_variable executePictureCv; /* A condition variable to wait for the picture to be taken. */
    };
} // namespace instruction

#endif // PICTUREINSTRUCTION_HPP
