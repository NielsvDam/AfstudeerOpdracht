#ifndef KNOCKOVERSTATEPIPELINE_HPP
#define KNOCKOVERSTATEPIPELINE_HPP

#include <vector>                   // std::vector
#include <geometry_msgs/msg/pose.h> // geometry_msgs::msg::Pose

#include "AbstractStatePipeline.hpp"            // state_pipeline::AbstractStatePipeline
#include "StateEngine/State/KnockOverState.hpp" // state::KnockOverState

/**
 * @namespace state_pipeline
 * @brief Contains classes related to the state pipeline mechanism.
 *
 * The state_pipeline namespace encapsulates the functionality required to create and manage a sequence of instructions for
 * different states. It includes the AbstractStatePipeline class, which serves as a base class for state pipelines. Each
 * inheriting class is responsible for creating the instructions for a specific state.
 */
namespace state_pipeline
{
    /**
     * @class KnockOverStatePipeline
     * @brief A state pipeline for creating instructions for the knock-over state.
     *
     * @details
     * - setStackPose(pose) sets the pose of the stack.
     * - setDetectedObjects(pose[]) sets the detected objects.
     */
    class KnockOverStatePipeline : public AbstractStatePipeline
    {
    public:
        virtual ~KnockOverStatePipeline();
        /**
         * @brief Constructor for KnockOverStatePipeline.
         *
         * @param startJointValues The initial joint values of the robot.
         * @param moveGroup A shared pointer to the MoveGroupInterface for planning.
         *
         * @note More info in AbstractStatePipeline::AbstractStatePipeline().
         */
        KnockOverStatePipeline(
            const moveit_msgs::msg::RobotState& startJointValues,
            const std::shared_ptr<moveit::planning_interface::MoveGroupInterface>& moveGroup);
        /**
         * @brief Set the pose of the stack.
         *
         * @param pose The pose of the stack.
         */
        void setStackPose(geometry_msgs::msg::Pose pose);
        /**
         * @brief Set the detected objects.
         *
         * The detected objects are used to autonomously decide from where the robot should start and end the knock-over
         * move.
         *
         * @param objects A vector of poses representing the detected objects.
         */
        void setDetectedObjects(const std::vector<geometry_msgs::msg::Pose>& objects);
    protected:
        /**
         * @brief Create an instruction based on the given instruction number.
         *
         * @note More info in AbstractStatePipeline::createInstruction().
         *
         * @param instructionNumber The index number of the instruction.
         * @return A shared pointer to the created instruction.
         */
        std::shared_ptr<instruction::AbstractInstruction> createInstruction(uint8_t instructionNumber) override;
    private:
        std::shared_ptr<geometry_msgs::msg::Pose> stackPose;                    /* The pose of the stack. */
        std::vector<geometry_msgs::msg::Pose> detectedObjects;                  /* The poses of detected objects. */
        inline static const std::string LOGGER_NAME = "KnockOverStatePipeline"; /* The logger name for this pipeline. */
    };

} // namespace state_pipeline

#endif // KNOCKOVERSTATEPIPELINE_HPP
