#ifndef SHUFFLESTATEPIPELINE_HPP
#define SHUFFLESTATEPIPELINE_HPP

#include <geometry_msgs/msg/pose.h> // geometry_msgs::msg::Pose

#include "AbstractStatePipeline.hpp"          // state_pipeline::AbstractStatePipeline
#include "StateEngine/State/ShuffleState.hpp" // state::ShuffleState

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
     * @class ShuffleStatePipeline
     * @brief A state pipeline for creating instructions for the shuffle state.
     */
    class ShuffleStatePipeline : public AbstractStatePipeline
    {
    public:
        virtual ~ShuffleStatePipeline();
        /**
         * @brief Constructor for ShuffleStatePipeline.
         *
         * @param startJointValues The initial joint values of the robot.
         * @param moveGroup A shared pointer to the MoveGroupInterface for planning.
         *
         * @note More info in AbstractStatePipeline::AbstractStatePipeline().
         */
        ShuffleStatePipeline(
            const moveit_msgs::msg::RobotState& startJointValues,
            const std::shared_ptr<moveit::planning_interface::MoveGroupInterface>& moveGroup);
        /**
         * @brief Set the center of the shuffle circle.
         *
         * @param pose The pose representing the center of the shuffle circle.
         */
        void setShuffleCenter(geometry_msgs::msg::Pose pose);
        /**
         * @brief Set the radius of the shuffle circle.
         *
         * @param radius The radius of the shuffle circle.
         */
        void setShuffleRadius(unsigned int radius);
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
        std::shared_ptr<geometry_msgs::msg::Pose> shuffleCenter;              /* The center of the shuffle circle. */
        unsigned int shuffleRadius;                                           /* The radius of the shuffle circle. */
        inline static const std::string LOGGER_NAME = "ShuffleStatePipeline"; /* The logger name for this pipeline. */
    };
} // namespace state_pipeline

#endif // SHUFFLESTATEPIPELINE_HPP
