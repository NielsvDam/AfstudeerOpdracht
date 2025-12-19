#ifndef PICKSTATEPIPELINE_HPP
#define PICKSTATEPIPELINE_HPP

#define NEAR_OBJECT_VELOCITY_SCALING 0.05 // The velocity scaling for any movements near/directly with the objects. Raising this whilst using a robot (instead of a cobot) is seriously ill-advised.
#define FAR_VELOCITY_SCALING 0.1 // The velocity scaling when moving from the picture pose to the above crate pos for the object.
// Scaling also defined in PlaceStatePipeline.hpp, not shared between files yet.

#include "AbstractStatePipeline.hpp"           // state_pipeline::AbstractStatePipeline
#include "StateEngine/State/PickState.hpp"     // state::PickState
#include "PickSolutionFinder/PickSolution.hpp" // pick_solution_finder::PickSolution

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
     * @class PickStatePipeline
     * @brief A state pipeline for creating instructions for the pick state.
     *
     * @details
     * - setPickSolution(pickSolution) sets the pick solution to use.
     */
    class PickStatePipeline : public AbstractStatePipeline
    {
    public:
        virtual ~PickStatePipeline();
        /**
         * @brief Constructor for PickStatePipeline.
         *
         * @param startJointValues The initial joint values of the robot.
         * @param moveGroup A shared pointer to the MoveGroupInterface for planning.
         *
         * @note More info in AbstractStatePipeline::AbstractStatePipeline().
         */
        PickStatePipeline(
            const moveit_msgs::msg::RobotState& startJointValues,
            const std::shared_ptr<moveit::planning_interface::MoveGroupInterface>& moveGroup);
        /**
         * @brief Sets the pick solution
         *
         * @param pickSolution The pick solution to set.
         */
        void setPickSolution(const std::shared_ptr<pick_solution_finder::PickSolution>& pickSolution);
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
        std::shared_ptr<pick_solution_finder::PickSolution> pickSolution;  /* The pick solution to use. */
        inline static const std::string LOGGER_NAME = "PickStatePipeline"; /*The logger name for this pipeline.*/
    };

} // namespace state_pipeline

#endif // PICKSTATEPIPELINE_HPP
