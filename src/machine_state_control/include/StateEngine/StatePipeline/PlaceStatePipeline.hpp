#ifndef PLACESTATEPIPELINE_HPP
#define PLACESTATEPIPELINE_HPP

#define NEAR_OBJECT_VELOCITY_SCALING 0.05 // The velocity scaling for any movements near/directly with the objects. Raising this whilst using a robot (instead of a cobot) is seriously ill-advised.
// Scaling also defined in PickStatePipeline.hpp, not shared between files yet.

#include <tf2/LinearMath/Quaternion.h> // tf2::Quaternion

#include "AbstractStatePipeline.hpp"        // state_pipeline::AbstractStatePipeline
#include "StateEngine/State/PlaceState.hpp" // state::PlaceState

#include <geometry_msgs/msg/pose.h>         // geometry_msgs::msg::Pose
#include <geometry_msgs/msg/pose_stamped.h> // geometry_msgs::msg::PoseStamped

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
     * @class PlaceStatePipeline
     * @brief A state pipeline for creating instructions for the place state.
     */
    class PlaceStatePipeline : public AbstractStatePipeline
    {
    public:
        virtual ~PlaceStatePipeline();
        /**
         * @brief Constructor for PlaceStatePipeline.
         *
         * @param startJointValues The initial joint values of the robot.
         * @param pickOrientation The orientation of the pick pose.
         * @param moveGroup A shared pointer to the MoveGroupInterface for planning.
         *
         * @note More info in AbstractStatePipeline::AbstractStatePipeline().
         */
        PlaceStatePipeline(
            const moveit_msgs::msg::RobotState& startJointValues,
            const tf2::Quaternion& pickOrientation,
            const std::shared_ptr<moveit::planning_interface::MoveGroupInterface>& moveGroup);
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
        /**
         * @brief Get the pose of the vice for placing the object.
         * 
         * @return geometry_msgs::msg::Pose
         */
        geometry_msgs::msg::Pose getNextPlacePose();
        inline static const std::string LOGGER_NAME = "PlaceStatePipeline"; /* The logger name for this pipeline. */
        geometry_msgs::msg::Pose placePose; /* The pose of the vice for placing the object. */
        tf2::Quaternion pickOrientation;    /* The orientation of the pick pose. */
        const std::shared_ptr<moveit::planning_interface::MoveGroupInterface>& moveGroup;
    };
} // namespace state_pipeline

#endif // PLACESTATEPIPELINE_HPP
