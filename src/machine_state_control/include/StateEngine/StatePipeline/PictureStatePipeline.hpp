#ifndef PICTURESTATEPIPELINE_HPP
#define PICTURESTATEPIPELINE_HPP

#include <geometry_msgs/msg/pose.h>         // geometry_msgs::msg::Pose
#include <geometry_msgs/msg/pose_stamped.h> // geometry_msgs::msg::PoseStamped
#include <geometry_msgs/msg/transform_stamped.h> // geometry_msgs::msg::TransformStamped

#include "AbstractStatePipeline.hpp"          // state_pipeline::AbstractStatePipeline
#include "StateEngine/State/PictureState.hpp" // state::PictureState

// The maths
#include <tf2_ros/buffer.h>

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
     * @brief A state pipeline for creating instructions for the picture state.
     *
     * @note When the robot gripper is complexly parked inside of the crate, it may be impossible for the planners to
     *       directly plan to the picture pose. In such cases, an alternative strategy is used: first, a plan is created to
     *       move the TCP up, so it hovers above the crate again. After that, it continues to plan to the picture pose.
     *
     * @details
     * - setAlternativePose(pose) sets an alternative pose from where to take the picture.
     */
    class PictureStatePipeline : public AbstractStatePipeline
    {
    public:
        virtual ~PictureStatePipeline();
        /**
         * @brief Constructor for PictureStatePipeline.
         *
         * @param startJointValues The initial joint values of the robot.
         * @param moveGroup A shared pointer to the MoveGroupInterface for planning.
         *
         * @note More info in AbstractStatePipeline::AbstractStatePipeline().
         */
        PictureStatePipeline(
            const moveit_msgs::msg::RobotState& startJointValues,
            const std::shared_ptr<moveit::planning_interface::MoveGroupInterface>& moveGroup);
        /**
         * @brief Set an alternative pose from where to take the picture.
         *
         * @param pose The alternative pose.
         */
        void setAlternativePose(const geometry_msgs::msg::PoseStamped& pose);
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
        bool indirectStrategy;                      /* Flag to indicate if the indirect strategy is active. */
        geometry_msgs::msg::PoseStamped cameraPose; /* The position to take the picture from. */
        // geometry_msgs::msg::PoseStamped cameraTcp; /* The position of the actual camera_tcp from the URDF file, used to calculate the offset between last link &  */
        // geometry_msgs::msg::PoseStamped gripperTcp; /* The position of the 'gripper' defined in the params config file. Currently not the gripper, but the last part of the arm. */
        inline static const std::string LOGGER_NAME = "PictureStatePipeline";
    };
} // namespace state_pipeline

#endif // PICTURESTATEPIPELINE_HPP
