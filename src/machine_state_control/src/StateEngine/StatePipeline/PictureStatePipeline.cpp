#include "StateEngine/StatePipeline/PictureStatePipeline.hpp"

#include "StateEngine/Instruction/MovementInstruction.hpp" // instruction::MovementInstruction
#include "StateEngine/Instruction/SleepInstruction.hpp"    // instruction::SleepInstruction
#include "StateEngine/Instruction/PictureInstruction.hpp"  // instruction::PictureInstruction
#include "MachineStateControlNode.hpp"

namespace state_pipeline
{
    PictureStatePipeline::~PictureStatePipeline() {}

    PictureStatePipeline::PictureStatePipeline(
        const moveit_msgs::msg::RobotState& startJointValues,
        const std::shared_ptr<moveit::planning_interface::MoveGroupInterface>& moveGroup)
        : AbstractStatePipeline(
              LOGGER_NAME,
              startJointValues,
              moveGroup,
              std::make_shared<state::PictureState>(startJointValues)),
          indirectStrategy(false)
    {
        // get the frame of the bottom of the crate
        const std::string crateBottomFrame =
            MachineStateControlNode::getInstance()->get_parameter("crate_inner_bottom_center_link").as_string();
        // get the picture distance
        double pictureDistance = MachineStateControlNode::getInstance()->get_parameter("picture_distance").as_double();
        // get the pose the frame
        cameraPose = moveGroup->getCurrentPose(crateBottomFrame);
        // adjust the picture pose
        cameraPose.pose.position.z += pictureDistance;
        RCLCPP_INFO(logger, "PictureStatePipeline created.");
    }

    void PictureStatePipeline::setAlternativePose(const geometry_msgs::msg::PoseStamped& pose)
    {
        this->cameraPose = pose;
        RCLCPP_INFO(
            logger,
            "Alternative picture pose set to x: %f, y: %f, z: %f. rotation: %f, %f, %f, %f. frame: %s",
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z,
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
            pose.header.frame_id.c_str());
    }

    std::shared_ptr<instruction::AbstractInstruction> PictureStatePipeline::createInstruction(uint8_t instructionNumber)
    {
        switch (instructionNumber)
        {
            case 0: // move to PicturePose (or to PrePicturePose if the direct approach fails)
            {
                try
                {
                    auto trajectory = createTrajectory(cameraPose, "rv5as_camera_tcp", "PTP");
                    // return the direct movement instruction
                    return std::make_shared<instruction::MovementInstruction>(
                        trajectory,
                        getGoalJointTolerance(),
                        "move directly to PicturePose");
                }
                // If the direct approach fails, try the indirect approach (by moving the TCP up, into the void).
                catch (const std::runtime_error& e)
                {
                    RCLCPP_INFO(logger, "Failed to create trajectory directly to PicturePose, trying a indirect strategy");
                    indirectStrategy = true; // set the indirect strategy flag
                    geometry_msgs::msg::Pose prevPose = getStartPose("rv5as_default_tcp");
                    prevPose.position.z = 0.0; // set the z position to 0 (which is above the crate)
                    auto trajectory = createTrajectory(prevPose, "rv5as_default_tcp", "LIN");
                    // return the indirect movement instruction
                    return std::make_shared<instruction::MovementInstruction>(
                        trajectory,
                        getGoalJointTolerance(),
                        "move to PrePicturePose");
                }
            }
            case 1: // move to PicturePose (if the indirect strategy was used as a fallback solution)
            {
                if (indirectStrategy)
                {
                    auto trajectory = createTrajectory(cameraPose, "rv5as_camera_tcp", "PTP");
                    return std::make_shared<instruction::MovementInstruction>(
                        trajectory,
                        getGoalJointTolerance(),
                        "move to PicturePose");
                }
                [[fallthrough]]; // Suppress [-Wimplicit-fallthrough=], it is intended to fall through here
            }
            case 2: // take picture
            {
                markCompleted();
                return std::make_shared<instruction::PictureInstruction>("take picture");
            }
            default:
            {
                RCLCPP_WARN(logger, "Unknown instruction number %d", instructionNumber);
                throw std::invalid_argument("Unknown instruction number");
            }
        }
        return nullptr;
    }
} // namespace state_pipeline