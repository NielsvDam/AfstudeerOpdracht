#include "StateEngine/StatePipeline/PictureStatePipeline.hpp"

#include "StateEngine/Instruction/MovementInstruction.hpp" // instruction::MovementInstruction
#include "StateEngine/Instruction/SleepInstruction.hpp"    // instruction::SleepInstruction
#include "StateEngine/Instruction/PictureInstruction.hpp"  // instruction::PictureInstruction
#include "MachineStateControlNode.hpp"

// DEBUG

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
            // <<! DEBUG prints, remove when not needed. >>
            RCLCPP_INFO(logger, "Got the following position: ");
            cameraPose = moveGroup->getCurrentPose(crateBottomFrame);
            RCLCPP_INFO(logger, "Pose: position(%.3f, %.3f, %.3f), rotation(%f, %f, %f, %f)",cameraPose.pose.position.x,cameraPose.pose.position.y,cameraPose.pose.position.z,cameraPose.pose.orientation.w,cameraPose.pose.orientation.x,cameraPose.pose.orientation.y,cameraPose.pose.orientation.z);
        // get the picture distance
        double pictureDistance = MachineStateControlNode::getInstance()->get_parameter("picture_distance").as_double();
        // get the pose the frame
        cameraPose = moveGroup->getCurrentPose(crateBottomFrame);
        // adjust the picture pose
        cameraPose.pose.position.z += pictureDistance;
        // rotate the picture pose to correct for any strange rotations
        // TODO: Get these orientations/calculate them from the URDF file, in order to prevent hardcoding them.
        // #define RAD_ANGLE_45 0.707107; // So essentially: Replace this with a function to get and transform the rotations from rpy out of the file to quaternion, preferably through external file config.
        // cameraPose.pose.orientation.w -= RAD_ANGLE_45;
        // cameraPose.pose.orientation.z -= RAD_ANGLE_45;
        RCLCPP_INFO(logger, "PictureStatePipeline created.");
        // <<! DEBUG >>
        RCLCPP_INFO(logger, "Pose: position(%.3f, %.3f, %.3f), rotation(%f, %f, %f, %f)",cameraPose.pose.position.x,cameraPose.pose.position.y,cameraPose.pose.position.z,cameraPose.pose.orientation.w,cameraPose.pose.orientation.x,cameraPose.pose.orientation.y,cameraPose.pose.orientation.z);
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
                    auto trajectory = createTrajectory(cameraPose, "tool_tcp", "PTP"); // HARDCODED: TODO, was rv5as_camera_tcp :: It appears to not be fixed by setting the camera_tcp to the new one, as this seems to utterly ruin the path planner.
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
                    geometry_msgs::msg::Pose prevPose = getStartPose("tool_tcp");  // HARDCODED : TODO : was tool_tcp
                    prevPose.position.z = 0.0; // set the z position to 0 (which is above the crate)
                    auto trajectory = createTrajectory(prevPose, "tool_tcp", "LIN");  // HARDCODED : TODO
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
                    auto trajectory = createTrajectory(cameraPose, "tool_tcp", "PTP"); // HARDCODED : TODO, see line 66 or near.
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