#include "StateEngine/StatePipeline/PictureStatePipeline.hpp"

#include "StateEngine/Instruction/MovementInstruction.hpp" // instruction::MovementInstruction
#include "StateEngine/Instruction/SleepInstruction.hpp"    // instruction::SleepInstruction
#include "StateEngine/Instruction/PictureInstruction.hpp"  // instruction::PictureInstruction
#include "MachineStateControlNode.hpp"

// For the transform logic.
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

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
        cameraPose = moveGroup->getCurrentPose(crateBottomFrame);
        // <<! DEBUG prints, remove when not needed. >>
        // RCLCPP_INFO(logger, "Got the following position: ");
        // RCLCPP_INFO(logger, "Pose: position(%.3f, %.3f, %.3f), rotation(%f, %f, %f, %f)",cameraPose.pose.position.x,cameraPose.pose.position.y,cameraPose.pose.position.z,cameraPose.pose.orientation.w,cameraPose.pose.orientation.x,cameraPose.pose.orientation.y,cameraPose.pose.orientation.z);
        
        // get the picture distance
        double pictureDistance = MachineStateControlNode::getInstance()->get_parameter("picture_distance").as_double();
        // get the pose the frame
        cameraPose = moveGroup->getCurrentPose(crateBottomFrame);
        // adjust the picture pose
        cameraPose.pose.position.z += pictureDistance;
        // rotate the picture pose to correct for any strange rotations
        // TODO: Get these orientations/calculate them from the URDF file, in order to prevent hardcoding them.
        // So essentially: Replace this with a function to get and transform the rotations from rpy out of the file to quaternion, preferably through external file config.
        // Rotational fix.
        #define RAD_45 0.707107
        cameraPose.pose.orientation.w = RAD_45;
        cameraPose.pose.orientation.y = RAD_45;
        cameraPose.pose.orientation.x = 0.0;
        cameraPose.pose.orientation.z = 0.0;
        // Positional fix.
        cameraPose.pose.position.x -= 0.37 - 0.31;
        cameraPose.pose.position.y -= -0.0175 - 0;
        cameraPose.pose.position.z -= 0.364 - 0.322;
        
        
        // // Print positional differences of camera & gripper.
        // const std::string cameraName = MachineStateControlNode::getInstance()->get_parameter("camera_tcp_link").as_string();
        // const std::string gripperName = MachineStateControlNode::getInstance()->get_parameter("gripper_tcp_link").as_string();

        // // Then, retrieve the positions.
        // cameraTcp = moveGroup->getCurrentPose(cameraName);
        // gripperTcp = moveGroup->getCurrentPose(gripperName);

        // RCLCPP_INFO(logger,"Positional offset 3: %.3f, %.3f, %.3f",cameraTcp.pose.position.x - gripperTcp.pose.position.x, cameraTcp.pose.position.y - gripperTcp.pose.position.y, cameraTcp.pose.position.z - gripperTcp.pose.position.z);

        // // Offset the cameraPose 
        //     // At first, set all the names that'll be used for this bit.
        //     const std::string cameraName = MachineStateControlNode::getInstance()->get_parameter("camera_tcp_link").as_string();
        //     const std::string gripperName = MachineStateControlNode::getInstance()->get_parameter("gripper_tcp_link").as_string();

        //     // Then, retrieve the positions.
        //     cameraTcp = moveGroup->getCurrentPose(cameraName);
        //     gripperTcp = moveGroup->getCurrentPose(gripperName);
        //     // Then, to do actual transformations on the objects, reconstruct the TF transforms from the messages.
        //     // Set up the buffer used to get the functions.
        //     // Set up buffer
        //     tf2::Transform gripperTransform, cameraTransform;
        //     tf2::fromMsg(gripperTcp.pose,gripperTransform);
        //     tf2::fromMsg(cameraTcp.pose,cameraTransform);
        //     tf2::Transform gripperTransformed = gripperTransform.inverse() * cameraTransform; // This should be the the transform to get something from the gripper (rn link_6) to the camera (rn camera_tcp).
        //     // Apply the values we just made to the transform, to try: Just do it via a multiplication.
        //     gripperTransform = gripperTransform * gripperTransformed;

        //     cameraTcp.pose;// Send halp, turn tf2::transform into the pose values.
            
        //     // Create a clock to sidestep the Node retrieval issue.
        //     auto tempClock = std::make_shared<rclcpp::Clock>();
        //     // I sure hope this works.
        //     geometry_msgs::msg::PoseStamped output;
        //     auto tf2_buffer = std::make_unique<tf2_ros::Buffer>(tempClock);
        //     auto tf2_listener = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer);
            
        //     // Then, use the newly made buffer to apply (and get) transforms.
        //     try {
        //         tf2_buffer->transform(gripperTcp,output, "camera_tcp");
        //         RCLCPP_INFO(logger,"Succesfully transformed pose, printing output.");
        //         RCLCPP_INFO(logger, "Pose: position(%.3f, %.3f, %.3f), rotation(%f, %f, %f, %f)",output.pose.position.x,output.pose.position.y,output.pose.position.z,output.pose.orientation.w,output.pose.orientation.x,output.pose.orientation.y,output.pose.orientation.z);
        //     } catch (const tf2::TransformException& ex) {
        //         RCLCPP_ERROR(logger,"Could not perform tranformation! Oops.");
        //     }
            


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
                    auto trajectory = createTrajectory(cameraPose, "tool_tcp", "PTP"); // HARDCODED: TODO, was rv5as_link_6 :: It appears to not be fixed by setting the link_6 to the new one, as this seems to utterly ruin the path planner.
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
                    geometry_msgs::msg::Pose prevPose = getStartPose("link_6");  // HARDCODED : TODO : was link_6
                    prevPose.position.z = 0.0; // set the z position to 0 (which is above the crate)
                    auto trajectory = createTrajectory(prevPose, "link_6", "LIN");  // HARDCODED : TODO
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
                    auto trajectory = createTrajectory(cameraPose, "link_6", "PTP"); // HARDCODED : TODO, see line 66 or near.
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