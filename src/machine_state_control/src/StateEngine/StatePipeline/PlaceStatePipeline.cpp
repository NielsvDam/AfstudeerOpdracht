#include "StateEngine/StatePipeline/PlaceStatePipeline.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> // tf2::toMsg
#include "Rad.hpp"                                 // RAD_180DEG, RAD_90DEG, RAD_45DEG

#include "StateEngine/Instruction/MovementInstruction.hpp" // instruction::MovementInstruction
#include "StateEngine/Instruction/GripperInstruction.hpp"  // instruction::GripperInstruction
#include "StateEngine/Instruction/SleepInstruction.hpp"    // instruction::SleepInstruction
#include "MachineStateControlNode.hpp"                     // ControlNode

namespace state_pipeline
{
    PlaceStatePipeline::~PlaceStatePipeline() {}

    PlaceStatePipeline::PlaceStatePipeline(
        const moveit_msgs::msg::RobotState& startJointValues,
        const tf2::Quaternion& pickOrientation,
        const std::shared_ptr<moveit::planning_interface::MoveGroupInterface>& moveGroup)
        : AbstractStatePipeline(
              LOGGER_NAME,
              startJointValues,
              moveGroup,
              std::make_shared<state::PlaceState>(startJointValues)),
          pickOrientation(pickOrientation),
          moveGroup(moveGroup)
    {
        RCLCPP_INFO(logger, "PlaceStatePipeline created.");
    }

    std::shared_ptr<instruction::AbstractInstruction> PlaceStatePipeline::createInstruction(uint8_t instructionNumber)
    {
        switch (instructionNumber)
        {
            case 0: // move to PrePlacePose
            {
                placePose = getNextPlacePose();
                auto prePlacePose = placePose;
                prePlacePose.position.z += 0.2;
                RCLCPP_INFO(logger, "Creating trajectory to pre-place position, printing position in next info message.");

                RCLCPP_INFO(logger, "Pre-place pose set to x: %f, y: %f, z: %f. rotation: %f, %f, %f, %f.",
                prePlacePose.position.x,
                prePlacePose.position.y,
                prePlacePose.position.z,
                prePlacePose.orientation.x,
                prePlacePose.orientation.y,
                prePlacePose.orientation.z,
                prePlacePose.orientation.w);

                // Send preplacepose instruction over.
                auto trajectory = createTrajectory(prePlacePose, "tool_tcp", "LIN"); // HARDCODED : TODO
                return std::make_shared<instruction::MovementInstruction>(
                    trajectory,
                    getGoalJointTolerance(),
                    "move to prePlacePose");
            }
            case 1: // move to PlacePose
            {
                placePose = getNextPlacePose();
                RCLCPP_INFO(logger, "Creating trajectory to place position, printing position in next info message.");

                RCLCPP_INFO(logger, "Place pose set to x: %f, y: %f, z: %f. rotation: %f, %f, %f, %f.",
                placePose.position.x,
                placePose.position.y,
                placePose.position.z,
                placePose.orientation.x,
                placePose.orientation.y,
                placePose.orientation.z,
                placePose.orientation.w);

                // Normal placepose instruction.
                auto trajectory = createTrajectory(placePose, "tool_tcp", "LIN"); // HARDCODED : TODO
                return std::make_shared<instruction::MovementInstruction>(
                    trajectory,
                    getGoalJointTolerance(),
                    "move to PlacePose");
            }
            case 2: // open gripper
            {
                return std::make_shared<instruction::GripperInstruction>(false, "open gripper");
            }
            case 3: // wait for gripper to close
            {
                return std::make_shared<instruction::SleepInstruction>(125, "wait for gripper to open");
            }
            case 4: // move to retract pose
            {
                // placePose.position.y += 0.0185; // Due to new gripper shape, moving back might/will cause collisions to occur with the object. Normally this line prevents collision on a parallel gripper. 
                placePose.position.z += 0.2;   // move up a bit. // Move up a bit more (0.02 -> 0.2) so I don't have to make "Post-post place pose", temporary fix to weird plan execution.
                auto trajectory = createTrajectory(placePose, "tool_tcp", "LIN", NEAR_OBJECT_VELOCITY_SCALING); // HARDCODED : TODO
                markCompleted();
                return std::make_shared<instruction::MovementInstruction>(
                    trajectory,
                    getGoalJointTolerance(),
                    "move to PostPlacePose");
            }
            default:
            {
                RCLCPP_ERROR(logger, "Unknown instruction number %d", instructionNumber);
                throw std::invalid_argument("Unknown instruction number");
            }
        }
        return nullptr;
    }

    // NOLINTNEXTLINE(readability-convert-member-functions-to-static) // it may be static now but may not be in the future
    geometry_msgs::msg::Pose PlaceStatePipeline::getNextPlacePose()
    {
        double roll = 0.0;
        double pitch = 0.0;
        double yaw = 0.0;
        tf2::Matrix3x3(pickOrientation).getRPY(roll, pitch, yaw);
        if (pitch > RAD_45DEG / 2)
        {
            pitch -= RAD_90DEG;
        }

        // std::shared_ptr<moveit::planning_interface::MoveGroupInterface>& moveGroup;

        std::string place_link = MachineStateControlNode::getInstance()->get_parameter("place_center_link").as_string();
        geometry_msgs::msg::PoseStamped place_center = moveGroup->getCurrentPose(place_link);
        // Make configurable
        const double range_x = 0.02;
        const double range_y = 0.02;

        const double Z_START = place_center.pose.position.z; // Non-const to allow usage in other non-const?
        const double MIN_X = place_center.pose.position.x - (range_x/2);
        const double MAX_X = place_center.pose.position.x + (range_x/2);

        const double MIN_Y = place_center.pose.position.y - (range_y/2);
        const double MAX_Y = place_center.pose.position.y + (range_y/2);

        const double BLOCK_LENGTH = 0.0185;
        const double Z_STACK_CLEARANCE = 0.003; // 3mm clearance to avoid collision with the block on top
        const double X_STEP = 0.10;                 // 5cm
        const double Y_STEP = BLOCK_LENGTH + 0.002; // + 2mm to avoid collision with other block
        const double MAX_Z = Z_START + (BLOCK_LENGTH * 2) + Z_STACK_CLEARANCE; // 2 blocks high + clearance
        double curX = MIN_X;
        double curY = MIN_Y;
        double curZ = Z_START + (BLOCK_LENGTH/2); // Additional height offset of /2 cube height to make sure collisions don't occur with the robot stand.
        bool finished = false;

        geometry_msgs::msg::Pose placePose;
        placePose.position.x = curX;
        placePose.position.y = curY;
        placePose.position.z = curZ + MachineStateControlNode::getInstance()->get_parameter("place_z_offset").as_double();

        tf2::Quaternion quaternion;
        quaternion.setRPY(0.0, pitch, -RAD_90DEG);
        placePose.orientation = tf2::toMsg(quaternion);

        if (finished)
        {
            curZ = Z_START;
            curX = 0.0;
            curY = 0.5;
        }
        else
        {
            // Step to next y (row)
            curY += Y_STEP;
            if (curY > MAX_Y)
            {
                curY = MIN_Y;
                curX += X_STEP;
                if (curX > MAX_X)
                {
                    
                    curX = MIN_X;
                    if(curZ == Z_START)
                    {
                        curZ += Z_STACK_CLEARANCE; // first time we are here, we need to move up a bit
                    }

                    curZ += BLOCK_LENGTH;
                    if (curZ >= MAX_Z)
                    {
                        finished = true;
                    }
                }
            }
        }

        return placePose;
    }
} // namespace state_pipeline