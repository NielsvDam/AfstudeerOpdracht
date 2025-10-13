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
          pickOrientation(pickOrientation)
    {
        RCLCPP_INFO(logger, "PlaceStatePipeline created.");
    }

    std::shared_ptr<instruction::AbstractInstruction> PlaceStatePipeline::createInstruction(uint8_t instructionNumber)
    {
        switch (instructionNumber)
        {
            case 0: // move to PlacePose
            {
                placePose = getNextPlacePose();
                auto trajectory = createTrajectory(placePose, "rv5as_default_tcp", "LIN");
                return std::make_shared<instruction::MovementInstruction>(
                    trajectory,
                    getGoalJointTolerance(),
                    "move to PlacePose");
            }
            case 1: // open gripper
            {
                return std::make_shared<instruction::GripperInstruction>(false, "open gripper");
            }
            case 2: // wait for gripper to close
            {
                return std::make_shared<instruction::SleepInstruction>(125, "wait for gripper to open");
            }
            case 3: // move to retract pose
            {
                placePose.position.y += 0.0185; // move back just a bit, to avoid collision with the block
                placePose.position.z += 0.02;   // move up a bit.
                auto trajectory = createTrajectory(placePose, "rv5as_default_tcp", "LIN", 0.08);
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

        static const double Z_START = -0.02 + 0.012;
        static const double MIN_X = -0.25;
        static const double MAX_X = 0.25;
        static const double MIN_Y = 0.25;
        static const double MAX_Y = 0.33;
        static const double BLOCK_LENGTH = 0.0185;
        static const double Z_STACK_CLEARANCE = 0.003; // 3mm clearance to avoid collision with the block on top
        static const double X_STEP = 0.10;                 // 5cm
        static const double Y_STEP = BLOCK_LENGTH + 0.002; // + 2mm to avoid collision with other block
        static const double MAX_Z = Z_START + (BLOCK_LENGTH * 2) + Z_STACK_CLEARANCE; // 2 blocks high + clearance

        static double curX = MIN_X;
        static double curY = MIN_Y;
        static double curZ = Z_START;
        static bool finished = false;

        geometry_msgs::msg::Pose placePose;
        placePose.position.x = curX;
        placePose.position.y = curY;
        placePose.position.z = curZ + MachineStateControlNode::getInstance()->get_parameter("place_z_offset").as_double();

        tf2::Quaternion quaternion;
        quaternion.setRPY(RAD_180DEG, pitch, -RAD_90DEG);
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