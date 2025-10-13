#include "PickSolutionFinder/PickPoseUtils.hpp"

#include <tf2/LinearMath/Quaternion.h>             // tf2::Quaternion
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> // tf2::fromMsg, tf2::toMsg
#include "Rad.hpp"                                 // RAD_90DEG, RAD_45DEG

namespace pick_solution_finder
{
    /* static */ std::vector<geometry_msgs::msg::Pose> PickPoseUtils::generatePickPoses(
        const geometry_msgs::msg::Pose& objectPose)
    {
        std::vector<geometry_msgs::msg::Pose> poses;
        // 1. Original pose
        poses.push_back(objectPose);
        // 2. Yaw +90
        poses.push_back(rotatePose(objectPose, 0, 0, RAD_90DEG));
        // 3. Original pose with pitch -45
        poses.push_back(rotatePose(objectPose, 0, -RAD_45DEG, 0));
        // 4. Yaw +90 with pitch -45
        poses.push_back(rotatePose(objectPose, 0, -RAD_45DEG, RAD_90DEG));
        // 5. Yaw +180 with pitch -45
        poses.push_back(rotatePose(objectPose, 0, -RAD_45DEG, RAD_180DEG));
        // 6. Yaw +270 with pitch -45
        poses.push_back(rotatePose(objectPose, 0, -RAD_45DEG, RAD_270DEG));
        return poses;
    }

    /* static */ geometry_msgs::msg::Pose PickPoseUtils::rotatePose(
        const geometry_msgs::msg::Pose& pose,
        double roll,
        double pitch,
        double yaw)
    {
        // Convert the msg::Quaternion to a tf2::Quaternion.
        tf2::Quaternion originalOrientation;
        tf2::fromMsg(pose.orientation, originalOrientation);

        // Create a Quaternion for the requested change in rotation.
        tf2::Quaternion requestedOrientation;
        requestedOrientation.setRPY(roll, pitch, yaw);

        // Multiply the original orientation by the requested orientation.
        tf2::Quaternion result = originalOrientation * requestedOrientation;
        result.normalize(); // Normalize the quaternion.

        // Create a new Pose message with the rotated orientation.
        geometry_msgs::msg::Pose rotatedPose = pose;
        rotatedPose.orientation = tf2::toMsg(result);
        return rotatedPose;
    }

    /* static */ geometry_msgs::msg::Pose PickPoseUtils::calculateRetractPose(
        const geometry_msgs::msg::Pose& pickPose,
        double retractDistance,
        bool relative)
    {
        geometry_msgs::msg::Pose retractPose = pickPose;
        // Check if the retract pose should be relative, or absolute
        if (relative)
        {
            // Create a tf2::Transform from the pose
            tf2::Transform transform;
            tf2::fromMsg(retractPose, transform);

            // Move the position up in Z direction relative to the object's orientation
            tf2::Vector3 translation(0, 0, -retractDistance);
            tf2::Transform translationTransform(tf2::Quaternion::getIdentity(), translation);
            transform = transform * translationTransform;

            // Convert the transform back to a Pose message
            tf2::toMsg(transform, retractPose);
        }
        else
        {
            // just move the pose up in Z direction regardless of the object's orientation
            retractPose.position.z += retractDistance;
        }
        return retractPose;
    }
} // namespace pick_solution_finder
