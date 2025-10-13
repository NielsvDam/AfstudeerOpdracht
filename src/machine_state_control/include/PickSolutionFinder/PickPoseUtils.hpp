#ifndef PICKPOSEUTILS_H
#define PICKPOSEUTILS_H

#include <vector>                     // std::vector
#include <geometry_msgs/msg/pose.hpp> // geometry_msgs::msg::Pose

/**
 * @namespace pick_solution_finder
 * @brief Contains classes related to the pick solution mechanism.
 *
 * The pick_solution_finder namespace encapsulates the functionality required to determine a suitable pick solution for a
 * located object. It includes the PickSolution class, which represents a found solution.
 */
namespace pick_solution_finder
{
    /**
     * @class PickPoseUtils
     * @brief A class for generating pick poses and calculating retract poses.
     *
     * The PickPoseUtils class provides static methods for generating pick poses, calculating retract poses and rotating
     * poses.
     */
    class PickPoseUtils
    {
    public:
        // Delete anything that can make this class instantiable or copyable
        PickPoseUtils() = delete;
        ~PickPoseUtils() = delete;
        PickPoseUtils(const PickPoseUtils&) = delete;
        PickPoseUtils& operator=(const PickPoseUtils&) = delete;

        /**
         * @brief Generate a vector of pick poses for the given objectPose.
         *
         * This function generates a set of poses by rotating the given objectPose around its position.
         * The resulting vector contains poses that are candidates for picking the object.
         *
         * Specifically, the pose is first rotated by 0 and 90 degrees around the YAW axis. For each of these YAW rotations,
         * the pose is further rotated by +45 and -45 degrees around the PITCH axis. All of the described rotations are
         * added to the result vector.
         *
         * @param objectPose The pose of the located object.
         *
         * @return A vector containing the pick poses.
         */
        static std::vector<geometry_msgs::msg::Pose> generatePickPoses(const geometry_msgs::msg::Pose& objectPose);
        /**
         * @brief Get a retract pose for the given pick pose.
         *
         * @param pickPose The pick pose.
         * @param retractDistance The distance to retract from the pick pose.
         * @param relative True if the retract pose should be relative to the pick pose, false if it should be absolute.
         *
         * @return The retract pose.
         */
        static geometry_msgs::msg::Pose calculateRetractPose(const geometry_msgs::msg::Pose& pickPose, double retractDistance, bool relative);
        /**
         * @brief Rotate a pose by the given roll, pitch, and yaw.
         *
         * @param pose The pose to rotate.
         * @param roll The roll angle in radians.
         * @param pitch The pitch angle in radians.
         * @param yaw The yaw angle in radians.
         *
         * @return The rotated pose.
         */
        static geometry_msgs::msg::Pose rotatePose(
            const geometry_msgs::msg::Pose& pose,
            double roll,
            double pitch,
            double yaw);
    private:
    };
} // namespace pick_solution_finder
#endif