#ifndef PICKSOLUTION_H
#define PICKSOLUTION_H

#include <geometry_msgs/msg/pose.hpp> // geometry_msgs::msg::Pose

/**
 * @namespace pick_solution_finder
 * @brief Contains classes related to the pick solution mechanism.
 *
 * The pick_solution_finder namespace encapsulates the functionality required to determine a suitable pick solution for a located
 * object. It includes the PickSolution class, which represents a found solution.
 */
namespace pick_solution_finder
{
    /**
     * @class PickSolution
     * @brief Represents a pick solution.
     *
     * The PickSolution class represents a found pick solution. It contains the pick pose and the retract pose
     * which are used to guide the robot arm to pick up an object and then retract safely.
     */
    class PickSolution
    {
    public:
        virtual ~PickSolution();
        /**
         * @brief Constructs a new PickSolution object.
         *
         * @param pickPose The pick pose.
         * @param retractPose The retract pose.
         */
        PickSolution(geometry_msgs::msg::Pose pickPose, geometry_msgs::msg::Pose retractPose);
        /**
         * @brief Get the pick pose.
         *
         * @return The pick pose.
         */
        geometry_msgs::msg::Pose getPickPose();
        /**
         * @brief Get the retract pose.
         *
         * @return The retract pose.
         */
        geometry_msgs::msg::Pose getRetractPose();
        /**
         * @brief Get the pose above the crate.
         *
         * Represents a pose for hovering above the retract pose (before descending down to it).
         *
         * @return The pose above the crate.
         */
        geometry_msgs::msg::Pose getAboveCratePose();
    private:
        geometry_msgs::msg::Pose pickPose;    /* The pick pose */
        geometry_msgs::msg::Pose retractPose; /* The retract pose */
    };
} // namespace pick_solution_finder

#endif