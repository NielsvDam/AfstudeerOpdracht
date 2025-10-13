#ifndef OBJECTPICKDECIDER_H
#define OBJECTPICKDECIDER_H

#include <moveit/move_group_interface/move_group_interface.h> // moveit::planning_interface::MoveGroupInterface
#include <geometry_msgs/msg/pose.hpp>                         // geometry_msgs::msg::Pose
#include <moveit/planning_scene/planning_scene.h>             // planning_scene::PlanningScene

#include <custom_msgs/msg/located_object.hpp>  // custom_msgs::msg::LocatedObject
#include "PickSolutionFinder/PickSolution.hpp" // pick_solution_finder::PickSolution

/**
 * @namespace pick_solution_finder
 * @brief Contains classes related to the pick solution mechanism.
 *
 * The pick_solution namespace encapsulates the functionality required to determine a suitable pick solution for a located
 * object. It includes the PickSolution class, which represents a found solution.
 */
namespace pick_solution_finder
{
    /**
     * @class PickSolutionFinder
     * @brief A class for finding a pick solution in a vector of located objects.
     *
     * The PickSolutionFinder class is responsible for identifying viable pick solutions from a collection of located
     * objects. It provides the method findSolution() to return a suitable pick solution, if one actually is possible.
     *
     * @details The process involves:
     * - Iterating through the located objects.
     * - Get a vector of pick poses for each object (see PoseUtils::generatePickPoses for more information).
     * - For each pick pose, get a corresponding retract pose (see PoseUtils::calculateRetractPose for more information).
     * - Checking both the pick and retract poses for collisions (see isPoseInCollision for more information).
     * - If both poses are collision-free, a pick solution is considered valid and returned immediately.
     */
    class PickSolutionFinder
    {
    public:
        virtual ~PickSolutionFinder();
        /**
         * @brief Constructs a new PickSolutionFinder object.
         *
         * @param detectedObjects A vector of detected objects.
         * @param unknownAreas A vector of unknown areas.
         *
         * @param moveGroup The move group, necessary to retrieve information about the robot and environment.
         */
        PickSolutionFinder(
            const std::vector<custom_msgs::msg::LocatedObject>& detectedObjects,
            const std::vector<custom_msgs::msg::LocatedObject>& unknownAreas,
            const std::shared_ptr<moveit::planning_interface::MoveGroupInterface>& moveGroup);
        /**
         * @brief Find a pick solution for any of the located objects.
         *
         * @return A shared pointer to the pick solution.
         */
        std::shared_ptr<PickSolution> findSolution();
    private:
        /**
         * @brief Check if the given pose causes a collision, when the robot's TCP is at the pose.
         */
        bool isPoseInCollision(const geometry_msgs::msg::Pose& pose);
        /**
         * @brief Add collision objects to the planning scene.
         *
         * @param scene The planning scene to add the collision objects to.
         */
        void addCollisionObjects(planning_scene::PlanningScene& scene);
        /**
         * @brief Check if a contact is problematic.
         *
         * @param first The name of the first object in the contact.
         * @param second The name of the second object in the contact.
         *
         * @return True if the contact is problematic, false otherwise.
         */
        static bool isProblematicContact(const std::string& first, const std::string& second);

        std::vector<custom_msgs::msg::LocatedObject> detectedObjects;              /* The detected objects. */
        std::vector<custom_msgs::msg::LocatedObject> unknownAreas;                 /* The unkown areas. */
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> moveGroup; /* The move group (to retrieve necessary
                                                                                      information about the robot and
                                                                                      environment) */
    };
} // namespace pick_solution_finder

#endif