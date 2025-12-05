#include "PickSolutionFinder/PickSolutionFinder.hpp"

#include <moveit/robot_state/robot_state.h>     // moveit::core::RobotState
#include <shape_msgs/msg/solid_primitive.hpp>   // shape_msgs::msg::SolidPrimitive
#include <moveit_msgs/msg/collision_object.hpp> // moveit_msgs::msg::CollisionObject

#include "PickSolutionFinder/PickPoseUtils.hpp" // generatePickPoses(), rotatePose(), calculateRetractPose()

#include "MachineStateControlNode.hpp"

namespace pick_solution_finder
{
    PickSolutionFinder::~PickSolutionFinder() {}

    PickSolutionFinder::PickSolutionFinder(
        const std::vector<custom_msgs::msg::LocatedObject>& detectedObjects,
        const std::vector<custom_msgs::msg::LocatedObject>& unknownAreas,
        const std::shared_ptr<moveit::planning_interface::MoveGroupInterface>& moveGroup)
        : detectedObjects(detectedObjects), unknownAreas(unknownAreas), moveGroup(moveGroup)
    {}

    std::shared_ptr<PickSolution> PickSolutionFinder::findSolution()
    {
        // Iterate through the pick solutions and validate each one
        for (const custom_msgs::msg::LocatedObject& locatedObject : detectedObjects)
        {
            // get a vector of pick poses for the given object.
            std::vector<geometry_msgs::msg::Pose> potentionalPickPoses =
                PickPoseUtils::generatePickPoses(locatedObject.pose);

            // test the poses for collisions
            for (const auto& pickPose : potentionalPickPoses)
            {
                // Rotate the just generated pose to account for 90degree angle difference due to URDF definition.
                auto rotatedPickPose = PickPoseUtils::rotatePose(pickPose,0.0,0.0,0.0); // <<! TODO:  Check which rotation is needed & crashes. >>

                // Check if the pick pose causes a collision
                if (!isPoseInCollision(rotatedPickPose))
                {
                    // Try to find a retract pose that does not cause a collision
                    for (bool relative : {true, false})
                    {
                        // Get a retract pose (relative or absolute)
                        const static double retractDistance = 0.03;
                        geometry_msgs::msg::Pose retractPose =
                            PickPoseUtils::calculateRetractPose(rotatedPickPose, retractDistance, relative);
                        // Check if the retract pose causes a collision
                        if (!isPoseInCollision(retractPose))
                        {
                            // The solution is found.
                            return std::make_shared<PickSolution>(rotatedPickPose, retractPose); // Changed all pickPose to rotatedPickPose since declaration at currently line 35.
                        }
                    }
                }
            }
        }
        // No valid pick solution found
        return nullptr;
    }

    bool PickSolutionFinder::isPoseInCollision(const geometry_msgs::msg::Pose& pose)
    {
        // Get the robot model
        moveit::core::RobotModelConstPtr robotModel = moveGroup->getRobotModel();
        // Create a robot state
        moveit::core::RobotState robotState(robotModel);
        // set the robot state to have the TCP at the given pose
        const moveit::core::JointModelGroup* jointModelGroup = robotModel->getJointModelGroup(moveGroup->getName());
        bool ikSuccess = robotState.setFromIK(jointModelGroup, pose, "tool_tcp"); // HARDCODED : TODO

        // Check valid IK solution (if the state is actually possible)
        if (!ikSuccess)
        {
            // Could not find a valid IK solution, the robot can't reach the pose
            return true;
        }

        // Create a collision request and result
        collision_detection::CollisionRequest collisionRequest;
        collision_detection::CollisionResult collisionResult;
        collisionRequest.contacts = true;                                // Enable detailed contact information
        collisionRequest.max_contacts = std::numeric_limits<int>::max(); // ensure we dont exclude any important contacts

        // Create a scene and check for collisions, given the requested robot state
        planning_scene::PlanningScene scene(robotModel);

        // Add the collision objects to the scene (if IgnoreBlockCollision is set to false)
        if (!MachineStateControlNode::getInstance()->get_parameter("ignore_block_collisions").as_bool())
        {
            addCollisionObjects(scene);
        }

        // Perform the collision request
        scene.checkCollision(collisionRequest, collisionResult, robotState);

        // Check for problematic collisions
        for (const auto& contact : collisionResult.contacts)
        {
            const auto& first = contact.first.first;
            const auto& second = contact.first.second; // no idea why we have to use first.first and first.second.
            if (isProblematicContact(first, second))
            {
                return true;
            }
        }
        // The pose is valid
        return false;
    }

    /* static */ bool PickSolutionFinder::isProblematicContact(const std::string& first, const std::string& second)
    {
        // Filter the contact names to remove everything but "block"
        auto filterBlockName = [](const std::string& name) -> std::string {
            if (name.find("collision_block") != std::string::npos)
            {
                return "collision_block";
            }
            return name;
        };
        std::string filteredFirst = filterBlockName(first);
        std::string filteredSecond = filterBlockName(second);
        // Define a list of problematic contacts
        const static std::vector<std::pair<std::string, std::string>> problematicContacts = {
            {"tool_gripper", "collision_block"}, // collision_block check.
            {"tool_gripper", "crate"},
            {"link_5", "crate"}, // rv5as_wrist to link_5
            {"camera_mount", "crate"}, 
            {"link_5", "robot_stand"},   // The following have changed: "robot_stand" < "rv5as_table_base"; "camera_mount" < "rv5as_camera"; "tool_gripper" < "rv5as_schrunk_assista"
            {"camera_mount", "robot_stand"}}; // HARDCODED : Might need a fix, currently manually changed.

        // Check if the contact pair is in the list of problematic contacts
        for (const auto& contact : problematicContacts)
        {
            if ((filteredFirst == contact.first && filteredSecond == contact.second) ||
                (filteredFirst == contact.second && filteredSecond == contact.first))
            {
                return true;
            }
        }
        return false;
    }

    void PickSolutionFinder::addCollisionObjects(planning_scene::PlanningScene& scene)
    {
        // Merge the detected objects and unknown areas into one vector
        std::vector<custom_msgs::msg::LocatedObject> objects = this->detectedObjects;
        objects.insert(objects.end(), this->unknownAreas.begin(), this->unknownAreas.end());
        std::size_t count = 0;
        for (const custom_msgs::msg::LocatedObject& object : objects)
        {
            moveit_msgs::msg::CollisionObject collisionObject;
            ++count;
            std::string id = "collision_block_" + std::to_string(count); // Create a unique id for the object
            collisionObject.id = id;
            collisionObject.header.frame_id = "world"; // All objects are in the world frame

            // Define the box shape using the width, length, and height from LocatedObject
            shape_msgs::msg::SolidPrimitive box;
            box.type = shape_msgs::msg::SolidPrimitive::BOX;
            box.dimensions = {object.width, object.length, object.height}; // Set dimensions

            // Set the pose from the LocatedObject
            collisionObject.primitives.push_back(box);
            collisionObject.primitive_poses.push_back(object.pose);
            collisionObject.operation = moveit_msgs::msg::CollisionObject::ADD;
            // Add the collision object to the planning scene
            scene.processCollisionObjectMsg(collisionObject);
        }
    }
} // namespace pick_solution_finder