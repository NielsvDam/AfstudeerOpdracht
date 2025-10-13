#ifndef ABSTRACTSTATEPIPELINE_HPP
#define ABSTRACTSTATEPIPELINE_HPP

#include <memory>                                             // std::shared_ptr
#include <cstdint>                                            // uint8_t
#include <moveit/move_group_interface/move_group_interface.h> // moveit::planning_interface::MoveGroupInterface
#include <moveit_msgs/msg/robot_state.hpp>                    // moveit_msgs::msg::RobotState
#include <moveit_msgs/msg/robot_trajectory.hpp>               // moveit_msgs::msg::RobotTrajectory

#include "StateEngine/State/AbstractState.hpp" // state::AbstractState

/**
 * @namespace state_pipeline
 * @brief Contains classes related to the state pipeline mechanism.
 *
 * The state_pipeline namespace encapsulates the functionality required to create and manage a sequence of instructions for
 * different states. It includes the AbstractStatePipeline class, which serves as a base class for state pipelines. Each
 * inheriting class is responsible for creating the instructions for a specific state.
 */
namespace state_pipeline
{
    /**
     * @class AbstractStatePipeline
     * @brief An abstract base class for state pipelines.
     *
     * This class provides a mechanism for creating a sequence of instructions for a state. The sub classes are responsible
     * for overriding the createInstruction() function to create the actual instructions. The createNextInstruction()
     * function is public and used to create the next instruction in the pipeline, it should be continously called untill it
     * returns false, which indicates the pipeline is finished.
     *
     * The getState() can directly be used, even if not all instructions have been created.
     *
     * @note This class is intended to be subclassed and cannot be instantiated directly.
     */
    class AbstractStatePipeline
    {
    public:
        virtual ~AbstractStatePipeline();
        /**
         * @brief Get the state of the pipeline.
         *
         * This function returns the state of the pipeline.
         *
         * @return std::shared_ptr<AbstractState> The state of the pipeline.
         */
        const std::shared_ptr<state::AbstractState> getState();
        /**
         * @brief Creates the next instruction in the state pipeline.
         *
         * This function generates the next instruction for the current state pipeline. It will return true if no more
         * instructions are to be created, false otherwise.
         *
         * If the pipeline failed creating the instruction (i.e. the instruction is a nullptr), this function throws a
         * runtime error. This is due to the fact that planning can be out of possibilities.
         *
         * @return true if the pipeline is done.
         *
         * @throws std::runtime_error if the created instruction is a nullptr.
         */
        bool createNextInstruction();
    protected:
        /**
         * @brief Construct a new AbstractStatePipeline object.
         *
         * @note This object can only be constructed by derived classes.
         *
         * @param loggerName The name of the logger to be used.
         * @param startJointValues The initial joint values of the robot.
         * @param moveGroup A shared pointer to the MoveGroupInterface for planning.
         * @param state A shared pointer to the initial state of the pipeline.
         */
        AbstractStatePipeline(
            const std::string& loggerName,
            const moveit_msgs::msg::RobotState& startJointValues,
            const std::shared_ptr<moveit::planning_interface::MoveGroupInterface>& moveGroup,
            const std::shared_ptr<state::AbstractState>& state);
        /**
         * @brief Creates an instruction.
         *
         * This is a pure virtual function that must be implemented by derived classes. It is responsible for creating and
         * returning a shared pointer to an instruction.
         *
         * The instruction number increments each time the function is called, which creates the possibility to use a
         * switch-case to handle the creation of each follow-up instruction. It is not mandatory to use this mechanism.
         *
         * @param instructionNumber A number which increments each call.
         * @return std::shared_ptr<AbstractInstruction> A shared pointer to the created instruction.
         */
        virtual std::shared_ptr<instruction::AbstractInstruction> createInstruction(uint8_t instructionNumber) = 0;

        /**
         * @brief Create a Trajectory object.
         * @note function is overloaded to accept a Pose object, instead of a PoseStamped
         *
         * This function creates a trajectory object for the given target pose and end effector link.
         *
         * @details The function follows these steps to create the trajectory:
         * 1. Attempts to create the trajectory using the preferred PilzPlannerId.
         * 2. If the first attempt fails, it tries to use the other Pilz planners (PTP, LIN, CIRC) to find a quick solution.
         * 3. If all Pilz planners fail, it falls back to using the OMPL planner, which takes long but is more likely to find
         *    a solution.
         *
         * @note Throws a std::runtime_error if the trajectory could not be created.
         *
         * @param targetPose The target pose for the trajectory.
         * @param endEffectorLink The name of the end effector link.
         * @param preferredPilzPlannerId The preferred planner ID for planning with pilz.
         * @param velocityScaling The scaling factor for the velocity.
         * @return moveit_msgs::msg::RobotTrajectory::SharedPtr The created trajectory.
         */
        moveit_msgs::msg::RobotTrajectory createTrajectory(
            const geometry_msgs::msg::Pose& targetPose,
            const std::string& endEffectorLink,
            const std::string& preferredPilzPlannerId,
            double velocityScaling = -1.0);
        /**
         * @brief Create a Trajectory object.
         *
         * This function creates a trajectory object for the given target pose and end effector link.
         *
         * @details The function follows these steps to create the trajectory:
         * 1. Attempts to create the trajectory using the preferred PilzPlannerId.
         * 2. If the first attempt fails, it tries to use the other Pilz planners (PTP, LIN, CIRC) to find a quick solution.
         * 3. If all Pilz planners fail, it falls back to using the OMPL planner, which takes long but is more likely to find
         *    a solution.
         *
         * @note Throws a std::runtime_error if the trajectory could not be created.
         *
         * @param targetPose The target pose for the trajectory.
         * @param endEffectorLink The name of the end effector link.
         * @param preferredPilzPlannerId The preferred planner ID for planning with pilz.
         * @param velocityScaling The scaling factor for the velocity.
         * @return moveit_msgs::msg::RobotTrajectory::SharedPtr The created trajectory.
         */
        moveit_msgs::msg::RobotTrajectory createTrajectory(
            const geometry_msgs::msg::PoseStamped& targetPose,
            const std::string& endEffectorLink,
            const std::string& preferredPilzPlannerId,
            double velocityScaling = -1.0);
        /**
         * @brief Get the pose of a end effector in the startJointValues.
         *
         * @param endEffectorLink The name of the end effector link.
         * @return geometry_msgs::msg::Pose The pose of the end effector link in the previous state.
         */
        geometry_msgs::msg::Pose getStartPose(const std::string& endEffectorLink);
        /**
         * @brief Get the goal joint tolerance of the move group.
         */
        double getGoalJointTolerance();
        /**
         * @brief Marks the pipeline as completed.
         */
        void markCompleted();

        rclcpp::Logger logger; /* The logger to be used */
    private:
        /**
         * @brief Plan a trajectory for a target pose.
         *
         * This function plans a trajectory for the given target pose and end effector link.
         *
         * @param targetPose The target pose for the trajectory.
         * @param endEffectorLink The name of the end effector link.
         * @param planningPipelineId The ID of the planning pipeline to be used.
         * @param plannerId The ID of the planner to be used.
         * @param velocityScaling The scaling factor for the velocity.
         * 
         * @return moveit_msgs::msg::RobotTrajectory::SharedPtr The planned trajectory.
         */
        moveit_msgs::msg::RobotTrajectory::SharedPtr planTrajectory(
            const geometry_msgs::msg::PoseStamped& targetPose,
            const std::string& endEffectorLink,
            const std::string& planningPipelineId,
            const std::string& plannerId,
            double velocityScaling);
        /**
         * @brief Publish a trajectory path (visualization purposes only)
         *
         * This function publishes the trajectory path of the given trajectory.
         *
         * @param trajectory The trajectory to publish.
         */
        void publishTrajectoryPath(moveit_msgs::msg::RobotTrajectory& trajectory);

        const moveit_msgs::msg::RobotState startJointValues; /* The joint values to start any planning from */
        const std::shared_ptr<state::AbstractState> state;   /* The created state of the pipeline */
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> moveGroup; /* Move group interface for planning */
        uint8_t curInstructionNumber; /* The instruction number which get incremented each time in createNextInstruction() */
        bool completed;               /* A boolean indicating if the pipeline is completed */
    };
} // namespace state_pipeline

#endif // ABSTRACTSTATEPIPELINE_HPP
