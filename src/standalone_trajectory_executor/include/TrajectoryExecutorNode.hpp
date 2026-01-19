#ifndef TRAJECTORYEXECUTORNODE_HPP
#define TRAJECTORYEXECUTORNODE_HPP

#define PUBLISH_CURRENT // Comment this out if you do not want to publish the currently executed trajectory.

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <custom_msgs/action/execute_trajectory.hpp>
#include <mutex>
#include <condition_variable>
#include <vector>
#include <string>

#ifdef PUBLISH_CURRENT
#include <moveit_msgs/msg/display_trajectory.hpp>
#endif

/**
 * @class TrajectoryExecutorNode
 * @brief A ROS2 node for executing joint trajectory actions.
 *
 * This class provides functionality for executing joint trajectories. It includes an action server for receiving and
 * executing trajectory goals, a publisher for sending trajectories to the controller, and a subscription to the joint states
 * topic to monitor the robot's state. Monitoring the robot state is used to check if a target state is reached.
 *
 * @note This node was created because MoveIt itself cannot execute and plan simultaneously, which we concluded after various
 * carefull experiments. By reviewing MoveIt's source code, we figured it can't do this in due to how the internal state is
 * managed. To achieve this functionality, this node was developed to handle the execution of trajectories while MoveIt is
 * planning.
 *
 * @details
 * - TROShe action server is created using the action name specified by the "action_name" parameter from the yaml file.
 * - The joint states subscription listens to the "/joint_states" topic, which gets checked against trajectory end positions.
 * - The trajectory publisher publishes to the topic specified by the "controller" parameter from the yaml file.
 */
class TrajectoryExecutorNode : public rclcpp::Node
{
public:
    virtual ~TrajectoryExecutorNode();
    /**
     * @brief Construct a new TrajectoryExecutorNode object.
     *
     * This constructor initializes the TrajectoryExecutorNode by:
     * - Creating an action server for executing trajectories.
     * - Subscribing to the joint states topic to keep track of the joint state.
     * - Creating a publisher for sending trajectories to the controller.
     *
     * @details
     * - The action server is created using the action name specified by the "action_name" parameter from the
     *   'executor_params.yaml' file.
     * - The joint states subscription listens to the "/joint_states" topic, which gets checked against trajectory
     *   endpositions.
     * - The trajectory publisher publishes to the topic specified by the "controller" parameter from the
     *   'executor_params.yaml' file.
     */
    TrajectoryExecutorNode();
private:
    /**
     * @brief Handles the reception of a new goal for trajectory execution.
     *
     * This function is called when a new goal is received by the action server. It performs several checks to validate the
     * goal before accepting or rejecting it.
     *
     * @param uuid The unique identifier of the goal. We don't use it in this implementation.
     * @param goal A shared pointer to the goal message containing the trajectory to be executed.
     *
     * @return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE if the goal is valid and can be executed,
     *         rclcpp_action::GoalResponse::REJECT otherwise.
     *
     * @pre \p goal its trajectory must not be empty.
     * @pre \p goal its joint_tolerance must be greater than zero. Zero is a unrealistic value and likely caused by an error
     *              since that is the default value when it is not specified by the action caller.
     * @pre \p goal its trajectory must start at the current state of the robot within the specified tolerance.
     */
    rclcpp_action::GoalResponse handleActionGoal(
        const rclcpp_action::GoalUUID& uuid,
        const std::shared_ptr<const custom_msgs::action::ExecuteTrajectory::Goal> goal);

    /**
     * @brief Handles the cancellation of an action goal.
     *
     * It currently rejects all cancel requests because there is no caller that benefits from this feature.
     *
     * @param goal_handle A shared pointer to the goal handle of the action to be cancelled.
     * @return rclcpp_action::CancelResponse::REJECT as the cancel logic is not implemented.
     */
    rclcpp_action::CancelResponse handleActionCancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<custom_msgs::action::ExecuteTrajectory>> goal_handle);

    /**
     * @brief Handles the acceptance of an action goal.
     *
     * This function is called when an action goal is accepted and forwards it to the executeAction function. Which gets
     * executed in a separate thread.
     *
     * @param goal_handle A shared pointer to the goal handle of the accepted action.
     */
    void handleActionAccepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<custom_msgs::action::ExecuteTrajectory>> goal_handle);

    /**
     * @brief Executes the trajectory action.
     *
     * This function publishes the trajectory to the controller, then waits for the final joint state to match the target
     * joint state within a specified tolerance. Once the target joint state is reached it publishes the result to the action
     * handle.
     *
     * @note A timeout mechanism should not implemented yet. The JointTrajectory.msg holds this duration per point, it should
     * be implemented in the future.
     *
     * @param goal_handle A shared pointer to the goal handle of the trajectory execution action.
     */
    void executeAction(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<custom_msgs::action::ExecuteTrajectory>> goal_handle);

    /**
     * @brief Callback function to handle incoming joint state messages.
     *
     * This function is called when a new joint state message is received. It updates the current joint state with the
     * received message and notifies any waiting threads for an update.
     *
     * @param msg A shared pointer to the received joint state message.
     *
     * @throws std::runtime_error If the received joint state message is null.
     */
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

    rclcpp_action::Server<custom_msgs::action::ExecuteTrajectory>::SharedPtr executeTrajectoryAction;    /* The action server
                                                                                                            for executing
                                                                                                            trajectories.*/
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointStatesSubscription;               /* The subscription
                                                                                                            to the joint
                                                                                                            states topic. */
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectoryToControllerPublisher; /* The publisher for
                                                                                                            sending
                                                                                                            trajectories to
                                                                                                            the controller.*/
    std::mutex currentStateMutex;                   /* A mutex to protect the current joint state. */
    std::condition_variable jointStateCv;           /* A condition variable to notify waiting threads
                                                       for an update on the joint state. */
    sensor_msgs::msg::JointState currentJointState; /* The current joint state of the robot. */

    #ifdef PUBLISH_CURRENT
    rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr currentPathPublisher;
    // Addition of a Trajectory publisher for both debugging and path-execution monitoring. (I.E, this will publish the currently executing path only.)
    #endif

};

#endif // TRAJECTORYEXECUTORNODE_HPP
