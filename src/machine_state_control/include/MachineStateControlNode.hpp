#ifndef CONTROLNODE_HPP
#define CONTROLNODE_HPP

#include <memory>                            // std::shared_ptr
#include <rclcpp/rclcpp.hpp>                 // rclcpp::Node
#include <rclcpp_action/rclcpp_action.hpp>   // rclcpp_action::Client
#include <nav_msgs/msg/path.hpp>             // nav_msgs::msg::Path

#include <custom_msgs/srv/machine_control.hpp>       // custom_msgs::srv::MachineControl
#include <custom_msgs/action/execute_trajectory.hpp> // custom_msgs::action::ExecuteTrajectory
#include <custom_msgs/action/picture.hpp>            // custom_msgs::action::Picture
#include "StateEngine/StateExecutionHandler.hpp"     // state_engine::StateExecutionHandler
#include "StateEngine/StateEngine.hpp"               // state_engine::StateEngine
#include "OperationStateEnum.hpp"                    // OperationState

/**
 * @class MachineStateControlNode
 * @brief The MachineStateControlNode is the main node of the machine state control.
 *
 * The MachineStateControlNode is responsible for the initialization of the state engine and the handling of the machine control service.
 *
 * @note The MachineStateControlNode is a singleton.
 */
class MachineStateControlNode : public rclcpp::Node
{
public:
    virtual ~MachineStateControlNode();
    MachineStateControlNode(MachineStateControlNode const&) = delete;            /* the node should never be copied */
    MachineStateControlNode& operator=(MachineStateControlNode const&) = delete; /* the node should never be copied */
    /**
     * @brief Initializes the MachineStateControlNode.
     */
    void initialize();
    /**
     * @brief Get the instance of the MachineStateControlNode.
     *
     * @return std::shared_ptr<MachineStateControlNode> The instance of the MachineStateControlNode.
     */
    static std::shared_ptr<MachineStateControlNode> getInstance();
    /**
     * @brief Get the Execute Trajectory Client.
     *
     * @return rclcpp_action::Client<custom_msgs::action::ExecuteTrajectory>::SharedPtr The Execute Trajectory Client.
     */
    rclcpp_action::Client<custom_msgs::action::ExecuteTrajectory>::SharedPtr getExecuteTrajectoryClient() const;
    /**
     * @brief Get the Picture Client.
     *
     * @return rclcpp_action::Client<custom_msgs::action::Picture>::SharedPtr  The Picture Client.
     */
    rclcpp_action::Client<custom_msgs::action::Picture>::SharedPtr getPictureClient() const;
    /**
     * @brief Callback for the Execute Trajectory Action result.
     *
     * @param goal_handle The goal handle of the action.
     */
    void pictureResultCallback(const rclcpp_action::ClientGoalHandle<custom_msgs::action::Picture>::WrappedResult& result);
    /**
     * @brief Get the Path Publisher.
     *
     * @return rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr The Path Publisher.
     */
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr getPathPublisher() const;
private:
    /**
     * @brief Constructs a new MachineStateControlNode object.
     */
    MachineStateControlNode();

    /**
     * @brief Callback for the machine control service.
     *
     * @param request The request for the service.
     * @param response The response for the service.
     */
    void controlOperationStateCallback(
        const std::shared_ptr<custom_msgs::srv::MachineControl::Request>& request,
        const std::shared_ptr<custom_msgs::srv::MachineControl::Response>& response);

    state_engine::StateEngine stateEngine; /* The state engine of the machine control. */

    rclcpp::Service<custom_msgs::srv::MachineControl>::SharedPtr controlOperationStateServer; /* The machine control
                                                                                                 service */
    rclcpp_action::Client<custom_msgs::action::ExecuteTrajectory>::SharedPtr executeTrajectoryClient; /* The
                                                                                                         ExecuteTrajectory
                                                                                                         Client */
    rclcpp_action::Client<custom_msgs::action::Picture>::SharedPtr pictureClient; /* The picture service client */
    // path publisher
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pathPublisher; /* The path publisher */
};

#endif // CONTROLNODE_HPP
