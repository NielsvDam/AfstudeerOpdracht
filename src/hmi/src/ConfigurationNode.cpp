#include "ConfigurationNode.hpp"
#include <custom_msgs/srv/machine_control.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

ConfigurationNode::~ConfigurationNode() {}

ConfigurationNode::ConfigurationNode() : Node("configuration_node") {}

// control functions
void ConfigurationNode::start()
{
    auto client = this->create_client<custom_msgs::srv::MachineControl>("machine_control");
    while (!client->wait_for_service(std::chrono::seconds(1)))
    {
        RCLCPP_WARN(this->get_logger(), "Waiting for the machine control service to be available...");
    }
    auto request = std::make_shared<custom_msgs::srv::MachineControl::Request>();
    request->command = "start";
    auto result = client->async_send_request(request);
    result.wait();
    if (result.get()->success)
    {
        RCLCPP_INFO(this->get_logger(), "Machine control START successfully");
    }
    else
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "Failed to start machine control service. Message: %s",
            result.get()->message.c_str());
    }
}

void ConfigurationNode::pause()
{
    auto client = this->create_client<custom_msgs::srv::MachineControl>("machine_control");
    while (!client->wait_for_service(std::chrono::seconds(1)))
    {
        RCLCPP_WARN(this->get_logger(), "Waiting for the machine control service to be available...");
    }
    auto request = std::make_shared<custom_msgs::srv::MachineControl::Request>();
    request->command = "pause";
    auto result = client->async_send_request(request);
    result.wait();
    if (result.get()->success)
    {
        RCLCPP_INFO(this->get_logger(), "Machine control PAUSED successfully");
    }
    else
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "Failed to pause machine control service. Message: %s",
            result.get()->message.c_str());
    }
}

void ConfigurationNode::stop()
{
    auto client = this->create_client<custom_msgs::srv::MachineControl>("machine_control");
    while (!client->wait_for_service(std::chrono::seconds(1)))
    {
        RCLCPP_WARN(this->get_logger(), "Waiting for the machine control service to be available...");
    }
    auto request = std::make_shared<custom_msgs::srv::MachineControl::Request>();
    request->command = "stop";
    auto result = client->async_send_request(request);
    result.wait();
    if (result.get()->success)
    {
        RCLCPP_INFO(this->get_logger(), "Machine control STOPPED successfully");
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to stop machine control. Message: %s", result.get()->message.c_str());
    }
}

void ConfigurationNode::park()
{
    auto action_client = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
        this,
        "/rv5as_controller/follow_joint_trajectory");

    // Wait for the action server to be available
    while (!action_client->wait_for_action_server(std::chrono::seconds(1)))
    {
        RCLCPP_WARN(this->get_logger(), "Waiting for action server /rv5as_controller/follow_joint_trajectory...");
    }

    // First goal
    control_msgs::action::FollowJointTrajectory::Goal goal1;
    goal1.trajectory.joint_names =
        {"rv5as_joint_1", "rv5as_joint_2", "rv5as_joint_3", "rv5as_joint_4", "rv5as_joint_5", "rv5as_joint_6"};
    trajectory_msgs::msg::JointTrajectoryPoint point1;
    point1.positions = {1.57079, 0.331612, 1.815142, 0.05235, 1.012290, 0.0};
    point1.velocities = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
    point1.time_from_start = rclcpp::Duration::from_seconds(4.0);
    goal1.trajectory.points.push_back(point1);

    auto future_goal_handle1 = action_client->async_send_goal(goal1);
    future_goal_handle1.wait();
    const auto& goal_handle1 = future_goal_handle1.get();
    if (goal_handle1)
    {
        auto future_result1 = action_client->async_get_result(goal_handle1);
        future_result1.wait();
        RCLCPP_INFO(this->get_logger(), "First park trajectory sent");
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "First goal was rejected");
        return;
    }

    // Second goal
    control_msgs::action::FollowJointTrajectory::Goal goal2;
    goal2.trajectory.joint_names =
        {"rv5as_joint_1", "rv5as_joint_2", "rv5as_joint_3", "rv5as_joint_4", "rv5as_joint_5", "rv5as_joint_6"};
    trajectory_msgs::msg::JointTrajectoryPoint point2;
    point2.positions = {-0.122173, -0.610865, 2.565634, 0.0, 1.151917, -0.191986};
    point2.velocities = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
    point2.time_from_start = rclcpp::Duration::from_seconds(4.0);
    goal2.trajectory.points.push_back(point2);

    auto future_goal_handle2 = action_client->async_send_goal(goal2);
    future_goal_handle2.wait();
    const auto& goal_handle2 = future_goal_handle2.get();
    if (goal_handle2)
    {
        auto future_result2 = action_client->async_get_result(goal_handle2);
        future_result2.wait();
        RCLCPP_INFO(this->get_logger(), "Second park trajectory sent");
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Second goal was rejected");
        return;
    }

    // Third goal
    control_msgs::action::FollowJointTrajectory::Goal goal3;
    goal3.trajectory.joint_names =
        {"rv5as_joint_1", "rv5as_joint_2", "rv5as_joint_3", "rv5as_joint_4", "rv5as_joint_5", "rv5as_joint_6"};
    trajectory_msgs::msg::JointTrajectoryPoint point3;
    point3.positions = {-0.1396263402, -0.1570796327, 2.6005405855, -0.0698131701, 1.1868238914, -0.0698131701};
    point3.velocities = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
    point3.time_from_start = rclcpp::Duration::from_seconds(4.0);
    goal3.trajectory.points.push_back(point3);
    auto future_goal_handle3 = action_client->async_send_goal(goal3);
    future_goal_handle3.wait();
    const auto& goal_handle3 = future_goal_handle3.get();
    if (goal_handle3)
    {
        auto future_result3 = action_client->async_get_result(goal_handle3);
        future_result3.wait();
        RCLCPP_INFO(this->get_logger(), "Third park trajectory sent");
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Third goal was rejected");
        return;
    }
}

void ConfigurationNode::unPark()
{
    auto action_client = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
        this,
        "/rv5as_controller/follow_joint_trajectory");

    // Wait for the action server to be available
    while (!action_client->wait_for_action_server(std::chrono::seconds(1)))
    {
        RCLCPP_WARN(this->get_logger(), "Waiting for action server /rv5as_controller/follow_joint_trajectory...");
    }

    // First goal
    control_msgs::action::FollowJointTrajectory::Goal goal1;
    goal1.trajectory.joint_names =
        {"rv5as_joint_1", "rv5as_joint_2", "rv5as_joint_3", "rv5as_joint_4", "rv5as_joint_5", "rv5as_joint_6"};
    trajectory_msgs::msg::JointTrajectoryPoint point1;
    point1.positions = {-0.122173, -0.610865, 2.565634, 0.0, 1.151917, -0.191986};
    point1.velocities = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
    point1.time_from_start = rclcpp::Duration::from_seconds(4.0);
    goal1.trajectory.points.push_back(point1);

    auto future_goal_handle1 = action_client->async_send_goal(goal1);
    future_goal_handle1.wait();
    const auto& goal_handle1 = future_goal_handle1.get();
    if (goal_handle1)
    {
        auto future_result1 = action_client->async_get_result(goal_handle1);
        future_result1.wait();
        RCLCPP_INFO(this->get_logger(), "First park trajectory sent");
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "First goal was rejected");
        return;
    }

    // Second goal
    control_msgs::action::FollowJointTrajectory::Goal goal2;
    goal2.trajectory.joint_names =
        {"rv5as_joint_1", "rv5as_joint_2", "rv5as_joint_3", "rv5as_joint_4", "rv5as_joint_5", "rv5as_joint_6"};
    trajectory_msgs::msg::JointTrajectoryPoint point2;
    point2.positions = {1.57079, 0.331612, 1.815142, 0.05235, 1.012290, 0.0};
    point2.velocities = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
    point2.time_from_start = rclcpp::Duration::from_seconds(4.0);
    goal2.trajectory.points.push_back(point2);

    auto future_goal_handle2 = action_client->async_send_goal(goal2);
    future_goal_handle2.wait();
    const auto& goal_handle2 = future_goal_handle2.get();
    if (goal_handle2)
    {
        auto future_result2 = action_client->async_get_result(goal_handle2);
        future_result2.wait();
        RCLCPP_INFO(this->get_logger(), "Second park trajectory sent");
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Second goal was rejected");
        return;
    }
}

void ConfigurationNode::setParameterOnNode(const std::string& targetNodeName, const rclcpp::Parameter& parameter)
{
    auto parameter_client = std::make_shared<rclcpp::AsyncParametersClient>(this, targetNodeName);

    // Wait for the target node's parameter service to become available
    if (!parameter_client->wait_for_service(std::chrono::seconds(5)))
    {
        RCLCPP_ERROR(this->get_logger(), "Timeout: Could not connect to parameter service of %s", targetNodeName.c_str());
        return;
    }

    // Set the parameter asynchronously with a callback
    parameter_client->set_parameters(
        {parameter},
        [this, targetNodeName, parameter](
            const std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>& result) {
            try
            {
                auto response = result.get();
                if (!response.empty() && response.front().successful)
                {
                    RCLCPP_INFO(
                        this->get_logger(),
                        "Successfully set parameter on %s. Parameter name: %s, value: %s",
                        targetNodeName.c_str(),
                        parameter.get_name().c_str(),
                        parameter.value_to_string().c_str());
                }
                else
                {
                    RCLCPP_ERROR(
                        this->get_logger(),
                        "Failed to set parameter on %s. Parameter name: %s, value: %s",
                        targetNodeName.c_str(),
                        parameter.get_name().c_str(),
                        parameter.value_to_string().c_str());
                }
            }
            catch (const std::exception& e)
            {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "Exception while setting parameter on %s: %s",
                    targetNodeName.c_str(),
                    e.what());
            }
        });
}