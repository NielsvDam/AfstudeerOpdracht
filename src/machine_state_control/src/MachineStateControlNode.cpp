#include "MachineStateControlNode.hpp"

MachineStateControlNode::~MachineStateControlNode() {}

MachineStateControlNode::MachineStateControlNode()
    : Node("machine_state_control_node", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)),
      // Create the machine control service
      controlOperationStateServer(create_service<custom_msgs::srv::MachineControl>(
          "/machine_control",
          std::bind(
              &MachineStateControlNode::controlOperationStateCallback,
              this,
              std::placeholders::_1,
              std::placeholders::_2))),
      // Create the execute trajectory action client
      executeTrajectoryClient(
          rclcpp_action::create_client<custom_msgs::action::ExecuteTrajectory>(this, "/execute_trajectory2")),
      // Create the picture service client
      pictureClient(rclcpp_action::create_client<custom_msgs::action::Picture>(this, "/take_picture")),
      // Create the path publisher
      pathPublisher(create_publisher<nav_msgs::msg::Path>("/path", 10)),
      gripperController(GripperController(get_parameter("signal_gripper_in").as_string(), get_parameter("signal_gripper_out").as_string(), create_client<abb_robot_msgs::srv::GetIOSignal>("/rws_client/get_io_signal"), create_client<abb_robot_msgs::srv::SetIOSignal>("/rws_client/set_io_signal")))
{
    RCLCPP_INFO(this->get_logger(), "Machine control service started");
}

/* static */ std::shared_ptr<MachineStateControlNode> MachineStateControlNode::getInstance()
{
    static std::shared_ptr<MachineStateControlNode> instance{new MachineStateControlNode};
    return instance;
}

void MachineStateControlNode::initialize()
{
    stateEngine.initialize();
}

rclcpp_action::Client<custom_msgs::action::ExecuteTrajectory>::SharedPtr MachineStateControlNode::
    getExecuteTrajectoryClient() const
{
    return executeTrajectoryClient;
}

rclcpp_action::Client<custom_msgs::action::Picture>::SharedPtr MachineStateControlNode::getPictureClient() const
{
    return pictureClient;
}

rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr MachineStateControlNode::getPathPublisher() const
{
    return pathPublisher;
}

GripperController& MachineStateControlNode::getGripperController()
{
    return gripperController;
}

void MachineStateControlNode::controlOperationStateCallback(
    const std::shared_ptr<custom_msgs::srv::MachineControl::Request>& request,
    const std::shared_ptr<custom_msgs::srv::MachineControl::Response>& response)
{
    RCLCPP_INFO(this->get_logger(), "Recieved a request about status.");
    if (request->command == "start")
    {
        stateEngine.setOperationState(OperationState::RUNNING);
        response->success = true;
        RCLCPP_INFO(this->get_logger(), "Set the result 'success' to true.");
    }
    else if (request->command == "pause")
    {
        stateEngine.setOperationState(OperationState::PAUSED);
        response->success = true;
    }
    else if (request->command == "stop")
    {
        stateEngine.setOperationState(OperationState::IDLE);
        response->success = true;
    }
    else
    {
        response->success = false;
        response->message = "Invalid command";
    }
}

void MachineStateControlNode::pictureResultCallback(
    const rclcpp_action::ClientGoalHandle<custom_msgs::action::Picture>::WrappedResult& result)
{
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
    {
        RCLCPP_INFO(this->get_logger(), "Object detector returned a result.");
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "object detector failed with error code %d", static_cast<int>(result.code));
        return; // TODO: error handling?
    }
    // Print the number of detected objects and unknown areas
    std::size_t detectedObjects = result.result->detected_blocks.size();
    std::size_t unknownAreas = result.result->unknown_areas.size();
    RCLCPP_INFO(
        this->get_logger(),
        "Object detector resulted in %zu objects and %zu unknown areas",
        detectedObjects,
        unknownAreas);
    // Inform the state engine about the detected objects
    stateEngine.objectsDetected(result.result->detected_blocks, result.result->unknown_areas);
}