#include <GripperController.hpp>

#include <MachineStateControlNode.hpp> // Used to create the service clients.

using namespace std::chrono_literals; // Easy of use for the chrono_literals section.

GripperController::~GripperController() {}

GripperController::GripperController() : 
    logger(rclcpp::get_logger(LOGGER_NAME)), 
    signalIn(MachineStateControlNode::getInstance()->get_parameter("signal_gripper_in").as_string()), 
    signalOut(MachineStateControlNode::getInstance()->get_parameter("signal_gripper_out").as_string())
{
    this->getClient = MachineStateControlNode::getInstance()->create_client<abb_robot_msgs::srv::GetIOSignal>("/rws_client/get_io_signal");
    this->setClient = MachineStateControlNode::getInstance()->create_client<abb_robot_msgs::srv::SetIOSignal>("/rws_client/set_io_signal");
}

bool GripperController::setGripperState(bool state) { // Only 1 function to set it, because open/close functions would be almost identical.
    auto closeRequestIn = std::make_shared<abb_robot_msgs::srv::SetIOSignal::Request>();
    auto closeRequestOut = std::make_shared<abb_robot_msgs::srv::SetIOSignal::Request>();
    closeRequestIn->signal = signalIn;
    closeRequestOut->signal = signalOut;

    if (state) { // If state = true, the gripper will close by driving pressure, if false, it'll open by venting said pressure.
        closeRequestIn->value = "1";
        closeRequestOut->value = "0";
    } else {
        closeRequestIn->value = "0";
        closeRequestOut->value = "1";
    }

    // Order: First close intake, then exhaust.
    auto responseIn = setClient->async_send_request(closeRequestIn);
    auto responseOut = setClient->async_send_request(closeRequestOut);

    std::future_status statusIn = responseIn.wait_for(2s);
    std::future_status statusOut = responseOut.wait_for(2s);
    if (statusIn != std::future_status::ready || statusOut != std::future_status::ready) { // If either aren't ready after 4 seconds total, time out.
        RCLCPP_ERROR(logger,"Responses timed out after 4 seconds total.");
        return false;
    }

    std::shared_ptr<abb_robot_msgs::srv::SetIOSignal_Response> resultIn, resultOut; // Get the results from the Future reference, this function is blocking if not preceeded by a "future_status::ready" signal.
    resultIn = responseIn.get();
    resultOut = responseOut.get();

    if (resultIn->result_code != 1 || resultOut->result_code != 1) {
        RCLCPP_ERROR(logger, "Either called services returned a non-1 result code: %i, %i. The following messages were given (empty=succeeded): %s, %s", resultIn->result_code, resultOut->result_code, resultIn->message.c_str(), resultOut->message.c_str());
        return false;
    }
    // Request succeeded if all these checks succeed.

    return true; // Return true for succes.
}

int8_t GripperController::getGripperState() {
    auto requestRead = std::make_shared<abb_robot_msgs::srv::GetIOSignal::Request>();
    requestRead->signal = signalIn; // Only need to check one to know if it's open or closed, checking both can be added for extra security.

    auto requestResult = getClient->async_send_request(requestRead);

    // // Spin the machinestatecontrol node until completed, probably not the best way to do it, adapted from .
    // // Due to issues with a adaption from https://docs.ros.org/en/crystal/Tutorials/Writing-A-Simple-Cpp-Service-And-Client.html, the get() command is used in it's blocking form.
    RCLCPP_WARN(logger,"Carrying out possible freezing actions.");
    std::future_status status = requestResult.wait_for(5s);
    std::shared_ptr<abb_robot_msgs::srv::GetIOSignal_Response> response;

    if (status == std::future_status::ready) {
        response = requestResult.get();
    } else {
        RCLCPP_ERROR(logger, "Response timed out after 5 seconds.");
        return int8_t(-1); // Timed out does mean failed.
    }

    if (response->result_code != 1) {
        RCLCPP_ERROR(logger, "Service returned non-1 result code: %i, with message: %s", response->result_code, response->message.c_str());
        return int8_t(-1); // Return a failed statement.
    }

    try {
        return int8_t(std::stoi(response->value));
    } catch(const std::exception& e) { // Catch all expressions for the sake of simplicity, this atleast prevents crashing
        RCLCPP_ERROR(logger,"Issue with conversion, message was: %s", response->value.c_str());
        return int8_t(-1);
    }
}

