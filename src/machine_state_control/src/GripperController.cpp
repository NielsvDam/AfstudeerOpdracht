#include <GripperController.hpp>

#include <MachineStateControlNode.hpp> // Used to create the service clients.

using namespace std::chrono_literals; // Easy of use for the chrono_literals section.

GripperController::~GripperController() {};

GripperController::GripperController(std::string signal_in, std::string signal_out, rclcpp::Client<abb_robot_msgs::srv::GetIOSignal>::SharedPtr get_client, rclcpp::Client<abb_robot_msgs::srv::SetIOSignal>::SharedPtr set_client) : logger(rclcpp::get_logger(GRIPPER_LOGGER_NAME))
{
    signalIn = signal_in;
    signalOut = signal_out;

    getClient = get_client;
    setClient = set_client;

    RCLCPP_INFO(logger, "GripperController created succesfully.");
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
    auto requestReadIn = std::make_shared<abb_robot_msgs::srv::GetIOSignal::Request>();
    auto requestReadOut = std::make_shared<abb_robot_msgs::srv::GetIOSignal::Request>();
    requestReadIn->signal = signalIn; // Check In to determine open/closed state.
    requestReadOut->signal = signalOut; // Check out to test and prevent error-states from occuring.

    auto requestResultIn = getClient->async_send_request(requestReadIn);
    auto requestResultOut = getClient->async_send_request(requestReadOut);

    // // Due to issues with a adaption from https://docs.ros.org/en/crystal/Tutorials/Writing-A-Simple-Cpp-Service-And-Client.html, the get() command is used in it's blocking form (spin_until_finished flag)
    RCLCPP_WARN(logger,"Carrying out possible freezing actions.");
    std::future_status statusIn = requestResultIn.wait_for(2s);
    std::future_status statusOut = requestResultOut.wait_for(2s);
    std::shared_ptr<abb_robot_msgs::srv::GetIOSignal_Response> responseIn;
    std::shared_ptr<abb_robot_msgs::srv::GetIOSignal_Response> responseOut;

    if (statusIn == std::future_status::ready && statusOut == std::future_status::ready) {
        responseIn = requestResultIn.get();
        responseOut = requestResultOut.get();
    } else {
        RCLCPP_ERROR(logger, "Response timed out after 5 seconds.");
        return int8_t(-1); // Timed out does mean failed.
    }

    if (responseIn->result_code != 1 || responseOut->result_code != 1) {
        RCLCPP_ERROR(logger, "Service returned non-1 result code: %i, %i; With messages: %s; %s", responseIn->result_code, responseOut->result_code, responseIn->message.c_str(), responseOut->message.c_str());
        return int8_t(-1); // Return a failed statement.
    }

    if(!((responseIn->value == "0" && responseOut->value == "1") || (responseIn->value == "1" && responseOut->value == "0"))) {
        RCLCPP_ERROR(logger, "Gripper in errorstate! Fixes itself next setGripperState command.");
        return int8_t(2); // Return a 2 for a succesful get, but errorstate in the actual gripper IO.
    }

    try {
        return int8_t(std::stoi(responseIn->value));
    } catch(const std::exception& e) { // Catch all expressions for the sake of simplicity, this atleast prevents crashing
        RCLCPP_ERROR(logger,"Issue with conversion, message was: %s", responseIn->value.c_str());
        return int8_t(-1);
    }
}

