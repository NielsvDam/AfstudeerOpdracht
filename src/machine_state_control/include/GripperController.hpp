#ifndef GRIPPERCONTROL_HPP // Make sure this isn't ran multiple times through file includes.
#define GRIPPERCONTROL_HPP

#include <memory> // std::shared_ptr access.
#include <rclcpp/rclcpp.hpp> // Access to clients & logger name.
#include <chrono> // Used for Future (object) statements.

// 2 External includes to get the IO controller services known. TODO : Replace these by a 
#include <abb_robot_msgs/srv/get_io_signal.hpp>
#include <abb_robot_msgs/srv/set_io_signal.hpp>

// Configuration for Logger etc. #define throws a error due to string usage, so:
static const char GRIPPER_LOGGER_NAME[] = "GripperController"; // Solution comes with aid from: https://forums.codeguru.com/showthread.php?48598-Is-it-bad-coding-practice-to-use-defines-for-string-literals. 

/**
 * @class GripperController
 * @brief The object used to control the gripper in the statemachine. This way it is (partially) robot-independent, only needing this object to be changed with different IO controllers.
 * 
 * 
 * @note The GripperController is not a singleton (like the MachineStateControlNode), but is contained within it.
 */
class GripperController
{
public:
    /**
     *  @brief Create the GripperController, intended to only be carried out for the MSC-Node.
     * 
     *  @warning Really long constructor, in order to carry out a (last-minute) fix for self-referencing the MachineStateControlNode. Makes for some ugly codelines, sorry!
     */
    GripperController(std::string signal_in, std::string signal_out, rclcpp::Client<abb_robot_msgs::srv::GetIOSignal>::SharedPtr get_client, rclcpp::Client<abb_robot_msgs::srv::SetIOSignal>::SharedPtr set_client);
    virtual ~GripperController(); // Deconstructor.

    /**
     * @brief Set the gripper to closed/open (true/false).
     * 
     * @return bool Returns a bool to say if it worked or failed. (Same state as request = worked.)
     */
    bool setGripperState(bool state);

    /**
     * @brief Get the current state of the gripper.
     * 
     *@ @return Returns a 0/1 for open/closed, -1 if failed.
     */
    int8_t getGripperState();

private:
    rclcpp::Logger logger; /* The logger entity/object to write output messages to. */

    rclcpp::Client<abb_robot_msgs::srv::GetIOSignal>::SharedPtr getClient; /* Service client for getting IO state. */

    rclcpp::Client<abb_robot_msgs::srv::SetIOSignal>::SharedPtr setClient; /* Service client for setting IO state. */

    std::string signalIn; /* Intake signal name, set from params for statemachine. */
    std::string signalOut; /* Exhaust signal name, set from params for statemachine. */

};

#endif