#include <rclcpp/rclcpp.hpp>                            // rclcpp::init, rclcpp::spin, rclcpp::shutdown
#include <rclcpp/executors/multi_threaded_executor.hpp> // rclcpp::executors::MultiThreadedExecutor

#include "MachineStateControlNode.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = MachineStateControlNode::getInstance();
    node->initialize();

    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
