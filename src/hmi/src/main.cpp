#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <QApplication>
#include <QWidget>
#include <QSlider>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <tf2/LinearMath/Quaternion.h>
#include <thread>
#include <cmath>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <csignal>
#include "HumanMachineInterface.hpp"
#include "ConfigurationNode.hpp"

void signalHandler(int signum) {
    std::cout << "Interrupt signal (" << signum << ") received.\n";
    rclcpp::shutdown();
    QApplication::quit();
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    QApplication qtApp(argc, argv);

    auto configNode = std::make_shared<ConfigurationNode>();

    std::thread rosThread([configNode]() {
        rclcpp::spin(configNode);
    });

    HumanMachineInterface window(configNode);
    window.show();

    // Register signal handler
    signal(SIGINT, signalHandler);

    int result = QApplication::exec();

    rclcpp::shutdown();
    rosThread.join();

    return result;
}