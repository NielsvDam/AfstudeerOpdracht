#ifndef CONFIGURATIONNODE_HPP
#define CONFIGURATIONNODE_HPP


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


class ConfigurationNode : public rclcpp::Node
{
    public:
        ConfigurationNode();
        virtual ~ConfigurationNode();

        // control functions
        void start();
        void pause();
        void stop();
        void park();
        void unPark();
        void setParameterOnNode(const std::string& target_node_name, const rclcpp::Parameter& parameter);
    private:
        void publishCameraTransform(double x, double y, double z, double roll, double pitch, double yaw);
};


#endif // CONFIGURATIONNODE_HPP
