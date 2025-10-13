#ifndef HUMANMACHINEINTERFACE_HPP
#define HUMANMACHINEINTERFACE_HPP

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
#include <QGroupBox>
#include <QToolBox>
#include <tf2/LinearMath/Quaternion.h>
#include <thread>
#include <cmath>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <functional>
#include "ConfigurationNode.hpp"
#include <list>

#include "config/Config.hpp"
#include "config/ParameterSection.hpp"

enum MachineState
{
    Idle,
    Running,
    Paused
};

class HumanMachineInterface : public QWidget
{
public:
    explicit HumanMachineInterface(std::shared_ptr<ConfigurationNode> configurationNode);
    virtual ~HumanMachineInterface();
private:
    QVBoxLayout* createVBoxLayout();
    QHBoxLayout* createHBoxLayout();
    QLabel* createLabel();
    QLabel* createValueLabel();
    QSlider* createSlider();
    QPushButton* createButton(const QString& text);
    QGroupBox* createGroupBox(const QString& title);
    QToolBox* createToolBox();
    QWidget* createWidget();

    void addSlider(QVBoxLayout* layout, const std::string& packageName, std::shared_ptr<ParameterSection> parameter);

    void updateButtonStates();
    void applyStyles();

    void onStartClicked();
    void onPauseClicked();
    void onParkClicked();
    void onUnparkClicked();
    void onSaveClicked();

    void publishInitialParameters();

    std::shared_ptr<ConfigurationNode> configurationNode;

    Config config;

    // Since QT works with pointers, we are must facilitate the storage.
    std::list<QVBoxLayout> sliderLayouts;
    std::list<QHBoxLayout> labelLayouts;
    std::list<QLabel> labels;
    std::list<QLabel> valueLabels;
    std::list<QSlider> sliderWidgets;
    std::list<QPushButton> buttons;
    std::list<QGroupBox> groupBoxes;
    std::list<QToolBox> toolBoxes;
    std::list<QWidget> widgets;

    MachineState state;
    QPushButton* startButton;
    QPushButton* pauseButton;
    QPushButton* parkButton;
    QPushButton* unparkButton;
    QPushButton* saveButton;
};

#endif // HUMANMACHINEINTERFACE_HPP
