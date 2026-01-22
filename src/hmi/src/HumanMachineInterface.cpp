#include "HumanMachineInterface.hpp"

HumanMachineInterface::~HumanMachineInterface() {}

HumanMachineInterface::HumanMachineInterface(std::shared_ptr<ConfigurationNode> configurationNode)
    : configurationNode(configurationNode), // NOLINT (performance-unnecessary-value-param)
      state(Idle),
      startButton(createButton("Start")),
      pauseButton(createButton("Pause")),
      parkButton(createButton("Park")),
      unparkButton(createButton("Unpark")),
      saveButton(createButton("Save"))
{
    // Set the initial size of the main window
    resize(400, 800);

    // Create main layout
    QVBoxLayout* mainLayout = createVBoxLayout();

    // Create settings layout
    QVBoxLayout* settingsLayout = createVBoxLayout();

    // Create control layout
    QHBoxLayout* controlLayout = createHBoxLayout();

    // Create a toolbox for collapsible sections
    QToolBox* slidersToolBox = createToolBox();

    for (auto& package : config.getPackageSections())
    {
        QVBoxLayout* sliderSection = createVBoxLayout();
        for (auto& parameter : package.getParameters())
        {
            addSlider(sliderSection, package.getName(), parameter);
        }
        QWidget* additionalControlsWidget = createWidget();
        additionalControlsWidget->setLayout(sliderSection);
        slidersToolBox->addItem(additionalControlsWidget, QString::fromStdString(package.getName()));

        // Add toolbox to the settings layout
        settingsLayout->addWidget(slidersToolBox);
    }

    // Create a separate button for saving settings
    connect(saveButton, &QPushButton::clicked, this, &HumanMachineInterface::onSaveClicked);
    settingsLayout->addWidget(saveButton);

    QGroupBox* settingsGroupBox = createGroupBox("System tuning");
    settingsGroupBox->setLayout(settingsLayout);

    // Create a group box for machine control buttons
    connect(startButton, &QPushButton::clicked, this, &HumanMachineInterface::onStartClicked);
    connect(pauseButton, &QPushButton::clicked, this, &HumanMachineInterface::onPauseClicked);
    connect(parkButton, &QPushButton::clicked, this, &HumanMachineInterface::onParkClicked);
    connect(unparkButton, &QPushButton::clicked, this, &HumanMachineInterface::onUnparkClicked);
    controlLayout->addWidget(startButton);
    controlLayout->addWidget(pauseButton);
    controlLayout->addWidget(parkButton);
    controlLayout->addWidget(unparkButton);

    QGroupBox* controlGroupBox = createGroupBox("Machine Control");
    controlGroupBox->setLayout(controlLayout);

    // Add settings group box and control group box to the main layout
    mainLayout->addWidget(settingsGroupBox);
    mainLayout->addWidget(controlGroupBox);
    setLayout(mainLayout);

    // Apply stylesheets for visual appearance
    applyStyles();

    updateButtonStates();

    publishInitialParameters();
}

QVBoxLayout* HumanMachineInterface::createVBoxLayout()
{
    sliderLayouts.emplace_back();
    return &sliderLayouts.back();
}

QHBoxLayout* HumanMachineInterface::createHBoxLayout()
{
    labelLayouts.emplace_back();
    return &labelLayouts.back();
}

QLabel* HumanMachineInterface::createLabel()
{
    labels.emplace_back();
    return &labels.back();
}

QLabel* HumanMachineInterface::createValueLabel()
{
    valueLabels.emplace_back("0.00");
    return &valueLabels.back();
}

QSlider* HumanMachineInterface::createSlider()
{
    sliderWidgets.emplace_back(Qt::Horizontal);
    return &sliderWidgets.back();
}

QPushButton* HumanMachineInterface::createButton(const QString& text)
{
    buttons.emplace_back(text);
    return &buttons.back();
}

QGroupBox* HumanMachineInterface::createGroupBox(const QString& title)
{
    groupBoxes.emplace_back(title);
    return &groupBoxes.back();
}

QToolBox* HumanMachineInterface::createToolBox()
{
    toolBoxes.emplace_back();
    return &toolBoxes.back();
}

QWidget* HumanMachineInterface::createWidget()
{
    widgets.emplace_back();
    return &widgets.back();
}

void HumanMachineInterface::addSlider(
    QVBoxLayout* layout,
    const std::string& packageName,
    // NOLINTNEXTLINE (performance-unnecessary-value-param) // Its not possible due to async callback
    std::shared_ptr<ParameterSection> parameter)
{
    QVBoxLayout* sliderLayout = createVBoxLayout();
    QHBoxLayout* labelLayout = createHBoxLayout();
    QLabel* label = createLabel();
    QLabel* valueLabel = createValueLabel();
    QSlider* slider = createSlider();

    label->setText(QString::fromStdString(parameter->getName()));
    labelLayout->addWidget(label);
    labelLayout->addStretch();
    labelLayout->addWidget(valueLabel);

    double factor = parameter->getStep();
    switch (parameter->getType())
    {
        case BOOL:
        {
            slider->setRange(0, 1);
            slider->setValue(static_cast<int>(parameter->getAsBool()));
            slider->setTickInterval(1);
            connect(slider, &QSlider::valueChanged, this, [this, valueLabel, packageName, parameter](int value) {
                bool result = value > 0.5;
                parameter->setValue(result);
                valueLabel->setText(QString::fromStdString(parameter->getValueAsString()));
                auto rosParam = rclcpp::Parameter(parameter->getName(), result);
                configurationNode->setParameterOnNode(packageName, rosParam);
            });
            break;
        }
        case DOUBLE:
        {
            slider->setRange(static_cast<int>(parameter->getMin() / factor), static_cast<int>(parameter->getMax() / factor));
            slider->setValue(static_cast<int>(parameter->getAsDouble() / factor));
            slider->setTickInterval((slider->maximum() - slider->minimum()) / 100); // Set tick interval to create 100 ticks
            connect(slider, &QSlider::valueChanged, this, [this, valueLabel, packageName, parameter, factor](int value) {
                double result = static_cast<double>(value) * factor;
                parameter->setValue(result);
                valueLabel->setText(QString::fromStdString(parameter->getValueAsString()));
                auto rosParam = rclcpp::Parameter(parameter->getName(), result);
                configurationNode->setParameterOnNode(packageName, rosParam);
            });
            break;
        }
        case INT:
        {
            slider->setRange(static_cast<int>(parameter->getMin() / factor), static_cast<int>(parameter->getMax() / factor));
            slider->setValue(parameter->getAsInt());
            slider->setTickInterval((slider->maximum() - slider->minimum()) / 100); // Set tick interval to create 100 ticks
            connect(slider, &QSlider::valueChanged, this, [this, valueLabel, packageName, parameter, factor](int value) {
                int result = static_cast<int>(value * factor);
                parameter->setValue(result);
                valueLabel->setText(QString::fromStdString(parameter->getValueAsString()));
                auto rosParam = rclcpp::Parameter(parameter->getName(), result);
                configurationNode->setParameterOnNode(packageName, rosParam);
            });
            break;
        }
        case UNSIGNED_INT:
        {
            slider->setRange(static_cast<int>(parameter->getMin() / factor), static_cast<int>(parameter->getMax() / factor));
            slider->setValue(static_cast<int>(parameter->getAsUnsignedInt()));
            slider->setTickInterval((slider->maximum() - slider->minimum()) / 100); // Set tick interval to create 100 ticks
            connect(slider, &QSlider::valueChanged, this, [this, valueLabel, packageName, parameter, factor](int value) {
                value = static_cast<int>(value * factor);
                unsigned int result = value;
                parameter->setValue(result);
                valueLabel->setText(QString::fromStdString(parameter->getValueAsString()));
                auto rosParam = rclcpp::Parameter(parameter->getName(), value); // can't use unsigned int because its
                                                                                // ambiguous
                configurationNode->setParameterOnNode(packageName, rosParam);
            });
            break;
        }
        default:
        {
            break;
        }
    }
    slider->setSingleStep(1); // always step by 1 since the slider is in integer values
    valueLabel->setText(QString::fromStdString(parameter->getValueAsString()));

    sliderLayout->addLayout(labelLayout);
    sliderLayout->addWidget(slider);
    // Add stripes/dots below the slider
    slider->setTickPosition(QSlider::TicksBelow);

    layout->addLayout(sliderLayout);
}

void HumanMachineInterface::publishInitialParameters()
{
    for (auto& package : config.getPackageSections())
    {
        for (auto& parameter : package.getParameters())
        {
            rclcpp::Parameter rosParam;
            switch (parameter->getType())
            {
                case BOOL:
                    rosParam = rclcpp::Parameter(parameter->getName(), parameter->getAsBool());
                    break;
                case DOUBLE:
                    rosParam = rclcpp::Parameter(parameter->getName(), parameter->getAsDouble());
                    break;
                case INT:
                    rosParam = rclcpp::Parameter(parameter->getName(), parameter->getAsInt());
                    break;
                case UNSIGNED_INT:
                    // unfortunately can't use unsigned int because its ambiguous
                    rosParam = rclcpp::Parameter(parameter->getName(), static_cast<int>(parameter->getAsUnsignedInt()));
                    break;
                default:
                    break;
            }
            configurationNode->setParameterOnNode(package.getName(), rosParam);
        }
    }
}

void HumanMachineInterface::updateButtonStates()
{
    switch (state)
    {
        case Idle:
            startButton->setEnabled(true);
            pauseButton->setEnabled(false);
            parkButton->setEnabled(true);
            unparkButton->setEnabled(true);
            break;
        case Running:
            startButton->setEnabled(false);
            pauseButton->setEnabled(true);
            parkButton->setEnabled(false);
            unparkButton->setEnabled(false);
            break;
        case Paused:
            startButton->setEnabled(true);
            pauseButton->setEnabled(false);
            parkButton->setEnabled(true);
            unparkButton->setEnabled(true);
            break;
    }
}

void HumanMachineInterface::onStartClicked()
{
    state = Running;
    updateButtonStates();
    configurationNode->start();
    RCLCPP_INFO(configurationNode->get_logger(), "Start signal processed!");
}

void HumanMachineInterface::onPauseClicked()
{
    state = Paused;
    updateButtonStates();
    configurationNode->pause();
}

void HumanMachineInterface::onParkClicked()
{
    configurationNode->park();
}

void HumanMachineInterface::onUnparkClicked()
{
    configurationNode->unPark();
}

void HumanMachineInterface::onSaveClicked()
{
    config.save();
}

void HumanMachineInterface::applyStyles()
{
    // Apply stylesheets for visual appearance
    setStyleSheet("QToolBox::tab {"
                  "   background: #c4c4c4;"       // Light gray background
                  "   border: 1px solid #dcdcdc;" // Light border
                  "   border-radius: 5px;"
                  "   padding: 5px;"
                  "}"
                  "QToolBox::tab:selected {"
                  "   background: #63bf67;"
                  "   font-weight: bold;"
                  "}"
                  "QGroupBox {"
                  "   font-weight: bold;"
                  "   border: 1px solid gray;"
                  "   border-radius: 5px;"
                  "   margin-top: 10px;"
                  "}"
                  "QGroupBox::title {"
                  "   subcontrol-origin: margin;"
                  "   subcontrol-position: top center;"
                  "   padding: 0 3px;"
                  "}"
                  "QPushButton {"
                  "   background-color: lightblue;"
                  "   border: 1px solid gray;"
                  "   border-radius: 5px;"
                  "   padding: 5px;"
                  "}"
                  "QPushButton:hover {"
                  "   background-color: blue;"
                  "   color: white;"
                  "}");
}