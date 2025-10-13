#include "StateEngine/StateEngine.hpp"
#include "MachineStateControlNode.hpp"

namespace state_engine
{
    StateEngine::~StateEngine()
    {
        plannerThreadRunning = false;
        executorThreadRunning = false;
        operationStateCv.notify_all();
        if (planningThread.joinable())
        {
            planningThread.join();
        }
        if (executorThread.joinable())
        {
            executorThread.join();
        }
    }

    StateEngine::StateEngine()
        : operationState(OperationState::IDLE),
          plannerThreadRunning(false),
          executorThreadRunning(false),
          stateExecutionHandler(),
          stateCreationHandler(stateExecutionHandler),
          logger(rclcpp::get_logger(LOGGER_NAME))
    {}

    void StateEngine::initialize()
    {
        stateCreationHandler.initialize();
        stateExecutionHandler.initialize();
        startThreads();
    }

    void StateEngine::startThreads()
    {
        // Start the planning thread
        plannerThreadRunning = true;
        planningThread = std::thread([this]() {
            RCLCPP_INFO(logger, "Planner thread started");
            while (plannerThreadRunning)
            {
                std::unique_lock<std::mutex> lock(operationStateMutex);
                operationStateCv.wait(lock, [this]() {
                    return operationState == OperationState::RUNNING || !plannerThreadRunning;
                });
                if (!plannerThreadRunning)
                {
                    break;
                }
                lock.unlock();
                stateCreationHandler.loop();
            }
            RCLCPP_INFO(logger, "Planner thread stopped");
        });
        // Start the executor thread
        executorThreadRunning = true;
        executorThread = std::thread([this]() {
            RCLCPP_INFO(logger, "Executor thread started");
            while (executorThreadRunning)
            {
                std::unique_lock<std::mutex> lock(operationStateMutex);
                operationStateCv.wait(lock, [this]() {
                    return operationState == OperationState::RUNNING || !executorThreadRunning;
                });
                if (!executorThreadRunning)
                {
                    break;
                }
                lock.unlock();
                stateExecutionHandler.loop();
            }
            RCLCPP_INFO(logger, "Executor thread stopped");
        });
    }

    void StateEngine::setOperationState(OperationState newState)
    {
        {
            std::lock_guard<std::mutex> lock(operationStateMutex);
            operationState = newState;
        }
        if (newState == OperationState::IDLE)
        {
            stateCreationHandler.reset();
            stateExecutionHandler.reset();
        }
        operationStateCv.notify_all();
    }

    void StateEngine::objectsDetected(
        const std::vector<custom_msgs::msg::LocatedObject>& detectedObjects,
        const std::vector<custom_msgs::msg::LocatedObject>& unknownAreas)
    {
        stateCreationHandler.objectsDetected(detectedObjects, unknownAreas);
    }

} // namespace state_engine