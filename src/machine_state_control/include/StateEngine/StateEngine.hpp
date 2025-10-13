#ifndef STATEENGINE_HPP
#define STATEENGINE_HPP

#include <memory>             // std::shared_ptr
#include <string>             // std::string
#include <thread>             // std::thread
#include <atomic>             // std::atomic
#include <condition_variable> // std::condition_variable
#include <mutex>              // std::mutex
#include <rclcpp/rclcpp.hpp>  // rclcpp::Logger

#include <custom_msgs/msg/located_object.hpp>    // custom_msgs::msg::LocatedObject
#include "StateEngine/StateCreationHandler.hpp"  // state_engine::StateCreationHandler
#include "StateEngine/StateExecutionHandler.hpp" // state_engine::StateExecutionHandler
#include "OperationStateEnum.hpp"                // OperationState

/**
 * @namespace state_engine
 * @brief Contains classes related to the control of the sate machine.
 *
 * The classes within the namespace are responsible for managing the state machine. This includes the creation of states, and
 * execution of them.
 */
namespace state_engine
{
    /**
     * @class StateEngine
     * @brief The StateEngine is a high-level class of the state machine.
     *
     * StateEngine manages the initialization and lifecycle of the internal components. It also provides a method for
     * changing the operation state.
     */
    class StateEngine
    {
    public:
        virtual ~StateEngine();
        StateEngine(StateEngine const&) = delete;
        StateEngine& operator=(StateEngine const&) = delete;
        /**
         * @brief Constructs a new StateEngine object.
         */
        StateEngine();
        /**
         * @brief Initializes the state engine.
         */
        void initialize();
        /**
         * @brief Changes the operation state.
         *
         * @param newState The new operation state.
         */
        void setOperationState(OperationState newState);
        /**
         * @brief inform state engine about detected objects
         *
         * @param detectedObjects A vector of detected objects.
         * @param unknownAreas A vector of unknown areas.
         */
        void objectsDetected(
            const std::vector<custom_msgs::msg::LocatedObject>& detectedObjects,
            const std::vector<custom_msgs::msg::LocatedObject>& unknownAreas);
    private:
        /**
         * @brief Starts the threads for the state engine.
         */
        void startThreads();

        OperationState operationState;            /* The current operation state. */
        std::mutex operationStateMutex;           /* A mutex to protect the operation state. */
        std::condition_variable operationStateCv; /* A condition variable to wait for the operation state to change. */

        std::atomic<bool> plannerThreadRunning;  /* A boolean indicating if the planner thread is running. */
        std::atomic<bool> executorThreadRunning; /* A boolean indicating if the executor thread is running. */

        std::thread planningThread; /* The thread that creates the state pipelines. */
        std::thread executorThread; /* The thread that executes the instructions */

        StateExecutionHandler stateExecutionHandler; /* The state execution handler. */
        StateCreationHandler stateCreationHandler;   /* The state creation handler. */

        const std::string LOGGER_NAME = "StateEngine"; /* The name of the logger. */
        rclcpp::Logger logger;                         /* A logger object for logging messages. */
    };
} // namespace state_engine

#endif // STATEENGINE_HPP
