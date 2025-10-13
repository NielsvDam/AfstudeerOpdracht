#ifndef STATECREATIONHANDLER_HPP
#define STATECREATIONHANDLER_HPP

#include <memory>                                             // std::shared_ptr
#include <string>                                             // std::string
#include <deque>                                              // std::deque
#include <condition_variable>                                 // std::condition_variable
#include <mutex>                                              // std::mutex
#include <rclcpp/rclcpp.hpp>                                  // rclcpp::Logger
#include <moveit/move_group_interface/move_group_interface.h> // moveit::planning_interface::MoveGroupInterface

#include <custom_msgs/msg/located_object.hpp>                  // custom_msgs::msg::LocatedObject
#include "StateEngine/StatePipeline/AbstractStatePipeline.hpp" // state_pipeline::AbstractStatePipeline
#include "StateEngine/StateExecutionHandler.hpp"               // state_engine::StateExecutionHandler
#include "PickSolutionFinder/PickSolution.hpp"                 // pick_solution_finder::PickSolution

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
     * @class StateCreationHandler
     * @brief Handles the creation and dispatching of states.
     *
     * The StateCreationHandler is responsible for creating new states and dispatching them to the state execution handler.
     */
    class StateCreationHandler
    {
    public:
        virtual ~StateCreationHandler();
        StateCreationHandler(StateCreationHandler const&) = delete;            /* the executor should never be copied */
        StateCreationHandler& operator=(StateCreationHandler const&) = delete; /* the executor should never be copied */
        /**
         * @brief Constructs a new StateCreationHandler object.
         *
         * @param stateExecutionHandler A reference to the state execution handler.
         */
        explicit StateCreationHandler(StateExecutionHandler& stateExecutionHandler);
        /**
         * @brief Initializes the state creation handler.
         */
        void initialize();
        /**
         * @brief Runs the main loop of the state creation handler, creating new states and dispatching them to the state
         * execution handler.
         */
        void loop();
        /**
         * @brief Resets the state creation handler.
         *
         * Removes all states from the history and stops creating new states.
         */
        void reset();
        /**
         * @brief inform about detected objects
         *
         * @param detectedObjects the detected objects.
         * @param unknownAreas the unknown areas.
         */
        void objectsDetected(
            const std::vector<custom_msgs::msg::LocatedObject>& detectedObjects,
            const std::vector<custom_msgs::msg::LocatedObject>& unknownAreas);
    private:
        /**
         * @brief Get the next state pipeline.
         *
         * This function decides which state comes next, based on the state history.
         *
         * @return std::shared_ptr<AbstractStatePipeline> the next state pipeline.
         */
        std::shared_ptr<state_pipeline::AbstractStatePipeline> getNextStatePipeline();
        /**
         * @brief Add a state to the history
         *
         * @param pipeline the state to add.
         */
        void addStateToHistory(const std::shared_ptr<state::AbstractState>& state);
        /**
         * @brief Get a state from the history
         *
         * @param index how many states from the back.
         * @return std::shared_ptr<AbstractState> the state.
         */
        std::shared_ptr<state::AbstractState> getPreviousState(size_t index) const;
        /**
         * @brief Wait for the pick solution to be available.
         *
         * @return std::shared_ptr<PickSolution> the pick solution.
         */
        std::shared_ptr<pick_solution_finder::PickSolution> awaitPickSolution();

        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> moveGroup; /* The move group for planning. */

        static const size_t maxStateHistory = 10; /* The maximum number of states to keep in history. */
        std::deque<std::shared_ptr<state::AbstractState>> stateHistory; /* A history of created states. */

        StateExecutionHandler& stateExecutionHandler; /* A reference to the state execution handler. */

        std::mutex currentStatePipelineMtx; /* A mutex to protect the state pipeline. */
        std::shared_ptr<state_pipeline::AbstractStatePipeline> currentStatePipeline; /* The current state pipeline. */

        std::shared_ptr<pick_solution_finder::PickSolution> pickSolution; /* The found pick solution .*/
        std::condition_variable pickSolutionCv; /* A condition variable to signal that the pickSolution are available. */
        bool pickSolutionNotified;              /* A flag to indicate that the pickSolution are available. */
        std::mutex pickSolutionMtx;             /* A mutex to protect the pickSolution. */
        tf2::Quaternion curPickOrientation;     /* The orientation of the pick pose. */
        inline static const std::string LOGGER_NAME = "StateCreationHandler"; /* The name of the logger. */
        rclcpp::Logger logger;                                                /* A logger object for logging messages. */
    };
} // namespace state_engine

#endif // STATECREATIONHANDLER_HPP
