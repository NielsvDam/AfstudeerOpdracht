#ifndef STATEEXECUTIONHANDLER_HPP
#define STATEEXECUTIONHANDLER_HPP

#include <queue>  // std::queue
#include <memory> // std::shared_ptr

#include "StateEngine/State/AbstractState.hpp" // state::AbstractState

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
     * @class StateExecutionHandler
     * @brief Handles the execution of states.
     *
     * The StateExecutionHandler is responsible for executing the instructions of the states.
     */
    class StateExecutionHandler
    {
    public:
        virtual ~StateExecutionHandler();
        StateExecutionHandler(StateExecutionHandler const&) = delete;            /* the executor should never be copied */
        StateExecutionHandler& operator=(StateExecutionHandler const&) = delete; /* the executor should never be copied */
        /**
         * @brief Constructor for StateExecutionHandler.
         *
         * Initializes a new instance of the StateExecutionHandler class.
         */
        StateExecutionHandler();
        /**
         * @brief Adds a state to the state machine.
         *
         * @param state A shared pointer to the state to be added.
         */
        void addState(std::shared_ptr<state::AbstractState>& state);
        /**
         * @brief Initializes the state machine.
         */
        void initialize();
        /**
         * @brief Runs the main loop of the executor, executing instructions of queued states.
         *
         * Runs the main loop of the executor, executing instructions of queued states.
         */
        void loop();
        /**
         * @brief Resets the executor.
         *
         * Removes all states from the queue and stops executing instructions.
         */
        void reset();
    private:
        std::condition_variable queueCv; /* A condition variable that is notified once a state is added */
        std::mutex queueMutex;           /* A mutex to protect the queue. */
        std::queue<std::shared_ptr<state::AbstractState>> queue; /* The queue of states to be executed */
        std::shared_ptr<state::AbstractState> currentState;      /* The current state in execution */
    };
} // namespace state_engine

#endif // STATEEXECUTIONHANDLER_HPP
