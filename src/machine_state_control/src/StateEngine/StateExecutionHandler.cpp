#include "StateEngine/StateExecutionHandler.hpp"

#include "StateEngine/Instruction/AbstractInstruction.hpp" // instruction::AbstractInstruction

namespace state_engine
{
    StateExecutionHandler::~StateExecutionHandler() {}

    StateExecutionHandler::StateExecutionHandler() : currentState(nullptr) {}

    void StateExecutionHandler::initialize() {}

    void StateExecutionHandler::loop()
    {
        if (!currentState)
        {
            std::unique_lock<std::mutex> lock(queueMutex);
            while (queue.empty())
            {
                queueCv.wait(lock);
            }
            currentState = queue.front();
            queue.pop();
        }

        if (std::shared_ptr<instruction::AbstractInstruction> instruction = currentState->awaitNextInstruction())
        {
            instruction->execute();
        }
        else
        {
            currentState = nullptr;
        }
    }

    void StateExecutionHandler::reset()
    {
        std::lock_guard<std::mutex> lock(queueMutex);
        while (!queue.empty())
        {
            queue.pop();
        }
        queueCv.notify_all();
    }

    void StateExecutionHandler::addState(std::shared_ptr<state::AbstractState>& state)
    {
        std::lock_guard<std::mutex> lock(queueMutex);
        queue.push(state);
        queueCv.notify_one();
    }
} // namespace state_engine