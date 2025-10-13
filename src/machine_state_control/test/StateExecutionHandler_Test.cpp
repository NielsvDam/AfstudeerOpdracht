#include <gtest/gtest.h>
#include <memory>
#include <iostream>
#include <string>
#include <moveit_msgs/msg/robot_state.hpp>
#include <accessor/accessor.hpp>

#include "StateEngine/Instruction/AbstractInstruction.hpp"
#include "StateEngine/StateExecutionHandler.hpp"

// temp clas for abstract state testing
class TempState2 : public state::AbstractState
{
    public:
        virtual ~TempState2() {}
        TempState2(const moveit_msgs::msg::RobotState& startJointValues) : AbstractState(startJointValues) {}

        const std::string& name() const override
        {
            static const std::string name = "TempState";
            return name;
        }
};

// temp class for abstract instruction testing
class TempInstruction2 : public instruction::AbstractInstruction
{
    public:
        virtual ~TempInstruction2() {}
        TempInstruction2(const std::string& description) : AbstractInstruction(LOGGER_NAME, description) {}

        void execute() override
        {
            executed = true;
        }

        bool isExecuted() const
        {
            return executed;
        }
    private:
        inline static const std::string LOGGER_NAME = "TempInstruction"; /* The name of the logger. */
        bool executed = false;
};

// used to check what the current state is
MEMBER_ACCESSOR(StateExecutionHandler_currentState, state_engine::StateExecutionHandler, currentState, std::shared_ptr<state::AbstractState>)

/**
 * @brief Test instruction execution in executor.loop(). By adding and executing in various orders.
 *
 * Expected result: instruction should be executed each time loop is called. The current state should unset after the last
 * instruction.
 */
TEST(StateExecutionHandlerTest, TestExecutorLoop)
{
    state_engine::StateExecutionHandler stateExecutionHandler;

    // Get illegal access to the current state
    auto currentState = accessor::accessMember<StateExecutionHandler_currentState>(stateExecutionHandler);

    // Create a state
    moveit_msgs::msg::RobotState startJointValues; // create a dummy robot state
    std::shared_ptr<state::AbstractState> state = std::make_shared<TempState2>(startJointValues);

    // Add instructions in various orders
    std::shared_ptr<TempInstruction2> instruction1 = std::make_shared<TempInstruction2>("instruction 1");
    std::shared_ptr<TempInstruction2> instruction2 = std::make_shared<TempInstruction2>("instruction 2");
    std::shared_ptr<TempInstruction2> instruction3 = std::make_shared<TempInstruction2>("instruction 3");
    std::shared_ptr<TempInstruction2> instruction4 = std::make_shared<TempInstruction2>("instruction 4");
    std::shared_ptr<TempInstruction2> instruction5 = std::make_shared<TempInstruction2>("instruction 5");

    state->addInstruction(instruction1);
    state->addInstruction(instruction2);
    state->addInstruction(instruction3);

    EXPECT_FALSE(currentState.get());      // check that no state is active
    stateExecutionHandler.addState(state); // add the state to the executor
    EXPECT_FALSE(currentState.get()); // new current state should be picked up in the next loop, since the loop is watching
                                      // the queue

    stateExecutionHandler.loop();    // state 2 should now turn active, and instruction 1 should be executed instantly
    EXPECT_TRUE(currentState.get()); // check if any state is picked up
    EXPECT_EQ(currentState.get().get(), state.get()); // check if its our state, by comparing the memory address.
    EXPECT_TRUE(instruction1->isExecuted());          // instruction 1 should instantly be executed

    stateExecutionHandler.loop(); // execute instruction 2
    EXPECT_TRUE(instruction2->isExecuted());
    EXPECT_FALSE(instruction3->isExecuted());

    stateExecutionHandler.loop(); // execute instruction 3
    EXPECT_TRUE(instruction3->isExecuted());

    state->addInstruction(instruction4);
    state->addInstruction(instruction5, true); // add final instruction

    stateExecutionHandler.loop(); // execute instruction 4
    stateExecutionHandler.loop(); // execute instruction 5
    EXPECT_TRUE(instruction4->isExecuted());
    EXPECT_TRUE(instruction5->isExecuted());
    stateExecutionHandler.loop(); // should now unset the current state

    EXPECT_FALSE(currentState.get());
}

/**
 * @brief Test if the executer decides to read the next state from the queue when a state is done.
 *
 * Expected result: When the current state has no more instructions, the executor should pick the next state from the queue.
 * It should also instantly execute the first instruction of this new state.
 */
TEST(StateExecutionHandlerTest, TestExecutorQueue)
{
    state_engine::StateExecutionHandler stateExecutionHandler;
    stateExecutionHandler.initialize();

    // Get illegal access to the current state
    auto currentState = accessor::accessMember<StateExecutionHandler_currentState>(stateExecutionHandler);

    // Create two states
    moveit_msgs::msg::RobotState startJointValues; // create a dummy robot state
    std::shared_ptr<state::AbstractState> state1 = std::make_shared<TempState2>(startJointValues);
    std::shared_ptr<state::AbstractState> state2 = std::make_shared<TempState2>(startJointValues);

    // create instructions for state 1
    std::shared_ptr<TempInstruction2> instruction1 = std::make_shared<TempInstruction2>("instruction 1");
    std::shared_ptr<TempInstruction2> instruction2 = std::make_shared<TempInstruction2>("instruction 2");
    std::shared_ptr<TempInstruction2> instruction3 = std::make_shared<TempInstruction2>("instruction 3");
    // create instructions for state 2
    std::shared_ptr<TempInstruction2> instruction4 = std::make_shared<TempInstruction2>("instruction 4");
    // add both states to the queue
    stateExecutionHandler.addState(state1);
    stateExecutionHandler.addState(state2); // (can be added at any time)
    // Add instructions to the state1
    state1->addInstruction(instruction1);
    state1->addInstruction(instruction2);
    state1->addInstruction(instruction3, true); // add the final instruction to state1
    // Add instructions to the state2
    state2->addInstruction(instruction4);

    stateExecutionHandler.loop(); // execute instruction 1
    stateExecutionHandler.loop(); // execute instruction 2
    stateExecutionHandler.loop(); // execute instruction 3
    stateExecutionHandler.loop(); // this iteration it figures out all instructions are executed, and then unsets the current
                                  // state. The next iteration it should read the queue again.
    EXPECT_FALSE(currentState.get()); // Check if the current state is unset.

    EXPECT_FALSE(instruction4->isExecuted());          // first instruction of state 2 should not be executed yet
    stateExecutionHandler.loop();                      // state 2 should now turn active.
    EXPECT_EQ(currentState.get().get(), state2.get()); // check if state 2 is active, by comparing the memory address.
    EXPECT_TRUE(instruction4->isExecuted());           // instruction4 should instantly be executed.
}