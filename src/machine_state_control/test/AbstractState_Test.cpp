#include <gtest/gtest.h>
#include <memory>
#include <functional> // for std::invoke
#include <iostream>
#include <string>
#include <moveit_msgs/msg/robot_state.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include "StateEngine/State/AbstractState.hpp"
#include "StateEngine/Instruction/AbstractInstruction.hpp"
#include "StateEngine/Instruction/MovementInstruction.hpp"

// temp clas for abstract state testing
class TempState : public state::AbstractState
{
public:
    virtual ~TempState() {}
    TempState(const moveit_msgs::msg::RobotState& startJointValues) : AbstractState(startJointValues) {}

    const std::string& name() const override
    {
        static const std::string name = "TempState";
        return name;
    }
};

// temp class for abstract instruction testing
class TempInstruction : public instruction::AbstractInstruction
{
public:
    virtual ~TempInstruction() {}
    TempInstruction(const std::string& description) : AbstractInstruction(LOGGER_NAME, description) {}

    void execute() override {} // no need to implement
private:
    inline static const std::string LOGGER_NAME = "TempInstruction"; /* The name of the logger. */
};

/**
 * @brief Test pre: instruction must not be nullptr.
 *
 * Expected result: A runtime error should be thrown.
 */
TEST(AbstractStateTest, TestAddInstructionNullptr)
{
    // Create a TempState object
    moveit_msgs::msg::RobotState startJointValues;
    TempState tempState(startJointValues);
    // Expect a runtime exception to be thrown
    EXPECT_THROW(tempState.addInstruction(nullptr), std::runtime_error);
}

/**
 * @brief Test pre: a instruction marked state-final has not been added to the state already.
 *
 * Expected result: A runtime error should be thrown.
 */
TEST(AbstractStateTest, TestAddInstructionAfterFinal)
{
    // Create a TempState object
    moveit_msgs::msg::RobotState tempJointPos;
    TempState tempState(tempJointPos);
    // Expect a runtime exception to be thrown
    std::shared_ptr<instruction::AbstractInstruction> instruction1 = std::make_shared<TempInstruction>("Instruction 1");
    std::shared_ptr<instruction::AbstractInstruction> instruction2 = std::make_shared<TempInstruction>("Instruction 2");
    tempState.addInstruction(instruction1, true);
    EXPECT_THROW(tempState.addInstruction(instruction2), std::runtime_error);
}
/**
 * @brief Test getFinalJointValues function when adding empty plan
 *
 * Expected result: The current goal joint values should be the same as the start joint values
 */
TEST(AbstractStateTest, TestGetGoalJointPositionsEmptyPlan)
{
    // Create a TempState object
    moveit_msgs::msg::RobotState startJointPos;
    startJointPos.joint_state.name = {"joint1", "joint2", "joint3"};
    startJointPos.joint_state.position = {0.5, 0.2, 0.3};
    TempState tempState(startJointPos);
    // Expect the goal joint values to be the same as the start joint values
    EXPECT_EQ(tempState.getLastJointValues(), startJointPos);

    // add another instruction to validate that the goal joint values remain the same
    std::shared_ptr<instruction::AbstractInstruction> instruction1 =
        std::make_shared<TempInstruction>("A non movement instruction");
    tempState.addInstruction(instruction1);
    EXPECT_EQ(tempState.getLastJointValues(), startJointPos);

    // Create a MovementInstruction object, which will change the goal positions of the state
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    std::shared_ptr<instruction::MovementInstruction> instruction2 =
        std::make_shared<instruction::MovementInstruction>(plan.trajectory_, 0.001);
    tempState.addInstruction(instruction2);
    EXPECT_EQ(tempState.getLastJointValues(), startJointPos);
}

/**
 * @brief Test getFinalJointValues function
 *
 * Expected result: The state currentGoalJointPositions should equal the goaljointPositions of the last added instruction
 */
TEST(AbstractStateTest, TestGetGoalJointPositions)
{
    // Create a TempState object
    moveit_msgs::msg::RobotState startJointPos;
    startJointPos.joint_state.name = {"joint1", "joint2", "joint3"};
    startJointPos.joint_state.position = {0.5, 0.2, 0.3};
    TempState tempState(startJointPos);
    // Expect the goal joint values to be the same as the provided start joint values
    EXPECT_EQ(tempState.getLastJointValues(), startJointPos);

    // add another instruction to validate that the goal joint values remain the same
    std::shared_ptr<instruction::AbstractInstruction> instruction1 =
        std::make_shared<TempInstruction>("A non movement instruction");
    tempState.addInstruction(instruction1);
    EXPECT_EQ(tempState.getLastJointValues(), startJointPos);

    // Create a MovementInstruction object, which will change the goal positions of the state
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.start_state_ = startJointPos;
    plan.trajectory_.joint_trajectory.joint_names = {"joint1", "joint2", "joint3"};
    plan.trajectory_.joint_trajectory.points.resize(1);
    plan.trajectory_.joint_trajectory.points[0].positions = {1.0, 1.1, 1.2};
    std::shared_ptr<instruction::MovementInstruction> instruction2 =
        std::make_shared<instruction::MovementInstruction>(plan.trajectory_, 0.001);
    tempState.addInstruction(instruction2);
    EXPECT_EQ(tempState.getLastJointValues(), instruction2->getFinalJointValues());
}

/**
 * @brief Test awaitNextInstruction function multiple times
 *
 * Expected result: The function should block until an instruction is available and then return it each time.
 */
TEST(AbstractStateTest, TestAwaitNextInstructionMultipleTimes)
{
    // Create a TempState object
    moveit_msgs::msg::RobotState startJointPos;
    TempState tempState(startJointPos);

    // Create a thread to add instructions after a delay
    std::thread instructionAdder([&tempState]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        std::shared_ptr<instruction::AbstractInstruction> instruction1 = std::make_shared<TempInstruction>("Instruction 1");
        tempState.addInstruction(instruction1);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        std::shared_ptr<instruction::AbstractInstruction> instruction2 = std::make_shared<TempInstruction>("Instruction 2");
        tempState.addInstruction(instruction2);
    });

    // Call awaitNextInstruction and expect it to return the added instructions
    auto instruction1 = tempState.awaitNextInstruction();
    EXPECT_NE(instruction1, nullptr);
    EXPECT_EQ(instruction1->getDescription(), "Instruction 1");

    auto instruction2 = tempState.awaitNextInstruction();
    EXPECT_NE(instruction2, nullptr);
    EXPECT_EQ(instruction2->getDescription(), "Instruction 2");

    // Join the thread
    instructionAdder.join();
}