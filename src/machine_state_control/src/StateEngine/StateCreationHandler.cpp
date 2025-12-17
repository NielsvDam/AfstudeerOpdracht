#include "StateEngine/StateCreationHandler.hpp"

#include <moveit/robot_state/conversions.h> // moveit::core::robotStateToRobotStateMsg

#include "StateEngine/StatePipeline/PictureStatePipeline.hpp"   // state_pipeline::PictureStatePipeline
#include "StateEngine/StatePipeline/PlaceStatePipeline.hpp"     // state_pipeline::PlaceStatePipeline
#include "StateEngine/StatePipeline/PickStatePipeline.hpp"      // state_pipeline::PickStatePipeline
#include "StateEngine/StatePipeline/ShuffleStatePipeline.hpp"   // state_pipeline::ShuffleStatePipeline
#include "StateEngine/StatePipeline/KnockOverStatePipeline.hpp" // state_pipeline::KnockOverStatePipeline
#include "StateEngine/State/AbstractCrateState.hpp"             // state::AbstractCrateState
#include "StateEngine/State/AbstractDisplaceState.hpp"          // state::AbstractDisplaceState
#include "StateEngine/State/AbstractState.hpp"                  // state::AbstractState
#include "StateEngine/State/PictureState.hpp"                   // state::PictureState
#include "StateEngine/State/PickState.hpp"                      // state::PickState
#include "StateEngine/State/PlaceState.hpp"                     // state::PlaceState
#include "StateEngine/State/ShuffleState.hpp"                   // state::ShuffleState
#include "StateEngine/State/KnockOverState.hpp"                 // state::KnockOverState
#include "PickSolutionFinder/PickSolutionFinder.hpp"            // pick_solution_finder::PickSolutionFinder
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>              // tf2::fromMsg
#include "MachineStateControlNode.hpp"
namespace state_engine
{
    StateCreationHandler::~StateCreationHandler() {}

    StateCreationHandler::StateCreationHandler(StateExecutionHandler& stateExecutionHandler)
        : moveGroup(nullptr),
          stateExecutionHandler(stateExecutionHandler),
          currentStatePipeline(nullptr),
          pickSolution(nullptr),
          pickSolutionNotified(false),
          curPickOrientation(),
          logger(rclcpp::get_logger(LOGGER_NAME))
    {}

    void StateCreationHandler::initialize()
    {
        RCLCPP_INFO(logger, "Initializing StateCreationHandler");

        // initialize the move group
        auto controlNode = MachineStateControlNode::getInstance();
        std::string planningGroup = controlNode->get_parameter("planning_group").as_string();
        double acceleration = controlNode->get_parameter("acceleration_scaling").as_double();
        moveGroup = std::make_shared<moveit::planning_interface::MoveGroupInterface>(controlNode, planningGroup);
        moveGroup->setMaxAccelerationScalingFactor(acceleration);
        moveGroup->setGoalPositionTolerance(0.001);
        moveGroup->setGoalJointTolerance(0.005);
        moveGroup->setPlanningTime(4.0);
        moveGroup->setNumPlanningAttempts(20);
        moveGroup->startStateMonitor();
    }

    void StateCreationHandler::loop()
    {
        std::lock_guard<std::mutex> lock(currentStatePipelineMtx);
        try
        {
            // if the pipeline is done (or there is no pipeline), select the next pipeline
            if (!currentStatePipeline || currentStatePipeline->createNextInstruction())
            {
                currentStatePipeline = getNextStatePipeline();
                // get the state from the pipeline
                std::shared_ptr<state::AbstractState> state = currentStatePipeline->getState();
                // send the state to the executor
                stateExecutionHandler.addState(state);
                // add the state to the history
                addStateToHistory(state);
            }
        }
        // if an error occurs, abort the current state and start over with a new picture state, this is a fallback solution
        // when any motion-planning is failed, and no other alternative behavior is possible.
        catch (const std::runtime_error& e)
        {
            RCLCPP_WARN(logger, "Error while creating next instruction: %s", e.what());
            RCLCPP_INFO(logger, "Aborting current state and creating a new PictureStatePipeline as a fallback solution");
            // abort the current state
            std::shared_ptr<state::AbstractState> state = currentStatePipeline->getState();
            state->abort();
            // retrieve the last joint values of the state.
            auto startJointValues = state->getLastJointValues();
            // create a new picture state pipeline
            currentStatePipeline = std::make_shared<state_pipeline::PictureStatePipeline>(startJointValues, moveGroup);
            // get the state from the pipeline and send it to the executor
            std::shared_ptr<state::AbstractState> newState = currentStatePipeline->getState();
            stateExecutionHandler.addState(newState);
            // clear history to prevent incorrect state transitions
            stateHistory.clear();
            // add the new state to the history
            addStateToHistory(newState);
        }
    }

    void StateCreationHandler::reset()
    {
        std::lock_guard<std::mutex> lock(currentStatePipelineMtx);
        currentStatePipeline = nullptr;
        stateHistory.clear();
    }

    std::shared_ptr<state_pipeline::AbstractStatePipeline> StateCreationHandler::getNextStatePipeline()
    {
        std::shared_ptr<state::AbstractState> previousState = getPreviousState(0);
        moveit_msgs::msg::RobotState startJointValues;

        if (!previousState) // if there is no previous state, use the actual current joint state of the robot
        {
            moveit::core::robotStateToRobotStateMsg(*moveGroup->getCurrentState(), startJointValues);
        }
        else
        {
            startJointValues = previousState->getLastJointValues();
        }

        bool previousWasPick = std::dynamic_pointer_cast<state::PickState>(previousState) != nullptr;
        bool previousWasPlace = std::dynamic_pointer_cast<state::PlaceState>(previousState) != nullptr;
        bool previousWasPicture = std::dynamic_pointer_cast<state::PictureState>(previousState) != nullptr;
        bool pickedBeforePicture = std::dynamic_pointer_cast<state::PickState>(getPreviousState(1)) != nullptr;

        if (!previousState || previousWasPick) // check if a previous state is present, or if the previous state was
                                               // 'PickState'
        {
            return std::make_shared<state_pipeline::PictureStatePipeline>(startJointValues, moveGroup);
        }
        if (previousWasPicture && pickedBeforePicture) // check if we picked before the picture, then go to place state.
        {
            return std::make_shared<state_pipeline::PlaceStatePipeline>(startJointValues, curPickOrientation, moveGroup);
        }
        if (previousWasPicture || previousWasPlace)
        {
            RCLCPP_INFO(logger, "Waiting for pick solution");
            std::shared_ptr<pick_solution_finder::PickSolution> pickSolution = awaitPickSolution();

            if (!pickSolution) // check if there are any pick options, otherwise fall back to picture state.
            {
                RCLCPP_INFO(logger, "No pick solution found, creating a PictureStatePipeline as a fallback solution");
                return std::make_shared<state_pipeline::PictureStatePipeline>(startJointValues, moveGroup);
            }
            // return the pick state pipeline.
            auto pipeline = std::make_shared<state_pipeline::PickStatePipeline>(startJointValues, moveGroup);
            pipeline->setPickSolution(pickSolution);
            tf2::fromMsg(pickSolution->getPickPose().orientation, curPickOrientation); // overwrite the pick orientation
            return pipeline;
        }
        return nullptr;
    }

    void StateCreationHandler::addStateToHistory(const std::shared_ptr<state::AbstractState>& state)
    {
        if (stateHistory.size() >= maxStateHistory)
        {
            stateHistory.pop_front();
        }
        stateHistory.push_back(state);
    }

    std::shared_ptr<state::AbstractState> StateCreationHandler::getPreviousState(size_t index) const
    {
        if (stateHistory.empty() || index >= stateHistory.size())
        {
            return nullptr;
        }
        // Use a reverse iterator to access the element from the end
        auto it = stateHistory.rbegin();
        std::advance(it, index);
        return *it;
    }

    std::shared_ptr<pick_solution_finder::PickSolution> StateCreationHandler::awaitPickSolution()
    {
        std::unique_lock<std::mutex> lock(pickSolutionMtx);
        pickSolutionCv.wait(lock, [this] { return pickSolutionNotified; });
        pickSolutionNotified = false;

        if (pickSolution)
        {
            std::shared_ptr<pick_solution_finder::PickSolution> pickSolutionCopy = std::move(pickSolution);
            pickSolution = nullptr;
            return pickSolutionCopy;
        }
        return nullptr;
    }

    void StateCreationHandler::objectsDetected(
        const std::vector<custom_msgs::msg::LocatedObject>& detectedObjects,
        const std::vector<custom_msgs::msg::LocatedObject>& unknownAreas)
    {
        std::lock_guard<std::mutex> lock(pickSolutionMtx);
        // Attempt to find the pick solution using the pick solution finder
        pick_solution_finder::PickSolutionFinder pickSolutionFinder(logger, detectedObjects, unknownAreas, moveGroup);
        pickSolution = pickSolutionFinder.findSolution(); // its nullptr if no solution is found. This is intended
        pickSolutionNotified = true;
        pickSolutionCv.notify_all();
    }
} // namespace state_engine