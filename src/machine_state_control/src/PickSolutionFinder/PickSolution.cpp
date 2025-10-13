#include "PickSolutionFinder/PickSolution.hpp"

namespace pick_solution_finder
{
    PickSolution::~PickSolution() {}

    PickSolution::PickSolution(geometry_msgs::msg::Pose pickPose, geometry_msgs::msg::Pose retractPose)
        : pickPose(pickPose), retractPose(retractPose)
    {}

    geometry_msgs::msg::Pose PickSolution::getPickPose()
    {
        return pickPose;
    }

    geometry_msgs::msg::Pose PickSolution::getRetractPose()
    {
        return retractPose;
    }

    geometry_msgs::msg::Pose PickSolution::getAboveCratePose()
    {
        geometry_msgs::msg::Pose aboveCratePose = retractPose;
        aboveCratePose.position.z = 0.0; // Set Z to 0.0 in the world frame
        return aboveCratePose;
    }
} // namespace pick_solution_finder