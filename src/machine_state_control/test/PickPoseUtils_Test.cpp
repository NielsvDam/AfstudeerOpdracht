#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <memory>
#include <functional>                              // for std::invoke
#include <geometry_msgs/msg/pose.hpp>              // geometry_msgs::msg::Pose
#include <tf2/LinearMath/Quaternion.h>             // tf2::Quaternion
#include <tf2/LinearMath/Transform.h>              // tf2::Transform
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> // tf2::fromMsg, tf2::toMsg
#include <tf2/LinearMath/Vector3.h>                // tf2::Vector3
#include "Rad.hpp"                                 // RAD_180DEG, RAD_90DEG, RAD_45DEG

#include "PickSolutionFinder/PickPoseUtils.hpp"

TEST(PickPoseUtilsTest, testGeneratePickPoses)
{
    // Helper function to create a pose with given RPY
    auto createPose = [&](double roll, double pitch, double yaw) {
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        geometry_msgs::msg::Pose pose;
        pose.position.x = 0.0;
        pose.position.y = 0.0;
        pose.position.z = 0.0;
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();
        return pose;
    };

    geometry_msgs::msg::Pose objectPose = createPose(0.0, 0.0, 0.0);

    std::vector<geometry_msgs::msg::Pose> pickPoses = pick_solution_finder::PickPoseUtils::generatePickPoses(objectPose);

    // Check that the number of generated pick poses is correct
    EXPECT_EQ(pickPoses.size(), 6);

    // Expected poses
    std::vector<geometry_msgs::msg::Pose> expectedPoses;

    // 0 degrees YAW
    expectedPoses.push_back(createPose(0.0, 0.0, 0.0));
    // 90 degrees YAW
    expectedPoses.push_back(createPose(0.0, 0.0, RAD_90DEG));
    // 0 degrees YAW, +45 degrees PITCH
    expectedPoses.push_back(createPose(0.0, -RAD_45DEG, 0.0));
    // 0 degrees YAW, -45 degrees PITCH
    expectedPoses.push_back(createPose(0.0, -RAD_45DEG, RAD_90DEG));
    // 90 degrees YAW, +45 degrees PITCH
    expectedPoses.push_back(createPose(0.0, -RAD_45DEG, RAD_180DEG));
    // 90 degrees YAW, -45 degrees PITCH
    expectedPoses.push_back(createPose(0.0, -RAD_45DEG, RAD_270DEG));

    // Compare the generated poses with the expected poses
    for (size_t i = 0; i < pickPoses.size(); ++i)
    {
        tf2::Quaternion generatedQuaternion(
            pickPoses.at(i).orientation.x,
            pickPoses.at(i).orientation.y,
            pickPoses.at(i).orientation.z,
            pickPoses.at(i).orientation.w);

        tf2::Quaternion expectedq(
            expectedPoses.at(i).orientation.x,
            expectedPoses.at(i).orientation.y,
            expectedPoses.at(i).orientation.z,
            expectedPoses.at(i).orientation.w);

        double generatedRoll, generatedPitch, generatedYaw;
        tf2::Matrix3x3(generatedQuaternion).getRPY(generatedRoll, generatedPitch, generatedYaw);

        double expectedroll, expectedpitch, expectedyaw;
        tf2::Matrix3x3(expectedq).getRPY(expectedroll, expectedpitch, expectedyaw);

        EXPECT_NEAR(generatedRoll, expectedroll, 1e-5);
        EXPECT_NEAR(generatedPitch, expectedpitch, 1e-5);
        EXPECT_NEAR(generatedYaw, expectedyaw, 1e-5);
    }
}

TEST(PickPoseUtilsTest, testCalculateRetractPoseAbsolute)
{
    bool relative = false;
    double retractDistance = 0.03;

    geometry_msgs::msg::Pose pickPose;
    pickPose.position.x = 1.0;
    pickPose.position.y = 2.0;
    pickPose.position.z = 3.0;
    pickPose.orientation.x = 0.0;
    pickPose.orientation.y = 0.0;
    pickPose.orientation.z = 0.0;
    pickPose.orientation.w = 1.0;

    geometry_msgs::msg::Pose expectedRetractPose;
    expectedRetractPose.position.x = 1.0;
    expectedRetractPose.position.y = 2.0;
    expectedRetractPose.position.z = 3.0 + retractDistance;
    expectedRetractPose.orientation.x = 0.0;
    expectedRetractPose.orientation.y = 0.0;
    expectedRetractPose.orientation.z = 0.0;
    expectedRetractPose.orientation.w = 1.0;

    geometry_msgs::msg::Pose retractPose =
        pick_solution_finder::PickPoseUtils::calculateRetractPose(pickPose, retractDistance, relative);

    EXPECT_EQ(retractPose, expectedRetractPose);
}

TEST(PickPoseUtilsTest, testCalculateRetractPoseRelative)
{
    bool relative = true;
    double retractDistance = 0.03;

    geometry_msgs::msg::Pose pickPose;
    pickPose.position.x = 1.0;
    pickPose.position.y = 2.0;
    pickPose.position.z = 3.0;
    pickPose.orientation.x = 0.5; // Some non-zero orientation
    pickPose.orientation.y = 0.5;
    pickPose.orientation.z = 0.5;
    pickPose.orientation.w = 0.5;

    // Convert quaternion to rotation matrix
    double qw = pickPose.orientation.w;
    double qx = pickPose.orientation.x;
    double qy = pickPose.orientation.y;
    double qz = pickPose.orientation.z;

    // Calculate the rotation matrix (https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation)
    double rotationMatrix[3][3];
    rotationMatrix[0][0] = 1 - 2 * (qy * qy + qz * qz);
    rotationMatrix[0][1] = 2 * (qx * qy - qz * qw);
    rotationMatrix[0][2] = 2 * (qx * qz + qy * qw);
    rotationMatrix[1][0] = 2 * (qx * qy + qz * qw);
    rotationMatrix[1][1] = 1 - 2 * (qx * qx + qz * qz);
    rotationMatrix[1][2] = 2 * (qy * qz - qx * qw);
    rotationMatrix[2][0] = 2 * (qx * qz - qy * qw);
    rotationMatrix[2][1] = 2 * (qy * qz + qx * qw);
    rotationMatrix[2][2] = 1 - 2 * (qx * qx + qy * qy);

    // Apply the translation in the object's frame
    double relativeDx = 0;
    double relativeDy = 0;
    double relativeDz = -retractDistance;

    // Transform the translation to the world frame
    double worldDx =
        rotationMatrix[0][0] * relativeDx + rotationMatrix[0][1] * relativeDy + rotationMatrix[0][2] * relativeDz;
    double worldDy =
        rotationMatrix[1][0] * relativeDx + rotationMatrix[1][1] * relativeDy + rotationMatrix[1][2] * relativeDz;
    double worldDz =
        rotationMatrix[2][0] * relativeDx + rotationMatrix[2][1] * relativeDy + rotationMatrix[2][2] * relativeDz;

    geometry_msgs::msg::Pose expectedRetractPose;
    expectedRetractPose.position.x = pickPose.position.x + worldDx;
    expectedRetractPose.position.y = pickPose.position.y + worldDy;
    expectedRetractPose.position.z = pickPose.position.z + worldDz;
    expectedRetractPose.orientation.x = pickPose.orientation.x;
    expectedRetractPose.orientation.y = pickPose.orientation.y;
    expectedRetractPose.orientation.z = pickPose.orientation.z;
    expectedRetractPose.orientation.w = pickPose.orientation.w;

    geometry_msgs::msg::Pose retractPose =
        pick_solution_finder::PickPoseUtils::calculateRetractPose(pickPose, retractDistance, relative);

    EXPECT_NEAR(retractPose.position.x, expectedRetractPose.position.x, 1e-5);
    EXPECT_NEAR(retractPose.position.y, expectedRetractPose.position.y, 1e-5);
    EXPECT_NEAR(retractPose.position.z, expectedRetractPose.position.z, 1e-5);
    EXPECT_NEAR(retractPose.orientation.x, expectedRetractPose.orientation.x, 1e-5);
    EXPECT_NEAR(retractPose.orientation.y, expectedRetractPose.orientation.y, 1e-5);
    EXPECT_NEAR(retractPose.orientation.z, expectedRetractPose.orientation.z, 1e-5);
    EXPECT_NEAR(retractPose.orientation.w, expectedRetractPose.orientation.w, 1e-5);
}

TEST(PickPoseUtilsTest, testRotatePoseSingleAxis)
{
    geometry_msgs::msg::Pose pose;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;

    // Test 45 degrees rotation around the Z-axis (yaw)
    double yaw = RAD_45DEG; // 45 degrees
    double pitch = 0.0;
    double roll = 0.0;

    geometry_msgs::msg::Pose rotatedPose = pick_solution_finder::PickPoseUtils::rotatePose(pose, roll, pitch, yaw);

    // Convert the resulting quaternion to RPY
    tf2::Quaternion q(
        rotatedPose.orientation.x,
        rotatedPose.orientation.y,
        rotatedPose.orientation.z,
        rotatedPose.orientation.w);
    double resultRoll, resultPitch, resultYaw;
    tf2::Matrix3x3(q).getRPY(resultRoll, resultPitch, resultYaw);

    // Expected RPY values
    double expectedRoll = roll;
    double expectePitch = pitch;
    double expectedYaw = yaw;

    // Compare the resulting RPY with the expected values
    EXPECT_NEAR(resultRoll, expectedRoll, 1e-5);
    EXPECT_NEAR(resultPitch, expectePitch, 1e-5);
    EXPECT_NEAR(resultYaw, expectedYaw, 1e-5);
}

TEST(PickPoseUtilsTest, testRotatePoseCombined)
{
    geometry_msgs::msg::Pose pose;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;

    // Test combined rotation around X, Y, and Z axes
    double yaw = RAD_45DEG;  // 45 degrees
    double pitch = M_PI / 6; // 30 degrees
    double roll = M_PI / 3;  // 60 degrees

    geometry_msgs::msg::Pose rotatedPose = pick_solution_finder::PickPoseUtils::rotatePose(pose, roll, pitch, yaw);

    // Convert the resulting quaternion to RPY
    tf2::Quaternion q(
        rotatedPose.orientation.x,
        rotatedPose.orientation.y,
        rotatedPose.orientation.z,
        rotatedPose.orientation.w);
    double resultRoll, resultPitch, resultYaw;
    tf2::Matrix3x3(q).getRPY(resultRoll, resultPitch, resultYaw);

    // Expected RPY values
    double expectedRoll = roll;
    double expectedPitch = pitch;
    double expectedYaw = yaw;

    // Compare the resulting RPY with the expected values
    EXPECT_NEAR(resultRoll, expectedRoll, 1e-5);
    EXPECT_NEAR(resultPitch, expectedPitch, 1e-5);
    EXPECT_NEAR(resultYaw, expectedYaw, 1e-5);
}

TEST(PickPoseUtilsTest, testRotatePoseWithInitialRotation)
{
    // Create a pose with an initial rotation
    geometry_msgs::msg::Pose pose;
    tf2::Quaternion initialQuaternion;
    double initialRoll = M_PI / 6;   // 30 degrees roll,
    double initialPitch = RAD_45DEG; // 45 degrees pitch
    double initialYaw = M_PI / 3;    // 60 degrees yaw
    initialQuaternion.setRPY(initialRoll, initialPitch, initialYaw);
    pose.orientation.x = initialQuaternion.x();
    pose.orientation.y = initialQuaternion.y();
    pose.orientation.z = initialQuaternion.z();
    pose.orientation.w = initialQuaternion.w();

    // Create the pose RPY modifications
    double rollModification = 0.0;
    double pitchModification = 0.0;
    double yawModification = RAD_45DEG; // 45 degrees

    // Rotate the pose with the modifications
    geometry_msgs::msg::Pose rotatedPose =
        pick_solution_finder::PickPoseUtils::rotatePose(pose, rollModification, pitchModification, yawModification);

    // Convert the resulting quaternion of the rotated pose
    tf2::Quaternion resultQuaternion(
        rotatedPose.orientation.x,
        rotatedPose.orientation.y,
        rotatedPose.orientation.z,
        rotatedPose.orientation.w);
    // Convert the resulting quaternion to RPY
    double resultRoll;
    double resultPitch;
    double resultYaw;
    tf2::Matrix3x3(resultQuaternion).getRPY(resultRoll, resultPitch, resultYaw);

    // Hardcoded expected values
    double expectedRoll = 0.886077;  // 50.77 degrees in radians
    double expectedPitch = 0.25268;  // 14.48 degrees in radians
    double expectedYaw = 1.73192;    // 99.23 degrees in radians

    // Compare the resulting RPY with the expected values
    EXPECT_NEAR(resultRoll, expectedRoll, 1e-5);
    EXPECT_NEAR(resultPitch, expectedPitch, 1e-5);
    EXPECT_NEAR(resultYaw, expectedYaw, 1e-5);
}
