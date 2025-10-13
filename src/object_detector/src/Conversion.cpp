#include "Conversion.hpp"

#include <tf2/LinearMath/Quaternion.h> // tf2::Quaternion

#include "Constants.hpp"

/* static */ geometry_msgs::msg::Pose Conversion::matrixPointToPose(
    float row,
    float col,
    float height,
    float roll,
    float pitch,
    float yaw)
{
    geometry_msgs::msg::Pose pose;
    float cellSizeFloat = static_cast<float>(CELL_SIZE);
    // NOLINTNEXTLINE(bugprone-integer-division) // the define devides two ints, and we require float precision
    float originXFloat = static_cast<float>(MATRIX_ORIGIN_X);
    // NOLINTNEXTLINE(bugprone-integer-division) // the define devides two ints, and we require float precision
    float originYFloat = static_cast<float>(MATRIX_ORIGIN_Y);
    pose.position.x = (row - originXFloat) * cellSizeFloat * 0.001F;
    pose.position.y = (col - originYFloat) * cellSizeFloat * 0.001F;
    pose.position.z = height * 0.001F;

    tf2::Quaternion quaternion;
    quaternion.setRPY(roll, pitch, yaw);
    pose.orientation.x = quaternion.x();
    pose.orientation.y = quaternion.y();
    pose.orientation.z = quaternion.z();
    pose.orientation.w = quaternion.w();
    return pose;
}
