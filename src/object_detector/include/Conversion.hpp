#ifndef CONVERSION_H
#define CONVERSION_H

#include <sensor_msgs/msg/point_cloud2.hpp> // sensor_msgs::msg::PointCloud2
#include <geometry_msgs/msg/pose.hpp>       // geometry_msgs::msg::Pose
#include "Cell.hpp"

template <class T>
class Matrix; // due to circular dependency

/**
 * @class Conversion
 * @brief The Conversion class provides methods for conversion between pointclouds, matrices, and cells.
 */
class Conversion
{
public:
    // Delete anything that can make this class instantiable or copyable
    Conversion() = delete;
    ~Conversion() = delete;
    Conversion(const Conversion&) = delete;
    Conversion& operator=(const Conversion&) = delete;
    /**
     * @brief Convert a pointcloud message to a matrix.
     *
     * @tparam T type of value in the matrix
     * @param msg pointcloud message
     * @return Matrix<T> the matrix
     */
    template <typename T>
    static Matrix<T> pointcloudToMatrix(const sensor_msgs::msg::PointCloud2::SharedPtr& msg);
    /**
     * @brief Convert a matrix to a pointcloud message.
     *
     * @tparam T type of value in the matrix
     * @param input the matrix
     * @return sensor_msgs::msg::PointCloud2 the pointcloud message
     */
    template <typename T>
    static sensor_msgs::msg::PointCloud2 matrixToPointcloud(Matrix<T>& input);
    /**
     * @brief Convert a point in the matrix to a pose in the world frame.
     *
     * @param row row index
     * @param col column index
     * @param height height
     * @param roll roll
     * @param pitch pitch
     * @param yaw yaw
     * @return geometry_msgs::msg::Pose the pose
     */
    static geometry_msgs::msg::Pose matrixPointToPose(
        float row,
        float col,
        float height,
        float roll = 0,
        float pitch = 0,
        float yaw = 0);
    /**
     * @brief Convert a cell in the matrix to a pose in the world frame.
     *
     * @tparam T type of value in the cell
     * @param point the cell
     * @return geometry_msgs::msg::Pose the pose
     */
    template <typename T>
    static geometry_msgs::msg::Pose matrixCellToPose(Cell<T>& point);
private:
};

#include "Conversion.inc"

#endif