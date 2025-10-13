#include <gtest/gtest.h>
#include "Conversion.hpp"
#include "Matrix.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose.hpp>

TEST(ConversionClass, pointcloudToMatrix)
{
    // Create a dummy PointCloud2 message
    auto pointcloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pointcloud->height = 1;
    pointcloud->width = MATRIX_ROWS * MATRIX_COLUMNS;
    pointcloud->is_dense = false;
    pointcloud->is_bigendian = false;
    pointcloud->point_step = 12;
    pointcloud->row_step = pointcloud->point_step * pointcloud->width;
    pointcloud->data.resize(pointcloud->row_step * pointcloud->height);

    // Define the fields
    sensor_msgs::msg::PointField fieldX;
    fieldX.name = "x";
    fieldX.offset = 0;
    fieldX.datatype = sensor_msgs::msg::PointField::FLOAT32;
    fieldX.count = 1;
    pointcloud->fields.push_back(fieldX);

    sensor_msgs::msg::PointField fieldY;
    fieldY.name = "y";
    fieldY.offset = 4;
    fieldY.datatype = sensor_msgs::msg::PointField::FLOAT32;
    fieldY.count = 1;
    pointcloud->fields.push_back(fieldY);

    sensor_msgs::msg::PointField fieldZ;
    fieldZ.name = "z";
    fieldZ.offset = 8;
    fieldZ.datatype = sensor_msgs::msg::PointField::FLOAT32;
    fieldZ.count = 1;
    pointcloud->fields.push_back(fieldZ);

    // Fill the data with points using PointCloud2Iterator
    sensor_msgs::PointCloud2Iterator<float> iter_x(*pointcloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*pointcloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*pointcloud, "z");

    for (std::size_t row = 0; row < MATRIX_ROWS; ++row)
    {
        for (std::size_t col = 0; col < MATRIX_COLUMNS; ++col, ++iter_x, ++iter_y, ++iter_z)
        {
            *iter_x = (static_cast<float>(row) - MATRIX_ORIGIN_X) * CELL_SIZE / 1000.0;
            *iter_y = (static_cast<float>(col) - MATRIX_ORIGIN_Y) * CELL_SIZE / 1000.0;
            *iter_z = static_cast<float>(row * MATRIX_COLUMNS + col) / 1000.0; // Convert to meters
        }
    }

    // Call the function
    Matrix<float> matrix = Conversion::pointcloudToMatrix<float>(pointcloud);

    // Check the matrix values
    for (std::size_t row = 0; row < MATRIX_ROWS; ++row)
    {
        for (std::size_t col = 0; col < MATRIX_COLUMNS; ++col)
        {
            float expected_value = static_cast<float>(row * MATRIX_COLUMNS + col);
            EXPECT_FLOAT_EQ(matrix.at(row, col), expected_value);
        }
    }
}

TEST(ConversionClass, matrixToPointcloud)
{
    // Create a dummy matrix with appropriate size
    Matrix<float> matrix(MATRIX_ROWS, MATRIX_COLUMNS);

    // This code initializes a matrix with values starting from 0 and increasing by 1 for each cell.
    // The matrix is filled row by row, with each cell in a row being assigned a value that is one greater than the previous
    for (std::size_t row = 0; row < MATRIX_ROWS; ++row)
    {
        for (std::size_t col = 0; col < MATRIX_COLUMNS; ++col)
        {
            matrix.at(row, col) = static_cast<float>(row * MATRIX_COLUMNS + col);
        }
    }

    // Call the function
    sensor_msgs::msg::PointCloud2 pointcloud = Conversion::matrixToPointcloud(matrix);

    // Check the pointcloud values using a for loop
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(pointcloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(pointcloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(pointcloud, "z");

    for (std::size_t row = 0; row < MATRIX_ROWS; ++row)
    {
        for (std::size_t col = 0; col < MATRIX_COLUMNS; ++col, ++iter_x, ++iter_y, ++iter_z)
        {
            float expected_x = (static_cast<float>(row) - MATRIX_ORIGIN_X) * CELL_SIZE / 1000.0;
            float expected_y = (static_cast<float>(col) - MATRIX_ORIGIN_Y) * CELL_SIZE / 1000.0;
            float expected_z = matrix.at(row, col) / 1000.0;

            EXPECT_FLOAT_EQ(*iter_x, expected_x);
            EXPECT_FLOAT_EQ(*iter_y, expected_y);
            EXPECT_FLOAT_EQ(*iter_z, expected_z);
        }
    }
}

TEST(ConversionClass, matrixPointToPose)
{
    float row = 5.4;
    float col = 5.6;
    float height = 10.0;
    float roll = 0;
    float pitch = 0;
    float yaw = 0;

    geometry_msgs::msg::Pose pose = Conversion::matrixPointToPose(row, col, height, roll, pitch, yaw);
    // should be type float for multiplications
    float orginX = static_cast<float>(MATRIX_ORIGIN_X);
    float orginY = static_cast<float>(MATRIX_ORIGIN_Y);
    float cellSize = static_cast<float>(CELL_SIZE);
    // Adjust the calculations to account for the matrix origin being in the middle
    float x = (row - orginX) * cellSize * 0.001;
    float y = (col - orginY) * cellSize * 0.001;
    float z = height * 0.001;

    EXPECT_FLOAT_EQ(pose.position.x, x);
    EXPECT_FLOAT_EQ(pose.position.y, y);
    EXPECT_FLOAT_EQ(pose.position.z, z);

    tf2::Quaternion quaternion;
    quaternion.setRPY(roll, pitch, yaw);
    EXPECT_FLOAT_EQ(pose.orientation.x, quaternion.x());
    EXPECT_FLOAT_EQ(pose.orientation.y, quaternion.y());
    EXPECT_FLOAT_EQ(pose.orientation.z, quaternion.z());
    EXPECT_FLOAT_EQ(pose.orientation.w, quaternion.w());
}

TEST(ConversionClass, matrixCellToPose)
{
    Cell<float> cell(5, 5, 10.0);

    geometry_msgs::msg::Pose pose = Conversion::matrixCellToPose(cell);
    // should be type float for multiplications
    float orginX = static_cast<float>(MATRIX_ORIGIN_X);
    float orginY = static_cast<float>(MATRIX_ORIGIN_Y);
    float cellSize = static_cast<float>(CELL_SIZE);
    // Adjust the calculations to account for the matrix origin being in the middle
    float x = (cell.row - orginX) * cellSize * 0.001;
    float y = (cell.col - orginY) * cellSize * 0.001;
    float z = cell.value * 0.001;

    EXPECT_FLOAT_EQ(pose.position.x, x);
    EXPECT_FLOAT_EQ(pose.position.y, y);
    EXPECT_FLOAT_EQ(pose.position.z, z);
}