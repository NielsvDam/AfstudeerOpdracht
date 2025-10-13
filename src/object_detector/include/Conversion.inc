#include <sensor_msgs/point_cloud2_iterator.hpp>
#include "Constants.hpp"

#include "Matrix.hpp"

template <typename T>
Matrix<T> Conversion::pointcloudToMatrix(const sensor_msgs::msg::PointCloud2::SharedPtr& msg)
{
    if (!msg)
    {
        throw std::runtime_error("Received nullptr pointcloud message");
    }
    Matrix<T> matrix(MATRIX_ROWS, MATRIX_COLUMNS, NAN);

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
    {
        // Compute the cell index with offset
        float x = *iter_x;
        float y = *iter_y;
        float roundedX = std::round(x * 1000.0 / CELL_SIZE);
        float roundedY = std::round(y * 1000.0 / CELL_SIZE);
        roundedX += MATRIX_ORIGIN_X;
        roundedY += MATRIX_ORIGIN_Y;
        if (roundedX < 0.0 || roundedY < 0.0 || roundedX >= matrix.getRows() || roundedY >= matrix.getCols())
        {
            continue;
        }
        std::size_t row = static_cast<std::size_t>(roundedX);
        std::size_t column = static_cast<std::size_t>(roundedY);
        float measurement = *iter_z;
        float cell = matrix.at(row, column);
        // Update the matrix only if the measurement is lower than the current value.
        // This is done because the pointcloud contains multiple measurements in the same cell. Averaging them is not
        // correct, since not all measurements reffer to the same point in space. The data contains measurements of
        // points 'under' a object and we are not interested in this.
        if ((std::isnan(cell) || (measurement < cell && measurement != 0.0)))
        {
            matrix.at(row, column) = measurement * 1000; // Convert meters to mm
        }
    }

    return matrix;
}

template <typename T>
sensor_msgs::msg::PointCloud2 Conversion::matrixToPointcloud(Matrix<T>& input)
{
    sensor_msgs::msg::PointCloud2 output;
    output.height = 1;
    output.width = input.getRows() * input.getCols();
    output.is_dense = false;
    output.is_bigendian = false;
    output.fields.resize(3);
    output.fields[0].name = "x";
    output.fields[0].offset = 0;
    output.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    output.fields[0].count = 1;
    output.fields[1].name = "y";
    output.fields[1].offset = 4;
    output.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    output.fields[1].count = 1;
    output.fields[2].name = "z";
    output.fields[2].offset = 8;
    output.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    output.fields[2].count = 1;
    output.point_step = 12;
    output.row_step = output.point_step * output.width;
    output.data.resize(output.row_step * output.height);

    // Fill the PointCloud2 message with the matrix data
    sensor_msgs::PointCloud2Iterator<float> out_x(output, "x");
    sensor_msgs::PointCloud2Iterator<float> out_y(output, "y");
    sensor_msgs::PointCloud2Iterator<float> out_z(output, "z");

    for (std::size_t row = 0; row < input.getRows(); ++row)
    {
        for (std::size_t col = 0; col < input.getCols(); ++col, ++out_x, ++out_y, ++out_z)
        {
            *out_x = (static_cast<float>(row) - MATRIX_ORIGIN_X) * CELL_SIZE / 1000.0; // Convert mm back to meters and
                                                                                       // adjust by offsetX
            *out_y = (static_cast<float>(col) - MATRIX_ORIGIN_Y) * CELL_SIZE / 1000.0; // Convert mm back to meters and
                                                                                       // adjust by offsetY
            *out_z = input.at(row, col) / 1000.0;                                      // Convert mm back to meters
        }
    }
    return output;
}

template <typename T>
geometry_msgs::msg::Pose Conversion::matrixCellToPose(Cell<T>& point)
{
    return matrixPointToPose(point.row, point.col, point.value, 0, 0, 0);
}
