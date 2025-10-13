#ifndef MATRIXFILTERS_HPP
#define MATRIXFILTERS_HPP

#include <vector>                           // std::vector
#include <cstddef>                          // std::size_t
#include <cstdint>                          // uint8_t
#include <sensor_msgs/msg/point_cloud2.hpp> // sensor_msgs::msg::PointCloud2
#include <geometry_msgs/msg/pose.hpp>       // geometry_msgs::msg::Pose

#include "Matrix.hpp"
#include "Constants.hpp"

class MatrixFilters
{
public:
    // Delete anything that can make this class instantiable or copyable
    MatrixFilters() = delete;
    ~MatrixFilters() = delete;
    MatrixFilters(const MatrixFilters&) = delete;
    MatrixFilters& operator=(const MatrixFilters&) = delete;
    /**
     * @brief Filter out NaN values in the matrix by replacing them with the nearest non-NaN value.
     *
     * @tparam T type of value in the matrix
     * @param matrix the matrix
     */
    template <typename T>
    static void nanFilter(Matrix<T>& matrix);
    /**
     * @brief Extract the (top) sufaces in the matrix and filter out the rest.
     *
     * @tparam T type of value in the matrix
     * @param vector the vector
     */
    template <typename T>
    static void sufaceExtractionFilter(Matrix<T>& matrix);

    template <typename T>
    static Matrix<T> morphOpen(Matrix<T>& matrix, uint8_t kernelSize, uint8_t iterations);

    template <typename T>
    static Matrix<T> erode(Matrix<T>& input, uint8_t kernelSize);

    template <typename T>
    static Matrix<T> dilate(Matrix<T>& input, uint8_t kernelSize);
private:
    /**
     * @brief Extract the (top) sufaces in the vector and filter out the rest.
     *
     * @tparam T type of value in the vector
     * @param vector the vector
     */
    template <typename T>
    static void sufaceExtractionFilter(std::vector<T>& vector);
};

#include "MatrixFilters.inc"

#endif /* MATRIXFILTERS_HPP */
