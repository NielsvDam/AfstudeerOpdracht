#ifndef MATRIXSEGMENT_HPP
#define MATRIXSEGMENT_HPP

#include <vector>                     // std::vector
#include <cstddef>                    // std::size_t
#include <geometry_msgs/msg/pose.hpp> // geometry_msgs::msg::Pose

#include "Cell.hpp"
#include "Constants.hpp"

template <class T>
class Matrix; // due to circular dependency

/**
 * @class MatrixSegment
 * @brief The MatrixSegment class provides a matrix that is a segment of a larger matrix.
 */
template <typename T>
class MatrixSegment : public Matrix<T>
{
public:
    virtual ~MatrixSegment();
    MatrixSegment() = delete; /* the matrix should never be created without a size */
    /**
     * @brief Construct a new MatrixSegment object.
     *
     * @param rows the number of rows
     * @param cols the number of columns
     * @param rowOrigin the row origin of the segment in the original matrix
     * @param colOrigin the column origin of the segment in the original matrix
     */
    MatrixSegment(std::size_t rows, std::size_t cols, std::size_t rowOrigin, std::size_t colOrigin);
    /**
     * @brief Executes the detection algorithm.
     */
    void detectBlocks();
    /**
     * @brief Get the row origin of the segment in the original matrix.
     *
     * @return std::size_t the row origin
     */
    std::size_t getRowOrigin() const;
    /**
     * @brief Get the column origin of the segment in the original matrix.
     *
     * @return std::size_t the column origin
     */
    std::size_t getColOrigin() const;
    /**
     * @brief Get the number of cells in the segment.
     *
     * @return std::size_t the number of cells
     */
    std::size_t getCellCount();
    /**
     * @brief Get the number of found blocks in the segment.
     *
     * @return std::size_t the number of blocks
     */
    std::size_t getBlockCount();
    /**
     * @brief Get the poses of the found blocks. The poses are offset from the origin of the segment.
     *
     * @return std::vector<geometry_msgs::msg::Pose> the offset poses
     */
    std::vector<geometry_msgs::msg::Pose> getOffsetPoses();
private:
    /**
     * @brief Get the occupied cells in the segment.
     *
     * @return std::vector<Cell<T>> the occupied cells
     */
    std::vector<Cell<T>> getOccupiedCells();
    
    std::size_t rowOrigin;                       /* the row origin of the segment in the original matrix */
    std::size_t colOrigin;                       /* the column origin of the segment in the original matrix */
    std::vector<geometry_msgs::msg::Pose> poses; /* the poses of the found blocks */
};

#include "MatrixSegment.inc"

#endif /* MATRIXSEGMENT_HPP */