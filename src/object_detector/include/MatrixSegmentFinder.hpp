#ifndef CLUSTERALGORITHMS_HPP
#define CLUSTERALGORITHMS_HPP

#include <vector>  // std::vector
#include <cstddef> // std::size_t

#include "Constants.hpp"
#include "Cell.hpp"
#include "Matrix.hpp"
#include "MatrixSegment.hpp"

/**
 * @class MatrixSegmentFinder
 * @brief The MatrixSegmentFinder class provides functionality to find segments in a matrix.
 */
template <typename T>
class MatrixSegmentFinder
{
public:
    /**
     * @brief Constructs a new MatrixSegmentFinder.
     *
     * @tparam T type of value in the matrix
     * @param matrix the matrix
     */
    explicit MatrixSegmentFinder(Matrix<T>& matrix);
    /**
     * @brief Get the segments found in the matrix.
     *
     * @return std::vector<MatrixSegment<T>>& The segments found in the matrix.
     */
    std::vector<MatrixSegment<T>>& getSegments();
    /**
     * @brief Get the count of segments found.
     *
     * @return std::size_t The count of segments found.
     */
    std::size_t getSegmentCount() const;
private:
    /**
     * @brief Find clusters of cells in the matrix.
     *
     * @param matrix the matrix
     * @return std::vector<std::vector<Cell<T>>> Clusters of cells in the matrix.
     */
    std::vector<std::vector<Cell<T>>> findClusteredCells();
    /**
     * @brief Check if a cell is valid to visit.
     *
     * Checks the following conditions:
     * - the cell is within bounds
     * - the cell is not visited before
     * - the cell is not NaN
     *
     * @param row the row of the cell
     * @param col the column of the cell
     * @return bool true if the cell is valid to visit, false otherwise.
     */
    bool isValidToVisit(std::size_t row, std::size_t col);
    /**
     * @brief Perform a depth first search.
     *
     * @param startCell the cell to start the search from
     * @return std::vector<Cell<T>> The cluster of cells found.
     */
    std::vector<Cell<T>> depthFirstSearch(const Cell<T>& startCell);

    Matrix<T>& matrix;                            /* The matrix to find segments in. */
    std::vector<MatrixSegment<T>> matrixSegments; /* The segments found in the matrix. */
    Matrix<uint8_t> visited; /* The matrix to store visited cells (can't be type bool due to a c++ limitation) */
};

#include "MatrixSegmentFinder.inc"

#endif /* CLUSTERALGORITHMS_HPP */