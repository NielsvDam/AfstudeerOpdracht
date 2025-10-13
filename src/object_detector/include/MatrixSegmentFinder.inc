#include <stack>  // std::stack
#include <limits> // std::numeric_limits
#include <cmath>  // NAN, std::isnan

template <typename T>
MatrixSegmentFinder<T>::MatrixSegmentFinder(Matrix<T>& matrix)
    : matrix(matrix), matrixSegments(), visited(matrix.getRows(), matrix.getCols(), false)
{
    std::vector<std::vector<Cell<T>>> cellClusters = findClusteredCells();
    for (const std::vector<Cell<T>>& cellCluster : cellClusters)
    {
        // find the bounding box of the cluster
        std::size_t minR = std::numeric_limits<std::size_t>::max();
        std::size_t maxR = 0;
        std::size_t minC = std::numeric_limits<std::size_t>::max();
        std::size_t maxC = 0;
        // iterate over the cells to find the bounding box,
        for (const Cell<T>& cell : cellCluster)
        {
            minR = std::min(minR, cell.row);
            maxR = std::max(maxR, cell.row);
            minC = std::min(minC, cell.col);
            maxC = std::max(maxC, cell.col);
        }
        // determine the origin of the matrix segment
        std::size_t originColumn = minC;
        std::size_t originRow = minR;
        // determine the size of the matrix segment
        std::size_t rows = maxR - minR + 1;
        std::size_t cols = maxC - minC + 1;
        // create a matrix segment
        MatrixSegment<T> matrixSegment(rows, cols, originRow, originColumn);
        // populate the matrix segment with the values from the cluster
        for (const Cell<T>& cell : cellCluster)
        {
            // determine where in the matrix segment the cell should be placed.
            std::size_t targetRow = cell.row - originRow;
            std::size_t targetCol = cell.col - originColumn;
            // set the cell in the matrix segment
            matrixSegment.at(targetRow, targetCol) = cell.value;
        }
        // add the matrix segment to the vector
        matrixSegments.push_back(matrixSegment);
    }
}

template <typename T>
std::vector<MatrixSegment<T>>& MatrixSegmentFinder<T>::getSegments()
{
    return matrixSegments;
}

template <typename T>
bool MatrixSegmentFinder<T>::isValidToVisit(std::size_t row, std::size_t col)
{
    // check if the cell is within boundsq
    if (row >= matrix.getRows() || col >= matrix.getCols())
    {
        return false;
    }
    // check if the cell is NaN
    bool isNan = std::isnan(matrix.at(row, col));
    // check if the cell is visited
    bool isVisited = visited.at(row, col);
    return !isNan && !isVisited;
}

template <typename T>
std::size_t MatrixSegmentFinder<T>::getSegmentCount() const
{
    return matrixSegments.size();
}

template <typename T>
std::vector<Cell<T>> MatrixSegmentFinder<T>::depthFirstSearch(const Cell<T>& startCell)
{
    std::vector<Cell<T>> clusterOfPoints; // vector to store the cells that form a cluster
    std::stack<Cell<T>> stack;            // stack to store the cells to be visited
    stack.push(startCell);                // push the starting cell to the stack
    while (!stack.empty())
    {
        Cell<T> cell = stack.top();             // get the top cell from the stack
        stack.pop();                            // remove the top cell from the stack
        if (isValidToVisit(cell.row, cell.col)) // check if the cell can be visited
        {
            visited.at(cell.row, cell.col) = true; // found a valid cell, mark it as visited
            clusterOfPoints.push_back(cell);       // add the cell to the points vector
            static const Matrix<int8_t> directions = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}}; // up, down, left, right
            // perform search in all four directions
            for (const std::vector<int8_t>& direction : directions)
            {
                int newRow = cell.row + direction.at(0); // calculate the new row
                int newCol = cell.col + direction.at(1); // calculate the new column
                // check if the cell is valid to visit (not visited before, within bounds, and not NaN)
                if (isValidToVisit(newRow, newCol))
                {
                    // push the new cell to the stack
                    Cell<T> newCell(newRow, newCol, matrix.at(newRow, newCol));
                    stack.push(newCell);
                }
            }
        }
    }
    return clusterOfPoints;
}

template <typename T>
std::vector<std::vector<Cell<T>>> MatrixSegmentFinder<T>::findClusteredCells()
{
    std::vector<std::vector<Cell<T>>> clusters; // vector to store the clusters
    // iterate over the matrix.
    for (std::size_t row = 0; row < matrix.getRows(); ++row)
    {
        for (std::size_t col = 0; col < matrix.getCols(); ++col)
        {
            // check if the cell is valid to visit (not visited before, within bounds, and not NaN)
            if (isValidToVisit(row, col))
            {
                // find clusters of cells
                Cell<T> cell(row, col, matrix.at(row, col));
                std::vector<Cell<T>> points = depthFirstSearch(cell);
                // store the cluster in the vector
                clusters.push_back(points);
            }
        }
    }
    return clusters;
}
