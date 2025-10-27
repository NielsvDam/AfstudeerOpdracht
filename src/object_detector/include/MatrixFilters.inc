template <typename T>
void MatrixFilters::nanFilter(Matrix<T>& matrix)
{
    // iterate over all cells
    for (std::size_t i = 0; i < matrix.getRows(); ++i)
    {
        for (std::size_t j = 0; j < matrix.getCols(); ++j)
        {
            // check if the cell is NaN
            if (std::isnan(matrix.at(i, j)))
            {
                float highest_value = -std::numeric_limits<float>::infinity();
                static const Matrix<int8_t> directions{{0, -1}, {0, 1}, {-1, 0}, {1, 0}}; // left, right, up, down
                // scan in all directions and find the highest value
                for (const std::vector<int8_t>& direction : directions)
                {
                    // step one into the direction
                    int curRow = i + direction.at(0);
                    int curCol = j + direction.at(1);
                    int rows = static_cast<int>(matrix.getRows()); // to compare with equal signedness
                    int cols = static_cast<int>(matrix.getCols()); // to compare with equal signedness
                    // keep scanning until a non-NaN value is found
                    while (curRow >= 0 && curRow < rows && curCol >= 0 && curCol < cols)
                    {
                        if (!std::isnan(matrix.at(curRow, curCol)))
                        {
                            highest_value = std::max(highest_value, matrix.at(curRow, curCol));
                            break;
                        }
                        // step one further into the direction
                        curRow += direction.at(0);
                        curCol += direction.at(1);
                    }
                }
                // only update if a value was found.
                if (highest_value != -std::numeric_limits<float>::infinity())
                {
                    matrix.at(i, j) = highest_value;
                }
            }
        }
    }
}

template <typename T>
void MatrixFilters::sufaceExtractionFilter(Matrix<T>& matrix)
{
    // collect the higher portions of each row
    Matrix<T> rowFilteredMatrix(matrix);
    for (std::size_t col = 0; col < matrix.getCols(); ++col)
    {
        std::vector<T> columnVector = matrix.getColumnVector(col);
        MatrixFilters::sufaceExtractionFilter(columnVector);
        rowFilteredMatrix.setColumnVector(columnVector, col);
    }
    // collect the higher portions of each column
    Matrix<T> columnFilteredMatrix(matrix);
    for (std::size_t row = 0; row < matrix.getRows(); ++row)
    {
        std::vector<T> rowVector = matrix.getRowVector(row);
        MatrixFilters::sufaceExtractionFilter(rowVector);
        columnFilteredMatrix.setRowVector(rowVector, row);
    }
    matrix = rowFilteredMatrix;
    matrix.merge(columnFilteredMatrix);
}

template <typename T>
void MatrixFilters::sufaceExtractionFilter(std::vector<T>& vector)
{
    std::vector<T> resultVector(vector.size(), NAN);
    bool collectingSurface = false;
    for (std::size_t i = 0; i < vector.size(); ++i)
    {
        float curValue = vector.at(i);
        if (!collectingSurface)
        {
            float greatestWindowValue = NAN;
            for (std::size_t j = 1; j <= SURFACE_COLLECT_HISTORY_WINDOW_SIZE; ++j)
            {
                if (i < j) // Check if windowIndex would be out of bounds
                {
                    continue;
                }
                std::size_t windowIndex = i - j;
                float windowValue = vector.at(windowIndex);
                if (std::isnan(windowValue))
                {
                    continue;
                }
                if (std::isnan(greatestWindowValue) || windowValue > greatestWindowValue)
                {
                    greatestWindowValue = windowValue;
                }
            }
            float diff = greatestWindowValue - curValue;
            if (diff > BLOCK_SIZE - BLOCK_HEIGHT_INDICATION_TOLERANCE)
            {
                collectingSurface = true;
                resultVector.at(i) = curValue;
            }
        }
        else
        {
            if (i == 0) // Ensure we dont go out of bound (though collectingSurface can only be true if i > 0)
            {
                continue;
            }
            float prevValue = vector.at(i - 1);
            float diff = std::abs(curValue - prevValue);
            if (diff < SURFACE_COLLECT_MAX_DIFF)
            {
                resultVector.at(i) = curValue;
            }
            else
            {
                collectingSurface = false;
            }
        }
    }
    vector = resultVector;
}

template <typename T>
Matrix<T> MatrixFilters::morphOpen(Matrix<T>& matrix, uint8_t kernelSize, uint8_t iterations)
{
    Matrix<T> result = matrix;
    for (uint8_t i = 0; i < iterations; i++)
    {
        result = MatrixFilters::erode(result, kernelSize);
        result = MatrixFilters::dilate(result, kernelSize);
    }
    return result;
}

template <typename T>
Matrix<T> MatrixFilters::erode(Matrix<T>& input, uint8_t kernelSize)
{
    // TODO: Implement this function
    return input;
}

template <typename T>
Matrix<T> MatrixFilters::dilate(Matrix<T>& input, uint8_t kernelSize)
{
    // TODO: Implement this function
    return input;
}