#include <cassert>            // assert
#include <stdexcept>          // std::invalid_argument, std::out_of_range
#include <cmath>              // std::isnan, std::abs
#include <utility>            // std::move, std::swap
#include <algorithm>          // std::any_of, std::copy
#include <opencv2/opencv.hpp> // for writeMatrixToImage()
#include <fstream>            // for writeMatrixToCSV()

#include "MatrixSegment.hpp"
/**
 *
 */
template <class T>
Matrix<T>::Matrix(std::size_t rows, std::size_t cols, T value) : matrix(rows, std::vector<T>(cols, value))
{
    if (rows == 0 && cols == 0)
    {
        throw std::invalid_argument("Matrix must have at least 1 row and 1 column");
    }
    if (rows == 0)
    {
        throw std::invalid_argument("Matrix must have at least 1 row");
    }
    if (cols == 0)
    {
        throw std::invalid_argument("Matrix must have at least 1 column");
    }
}
/**
 *
 */
template <class T>
Matrix<T>::Matrix(const std::initializer_list<std::initializer_list<T>>& aList)
{
    if (aList.size() == 0 || (*aList.begin()).size() == 0)
    {
        throw std::invalid_argument("Matrix must have at least 1 row and 1 column");
    }

    auto cols = (*aList.begin()).size();
    if (std::any_of(aList.begin(), aList.end(), [cols](const std::initializer_list<T>& row) { return row.size() != cols; }))
    {
        throw std::invalid_argument("All rows must have the same number of columns");
    }

    std::size_t rows = aList.size();
    matrix.resize(rows);
    for (std::size_t i = 0; i < rows; ++i)
    {
        matrix[i].resize(cols);
        std::copy(aList.begin()[i].begin(), aList.begin()[i].end(), matrix[i].begin());
    }
}
/**
 *
 */
template <class T>
Matrix<T>::Matrix(const Matrix<T>& aMatrix) : matrix(aMatrix.matrix)
{}
/**
 *
 */
template <class T>
std::vector<T>& Matrix<T>::at(std::size_t aRowIndex)
{
    if (aRowIndex >= getRows())
    {
        throw std::out_of_range(
            "Row index " + std::to_string(aRowIndex) + " out of range. Matrix has " + std::to_string(getRows()) + " rows.");
    }
    return matrix.at(aRowIndex);
}
/**
 *
 */
template <class T>
const std::vector<T>& Matrix<T>::at(std::size_t aRowIndex) const
{
    if (aRowIndex >= getRows())
    {
        throw std::out_of_range(
            "Row index " + std::to_string(aRowIndex) + " out of range. Matrix has " + std::to_string(getRows()) + " rows.");
    }
    return matrix.at(aRowIndex);
}
/**
 *
 */
template <class T>
T& Matrix<T>::at(std::size_t aRowIndex, std::size_t aColumnIndex)
{
    if (aRowIndex >= getRows())
    {
        throw std::out_of_range(
            "Row index " + std::to_string(aRowIndex) + " out of range. Matrix has " + std::to_string(getRows()) + " rows.");
    }
    if (aColumnIndex >= getCols())
    {
        throw std::out_of_range(
            "Column index " + std::to_string(aColumnIndex) + " out of range. Matrix has " + std::to_string(getCols()) +
            " columns.");
    }

    return at(aRowIndex).at(aColumnIndex);
}
/**
 *
 */
template <class T>
const T& Matrix<T>::at(std::size_t aRowIndex, std::size_t aColumnIndex) const
{
    if (aRowIndex >= getRows())
    {
        throw std::out_of_range(
            "Row index " + std::to_string(aRowIndex) + " out of range. Matrix has " + std::to_string(getRows()) + " rows.");
    }
    if (aColumnIndex >= getCols())
    {
        throw std::out_of_range(
            "Column index " + std::to_string(aColumnIndex) + " out of range. Matrix has " + std::to_string(getCols()) +
            " columns.");
    }
    return at(aRowIndex).at(aColumnIndex);
}
/**
 *
 */
template <class T>
std::vector<T>& Matrix<T>::operator[](std::size_t aRowIndex)
{
    return at(aRowIndex);
}
/**
 *
 */
template <class T>
const std::vector<T>& Matrix<T>::operator[](std::size_t aRowIndex) const
{
    return at(aRowIndex);
}

/**
 *
 */
template <typename T>
std::vector<T> Matrix<T>::getRowVector(std::size_t aRowIndex) const
{
    return at(aRowIndex);
}

template <typename T>
std::vector<T> Matrix<T>::getColumnVector(std::size_t aColumnIndex) const
{
    if (matrix.empty())
    {
        throw std::out_of_range("Matrix is empty, cannot get column vector at index " + std::to_string(aColumnIndex));
    }
    if (aColumnIndex >= getCols())
    {
        throw std::out_of_range(
            "Column index " + std::to_string(aColumnIndex) + " out of range. Matrix has " + std::to_string(getCols()) +
            " columns.");
    }
    std::vector<T> columnVector = std::vector<T>(getRows());
    for (std::size_t row = 0; row < getRows(); ++row)
    {
        columnVector.at(row) = at(row, aColumnIndex);
    }
    return columnVector;
}

template <typename T>
void Matrix<T>::setRowVector(const std::vector<T>& aRowVector, std::size_t aRowIndex)
{
    if (aRowVector.size() != getCols())
    {
        throw std::invalid_argument(
            "Provided row vector has " + std::to_string(aRowVector.size()) + " elements, but matrix has " +
            std::to_string(getCols()) + " columns.");
    }
    if (aRowIndex >= getRows())
    {
        throw std::out_of_range(
            "Row index " + std::to_string(aRowIndex) + " out of range. Matrix has " + std::to_string(getRows()) + " rows.");
    }

    at(aRowIndex) = aRowVector;
}

template <typename T>
void Matrix<T>::setColumnVector(const std::vector<T>& aColumnVector, std::size_t aColumnIndex)
{
    if (matrix.empty())
    {
        throw std::logic_error(
            "Matrix is empty, which should not be possible. Cannot get column vector at index " +
            std::to_string(aColumnIndex));
    }
    if (aColumnVector.size() != getRows())
    {
        throw std::invalid_argument(
            "Provided column vector has " + std::to_string(aColumnVector.size()) + " elements, but matrix has " +
            std::to_string(getRows()) + " rows.");
    }
    if (aColumnIndex >= getCols())
    {
        throw std::out_of_range(
            "Column index " + std::to_string(aColumnIndex) + " out of range. Matrix has " + std::to_string(getCols()) +
            " columns.");
    }
    for (std::size_t row = 0; row < getRows(); ++row)
    {
        at(row, aColumnIndex) = aColumnVector.at(row);
    }
}

/**
 *
 */
template <class T>
Matrix<T>& Matrix<T>::operator=(const Matrix<T>& rhs)
{
    if (this != &rhs)
    {
        matrix = rhs.matrix;
    }
    return *this;
}
/**
 *
 */
template <class T>
bool Matrix<T>::operator==(const Matrix<T>& rhs) const
{
    if (getCols() != rhs.getCols() || getRows() != rhs.getRows())
    {
        return false;
    }

    for (std::size_t row = 0; row < getRows(); ++row)
    {
        for (std::size_t col = 0; col < getCols(); ++col)
        {
            if (std::isnan(at(row, col)) || std::isnan(rhs.at(row, col)))
            {
                if (std::isnan(at(row, col)) && std::isnan(rhs.at(row, col)))
                {
                    continue;
                }
                else
                {
                    return false;
                }
            }
            if (at(row, col) != rhs.at(row, col))
            {
                return false;
            }
        }
    }
    return true;
}
/**
 *
 */
template <class T>
bool Matrix<T>::operator!=(const Matrix<T>& rhs) const
{
    return !(*this == rhs);
}
/**
 *
 */
template <class T>
template <class T2>
Matrix<T>& Matrix<T>::operator*=(const T2& scalar)
{
    static_assert(
        std::is_arithmetic<T2>::value,
        "Value T2 must be arithmetic, see http://en.cppreference.com/w/cpp/types/is_arithmetic");
    for (std::size_t row = 0; row < getRows(); ++row)
    {
        for (std::size_t column = 0; column < getCols(); ++column)
        {
            at(row, column) *= scalar;
        }
    }
    return *this;
}
/**
 *
 */
template <class T>
template <class T2>
Matrix<T> Matrix<T>::operator*(const T2& scalar) const
{
    static_assert(
        std::is_arithmetic<T2>::value,
        "Value T2 must be arithmetic, see http://en.cppreference.com/w/cpp/types/is_arithmetic");

    Matrix<T> result(*this);
    return result *= scalar;
}

template <class T>
template <class T2>
Matrix<T> Matrix<T>::operator^(const T2& scalar) const
{
    static_assert(
        std::is_arithmetic<T2>::value,
        "Value T2 must be arithmetic, see http://en.cppreference.com/w/cpp/types/is_arithmetic");

    T2 absScalar = std::abs(scalar);
    if (absScalar == 0)
    {
        // Als de exponent nul is, retourneer dan de identiteitsmatrix
        return identity();
    }
    else if (absScalar == 1)
    {
        // Als de exponent één is, retourneer dan de matrix zelf
        return *this;
    }
    else if (scalar > 0)
    {
        // Als de exponent positief is, gebruik exponentiële machtsverheffing
        Matrix<T> result = *this;
        Matrix<T> base = *this;
        for (std::size_t i = 1; i < absScalar; ++i)
        {
            result = result * base;
        }
        return result;
    }
    else
    {
        // Als de exponent negatief is, retourneer dan de inverse van de matrix tot de macht van de positieve exponent
        Matrix<T> inverseMatrix = this->inverse();
        return (inverseMatrix ^ absScalar);
    }
}
/**
 *
 */
template <class T>
template <class T2>
Matrix<T>& Matrix<T>::operator/=(const T2& aScalar)
{
    static_assert(
        std::is_arithmetic<T2>::value,
        "Value T2 must be arithmetic, see http://en.cppreference.com/w/cpp/types/is_arithmetic");

    for (std::size_t row = 0; row < getRows(); ++row)
    {
        for (std::size_t column = 0; column < getCols(); ++column)
        {
            at(row, column) /= aScalar;
        }
    }
    return *this;
}
/**
 *
 */
template <class T>
template <class T2>
Matrix<T> Matrix<T>::operator/(const T2& aScalar) const
{
    static_assert(
        std::is_arithmetic<T2>::value,
        "Value T2 must be arithmetic, see http://en.cppreference.com/w/cpp/types/is_arithmetic");

    Matrix<T> result(*this);
    return result /= aScalar;
}
/**
 *
 */
template <class T>
Matrix<T>& Matrix<T>::operator+=(const Matrix<T>& rhs)
{
    if (getRows() != rhs.getRows() || getCols() != rhs.getCols())
    {
        throw std::invalid_argument("Matrices must have the same dimensions to add them");
    }

    for (std::size_t row = 0; row < getRows(); ++row)
    {
        for (std::size_t column = 0; column < getCols(); ++column)
        {
            at(row, column) += rhs.at(row, column);
        }
    }
    return *this;
}
/**
 *
 */
template <class T>
Matrix<T> Matrix<T>::operator+(const Matrix<T>& rhs) const
{

    Matrix<T> result(*this);
    return result += rhs;
}
/**
 *
 */
template <class T>
Matrix<T>& Matrix<T>::operator-=(const Matrix<T>& rhs)
{
    if (getRows() != rhs.getRows() || getCols() != rhs.getCols())
    {
        throw std::invalid_argument("Matrices must have the same dimensions to subtract them");
    }

    for (std::size_t row = 0; row < getRows(); ++row)
    {
        for (std::size_t column = 0; column < getCols(); ++column)
        {
            at(row, column) -= rhs.at(row, column);
        }
    }
    return *this;
}
/**
 *
 */
template <class T>
Matrix<T> Matrix<T>::operator-(const Matrix<T>& rhs) const
{
    Matrix<T> result(*this);
    return result -= rhs;
}

/**
 * Multiplies two matrices and returns the result.
 *
 * (ROWS, COLS) * (COLS, P) -> (ROWS, P)
 *
 * @tparam T The type of the matrix elements.
 * @param rhs The matrix to multiply with.
 * @return The resulting matrix after multiplication.
 */
template <class T>
Matrix<T> Matrix<T>::operator*(const Matrix<T>& rhs) const
{
    if (getCols() != rhs.getRows())
    {
        throw std::invalid_argument(
            "The number of columns of the first matrix must be equal to the number of rows of the second matrix");
    }
    Matrix<T> result(getRows(), rhs.getCols());
    for (std::size_t row = 0; row < getRows(); ++row) // for each row
    {
        for (std::size_t column = 0; column < rhs.getCols(); ++column) // for each column
        {
            result.at(row, column) = 0;                 // initialize the result at the current position
            for (std::size_t i = 0; i < getCols(); ++i) // iterate over each row from the first matrix and the column from
                                                        // the second matrix
            {
                // do the multiplication (for every position) and add the result
                result.at(row, column) += at(row, i) * rhs.at(i, column);
            }
        }
    }
    return result;
}
/**
 *
 */
template <class T>
void Matrix<T>::merge(const Matrix<T>& aMatrix)
{
    if (getRows() != aMatrix.getRows() || getCols() != aMatrix.getCols())
    {
        throw std::invalid_argument("The matrices must have the same dimensions");
    }
    for (std::size_t row = 0; row < getRows(); ++row)
    {
        for (std::size_t column = 0; column < getCols(); ++column)
        {
            T valueA = at(row, column);
            T valueB = aMatrix.at(row, column);
            if (std::isnan(valueA) && std::isnan(valueB))
            {
                continue;
            }
            if (std::isnan(valueA))
            {
                at(row, column) = valueB;
            }
            else if (std::isnan(valueB))
            {
                at(row, column) = valueA;
            }
            else
            {
                at(row, column) = (valueA + valueB) / 2;
            }
        }
    }
}

/**
 *
 */
template <class T>
void Matrix<T>::merge(const MatrixSegment<T>& aMatrixSegment)
{
    if (aMatrixSegment.getRowOrigin() + aMatrixSegment.getRows() > getRows() ||
        aMatrixSegment.getColOrigin() + aMatrixSegment.getCols() > getCols())
    {
        throw std::invalid_argument("The segment matrix exceeds the bounds of the original matrix");
    }

    for (std::size_t row = 0; row < aMatrixSegment.getRows(); ++row)
    {
        for (std::size_t column = 0; column < aMatrixSegment.getCols(); ++column)
        {
            std::size_t targetRow = row + aMatrixSegment.getRowOrigin();
            std::size_t targetColumn = column + aMatrixSegment.getColOrigin();

            T valueA = at(targetRow, targetColumn);
            T valueB = aMatrixSegment.at(row, column);

            if (std::isnan(valueA) && std::isnan(valueB))
            {
                continue;
            }
            if (std::isnan(valueA))
            {
                at(targetRow, targetColumn) = valueB;
            }
            else if (std::isnan(valueB))
            {
                at(targetRow, targetColumn) = valueA;
            }
            else
            {
                at(targetRow, targetColumn) = (valueA + valueB) / 2;
            }
        }
    }
}
/**
 *
 */
template <class T>
Matrix<T> Matrix<T>::transpose() const
{
    Matrix<T> result(getRows(), getCols());
    for (std::size_t row = 0; row < getRows(); ++row)
    {
        for (std::size_t column = 0; column < getCols(); ++column)
        {
            result.at(column, row) = at(row, column);
        }
    }
    return result;
}
/**
 *
 */
template <class T>
Matrix<T> Matrix<T>::identity() const
{
    if (getRows() != getCols())
    {
        throw std::invalid_argument("Identity matrix can only be created for square matrices");
    }
    Matrix<T> result = *this;
    for (std::size_t row = 0; row < getRows(); ++row)
    {
        for (std::size_t column = 0; column < getCols(); ++column)
        {
            if (row == column)
            {
                result.at(row).at(column) = 1;
            }
            else
            {
                result.at(row).at(column) = 0;
            }
        }
    }
    return result;
}
/**
 * Performs the gauss algorithm on the matrix.
 *
 * @tparam T The type of the matrix elements.
 * @return The matrix in row echelon form.
 */
template <class T>
Matrix<T> Matrix<T>::gauss() const
{
    // need copy of matrix to not change the original
    Matrix<T> mat = *this;

    // loop through each pivot position.
    for (std::size_t pivot = 0; pivot < getRows(); ++pivot)
    {
        // check if the pivot is 0 (if it is, we need to swap rows to get a non-zero value in the pivot position)
        if (mat.at(pivot, pivot) == 0)
        {
            // Find a row with a non-zero value in the pivot column
            for (std::size_t row = pivot + 1; row < getRows(); ++row)
            {

                // If the row is not the pivot row and the value is not 0
                if (row != pivot && mat.at(row, pivot) != 0)
                {
                    // Swap rows
                    for (std::size_t swapCol = 0; swapCol < getCols(); ++swapCol)
                    {
                        std::swap(mat.at(pivot, swapCol), mat.at(row, swapCol));
                    }
                    break;
                }
            }
        }
        // perform a row operation to make the pivot 1 (divide the row by scalar)
        T scalar = mat.at(pivot, pivot);
        for (std::size_t col = pivot; col < getCols(); ++col)
        {
            mat.at(pivot, col) /= scalar;
        }

        // perform a row operation to make the other values in the pivot column 0
        // +1 because we don't want to change the pivot row itself
        for (std::size_t row = pivot + 1; row < getRows(); ++row)
        {
            if (mat.at(row, pivot) != 0)
            {
                // subtract the pivot row multiplied by the value in the pivot column from the current row
                T scalar2 = mat.at(row, pivot);
                for (std::size_t col = pivot; col < getCols(); ++col)
                {
                    mat.at(row, col) -= mat.at(pivot, col) * scalar2;
                }
            }
        }
    }
    return mat;
}
/**
 * Performs the gauss-jordan algorithm on the matrix.
 *
 * @tparam T The type of the matrix elements.
 * @return The matrix in reduced row echelon form.
 */
template <class T>
Matrix<T> Matrix<T>::gaussJordan() const
{
    // do the gauss algorithm
    Matrix<T> mat = gauss();

    // loop through the each pivot position.
    // start from the last pivot and go to the first
    for (std::size_t pivot = getRows() - 1; pivot > 0; --pivot)
    {
        // perform a row operation to make the other values in the pivot column 0
        // -1 because we don't want to change the pivot row itself
        for (std::size_t row = pivot - 1; row < getRows(); --row)
        {
            if (mat.at(row, pivot) != 0)
            {
                // subtract the pivot row multiplied by the value in the pivot column from the current row
                T scalar = mat.at(row, pivot);
                for (std::size_t col = pivot; col < getCols(); ++col)
                {
                    mat.at(row, col) -= mat.at(pivot, col) * scalar;
                }
            }
        }
    }
    return mat;
}
/**
 *
 */
template <class T>
std::vector<T> Matrix<T>::solve() const
{
    Matrix<T> mat = gaussJordan();
    return mat.getColumnVector(getCols() - 1);
}
/**
 * dictaat 3.2.3. INVERTEREN MET GAUSS-JORDAN ELIMINATIE
 */
template <class T>
Matrix<T> Matrix<T>::inverse() const
{
    if (getRows() != getCols())
    {
        throw std::invalid_argument("Inverse can only be calculated for square matrices");
    }

    Matrix<T> identiteitsMatrix = identity();
    // Breid de matrix aan de rechterkant met de identiteitsmatrix uit:
    Matrix<T> uitgebreideMatrix(getRows(), getCols() * 2);
    for (std::size_t i = 0; i < getRows(); ++i)
    {
        for (std::size_t j = 0; j < getCols(); ++j)
        {
            uitgebreideMatrix.at(i, j) = at(i, j);                               // Copy the original matrix to the left half
            uitgebreideMatrix.at(i, j + getCols()) = identiteitsMatrix.at(i, j); // Copy the identity matrix to the right
                                                                                 // half
        }
    }

    // dictaat: Pas vervolgens Gauss-Jordan net zolang toe totdat de identiteitsmatrix aan de linkerkant staat
    uitgebreideMatrix = uitgebreideMatrix.gaussJordan();

    // Extract the inverse matrix
    Matrix<T> inverse(getRows(), getCols());
    for (std::size_t i = 0; i < getRows(); ++i)
    {
        for (std::size_t j = 0; j < getCols(); ++j)
        {
            inverse.at(i, j) = uitgebreideMatrix.at(i, j + getCols());
        }
    }
    return inverse;
}

template <class T>
void Matrix<T>::writeMatrixToImage(const std::string& path) const
{
    int type = cv::DataType<T>::type;
    // Add more types if needed
    cv::Mat image(getRows(), getCols(), type);

    for (std::size_t i = 0; i < getRows(); ++i)
    {
        for (std::size_t j = 0; j < getCols(); ++j)
        {
            image.at<T>(i, j) = at(i, j);
        }
    }
    cv::imwrite(path, image);
}

template <class T>
void Matrix<T>::writeMatrixToCSV(const std::string& path) const
{
    // Export matrices to CSV files
    std::ofstream matrixFile(path);
    for (std::size_t i = 0; i < getRows(); ++i)
    {
        for (std::size_t j = 0; j < getCols(); ++j)
        {
            matrixFile << at(i, j);
            if (j < getCols() - 1)
            {
                matrixFile << ",";
            }
        }
        matrixFile << "\n";
    }
    matrixFile.close();
}

template <class T>
std::vector<T> Matrix<T>::power(const Matrix<T>& matrix, const std::vector<T>& stateVector)
{
    // Vermenigvuldig de matrix met de vector
    std::vector<T> result(getRows(), 0);
    for (std::size_t i = 0; i < getRows(); ++i)
    {
        for (std::size_t j = 0; j < getCols(); ++j)
        {
            result.at(i) += at(i, j) * stateVector.at(j);
        }
    }
    return result;
}
template <class T>
std::vector<T> Matrix<T>::findSteadyState(
    const std::vector<T>& initialState,
    const T aPrecision /*= std::numeric_limits<T>::epsilon()*/,
    const unsigned long aFactor /*= 1*/)
{
    Matrix<T> powerMatrix = identity();
    std::vector<T> currentState = initialState;
    std::vector<T> previousState(initialState.size());

    while (true)
    {
        powerMatrix = powerMatrix * *this;
        currentState = powerMatrix * initialState;
        if (equals(currentState, previousState, aPrecision, aFactor))
        {
            break;
        }
        previousState = currentState;
    }

    return currentState;
}

/**
 *
 */
template <class T>
std::string Matrix<T>::to_string() const
{
    std::string result = "Matrix<" + std::to_string(getCols()) + "," + std::to_string(getRows()) + ">\n{\n";
    for (std::size_t i = 0; i < getRows(); ++i)
    {
        for (std::size_t j = 0; j < getCols(); ++j)
        {
            T value = at(i, j);
            result += std::to_string(value) + ",";
        }
        result += "\n";
    }
    result += "}";
    return result;
}

template <typename T>
bool equals(
    const std::vector<T>& lhs,
    const std::vector<T>& rhs,
    const T aPrecision /*= std::numeric_limits<T>::epsilon()*/,
    const unsigned long aFactor /*= 1*/)
{
    if (lhs.size() != rhs.size())
    {
        return false;
    }

    for (std::size_t i = 0; i < lhs.size(); ++i)
    {
        if (std::isnan(lhs.at(i)) || std::isnan(rhs.at(i)))
        {
            if (std::isnan(lhs.at(i)) && std::isnan(rhs.at(i)))
            {
                continue;
            }
            else
            {
                return false;
            }
        }
        if (std::abs(lhs.at(i) - rhs.at(i)) > aPrecision * aFactor)
        {
            return false;
        }
    }
    return true;
}

template <typename T>
bool equals(
    const Matrix<T>& lhs,
    const Matrix<T>& rhs,
    const T aPrecision /*= std::numeric_limits<T>::epsilon()*/,
    const unsigned long aFactor /*= 1*/)
{
    if (lhs.getRows() != rhs.getRows() || lhs.getCols() != rhs.getCols())
    {
        return false;
    }

    for (std::size_t row = 0; row < lhs.getRows(); ++row)
    {
        for (std::size_t col = 0; col < lhs.getCols(); ++col)
        {
            if (std::isnan(lhs.at(row, col)) || std::isnan(rhs.at(row, col)))
            {
                if (std::isnan(lhs.at(row, col)) && std::isnan(rhs.at(row, col)))
                {
                    continue;
                }
                else
                {
                    return false;
                }
            }
            if (std::abs(lhs.at(row, col) - rhs.at(row, col)) > aPrecision * aFactor)
            {
                return false;
            }
        }
    }
    return true;
}
