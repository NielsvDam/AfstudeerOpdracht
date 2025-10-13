#ifndef MATRIX_HPP
#define MATRIX_HPP

#include <vector>           // std::vector
#include <initializer_list> // std::initializer_list
#include <limits>           // std::numeric_limits
#include <cstddef>          // std::size_t
#include <string>           // std::string
#include <ostream>          // std::ostream

template <class T>
class MatrixSegment; // due to circular dependency

/**
 * The Matrix class is an implementation of the mathematical concept of a matrix (not the movie).
 * @see https://en.wikipedia.org/wiki/Matrix_(mathematics) for more information.
 *
 * typename T: T must be an arithmetic type, i.e. an integral or floating point type
 */
template <typename T>
class Matrix
{
public:
    /**
     * @name Compile-time assertion checking: see http://en.cppreference.com/w/cpp/language/static_assert
     */
    //@{
    /**
     *
     */
    static_assert(
        std::is_arithmetic<T>::value,
        "Value T must be arithmetic, see http://en.cppreference.com/w/cpp/types/is_arithmetic");
    /**
     *
     */
    //@}
    /**
     * @name Constructors and destructor
     */
    //@{
    /**
     * Default ctor. Initialises all cells with 0 or the given value.
     */
    Matrix(std::size_t rows, std::size_t cols, T value = 0);
    /**
     * Ctor with a list of lists of values where aList must contain ROWS elements and each list in aList must contain COLS
     * elements
     */
    Matrix(const std::initializer_list<std::initializer_list<T>>& aList); // cppcheck-suppress noExplicitConstructor
    /**
     * Cpy ctor
     */
    Matrix(const Matrix<T>& aMatrix);
    /**
     * Dtor
     */
    virtual ~Matrix() = default;
    //@}
    /**
     * @name Dimension access
     */
    //@{
    /**
     *
     */
    std::size_t getRows() const
    {
        return matrix.size();
    }
    /**
     *
     */
    std::size_t getCols() const
    {
        if (matrix.empty())
        {
            return 0;
        }
        else
        {
            return at(0).size();
        }
    }
    //@}
    /**
     * @name Element access
     */
    //@{
    /**
     * Returns the row at aRowIndex
     * If aRowIndex > getRows() an exception of type std::out_of_range is thrown.
     */
    std::vector<T>& at(std::size_t aRowIndex);
    /**
     * Returns the row at aRowIndex
     * If aRowIndex > getRows() an exception of type std::out_of_range is thrown.
     */
    const std::vector<T>& at(std::size_t aRowIndex) const;
    /**
     * Returns the cell at (aRowIndex,aColumnIndex)
     * If aRowIndex > getRows() or aColumnIndex > getCols an exception of type std::out_of_range is thrown.
     */
    T& at(std::size_t aRowIndex, std::size_t aColumnIndex);
    /**
     * Returns the element at (aRowIndex,aColumnIndex)
     * If aRowIndex > getRows() or aColumnIndex > getCols an exception of type std::out_of_range is thrown.
     */
    const T& at(std::size_t aRowIndex, std::size_t aColumnIndex) const;
    /**
     * Returns the row at aRowIndex. No range checking is done.
     */
    std::vector<T>& operator[](std::size_t aRowIndex);
    /**
     * Returns the row at aRowIndex. No range checking is done.
     */
    const std::vector<T>& operator[](std::size_t aRowIndex) const;
    /**
     * Returns the rowVector at aRowIndex
     */
    std::vector<T> getRowVector(std::size_t aRowIndex) const;
    /**
     * Returns the columnVector at aColumnIndex
     */
    std::vector<T> getColumnVector(std::size_t aColumnIndex) const;
    /**
     * Sets the rowVector at aRowIndex
     */
    void setRowVector(const std::vector<T>& aRowVector, std::size_t aRowIndex);
    /**
     * Sets the columnVector at aColumnIndex
     */
    void setColumnVector(const std::vector<T>& aColumnVector, std::size_t aColumnIndex);
    /**
     * Begin method for non-const objects
     */
    auto begin()
    {
        return matrix.begin();
    }
    /**
     * End method for non-const objects
     */
    auto end()
    {
        return matrix.end();
    }
    /**
     * Begin method for const objects
     */
    auto begin() const
    {
        return matrix.begin();
    }
    /**
     * End method for const objects
     */
    auto end() const
    {
        return matrix.end();
    }
    //@}
    /**
     * @name Matrix operators
     */
    //@{
    /**
     * Assignment operator
     */
    Matrix<T>& operator=(const Matrix<T>& rhs);
    /**
     * Comparison operator
     */
    bool operator==(const Matrix<T>& rhs) const;
    /**
     * !Comparison operator
     */
    bool operator!=(const Matrix<T>& rhs) const;
    //@}
    /**
     * @name Scalar arithmetic operations supporting only rhs-scalars
     */
    //@{
    /**
     *
     */
    template <class T2 = T>
    Matrix<T>& operator*=(const T2& scalar);
    /**
     *
     */
    template <class T2 = T>
    Matrix<T> operator*(const T2& scalar) const;
    /**
     *
     */
    template <class T2 = T>
    Matrix<T> operator^(const T2& scalar) const;
    /**
     *
     */
    template <class T2 = T>
    Matrix<T>& operator/=(const T2& scalar);
    /**
     *
     */
    template <class T2 = T>
    Matrix<T> operator/(const T2& scalar) const;
    //@}
    /**
     * @name Matrix arithmetic operations
     */
    //@{
    /**
     *
     */
    Matrix<T>& operator+=(const Matrix<T>& rhs);
    /**
     *
     */
    Matrix<T> operator+(const Matrix<T>& rhs) const;
    /**
     *
     */
    Matrix<T>& operator-=(const Matrix<T>& rhs);
    /**
     *
     */
    Matrix<T> operator-(const Matrix<T>& rhs) const;
    /**
     *
     */
    Matrix<T> operator*(const Matrix<T>& rhs) const;
    //@}
    /**
     * @name Matrix functions
     */
    //@{
    /**
     * @see https://en.wikipedia.org/wiki/Transpose
     */
    Matrix<T> transpose() const;
    /**
     * @see https://en.wikipedia.org/wiki/Identity_matrix
     */
    Matrix<T> identity() const;
    /**
     * @see https://en.wikipedia.org/wiki/Gaussian_elimination
     */
    Matrix<T> gauss() const;
    /**
     * @see https://en.wikipedia.org/wiki/Invertible_matrix
     */
    Matrix<T> gaussJordan() const;
    /**
     *
     */
    std::vector<T> solve() const;
    /**
     * @see https://en.wikipedia.org/wiki/Invertible_matrix
     */
    Matrix<T> inverse() const;
    //@}
    /**
     * @name Debug writers
     */
    //@{
    /**
     * Writes the matrix to a CSV file
     */
    void writeMatrixToCSV(const std::string& path) const;
    /**
     * Writes the matrix to an image file
     */
    void writeMatrixToImage(const std::string& path) const;
    //@}
    /**
     * @name Other methods
     */
    //@{
    /**
     * Merge a matrices by averaging the values of the corresponding cells. When a cell is NaN in one matrix and
     * not in the other, the value of the other matrix is used.
     */
    void merge(const Matrix<T>& aMatrix);
    /**
     * Merge a matrixsegment. The merging behaviour is the same as for the normal merge method.
     * With this method the matrixsegment is merged at the correct position in the matrix.
     */
    void merge(const MatrixSegment<T>& aMatrixSegement);
    /**
     * @see https://en.wikipedia.org/wiki/Matrix_multiplication
     * @tparam T The type of the matrix elements.
     */
    std::vector<T> power(const Matrix<T>& matrix, const std::vector<T>& stateVector);
    /**
     * @see https://en.wikipedia.org/wiki/Power_iteration
     */
    std::vector<T> findSteadyState(
        const std::vector<T>& initialState,
        const T aPrecision = std::numeric_limits<T>::epsilon(),
        const unsigned long aFactor = 1);
    /**
     * @return a string representation of the matrix for printing on the screen
     */
    std::string to_string() const;
    //@}
protected:
    std::vector<std::vector<T>> matrix;
};

/**
 *
 */
template <typename T>
inline bool operator==(const Matrix<T>& lhs, const Matrix<T>& rhs)
{
    return lhs.matrix == rhs.matrix;
}
template <typename T>
inline bool operator!=(const Matrix<T>& lhs, const Matrix<T>& rhs)
{
    return lhs.matrix != rhs.matrix;
}

/**
 *
 */
template <typename T>
inline std::ostream& operator<<(std::ostream& stream, const Matrix<T>& aMatrix)
{
    return stream << aMatrix.to_string();
}

template <typename T>
inline Matrix<T> createIdentity(const std::size_t aSize)
{
    Matrix<T> result(aSize, aSize);
    return result.identity();
}

/**
 * Compare two row vectors using a aPrecision and a factor. The actual used precision is aPrecission*aFactor.
 *
 */
template <typename T>
bool equals(
    const std::vector<T>& lhs,
    const std::vector<T>& rhs,
    const T aPrecision = std::numeric_limits<T>::epsilon(),
    const unsigned long aFactor = 1);
/**
 * Compare two matrices using a Precision and a factor. The actual used  precision is aPrecission*aFactor.
 * If all rows are equal according this aPrecission*aFactor true is returned, false otherwise
 */
template <typename T>
bool equals(
    const Matrix<T>& lhs,
    const Matrix<T>& rhs,
    const T aPrecision = std::numeric_limits<T>::epsilon(),
    const unsigned long aFactor = 1);

#include "Matrix.inc"

#endif /* MATRIX_HPP_ */
