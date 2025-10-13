#ifndef CELL_HPP
#define CELL_HPP

#include <cstddef> // std::size_t

/**
 * @class Cell
 *
 * @brief The Cell class represents a copy of a cell in a matrix.
 *
 * @tparam T type of the value
 */
template <typename T>
class Cell
{
public:
    virtual ~Cell();
    /**
     * @brief Construct a new Cell object
     *
     * @tparam T type of the value
     * @param row row index
     * @param col column index
     * @param value value of the cell
     */
    Cell(std::size_t row, std::size_t col, T value);
    /**
     * @brief Construct a new Cell object
     *
     * @tparam T type of the value
     */
    Cell();
    /**
     * @brief Copy constructor
     *
     * @param aCell cell to copy
     */
    Cell(const Cell& aCell) = default;
    /**
     * @brief Copy assignment operator
     *
     * @param aCell cell to copy
     * @return Cell& reference to the new cell
     */
    Cell& operator=(const Cell& aCell) = default;
    /**
     * @brief get the row index of the cell
     *
     * @return std::size_t row index
     */
    std::size_t getRow() const;
    /**
     * @brief get the column index of the cell
     *
     * @return std::size_t column index
     */
    std::size_t getCol() const;
    /**
     * @brief set the value of the cell
     *
     * @param value value of the cell
     */
    void setRow(std::size_t row);
    /**
     * @brief set the value of the cell
     *
     * @param value value of the cell
     */
    void setCol(std::size_t col);

    std::size_t row; /**< row index */
    std::size_t col; /**< column index */
    T value;         /**< value of the cell */
};

#include "Cell.inc"

#endif /* CELL_HPP */