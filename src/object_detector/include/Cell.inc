#include <cmath> // NAN

template <class T>
Cell<T>::~Cell()
{}

template <class T>
Cell<T>::Cell(std::size_t row, std::size_t col, T value) : row(row), col(col), value(value)
{}

template <class T>
Cell<T>::Cell() : row(0), col(0), value(NAN)
{}

template <class T>
std::size_t Cell<T>::getRow() const
{
    return row;
}

template <class T>
std::size_t Cell<T>::getCol() const
{
    return col;
}

template <class T>
void Cell<T>::setRow(std::size_t row)
{
    this->row = row;
}

template <class T>
void Cell<T>::setCol(std::size_t col)
{
    this->col = col;
}