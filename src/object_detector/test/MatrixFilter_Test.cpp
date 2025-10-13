#include <gtest/gtest.h>
#include "Matrix.hpp"
#include "MatrixFilters.hpp"
#include "Constants.hpp"
#include <cmath>
#include <iostream>

// Test if the nan filter works on a matrix missing two values.
TEST(MatrixFiltersTest, TestNanFilterSimple)
{
    Matrix<float> matrix{
        {1.0, 2.0, 3.0, 4.0, 5.0, 6.0},
        {7.0, 8.0, 9.0, 10.0, 11.0, 12.0},
        {13.0, 14.0, NAN, 16.0, 17.0, 18.0},
        {19.0, 20.0, NAN, 22.0, 23.0, 24.0},
        {25.0, 26.0, 27.0, 28.0, 29.0, 30.0},
        {31.0, 32.0, 33.0, 34.0, 35.0, 36.0}};
    MatrixFilters::nanFilter(matrix);
    ASSERT_FLOAT_EQ(matrix.at(2, 2), 27);
    ASSERT_FLOAT_EQ(matrix.at(3, 2), 27);
}

// Test if the nan filter works on all directions.
TEST(MatrixFiltersTest, TestNanFilterAllDirections)
{
    Matrix<float> matrix{
        {1.0, 2.0, 3.0, 4.0, 5.0, 6.0},
        {7.0, 8.0, NAN, 10.0, 11.0, 12.0},
        {13.0, NAN, NAN, NAN, NAN, 18.0},
        {19.0, NAN, NAN, NAN, NAN, 50.0},
        {25.0, 26.0, 27.0, 28.0, 29.0, 30.0},
        {31.0, 32.0, 33.0, 34.0, 35.0, 36.0}};
    MatrixFilters::nanFilter(matrix);
    ASSERT_FLOAT_EQ(matrix.at(2, 2), 27);
    ASSERT_FLOAT_EQ(matrix.at(3, 2), 50);
}

// The filter does all rows first, and then all columns.
// This leads to a specific order of filling the NAN values, which is tested here.
TEST(MatrixFiltersTest, TestNanFilterMissingColAndRow)
{
    Matrix<float> matrix{
        {1.0, 2.0, NAN, 4.0, 5.0, 6.0},
        {7.0, 8.0, NAN, 10.0, 11.0, 12.0},
        {NAN, NAN, NAN, NAN, NAN, NAN},
        {19.0, 20.0, NAN, 22.0, 23.0, 24.0},
        {25.0, 26.0, NAN, 28.0, 29.0, 30.0},
        {31.0, 32.0, NAN, 34.0, 35.0, 36.0}};
    MatrixFilters::nanFilter(matrix);
    ASSERT_FLOAT_EQ(matrix.at(2, 0), 19.0);
    ASSERT_FLOAT_EQ(matrix.at(2, 1), 20.0);
    ASSERT_FLOAT_EQ(matrix.at(2, 2), 20.0);
    ASSERT_FLOAT_EQ(matrix.at(3, 2), 22.0);
}

// Note: This would be incorrect if the nan filter would do another cycle in reverse (starting at the bottom right corner).
// Implementing this would take care of the NAN values in all corners, but this isnt actually an issue.
TEST(MatrixFiltersTest, TestNanCorner)
{
    Matrix<float> matrix{
        {NAN, NAN, NAN, NAN, NAN, NAN},
        {NAN, NAN, NAN, NAN, NAN, NAN},
        {NAN, NAN, 15.0, 16.0, 17.0, 18.0},
        {NAN, NAN, 21.0, 22.0, 23.0, 24.0},
        {NAN, NAN, 27.0, 28.0, 29.0, 30.0},
        {NAN, NAN, 33.0, 34.0, 35.0, 36.0}};
    MatrixFilters::nanFilter(matrix);
    // test that these values could not be filled
    ASSERT_TRUE(std::isnan(matrix.at(0, 0)));
    ASSERT_TRUE(std::isnan(matrix.at(0, 1)));
    ASSERT_TRUE(std::isnan(matrix.at(1, 0)));
    ASSERT_TRUE(std::isnan(matrix.at(1, 1)));
    // test if the rest is filled correctly
    ASSERT_FLOAT_EQ(matrix.at(2, 0), 15.0);
    ASSERT_FLOAT_EQ(matrix.at(2, 1), 15.0);
    ASSERT_FLOAT_EQ(matrix.at(0, 2), 15.0);
    ASSERT_FLOAT_EQ(matrix.at(1, 2), 15.0);
}
// Test if only the portions of a block are included. The block is 18.9mm, the detection tolerance is 5mm.
TEST(MatrixFiltersTest, testSurfaceExtractionOnColumn)
{
    Matrix<float> matrix{
        {298, 298, 297, 297, 297, 297, 297, 297, 280, 280, 280, 281, 281, 281, 281, 281, 297, 297, 297, 297}};
    MatrixFilters::sufaceExtractionFilter(matrix);
    ASSERT_TRUE(std::isnan(matrix.at(0, 0)));
    ASSERT_TRUE(std::isnan(matrix.at(0, 1)));
    ASSERT_TRUE(std::isnan(matrix.at(0, 2)));
    ASSERT_TRUE(std::isnan(matrix.at(0, 3)));
    ASSERT_TRUE(std::isnan(matrix.at(0, 4)));
    ASSERT_TRUE(std::isnan(matrix.at(0, 5)));
    ASSERT_TRUE(std::isnan(matrix.at(0, 6)));
    ASSERT_TRUE(std::isnan(matrix.at(0, 7)));
    ASSERT_FLOAT_EQ(matrix.at(0, 8), 280);
    ASSERT_FLOAT_EQ(matrix.at(0, 9), 280);
    ASSERT_FLOAT_EQ(matrix.at(0, 10), 280);
    ASSERT_FLOAT_EQ(matrix.at(0, 11), 281);
    ASSERT_FLOAT_EQ(matrix.at(0, 12), 281);
    ASSERT_FLOAT_EQ(matrix.at(0, 13), 281);
    ASSERT_FLOAT_EQ(matrix.at(0, 14), 281);
    ASSERT_FLOAT_EQ(matrix.at(0, 15), 281);
    ASSERT_TRUE(std::isnan(matrix.at(0, 16)));
    ASSERT_TRUE(std::isnan(matrix.at(0, 17)));
    ASSERT_TRUE(std::isnan(matrix.at(0, 18)));
    ASSERT_TRUE(std::isnan(matrix.at(0, 19)));
}
// Test if only the portions of a block are included. The block is 18.9mm, the detection tolerance is 5mm.
TEST(MatrixFiltersTest, testSurfaceExtractionOnRow)
{
    Matrix<float> matrix{{298}, {298}, {297}, {297}, {297}, {297}, {297}, {297}, {280}, {280},
                         {280}, {281}, {281}, {281}, {281}, {281}, {297}, {297}, {297}, {297}};
    MatrixFilters::sufaceExtractionFilter(matrix);
    ASSERT_TRUE(std::isnan(matrix.at(0, 0)));
    ASSERT_TRUE(std::isnan(matrix.at(1, 0)));
    ASSERT_TRUE(std::isnan(matrix.at(2, 0)));
    ASSERT_TRUE(std::isnan(matrix.at(3, 0)));
    ASSERT_TRUE(std::isnan(matrix.at(4, 0)));
    ASSERT_TRUE(std::isnan(matrix.at(5, 0)));
    ASSERT_TRUE(std::isnan(matrix.at(6, 0)));
    ASSERT_TRUE(std::isnan(matrix.at(7, 0)));
    ASSERT_FLOAT_EQ(matrix.at(8, 0), 280);
    ASSERT_FLOAT_EQ(matrix.at(9, 0), 280);
    ASSERT_FLOAT_EQ(matrix.at(10, 0), 280);
    ASSERT_FLOAT_EQ(matrix.at(11, 0), 281);
    ASSERT_FLOAT_EQ(matrix.at(12, 0), 281);
    ASSERT_FLOAT_EQ(matrix.at(13, 0), 281);
    ASSERT_FLOAT_EQ(matrix.at(14, 0), 281);
    ASSERT_FLOAT_EQ(matrix.at(15, 0), 281);
    ASSERT_TRUE(std::isnan(matrix.at(16, 0)));
    ASSERT_TRUE(std::isnan(matrix.at(17, 0)));
    ASSERT_TRUE(std::isnan(matrix.at(18, 0)));
    ASSERT_TRUE(std::isnan(matrix.at(19, 0)));
}
// Test if a gradial jump is accepted. In this case the jump distributed over a couple of cells.
// 15 - 0.5 == 14.5.
// 14.5 > (BLOCK_SIZE - BLOCK_HEIGHT_INDICATION_TOLERANCE)
TEST(MatrixFiltersTest, testSurfaceExtractionWindow)
{
    Matrix<float> matrix{{15.0, 7.0, 6.0, 0.5}}; // 15 - 0.5 == 14.5.
    MatrixFilters::sufaceExtractionFilter(matrix);
    ASSERT_TRUE(std::isnan(matrix.at(0, 0)));
    ASSERT_TRUE(std::isnan(matrix.at(0, 1)));
    ASSERT_TRUE(std::isnan(matrix.at(0, 2)));
    ASSERT_FLOAT_EQ(matrix.at(0, 3), 0.5);
}
// Test that only downwards jumps are accepted. A block never starts after a upwards jump.
TEST(MatrixFiltersTest, testSurfaceExtractionNoUpJumps)
{
    Matrix<float> matrix{{0.0, 20.0}};
    MatrixFilters::sufaceExtractionFilter(matrix);
    ASSERT_TRUE(std::isnan(matrix.at(0, 0)));
    ASSERT_TRUE(std::isnan(matrix.at(0, 1)));
    // test the opposite since we tested if something is NOT detected.
    matrix = Matrix<float>{{20.0, 0.0}};
    MatrixFilters::sufaceExtractionFilter(matrix);
    ASSERT_TRUE(std::isnan(matrix.at(0, 0)));
    ASSERT_FLOAT_EQ(matrix.at(0, 1), 0.0);
}

TEST(MatrixFiltersTest, testMerge)
{
    // TODO: Implement this test
}