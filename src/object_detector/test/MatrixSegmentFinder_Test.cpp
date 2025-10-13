#include <gtest/gtest.h>
#include "Matrix.hpp"
#include "MatrixFilters.hpp"
#include "Constants.hpp"
#include <cmath>
#include <iostream>

#include "Matrix.hpp"
#include "MatrixSegment.hpp"
#include "MatrixSegmentFinder.hpp"

TEST(MatrixSegmentFinderTest, testSingleCluster)
{
    Matrix<float> matrix{
        {NAN, NAN, NAN, NAN, NAN},
        {NAN, 0.0, 0.1, NAN, NAN},
        {NAN, 1.0, 1.1, 1.2, NAN},
        {NAN, 2.0, 2.1, 2.2, NAN},
        {NAN, NAN, NAN, NAN, NAN}};

    // {0.0, 0.1, NAN}
    // {1.0, 1.1, 1.2}
    // {2.0, 2.1, 2.2}
    MatrixSegment<float> expectedSegment = MatrixSegment<float>(3, 3, 1, 1);
    expectedSegment.at(0, 0) = 0.0;
    expectedSegment.at(0, 1) = 0.1;
    expectedSegment.at(0, 2) = NAN;

    expectedSegment.at(1, 0) = 1.0;
    expectedSegment.at(1, 1) = 1.1;
    expectedSegment.at(1, 2) = 1.2;

    expectedSegment.at(2, 0) = 2.0;
    expectedSegment.at(2, 1) = 2.1;
    expectedSegment.at(2, 2) = 2.2;

    MatrixSegmentFinder<float> finder(matrix);
    std::vector<MatrixSegment<float>> segments = finder.getSegments();

    ASSERT_EQ(segments.size(), 1);
    ASSERT_EQ(segments[0].getRows(), expectedSegment.getRows());
    ASSERT_EQ(segments[0].getCols(), expectedSegment.getCols());
    ASSERT_EQ(segments[0].getRowOrigin(), expectedSegment.getRowOrigin());
    ASSERT_EQ(segments[0].getColOrigin(), expectedSegment.getColOrigin());
    ASSERT_EQ(segments[0], expectedSegment);
}

TEST(MatrixSegmentFinderTest, singleValueClusters)
{
    Matrix<float> matrix0{
        {0.0, NAN, NAN, NAN, NAN},
        {NAN, NAN, NAN, NAN, NAN},
        {NAN, NAN, NAN, NAN, NAN},
        {NAN, NAN, NAN, NAN, NAN},
        {NAN, NAN, NAN, NAN, NAN}};
    Matrix<float> matrix1{
        {NAN, NAN, NAN, NAN, NAN},
        {NAN, NAN, NAN, NAN, NAN},
        {NAN, NAN, NAN, NAN, NAN},
        {NAN, NAN, NAN, NAN, NAN},
        {NAN, NAN, NAN, NAN, 4.4}};
    Matrix<float> matrix2{
        {NAN, NAN, NAN, NAN, 0.4},
        {NAN, NAN, NAN, NAN, NAN},
        {NAN, NAN, NAN, NAN, NAN},
        {NAN, NAN, NAN, NAN, NAN},
        {NAN, NAN, NAN, NAN, NAN}};
    Matrix<float> matrix3{
        {NAN, NAN, NAN, NAN, NAN},
        {NAN, NAN, NAN, NAN, NAN},
        {NAN, NAN, NAN, NAN, NAN},
        {NAN, NAN, NAN, NAN, NAN},
        {4.0, NAN, NAN, NAN, NAN}};
    Matrix<float> matrix4{
        {NAN, NAN, NAN, NAN, NAN},
        {NAN, NAN, NAN, NAN, NAN},
        {NAN, NAN, 2.2, NAN, NAN},
        {NAN, NAN, NAN, NAN, NAN},
        {NAN, NAN, NAN, NAN, NAN}};

    MatrixSegment<float> expectedSegment0 = MatrixSegment<float>(1, 1, 0, 0);
    expectedSegment0.at(0, 0) = 0.0;

    MatrixSegment<float> expectedSegment1 = MatrixSegment<float>(1, 1, 4, 4);
    expectedSegment1.at(0, 0) = 4.4;

    MatrixSegment<float> expectedSegment2 = MatrixSegment<float>(1, 1, 0, 4);
    expectedSegment2.at(0, 0) = 0.4;

    MatrixSegment<float> expectedSegment3 = MatrixSegment<float>(1, 1, 4, 0);
    expectedSegment3.at(0, 0) = 4.0;

    MatrixSegment<float> expectedSegment4 = MatrixSegment<float>(1, 1, 2, 2);
    expectedSegment4.at(0, 0) = 2.2;

    MatrixSegmentFinder<float> finder0(matrix0);
    std::vector<MatrixSegment<float>> segments0 = finder0.getSegments();
    ASSERT_EQ(segments0.size(), 1);
    ASSERT_EQ(segments0[0].getRows(), expectedSegment0.getRows());
    ASSERT_EQ(segments0[0].getCols(), expectedSegment0.getCols());
    ASSERT_EQ(segments0[0].getRowOrigin(), expectedSegment0.getRowOrigin());
    ASSERT_EQ(segments0[0].getColOrigin(), expectedSegment0.getColOrigin());
    ASSERT_EQ(segments0[0], expectedSegment0);

    MatrixSegmentFinder<float> finder1(matrix1);
    std::vector<MatrixSegment<float>> segments1 = finder1.getSegments();
    ASSERT_EQ(segments1.size(), 1);
    ASSERT_EQ(segments1[0].getRows(), expectedSegment1.getRows());
    ASSERT_EQ(segments1[0].getCols(), expectedSegment1.getCols());
    ASSERT_EQ(segments1[0].getRowOrigin(), expectedSegment1.getRowOrigin());
    ASSERT_EQ(segments1[0].getColOrigin(), expectedSegment1.getColOrigin());
    ASSERT_EQ(segments1[0], expectedSegment1);

    MatrixSegmentFinder<float> finder2(matrix2);
    std::vector<MatrixSegment<float>> segments2 = finder2.getSegments();
    ASSERT_EQ(segments2.size(), 1);
    ASSERT_EQ(segments2[0].getRows(), expectedSegment2.getRows());
    ASSERT_EQ(segments2[0].getCols(), expectedSegment2.getCols());
    ASSERT_EQ(segments2[0].getRowOrigin(), expectedSegment2.getRowOrigin());
    ASSERT_EQ(segments2[0].getColOrigin(), expectedSegment2.getColOrigin());
    ASSERT_EQ(segments2[0], expectedSegment2);

    MatrixSegmentFinder<float> finder3(matrix3);
    std::vector<MatrixSegment<float>> segments3 = finder3.getSegments();
    ASSERT_EQ(segments3.size(), 1);
    ASSERT_EQ(segments3[0].getRows(), expectedSegment3.getRows());
    ASSERT_EQ(segments3[0].getCols(), expectedSegment3.getCols());
    ASSERT_EQ(segments3[0].getRowOrigin(), expectedSegment3.getRowOrigin());
    ASSERT_EQ(segments3[0].getColOrigin(), expectedSegment3.getColOrigin());
    ASSERT_EQ(segments3[0], expectedSegment3);

    MatrixSegmentFinder<float> finder4(matrix4);
    std::vector<MatrixSegment<float>> segments4 = finder4.getSegments();
    ASSERT_EQ(segments4.size(), 1);
    ASSERT_EQ(segments4[0].getRows(), expectedSegment4.getRows());
    ASSERT_EQ(segments4[0].getCols(), expectedSegment4.getCols());
    ASSERT_EQ(segments4[0].getRowOrigin(), expectedSegment4.getRowOrigin());
    ASSERT_EQ(segments4[0].getColOrigin(), expectedSegment4.getColOrigin());
    ASSERT_EQ(segments4[0], expectedSegment4);
}

TEST(MatrixSegmentFinderTest, testEdgeClusters)
{
    Matrix<float> matrix{
        {0.0, 0.1, NAN, NAN, NAN},
        {1.0, NAN, NAN, NAN, NAN},
        {NAN, NAN, NAN, NAN, NAN},
        {NAN, NAN, NAN, NAN, NAN},
        {NAN, NAN, NAN, NAN, 4.4}};

    // {0.0, 0.1}
    // {1.0, NAN}
    MatrixSegment<float> expectedSegment0 = MatrixSegment<float>(2, 2, 0, 0);
    expectedSegment0.at(0, 0) = 0.0;
    expectedSegment0.at(0, 1) = 0.1;
    expectedSegment0.at(1, 0) = 1.0;
    expectedSegment0.at(1, 1) = NAN;

    // {4.4}
    MatrixSegment<float> expectedSegment1 = MatrixSegment<float>(1, 1, 4, 4);
    expectedSegment1.at(0, 0) = 4.4;

    MatrixSegmentFinder<float> finder(matrix);
    std::vector<MatrixSegment<float>> segments = finder.getSegments();

    ASSERT_EQ(segments.size(), 2);
    ASSERT_EQ(segments[0].getRows(), expectedSegment0.getRows());
    ASSERT_EQ(segments[0].getCols(), expectedSegment0.getCols());
    ASSERT_EQ(segments[0].getRowOrigin(), expectedSegment0.getRowOrigin());
    ASSERT_EQ(segments[0].getColOrigin(), expectedSegment0.getColOrigin());
    ASSERT_EQ(segments[0], expectedSegment0);

    ASSERT_EQ(segments[1].getRows(), expectedSegment1.getRows());
    ASSERT_EQ(segments[1].getCols(), expectedSegment1.getCols());
    ASSERT_EQ(segments[1].getRowOrigin(), expectedSegment1.getRowOrigin());
    ASSERT_EQ(segments[1].getColOrigin(), expectedSegment1.getColOrigin());
    ASSERT_EQ(segments[1], expectedSegment1);
}

TEST(MatrixSegmentFinderTest, testComplexClusters)
{
    Matrix<float> matrix{
        {0.0, 0.1, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN},
        {1.0, 1.1, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN},
        {NAN, NAN, 0.0, 0.1, NAN, NAN, NAN, NAN, NAN, NAN},
        {NAN, NAN, NAN, 1.1, 1.2, NAN, NAN, NAN, NAN, NAN},
        {NAN, NAN, NAN, 2.1, 2.2, 2.3, NAN, NAN, NAN, NAN},
        {NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN},
        {NAN, NAN, NAN, NAN, 0.0, 0.1, 0.2, NAN, NAN, NAN},
        {NAN, NAN, NAN, NAN, NAN, NAN, 1.2, 1.3, NAN, NAN},
        {NAN, NAN, NAN, NAN, NAN, NAN, 2.2, 2.3, 2.4, NAN},
        {NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, 3.4, 3.5}};

    // {0.0, 0.1}
    // {1.0, 1.1}
    MatrixSegment<float> expectedSegment0 = MatrixSegment<float>(2, 2, 0, 0);
    expectedSegment0.at(0, 0) = 0.0;
    expectedSegment0.at(0, 1) = 0.1;
    expectedSegment0.at(1, 0) = 1.0;
    expectedSegment0.at(1, 1) = 1.1;

    // {0.0, 0.1, 0.2}
    // {NAN, 1.2, 1.3}
    // {NAN, 2.2, 2.3}
    MatrixSegment<float> expectedSegment1 = MatrixSegment<float>(3, 4, 2, 2);
    expectedSegment1.at(0, 0) = 0.0;
    expectedSegment1.at(1, 0) = NAN;
    expectedSegment1.at(2, 0) = NAN;

    expectedSegment1.at(0, 1) = 0.1;
    expectedSegment1.at(1, 1) = 1.1;
    expectedSegment1.at(2, 1) = 2.1;

    expectedSegment1.at(0, 2) = NAN;
    expectedSegment1.at(1, 2) = 1.2;
    expectedSegment1.at(2, 2) = 2.2;

    expectedSegment1.at(0, 3) = NAN;
    expectedSegment1.at(1, 3) = NAN;
    expectedSegment1.at(2, 3) = 2.3;

    // {0.0, 0.1, 0.2, NAN, NAN, NAN}
    // {NAN, NAN, 1.2, 1.3, NAN, NAN}
    // {NAN, NAN, 2.2, 2.3, 2.4, NAN}
    // {NAN, NAN, NAN, NAN, 3.4, 3.5}
    MatrixSegment<float> expectedSegment2 = MatrixSegment<float>(4, 6, 6, 4);
    expectedSegment2.at(0, 0) = 0.0;
    expectedSegment2.at(1, 0) = NAN;
    expectedSegment2.at(2, 0) = NAN;
    expectedSegment2.at(3, 0) = NAN;

    expectedSegment2.at(0, 1) = 0.1;
    expectedSegment2.at(1, 1) = NAN;
    expectedSegment2.at(2, 1) = NAN;
    expectedSegment2.at(3, 1) = NAN;

    expectedSegment2.at(0, 2) = 0.2;
    expectedSegment2.at(1, 2) = 1.2;
    expectedSegment2.at(2, 2) = 2.2;
    expectedSegment2.at(3, 2) = NAN;

    expectedSegment2.at(0, 3) = NAN;
    expectedSegment2.at(1, 3) = 1.3;
    expectedSegment2.at(2, 3) = 2.3;
    expectedSegment2.at(3, 3) = NAN;

    expectedSegment2.at(0, 4) = NAN;
    expectedSegment2.at(1, 4) = NAN;
    expectedSegment2.at(2, 4) = 2.4;
    expectedSegment2.at(3, 4) = 3.4;

    expectedSegment2.at(0, 5) = NAN;
    expectedSegment2.at(1, 5) = NAN;
    expectedSegment2.at(2, 5) = NAN;
    expectedSegment2.at(3, 5) = 3.5;

    MatrixSegmentFinder<float> finder(matrix);
    std::vector<MatrixSegment<float>> segments = finder.getSegments();

    ASSERT_EQ(segments.size(), 3);
    ASSERT_EQ(segments[0].getRows(), expectedSegment0.getRows());
    ASSERT_EQ(segments[0].getCols(), expectedSegment0.getCols());
    ASSERT_EQ(segments[0].getRowOrigin(), expectedSegment0.getRowOrigin());
    ASSERT_EQ(segments[0].getColOrigin(), expectedSegment0.getColOrigin());
    ASSERT_EQ(segments[0], expectedSegment0);

    ASSERT_EQ(segments[1].getRows(), expectedSegment1.getRows());
    ASSERT_EQ(segments[1].getCols(), expectedSegment1.getCols());
    ASSERT_EQ(segments[1].getRowOrigin(), expectedSegment1.getRowOrigin());
    ASSERT_EQ(segments[1].getColOrigin(), expectedSegment1.getColOrigin());
    ASSERT_EQ(segments[1], expectedSegment1);

    ASSERT_EQ(segments[2].getRows(), expectedSegment2.getRows());
    ASSERT_EQ(segments[2].getCols(), expectedSegment2.getCols());
    ASSERT_EQ(segments[2].getRowOrigin(), expectedSegment2.getRowOrigin());
    ASSERT_EQ(segments[2].getColOrigin(), expectedSegment2.getColOrigin());
    ASSERT_EQ(segments[2], expectedSegment2);
}

TEST(MatrixSegmentFinderTest, testMergeCorrectness)
{
    Matrix<float> matrix{
        {0.0, 0.1, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN},
        {1.0, 1.1, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN},
        {NAN, NAN, 0.0, 0.1, NAN, NAN, NAN, NAN, NAN, NAN},
        {NAN, NAN, NAN, 1.1, 1.2, NAN, NAN, NAN, NAN, NAN},
        {NAN, NAN, NAN, 2.1, 2.2, 2.3, NAN, NAN, NAN, NAN},
        {NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN},
        {NAN, NAN, NAN, NAN, 0.0, 0.1, 0.2, NAN, NAN, NAN},
        {NAN, NAN, NAN, NAN, NAN, NAN, 1.2, 1.3, NAN, NAN},
        {NAN, NAN, NAN, NAN, NAN, NAN, 2.2, 2.3, 2.4, NAN},
        {NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, 3.4, 3.5}};

    
    MatrixSegmentFinder<float> finder(matrix);
    std::vector<MatrixSegment<float>> segments = finder.getSegments();

    Matrix<float> result = Matrix<float>(matrix.getRows(), matrix.getCols(), NAN);
    for (MatrixSegment<float> segment : segments)
    {
        result.merge(segment);
    }

    ASSERT_EQ(matrix, result);
}