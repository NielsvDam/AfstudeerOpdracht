#include <gtest/gtest.h>
#include <string>
#include <limits>
#include "Matrix.hpp"
#include "MatrixSegment.hpp"

TEST(MatrixConstructors, DefaultContructor)
{
    std::string m0_as_string("Matrix<3,3>\n"
                             "{\n"
                             "0.000000,0.000000,0.000000,\n"
                             "0.000000,0.000000,0.000000,\n"
                             "0.000000,0.000000,0.000000,\n"
                             "}");
    Matrix<double> m0(3, 3);
    EXPECT_EQ(m0_as_string, m0.to_string());

    std::string m1_as_string("Matrix<3,3>\n"
                             "{\n"
                             "1.000000,1.000000,1.000000,\n"
                             "1.000000,1.000000,1.000000,\n"
                             "1.000000,1.000000,1.000000,\n"
                             "}");
    Matrix<double> m1(3, 3, 1);
    EXPECT_EQ(m1_as_string, m1.to_string());
}
TEST(MatrixConstructors, ArrayListConstructor)
{
    std::string m0_as_string("Matrix<3,3>\n"
                             "{\n"
                             "1.000000,2.000000,3.000000,\n"
                             "4.000000,5.000000,6.000000,\n"
                             "7.000000,8.000000,9.000000,\n"
                             "}");
    Matrix<double> m0{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};
    EXPECT_EQ(m0_as_string, m0.to_string());
}
TEST(MatrixConstructors, CopyConstructor)
{
    std::string m0_as_string("Matrix<3,3>\n"
                             "{\n"
                             "1.000000,2.000000,3.000000,\n"
                             "4.000000,5.000000,6.000000,\n"
                             "7.000000,8.000000,9.000000,\n"
                             "}");
    Matrix<double> m0{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};

    Matrix<double> m1 = m0;

    EXPECT_EQ(m0.getRows(), m1.getRows());
    EXPECT_EQ(m0.getCols(), m1.getCols());
    EXPECT_EQ(m0.to_string(), m1.to_string());
    EXPECT_EQ(m0, m1);
}

TEST(MatrixElementAccess, At)
{
    Matrix<double> m0{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};

    EXPECT_THROW(m0.at(m0.getRows() + 1), std::out_of_range);

    EXPECT_THROW(m0.at(m0.getRows(), m0.getCols() + 1), std::out_of_range);

    EXPECT_THROW(m0[m0.getRows() + 1], std::out_of_range);
}

TEST(MatrixOperators, AssignmentOperator)
{
    Matrix<double> m0(3, 3);
    Matrix<double> m1{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};
    m0 = m1;
    EXPECT_EQ(m0, m1);
}
TEST(MatrixOperators, ComparisonOperator)
{
    Matrix<double> m0{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};
    Matrix<double> m1{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};
    EXPECT_EQ(m0, m1);

    Matrix<double> m2{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};
    Matrix<double> m3{{9, 8, 7}, {6, 5, 4}, {3, 2, 1}};
    EXPECT_NE(m2, m3);

    // test different sizes
    Matrix<double> m4{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};
    Matrix<double> m5{{1, 2}, {4, 5}};
    EXPECT_NE(m4, m5);

    // test NaN (not equal)
    Matrix<double> m6{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};
    Matrix<double> m7{{1, 2, 3}, {4, 5, 6}, {7, 8, NAN}};
    EXPECT_NE(m6, m7);

    // test NaN (equal)
    Matrix<double> m8{{1, 2, 3}, {4, 5, 6}, {7, 8, NAN}};
    Matrix<double> m9{{1, 2, 3}, {4, 5, 6}, {7, 8, NAN}};
    EXPECT_EQ(m8, m9);
}
TEST(MatrixOperators, ScalarMultiplication)
{
    Matrix<double> m0{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};
    Matrix<double> m1{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};
    Matrix<double> m2{{1 * 2, 2 * 2, 3 * 2}, {4 * 2, 5 * 2, 6 * 2}, {7 * 2, 8 * 2, 9 * 2}};

    EXPECT_EQ(m2, m1 * 2);
    EXPECT_EQ(m0, m1);
    EXPECT_EQ(m2, m1 *= 2);
    EXPECT_EQ(m2, m1);
}
TEST(MatrixOperators, ScalarDivision)
{
    Matrix<double> m0{{1 * 2, 2 * 2, 3 * 2}, {4 * 2, 5 * 2, 6 * 2}, {7 * 2, 8 * 2, 9 * 2}};
    Matrix<double> m1{{1 * 2, 2 * 2, 3 * 2}, {4 * 2, 5 * 2, 6 * 2}, {7 * 2, 8 * 2, 9 * 2}};
    Matrix<double> m2{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};

    EXPECT_EQ(m2, m1 / 2);
    EXPECT_EQ(m0, m1);
    EXPECT_EQ(m2, m1 /= 2);
    EXPECT_EQ(m2, m1);
}
TEST(MatrixOperators, MatrixAddition)
{
    Matrix<double> m0{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};
    Matrix<double> m1{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};
    Matrix<double> m2{{1 * 2, 2 * 2, 3 * 2}, {4 * 2, 5 * 2, 6 * 2}, {7 * 2, 8 * 2, 9 * 2}};

    EXPECT_EQ(m2, m0 + m1);
    EXPECT_EQ(m0, m1);
    EXPECT_EQ(m2, m1 += m0);
    EXPECT_EQ(m2, m1);
}
TEST(MatrixOperators, MatrixAdditionThrows)
{
    Matrix<double> m0(2, 2, 0.0);
    Matrix<double> m1(3, 2, 0.0);
    EXPECT_THROW(m0 + m1, std::invalid_argument); // sizes not equal
    Matrix<double> m2(2, 2, 0.0);
    Matrix<double> m3(2, 3, 0.0);
    EXPECT_THROW(m2 + m3, std::invalid_argument); // sizes not equal
    Matrix<double> m4(3, 2, 0.0);
    Matrix<double> m5(2, 2, 0.0);
    EXPECT_THROW(m2 += m3, std::invalid_argument); // sizes not equal
    Matrix<double> m6(2, 2, 1.0);
    Matrix<double> m7(2, 2, 2.0);
    EXPECT_NO_THROW(m6 + m7); // test no throw
}
TEST(MatrixOperators, MatrixSubtraction)
{
    Matrix<double> m0{{1 * 2, 2 * 2, 3 * 2}, {4 * 2, 5 * 2, 6 * 2}, {7 * 2, 8 * 2, 9 * 2}};
    Matrix<double> m1{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};
    Matrix<double> m2{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};

    EXPECT_EQ(m2, m0 - m1);
    EXPECT_EQ(m2, m0 -= m1);
}
TEST(MatrixOperators, MatrixSubtractionThrows)
{
    Matrix<double> m0(2, 2, 0.0);
    Matrix<double> m1(3, 2, 0.0);
    EXPECT_THROW(m0 - m1, std::invalid_argument); // sizes not equal
    Matrix<double> m2(2, 2, 0.0);
    Matrix<double> m3(2, 3, 0.0);
    EXPECT_THROW(m2 - m3, std::invalid_argument); // sizes not equal
    Matrix<double> m4(3, 2, 0.0);
    Matrix<double> m5(2, 2, 0.0);
    EXPECT_THROW(m2 -= m3, std::invalid_argument); // sizes not equal
    Matrix<double> m6(2, 2, 1.0);
    Matrix<double> m7(2, 2, 2.0);
    EXPECT_NO_THROW(m6 - m7); // test no throw
}
TEST(MatrixOperators, MatrixMatrixMultiplication)
{
    Matrix<double> m0{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};
    Matrix<double> m1{{30, 36, 42}, {66, 81, 96}, {102, 126, 150}};

    EXPECT_EQ(m1, m0 * m0);
}
TEST(MatrixOperators, MatrixMatrixMultiplicationThrows)
{
    Matrix<double> m0(2, 3, 0.0);
    Matrix<double> m1(2, 2, 0.0);
    EXPECT_THROW(m0 * m1, std::invalid_argument); // check lhs.cols == rhs.rows
    Matrix<double> m2(3, 2, 0.0);
    Matrix<double> m3(2, 3, 0.0);
    EXPECT_NO_THROW(m2 * m3); // should work fine when lhs.rows != rhs.cols
    Matrix<double> m4(2, 2, 0.0);
    Matrix<double> m5(2, 2, 0.0);
    EXPECT_NO_THROW(m4 * m5); // test no throw
}
TEST(MatrixOperators, MatrixColumnVectorMultiplication)
{
    Matrix<double> m0{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};
    Matrix<double> m1{{{1}}, {{2}}, {{3}}};
    Matrix<double> m2{{{14}}, {{32}}, {{50}}};

    Matrix<double> result = m0 * m1;
    // std::cout << "Result of m0 * m1: " << result.to_string() << std::endl;

    EXPECT_EQ(m2, result);
}
TEST(MatrixOperators, MatrixRowVectorMultiplication)
{
    Matrix<double> m0{{1, 2, 3}};
    Matrix<double> m1{{{1}}, {{2}}, {{3}}};
    Matrix<double> m2{{14}};

    Matrix<double> result = m0 * m1;
    // std::cout << "Result of m0 * m1: " << result.to_string() << std::endl;

    EXPECT_EQ(m2, result);
}
TEST(MatrixFunctions, MatrixTranspose)
{
    // See https://en.wikipedia.org/wiki/Transpose
    Matrix<double> m0{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};
    Matrix<double> m1{{9, 8, 7}, {6, 5, 4}, {3, 2, 1}};

    EXPECT_EQ(m0, m0.transpose().transpose());
    EXPECT_EQ((m0 + m1).transpose(), m0.transpose() + m1.transpose());
    EXPECT_EQ((m0 * 4.0).transpose(), m0.transpose() * 4.0);
}
TEST(MatrixFunctions, MatrixIdentity)
{
    Matrix<double> m0{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    Matrix<double> m1{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};

    EXPECT_EQ(m0, m1.identity());
    EXPECT_EQ(m1, m1 * m1.identity());
    EXPECT_EQ(m1, m1.identity() * m1);
}
TEST(MatrixFunctions, MatrixGauss)
{
    Matrix<double> m0{{0, 1, 1, 5}, {3, 2, 2, 13}, {1, -1, 3, 8}};
    Matrix<double> m1(m0.gauss());

    double c = m1.at(2, 3);
    double b = m1.at(1, 3) - m1.at(1, 2) * c;
    double a = m1.at(0, 3) - m1.at(0, 1) * b - m1.at(0, 2) * c;

    EXPECT_NEAR(a, 1.0, 0.00001);
    EXPECT_NEAR(b, 2.0, 0.00001);
    EXPECT_NEAR(c, 3.0, 0.00001);
}
TEST(MatrixFunctions, MatrixGaussJordan)
{
    Matrix<double> m0{{0, 1, 1, 5}, {3, 2, 2, 13}, {1, -1, 3, 8}};
    Matrix<double> m1{{1, 0, 0, 1}, {0, 1, 0, 2}, {0, 0, 1, 3}};
    // EXPECT_EQ(m0.gaussJordan(),m1);
    EXPECT_EQ(true, equals(m0.gaussJordan(), m1, std::numeric_limits<double>::epsilon(), 100));
}
TEST(MatrixFunctions, MatrixSolve)
{
    Matrix<double> m0{{0, 1, 1, 5}, {3, 2, 2, 13}, {1, -1, 3, 8}};
    std::vector<double> m1{1, 2, 3};

    // The following test fails because of rounding errors, use equals instead.
    // EXPECT_EQ(m0.solve(),m1);

    EXPECT_EQ(true, equals(m0.solve(), m1, std::numeric_limits<double>::epsilon(), 100));
}
TEST(MatrixFunctions, MatrixInverse)
{
    Matrix<double> m0{{1, 2, 0}, {1, 0, 1}, {2, 2, 2}};

    EXPECT_EQ(m0.identity(), m0 * m0.inverse());
    EXPECT_EQ(m0.identity(), m0.inverse() * m0);

    Matrix<double> m1{{1, 2, 3}, {0, 1, 5}, {5, 6, 0}};

    // The following tests fail because of rounding errors, use equals instead.
    // EXPECT_EQ( m1.identity(),m1*m1.inverse());
    // EXPECT_EQ( m1.identity(),m1.inverse()*m1);

    EXPECT_EQ(true, equals(m1.identity(), m1 * m1.inverse(), std::numeric_limits<double>::epsilon(), 100));
    EXPECT_EQ(true, equals(m1.identity(), m1.inverse() * m1, std::numeric_limits<double>::epsilon(), 100));
}
TEST(MatrixFunctions, MatrixColumnVectorEquality)
{
    Matrix<double> m0{{{1}}, {{2}}, {{3}}};
    Matrix<double> m1{{{1}}, {{2}}, {{3}}};
    EXPECT_EQ(true, equals(m0, m1 /*,std::numeric_limits<double>::epsilon()*/));
    // test NaN (not equal)
    Matrix<double> m2{{{1}}, {{2}}, {{3}}};
    Matrix<double> m3{{{1}}, {{2}}, {{NAN}}};
    EXPECT_EQ(false, equals(m2, m3 /*,std::numeric_limits<double>::epsilon()*/));
    // test NaN (equal)
    Matrix<double> m4{{{1}}, {{2}}, {{NAN}}};
    Matrix<double> m5{{{1}}, {{2}}, {{NAN}}};
    EXPECT_EQ(true, equals(m4, m5 /*,std::numeric_limits<double>::epsilon()*/));
}
TEST(MatrixFunctions, MatrixRowVectorEquality)
{
    Matrix<double> m0{{1, 2, 3}};
    Matrix<double> m1{{1, 2, 3}};
    EXPECT_EQ(true, equals(m0, m1 /*,std::numeric_limits<double>::epsilon()*/));
    // test NaN (not equal)
    Matrix<double> m2{{1, 2, 3}};
    Matrix<double> m3{{1, 2, NAN}};
    EXPECT_EQ(false, equals(m2, m3 /*,std::numeric_limits<double>::epsilon()*/));
    // test NaN (equal)
    Matrix<double> m4{{1, 2, NAN}};
    Matrix<double> m5{{1, 2, NAN}};
    EXPECT_EQ(true, equals(m4, m5 /*,std::numeric_limits<double>::epsilon()*/));
}

TEST(MatrixFunctions, GetRowVector)
{
    Matrix<double> m0{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};
    std::vector<double> m1{1, 2, 3};
    std::vector<double> m2{4, 5, 6};
    std::vector<double> m3{7, 8, 9};

    EXPECT_EQ(m1, m0.getRowVector(0));
    EXPECT_EQ(m2, m0.getRowVector(1));
    EXPECT_EQ(m3, m0.getRowVector(2));
}

TEST(MatrixFunctions, GetRowVectorThrows)
{
    Matrix<double> m0(2, 2, 0.0);
    EXPECT_THROW(m0.getRowVector(m0.getRows() + 1), std::out_of_range);
}

TEST(MatrixFunctions, SetRowVector)
{
    Matrix<double> m0{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};
    std::vector<double> m1{1, 2, 3};
    std::vector<double> m2{4, 5, 6};
    std::vector<double> m3{7, 8, 9};

    m0.setRowVector(m1, 0);
    m0.setRowVector(m2, 1);
    m0.setRowVector(m3, 2);

    EXPECT_EQ(m1, m0.getRowVector(0));
    EXPECT_EQ(m2, m0.getRowVector(1));
    EXPECT_EQ(m3, m0.getRowVector(2));
}

TEST(MatrixFunctions, SetRowVectorThrows)
{
    Matrix<double> m0(2, 2, 0.0);
    std::vector<double> m1{1, 2, 3};
    EXPECT_THROW(m0.setRowVector(m1, m0.getRows() + 1), std::invalid_argument); // sizes not equal

    Matrix<double> m2(2, 2, 0.0);
    std::vector<double> m3{1, 2};
    EXPECT_THROW(m2.setRowVector(m3, m2.getRows() + 1), std::out_of_range); // index out of range

    // test no throw
    Matrix<double> m4(2, 2, 0.0);
    std::vector<double> m5{1, 2};
    EXPECT_NO_THROW(m4.setRowVector(m5, 0));
}

TEST(MatrixFunctions, GetColumnVector)
{
    Matrix<double> m0{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};
    std::vector<double> m1{1, 4, 7};
    std::vector<double> m2{2, 5, 8};
    std::vector<double> m3{3, 6, 9};

    EXPECT_EQ(m1, m0.getColumnVector(0));
    EXPECT_EQ(m2, m0.getColumnVector(1));
    EXPECT_EQ(m3, m0.getColumnVector(2));
}

TEST(MatrixFunctions, GetColumnVectorThrows)
{
    Matrix<double> m0(2, 2, 0.0);
    EXPECT_THROW(m0.getColumnVector(m0.getCols() + 1), std::out_of_range);

    // test no throw
    Matrix<double> m1(2, 2, 0.0);
    EXPECT_NO_THROW(m1.getColumnVector(0));
}

TEST(MatrixFunctions, SetColumnVector)
{
    Matrix<double> m0{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};
    std::vector<double> m1{1, 4, 7};
    std::vector<double> m2{2, 5, 8};
    std::vector<double> m3{3, 6, 9};

    m0.setColumnVector(m1, 0);
    m0.setColumnVector(m2, 1);
    m0.setColumnVector(m3, 2);

    EXPECT_EQ(m1, m0.getColumnVector(0));
    EXPECT_EQ(m2, m0.getColumnVector(1));
    EXPECT_EQ(m3, m0.getColumnVector(2));
}

TEST(MatrixFunctions, SetColumnVectorThrows)
{
    Matrix<double> m0(2, 2, 0.0);
    std::vector<double> m1{1, 2, 3};
    EXPECT_THROW(m0.setColumnVector(m1, 0), std::invalid_argument); // sizes not equal

    Matrix<double> m2(2, 2, 0.0);
    std::vector<double> m3{1, 2};
    EXPECT_THROW(m2.setColumnVector(m3, m2.getCols() + 1), std::out_of_range); // index out of range

    // test no throw
    Matrix<double> m4(2, 2, 0.0);
    std::vector<double> m5{1, 2};
    EXPECT_NO_THROW(m4.setColumnVector(m5, 0));
}

TEST(MatrixFunctions, TestMerge)
{
    Matrix<float> matrixA{
        {NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN},
        {NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN},
        {NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN},
        {NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN},
        {NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN},
        {NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN},
        {NAN, NAN, NAN, NAN, 0.0, 0.1, 0.2, NAN, NAN, NAN},
        {NAN, NAN, NAN, NAN, NAN, NAN, 1.2, 1.3, NAN, NAN},
        {NAN, NAN, NAN, NAN, NAN, NAN, 2.2, 2.3, 2.4, NAN},
        {NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, 3.4, 3.5}};
    Matrix<float> matrixB{
        {0.0, 0.1, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN},
        {1.0, 1.1, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN},
        {NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN},
        {NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN},
        {NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN},
        {NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN},
        {NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN},
        {NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN},
        {NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN},
        {NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN}};
    Matrix<float> expectedResult1{
        {0.0, 0.1, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN},
        {1.0, 1.1, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN},
        {NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN},
        {NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN},
        {NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN},
        {NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN},
        {NAN, NAN, NAN, NAN, 0.0, 0.1, 0.2, NAN, NAN, NAN},
        {NAN, NAN, NAN, NAN, NAN, NAN, 1.2, 1.3, NAN, NAN},
        {NAN, NAN, NAN, NAN, NAN, NAN, 2.2, 2.3, 2.4, NAN},
        {NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, 3.4, 3.5}};

    matrixA.merge(matrixB);
    EXPECT_EQ(expectedResult1, matrixA);

    // test avering feature
    Matrix<float> matrixC{{10.0, 20.0, 30.0}};
    Matrix<float> matrixD{{1.0, 2.0, 3.0}};
    Matrix<float> expectedResult2{{5.5, 11.0, 16.5}};

    matrixC.merge(matrixD);
    EXPECT_EQ(expectedResult2, matrixC);
}

TEST(MatrixFunctions, TestMergeSegments)
{
    Matrix<float> expected{
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

    // {0.0, 0.1},
    // {1.0, 1.1},
    MatrixSegment<float> segment0 = MatrixSegment<float>(2, 2, 0, 0);
    segment0.at(0, 0) = 0.0;
    segment0.at(0, 1) = 0.1;
    segment0.at(1, 0) = 1.0;
    segment0.at(1, 1) = 1.1;

    // {0.0, 0.1, NAN, NAN}
    // {NAN, 1.1, 1.2, NAN}
    // {NAN, 2.1, 2.2, 2.3}
    MatrixSegment<float> segment1 = MatrixSegment<float>(3, 4, 2, 2);
    segment1.at(0, 0) = 0.0;
    segment1.at(1, 0) = NAN;
    segment1.at(2, 0) = NAN;

    segment1.at(0, 1) = 0.1;
    segment1.at(1, 1) = 1.1;
    segment1.at(2, 1) = 2.1;

    segment1.at(0, 2) = NAN;
    segment1.at(1, 2) = 1.2;
    segment1.at(2, 2) = 2.2;

    segment1.at(0, 3) = NAN;
    segment1.at(1, 3) = NAN;
    segment1.at(2, 3) = 2.3;

    // {0.0, 0.1, 0.2, NAN, NAN, NAN}
    // {NAN, NAN, 1.2, 1.3, NAN, NAN}
    // {NAN, NAN, 2.2, 2.3, 2.4, NAN}
    // {NAN, NAN, NAN, NAN, 3.4, 3.5}
    MatrixSegment<float> segment2 = MatrixSegment<float>(4, 6, 6, 4);
    segment2.at(0, 0) = 0.0;
    segment2.at(1, 0) = NAN;
    segment2.at(2, 0) = NAN;
    segment2.at(3, 0) = NAN;

    segment2.at(0, 1) = 0.1;
    segment2.at(1, 1) = NAN;
    segment2.at(2, 1) = NAN;
    segment2.at(3, 1) = NAN;

    segment2.at(0, 2) = 0.2;
    segment2.at(1, 2) = 1.2;
    segment2.at(2, 2) = 2.2;
    segment2.at(3, 2) = NAN;

    segment2.at(0, 3) = NAN;
    segment2.at(1, 3) = 1.3;
    segment2.at(2, 3) = 2.3;
    segment2.at(3, 3) = NAN;

    segment2.at(0, 4) = NAN;
    segment2.at(1, 4) = NAN;
    segment2.at(2, 4) = 2.4;
    segment2.at(3, 4) = 3.4;

    segment2.at(0, 5) = NAN;
    segment2.at(1, 5) = NAN;
    segment2.at(2, 5) = NAN;
    segment2.at(3, 5) = 3.5;

    Matrix<float> matrix(expected.getRows(), expected.getCols(), NAN);
    matrix.merge(segment0);
    matrix.merge(segment1);
    matrix.merge(segment2);
    EXPECT_EQ(expected, matrix);
}

TEST(MatrixFunctions, TestMergeSegmentsThrows)
{
    Matrix<float> matrix(2, 2, NAN);
    MatrixSegment<float> segment(3, 3, 0, 0);
    EXPECT_THROW(matrix.merge(segment), std::invalid_argument); // size exceeds matrix size, impossible to merge

    Matrix<float> matrix2(4, 4, NAN);
    MatrixSegment<float> segment2(2, 2, 3, 0);
    EXPECT_THROW(matrix2.merge(segment2), std::invalid_argument); // row origin exceeds matrix size, impossible to merge

    Matrix<float> matrix3(4, 4, NAN);
    MatrixSegment<float> segment3(2, 2, 0, 3);
    EXPECT_THROW(matrix3.merge(segment3), std::invalid_argument); // col origin exceeds matrix size, impossible to merge

    Matrix<float> matrix4(4, 4, NAN);
    MatrixSegment<float> segment4(2, 2, 3, 3);
    EXPECT_THROW(matrix4.merge(segment4), std::invalid_argument); // row and col origin exceeds matrix size, impossible to merge
}   