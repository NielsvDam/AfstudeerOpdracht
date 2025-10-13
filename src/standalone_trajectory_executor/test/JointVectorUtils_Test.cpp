#include <gtest/gtest.h>
#include <memory>
#include <vector>
#include <accessor/accessor.hpp>
#include "JointVectorUtils.hpp"

/**
 * @brief Test if the equals function throws an exception when the vectors are of different sizes.
 *
 * Expected result: runtime_error should be thrown.
 */
TEST(JointVectorUtilsTest, TestEqualsFunctionIllegalSizes)
{
    // Create vectors of different sizes
    std::vector<double> jointValues1 = {0.4, 0.6, 0.8};
    std::vector<double> jointValues2 = {0.4, 0.6, 0.8, 0.7};
    double epsilon = 0.01;
    // Check if the vectors throw an exception
    EXPECT_THROW(JointVectorUtils::equals(jointValues1, jointValues2, epsilon), std::runtime_error);
}

/**
 * @brief Test if the order function throws an exception when the vectors are of different sizes.
 *
 * Expected result: runtime_error should be thrown.
 */
TEST(JointVectorUtilsTest, TestOrderFunctionIllegalSizes)
{
    // Create vectors of different sizes
    std::vector<std::string> jointNames1 = {"joint3", "joint1", "joint5", "joint2", "joint4", "joint6", "joint8"};
    std::vector<double> jointValues1 = {0.4, 0.6, 0.8, 0.7};
    // Check if the vectors throw an exception
    EXPECT_THROW(JointVectorUtils::order(jointNames1, jointValues1), std::runtime_error);
}

/**
 * @brief Test if the order function orders the vectors correctly.
 *
 * Expected result: The vectors should be ordered in the same way.
 */
TEST(JointVectorUtilsTest, TestOrderJointVectorFunction)
{
    // Create vectors of joint names and positions without any order
    std::vector<std::string> jointNames1 = {"joint3", "joint1", "joint5", "joint2", "joint4", "joint6", "joint8", "joint7"};
    std::vector<double> jointValues1 = {0.3, 0.6, 0.1, 0.7, 0.5, 0.8, 0.2, 0.4};

    // Create vectors of joint names and positions without any order
    std::vector<std::string> jointNames2 = {"joint2", "joint4", "joint1", "joint3", "joint5", "joint6", "joint8", "joint7"};
    std::vector<double> jointValues2 = {0.7, 0.5, 0.6, 0.3, 0.1, 0.8, 0.2, 0.4};

    // check if the vectors are not equal
    EXPECT_NE(jointNames1, jointNames2);
    EXPECT_NE(jointValues1, jointValues2);

    // order the vectors
    JointVectorUtils::order(jointNames1, jointValues1);
    JointVectorUtils::order(jointNames2, jointValues2);

    // check if the vectors are equal
    EXPECT_EQ(jointNames1, jointNames2);
    EXPECT_EQ(jointValues1, jointValues2);
}

/**
 * @brief Test if the equals function compares vectors correctly.
 *
 * Expected result: It should be precisely true within the given epsilon tolerance and only then.
 */
TEST(JointVectorUtilsTest, TestEqualsFunction)
{
    // Create vectors of joint names and positions
    std::vector<double> jointValues1 = {0.4, 0.6, 0.8, 0.7};
    std::vector<double> jointValues2 = {0.4, 0.6, 0.8, 0.7};
    double epsilon = 0.01;

    // Check if the vectors are equal
    EXPECT_TRUE(JointVectorUtils::equals(jointValues1, jointValues2, epsilon));

    // test what is just allowed (in the + range)
    std::vector<double> jointValues3 = {0.4, 0.6, 0.8, 0.709};
    EXPECT_TRUE(JointVectorUtils::equals(jointValues1, jointValues3, epsilon));

    /// test what is just allowed (in the - range)
    std::vector<double> jointValues4 = {0.4, 0.6, 0.8, 0.691};
    EXPECT_TRUE(JointVectorUtils::equals(jointValues1, jointValues4, epsilon));

    // test what is just not allowed (in the + range)
    std::vector<double> jointValues5 = {0.4, 0.6, 0.8, 0.711};
    EXPECT_FALSE(JointVectorUtils::equals(jointValues1, jointValues5, epsilon));

    // test what is just not allowed (in the - range)
    std::vector<double> jointValues6 = {0.4, 0.6, 0.8, 0.689};
    EXPECT_FALSE(JointVectorUtils::equals(jointValues1, jointValues6, epsilon));
}
/**
 * @brief Test the equals function with a higher precision.
 *
 * Expected result: It should work just the same, but with a very small epsilon.
 */
TEST(JointVectorUtilsTest, TestEqualsFunctionHighPrecision)
{
    // Create vectors of joint names and positions
    std::vector<double> jointValues1 = {0.4, 0.6, 0.8, 0.7};
    std::vector<double> jointValues2 = {0.4, 0.6, 0.8, 0.7};
    double epsilon = 0.000001; // test with higher precision

    // Check if the vectors are equal
    EXPECT_TRUE(JointVectorUtils::equals(jointValues1, jointValues2, epsilon));

    // test what is just allowed (in the + range)
    std::vector<double> jointValues3 = {0.4, 0.6, 0.8, 0.7000009};
    EXPECT_TRUE(JointVectorUtils::equals(jointValues1, jointValues3, epsilon));

    /// test what is just allowed (in the - range)
    std::vector<double> jointValues4 = {0.4, 0.6, 0.8, 0.6999991};
    EXPECT_TRUE(JointVectorUtils::equals(jointValues1, jointValues4, epsilon));

    // test what is just not allowed (in the + range)
    std::vector<double> jointValues5 = {0.4, 0.6, 0.8, 0.7100001};
    EXPECT_FALSE(JointVectorUtils::equals(jointValues1, jointValues5, epsilon));

    // test what is just not allowed (in the - range)
    std::vector<double> jointValues6 = {0.4, 0.6, 0.8, 0.699989};
    EXPECT_FALSE(JointVectorUtils::equals(jointValues1, jointValues6, epsilon));
}
/**
 * @brief Test the equals function with a negative epsilon.
 *
 * Expected result: It should work just the same
 */
TEST(JointVectorUtilsTest, TestEqualsFunctionNegativeEpsilon)
{
    // Create vectors of joint names and positions
    std::vector<double> jointValues1 = {0.4, 0.6, 0.8, 0.7};
    std::vector<double> jointValues2 = {0.4, 0.6, 0.8, 0.7};
    double epsilon = -0.01;

    // Check if the vectors are equal
    EXPECT_TRUE(JointVectorUtils::equals(jointValues1, jointValues2, epsilon));

    // Create vectors with one value slightly above the threshold
    std::vector<double> jointValues3 = {0.4, 0.6, 0.8, 0.71};
    EXPECT_FALSE(JointVectorUtils::equals(jointValues1, jointValues3, epsilon));

    // Create vectors with one value slightly below the threshold
    std::vector<double> jointValues4 = {0.4, 0.6, 0.8, 0.69};
    EXPECT_FALSE(JointVectorUtils::equals(jointValues1, jointValues4, epsilon));
}