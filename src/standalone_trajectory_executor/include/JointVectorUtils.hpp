#ifndef JOINTVECTORUTILS_HPP
#define JOINTVECTORUTILS_HPP

#include <vector>
#include <string>
#include <algorithm>
#include <numeric>
#include <stdexcept>

/**
 * @class JointVectorUtils
 * @brief A utility class for operations on vectors of joint names and positions.
 *
 * This class provides static methods to order joint names and positions, and to compare vectors of joint positions
 * with a epsilon tolerance. The class is non-instantiable and non-copyable.
 */
class JointVectorUtils
{
public:
    // Delete anything that can make this class instantiable or copyable
    JointVectorUtils() = delete;
    ~JointVectorUtils() = delete;
    JointVectorUtils(const JointVectorUtils&) = delete;
    JointVectorUtils& operator=(const JointVectorUtils&) = delete;
    /**
     * @brief Orders joint names and corresponding positions in alphabetical order the names.
     *
     * This function takes two vectors: one containing joint names and the other containing corresponding joint
     * positions. It sorts them based on the alphabetical order of the joint names.
     *
     * @note This method uses the indirect sorting algorithm.
     *
     * @param nameVector A vector of joint names.
     * @param positionVector A vector of joint positions, with corresponding indices to the nameVector.
     *
     * @throws std::runtime_error If the size of nameVector and positionVector are not equal.
     */
    static void order(std::vector<std::string>& jointNameVector, std::vector<double>& jointPositionVector);
    /**
     * @brief Compares two vectors of for equality, given an epsilon tolerance.
     *
     * @param jointVectorA The first vector of joint positions.
     * @param jointVectorB The second vector of joint positions.
     * @param epsilon The tolerance within which joint positions are considered equal.
     * @return true if the vectors are equal within the specified tolerance, false otherwise.
     *
     * @throws std::runtime_error if the vectors are of different sizes.
     */
    static bool equals(const std::vector<double>& jointVectorA, const std::vector<double>& jointVectorB, double epsilon);
private:
};

#endif // JOINTVECTORUTILS_HPP
