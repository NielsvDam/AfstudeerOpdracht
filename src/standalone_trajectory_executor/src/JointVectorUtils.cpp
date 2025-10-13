#include "JointVectorUtils.hpp"

/* static */ void JointVectorUtils::order(std::vector<std::string>& nameVector, std::vector<double>& positionVector)
{
    // Verify that the vectors are of the same size, throw an error if they are not
    if (nameVector.size() != positionVector.size())
    {
        throw std::runtime_error(
            "Error: cannot order vectors of different sizes. "
            "nameVector size: " +
            std::to_string(nameVector.size()) + ", positionVector size: " + std::to_string(positionVector.size()));
    }
    // Create a vector of indices, which will be sorted based on the joint names
    std::vector<size_t> indices(nameVector.size());
    // Fill the indices vector with the numbers 0 to nameVector.size()
    std::iota(indices.begin(), indices.end(), 0);
    // Sort the indices based on the joint names
    std::sort(indices.begin(), indices.end(), [&nameVector](size_t a, size_t b) { return nameVector[a] < nameVector[b]; });
    // Create new vectors for the sorted joint names and positions
    std::vector<std::string> sortedJointNameVector(nameVector.size());
    std::vector<double> sortedJointPositionVector(positionVector.size());
    // Fill the new vectors with the sorted joint names and positions
    for (size_t i = 0; i < indices.size(); ++i)
    {
        // Copy the joint names and positions to the new vectors
        sortedJointNameVector[i] = nameVector[indices[i]];
        sortedJointPositionVector[i] = positionVector[indices[i]];
    }
    // Move the sorted vectors to the original vectors, to avoid copying
    nameVector = std::move(sortedJointNameVector);
    positionVector = std::move(sortedJointPositionVector);
}

/* static */ bool JointVectorUtils::equals(
    const std::vector<double>& jointVectorA,
    const std::vector<double>& jointVectorB,
    double epsilon)
{
    // Verify that the vectors are of the same size, this is a precondition.
    if (jointVectorA.size() != jointVectorB.size())
    {
        throw std::runtime_error(
            "Error: cannot compare vectors of different sizes. "
            "Vector A size: " +
            std::to_string(jointVectorA.size()) + ", Vector B size: " + std::to_string(jointVectorB.size()));
    }
    // Compare each element of the vectors
    epsilon = std::abs(epsilon);
    return std::equal(
        jointVectorA.begin(),
        jointVectorA.end(),
        jointVectorB.begin(),
        jointVectorB.end(),
        [epsilon](double a, double b) {
            return std::abs(a - b) <= epsilon;
        });
}