
#include <opencv2/opencv.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include "Matrix.hpp"
#include "Conversion.hpp"

// #define PROTOTYPE_PROCES // Uncomment this to enable a currently partially functional detection system for cubes: They are still send to the system as 18mm cubes!

template <class T>
MatrixSegment<T>::~MatrixSegment()
{}

template <class T>
MatrixSegment<T>::MatrixSegment(std::size_t rows, std::size_t cols, std::size_t rowOrigin, std::size_t colOrigin)
    : Matrix<T>(rows, cols, NAN), rowOrigin(rowOrigin), colOrigin(colOrigin)
{}

template <class T>
void MatrixSegment<T>::detectBlocks()
{
    std::vector<Cell<T>> cells = getOccupiedCells();
    // Convert points to a format suitable for OpenCV and calculate the average height
    std::vector<cv::Point2f> cvPoints;
    T avgHeight = 0;
    for (const Cell<T>& cell : cells)
    {
        avgHeight += cell.value;
        cvPoints.emplace_back(cell.row, cell.col);
    }
    avgHeight /= cells.size();

    // Use OpenCV to find the minimum area rectangle
    cv::RotatedRect minRect = cv::minAreaRect(cvPoints);
    // Get the four points of the rectangle
    cv::Point2f boxPoints[4];
    minRect.points(boxPoints);

    // Check if the rectangle size is within the allowed range
    float width = minRect.size.width * CELL_SIZE;
    float height = minRect.size.height * CELL_SIZE;



    // Check if the rectangle represents more than one block
    if (std::abs(width - BLOCK_SIZE) > SINGLE_BLOCK_DETECTION_TOLERANCE ||
        std::abs(height - BLOCK_SIZE) > SINGLE_BLOCK_DETECTION_TOLERANCE)
    {
        #ifdef PROTOTYPE_PROCES
        // Segmentation can be carried out here, as well as before the seperation of segments: Though a more aggresive segmentation can be applied here to segment even closer blocks or parts.
        // For now, a shorter implementation to also return blocks within the boundaries of the gripper: Currently these are 12mm to 24mm.
        if ((std::abs(width) > GRIPPER_MAXIMUM_RANGE || std::abs(height) > GRIPPER_MAXIMUM_RANGE) || (std::abs(width) < GRIPPER_MINIMUM_RANGE || std::abs(height) < GRIPPER_MINIMUM_RANGE)) { return; } // Object is too big.
        cv::Point2f rectCenter = minRect.center;
        float angle = minRect.angle;
        avgHeight += (width+height)/4; // A bit of a shortcut, but a cube has the same lengths for each dimension, average height & width for slightly more reliable number.

        float rowPoint = rectCenter.x + rowOrigin;
        float colPoint = rectCenter.y + colOrigin;

        geometry_msgs::msg::Pose pose =
            Conversion::matrixPointToPose(rowPoint, colPoint, avgHeight, 0.0, 180.0 * M_PI / 180.0, angle * M_PI / 180.0); // Turn it back into a quaternion, see below this if statement.

        poses.push_back(pose); // Add the result to the list to be send back. !!! - Currently, these will be displayed as 18mm blocks, with currently no way to transfer scale data.
                                // Adding a custom edit of the Pose type to include detected scale would be useful.
        #endif

        // TODO: implement algorithm to detect multiple blocks in segment.
        //
        // Advice: Test erode filter on contours, since schrinking the contours results in very usable information about
        // multiple blocks in a cluster.
        // This idea has been found by experimenting with offset lines in Fusion360 (see /docs/schrink.gif)

        // Erode > Seperate/Identify individual objects > Dilate.
        // In (very) short terms: To seperate individual objects, be it for detecting close objects or stacked/piled, segmentation algorithms are used. Something like a watershed algorithm tends to work.
        return;
    }

    // Get the center of the rectangle
    cv::Point2f rectCenter = minRect.center;

    // Get the angle of the rectangle
    float angle = minRect.angle;

    // Create the pose
    avgHeight += BLOCK_SIZE / 2;
    // TODO: applying offsets here is a bit of a hack. The found poses in the matrix should be stored and converted in a
    // seperate method
    float rowPoint = rectCenter.x + rowOrigin;
    float colPoint = rectCenter.y + colOrigin;
    geometry_msgs::msg::Pose pose =
        Conversion::matrixPointToPose(rowPoint, colPoint, avgHeight, 0.0, 180.0 * M_PI / 180.0, angle * M_PI / 180.0);

    // Convert the angle to a quaternion
    poses.push_back(pose);
}

template <class T>
std::size_t MatrixSegment<T>::getRowOrigin() const
{
    return rowOrigin;
}

template <class T>
std::size_t MatrixSegment<T>::getColOrigin() const
{
    return colOrigin;
}

template <class T>
std::size_t MatrixSegment<T>::getCellCount()
{
    return getOccupiedCells().size();
}

template <class T>
std::size_t MatrixSegment<T>::getBlockCount()
{
    return poses.size();
}

template <class T>
std::vector<geometry_msgs::msg::Pose> MatrixSegment<T>::getOffsetPoses()
{
    return poses;
}

template <class T>
std::vector<Cell<T>> MatrixSegment<T>::getOccupiedCells()
{
    std::vector<Cell<T>> cells;
    for (std::size_t row = 0; row < this->getRows(); ++row)
    {
        for (std::size_t col = 0; col < this->getCols(); ++col)
        {
            if (!std::isnan(this->at(row, col)))
            {
                cells.emplace_back(row, col, this->at(row, col));
            }
        }
    }
    return cells;
}
