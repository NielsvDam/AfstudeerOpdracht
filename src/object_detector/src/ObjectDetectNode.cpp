#include "ObjectDetectNode.hpp"

#include <cmath>                                   // NAN
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> // tf2::doTransform for geometry_msgs::msg::PoseStamped
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>     // tf2::doTransform for sensor_msgs::msg::PointCloud2
#include <std_msgs/msg/header.hpp>                 // std_msgs::msg::Header

#include "MatrixSegmentFinder.hpp"
#include "MatrixSegment.hpp"
#include "MatrixFilters.hpp"
#include "Conversion.hpp"

// #define DELAYED
// Additions to allow for temporary delay to be added. Not needed when simulating or when running with static camera.
#ifdef DELAYED

#define START_DELAY 500
#define END_DELAY 100

#include <thread>
#include <chrono>

#endif

ObjectDetectNode::~ObjectDetectNode() {}

ObjectDetectNode::ObjectDetectNode()
    : Node("object_detect_node", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)),
      // Create the action to take a picture
      pictureServer(rclcpp_action::create_server<custom_msgs::action::Picture>(
          this,
          "/take_picture",
          std::bind(&ObjectDetectNode::handlePictureActionGoal, this, std::placeholders::_1, std::placeholders::_2),
          std::bind(&ObjectDetectNode::handlePictureActionCancel, this, std::placeholders::_1),
          std::bind(&ObjectDetectNode::handlePictureActionAccepted, this, std::placeholders::_1))),
      originalMatrixPublisher(this->create_publisher<sensor_msgs::msg::PointCloud2>("/object_detect/original_matrix", 10)),
      // Create a publisher for the dubuggin pointcloud
      debuggingMatrixPublisher(this->create_publisher<sensor_msgs::msg::PointCloud2>("/object_detect/debugging_matrix", 10)),
      // Create a publisher for the filtered point cloud
      detectionMatrixPublisher(this->create_publisher<sensor_msgs::msg::PointCloud2>("/object_detect/detection_matrix", 10)),
      // Create a publisher for the markers
      markerPublisher(this->create_publisher<visualization_msgs::msg::MarkerArray>("/object_detect/markers", 10)),
      // Create a publisher for the poses
      posePublisher(this->create_publisher<geometry_msgs::msg::PoseArray>("/object_detect/poses", 10)),
      // Create a static transform broadcaster, to publish the camera transform
      transformBroadcaster(std::make_shared<tf2_ros::StaticTransformBroadcaster>(this)),
      // Create a parameter event handler
      paramSubscriber(std::make_shared<rclcpp::ParameterEventHandler>(this)),
      tfBuffer(this->get_clock()),
      tfListener(tfBuffer),
      useFakeDetector(this->get_parameter("use_fake_detector").as_bool()),
      fakeDetectorIndexCounter(0)
{
    RCLCPP_INFO(this->get_logger(), "Object detect node started");

    // Publish the initial camera transform
    this->publishCameraTransform();

    // Set a callback for the camera tuning parameters
    auto parameterUpdatedCallback = [this](const rclcpp::Parameter& parameter) {
        RCLCPP_INFO(
            this->get_logger(),
            "Received an update to parameter \"%s\" of type %s: \"%f\"",
            parameter.get_name().c_str(),
            parameter.get_type_name().c_str(),
            parameter.as_double());
        this->publishCameraTransform();
    };
    // the handles have to be stored, otherwise the callback will be removed
    callbackHandleX = paramSubscriber->add_parameter_callback("camera_tuning_x", parameterUpdatedCallback);
    callbackHandleY = paramSubscriber->add_parameter_callback("camera_tuning_y", parameterUpdatedCallback);
    callbackHandleZ = paramSubscriber->add_parameter_callback("camera_tuning_z", parameterUpdatedCallback);
    callbackHandleRoll = paramSubscriber->add_parameter_callback("camera_tuning_roll", parameterUpdatedCallback);
    callbackHandlePitch = paramSubscriber->add_parameter_callback("camera_tuning_pitch", parameterUpdatedCallback);
    callbackHandleYaw = paramSubscriber->add_parameter_callback("camera_tuning_yaw", parameterUpdatedCallback);
}

void ObjectDetectNode::publishCameraTransform()
{
    double x = this->get_parameter("camera_tuning_x").as_double();
    double y = this->get_parameter("camera_tuning_y").as_double();
    double z = this->get_parameter("camera_tuning_z").as_double();
    double roll = this->get_parameter("camera_tuning_roll").as_double();
    double pitch = this->get_parameter("camera_tuning_pitch").as_double();
    double yaw = this->get_parameter("camera_tuning_yaw").as_double();

    tf2::Quaternion quaternion;
    quaternion.setRPY(roll, pitch, yaw);

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = this->get_clock()->now();
    transform.header.frame_id = "camera_tcp"; // HARDCODED, TODO | rv5as_camera_tcp
    transform.child_frame_id = "camera_link"; // HARDCODED, TODO | camera_link // Disabled to test the effects.
    transform.transform.translation.x = x;
    transform.transform.translation.y = y;
    transform.transform.translation.z = z;
    transform.transform.rotation.x = quaternion.x();
    transform.transform.rotation.y = quaternion.y();
    transform.transform.rotation.z = quaternion.z();
    transform.transform.rotation.w = quaternion.w();
    // send the transform
    transformBroadcaster->sendTransform(transform);
    RCLCPP_INFO(
        this->get_logger(),
        "Published camera transform with translation: [%f, %f, %f] and quaternion: [%f, %f, %f, %f]",
        x,
        y,
        z,
        quaternion.x(),
        quaternion.y(),
        quaternion.z(),
        quaternion.w());
}

/* static */ std::shared_ptr<ObjectDetectNode> ObjectDetectNode::getInstance()
{
    static std::shared_ptr<ObjectDetectNode> instance{new ObjectDetectNode};
    return instance;
}

rclcpp_action::GoalResponse ObjectDetectNode::handlePictureActionGoal(
    const rclcpp_action::GoalUUID& uuid,
    // NOLINTNEXTLINE (performance-unnecessary-value-param) // can't change definition in ros sourcecode
    const std::shared_ptr<const custom_msgs::action::Picture::Goal> goal)
{
    (void)uuid; // suppress unused parameter warning
    (void)goal; // suppress unused parameter warning
    RCLCPP_INFO(this->get_logger(), "Picture action request received");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ObjectDetectNode::handlePictureActionCancel(
    // NOLINTNEXTLINE (performance-unnecessary-value-param) // can't change definition in ros sourcecode
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<custom_msgs::action::Picture>> goal_handle)
{
    (void)goal_handle; // suppress unused parameter warning
    RCLCPP_INFO(this->get_logger(), "Cancel requested is not yet implemented. Rejecting request.");
    // TODO: Implement cancel logic, however there currently is no demand for this.
    return rclcpp_action::CancelResponse::REJECT;
}

void ObjectDetectNode::handlePictureActionAccepted(
    // NOLINTNEXTLINE (performance-unnecessary-value-param) // can't change definition in ros sourcecode
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<custom_msgs::action::Picture>> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Picture action accepted");
    if (useFakeDetector)
    {
        std::thread{std::bind(&ObjectDetectNode::executeFakePictureAction, this, std::placeholders::_1), goal_handle}
            .detach();
    }
    else
    {
        std::thread{std::bind(&ObjectDetectNode::executePictureAction, this, std::placeholders::_1), goal_handle}.detach();
    }
}

void ObjectDetectNode::executeFakePictureAction(
    // NOLINTNEXTLINE (performance-unnecessary-value-param) // can't change definition in ros sourcecode
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<custom_msgs::action::Picture>> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "picture action accepted");
    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Simulate the time it takes to take a picture
    // send feedback that the picture has been taken.
    auto feedback = std::make_shared<custom_msgs::action::Picture::Feedback>();
    feedback->picture_taken = true;
    RCLCPP_INFO(this->get_logger(), "picture taken, publishing feedback");
    goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(this->get_logger(), "feedback has been published");

    // Retrieve the dictionary of poses from the parameter
    std::map<std::string, rclcpp::Parameter> poseParams;
    this->get_parameters("fake_detection_poses", poseParams);
    std::vector<geometry_msgs::msg::Pose> objectPoses;
    // Get the pose values from the parameter, at the current element.
    auto iterator = poseParams.begin();
    std::advance(iterator, fakeDetectorIndexCounter);
    auto poseValues = iterator->second.as_double_array();
    // Create a pose from the values
    geometry_msgs::msg::Pose pose;
    pose.position.x = poseValues.at(0);
    pose.position.y = poseValues.at(1);
    pose.position.z = poseValues.at(2);
    pose.orientation.x = poseValues.at(3);
    pose.orientation.y = poseValues.at(4);
    pose.orientation.z = poseValues.at(5);
    pose.orientation.w = poseValues.at(6);
    objectPoses.push_back(pose);
    // Increment the counter and reset it if it exceeds the size of the parameter
    ++fakeDetectorIndexCounter;
    if (fakeDetectorIndexCounter >= poseParams.size())
    {
        fakeDetectorIndexCounter = 0;
    }

    // Return the result
    auto result = std::make_shared<custom_msgs::action::Picture::Result>();
    for (auto aPose : objectPoses)
    {
        // create a new located object and add it to the result
        custom_msgs::msg::LocatedObject aObject;
        aObject.pose = aPose;
        aObject.width = 0.018;
        aObject.length = 0.018;
        aObject.height = 0.018;
        result->detected_blocks.push_back(aObject);
    }
    std::this_thread::sleep_for(std::chrono::seconds(1)); // Simulate the time it takes to detect objects
    RCLCPP_INFO(this->get_logger(), "objects detected");
    RCLCPP_INFO(this->get_logger(), "publishing results to rviz2");
    publishMarkers(objectPoses);
    publishPoses(objectPoses);
    RCLCPP_INFO(this->get_logger(), "publishing results to action");
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "done");
}

void ObjectDetectNode::executePictureAction(
    // NOLINTNEXTLINE (performance-unnecessary-value-param) // can't change definition in ros sourcecode
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<custom_msgs::action::Picture>> goal_handle)
{

    #ifdef DELAYED
    std::this_thread::sleep_for(std::chrono::milliseconds(START_DELAY));
    #endif
    // retrieve the latest pointcloud
    RCLCPP_INFO(this->get_logger(), "waiting for pointcloud");
    const sensor_msgs::msg::PointCloud2::SharedPtr pointCloud = waitForPointCloud();
    RCLCPP_INFO(this->get_logger(), "pointcloud received");

    // get the transformation immediately.
    geometry_msgs::msg::TransformStamped transformation;
    try
    {
        transformation = tfBuffer.lookupTransform("world", pointCloud->header.frame_id, tf2::TimePoint());
    }
    catch (tf2::TransformException& ex)
    {
        RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
        auto result = std::make_shared<custom_msgs::action::Picture::Result>();
        goal_handle->abort(result);
    }

    #ifdef DELAYED
    std::this_thread::sleep_for(std::chrono::milliseconds(END_DELAY));
    #endif

    // send feedback that the picture has been taken.
    auto feedback = std::make_shared<custom_msgs::action::Picture::Feedback>();
    feedback->picture_taken = true;
    goal_handle->publish_feedback(feedback);

    // Convert the pointcloud to a matrix
    Matrix<float> originalMatrix = Conversion::pointcloudToMatrix<float>(pointCloud);
    Matrix<float> detectionMatrix(originalMatrix.getRows(), originalMatrix.getCols(), NAN); // matrix to apply filters to
    Matrix<float> debuggingMatrix(originalMatrix.getRows(), originalMatrix.getCols(), NAN); // matrix to store
                                                                                            // intermediate results for
                                                                                            // debugging purposes

    // Call the detectObjects function
    std::vector<geometry_msgs::msg::Pose> objectPoses = detectObjects(originalMatrix, detectionMatrix, debuggingMatrix);

    RCLCPP_INFO(this->get_logger(), "objects detected");

    // convert the matrices to pointclouds
    RCLCPP_INFO(this->get_logger(), "applying PC conversions to matrices");
    sensor_msgs::msg::PointCloud2 originalMatrixPc = Conversion::matrixToPointcloud(originalMatrix);
    sensor_msgs::msg::PointCloud2 debuggingMatrixPc = Conversion::matrixToPointcloud(debuggingMatrix);
    sensor_msgs::msg::PointCloud2 detectionMatrixPc = Conversion::matrixToPointcloud(detectionMatrix);

    // apply transformations.
    RCLCPP_INFO(this->get_logger(), "applying transformations to pointclouds and poses");
    transformPosesToWorld(objectPoses, transformation);
    transformPointcloudToWorld(originalMatrixPc, transformation);
    transformPointcloudToWorld(debuggingMatrixPc, transformation);
    transformPointcloudToWorld(detectionMatrixPc, transformation);

    // Publish markers, pointclouds and poses (for rviz2 visualization)
    RCLCPP_INFO(this->get_logger(), "publishing results to rviz2");
    publishMarkers(objectPoses);
    publishPoses(objectPoses);
    publishPointCloud(originalMatrixPc, originalMatrixPublisher);
    publishPointCloud(debuggingMatrixPc, debuggingMatrixPublisher);
    publishPointCloud(detectionMatrixPc, detectionMatrixPublisher);

    // Return the result
    auto result = std::make_shared<custom_msgs::action::Picture::Result>();
    for (auto pose : objectPoses)
    {
        // create a new located object and add it to the result
        custom_msgs::msg::LocatedObject aObject;
        aObject.pose = pose;
        aObject.width = 0.018;  // Size temporarily fixed for our scope
        aObject.length = 0.018; // Size temporarily fixed for our scope
        aObject.height = 0.018; // Size temporarily fixed for our scope
        result->detected_blocks.push_back(aObject);
    }
    RCLCPP_INFO(this->get_logger(), "publishing results to action");
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "done");
}

sensor_msgs::msg::PointCloud2::SharedPtr ObjectDetectNode::waitForPointCloud()
{
    // Create a promise and future to wait for the message
    std::promise<sensor_msgs::msg::PointCloud2::SharedPtr> promise;
    auto future = promise.get_future();

    // Create a temporary subscription, to receive the message only once
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription =
        this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/camera/depth/color/points",
            10,
            // NOLINTNEXTLINE (performance-unnecessary-value-param) // unfortunately a promise can't work with references
            [&promise](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { promise.set_value(msg); });

    // Wait for the message
    future.wait();
    subscription.reset(); // unsubscribe
    return future.get();
}

/* static */ void ObjectDetectNode::transformPointcloudToWorld(
    sensor_msgs::msg::PointCloud2& pointCloud,
    geometry_msgs::msg::TransformStamped& transformStamped)
{
    // create a new pointcloud to apply the transform to
    sensor_msgs::msg::PointCloud2 transformedPointCloud;
    // set header to the corrosponding frame and timestamp
    std_msgs::msg::Header header;
    header.stamp = transformStamped.header.stamp;
    header.frame_id = transformStamped.header.frame_id;
    pointCloud.header = header;
    // apply transform
    tf2::doTransform(pointCloud, transformedPointCloud, transformStamped);
    // overwrite the pointcloud with the transformed pointcloud
    pointCloud = transformedPointCloud;
}

/* static */ void ObjectDetectNode::transformPosesToWorld(
    std::vector<geometry_msgs::msg::Pose>& objectPoses,
    geometry_msgs::msg::TransformStamped& transformStamped)
{
    // create a new vector to store the transformed objectPoses
    std::vector<geometry_msgs::msg::Pose> transformedPoses;
    // set header to the corrosponding frame and timestamp
    std_msgs::msg::Header header;
    header.stamp = transformStamped.header.stamp;
    header.frame_id = transformStamped.header.frame_id;
    // apply transform
    for (auto& pose : objectPoses)
    {
        // create a new poseStamped to apply the transform to
        geometry_msgs::msg::PoseStamped poseStamped;
        poseStamped.pose = pose;
        poseStamped.header = header;
        // apply transform
        tf2::doTransform(poseStamped, poseStamped, transformStamped);
        transformedPoses.push_back(poseStamped.pose);
    }
    // overwrite the objectPoses with the transformed objectPoses
    objectPoses = transformedPoses;
}

/* static */ std::vector<geometry_msgs::msg::Pose> ObjectDetectNode::detectObjects(
    const Matrix<float>& originalMatrix,
    Matrix<float>& detectionMatrix,
    Matrix<float>& debuggingMatrix)
{
    (void)debuggingMatrix; // suppress unused parameter warning, it remains here since its handy for debugging purposes
    // Filter the matrix
    detectionMatrix = originalMatrix;
    MatrixFilters::nanFilter(detectionMatrix);
    debuggingMatrix = detectionMatrix; // Output NAN-filtered matrix without extracting surface as debugging matrix.
    MatrixFilters::sufaceExtractionFilter(detectionMatrix);

    // MatrixFilters::morphOpen(detectionMatrix, morphologyKernelSize, morphologyIterations);

    std::vector<geometry_msgs::msg::Pose> poses;
    MatrixSegmentFinder<float> matrixSegmentFinder(detectionMatrix);

    for (MatrixSegment<float>& matrixSegment : matrixSegmentFinder.getSegments())
    {
        matrixSegment.detectBlocks();
        std::vector<geometry_msgs::msg::Pose> offsetPoses = matrixSegment.getOffsetPoses();
        for (auto pose : offsetPoses)
        {
            poses.push_back(pose);
        }
    }
    return poses;
}

void ObjectDetectNode::publishMarkers(std::vector<geometry_msgs::msg::Pose>& poses)
{
    visualization_msgs::msg::MarkerArray markerArray;

    static std::vector<visualization_msgs::msg::Marker> existingMarkers;

    // Mark extra markers for deletion
    for (std::size_t i = poses.size(); i < existingMarkers.size(); ++i)
    {
        visualization_msgs::msg::Marker deleteMarker = existingMarkers.at(i);
        deleteMarker.action = visualization_msgs::msg::Marker::DELETE;
        markerArray.markers.push_back(deleteMarker);
    }

    // Resize the vector to match the size of poses
    existingMarkers.resize(poses.size());

    // Update existing markers and add new ones
    for (std::size_t i = 0; i < poses.size(); ++i)
    {
        // header
        std_msgs::msg::Header header;
        header.frame_id = "world";
        // marker
        visualization_msgs::msg::Marker marker;
        marker.header = header;
        marker.ns = "object_detection";
        marker.id = static_cast<int>(i);
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose = poses.at(i); // Use the full pose, including orientation
        marker.scale.x = 0.0189;
        marker.scale.y = 0.0189;
        marker.scale.z = 0.0189;
        marker.color.a = 0.8;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        markerArray.markers.push_back(marker);
        existingMarkers.at(i) = marker;
    }

    // Publish the marker array
    markerPublisher->publish(markerArray);
}

void ObjectDetectNode::publishPoses(std::vector<geometry_msgs::msg::Pose>& poses)
{
    // header
    std_msgs::msg::Header header;
    header.frame_id = "world";
    // poses
    geometry_msgs::msg::PoseArray poseArray;
    poseArray.header = header;
    poseArray.poses = poses;
    posePublisher->publish(poseArray);
}

// NOLINTNEXTLINE (readability-convert-member-functions-to-static) // It can be static but its not meant to be
void ObjectDetectNode::publishPointCloud(
    sensor_msgs::msg::PointCloud2& pointcloud,
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& publisher)
{
    pointcloud.header.frame_id = "world";
    publisher->publish(pointcloud);
}
