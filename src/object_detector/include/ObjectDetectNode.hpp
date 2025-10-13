#ifndef ObjectDetectNode_HPP
#define ObjectDetectNode_HPP

#include <memory>                                  // std::shared_ptr
#include <rclcpp/rclcpp.hpp>                       // rclcpp::Node
#include <rclcpp_action/rclcpp_action.hpp>         // rclcpp_action::Server
#include <sensor_msgs/msg/point_cloud2.hpp>        // sensor_msgs::msg::PointCloud2
#include <geometry_msgs/msg/pose.hpp>              // geometry_msgs::msg::Pose
#include <geometry_msgs/msg/pose_array.hpp>        // geometry_msgs::msg::PoseArray
#include <geometry_msgs/msg/transform_stamped.hpp> // geometry_msgs::msg::TransformStamped
#include <visualization_msgs/msg/marker_array.hpp> // visualization_msgs::msg::MarkerArray
#include <tf2_ros/buffer.h>                        // tf2_ros::Buffer
#include <tf2_ros/transform_listener.h>            // tf2_ros::TransformListener
#include <tf2_ros/static_transform_broadcaster.h>  // tf2_ros::StaticTransformBroadcaster

#include <custom_msgs/action/picture.hpp> // custom_msgs::action::Picture
#include "Matrix.hpp"

/**
 * @class ObjectDetectNode
 * @brief The ObjectDetectNode class provides a node for object detection in a pointcloud.
 */
class ObjectDetectNode : public rclcpp::Node
{
public:
    virtual ~ObjectDetectNode();
    ObjectDetectNode(ObjectDetectNode const&) = delete;            /* the node should never be copied */
    ObjectDetectNode& operator=(ObjectDetectNode const&) = delete; /* the node should never be copied */
    /**
     * @brief Get the instance of the ObjectDetectNode.
     *
     * @return std::shared_ptr<ObjectDetectNode> The instance of the ObjectDetectNode.
     */
    static std::shared_ptr<ObjectDetectNode> getInstance();
private:
    /**
     * @brief Constructs a new ObjectDetectNode object.
     */
    ObjectDetectNode();
    /**
     * @brief Publish the camera transform.
     */
    void publishCameraTransform();
    /**
     * @brief Handle the goal of the picture action.
     *
     * @param uuid the uuid of the goal
     * @param goal the goal
     */
    rclcpp_action::GoalResponse handlePictureActionGoal(
        const rclcpp_action::GoalUUID& uuid,
        const std::shared_ptr<const custom_msgs::action::Picture::Goal> goal);
    /**
     * @brief Handle the cancel of the picture action.
     *
     * @param goal_handle the goal handle
     */
    rclcpp_action::CancelResponse handlePictureActionCancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<custom_msgs::action::Picture>> goal_handle);
    /**
     * @brief Handle the accepted picture action.
     *
     * @param goal_handle the goal handle
     */
    void handlePictureActionAccepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<custom_msgs::action::Picture>> goal_handle);
    /**
     * @brief Execute the picture action.
     *
     * @param goal_handle the goal handle
     */
    void executePictureAction(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<custom_msgs::action::Picture>> goal_handle);
    /**
     * @brief Execute a (fake) picture action.
     *
     * @param goal_handle the goal handle
     */
    void executeFakePictureAction(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<custom_msgs::action::Picture>> goal_handle);
    /**
     * @brief Wait for a pointcloud to be received.
     *
     * @return sensor_msgs::msg::PointCloud2::SharedPtr The received pointcloud.
     */
    sensor_msgs::msg::PointCloud2::SharedPtr waitForPointCloud();
    /**
     * @brief Transform a pointcloud to the world frame.
     *
     * @param pointcloud the pointcloud to transform
     * @param transformStamped the transform to apply
     */
    static void transformPointcloudToWorld(
        sensor_msgs::msg::PointCloud2& pointcloud,
        geometry_msgs::msg::TransformStamped& transformStamped);
    /**
     * @brief Transform a vector of poses to the world frame.
     *
     * @param objectPoses the poses to transform
     * @param transformStamped the transform to apply
     */
    static void transformPosesToWorld(
        std::vector<geometry_msgs::msg::Pose>& objectPoses,
        geometry_msgs::msg::TransformStamped& transformStamped);
    /**
     * @brief Publish markers at the for object visualization in rviz2.
     *
     * @param poses the poses of the detected objects
     */
    void publishMarkers(std::vector<geometry_msgs::msg::Pose>& poses);
    /**
     * @brief Publish poses for object visualization in rviz2.
     *
     * @param poses the poses of the detected objects
     */
    void publishPoses(std::vector<geometry_msgs::msg::Pose>& poses);
    /**
     * @brief Publish a pointcloud.
     *
     * @param pointcloud the pointcloud to publish
     * @param publisher the publisher to publish the pointcloud
     */
    void publishPointCloud(
        sensor_msgs::msg::PointCloud2& pointcloud,
        const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& publisher);
    /**
     * @brief Detect objects a matrix.
     *
     * @param originalMatrix the original uneditable matrix
     * @param detectionMatrix a matrix to apply filters. Also visualized in rviz2 afterwards
     * @param debuggingMatrix a matrix to store results for debugging purposes. Also visualized in rviz2 afterwards
     * @return std::vector<geometry_msgs::msg::Pose> the poses of the detected objects
     */
    static std::vector<geometry_msgs::msg::Pose> detectObjects(
        const Matrix<float>& originalMatrix,
        Matrix<float>& detectionMatrix,
        Matrix<float>& debuggingMatrix);

    rclcpp_action::Server<custom_msgs::action::Picture>::SharedPtr pictureServer;         /* The action server to request a
                                                                                             picture. */
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr originalMatrixPublisher;  /* The publisher for the original
                                                                                             matrix. */
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debuggingMatrixPublisher; /* The publisher for the debugging
                                                                                             matrix. */
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr detectionMatrixPublisher; /* The publisher for the detection
                                                                                             matrix. */
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markerPublisher;   /* The publisher for the
                                                                                             markers. */
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr posePublisher;            /* The publisher for the poses. */

    std::vector<visualization_msgs::msg::Marker> existingMarkers; /* rviz2 cubes that are currently visualized. */

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> transformBroadcaster; /* The broadcaster for the camera transforms.
                                                                                */

    std::shared_ptr<rclcpp::ParameterEventHandler> paramSubscriber; /* The subscriber for the parameter changes. */

    std::shared_ptr<rclcpp::ParameterCallbackHandle> callbackHandleX;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> callbackHandleY;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> callbackHandleZ;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> callbackHandleRoll;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> callbackHandlePitch;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> callbackHandleYaw;

    tf2_ros::Buffer tfBuffer;              /* The buffer for the transforms. */
    tf2_ros::TransformListener tfListener; /* The listener for the transforms. */

    // Fake detector variables
    bool useFakeDetector;                 /* Whether to use the fake detector. */
    std::size_t fakeDetectorIndexCounter; /* The counter for the fake detector. */
};

#endif // ObjectDetectNode_HPP
