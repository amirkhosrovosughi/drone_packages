#ifndef FEATURE_2DTO3D_TRANSFER_HPP 
#define FEATURE_2DTO3D_TRANSFER_HPP

#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include <geometry_msgs/msg/point.hpp>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include <drone_msgs/msg/point_list.hpp>
#include "common_utilities/transform_util.hpp"
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>

/**
 * @class Feature2DTo3DTransfer
 * @brief Node to convert 2D detected features into 3D coordinates using camera info and TF transforms.
 */
class Feature2DTo3DTransfer : public rclcpp::Node {
  typedef Eigen::Matrix<double, 3, 4> Matrix3x4;
  typedef Eigen::Matrix<double, 4, 3> Matrix4x3;
  typedef Eigen::Matrix<double, 4, 4> Matrix4x4;
  typedef Eigen::Matrix<double, 3, 1> Vector3;
  typedef Eigen::Matrix<double, 4, 1> Vector4;

public:
  /**
   * @brief Construct a new Feature2DTo3DTransfer node.
   */
  Feature2DTo3DTransfer();

private:
  /**
   * @brief Callback to process incoming 2D feature coordinates and convert them to 3D points.
   * @param coordinate2DList List of detected 2D features with depth info.
   */
  void detectionCallback(const vision_msgs::msg::Detection3DArray bboxArray);

  /**
   * @brief Callback to store camera intrinsic parameters from CameraInfo message.
   * @param cameraInfo Camera intrinsic info.
   */
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo cameraInfo);

  /**
   * @brief Update the transformation matrix between base and camera frames.
   */
  void updateTransform();

private:
  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr _featureBoundingBoxSubscriber; ///< Subscriber for detected 2D bounding boxes with depth
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr _cameraInfoSubscriber; ///< Subscriber for camera intrinsic information

  rclcpp::Publisher<drone_msgs::msg::PointList>::SharedPtr _feature3DCoordinateCameraPublisher; ///< Publisher for 3D features in camera frame  coordinate
  rclcpp::Publisher<drone_msgs::msg::PointList>::SharedPtr _feature3DCoordinateBasePublisher; ///< Publisher for 3D features in base frame coordinate
  rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr _featureBoundingBoxPublisher; ///< Publisher for bbox without valid depth

  std::unique_ptr<tf2_ros::Buffer> _tfBuffer; ///< TF2 buffer for transform lookup
  std::shared_ptr<tf2_ros::TransformListener> _tfListener; ///< TF2 listener
  rclcpp::TimerBase::SharedPtr _timer; ///< Timer for updating transforms

  float _frameWidth; ///< Camera frame width
  float _frameHeight; ///< Camera frame height
  float _frameCx; ///< Principal point cx
  float _frameCy; ///< Principal point cy
  float _frameFx; ///< Focal length fx
  float _frameFy; ///< Focal length fy
  Matrix4x4 _base2Camera; ///< Transform matrix from base to camera
  bool _cameraInfoLoaded = false; ///< Flag indicating if camera info has been received
  bool _cameraTransformLoaded = false; ///< Flag indicating if camera transform has been loaded
};

#endif  // FEATURE_2DTO3D_TRANSFER_HPP 
