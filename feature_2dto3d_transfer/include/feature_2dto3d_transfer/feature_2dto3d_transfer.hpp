#ifndef FEATURE_2DTO3D_TRANSFER_HPP 
#define FEATURE_2DTO3D_TRANSFER_HPP

#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include <geometry_msgs/msg/point.hpp>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include <drone_msgs/msg/detected_feature.hpp>
#include <drone_msgs/msg/detected_feature_list.hpp>
#include <drone_msgs/msg/point_list.hpp>
#include "common_utilities/transform_util.hpp"

class Feature2DTo3DTransfer : public rclcpp::Node {
  typedef Eigen::Matrix<double, 3, 4> Matrix3x4;
  typedef Eigen::Matrix<double, 4, 3> Matrix4x3;
  typedef Eigen::Matrix<double, 4, 4> Matrix4x4;
  typedef Eigen::Matrix<double, 3, 1> Vector3;
  typedef Eigen::Matrix<double, 4, 1> Vector4;

public:
  Feature2DTo3DTransfer();

private:
  void coordinate2DCallback(const drone_msgs::msg::DetectedFeatureList cooerdinate2DList);
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo cameraInfo);
  void updateTransform();

private:
  rclcpp::Subscription<drone_msgs::msg::DetectedFeatureList>::SharedPtr _featureCoordinateSubscriber;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr _cameraInfoSubscriber;

  rclcpp::Publisher<drone_msgs::msg::PointList>::SharedPtr _feature3DcoordinateCameraPublisher;
  rclcpp::Publisher<drone_msgs::msg::PointList>::SharedPtr _feature3DcoordinateBasePublisher;

  std::unique_ptr<tf2_ros::Buffer> _tfBuffer;
  std::shared_ptr<tf2_ros::TransformListener> _tflistener;
  rclcpp::TimerBase::SharedPtr _timer;

  Matrix4x3 _kInverse;
  Matrix4x4 _base2Camera;
  bool _cameraInfoLoaded = false;
  bool _cameraTransformLoaded = false;
};

#endif  // FEATURE_2DTO3D_TRANSFER_HPP 
