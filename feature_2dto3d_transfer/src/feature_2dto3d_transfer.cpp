// feature_2dto3d_transfer.cpp

#include "feature_2dto3d_transfer.hpp"
#include <cmath>

const std::string FROM_FRAME = "base_link";
const std::string TO_FRAME = "camera_frame";
const rclcpp::Duration maxTimeDifferent = rclcpp::Duration(0,50000000);

Feature2DTo3DTransfer::Feature2DTo3DTransfer()
  : Node("feature_2dto3d_transfer")
{
  _featureCoordinateSubscriber = this->create_subscription<drone_msgs::msg::DetectedFeatureList>(
              "/featureDetection/coordinate", 10, std::bind(&Feature2DTo3DTransfer::coordinate2DCallback, this, std::placeholders::_1));
  _cameraInfoSubscriber = this->create_subscription<sensor_msgs::msg::CameraInfo>(
              "/camera/camera_info", 10, std::bind(&Feature2DTo3DTransfer::cameraInfoCallback, this, std::placeholders::_1));

  auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().transient_local();
  _droneOdometrySubscriber = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
              "/fmu/out/vehicle_odometry", qos_profile, std::bind(&Feature2DTo3DTransfer::droneOdometryCallback, this, std::placeholders::_1));
  // need to subscribe to tf camera relative position from drone base
  _tfBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  _tflistener = std::make_shared<tf2_ros::TransformListener>(*_tfBuffer);

  
  // create publisher
  _feature3DcoordinateCameraPublisher = this->create_publisher<drone_msgs::msg::PointList>("/feature/coordinate/cameraFrame", 10);
  _feature3DcoordinateBasePublisher = this->create_publisher<drone_msgs::msg::PointList>("/feature/coordinate/baseLink", 10);

  //TODO: later change it in way that stops listening when it gets the valiue
  _timer = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&Feature2DTo3DTransfer::updateTransform, this));
}

void Feature2DTo3DTransfer::coordinate2DCallback(const drone_msgs::msg::DetectedFeatureList cooerdinate2DList)
{
  if (_cameraInfoLoaded)
  {
    rclcpp::Time sampleTime = rclcpp::Clock().now();

    drone_msgs::msg::PointList pointListCamera;
    drone_msgs::msg::PointList pointListBase;
    for (drone_msgs::msg::DetectedFeature cooerdinate2D : cooerdinate2DList.features)
    {
      float estimatedDepth = 0.0;

      Vector3 v2{cooerdinate2D.x, cooerdinate2D.y, 1};
      Vector4 v3 =  _kInverse*v2;
      Vector3 unitV = v3.head<3>().normalized();

      RCLCPP_DEBUG(this->get_logger(), "depth is %f", cooerdinate2D.depth);
      if (cooerdinate2D.depth <= 0.0 || cooerdinate2D.depth > 100.0)
      {
        
        // camera depth info is not valid, so try to use drone heigh and attitude to estimate depth, this comes 
        // with assumption that ground surface is flat
        // TODO: later explore other sort of depth sensor

        // check time stamp load to see if it is recent enough
        rclcpp::Duration timeLag = sampleTime - _odomTimeStamp;
        if (timeLag < maxTimeDifferent && _cameraTransformLoaded)
        { 
          Matrix4x4 cameraPose = _droneOdom * _base2Camera;

          Eigen::Matrix3f rotationMatrix = cameraPose.block<3, 3>(0, 0).cast<float>();
          Eigen::Vector3f euler_angles = rotationMatrix.eulerAngles(0, 1, 2); // ZYX order
          float cameraPitch = euler_angles[1];

          RCLCPP_DEBUG(this->get_logger(), "drone  height is: %f", _droneHeight);
          RCLCPP_DEBUG(this->get_logger(), "camera pitch is: %f", cameraPitch);
          if (cameraPitch >= 0)
          {
            continue;
          }
          estimatedDepth = _droneHeight / std::sin(-cameraPitch);
          RCLCPP_DEBUG(this->get_logger(), "estimatedDepth is: %f", estimatedDepth);

        }
        else
        {
        RCLCPP_INFO(this->get_logger(), "Skipped feature since depth value is not available");
        continue;
        }
      }
      else
      {
        estimatedDepth = cooerdinate2D.depth;
      }
    
      Vector3 camera2featrue = unitV * estimatedDepth;

      geometry_msgs::msg::Point point;
      point.x = camera2featrue[0];
      point.y = camera2featrue[1];
      point.z = camera2featrue[2];
      pointListCamera.points.push_back(point);

      if (_cameraTransformLoaded)
      {
        Vector4 camera2featrue4 = camera2featrue.homogeneous();
        Vector4 base2feature = _base2Camera*camera2featrue4;
        geometry_msgs::msg::Point pointB;
        pointB.x = base2feature[0];
        pointB.y = base2feature[1];
        pointB.z = base2feature[2];
        pointListBase.points.push_back(pointB);
      }
    }

    _feature3DcoordinateCameraPublisher->publish(pointListCamera);

    if (_cameraTransformLoaded)
    {
      _feature3DcoordinateBasePublisher->publish(pointListCamera);
    }
  }
}

void Feature2DTo3DTransfer::cameraInfoCallback(const sensor_msgs::msg::CameraInfo cameraInfo)
{
  if (!_cameraInfoLoaded)
  {
    Matrix3x4 K;
    K << cameraInfo.k[0], cameraInfo.k[1], cameraInfo.k[2], cameraInfo.k[3],
                cameraInfo.k[4], cameraInfo.k[5], cameraInfo.k[6], cameraInfo.k[7],
                cameraInfo.k[8], cameraInfo.k[9], cameraInfo.k[10], cameraInfo.k[11];

    _kInverse = K.transpose() * (K * K.transpose()).inverse();
    _cameraInfoLoaded = true;
    RCLCPP_INFO(this->get_logger(), "Camera information is loaded successfully");
  }
}

void Feature2DTo3DTransfer::updateTransform()
{
  if (!_cameraTransformLoaded)
  {
    geometry_msgs::msg::TransformStamped transformStamped;
    try
    {
      geometry_msgs::msg::TransformStamped transform = _tfBuffer->lookupTransform(TO_FRAME, FROM_FRAME, tf2::TimePointZero);
      // Extract translation and rotation from the transform message
      Eigen::Vector3d translation(transform.transform.translation.x,
                                  transform.transform.translation.y,
                                  transform.transform.translation.z);
      Eigen::Quaterniond rotation(transform.transform.rotation.w,
                                  transform.transform.rotation.x,
                                  transform.transform.rotation.y,
                                  transform.transform.rotation.z);

      // Construct the transformation matrix
      Eigen::Matrix3d rotation_matrix = rotation.toRotationMatrix();
      _base2Camera.block<3, 3>(0, 0) = rotation_matrix;
      _base2Camera.block<3, 1>(0, 3) = translation;

      _cameraTransformLoaded = true;
      RCLCPP_INFO(this->get_logger(), "Camera relative coordinate in loaded successfully");
  
    } catch (tf2::TransformException & ex) {
      RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s", TO_FRAME.c_str(), FROM_FRAME.c_str(), ex.what());
      return;
    }
  }
}

void Feature2DTo3DTransfer::droneOdometryCallback(const px4_msgs::msg::VehicleOdometry odometry)
{
  _odomTimeStamp = rclcpp::Clock().now();
  Eigen::Vector3d translation(odometry.position[0],
                              odometry.position[1],
                              odometry.position[2]);
  Eigen::Quaterniond rotation(odometry.q[0],
                              odometry.q[1],
                              odometry.q[2],
                              odometry.q[3]);
  Eigen::Matrix3d rotation_matrix = rotation.toRotationMatrix();
  _droneOdom.block<3, 3>(0, 0) = rotation_matrix;
  _droneOdom.block<3, 1>(0, 3) = translation;
  _droneHeight = translation[2];
}