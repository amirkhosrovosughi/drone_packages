// feature_2dto3d_transfer.cpp

#include "feature_2dto3d_transfer.hpp"

const std::string FROM_FRAME = "base_link";
const std::string TO_FRAME = "camera_frame";

Feature2DTo3DTransfer::Feature2DTo3DTransfer()
  : Node("feature_2dto3d_transfer")
{
  _featureCoordinateSubscriber = this->create_subscription<drone_msgs::msg::DetectedFeatureList>(
              "/featureDetection/coordinate", 10, std::bind(&Feature2DTo3DTransfer::coordinate2DCallback, this, std::placeholders::_1));
  _cameraInfoSubscriber = this->create_subscription<sensor_msgs::msg::CameraInfo>(
              "/camera/camera_info", 10, std::bind(&Feature2DTo3DTransfer::cameraInfoCallback, this, std::placeholders::_1));
  // need to subscribe to tf camera relative position from camera base
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
    drone_msgs::msg::PointList pointListCamera;
    drone_msgs::msg::PointList pointListBase;
    for (drone_msgs::msg::DetectedFeature cooerdinate2D : cooerdinate2DList.features)
    {
      RCLCPP_INFO(this->get_logger(), "depth is %f", cooerdinate2D.depth);
      if (cooerdinate2D.depth <= 0.0 || cooerdinate2D.depth > 100.0)
      {
        RCLCPP_INFO(this->get_logger(), "Skipped feature since value is not value");
        continue;
        // TODO: use attitude and height info to estimate the depth instead
      }
    

      Vector3 v2{cooerdinate2D.x, cooerdinate2D.y, 1};
      Vector4 v3 =  _kInverse*v2;

      Vector3 unitV = v3.head<3>().normalized();
      Vector3 camera2featrue = unitV*cooerdinate2D.depth;

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