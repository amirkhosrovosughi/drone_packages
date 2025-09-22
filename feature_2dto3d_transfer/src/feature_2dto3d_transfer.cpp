// feature_2dto3d_transfer.cpp

#include "feature_2dto3d_transfer.hpp"
#ifdef SOTRE_DEBUG_DATA
#include "data_logging_utils/data_logger.hpp"
#endif
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
  #ifdef SOTRE_DEBUG_DATA
  int plotCounter = 0;
  #endif
  if (_cameraInfoLoaded)
  {
    if (cooerdinate2DList.features.size() != 0)
    {
      RCLCPP_DEBUG(this->get_logger(), "get %d feature 2D coordinate(s)", cooerdinate2DList.features.size());
    }
    rclcpp::Time sampleTime = rclcpp::Clock().now();

    drone_msgs::msg::PointList pointListCamera;
    drone_msgs::msg::PointList pointListBase;
    for (drone_msgs::msg::DetectedFeature cooerdinate2D : cooerdinate2DList.features)
    {
      #ifdef SOTRE_DEBUG_DATA
      std::map<std::string, double> mapLog;  // add plotting
      #endif
      RCLCPP_DEBUG(this->get_logger(), "2D coordinate: (x,y) = (%f,%f)", cooerdinate2D.x , cooerdinate2D.y);
      float estimatedDepth = 0.0;

      float pixelX = (cooerdinate2D.x + 0.5) * _frameWidth;
      float pixelY = (cooerdinate2D.y + 0.5) * _frameHeight;

      #ifdef SOTRE_DEBUG_DATA
      mapLog["pixelX_" + std::to_string(plotCounter)] = pixelX;
      mapLog["pixelY_" + std::to_string(plotCounter)] = pixelY;
      #endif

      float normalizedX = (pixelX - _frameCx) / _frameFx;
      float normalizedY = (pixelY - _frameCy) / _frameFy;

      Vector3 pixelDirection{normalizedX, normalizedY, 1};
      RCLCPP_DEBUG(this->get_logger(), " pixelDirection (%f,%f,%f)", pixelDirection[0], pixelDirection[1], pixelDirection[2]);

      Vector3 unitVector = pixelDirection; //.normalized(); no need to normalization here as depth camera return depth in z axis direction not  distance
      RCLCPP_DEBUG(this->get_logger(), " unitVector (%f,%f,%f)", unitVector[0], unitVector[1], unitVector[2]);

      RCLCPP_DEBUG(this->get_logger(), "stereo camera depth is %f", cooerdinate2D.depth);
      if (cooerdinate2D.depth <= 0.0 || cooerdinate2D.depth > 100.0)
      {
        RCLCPP_INFO(this->get_logger(), "Skipped feature since depth value is not available");
        continue; //skip if depth data is not available
      }
      else
      {
        estimatedDepth = cooerdinate2D.depth;
        RCLCPP_DEBUG(this->get_logger(), "using camera depth %f", estimatedDepth);
      }

      #ifdef SOTRE_DEBUG_DATA
      mapLog["cooerdinate2D.depth_" + std::to_string(plotCounter)] = cooerdinate2D.depth;
      #endif
    
      Vector3 camera2featrue = unitVector * estimatedDepth;

      geometry_msgs::msg::Point point;
      point.x = camera2featrue[0];
      point.y = camera2featrue[1];
      point.z = camera2featrue[2];
      pointListCamera.points.push_back(point);

      #ifdef SOTRE_DEBUG_DATA
      mapLog["camera2featrue[0]_" + std::to_string(plotCounter)] = camera2featrue[0];
      mapLog["camera2featrue[1]_" + std::to_string(plotCounter)] = camera2featrue[1];
      mapLog["camera2featrue[2]_" + std::to_string(plotCounter)] = camera2featrue[2];
      #endif

      if (_cameraTransformLoaded)
      {
        Vector4 cameraFeatureHomogeneous = camera2featrue.homogeneous();
        RCLCPP_DEBUG(this->get_logger(), " cameraFeatureHomogeneous(%f,%f,%f)", cameraFeatureHomogeneous[0], cameraFeatureHomogeneous[1], cameraFeatureHomogeneous[2]);

        Vector4 base2feature = _base2Camera*cameraFeatureHomogeneous;
        RCLCPP_DEBUG(this->get_logger(), " base2feature in NED (%f,%f,%f)", base2feature[0], base2feature[1], base2feature[2]);

        #ifdef SOTRE_DEBUG_DATA
        mapLog["base2feature[0]_" + std::to_string(plotCounter)] = base2feature[0];
        mapLog["base2feature[1]_" + std::to_string(plotCounter)] = base2feature[1];
        mapLog["base2feature[2]_" + std::to_string(plotCounter)] = base2feature[2];
        #endif

        geometry_msgs::msg::Point pointB;
        pointB.x = base2feature[0];
        pointB.y = base2feature[1];
        pointB.z = base2feature[2];
        pointListBase.points.push_back(pointB);
      }

      #ifdef SOTRE_DEBUG_DATA
      data_logging_utils::DataLogger::log(mapLog);
      plotCounter++;
      #endif
    }

    _feature3DcoordinateCameraPublisher->publish(pointListCamera);

    if (_cameraTransformLoaded)
    {
      _feature3DcoordinateBasePublisher->publish(pointListBase);
    }
  }
}

void Feature2DTo3DTransfer::cameraInfoCallback(const sensor_msgs::msg::CameraInfo cameraInfo)
{
  if (!_cameraInfoLoaded)
  {
    _frameWidth = cameraInfo.width;
    _frameHeight = cameraInfo.height;
    _frameCx = cameraInfo.k[2];
    _frameCy = cameraInfo.k[5];
    _frameFx = cameraInfo.k[0];
    _frameFy = cameraInfo.k[4];

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
      geometry_msgs::msg::TransformStamped transform = _tfBuffer->lookupTransform(FROM_FRAME, TO_FRAME, tf2::TimePointZero);
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

      // Adjust to consider optic frame
      Eigen::Matrix4d body_to_optical_transform = Eigen::Matrix4d::Identity();
      body_to_optical_transform.block<3, 3>(0, 0) << 0, 0, 1,
                                                    -1, 0, 0,
                                                    0, -1, 0; // TODO: this is almost correct, just z is in wrong direction  
      _base2Camera =  _base2Camera * body_to_optical_transform;
      
      _cameraTransformLoaded = true;
      RCLCPP_INFO(this->get_logger(), "Camera relative coordinate in loaded successfully");
  
    } catch (tf2::TransformException & ex) {
      RCLCPP_INFO(this->get_logger(), "Could not find transform %s to %s: %s", TO_FRAME.c_str(), FROM_FRAME.c_str(), ex.what());
      return;
    }
  }
}
