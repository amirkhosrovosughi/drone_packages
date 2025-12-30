// feature_2dto3d_transfer.cpp

#include "feature_2dto3d_transfer.hpp"
#ifdef STORE_DEBUG_DATA
#include "data_logging_utils/data_logger.hpp"
#endif
#include <cmath>

const std::string FROM_FRAME = "base_link";
const std::string TO_FRAME = "camera_frame";
const std::string POLE_CLASS = "0";
const float MINIMUM_CONFIDENCE = 0.7f;

Feature2DTo3DTransfer::Feature2DTo3DTransfer()
  : Node("feature_2dto3d_transfer")
{
  _featureBoundingBoxSubscriber = this->create_subscription<vision_msgs::msg::Detection3DArray>(
              "/featureDetection/bbox", rclcpp::SensorDataQoS(), std::bind(&Feature2DTo3DTransfer::detectionCallback, this, std::placeholders::_1));
  _cameraInfoSubscriber = this->create_subscription<sensor_msgs::msg::CameraInfo>(
              "/camera_info", 10, std::bind(&Feature2DTo3DTransfer::cameraInfoCallback, this, std::placeholders::_1));

  // need to subscribe to tf camera relative position from drone base
  _tfBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  _tfListener = std::make_shared<tf2_ros::TransformListener>(*_tfBuffer);

  
  // create publisher
  _feature3DCoordinateCameraPublisher = this->create_publisher<drone_msgs::msg::PointList>("/feature/coordinate/cameraFrame", 10);
  _feature3DCoordinateBasePublisher = this->create_publisher<drone_msgs::msg::PointList>("/feature/coordinate/baseLink", 10);

  //TODO: later change it in way that stops listening when it gets the valiue
  _timer = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&Feature2DTo3DTransfer::updateTransform, this));
}

void Feature2DTo3DTransfer::detectionCallback(const vision_msgs::msg::Detection3DArray bboxArray)
{
  #ifdef STORE_DEBUG_DATA
  int plotCounter = 0;
  #endif
  if (_cameraInfoLoaded)
  {
    if (bboxArray.detections.size() != 0)
    {
      RCLCPP_DEBUG(this->get_logger(), "get %ld feature 2D coordinate(s)", bboxArray.detections.size());
    }
    rclcpp::Time sampleTime = rclcpp::Clock().now();

    drone_msgs::msg::PointList pointListCamera;
    drone_msgs::msg::PointList pointListBase;
    for (vision_msgs::msg::Detection3D detection : bboxArray.detections)
    {
      // if class != 0
      for (const auto& hypothesis : detection.results) {
        RCLCPP_DEBUG(this->get_logger(), "Object Class ID: '%s', Score: %f", 
                    hypothesis.hypothesis.class_id.c_str(), hypothesis.hypothesis.score);
        if (hypothesis.hypothesis.class_id != POLE_CLASS &&
            hypothesis.hypothesis.score > MINIMUM_CONFIDENCE)
        {
          RCLCPP_WARN(this->get_logger(), "None pole object, to not proceed"); 
          // there is no other object at the moment, mostly place holder for future
          continue;
        }
      }

      #ifdef STORE_DEBUG_DATA
      std::map<std::string, double> mapLog;  // add plotting
      #endif
      RCLCPP_DEBUG(this->get_logger(), "bounding box center: (x,y) = (%f,%f)",
              detection.bbox.center.position.x , detection.bbox.center.position.y);
      float estimatedDepth = 0.0;

      float pixelX = detection.bbox.center.position.x * _frameWidth;
      float pixelY = detection.bbox.center.position.y * _frameHeight;
      float depth = detection.bbox.center.position.z;  

      #ifdef STORE_DEBUG_DATA
      mapLog["pixelX_" + std::to_string(plotCounter)] = pixelX;
      mapLog["pixelY_" + std::to_string(plotCounter)] = pixelY;
      #endif

      float normalizedX = (pixelX - _frameCx) / _frameFx;
      float normalizedY = (pixelY - _frameCy) / _frameFy;

      Vector3 pixelDirection{normalizedX, normalizedY, 1};
      RCLCPP_DEBUG(this->get_logger(), " pixelDirection (%f,%f,%f)", pixelDirection[0], pixelDirection[1], pixelDirection[2]);

      Vector3 unitVector = pixelDirection; //.normalized(); no need to normalization here as depth camera return depth in z axis direction not  distance
      RCLCPP_DEBUG(this->get_logger(), " unitVector (%f,%f,%f)", unitVector[0], unitVector[1], unitVector[2]);

      RCLCPP_DEBUG(this->get_logger(), "stereo camera depth is %f", depth);
      if (depth <= 0.0 || depth > 100.0)
      {
        RCLCPP_INFO(this->get_logger(), "Skipped feature since depth value is not available");
        continue; //skip if depth data is not available
      }
      else
      {
        estimatedDepth = depth;
        RCLCPP_DEBUG(this->get_logger(), "using camera depth %f", estimatedDepth);
      }

      #ifdef STORE_DEBUG_DATA
      mapLog["depth_" + std::to_string(plotCounter)] = depth;
      #endif
    
      Vector3 camera2Feature = unitVector * estimatedDepth;

      geometry_msgs::msg::Point point;
      point.x = camera2Feature[0];
      point.y = camera2Feature[1];
      point.z = camera2Feature[2];
      pointListCamera.points.push_back(point);

      #ifdef STORE_DEBUG_DATA
      mapLog["camera2Feature[0]_" + std::to_string(plotCounter)] = camera2Feature[0];
      mapLog["camera2Feature[1]_" + std::to_string(plotCounter)] = camera2Feature[1];
      mapLog["camera2Feature[2]_" + std::to_string(plotCounter)] = camera2Feature[2];
      #endif

      if (_cameraTransformLoaded)
      {
        Vector4 cameraFeatureHomogeneous = camera2Feature.homogeneous();
        RCLCPP_DEBUG(this->get_logger(), " cameraFeatureHomogeneous(%f,%f,%f)", cameraFeatureHomogeneous[0], cameraFeatureHomogeneous[1], cameraFeatureHomogeneous[2]);

        Vector4 base2feature = _base2Camera*cameraFeatureHomogeneous;
        RCLCPP_DEBUG(this->get_logger(), " base2feature in NED (%f,%f,%f)", base2feature[0], base2feature[1], base2feature[2]);

        #ifdef STORE_DEBUG_DATA
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

      #ifdef STORE_DEBUG_DATA
      data_logging_utils::DataLogger::log(mapLog);
      plotCounter++;
      #endif
    }

    _feature3DCoordinateCameraPublisher->publish(pointListCamera);

    if (_cameraTransformLoaded)
    {
      _feature3DCoordinateBasePublisher->publish(pointListBase);
    }
  }
}

void Feature2DTo3DTransfer::cameraInfoCallback(const sensor_msgs::msg::CameraInfo cameraInfo)
{
  if (!_cameraInfoLoaded)
  {
    if (cameraInfo.header.frame_id.find("IMX214") == std::string::npos)
    {
      RCLCPP_INFO(this->get_logger(), "Skip Stereo camera frame info");
      return;
    }

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
                                                    0, -1, 0;
      _base2Camera =  _base2Camera * body_to_optical_transform;
      
      _cameraTransformLoaded = true;
      RCLCPP_INFO(this->get_logger(), "Camera relative coordinate is loaded successfully");
  
    } catch (tf2::TransformException & ex) {
      RCLCPP_INFO(this->get_logger(), "Could not find transform %s to %s: %s", TO_FRAME.c_str(), FROM_FRAME.c_str(), ex.what());
      return;
    }
  }
}
