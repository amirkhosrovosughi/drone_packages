// visual_feature_extraction.cpp

#include "visual_feature_extraction.hpp"

VisualFeatureExtraction::VisualFeatureExtraction()
  : Node("image_subscriber")
{
  rclcpp::QoS qos(10);
  auto rmw_qos_profile = qos.get_rmw_qos_profile();

  _colorCameraSubscriber.subscribe(this, "camera", rmw_qos_profile);
  _depthCameraSubscriber.subscribe(this, "depth_camera", rmw_qos_profile);
  
  _cameraSync = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::Image>>(_colorCameraSubscriber, _depthCameraSubscriber, 10);
  _cameraSync->registerCallback(std::bind(&VisualFeatureExtraction::cameraSyncCallback, this, std::placeholders::_1, std::placeholders::_2));
  
  // create publisher
  _featureCoordinatePublisher = this->create_publisher<drone_msgs::msg::DetectedFeatureList>("/featureDetection/coordinate", 10);

  // Initialize CvBridge
  _cvPtr = std::make_shared<cv_bridge::CvImage>();

// Decide which FeatureExtract child to use based on CMAKE flags
#ifndef DEEP_DETECTION
  _featureExtractor = std::make_unique<FeatureExtractClassic>();
  RCLCPP_INFO(get_logger(), "Using simple classic pole detector, using color contrast");
#else
  _featureExtractor = std::make_unique<FeatureExtractDeep>();
  RCLCPP_INFO(get_logger(), "Using deep pole detector, using YOLO8 model");
#endif
}

 void VisualFeatureExtraction::cameraSyncCallback(
      const sensor_msgs::msg::Image::ConstSharedPtr& imageColor,
      const sensor_msgs::msg::Image::ConstSharedPtr& imageDepth) {
      // Display the message on the console
  RCLCPP_DEBUG(get_logger(), "Receiving video frame ");

  size_t depthWidth = imageDepth->width;
  size_t depthHeight = imageDepth->height;

  // Convert ROS Image message to OpenCV image
  try
  {
    _cvPtr = cv_bridge::toCvCopy(imageColor, sensor_msgs::image_encodings::BGR8);
    _currentFrame = _cvPtr->image;

    _cvDepthPtr = cv_bridge::toCvShare(imageDepth, sensor_msgs::image_encodings::TYPE_32FC1);
    _depthFrame = _cvDepthPtr->image;
  }
  catch (cv_bridge::Exception& e)
  {
      RCLCPP_ERROR(get_logger(), "Error in cv conversion ");
  }

  // Object Detection (Replace with your own tool)
  // Placeholder: Replace this block with your feature detection logic
  std::vector<std::vector<double>> detectedCoordinates = _featureExtractor->extract(_currentFrame);

  RCLCPP_DEBUG(get_logger(), "Number detection poles %d", detectedCoordinates.size());

  auto featureList = drone_msgs::msg::DetectedFeatureList();

  for (const auto &point : detectedCoordinates) {
        drone_msgs::msg::DetectedFeature feature;
        feature.x = point[0];
        feature.y = point[1];

        RCLCPP_DEBUG(get_logger(), "feature relative coordinate is: (%f,%f)", feature.x , feature.y);

        u_int16_t depthPixelX = (point[0] + 0.5) * depthWidth;
        u_int16_t depthPixelY = (point[1] + 0.5) * depthHeight;
        RCLCPP_DEBUG(get_logger(), "feature depth coordinate is: (%d,%d)", depthPixelX, depthPixelY);

        float depthValue = _depthFrame.at<float>(depthPixelY, depthPixelX); 
        RCLCPP_DEBUG(get_logger(), "depth is: (%f)",  depthValue);

        feature.depth= depthValue;
        featureList.features.push_back(feature);
    }

    // Publish the feature
    rclcpp::Time now = this->get_clock()->now();
    featureList.header.stamp = now;
    _featureCoordinatePublisher->publish(std::move(featureList));
}