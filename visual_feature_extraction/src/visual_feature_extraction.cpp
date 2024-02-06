// visual_feature_extraction.cpp

#include "visual_feature_extraction.hpp"

VisualFeatureExtraction::VisualFeatureExtraction()
  : Node("image_subscriber")
{
  rclcpp::QoS qos(10);
  auto rmw_qos_profile = qos.get_rmw_qos_profile();

  _colorCameraSubscriber.subscribe(this, "camera", rmw_qos_profile);
  _depthCameraSubscriber.subscribe(this, "depth_camera", rmw_qos_profile);
  
  cameraSync = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::Image>>(_colorCameraSubscriber, _depthCameraSubscriber, 10);
  cameraSync->registerCallback(std::bind(&VisualFeatureExtraction::CameraSyncCallback, this, std::placeholders::_1, std::placeholders::_2));
  
  // create publisher
  _featureCoordinatePublisher = this->create_publisher<drone_msgs::msg::DetectedFeatureList>("/featureDetection/coordinate", 10);

  // Initialize CvBridge
  _cvPtr = std::make_shared<cv_bridge::CvImage>();

// Decide which FeatureExtract child to use based on CMAKE flags
#ifndef DEEP_DETECTION
  _featureExtractor = std::make_unique<FeatureExtractClassic>();
#else
  _featureExtractor = std::make_unique<FeatureExtractDeep>();  
#endif
}

 void VisualFeatureExtraction::CameraSyncCallback(
      const sensor_msgs::msg::Image::ConstSharedPtr& imageColor,
      const sensor_msgs::msg::Image::ConstSharedPtr& imageDepth) {
      // Display the message on the console
  RCLCPP_INFO(get_logger(), "Receiving video frame ");

  size_t depthWidth = imageDepth->width;
  size_t depthHeight = imageDepth->height;
  size_t depthStep = imageDepth->step;

  // Convert ROS Image message to OpenCV image
  _cvPtr = cv_bridge::toCvCopy(imageColor, sensor_msgs::image_encodings::BGR8);
  _currentFrame = _cvPtr->image;

  const float* image_data = reinterpret_cast<const float*>(imageDepth->data.data());

  // Object Detection (Replace with your own tool)
  // Placeholder: Replace this block with your feature detection logic
  std::vector<std::vector<double>> detectedCoordinates = _featureExtractor->extract(_currentFrame);

  auto featureList = drone_msgs::msg::DetectedFeatureList();

  for (const auto &point : detectedCoordinates) {
        drone_msgs::msg::DetectedFeature feature;
        feature.x = point[0];
        feature.y = point[1];

        RCLCPP_DEBUG(get_logger(), "feature relative coordinate is: (%f,%f)", feature.x , feature.y);

        u_int16_t depthPixelX = (point[0] + 0.5) * depthWidth;
        u_int16_t depthPixelY = (point[1] + 0.5) * depthHeight;
        RCLCPP_DEBUG(get_logger(), "feature depth coordinate is: (%d,%d)", depthPixelX, depthPixelY);

        size_t pixel_index = depthPixelY * depthStep + depthPixelX * (depthStep / depthWidth);
        double depthValue = image_data[pixel_index];

        feature.depth= depthValue;
        featureList.features.push_back(feature);
    }

    // Publish the feature
    _featureCoordinatePublisher->publish(std::move(featureList));
  }