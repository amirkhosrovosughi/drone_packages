// visual_feature_extraction.cpp

#include "visual_feature_extraction.hpp"

VisualFeatureExtraction::VisualFeatureExtraction()
  : Node("image_subscriber")
{
  // Create the subscriber
  _subscription = create_subscription<sensor_msgs::msg::Image>(
    "camera", 10, std::bind(&VisualFeatureExtraction::listenerCallback, this, std::placeholders::_1));
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

void VisualFeatureExtraction::listenerCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  // Display the message on the console
  RCLCPP_INFO(get_logger(), "Receiving video frame ");

  // Convert ROS Image message to OpenCV image
  _cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  _currentFrame = _cvPtr->image;

  // Object Detection (Replace with your own tool)
  // Placeholder: Replace this block with your feature detection logic
  std::vector<cv::Point> detectedCoordinates = _featureExtractor->extract(_currentFrame);

  auto featureList = drone_msgs::msg::DetectedFeatureList();

  for (const auto &point : detectedCoordinates) {
        drone_msgs::msg::DetectedFeature feature;
        feature.x = point.x;
        feature.y = point.y;
        feature.depth= -1;
        featureList.features.push_back(feature);
    }

    // Publish the feature
    _featureCoordinatePublisher->publish(std::move(featureList));
}
