// visual_feature_extraction.cpp

#include "visual_feature_extraction.hpp"

VisualFeatureExtraction::VisualFeatureExtraction()
  : Node("image_subscriber")
{
  // Create the subscriber
  _subscription = create_subscription<sensor_msgs::msg::Image>(
    "camera", 10, std::bind(&VisualFeatureExtraction::listenerCallback, this, std::placeholders::_1));

  // Initialize CvBridge
  _cvPtr = std::make_shared<cv_bridge::CvImage>();
}

void VisualFeatureExtraction::listenerCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  // Display the message on the console
  RCLCPP_INFO(get_logger(), "Receiving video frame");

  // Convert ROS Image message to OpenCV image
  _cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  _currentFrame = _cvPtr->image;

  // Object Detection (Replace with your own tool)
  // Placeholder: Replace this block with your feature detection logic
  // Example: cv::Mat detectedImage = your_feature_detection_function(_currentFrame);
  cv::Mat detectedImage = _currentFrame;

  // Show Results
  cv::imshow("Detected Frame", detectedImage);
  cv::waitKey(1);
}
