#ifndef VISUAL_FEATURE_EXTRACTION_HPP
#define VISUAL_FEATURE_EXTRACTION_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

#include "feature_extract.hpp"
#include "feature_extract_classic.hpp"

class VisualFeatureExtraction : public rclcpp::Node {
public:
  VisualFeatureExtraction();

private:
  void listenerCallback(const sensor_msgs::msg::Image::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _subscription;
  cv_bridge::CvImagePtr _cvPtr;
  cv::Mat _currentFrame;
  std::unique_ptr<FeatureExtract> _featureExtractor;
};

#endif  // VISUAL_FEATURE_EXTRACTION_HPP
