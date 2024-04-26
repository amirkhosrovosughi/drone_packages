#ifndef VISUAL_FEATURE_EXTRACTION_HPP
#define VISUAL_FEATURE_EXTRACTION_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include "feature_extract.hpp"
#include "feature_extract_classic.hpp"
#include <drone_msgs/msg/detected_feature.hpp>
#include <drone_msgs/msg/detected_feature_list.hpp>

class VisualFeatureExtraction : public rclcpp::Node {
public:
  VisualFeatureExtraction();

private:

  void CameraSyncCallback( const sensor_msgs::msg::Image::ConstSharedPtr& msg_1,
      const sensor_msgs::msg::Image::ConstSharedPtr& msg_2);

  rclcpp::Publisher<drone_msgs::msg::DetectedFeatureList>::SharedPtr _featureCoordinatePublisher;


  message_filters::Subscriber<sensor_msgs::msg::Image> _colorCameraSubscriber;
  message_filters::Subscriber<sensor_msgs::msg::Image> _depthCameraSubscriber;

  std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::Image>> cameraSync;

  cv_bridge::CvImagePtr _cvPtr;
  cv_bridge::CvImageConstPtr _cvDepthPtr;
  cv::Mat _currentFrame;
  std::unique_ptr<FeatureExtract> _featureExtractor;
};

#endif  // VISUAL_FEATURE_EXTRACTION_HPP
