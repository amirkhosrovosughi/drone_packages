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

/**
 * @brief ROS2 node for visual feature extraction from synchronized camera streams.
 */
class VisualFeatureExtraction : public rclcpp::Node {
public:
  /**
   * @brief Constructor. Initializes subscribers, synchronizers, and publishers.
   */
  VisualFeatureExtraction();

private:
  /**
   * @brief Callback for synchronized RGB and depth images.
   * 
   * @param imageColor Color image message.
   * @param imageDepth Depth image message.
   */
  void cameraSyncCallback( const sensor_msgs::msg::Image::ConstSharedPtr& msg_1,
      const sensor_msgs::msg::Image::ConstSharedPtr& msg_2);

    rclcpp::Publisher<drone_msgs::msg::DetectedFeatureList>::SharedPtr _featureCoordinatePublisher; ///< Publisher for extracted features.

  message_filters::Subscriber<sensor_msgs::msg::Image> _colorCameraSubscriber; ///< Color image subscriber.
  message_filters::Subscriber<sensor_msgs::msg::Image> _depthCameraSubscriber; ///< Depth image subscriber.
  std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::Image>> _cameraSync; ///< Synchronizer for camera streams.

  cv_bridge::CvImagePtr _cvPtr; ///< Latest color frame (CvBridge).
  cv_bridge::CvImageConstPtr _cvDepthPtr; ///< Latest depth frame (CvBridge).
  cv::Mat _currentFrame; ///< Current RGB frame.
  cv::Mat _depthFrame; ///< Current depth frame.
  std::unique_ptr<FeatureExtract> _featureExtractor; ///< Active feature extractor implementation.
};

#endif  // VISUAL_FEATURE_EXTRACTION_HPP
