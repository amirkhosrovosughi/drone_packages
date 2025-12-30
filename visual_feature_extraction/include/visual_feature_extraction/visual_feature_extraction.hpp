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
#include "visual_feature_extraction/feature_extract_classic.hpp"
#include "visual_feature_extraction/feature_extract_deep.hpp"
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>

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

    rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr _featureBoundingBoxPublisher; ///< Publisher for detected bboxs

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
