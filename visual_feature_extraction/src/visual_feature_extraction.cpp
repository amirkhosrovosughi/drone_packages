// visual_feature_extraction.cpp

#include "visual_feature_extraction/visual_feature_extraction.hpp"

static const std::string CAMERA_FRAME = "camera_frame";

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
  _featureBoundingBoxPublisher = this->create_publisher<vision_msgs::msg::Detection3DArray>("/featureDetection/bbox", rclcpp::SensorDataQoS());

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
  rclcpp::Time now = this->get_clock()->now();

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
  std::vector<Detection> detections = _featureExtractor->extract(_currentFrame);

  RCLCPP_DEBUG(get_logger(), "Number detection poles %ld", detections.size());

  auto array_bbox = vision_msgs::msg::Detection3DArray();
  array_bbox.header.stamp = now;
  array_bbox.header.frame_id = CAMERA_FRAME;

  for (const auto &d : detections) {
    vision_msgs::msg::Detection3D msg;
    vision_msgs::msg::BoundingBox3D bbox;
    vision_msgs::msg::ObjectHypothesisWithPose hyp;       

    std::stringstream ss;
    ss << "Rect(x:" << d.box.x << ", y:" << d.box.y << ", w:" << d.box.width << ", h:" << d.box.height << ")";
    std::string rect_str = ss.str();
    RCLCPP_DEBUG(get_logger(), "Original bbox is: %s (in pixels)", rect_str);
  
    float cx_rgb = d.box.x + 0.5f * d.box.width;
    float cy_rgb = d.box.y + 0.5f * d.box.height;

    float scale_x = static_cast<float>(_depthFrame.cols) / _currentFrame.cols;
    float scale_y = static_cast<float>(_depthFrame.rows) / _currentFrame.rows;

    int px = static_cast<int>(cx_rgb * scale_x);
    int py = static_cast<int>(cy_rgb * scale_y);

    if (px < 0 || px >= _depthFrame.cols ||
        py < 0 || py >= _depthFrame.rows)
    {
        RCLCPP_WARN(get_logger(),
            "Depth index out of bounds: (%d,%d)", px, py);
        continue;
    }

    // 4. Safe depth access
    float depth = _depthFrame.at<float>(py, px);

    // publish bb
    float img_w = static_cast<float>(_currentFrame.cols);
    float img_h = static_cast<float>(_currentFrame.rows);

    // center (normalized)
    float cx = (d.box.x + 0.5f * d.box.width)  / img_w;
    float cy = (d.box.y + 0.5f * d.box.height) / img_h;

    // size (normalized)
    float width  = d.box.width  / img_w;
    float height  = d.box.height / img_h;

    // Clamp for safety
    cx = std::clamp(cx, 0.0f, 1.0f);
    cy = std::clamp(cy, 0.0f, 1.0f);
    width  = std::clamp(width,  0.0f, 1.0f);
    height  = std::clamp(height,  0.0f, 1.0f);

    RCLCPP_DEBUG(
        get_logger(),
        "normalized bbox: class=%d cx=%.6f cy=%.6f w=%.6f h=%.6f",
        d.classId, cx, cy, width, height
    );

    RCLCPP_DEBUG(get_logger(), "depth is: (%f)",  depth);

    bbox.center.position.x = cx;
    bbox.center.position.y = cy;
    bbox.center.position.z = depth;

    bbox.size.x = width;
    bbox.size.y = height;

    msg.bbox = bbox;

    hyp.hypothesis.class_id  = std::to_string(d.classId);
    hyp.hypothesis.score = d.confidence;
    msg.results.push_back(hyp);

    array_bbox.detections.push_back(msg);
  }

  // Publish the bboxs
  _featureBoundingBoxPublisher->publish(std::move(array_bbox));

}
