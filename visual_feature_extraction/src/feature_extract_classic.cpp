#include "visual_feature_extraction/feature_extract_classic.hpp"

static const cv::Scalar COLOR_FILTER_LOWER_LIMIT(0, 100, 20);
static const cv::Scalar COLOR_FILTER_UPPER_LIMIT(10, 255, 255);
static constexpr double AREA_LIMIT = 100.0f;
static const float DEFAULT_CONFIDENCE = 1.0f; // hard-coded confidence
static const int DEFAULT_CLASS = 0; // CLASS 0 = pole (classic color-based detector)

static const cv::Scalar BOX_BOUND_COLOR(0, 255, 0);
static const cv::Scalar CENTER_COLOR(255, 97, 97);
static const cv::Scalar CONTOUR_COLOR(226, 198, 85);

/**
 * @brief Constructor for FeatureExtractClassic.
 */
FeatureExtractClassic::FeatureExtractClassic() {
  // Initialization logic for Classic feature extraction
  cv::namedWindow("Detected Frame", cv::WINDOW_NORMAL);
  cv::resizeWindow("Detected Frame", 1280, 720);
}

FeatureExtractClassic::~FeatureExtractClassic() {
  // Cleanup logic if needed
}

void FeatureExtractClassic::config(double threshold) {
  _threshold = threshold;
}

std::vector<Detection> FeatureExtractClassic::extract(const cv::Mat& inputImage)
{
  RCLCPP_DEBUG(
      rclcpp::get_logger("visual_feature_extraction"),
      "Extracting feature based on color filtering ...");

#ifdef DEBUG_FEATURE
  _processedImage = inputImage.clone();
#endif

  // Convert to HSV
  cv::Mat3b hsv;
  cv::cvtColor(inputImage, hsv, cv::COLOR_BGR2HSV);

  // Color thresholding
  cv::Mat1b colorMask;
  cv::inRange(hsv, COLOR_FILTER_LOWER_LIMIT, COLOR_FILTER_UPPER_LIMIT, colorMask);

  // Find contours
  std::vector<std::vector<cv::Point>> circleContours;
  cv::findContours(colorMask, circleContours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

  RCLCPP_DEBUG(
      rclcpp::get_logger("visual_feature_extraction"),
      "Number of contours: %ld",
      circleContours.size());

  std::vector<Detection> detections;

  for (const auto& contour : circleContours)
  {
    double area = cv::contourArea(contour);
    if (area < AREA_LIMIT)
      continue;

    // Bounding box in pixel coordinates
    cv::Rect bbox = cv::boundingRect(contour);


    std::stringstream ss;
    ss << "Rect(x:" << bbox.x << ", y:" << bbox.y << ", w:" << bbox.width << ", h:" << bbox.height << ")";
    std::string rect_str = ss.str();

    RCLCPP_INFO(
      rclcpp::get_logger("visual_feature_extraction"),
      "BoundingBox is: (%s)",
      ss.str().c_str()
    );

    Detection det;
    det.box = bbox;
    det.confidence = DEFAULT_CONFIDENCE;
    det.classId = DEFAULT_CLASS;
    detections.push_back(det);

#ifdef DEBUG_FEATURE
    cv::Point2f center;
    float radius;
    cv::minEnclosingCircle(contour, center, radius);

    RCLCPP_INFO(
        rclcpp::get_logger("visual_feature_extraction"),
        "Detection: center=(%.2f, %.2f), area=%.2f",
        center.x,
        center.y,
        area);

    cv::drawContours(
        _processedImage,
        std::vector<std::vector<cv::Point>>{contour},
        -1,
        CONTOUR_COLOR,
        2);

    cv::rectangle(
        _processedImage,
        bbox,
        BOX_BOUND_COLOR,
        2);

    cv::circle(
        _processedImage,
        center,
        static_cast<int>(radius),
        CENTER_COLOR,
        1);
#endif
  }

#ifdef DEBUG_FEATURE
  cv::imshow("Detected Frame", _processedImage);
#endif

  cv::waitKey(1);
  return detections;
}

void FeatureExtractClassic::fillCenter(const cv::Mat& mask, int width, cv::Mat& filledMask)
{
  cv::rectangle(filledMask, cv::Point(width, width), cv::Point(mask.cols - width, mask.rows - width), cv::Scalar(255), -1);
}

std::vector<cv::Point> FeatureExtractClassic::getContourCenter(const std::vector<std::vector<cv::Point>>& contours)
{
  std::vector<cv::Point> coordinates;
  for (const auto& contour : contours) {
    double area = cv::contourArea(contour);
    if (area > AREA_LIMIT) {
      cv::Moments moments = cv::moments(contour);
      if (moments.m00 != 0) {
        int cx = static_cast<int>(moments.m10 / moments.m00);
        int cy = static_cast<int>(moments.m01 / moments.m00);
        coordinates.emplace_back(cx, cy);
      }
    }
  }
  return coordinates;
}

