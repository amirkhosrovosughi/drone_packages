#include "feature_extract_classic.hpp"

static const cv::Scalar COLOR_FILTER_LOWER_LIMIT(0, 100, 20);
static const cv::Scalar COLOR_FILTER_UPPER_LIMIT(10, 255, 255);
static double AREA_LIMIT = 100.0;

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

/**
 * @brief Extracts coordinates of detected objects in the input image.
 * 
 * @param inputImage The input image for feature extraction.
 * @return Vector of cv::Point representing detected object coordinates.
 */
std::vector<std::vector<double>> FeatureExtractClassic::extract(const cv::Mat& inputImage)
{
  RCLCPP_INFO(rclcpp::get_logger("visual_feature_extraction"), "Extracting feature based on color filteration ...");
  
  // for test only, replace all inputImage with img in rest of this file
  // cv::Mat img = cv::imread("/home/avosughi/test1.jpg"); 

#ifdef DEBUG_FEATURE
    _processedImage = inputImage.clone();
#else
  // cv::imshow("Input Frame", inputImage);
#endif

    cv::Mat3b hsv;
    cv::cvtColor(inputImage, hsv, cv::COLOR_BGR2HSV); //AMIR, change for test

    // Extract circle coordinates
    cv::Mat1b colorMask;
    cv::inRange(hsv, COLOR_FILTER_LOWER_LIMIT, COLOR_FILTER_UPPER_LIMIT, colorMask);

#ifdef DEBUG_FEATURE
    // cv::imshow("Color Filtered Image", colorMask);
#endif

    std::vector<std::vector<cv::Point>> circleContours;
    cv::findContours(colorMask, circleContours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    RCLCPP_DEBUG(rclcpp::get_logger("visual_feature_extraction"), "Number of counter: %ld", circleContours.size());
    RCLCPP_DEBUG(rclcpp::get_logger("visual_feature_extraction"), "Width of colorMask: %d", colorMask.cols);
    RCLCPP_DEBUG(rclcpp::get_logger("visual_feature_extraction"), "Height of colorMask: %d", colorMask.rows);

    std::vector<cv::Point> detectedCoordinates = getContourCenter(circleContours);

#ifdef DEBUG_FEATURE
    for (const auto& c : circleContours)
    {
        double area = cv::contourArea(c);
          if (area > AREA_LIMIT)
          {
              // double perimeter = cv::arcLength(c, true);
              cv::Point2f center;
              float radius;
              cv::minEnclosingCircle(c, center, radius);

              RCLCPP_INFO(rclcpp::get_logger("visual_feature_extraction"), "Point: (%.2f, %.2f), area %f", center.x, center.y, area);

              cv::drawContours(_processedImage, std::vector<std::vector<cv::Point>>{c}, -1, cv::Scalar(226, 198, 85), 2);
              cv::circle(_processedImage, center, static_cast<int>(radius), cv::Scalar(255, 97, 97), 1);
              cv::circle(_processedImage, center, 0, cv::Scalar(255, 97, 97), 5);
        }
    }

    // visualize Results
    cv::imshow("Detected Frame", _processedImage);
#endif

    double colorMaskWidth = colorMask.cols;
    double colorMaskHeight = colorMask.rows;
    std::vector<std::vector<double>> relativeCoordinate;
    for (cv::Point point : detectedCoordinates)
    {
      std::vector<double> relativePoint;
      relativePoint.push_back( point.x / colorMaskWidth - 0.5);
      relativePoint.push_back( point.y / colorMaskHeight - 0.5);

      relativeCoordinate.push_back(relativePoint);
    }
    
    cv::waitKey(1);
    return relativeCoordinate;
}

/**
 * @brief Fills the center of the mask with a specified width.
 * 
 * @param mask The input mask.
 * @param width The width of the center to be filled.
 * @return The mask with the filled center.
 */
void FeatureExtractClassic::fillTheCenter(const cv::Mat& mask, int width, cv::Mat& filledMask)
{
  cv::rectangle(filledMask, cv::Point(width, width), cv::Point(mask.cols - width, mask.rows - width), cv::Scalar(255), -1);
}

/**
 * @brief Gets the center coordinates of the contours.
 * 
 * @param contours Vector of contours.
 * @return Vector of cv::Point representing contour center coordinates.
 */
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

