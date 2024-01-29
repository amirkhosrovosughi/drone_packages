#include "feature_extract_classic.hpp"

// static const cv::Scalar COLOR_FILTER_LOWER_LIMIT(60, 150, 100);
// static const cv::Scalar COLOR_FILTER_UPPER_LIMIT(90, 255, 255);
static const cv::Scalar COLOR_FILTER_LOWER_LIMIT(0, 243, 224);
static const cv::Scalar COLOR_FILTER_UPPER_LIMIT(20, 203, 184);
static double AREA_LIMIT = 20.0;

/**
 * @brief Constructor for FeatureExtractClassic.
 */
FeatureExtractClassic::FeatureExtractClassic() {
  // Initialization logic for Classic feature extraction
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
std::vector<cv::Point> FeatureExtractClassic::extract(const cv::Mat& inputImage)
{
#ifdef DEBUG_FEATURE
    _processedImage = inputImage.clone();
#endif

    cv::Mat hsv;
    cv::cvtColor(inputImage, hsv, cv::COLOR_BGR2HSV);

    // Extract circle coordinates
    cv::Mat circleMask;
    cv::inRange(hsv, COLOR_FILTER_LOWER_LIMIT, COLOR_FILTER_UPPER_LIMIT, circleMask);

    cv::Mat filledCircleMask = cv::Mat::zeros(circleMask.size(), circleMask.type());
    fillTheCenter(circleMask, 25, filledCircleMask);
    std::vector<std::vector<cv::Point>> circleContours;
    cv::findContours(filledCircleMask, circleContours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    std::vector<cv::Point> circleCoordinates = getContourCenter(circleContours);

    std::vector<cv::Point> detectedCoordinates;
    detectedCoordinates.insert(detectedCoordinates.end(), circleCoordinates.begin(), circleCoordinates.end());

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

                cv::drawContours(_processedImage, std::vector<std::vector<cv::Point>>{c}, -1, cv::Scalar(226, 198, 85), 2);
                cv::circle(_processedImage, center, static_cast<int>(radius), cv::Scalar(255, 97, 97), 1);
                cv::circle(_processedImage, center, 0, cv::Scalar(255, 97, 97), 5);
        }
    }
#endif


    // Show Results
    cv::imshow("Detected Frame", _processedImage); // replace later with edited image
    cv::waitKey(1);

    return circleCoordinates;
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

