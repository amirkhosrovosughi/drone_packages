#ifndef FEATURE_EXTRACT_HPP
#define FEATURE_EXTRACT_HPP

#include "opencv2/opencv.hpp"

/**
 * @brief Bounding Box of detection objects
 */
struct Detection
{
    cv::Rect box;
    float confidence;
    int classId;
};

/**
 * @brief Base abstract class for feature extraction.
 */
class FeatureExtract {
public:
  /**
   * @brief Virtual destructor.
   */
  virtual ~FeatureExtract() {}

  /**
   * @brief Extracts features from an input image.
   * 
   * @param inputImage Input image in OpenCV Mat format.
   * @return A vector of Detection objects
   */
  virtual std::vector<Detection> extract(const cv::Mat& inputImage) = 0;

  /**
   * @brief Configures the feature extractor with a given threshold.
   * 
   * @param threshold Threshold parameter for extraction.
   */
  virtual void config(double threshold) = 0;

protected:
    double _threshold; ///< Threshold parameter for extraction.
};

#endif  // FEATURE_EXTRACT_HPP
