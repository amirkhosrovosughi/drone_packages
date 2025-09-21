#ifndef FEATURE_EXTRACT_HPP
#define FEATURE_EXTRACT_HPP

#include "opencv2/opencv.hpp"
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
   * @return A 2D vector of doubles containing extracted feature coordinates.
   */
  virtual std::vector<std::vector<double>> extract(const cv::Mat& inputImage) = 0;

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
