#ifndef FEATURE_EXTRACT_DEEP_HPP
#define FEATURE_EXTRACT_DEEP_HPP

#include "feature_extract.hpp"

/**
 * @brief Feature extraction using deep learning methods (to be implemented).
 */
class FeatureExtractDeep : public FeatureExtract {
public:
  FeatureExtractDeep() {}
  virtual ~FeatureExtractDeep() {}

  virtual std::vector<std::vector<double>> extract(const cv::Mat& inputImage) override
  {
    // TODO: Implement deep feature extraction
  }
  virtual void config(double threshold) override
  {
    std::vector<cv::Point> detectedCoordinates;
    return detectedCoordinates;
  }

  // Add specific methods or members for Deep feature extraction
};

#endif  // FEATURE_EXTRACT_DEEP_HPP
