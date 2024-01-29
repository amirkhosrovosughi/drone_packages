#ifndef FEATURE_EXTRACT_DEEP_HPP
#define FEATURE_EXTRACT_DEEP_HPP

#include "feature_extract.hpp"

class FeatureExtractDeep : public FeatureExtract {
public:
  FeatureExtractDeep();
  virtual ~FeatureExtractDeep();

  virtual std::vector<cv::Point> extract(const cv::Mat& inputImage) override {}
  virtual void config(double threshold) override
  {
    std::vector<cv::Point> detectedCoordinates;
    return detectedCoordinates;
  }

  // Add specific methods or members for Deep feature extraction
};

#endif  // FEATURE_EXTRACT_DEEP_HPP
