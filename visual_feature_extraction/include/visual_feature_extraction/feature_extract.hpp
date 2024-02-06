#ifndef FEATURE_EXTRACT_HPP
#define FEATURE_EXTRACT_HPP

#include "opencv2/opencv.hpp"

class FeatureExtract {
public:
  virtual ~FeatureExtract() {}

  virtual std::vector<std::vector<double>> extract(const cv::Mat& inputImage) = 0;
  virtual void config(double threshold) = 0;

protected:
    double _threshold;
};

#endif  // FEATURE_EXTRACT_HPP
