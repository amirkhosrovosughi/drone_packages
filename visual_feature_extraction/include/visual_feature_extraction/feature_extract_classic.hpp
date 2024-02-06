#ifndef FEATURE_EXTRACT_CLASSIC_HPP
#define FEATURE_EXTRACT_CLASSIC_HPP

#include "rclcpp/rclcpp.hpp"

#include "feature_extract.hpp"

class FeatureExtractClassic : public FeatureExtract {
public:
    FeatureExtractClassic();
    virtual ~FeatureExtractClassic();

    virtual std::vector<std::vector<double>> extract(const cv::Mat& inputImage) override;
    virtual void config(double threshold) override;

private:
    // Specific methods or members for Classic feature extraction
    void fillTheCenter(const cv::Mat& mask, int width, cv::Mat& filledMask);
    std::vector<cv::Point> getContourCenter(const std::vector<std::vector<cv::Point>>& contours);

private:
    cv::Mat _processedImage;  
};

#endif  // FEATURE_EXTRACT_CLASSIC_HPP
