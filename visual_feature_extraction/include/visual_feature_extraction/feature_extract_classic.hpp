#ifndef FEATURE_EXTRACT_CLASSIC_HPP
#define FEATURE_EXTRACT_CLASSIC_HPP

#include "rclcpp/rclcpp.hpp"

#include "feature_extract.hpp"

/**
 * @brief Feature extraction using classical image processing methods.
 */
class FeatureExtractClassic : public FeatureExtract {
public: 
    /**
     * @brief Constructor.
     */
    FeatureExtractClassic();
    virtual ~FeatureExtractClassic();

    /**
     * @brief Extracts coordinates of detected objects in the input image.
     * 
     * @param inputImage The input image for feature extraction.
     * @return Vector of cv::Point representing detected object coordinates.
     */
    virtual std::vector<std::vector<double>> extract(const cv::Mat& inputImage) override;

    /**
     * @brief Configures the extractor.
     * 
     * @param threshold Threshold value for feature extraction.
     */
    virtual void config(double threshold) override;

private:
    
    /**
     * @brief Fills the center of the mask with a specified width.
     * 
     * @param mask The input mask.
     * @param width The width of the center to be filled.
     * @return The mask with the filled center.
     */
    void fillCenter(const cv::Mat& mask, int width, cv::Mat& filledMask);

    /**
     * @brief Gets the center coordinates of the contours.
     * 
     * @param contours Vector of contours.
     * @return Vector of cv::Point representing contour center coordinates.
     */
    std::vector<cv::Point> getContourCenter(const std::vector<std::vector<cv::Point>>& contours);

private:
    cv::Mat _processedImage;  
};

#endif  // FEATURE_EXTRACT_CLASSIC_HPP
