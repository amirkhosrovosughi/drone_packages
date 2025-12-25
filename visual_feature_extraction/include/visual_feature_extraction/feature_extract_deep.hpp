#ifndef FEATURE_EXTRACT_DEEP_HPP
#define FEATURE_EXTRACT_DEEP_HPP

#include "feature_extract.hpp"

#include "rclcpp/rclcpp.hpp"
#include <opencv2/dnn.hpp>
#include <onnxruntime_cxx_api.h>
#include <string>

/**
 * @brief Deep-learning based feature extractor using YOLO ONNX.
 */
class FeatureExtractDeep : public FeatureExtract {
private:
    struct Detection
    {
        cv::Rect box;
        float confidence;
        int classId;
    };

public:
    FeatureExtractDeep();
    virtual ~FeatureExtractDeep();

    std::vector<std::vector<double>> extract(const cv::Mat& inputImage) override;

    void config(double threshold) override;

private:
    void preprocess(const cv::Mat& frame, cv::Mat& blob);
    std::vector<Detection> postprocess(
        const cv::Mat& img,
        const std::vector<float>& output,
        int rows,
        int cols);

private:
    cv::dnn::Net _net;
    float _confThreshold = 0.25;
    float _nmsThreshold  = 0.45;
    int _inputWidth      = 640;
    int _inputHeight     = 640;
    
    Ort::Env _env{ORT_LOGGING_LEVEL_WARNING, "yolo_inference"};
    Ort::SessionOptions _sessionOptions;
    std::unique_ptr<Ort::Session> _session;

    Ort::AllocatorWithDefaultOptions _allocator;
    std::optional<Ort::AllocatedStringPtr> _inputName;
    std::optional<Ort::AllocatedStringPtr> _outputName;
};

#endif  // FEATURE_EXTRACT_DEEP_HPP
