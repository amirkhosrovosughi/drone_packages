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

    struct LetterBoxInfo
    {
        float scale;
        int pad_x;
        int pad_y;
    };

    LetterBoxInfo letterbox(
        const cv::Mat& src,
        cv::Mat& dst,
        int new_w,
        int new_h,
        const cv::Scalar& color = cv::Scalar(114,114,114))
    {
        float r = std::min(
            (float)new_w / src.cols,
            (float)new_h / src.rows
        );

        int resized_w = int(std::round(src.cols * r));
        int resized_h = int(std::round(src.rows * r));

        cv::Mat resized;
        cv::resize(src, resized, cv::Size(resized_w, resized_h));

        int pad_w = new_w - resized_w;
        int pad_h = new_h - resized_h;

        int pad_left   = pad_w / 2;
        int pad_right  = pad_w - pad_left;
        int pad_top    = pad_h / 2;
        int pad_bottom = pad_h - pad_top;

        cv::copyMakeBorder(
            resized,
            dst,
            pad_top, pad_bottom,
            pad_left, pad_right,
            cv::BORDER_CONSTANT,
            color
        );

        return { r, pad_left, pad_top };
    }

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
    LetterBoxInfo _letterboxInfo;
};

#endif  // FEATURE_EXTRACT_DEEP_HPP
