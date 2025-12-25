#include "feature_extract_deep.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "ament_index_cpp/get_package_share_directory.hpp"
// #include <onnxruntime_cxx_api.h>

static const std::string MODEL_PATH = "/model/best.onnx";

FeatureExtractDeep::FeatureExtractDeep()
    : _env(ORT_LOGGING_LEVEL_WARNING, "yolo_inference")
{
    _sessionOptions.SetIntraOpNumThreads(1);
    _sessionOptions.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);

    // Get package share directory
    std::string pkg_share_dir =
        ament_index_cpp::get_package_share_directory("visual_feature_extraction");

    std::string model_path = pkg_share_dir + MODEL_PATH;

    RCLCPP_DEBUG(rclcpp::get_logger("visual_feature_extraction"), "Deep detection path is: %S", model_path.c_str());

    try {
        _session = std::make_unique<Ort::Session>(
            _env,                       // Ort::Env&
            model_path.c_str(),          // const char* model path
            _sessionOptions              // Ort::SessionOptions&
        );
        RCLCPP_DEBUG(rclcpp::get_logger("visual_feature_extraction"), "Session created successfully");
    } catch (const Ort::Exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("visual_feature_extraction"), "FAILED to create ONNX session: %s", e.what());
        return;
    }

    // Allocate names
    _inputName  = _session->GetInputNameAllocated(0, _allocator);
    _outputName = _session->GetOutputNameAllocated(0, _allocator);

    // Read input shape
    Ort::TypeInfo inputTypeInfo = _session->GetInputTypeInfo(0);
    Ort::ConstTensorTypeAndShapeInfo inputInfo = inputTypeInfo.GetTensorTypeAndShapeInfo();

    std::vector<int64_t> inputShape = inputInfo.GetShape();

    #ifdef DEBUG_FEATURE
    // Inspecting inputInfo
    try {
        // Print element type
        ONNXTensorElementDataType elemType = inputInfo.GetElementType();
        RCLCPP_DEBUG(rclcpp::get_logger("visual_feature_extraction"), "Element type %s", elemType);

        // Print number of dimensions
        size_t dimCount = inputInfo.GetDimensionsCount();
        RCLCPP_DEBUG(rclcpp::get_logger("visual_feature_extraction"), "Element type %d", dimCount);

        // Try to print each dimension one by one
        std::vector<int64_t> dims(dimCount);
        inputInfo.GetDimensions(dims.data(), dims.size());

        std::cout << "Dims: ";
        for (auto d : dims) std::cout << d << " ";
        std::cout << std::endl;

    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("visual_feature_extraction"), "inputInfo inspection error: : %s", e.what());
    }

    // Debug print the raw dims
    std::cout << "Model input shape: [";
    for (size_t i = 0; i < inputShape.size(); ++i) {
      std::cout << inputShape[i] << (i + 1 < inputShape.size() ? ", " : "");
    }
    std::cout << "]\n";
    #endif

    _inputHeight    = inputShape[2];
    _inputWidth     = inputShape[3];

    _confThreshold = 0.25;
    _nmsThreshold  = 0.45;

    RCLCPP_INFO(rclcpp::get_logger("visual_feature_extraction"), "[INFO] ONNX Runtime YOLO model loaded");

    #ifdef DEBUG_FEATURE
    cv::namedWindow("Detected Frame", cv::WINDOW_NORMAL);
    cv::resizeWindow("Detected Frame", 1280, 720);
    #endif
}

FeatureExtractDeep::~FeatureExtractDeep()
{
}

void FeatureExtractDeep::config(double threshold)
{
    _threshold = threshold;
    _confThreshold = threshold;
}

void FeatureExtractDeep::preprocess(const cv::Mat& frame, cv::Mat& blob)
{
    cv::Mat resized, floatImg;
    cv::resize(frame, resized, cv::Size(_inputWidth, _inputHeight));
    resized.convertTo(floatImg, CV_32F, 1.0 / 255.0);

    // HWC → CHW
    std::vector<cv::Mat> channels(3);
    cv::split(floatImg, channels);

    blob = cv::Mat(3, _inputHeight * _inputWidth, CV_32F);
    for (int c = 0; c < 3; c++) {
        memcpy(blob.ptr<float>(c),
               channels[c].reshape(1, 1).data,
               _inputHeight * _inputWidth * sizeof(float));
    }
}

std::vector<FeatureExtractDeep::Detection> FeatureExtractDeep::postprocess(
        const cv::Mat& frame,
        const std::vector<float>& preds,
        int rows, int cols)
{
    std::vector<cv::Rect> boxes;
    std::vector<float> confidences;
    std::vector<int> classIds;

    const float* data = preds.data();

    int num_positions = cols;

    for (int i = 0; i < num_positions; ++i)
    {
        float conf = preds[4 * num_positions + i];

        if (conf < _confThreshold)
        {
            continue;
        }

        RCLCPP_DEBUG(rclcpp::get_logger("visual_feature_extraction"), "conf confidence: %f ", conf);

        float cx = preds[0 * num_positions + i];
        float cy = preds[1 * num_positions + i];
        float w  = preds[2 * num_positions + i];
        float h = preds[3 * num_positions + i];

        float scale_x = (float)frame.cols / _inputWidth;
        float scale_y = (float)frame.rows / _inputHeight;

        float x = (cx - 0.5f * w) * scale_x;
        float y = (cy - 0.5f * h) * scale_y;
        float width  = w * scale_x;
        float height = h * scale_y;

        RCLCPP_DEBUG(rclcpp::get_logger("visual_feature_extraction"), "(cx, cy): (%d, %d)", cx, cy);
        RCLCPP_DEBUG(rclcpp::get_logger("visual_feature_extraction"), "(x, y): (%d, %d)", x, y);

        RCLCPP_DEBUG(rclcpp::get_logger("visual_feature_extraction"), "(w, h): (%d, %d)", w, h);
        RCLCPP_DEBUG(rclcpp::get_logger("visual_feature_extraction"), "(width, height): (%d, %d)", width, height);

        boxes.emplace_back(x, y, width, height);
        confidences.emplace_back(conf);    
    }

    RCLCPP_INFO(rclcpp::get_logger("visual_feature_extraction"), "number of detected boxes: %d", boxes.size());

    // NMS
    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, _confThreshold, _nmsThreshold, indices);

    std::vector<Detection> detections;

    for (int idx : indices)
    {
        detections.push_back({
            boxes[idx],
            confidences[idx],
            0 // there is only one class
        });
    }

    return detections;
}

std::vector<std::vector<double>> FeatureExtractDeep::extract(const cv::Mat& inputImage)
{
    RCLCPP_DEBUG(rclcpp::get_logger("visual_feature_extraction"), "deep extraction of features");
    if (inputImage.empty())
        return {};

    cv::Mat blob;
    preprocess(inputImage, blob);

    std::array<int64_t, 4> inputShape{1, 3, _inputHeight, _inputWidth};

    size_t inputTensorSize = 3 * _inputHeight * _inputWidth;

    Ort::MemoryInfo memInfo = Ort::MemoryInfo::CreateCpu(
        OrtArenaAllocator, OrtMemTypeDefault);

    Ort::Value inputTensor = Ort::Value::CreateTensor<float>(
        memInfo,
        (float*)blob.data,
        inputTensorSize,
        inputShape.data(),
        inputShape.size());

    const char* input_names[]  = {_inputName->get()};
    const char* output_names[] = {_outputName->get()};

    auto outputTensors = _session->Run(
        Ort::RunOptions{nullptr},
        input_names,
        &inputTensor,
        1,
        output_names,
        1);

    float* rawOutput = outputTensors.front().GetTensorMutableData<float>();

    auto outInfo = outputTensors.front().GetTensorTypeAndShapeInfo();
    auto outShape = outInfo.GetShape();

    int rows = outShape[1];
    int cols = outShape[2];

    RCLCPP_DEBUG(rclcpp::get_logger("visual_feature_extraction"), "onnx output shape:  (%d, %d, %d): %d", outShape[0], rows, cols);

    std::vector<float> outputVec(rawOutput, rawOutput + rows * cols);

    auto detections = postprocess(inputImage, outputVec, rows, cols);

    std::vector<std::vector<double>> result;
    for (const auto& d : detections)
    {
        result.push_back({
            (d.box.x + d.box.width  * 0.5) / inputImage.cols,
            (d.box.y + d.box.height * 0.5) / inputImage.rows
        });
    }

    RCLCPP_INFO(rclcpp::get_logger("visual_feature_extraction"), "Number of detection poles  %ld", detections.size());

    #ifdef DEBUG_FEATURE
    // visualize Results
    cv::Mat debugImage = inputImage.clone();

    for (const auto& det : detections)
    {
        cv::rectangle(debugImage, det.box, cv::Scalar(0,255,0), 2);
        cv::putText(
            debugImage,
            cv::format("conf=%.2f", det.confidence),
            det.box.tl() + cv::Point(0, -5),
            cv::FONT_HERSHEY_SIMPLEX,
            0.5,
            cv::Scalar(0,255,0),
            1
        );
    }
    cv::imshow("Detected Frame", debugImage);
    cv::waitKey(1);
    #endif

    return result;
}
