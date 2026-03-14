#ifndef TRT_ENGINE_H
#define TRT_ENGINE_H

#include <NvInfer.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <memory>
#include "detection.h"
#include "logger.h"

/**
 * TensorRT YOLO Engine wrapper
 * Handles model loading, inference, and detection processing
 */
class TrtYolo {
public:
    explicit TrtYolo(Logger& logger);
    ~TrtYolo();

    /**
     * Load TensorRT engine from file
     * @param enginePath Path to .engine file
     */
    void load(const std::string& enginePath);

    /**
     * Run inference on a single frame
     * @param frameBGR Input frame in BGR format
     * @param confThresh Confidence threshold
     * @param iouThresh IoU threshold for NMS
     * @param finalDetections Output detections after NMS
     * @return true if inference succeeded
     */
    bool infer(const cv::Mat& frameBGR, float confThresh, float iouThresh,
               std::vector<Detection>& finalDetections);

    // Input dimensions
    int inW = 0;
    int inH = 0;

private:
    Logger& logger_;
    std::unique_ptr<nvinfer1::IRuntime, void(*)(nvinfer1::IRuntime*)> runtime_;
    std::unique_ptr<nvinfer1::ICudaEngine, void(*)(nvinfer1::ICudaEngine*)> engine_;
    std::unique_ptr<nvinfer1::IExecutionContext, void(*)(nvinfer1::IExecutionContext*)> context_;

    int inputIndex_ = -1;
    int outputIndex_ = -1;
    nvinfer1::Dims inputDims_{};
    nvinfer1::Dims outputDims_{};
    int attrs_ = 0;
    int numPreds_ = 0;

    void* deviceInput_ = nullptr;
    void* deviceOutput_ = nullptr;
    size_t inputSizeBytes_ = 0;
    size_t outputSizeBytes_ = 0;

    std::vector<float> inputData_;
    std::vector<float> outputData_;
    std::vector<Detection> rawDetections_;

    void checkCuda(cudaError_t status, const char* msg);
    std::vector<char> loadEngineFile(const std::string& path);
};

#endif // TRT_ENGINE_H

