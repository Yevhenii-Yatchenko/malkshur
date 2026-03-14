#include "trt_engine.h"
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <cuda_runtime.h>

void TrtYolo::checkCuda(cudaError_t status, const char* msg) {
    if (status != cudaSuccess) {
        std::cerr << msg << " CUDA error: " << cudaGetErrorString(status) << std::endl;
        std::exit(1);
    }
}

std::vector<char> TrtYolo::loadEngineFile(const std::string& path) {
    std::ifstream file(path, std::ios::binary);
    if (!file) {
        throw std::runtime_error("Failed to open engine file: " + path);
    }
    file.seekg(0, std::ios::end);
    std::size_t size = file.tellg();
    file.seekg(0, std::ios::beg);

    std::vector<char> data(size);
    file.read(data.data(), size);
    return data;
}

// Helper deleter functions for TensorRT objects
static void deleteRuntime(nvinfer1::IRuntime* p) {
    if (p) p->destroy();
}

static void deleteEngine(nvinfer1::ICudaEngine* p) {
    if (p) p->destroy();
}

static void deleteContext(nvinfer1::IExecutionContext* p) {
    if (p) p->destroy();
}

TrtYolo::TrtYolo(Logger& logger) 
    : logger_(logger)
    , runtime_(nullptr, deleteRuntime)
    , engine_(nullptr, deleteEngine)
    , context_(nullptr, deleteContext)
{
}

TrtYolo::~TrtYolo() {
    if (deviceInput_)  cudaFree(deviceInput_);
    if (deviceOutput_) cudaFree(deviceOutput_);
}

void TrtYolo::load(const std::string& enginePath) {
    auto engineData = loadEngineFile(enginePath);

    runtime_ = std::unique_ptr<nvinfer1::IRuntime, void(*)(nvinfer1::IRuntime*)>(
        nvinfer1::createInferRuntime(logger_), deleteRuntime);
    if (!runtime_) {
        throw std::runtime_error("Failed to create IRuntime");
    }

    engine_ = std::unique_ptr<nvinfer1::ICudaEngine, void(*)(nvinfer1::ICudaEngine*)>(
        runtime_->deserializeCudaEngine(engineData.data(), engineData.size()), deleteEngine);
    if (!engine_) {
        throw std::runtime_error("Failed to deserialize ICudaEngine");
    }

    context_ = std::unique_ptr<nvinfer1::IExecutionContext, void(*)(nvinfer1::IExecutionContext*)>(
        engine_->createExecutionContext(), deleteContext);
    if (!context_) {
        throw std::runtime_error("Failed to create IExecutionContext");
    }

    int nbBindings = engine_->getNbBindings();
    std::cout << "[INFO] Number of bindings: " << nbBindings << std::endl;

    for (int i = 0; i < nbBindings; ++i) {
        bool isInput = engine_->bindingIsInput(i);
        nvinfer1::Dims dims = context_->getBindingDimensions(i);
        auto dtype = engine_->getBindingDataType(i);
        std::cout << "  Binding " << i
                  << " name=" << engine_->getBindingName(i)
                  << " isInput=" << isInput
                  << " dims=";
        for (int d = 0; d < dims.nbDims; ++d) {
            std::cout << dims.d[d];
            if (d < dims.nbDims - 1) std::cout << "x";
        }
        std::cout << " dtype=" << static_cast<int>(dtype) << std::endl;

        if (isInput) {
            inputIndex_ = i;
            inputDims_ = dims;
        } else {
            outputIndex_ = i;
            outputDims_ = dims;
        }
    }

    if (inputIndex_ < 0 || outputIndex_ < 0) {
        throw std::runtime_error("Could not find input/output binding");
    }

    if (inputDims_.nbDims != 4) {
        throw std::runtime_error("Expected 4D input (1,3,H,W), got nbDims="
                                 + std::to_string(inputDims_.nbDims));
    }

    int batch = inputDims_.d[0];
    int ch    = inputDims_.d[1];
    inH       = inputDims_.d[2];
    inW       = inputDims_.d[3];
    if (batch != 1 || ch != 3) {
        std::cerr << "[WARN] Non-strict input shape expectations: batch="
                  << batch << " ch=" << ch << std::endl;
    }

    if (!(outputDims_.nbDims == 3 && outputDims_.d[0] == 1 && outputDims_.d[1] == 5)) {
        throw std::runtime_error("Expected output format (1,5,8400), got different format.");
    }

    attrs_    = outputDims_.d[1];   // 5
    numPreds_ = outputDims_.d[2];   // 8400

    std::cout << "[INFO] Output dims: ";
    for (int i = 0; i < outputDims_.nbDims; ++i) {
        std::cout << outputDims_.d[i];
        if (i < outputDims_.nbDims - 1) std::cout << "x";
    }
    std::cout << "  (attrs=" << attrs_ << ", numPreds=" << numPreds_ << ")\n";

    int batchSize = 1;
    inputSizeBytes_ = batchSize * 3 * inH * inW * sizeof(float);

    size_t outputElemCount = 1;
    for (int i = 0; i < outputDims_.nbDims; ++i) {
        outputElemCount *= outputDims_.d[i];
    }
    outputSizeBytes_ = outputElemCount * sizeof(float);

    checkCuda(cudaMalloc(&deviceInput_, inputSizeBytes_), "cudaMalloc input");
    checkCuda(cudaMalloc(&deviceOutput_, outputSizeBytes_), "cudaMalloc output");

    inputData_.resize(3 * inH * inW);
    outputData_.resize(outputElemCount);
    rawDetections_.reserve(numPreds_);
}

bool TrtYolo::infer(const cv::Mat& frameBGR, float confThresh, float iouThresh,
                    std::vector<Detection>& finalDetections) {
    if (frameBGR.empty()) return false;

    // 1. Preprocessing: resize -> RGB -> float32
    cv::Mat resized;
    cv::resize(frameBGR, resized, cv::Size(inW, inH));

    cv::Mat rgb;
    cv::cvtColor(resized, rgb, cv::COLOR_BGR2RGB);

    cv::Mat floatImg;
    rgb.convertTo(floatImg, CV_32FC3, 1.0f / 255.0f);

    int offsetR = 0;
    int offsetG = inH * inW;
    int offsetB = 2 * inH * inW;

    for (int y = 0; y < inH; ++y) {
        const float* row = floatImg.ptr<float>(y);
        for (int x = 0; x < inW; ++x) {
            float r = row[3 * x + 0];
            float g = row[3 * x + 1];
            float b = row[3 * x + 2];
            int idx = y * inW + x;
            inputData_[offsetR + idx] = r;
            inputData_[offsetG + idx] = g;
            inputData_[offsetB + idx] = b;
        }
    }

    // 2. Copy to GPU
    checkCuda(cudaMemcpy(deviceInput_, inputData_.data(), inputSizeBytes_,
                         cudaMemcpyHostToDevice),
              "cudaMemcpy H2D input");

    // 3. Run inference
    std::vector<void*> bindings(engine_->getNbBindings());
    bindings[inputIndex_]  = deviceInput_;
    bindings[outputIndex_] = deviceOutput_;

    bool ok = context_->enqueueV2(bindings.data(), 0, nullptr);
    if (!ok) {
        std::cerr << "[ERROR] enqueueV2 failed\n";
        return false;
    }

    // 4. Copy from GPU
    checkCuda(cudaMemcpy(outputData_.data(), deviceOutput_, outputSizeBytes_,
                         cudaMemcpyDeviceToHost),
              "cudaMemcpy D2H output");

    // 5. Parse raw detections
    rawDetections_.clear();

    for (int i = 0; i < numPreds_; ++i) {
        float x    = outputData_[(0 * attrs_ + 0) * numPreds_ + i];
        float y    = outputData_[(0 * attrs_ + 1) * numPreds_ + i];
        float w    = outputData_[(0 * attrs_ + 2) * numPreds_ + i];
        float h    = outputData_[(0 * attrs_ + 3) * numPreds_ + i];
        float conf = outputData_[(0 * attrs_ + 4) * numPreds_ + i];

        if (conf < confThresh)
            continue;

        float x1 = x - w / 2.0f;
        float y1 = y - h / 2.0f;
        float x2 = x + w / 2.0f;
        float y2 = y + h / 2.0f;

        x1 = std::max(0.0f, std::min(x1, (float)inW - 1));
        y1 = std::max(0.0f, std::min(y1, (float)inH - 1));
        x2 = std::max(0.0f, std::min(x2, (float)inW - 1));
        y2 = std::max(0.0f, std::min(y2, (float)inH - 1));

        Detection det;
        det.x1 = x1;
        det.y1 = y1;
        det.x2 = x2;
        det.y2 = y2;
        det.conf = conf;
        det.cls = 0; // Single class
        rawDetections_.push_back(det);
    }

    // 6. NMS
    finalDetections = nms(rawDetections_, iouThresh);
    return true;
}

