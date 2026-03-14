#ifndef LOGGER_H
#define LOGGER_H

#include <NvInfer.h>
#include <iostream>

/**
 * TensorRT Logger implementation
 * Filters and logs TensorRT messages
 */
class Logger : public nvinfer1::ILogger {
public:
    void log(Severity severity, const char* msg) noexcept override {
        if (severity <= Severity::kWARNING) {
            std::cout << "[TRT] " << msg << std::endl;
        }
    }
};

#endif // LOGGER_H

