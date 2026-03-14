#include "utils.h"
#include <cctype>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <ctime>
#include <iostream>
#include <sys/stat.h>
#include <dirent.h>
#include <unistd.h>
#include <cstring>

bool hasVideoExtension(const std::string& path) {
    auto pos = path.find_last_of('.');
    if (pos == std::string::npos) return false;
    std::string ext = path.substr(pos);
    for (auto& c : ext) c = std::tolower(c);
    return (ext == ".mp4" || ext == ".avi" || ext == ".mov" || ext == ".mkv");
}

bool parseShowFlag(const std::string& flag) {
    std::string f = flag;
    for (auto& c : f) c = std::tolower(c);

    if (f == "0" || f == "off" || f == "noshow" || f == "nodebug" || f == "false")
        return false;
    if (f == "1" || f == "on" || f == "show" || f == "debug" || f == "true")
        return true;
    return true; // default: show
}

bool parseTCPSource(const std::string& source, std::string& host, int& port) {
    if (source.substr(0, 6) != "tcp://") {
        return false;
    }

    std::string hostPort = source.substr(6);
    size_t colonPos = hostPort.find(':');

    if (colonPos != std::string::npos) {
        host = hostPort.substr(0, colonPos);
        try {
            port = std::stoi(hostPort.substr(colonPos + 1));
        } catch (...) {
            port = 5001;
        }
    } else {
        host = hostPort;
        port = 5001;
    }

    return true;
}

std::string createDetectionJson(const Detection& det, const cv::Mat& frame,
                                float scaleX, float scaleY, float cx_img, float cy_img,
                                const DirectionVectorInfo& vecInfo) {
    int origW = frame.cols;
    int origH = frame.rows;

    float x1f = det.x1 * scaleX;
    float y1f = det.y1 * scaleY;
    float x2f = det.x2 * scaleX;
    float y2f = det.y2 * scaleY;

    float obj_cx_orig = (x1f + x2f) * 0.5f;
    float obj_cy_orig = (y1f + y2f) * 0.5f;

    float cx_shift = obj_cx_orig - cx_img;
    float cy_shift = obj_cy_orig - cy_img;

    float x_min_shift = x1f - cx_img;
    float y_min_shift = y1f - cy_img;
    float x_max_shift = x2f - cx_img;
    float y_max_shift = y2f - cy_img;
    float boxW = x2f - x1f;
    float boxH = y2f - y1f;

    std::ostringstream json;
    json << "{";
    json << "\"image_info\":{";
    json << "\"format\":\"opencv_bgr\",";
    json << "\"channels\":" << frame.channels() << ",";
    json << "\"width\":" << origW << ",";
    json << "\"height\":" << origH << ",";
    json << "\"timestamp\":0";
    json << "},";

    json << "\"coordinates\":{";
    json << "\"x_min\":" << x_min_shift << ",";
    json << "\"y_min\":" << y_min_shift << ",";
    json << "\"x_max\":" << x_max_shift << ",";
    json << "\"y_max\":" << y_max_shift << ",";
    json << "\"width\":" << boxW << ",";
    json << "\"height\":" << boxH << ",";
    json << "\"center\":[" << cx_shift << "," << cy_shift << "]";
    json << "},";

    json << "\"class_id\":" << det.cls << ",";
    json << "\"confidence\":" << det.conf << ",";

    json << "\"direction_vector\":{";
    json << "\"direction\":["
         << vecInfo.vx << ","
         << vecInfo.vy << ","
         << vecInfo.vz << "],";
    json << "\"magnitude\":" << vecInfo.magnitude << ",";
    json << "\"magnitude_normalized\":" << vecInfo.magnitude_normalized;
    json << "}";

    json << "}";
    return json.str();
}

std::string getCSICameraPipeline(int sensorId, int width, int height, int fps) {
    std::ostringstream pipeline;
    pipeline << "nvarguscamerasrc sensor-id=" << sensorId << " ! "
             << "video/x-raw(memory:NVMM), "
             << "width=(int)" << width << ", "
             << "height=(int)" << height << ", "
             << "format=(string)NV12, "
             << "framerate=(fraction)" << fps << "/1 ! "
             << "nvvidconv flip-method=0 ! "
             << "video/x-raw, "
             << "width=(int)" << width << ", "
             << "height=(int)" << height << ", "
             << "format=(string)BGRx ! "
             << "videoconvert ! "
             << "video/x-raw, format=(string)BGR ! "
             << "appsink";
    return pipeline.str();
}

bool openCamera(cv::VideoCapture& cap, const std::string& source, int width, int height) {
    // Try to open as CSI camera first (for Jetson Nano)
    if (source == "cam" || source == "camera" || source == "0") {
        std::cout << "[INFO] Attempting to open CSI camera with GStreamer...\n";

        // Try CSI camera with GStreamer pipeline
        std::string pipeline = getCSICameraPipeline(0, width, height, 30);
        std::cout << "[INFO] GStreamer pipeline: " << pipeline << "\n";

        cap.open(pipeline, cv::CAP_GSTREAMER);

        if (cap.isOpened()) {
            std::cout << "[INFO] CSI camera opened successfully via GStreamer\n";
            return true;
        }

        std::cout << "[WARN] Failed to open CSI camera, trying USB camera...\n";

        // Fallback to USB camera
        cap.open(0);

        if (cap.isOpened()) {
            std::cout << "[INFO] USB camera opened successfully\n";
            return true;
        }

        return false;
    }

    // For other sources, use default OpenCV behavior
    return false;
}

bool prepareResultsDirectory(const std::string& resultsDir) {
    // Check if directory exists
    struct stat info;
    bool exists = (stat(resultsDir.c_str(), &info) == 0 && S_ISDIR(info.st_mode));
    
    if (!exists) {
        // Create directory
        if (mkdir(resultsDir.c_str(), 0755) != 0) {
            std::cerr << "[ERROR] Failed to create results directory: " << resultsDir << std::endl;
            return false;
        }
        std::cout << "[INFO] Created results directory: " << resultsDir << std::endl;
    }
    
    return true;
}

std::string getTimestampedSubdirectory(const std::string& resultsDir) {
    static std::string cachedSubdir;  // Cache subdirectory path for this run
    
    if (!cachedSubdir.empty()) {
        return cachedSubdir;  // Return cached path if already created
    }
    
    // Generate timestamp with milliseconds
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;
    
    std::ostringstream subdirName;
    subdirName << resultsDir << "/";
    subdirName << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S");
    subdirName << "_" << std::setfill('0') << std::setw(3) << ms.count();
    
    std::string subdirPath = subdirName.str();
    
    // Create subdirectory
    if (mkdir(subdirPath.c_str(), 0755) != 0) {
        std::cerr << "[ERROR] Failed to create timestamped subdirectory: " << subdirPath << std::endl;
        return "";
    }
    
    cachedSubdir = subdirPath;
    std::cout << "[INFO] Created timestamped subdirectory: " << subdirPath << std::endl;
    
    return cachedSubdir;
}

bool saveDetectionFrame(const cv::Mat& frame, const std::vector<Detection>& dets,
                        const DirectionVectorInfo& firstVec, const cv::Point& firstObjPt,
                        float cx_img, float cy_img, float scaleX, float scaleY,
                        int frameCount, const std::string& resultsDir,
                        double instFps, double avgFps) {
    if (dets.empty()) {
        return false;  // Only save frames with detections
    }
    
    // Create a copy of the frame to annotate
    cv::Mat annotatedFrame = frame.clone();
    
    // Draw bounding boxes and labels
    for (size_t i = 0; i < dets.size(); ++i) {
        const auto& d = dets[i];
        
        float x1f = d.x1 * scaleX;
        float y1f = d.y1 * scaleY;
        float x2f = d.x2 * scaleX;
        float y2f = d.y2 * scaleY;
        
        int x1 = (int)x1f;
        int y1 = (int)y1f;
        int x2 = (int)x2f;
        int y2 = (int)y2f;
        
        // Draw bounding box
        cv::rectangle(annotatedFrame, cv::Point(x1, y1), cv::Point(x2, y2),
                     cv::Scalar(0, 255, 0), 2);
        
        // Draw label
        char text[64];
        std::snprintf(text, sizeof(text), "shahed %.2f", d.conf);
        cv::putText(annotatedFrame, text, cv::Point(x1, std::max(y1 - 5, 0)),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
    }
    
    // Draw center point
    cv::Point centerPt((int)cx_img, (int)cy_img);
    cv::circle(annotatedFrame, centerPt, 5, cv::Scalar(0, 255, 0), -1);
    cv::circle(annotatedFrame, centerPt, 10, cv::Scalar(0, 255, 0), 2);
    
    // Draw direction vector arrow
    cv::arrowedLine(annotatedFrame, centerPt, firstObjPt,
                   cv::Scalar(0, 255, 255), 3, cv::LINE_AA, 0, 0.15);
    
    // Draw object point
    cv::circle(annotatedFrame, firstObjPt, 8, cv::Scalar(255, 0, 255), -1);
    cv::circle(annotatedFrame, firstObjPt, 8, cv::Scalar(0, 0, 0), 2);
    
    // Draw FPS info
    char fpsText[64];
    std::snprintf(fpsText, sizeof(fpsText), "FPS: inst %.1f  avg %.1f", instFps, avgFps);
    cv::putText(annotatedFrame, fpsText, cv::Point(10, 20),
               cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);
    
    // Draw vector information
    char line1[128];
    std::snprintf(line1, sizeof(line1), "Vector: [%+.2f,%+.2f,%+.2f]",
                 firstVec.vx, firstVec.vy, firstVec.vz);
    
    char line2[128];
    std::snprintf(line2, sizeof(line2), "Yaw: %+.1fdeg  Pitch: %+.1fdeg",
                 firstVec.yaw_deg, firstVec.pitch_deg);
    
    char line3[128];
    std::snprintf(line3, sizeof(line3), "Magnitude: %.1fpx (%.1f%%)",
                 firstVec.magnitude, firstVec.magnitude_normalized * 100.0f);
    
    std::vector<std::string> texts = {line1, line2, line3};
    
    int y0 = 30;
    int lineHeight = 25;
    int x0 = 10;
    cv::Scalar bg(0, 0, 0);
    cv::Scalar fg(255, 255, 255);
    double fontScale = 0.6;
    int thickness = 2;
    int baseline = 0;
    
    for (size_t i = 0; i < texts.size(); ++i) {
        cv::Size textSize = cv::getTextSize(
            texts[i], cv::FONT_HERSHEY_SIMPLEX,
            fontScale, thickness, &baseline
        );
        int y = y0 + (int)i * lineHeight;
        cv::rectangle(
            annotatedFrame,
            cv::Point(x0 - 5, y - textSize.height - 5),
            cv::Point(x0 + textSize.width + 5, y + baseline + 5),
            bg,
            cv::FILLED
        );
        cv::putText(
            annotatedFrame,
            texts[i],
            cv::Point(x0, y),
            cv::FONT_HERSHEY_SIMPLEX,
            fontScale,
            fg,
            thickness,
            cv::LINE_AA
        );
    }
    
    // Get or create timestamped subdirectory
    std::string subdir = getTimestampedSubdirectory(resultsDir);
    if (subdir.empty()) {
        std::cerr << "[ERROR] Failed to get timestamped subdirectory" << std::endl;
        return false;
    }
    
    // Generate simple filename with frame number
    std::ostringstream filename;
    filename << subdir << "/frame" << std::setfill('0') << std::setw(6) << frameCount << ".jpg";
    
    std::string filepath = filename.str();
    
    if (cv::imwrite(filepath, annotatedFrame)) {
        std::cout << "[INFO] Saved detection frame: " << filepath << std::endl;
        return true;
    } else {
        std::cerr << "[WARN] Failed to save detection frame: " << filepath << std::endl;
        return false;
    }
}

