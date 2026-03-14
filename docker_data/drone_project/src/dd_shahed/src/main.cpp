#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <chrono>
#include <stdexcept>
#include <opencv2/opencv.hpp>

#include "logger.h"
#include "trt_engine.h"
#include "detection.h"
#include "direction_vector.h"
#include "socket_client.h"
#include "tcp_frame_client.h"
#include "utils.h"

/**
 * Parse command line arguments (both positional and named)
 * Returns map of argument values and list of positional arguments
 */
std::map<std::string, std::string> parseNamedArgs(int argc, char** argv, std::vector<std::string>& positional) {
    std::map<std::string, std::string> namedArgs;
    
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        
        // Check if it's a named argument (starts with --)
        if (arg.size() > 2 && arg.substr(0, 2) == "--") {
            std::string key = arg.substr(2);
            std::string value;
            
            // Check for --key=value format
            size_t eqPos = key.find('=');
            if (eqPos != std::string::npos) {
                value = key.substr(eqPos + 1);
                key = key.substr(0, eqPos);
            } else if (i + 1 < argc && argv[i + 1][0] != '-') {
                // --key value format
                value = argv[i + 1];
                i++; // Skip next argument as it's the value
            } else {
                // Flag without value (boolean)
                value = "1";
            }
            
            // Normalize key (convert to lowercase, replace - with _)
            for (auto& c : key) {
                if (c == '-') c = '_';
                else c = std::tolower(c);
            }
            
            namedArgs[key] = value;
        } else {
            // Positional argument
            positional.push_back(arg);
        }
    }
    
    return namedArgs;
}

/**
 * Process single image mode
 */
int processSingleImage(TrtYolo& detector, const std::string& imagePath,
                       float confThresh, float iouThresh) {
    cv::Mat img = cv::imread(imagePath);
    if (img.empty()) {
        throw std::runtime_error("Failed to read image: " + imagePath);
    }

    std::vector<Detection> dets;
    auto t0 = std::chrono::high_resolution_clock::now();
    if (!detector.infer(img, confThresh, iouThresh, dets)) {
        throw std::runtime_error("infer() returned false");
    }
    auto t1 = std::chrono::high_resolution_clock::now();
    double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    double fps = (ms > 0.0) ? (1000.0 / ms) : 0.0;

    std::cout << "[INFO] Single image inference time: " << ms << " ms ("
              << fps << " FPS)\n";

    cv::Mat resized;
    cv::resize(img, resized, cv::Size(detector.inW, detector.inH));

    int imgW = resized.cols;
    int imgH = resized.rows;
    float cx_img = imgW / 2.0f;
    float cy_img = imgH / 2.0f;

    for (size_t i = 0; i < dets.size(); ++i) {
        const auto& d = dets[i];
        std::cout << "#" << i
                  << " conf=" << d.conf
                  << " cls=" << d.cls
                  << " box=(" << d.x1 << ", " << d.y1
                  << ", " << d.x2 << ", " << d.y2 << ")\n";

        float obj_cx = (d.x1 + d.x2) * 0.5f;
        float obj_cy = (d.y1 + d.y2) * 0.5f;

        float cx_shift = obj_cx - cx_img;
        float cy_shift = obj_cy - cy_img;

        DirectionVectorInfo vecInfo = computeDirectionVectorShifted(
            cx_shift, cy_shift, imgW, imgH
        );

        cv::rectangle(
            resized,
            cv::Point((int)d.x1, (int)d.y1),
            cv::Point((int)d.x2, (int)d.y2),
            cv::Scalar(0, 255, 0),
            2
        );
        char text[64];
        std::snprintf(text, sizeof(text), "shahed %.2f", d.conf);
        cv::putText(
            resized,
            text,
            cv::Point((int)d.x1, (int)std::max(d.y1 - 5.0f, 0.0f)),
            cv::FONT_HERSHEY_SIMPLEX,
            0.5,
            cv::Scalar(0, 255, 0),
            1
        );
    }

    if (!dets.empty()) {
        const auto& d0 = dets[0];
        float obj_cx = (d0.x1 + d0.x2) * 0.5f;
        float obj_cy = (d0.y1 + d0.y2) * 0.5f;

        cv::Point center((int)cx_img, (int)cy_img);
        cv::Point objPt((int)obj_cx, (int)obj_cy);

        cv::circle(resized, center, 5, cv::Scalar(0, 255, 0), -1);
        cv::arrowedLine(resized, center, objPt,
                        cv::Scalar(0, 255, 255), 2, cv::LINE_AA, 0, 0.15);
    }

    if (cv::imwrite("result.jpg", resized)) {
        std::cout << "[INFO] Saved result.jpg" << std::endl;
    } else {
        std::cout << "[WARN] Failed to save result.jpg" << std::endl;
    }

    return 0;
}

/**
 * Process TCP stream mode
 */
int processTCPStream(TrtYolo& detector, const std::string& tcpHost, int tcpPort,
                     float confThresh, float iouThresh, bool showWindow,
                     const std::string& serverHost, int serverPort, bool useTCP,
                     int detectEveryN, int takeDetectPhotoN) {
    TCPFrameClient tcpClient(tcpHost, tcpPort);

    std::cout << "[INFO] Connecting to frame server " << tcpHost << ":" << tcpPort << "..." << std::endl;
    if (!tcpClient.connect()) {
        throw std::runtime_error("Failed to connect to TCP frame server");
    }

    std::cout << "[INFO] Connected! Starting inference from TCP stream..." << std::endl;
    std::cout << "[INFO] Starting stream inference. Output window: "
              << (showWindow ? "ON" : "OFF") << ". Press 'q' to exit.\n";
    std::cout << "[INFO] Socket target: " << serverHost << ":" << serverPort
              << " (" << (useTCP ? "TCP" : "UDP") << ")" << std::endl;
    std::cout << "[INFO] Detection every " << detectEveryN << " frame(s)" << std::endl;

    cv::Mat frame;
    int frameCount = 0;
    double totalMs = 0.0;
    
    // Store last detection results for skipped frames
    std::vector<Detection> lastDets;
    float lastScaleX = 1.0f, lastScaleY = 1.0f;
    float lastCxImg = 0.0f, lastCyImg = 0.0f;
    int lastOrigW = 0, lastOrigH = 0;

    while (true) {
        auto t0 = std::chrono::high_resolution_clock::now();

        if (!tcpClient.read(frame)) {
            std::cout << "[INFO] TCP stream ended or connection lost" << std::endl;
            break;
        }

        if (frame.empty()) {
            std::cerr << "[WARN] Received empty frame from TCP stream\n";
            continue;
        }

        std::vector<Detection> dets;
        bool shouldDetect = (frameCount % detectEveryN == 0);
        
        auto tDetectStart = std::chrono::high_resolution_clock::now();
        if (shouldDetect) {
            if (!detector.infer(frame, confThresh, iouThresh, dets)) {
                std::cerr << "[WARN] infer() returned false for frame\n";
                continue;
            }
            // Store results for skipped frames
            lastDets = dets;
        } else {
            // Use last detection results
            dets = lastDets;
        }
        auto tDetectEnd = std::chrono::high_resolution_clock::now();
        auto t1 = std::chrono::high_resolution_clock::now();
        
        // Measure total frame processing time (for instFps)
        double frameMs = std::chrono::duration<double, std::milli>(t1 - t0).count();
        // Measure detection time only (for avgFps)
        double detectMs = shouldDetect ? std::chrono::duration<double, std::milli>(tDetectEnd - tDetectStart).count() : 0.0;

        frameCount++;
        if (shouldDetect) {
            totalMs += detectMs;
        }
        // Average FPS: based on detection time only (how fast we can detect)
        double avgFps = (totalMs > 0.0) ? (1000.0 * (frameCount / detectEveryN) / totalMs) : 0.0;
        // Instant FPS: based on total frame processing time (how fast we process frames)
        double instFps = (frameMs > 0.0) ? (1000.0 / frameMs) : 0.0;

        int origW = frame.cols;
        int origH = frame.rows;
        float scaleX = (float)origW / detector.inW;
        float scaleY = (float)origH / detector.inH;
        
        if (shouldDetect) {
            lastScaleX = scaleX;
            lastScaleY = scaleY;
            lastOrigW = origW;
            lastOrigH = origH;
        } else {
            scaleX = lastScaleX;
            scaleY = lastScaleY;
            origW = lastOrigW;
            origH = lastOrigH;
        }

        float maxConf = 0.0f;
        bool hasDet = !dets.empty();

        float cx_img = origW / 2.0f;
        float cy_img = origH / 2.0f;

        bool firstDone = false;
        DirectionVectorInfo firstVec{};
        cv::Point firstObjPt(origW / 2, origH / 2);

        for (size_t i = 0; i < dets.size(); ++i) {
            auto& d = dets[i];

            float x1f = d.x1 * scaleX;
            float y1f = d.y1 * scaleY;
            float x2f = d.x2 * scaleX;
            float y2f = d.y2 * scaleY;

            int x1 = (int)x1f;
            int y1 = (int)y1f;
            int x2 = (int)x2f;
            int y2 = (int)y2f;

            if (d.conf > maxConf)
                maxConf = d.conf;

            float obj_cx_orig = (x1f + x2f) * 0.5f;
            float obj_cy_orig = (y1f + y2f) * 0.5f;

            float cx_shift = obj_cx_orig - cx_img;
            float cy_shift = obj_cy_orig - cy_img;

            DirectionVectorInfo vecInfo = computeDirectionVectorShifted(
                cx_shift, cy_shift, origW, origH
            );

            if (!firstDone) {
                firstDone = true;
                firstVec = vecInfo;
                firstObjPt = cv::Point((int)obj_cx_orig, (int)obj_cy_orig);
            }

            std::string json = createDetectionJson(d, frame, scaleX, scaleY, cx_img, cy_img, vecInfo);

            if (useTCP) {
                sendJsonToServerTCP(serverHost, serverPort, json);
            } else {
                sendJsonToServerUDP(serverHost, serverPort, json);
            }

            if (showWindow) {
                cv::rectangle(frame, cv::Point(x1, y1), cv::Point(x2, y2),
                            cv::Scalar(0, 255, 0), 2);
                char text[64];
                std::snprintf(text, sizeof(text), "shahed %.2f", d.conf);
                cv::putText(frame, text, cv::Point(x1, std::max(y1 - 5, 0)),
                          cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
            }
        }

        if (!showWindow) {
            if (hasDet) {
                std::cout << "[INFO] frame=" << frameCount
                          << " FPS=" << instFps
                          << " max_conf=" << maxConf << std::endl;
            } else {
                std::cout << "[INFO] frame=" << frameCount
                          << " FPS=" << instFps
                          << " no_detections" << std::endl;
            }
        } else {
            char fpsText[64];
            std::snprintf(fpsText, sizeof(fpsText), "FPS: inst %.1f  avg %.1f",
                        instFps, avgFps);
            cv::putText(frame, fpsText, cv::Point(10, 20),
                      cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);
        }

        if (showWindow && hasDet) {
            cv::Point centerPt((int)cx_img, (int)cy_img);
            cv::circle(frame, centerPt, 5, cv::Scalar(0, 255, 0), -1);
            cv::circle(frame, centerPt, 10, cv::Scalar(0, 255, 0), 2);
            cv::arrowedLine(frame, centerPt, firstObjPt,
                          cv::Scalar(0, 255, 255), 3, cv::LINE_AA, 0, 0.15);
            cv::circle(frame, firstObjPt, 8, cv::Scalar(255, 0, 255), -1);
            cv::circle(frame, firstObjPt, 8, cv::Scalar(0, 0, 0), 2);
        }

        // Save detection frame if enabled and conditions are met
        if (takeDetectPhotoN > 0 && hasDet && (frameCount % takeDetectPhotoN == 0)) {
            saveDetectionFrame(frame, dets, firstVec, firstObjPt, cx_img, cy_img,
                             scaleX, scaleY, frameCount, "results", instFps, avgFps);
        }

        if (showWindow) {
            cv::imshow("YOLO-TensorRT (TCP Stream)", frame);
            char key = (char)cv::waitKey(1);
            if (key == 'q' || key == 27) {
                std::cout << "[INFO] Exit by key press.\n";
                break;
            }
        }

        if (showWindow && frameCount % 30 == 0) {
            std::cout << "[INFO] frame=" << frameCount
                      << " inst=" << instFps << " FPS"
                      << " avg=" << avgFps << " FPS\n";
        }
    }

    tcpClient.release();
    if (showWindow) cv::destroyAllWindows();

    if (useTCP) {
        closeTCPConnection();
    }

    int sent, failed;
    if (useTCP) {
        getTCPStats(sent, failed);
        std::cout << "\n[TCP STATS] Packets sent: " << sent
                  << ", Failed: " << failed << std::endl;
    } else {
        getUDPStats(sent, failed);
        std::cout << "\n[UDP STATS] Packets sent: " << sent
                  << ", Failed: " << failed << std::endl;
    }

    return 0;
}

/**
 * Process video/camera mode
 */
int processVideoCamera(TrtYolo& detector, const std::string& source,
                       float confThresh, float iouThresh, bool showWindow,
                       const std::string& serverHost, int serverPort, bool useTCP,
                       int cameraWidth, int cameraHeight, bool rotate180, int detectEveryN,
                       int takeDetectPhotoN) {
    cv::VideoCapture cap;
    if (source == "cam" || source == "camera" || source == "0") {
        // Try to open with CSI camera support
        if (!openCamera(cap, source, cameraWidth, cameraHeight)) {
            throw std::runtime_error("Failed to open camera");
        }
    } else {
        std::cout << "[INFO] Opening video: " << source << "\n";
        cap.open(source);
        if (!cap.isOpened()) {
            throw std::runtime_error("Failed to open video/camera source: " + source);
        }
    }

    cv::Mat frame;
    int frameCount = 0;
    double totalMs = 0.0;
    
    // Store last detection results for skipped frames
    std::vector<Detection> lastDets;
    float lastScaleX = 1.0f, lastScaleY = 1.0f;
    float lastCxImg = 0.0f, lastCyImg = 0.0f;
    int lastOrigW = 0, lastOrigH = 0;

    std::cout << "[INFO] Starting stream inference. "
              << "Output window: " << (showWindow ? "ON" : "OFF")
              << ". Press 'q' to exit.\n";
    std::cout << "[INFO] Socket target: " << serverHost << ":" << serverPort << std::endl;
    std::cout << "[INFO] Detection every " << detectEveryN << " frame(s)" << std::endl;

    while (true) {
        auto t0 = std::chrono::high_resolution_clock::now();
        
        if (!cap.read(frame) || frame.empty()) {
            std::cout << "[INFO] Video/stream ended.\n";
            break;
        }

        // Apply rotation if needed
        if (rotate180) {
            cv::rotate(frame, frame, cv::ROTATE_180);
        }

        bool shouldDetect = (frameCount % detectEveryN == 0);
        
        auto tDetectStart = std::chrono::high_resolution_clock::now();
        std::vector<Detection> dets;
        if (shouldDetect) {
            if (!detector.infer(frame, confThresh, iouThresh, dets)) {
                std::cerr << "[WARN] infer() returned false for frame\n";
                continue;
            }
            // Store results for skipped frames
            lastDets = dets;
        } else {
            // Use last detection results
            dets = lastDets;
        }
        auto tDetectEnd = std::chrono::high_resolution_clock::now();
        auto t1 = std::chrono::high_resolution_clock::now();
        
        // Measure total frame processing time (for instFps)
        double frameMs = std::chrono::duration<double, std::milli>(t1 - t0).count();
        // Measure detection time only (for avgFps)
        double detectMs = shouldDetect ? std::chrono::duration<double, std::milli>(tDetectEnd - tDetectStart).count() : 0.0;

        frameCount++;
        if (shouldDetect) {
            totalMs += detectMs;
        }
        // Average FPS: based on detection time only (how fast we can detect)
        double avgFps = (totalMs > 0.0) ? (1000.0 * (frameCount / detectEveryN) / totalMs) : 0.0;
        // Instant FPS: based on total frame processing time (how fast we process frames)
        double instFps = (frameMs > 0.0) ? (1000.0 / frameMs) : 0.0;

        int origW = frame.cols;
        int origH = frame.rows;
        float scaleX = (float)origW / detector.inW;
        float scaleY = (float)origH / detector.inH;
        
        if (shouldDetect) {
            lastScaleX = scaleX;
            lastScaleY = scaleY;
            lastOrigW = origW;
            lastOrigH = origH;
        } else {
            scaleX = lastScaleX;
            scaleY = lastScaleY;
            origW = lastOrigW;
            origH = lastOrigH;
        }

        float maxConf = 0.0f;
        bool hasDet = !dets.empty();

        float cx_img = origW / 2.0f;
        float cy_img = origH / 2.0f;

        bool firstDone = false;
        DirectionVectorInfo firstVec{};
        cv::Point firstObjPt(origW / 2, origH / 2);

        for (size_t i = 0; i < dets.size(); ++i) {
            auto& d = dets[i];

            float x1f = d.x1 * scaleX;
            float y1f = d.y1 * scaleY;
            float x2f = d.x2 * scaleX;
            float y2f = d.y2 * scaleY;

            int x1 = (int)x1f;
            int y1 = (int)y1f;
            int x2 = (int)x2f;
            int y2 = (int)y2f;

            if (d.conf > maxConf)
                maxConf = d.conf;

            float obj_cx_orig = (x1f + x2f) * 0.5f;
            float obj_cy_orig = (y1f + y2f) * 0.5f;

            float cx_shift = obj_cx_orig - cx_img;
            float cy_shift = obj_cy_orig - cy_img;

            DirectionVectorInfo vecInfo = computeDirectionVectorShifted(
                cx_shift, cy_shift, origW, origH
            );

            if (!firstDone) {
                firstDone = true;
                firstVec = vecInfo;
                firstObjPt = cv::Point((int)obj_cx_orig, (int)obj_cy_orig);
            }

            std::string json = createDetectionJson(d, frame, scaleX, scaleY, cx_img, cy_img, vecInfo);

            if (useTCP) {
                sendJsonToServerTCP(serverHost, serverPort, json);
            } else {
                sendJsonToServerUDP(serverHost, serverPort, json);
            }

            if (showWindow) {
                cv::rectangle(
                    frame,
                    cv::Point(x1, y1),
                    cv::Point(x2, y2),
                    cv::Scalar(0, 255, 0),
                    2
                );
                char text[64];
                std::snprintf(text, sizeof(text), "shahed %.2f", d.conf);
                cv::putText(
                    frame,
                    text,
                    cv::Point(x1, std::max(y1 - 5, 0)),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.5,
                    cv::Scalar(0, 255, 0),
                    1
                );
            }
        }

        if (!showWindow) {
            if (hasDet) {
                std::cout << "[INFO] frame=" << frameCount
                          << " FPS=" << instFps
                          << " max_conf=" << maxConf << std::endl;
            } else {
                std::cout << "[INFO] frame=" << frameCount
                          << " FPS=" << instFps
                          << " no_detections" << std::endl;
            }
        } else {
            char fpsText[64];
            std::snprintf(fpsText, sizeof(fpsText), "FPS: inst %.1f  avg %.1f",
                          instFps, avgFps);
            cv::putText(
                frame,
                fpsText,
                cv::Point(10, 20),
                cv::FONT_HERSHEY_SIMPLEX,
                0.5,
                cv::Scalar(0, 255, 255),
                1
            );
        }

        if (showWindow && hasDet) {
            cv::Point centerPt((int)cx_img, (int)cy_img);

            cv::circle(frame, centerPt, 5, cv::Scalar(0, 255, 0), -1);
            cv::circle(frame, centerPt, 10, cv::Scalar(0, 255, 0), 2);

            cv::arrowedLine(
                frame,
                centerPt,
                firstObjPt,
                cv::Scalar(0, 255, 255), 3, cv::LINE_AA, 0, 0.15
            );

            cv::circle(frame, firstObjPt, 8, cv::Scalar(255, 0, 255), -1);
            cv::circle(frame, firstObjPt, 8, cv::Scalar(0, 0, 0), 2);

            char line1[128];
            std::snprintf(line1, sizeof(line1),
                          "Vector: [%+.2f,%+.2f,%+.2f]",
                          firstVec.vx, firstVec.vy, firstVec.vz);

            char line2[128];
            std::snprintf(line2, sizeof(line2),
                          "Yaw: %+.1fdeg  Pitch: %+.1fdeg",
                          firstVec.yaw_deg, firstVec.pitch_deg);

            char line3[128];
            std::snprintf(line3, sizeof(line3),
                          "Magnitude: %.1fpx (%.1f%%)",
                          firstVec.magnitude,
                          firstVec.magnitude_normalized * 100.0f);

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
                    frame,
                    cv::Point(x0 - 5, y - textSize.height - 5),
                    cv::Point(x0 + textSize.width + 5, y + baseline + 5),
                    bg,
                    cv::FILLED
                );
                cv::putText(
                    frame,
                    texts[i],
                    cv::Point(x0, y),
                    cv::FONT_HERSHEY_SIMPLEX,
                    fontScale,
                    fg,
                    thickness,
                    cv::LINE_AA
                );
            }
        }

        // Save detection frame if enabled and conditions are met
        if (takeDetectPhotoN > 0 && hasDet && (frameCount % takeDetectPhotoN == 0)) {
            saveDetectionFrame(frame, dets, firstVec, firstObjPt, cx_img, cy_img,
                             scaleX, scaleY, frameCount, "results", instFps, avgFps);
        }

        if (showWindow) {
            cv::imshow("YOLO-TensorRT", frame);
            char key = (char)cv::waitKey(1);
            if (key == 'q' || key == 27) {
                std::cout << "[INFO] Exit by key press.\n";
                break;
            }
        }

        if (showWindow && frameCount % 30 == 0) {
            std::cout << "[INFO] frame=" << frameCount
                      << " inst=" << instFps << " FPS"
                      << " avg=" << avgFps << " FPS\n";
        }
    }

    cap.release();
    if (showWindow) cv::destroyAllWindows();

    if (useTCP) {
        closeTCPConnection();
    }

    int sent, failed;
    if (useTCP) {
        getTCPStats(sent, failed);
        std::cout << "\n[TCP STATS] Packets sent: " << sent
                  << ", Failed: " << failed << std::endl;
    } else {
        getUDPStats(sent, failed);
        std::cout << "\n[UDP STATS] Packets sent: " << sent
                  << ", Failed: " << failed << std::endl;
    }

    return 0;
}

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cout << "Usage:\n"
                  << "  Positional arguments:\n"
                  << "    " << argv[0] << " <engine> <input> [conf] [show|noshow] [server_host] [server_port] [udp|tcp] [camera_width] [camera_height] [rotate180] [detect_every] [take_detect_photo_n]\n"
                  << "\n"
                  << "  Named arguments (recommended):\n"
                  << "    " << argv[0] << " <engine> <input> [--conf <value>] [--show <show|noshow>] [--server-host <ip>] [--server-port <port>] [--protocol <udp|tcp>] [--camera-width <width>] [--camera-height <height>] [--rotate180 <0|1>] [--detect-every <N>] [--take-detect-photo-n <N>]\n"
                  << "\n"
                  << "  Named arguments can also use = format:\n"
                  << "    " << argv[0] << " <engine> <input> --conf=0.25 --show=show --server-host=172.17.0.1 --server-port=5000 --protocol=udp --camera-width=640 --camera-height=640 --rotate180=1 --detect-every=2 --take-detect-photo-n=30\n"
                  << "\n"
                  << "Parameters:\n"
                  << "  engine                - Path to TensorRT .engine model (required)\n"
                  << "  input                   - Video source: image.jpg, video.mp4, cam, or tcp://host:port (required)\n"
                  << "  --conf                  - Confidence threshold (default: 0.25)\n"
                  << "  --show                  - Display GUI: show/noshow (default: show)\n"
                  << "  --server-host           - Results server IP (default: 172.17.0.1)\n"
                  << "  --server-port           - Results server port (default: 5000)\n"
                  << "  --protocol              - Results protocol: udp or tcp (default: udp)\n"
                  << "  --camera-width          - Camera capture width (default: 1920)\n"
                  << "  --camera-height         - Camera capture height (default: 1080)\n"
                  << "  --rotate180             - Rotate image 180 degrees: 1/true/yes or 0/false/no (default: 0)\n"
                  << "  --detect-every          - Run detection every N frames (default: 1, every frame)\n"
                  << "  --take-detect-photo-n    - Save detection frames every N frames (0 = disabled, default: 0)\n"
                  << "                            Only saves frames with positive detections to results/ directory\n"
                  << "\n"
                  << "Examples:\n"
                  << "  # Camera with named arguments (640x640, rotated, UDP):\n"
                  << "  " << argv[0] << " model.engine cam --conf=0.25 --show=show --server-host=172.17.0.1 --server-port=5000 --protocol=udp --camera-width=640 --camera-height=640 --rotate180=1\n"
                  << "\n"
                  << "  # TCP video input + UDP results (positional):\n"
                  << "  " << argv[0] << " model.engine tcp://172.19.173.99:5001 0.25 show 172.19.173.99 5000 udp\n"
                  << "\n"
                  << "  # Camera with mixed arguments and photo capture:\n"
                  << "  " << argv[0] << " model.engine cam --conf 0.25 --show show --server-host 172.17.0.1 --server-port 5000 --protocol udp --camera-width 640 --camera-height 640 --rotate180 1 --take-detect-photo-n 30\n";
        return 0;
    }

    // Parse arguments (both named and positional)
    std::vector<std::string> positional;
    std::map<std::string, std::string> namedArgs = parseNamedArgs(argc, argv, positional);

    // Get required positional arguments
    if (positional.size() < 2) {
        std::cerr << "[ERROR] Missing required arguments: <engine> <input>\n";
        return 1;
    }

    std::string enginePath = positional[0];
    std::string source     = positional[1];

    // Parse named arguments or fall back to positional
    float confThresh = 0.25f;
    if (namedArgs.count("conf")) {
        confThresh = std::stof(namedArgs["conf"]);
    } else if (positional.size() >= 3) {
        confThresh = std::stof(positional[2]);
    }

    bool showWindow = true;
    if (namedArgs.count("show")) {
        showWindow = parseShowFlag(namedArgs["show"]);
    } else if (positional.size() >= 4) {
        showWindow = parseShowFlag(positional[3]);
    }

    std::string serverHost = "172.17.0.1";
    if (namedArgs.count("server_host") || namedArgs.count("server-host")) {
        serverHost = namedArgs.count("server_host") ? namedArgs["server_host"] : namedArgs["server-host"];
    } else if (positional.size() >= 5) {
        serverHost = positional[4];
    }

    int serverPort = 5000;
    if (namedArgs.count("server_port") || namedArgs.count("server-port")) {
        std::string portStr = namedArgs.count("server_port") ? namedArgs["server_port"] : namedArgs["server-port"];
        serverPort = std::stoi(portStr);
    } else if (positional.size() >= 6) {
        serverPort = std::stoi(positional[5]);
    }

    std::string protocol = "udp";
    if (namedArgs.count("protocol")) {
        protocol = namedArgs["protocol"];
        for (auto& c : protocol) c = std::tolower(c);
    } else if (positional.size() >= 7) {
        protocol = positional[6];
        for (auto& c : protocol) c = std::tolower(c);
    }
    bool useTCP = (protocol == "tcp");

    // Camera parameters (defaults: 1920x1080, no rotation)
    int cameraWidth = 1920;
    if (namedArgs.count("camera_width") || namedArgs.count("camera-width")) {
        std::string widthStr = namedArgs.count("camera_width") ? namedArgs["camera_width"] : namedArgs["camera-width"];
        cameraWidth = std::stoi(widthStr);
    } else if (positional.size() >= 8) {
        cameraWidth = std::stoi(positional[7]);
    }

    int cameraHeight = 1080;
    if (namedArgs.count("camera_height") || namedArgs.count("camera-height")) {
        std::string heightStr = namedArgs.count("camera_height") ? namedArgs["camera_height"] : namedArgs["camera-height"];
        cameraHeight = std::stoi(heightStr);
    } else if (positional.size() >= 9) {
        cameraHeight = std::stoi(positional[8]);
    }

    bool rotate180 = false;
    if (namedArgs.count("rotate180") || namedArgs.count("rotate_180")) {
        std::string rotateStr = namedArgs.count("rotate180") ? namedArgs["rotate180"] : namedArgs["rotate_180"];
        for (auto& c : rotateStr) c = std::tolower(c);
        rotate180 = (rotateStr == "1" || rotateStr == "true" || rotateStr == "yes" || rotateStr == "on");
    } else if (positional.size() >= 10) {
        std::string rotateStr = positional[9];
        for (auto& c : rotateStr) c = std::tolower(c);
        rotate180 = (rotateStr == "1" || rotateStr == "true" || rotateStr == "yes" || rotateStr == "on");
    }

    // Detection frequency (default: every frame)
    int detectEveryN = 1;
    if (namedArgs.count("detect_every") || namedArgs.count("detect-every")) {
        std::string detectStr = namedArgs.count("detect_every") ? namedArgs["detect_every"] : namedArgs["detect-every"];
        detectEveryN = std::stoi(detectStr);
        if (detectEveryN < 1) detectEveryN = 1;
    } else if (positional.size() >= 11) {
        detectEveryN = std::stoi(positional[10]);
        if (detectEveryN < 1) detectEveryN = 1;
    }

    // Photo capture frequency (0 = disabled)
    int takeDetectPhotoN = 0;
    if (namedArgs.count("take_detect_photo_n") || namedArgs.count("take-detect-photo-n")) {
        std::string photoStr = namedArgs.count("take_detect_photo_n") ? namedArgs["take_detect_photo_n"] : namedArgs["take-detect-photo-n"];
        takeDetectPhotoN = std::stoi(photoStr);
        if (takeDetectPhotoN < 1) takeDetectPhotoN = 0;
    } else if (positional.size() >= 12) {
        takeDetectPhotoN = std::stoi(positional[11]);
        if (takeDetectPhotoN < 1) takeDetectPhotoN = 0;
    }

    std::string tcpHost;
    int tcpPort = 5001;
    bool isTCPStream = parseTCPSource(source, tcpHost, tcpPort);

    float iouThresh = 0.5f;

    // Prepare results directory if photo capture is enabled
    if (takeDetectPhotoN > 0) {
        if (!prepareResultsDirectory("results")) {
            std::cerr << "[WARN] Failed to prepare results directory, photo capture disabled" << std::endl;
            takeDetectPhotoN = 0;
        } else {
            std::cout << "[INFO] Photo capture enabled: saving every " << takeDetectPhotoN << " frame(s) with detections" << std::endl;
            std::cout << "[INFO] Frames will be saved in timestamped subdirectories under results/" << std::endl;
        }
    }

    std::cout << "[INFO] Configuration:\n";
    std::cout << "  Engine: " << enginePath << "\n";
    if (isTCPStream) {
        std::cout << "  Input: TCP stream from " << tcpHost << ":" << tcpPort << "\n";
    } else {
        std::cout << "  Input: " << source << "\n";
        if (source == "cam" || source == "camera" || source == "0") {
            std::cout << "  Camera size: " << cameraWidth << "x" << cameraHeight << "\n";
            std::cout << "  Rotate 180: " << (rotate180 ? "YES" : "NO") << "\n";
        }
    }
    std::cout << "  Results protocol: " << (useTCP ? "TCP" : "UDP") << "\n";
    std::cout << "  Results server: " << serverHost << ":" << serverPort << "\n";
    std::cout << std::endl;

    Logger logger;
    try {
        TrtYolo detector(logger);
        std::cout << "[INFO] Loading engine: " << enginePath << std::endl;
        detector.load(enginePath);

        if (!isTCPStream && !hasVideoExtension(source) &&
            source != "cam" && source != "camera" && source != "0") {
            return processSingleImage(detector, source, confThresh, iouThresh);
        } else if (isTCPStream) {
            return processTCPStream(detector, tcpHost, tcpPort, confThresh, iouThresh,
                                   showWindow, serverHost, serverPort, useTCP, detectEveryN, takeDetectPhotoN);
        } else {
            return processVideoCamera(detector, source, confThresh, iouThresh,
                                    showWindow, serverHost, serverPort, useTCP,
                                    cameraWidth, cameraHeight, rotate180, detectEveryN, takeDetectPhotoN);
        }

    } catch (const std::exception& e) {
        std::cerr << "[ERROR] Exception: " << e.what() << std::endl;
        if (useTCP) {
            closeTCPConnection();
        }
        return 1;
    }
}

