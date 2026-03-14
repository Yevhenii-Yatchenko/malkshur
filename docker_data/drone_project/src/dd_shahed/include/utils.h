#ifndef UTILS_H
#define UTILS_H

#include <string>
#include <opencv2/opencv.hpp>
#include "detection.h"
#include "direction_vector.h"

/**
 * Check if file has video extension
 */
bool hasVideoExtension(const std::string& path);

/**
 * Parse show flag from string
 */
bool parseShowFlag(const std::string& flag);

/**
 * Parse TCP source format: tcp://host:port
 */
bool parseTCPSource(const std::string& source, std::string& host, int& port);

/**
 * Create JSON string from detection
 */
std::string createDetectionJson(const Detection& det, const cv::Mat& frame,
                                float scaleX, float scaleY, float cx_img, float cy_img,
                                const DirectionVectorInfo& vecInfo);

/**
 * Open CSI camera on Jetson Nano using GStreamer pipeline
 * @param sensorId Camera sensor ID (0 or 1)
 * @param width Capture width (default 1920)
 * @param height Capture height (default 1080)
 * @param fps Framerate (default 30)
 * @return GStreamer pipeline string
 */
std::string getCSICameraPipeline(int sensorId = 0, int width = 1920, int height = 1080, int fps = 30);

/**
 * Open camera with CSI support for Jetson Nano
 * @param cap VideoCapture object to initialize
 * @param source Camera source ("cam", "camera", "0" or camera index)
 * @param width Camera capture width (default 1920)
 * @param height Camera capture height (default 1080)
 * @return true if successful
 */
bool openCamera(cv::VideoCapture& cap, const std::string& source, int width = 1920, int height = 1080);

/**
 * Prepare results directory: create if not exists
 * @param resultsDir Path to results directory (default: "results")
 * @return true if successful
 */
bool prepareResultsDirectory(const std::string& resultsDir = "results");

/**
 * Get or create timestamped subdirectory in results directory
 * Creates subdirectory with format: YYYYMMDD_HHMMSS_mmm
 * @param resultsDir Base results directory path
 * @return Path to timestamped subdirectory, empty string on error
 */
std::string getTimestampedSubdirectory(const std::string& resultsDir);

/**
 * Save detection frame with vectors and bbox annotations
 * @param frame Frame to save (will be copied and annotated)
 * @param dets Detection results
 * @param firstVec Direction vector info for first detection
 * @param firstObjPt Object point for first detection
 * @param cx_img Image center X
 * @param cy_img Image center Y
 * @param scaleX Scale factor X
 * @param scaleY Scale factor Y
 * @param frameCount Frame number (for unique filename)
 * @param resultsDir Results directory path
 * @param instFps Instant FPS (for display)
 * @param avgFps Average FPS (for display)
 * @return true if successful
 */
bool saveDetectionFrame(const cv::Mat& frame, const std::vector<Detection>& dets,
                        const DirectionVectorInfo& firstVec, const cv::Point& firstObjPt,
                        float cx_img, float cy_img, float scaleX, float scaleY,
                        int frameCount, const std::string& resultsDir,
                        double instFps, double avgFps);

#endif // UTILS_H

