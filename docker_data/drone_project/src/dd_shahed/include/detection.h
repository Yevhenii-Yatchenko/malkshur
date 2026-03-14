#ifndef DETECTION_H
#define DETECTION_H

#include <vector>

/**
 * Detection bounding box structure
 */
struct Detection {
    float x1, y1, x2, y2;  // Bounding box coordinates
    float conf;             // Confidence score
    int cls;                // Class ID
};

/**
 * Calculate Intersection over Union (IoU) between two detections
 */
float iou(const Detection& a, const Detection& b);

/**
 * Non-Maximum Suppression (NMS) to filter overlapping detections
 * @param dets Input detections
 * @param iouThresh IoU threshold for filtering
 * @return Filtered detections
 */
std::vector<Detection> nms(const std::vector<Detection>& dets, float iouThresh);

#endif // DETECTION_H

