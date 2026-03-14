#include "detection.h"
#include <algorithm>
#include <cmath>

float iou(const Detection& a, const Detection& b) {
    float xx1 = std::max(a.x1, b.x1);
    float yy1 = std::max(a.y1, b.y1);
    float xx2 = std::min(a.x2, b.x2);
    float yy2 = std::min(a.y2, b.y2);

    float w = std::max(0.0f, xx2 - xx1);
    float h = std::max(0.0f, yy2 - yy1);
    float inter = w * h;

    float areaA = std::max(0.0f, a.x2 - a.x1) * std::max(0.0f, a.y2 - a.y1);
    float areaB = std::max(0.0f, b.x2 - b.x1) * std::max(0.0f, b.y2 - b.y1);

    float uni = areaA + areaB - inter;
    if (uni <= 0.0f) return 0.0f;
    return inter / uni;
}

std::vector<Detection> nms(const std::vector<Detection>& dets, float iouThresh) {
    if (dets.empty()) return {};

    std::vector<Detection> sorted = dets;
    std::sort(sorted.begin(), sorted.end(),
              [](const Detection& a, const Detection& b) {
                  return a.conf > b.conf;
              });

    std::vector<Detection> result;
    for (const auto& det : sorted) {
        bool keep = true;
        for (const auto& rd : result) {
            if (iou(det, rd) > iouThresh) {
                keep = false;
                break;
            }
        }
        if (keep) {
            result.push_back(det);
        }
    }
    return result;
}

