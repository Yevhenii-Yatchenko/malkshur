#include "direction_vector.h"
#include <cmath>

DirectionVectorInfo computeDirectionVectorShifted(
    float cx_shift, float cy_shift,
    int image_w, int image_h)
{
    DirectionVectorInfo info{};

    // Distance in pixels from center
    float mag = std::sqrt(cx_shift * cx_shift + cy_shift * cy_shift);

    float max_dist = std::sqrt(
        std::pow(image_w / 2.0f, 2.0f) +
        std::pow(image_h / 2.0f, 2.0f)
    );
    float mag_norm = (max_dist > 0.0f) ? (mag / max_dist) : 0.0f;

    // Angular deviations (degrees)
    float x_angle = (cx_shift / (image_w / 2.0f)) * (CAMERA_FOV_X / 2.0f);
    float y_angle = (-cy_shift / (image_h / 2.0f)) * (CAMERA_FOV_Y / 2.0f);

    float vx = std::tan(x_angle * float(M_PI) / 180.0f);
    float vy = std::tan(y_angle * float(M_PI) / 180.0f);
    float vz = 1.0f;

    float norm = std::sqrt(vx * vx + vy * vy + vz * vz);
    if (norm > 0.0f) {
        vx /= norm;
        vy /= norm;
        vz /= norm;
    } else {
        vx = 0.0f;
        vy = 0.0f;
        vz = 1.0f;
    }

    // Yaw/pitch for visualization
    float yaw_deg   = std::atan2(vx,  vz) * 180.0f / float(M_PI);
    float pitch_deg = std::atan2(-vy, vz) * 180.0f / float(M_PI);

    info.vx = vx;
    info.vy = vy;
    info.vz = vz;
    info.magnitude = mag;
    info.magnitude_normalized = mag_norm;
    info.yaw_deg = yaw_deg;
    info.pitch_deg = pitch_deg;
    return info;
}

