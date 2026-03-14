#ifndef DIRECTION_VECTOR_H
#define DIRECTION_VECTOR_H

// Camera FOV constants for IMX219-83
// FOV: 83° (diag), 73° (horiz), 50° (vert)
static const float CAMERA_FOV_X = 73.0f;  // Horizontal FOV (degrees)
static const float CAMERA_FOV_Y = 50.0f;  // Vertical FOV (degrees)

/**
 * Direction vector information
 */
struct DirectionVectorInfo {
    float vx;
    float vy;
    float vz;
    float magnitude;             // pixels
    float magnitude_normalized;  // 0..1
    float yaw_deg;               // for visualization
    float pitch_deg;             // for visualization
};

/**
 * Compute direction vector from pixel offset
 * @param cx_shift X offset from image center (pixels)
 * @param cy_shift Y offset from image center (pixels)
 * @param image_w Image width (pixels)
 * @param image_h Image height (pixels)
 * @return Direction vector information
 */
DirectionVectorInfo computeDirectionVectorShifted(
    float cx_shift, float cy_shift,
    int image_w, int image_h
);

#endif // DIRECTION_VECTOR_H

