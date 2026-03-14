# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with the sky_anchor drone stabilization system.

## Project Overview

Sky Anchor is a computer vision-based drift correction system for drones. It uses ORB (Oriented FAST and Rotated BRIEF) features to detect position drift and sends correction commands to maintain stable hover. The system runs as a TCP server sending JSON-formatted control commands to connected clients.

### Recent Analysis Findings

- The system uses a custom unbuffered logger with immediate file writing
- SkyAnchorServer is a multi-threaded TCP server broadcasting to all clients
- Controller has 5 image normalization modes for different lighting conditions
- Camera module supports both USB and CSI cameras with optimized settings
- Error recovery is built into camera init, frame capture, and feature matching
- Debug images are saved with microsecond timestamps when enabled
- The TCP server sends drift corrections as JSON with dx, dy, angle, and confidence

## Architecture

### Module Structure

```
sky_anchor/
├── main.py                    # Entry point - instantiates and runs Controller
├── app/
│   ├── controller.py          # Main Controller class - orchestrates the system
│   ├── shift_estimator.py     # ShiftEstimator classes (CPU/CUDA) for drift detection
│   ├── camera.py              # CameraInitializer for USB/CSI camera setup
│   ├── coordinate_server.py   # TCP server for sending control commands
│   ├── config.py              # Environment configuration
│   └── logger.py              # Logging utilities
```

### Key Classes

1. **Controller** (`app/controller.py`)
   - Main orchestration class managing the drift correction system
   - Initializes camera, sky anchor server, and shift estimator
   - Runs continuous control loop with error recovery
   - Handles image normalization with 5 different modes (0-4)
   - Saves debug images when DEBUG_MODE=True
   - Key features:
     - Hardcoded log path: `/home/jetson/Documents/DroneProject/logs/sky_anchor_main.log`
     - Captures 5 frames before accepting reference frame
     - Applies shift/angle thresholds before sending commands
     - Sends control data via `__sky_anchor_server.tick()`

2. **ShiftEstimator** (`app/shift_estimator.py`)
   - Abstract base class with `estimate_shift()` method
   - **CpuShiftEstimator**: CPU-based ORB feature matching
   - **CudaShiftEstimator**: GPU-accelerated version
   - Factory pattern: `get_shift_estimator()` returns appropriate implementation

3. **CameraInitializer** (`app/camera.py`)
   - Handles USB and CSI (MIPI) camera initialization
   - Returns OpenCV VideoCapture object
   - USB camera settings:
     - Auto-exposure briefly enabled, then manual exposure (-7)
     - Gain set to 50 for brightness compensation
     - Buffer size minimized (1) for low latency
   - CSI camera uses GStreamer pipeline with NVMM memory

4. **SkyAnchorServer** (`app/sky_anchor_server.py`)
   - Multi-threaded TCP server on localhost:8888
   - Broadcasts JSON data to all connected clients
   - Thread-safe client management with automatic disconnection handling
   - Key methods:
     - `start()`: Non-blocking server initialization
     - `tick(data)`: Broadcasts data to all clients (returns client count)
     - `handle_client()`: Per-client connection handler
   - Supports up to 100 simultaneous connections

5. **UnbufferedLogger** (`app/logger.py`)
   - Custom logging implementation with immediate file writing
   - Line-buffered file I/O with forced flush and OS sync
   - Singleton pattern via `get_logger()` function
   - Features:
     - Microsecond timestamp precision
     - Five log levels: DEBUG, INFO, WARNING, ERROR, CRITICAL
     - Optional console output
     - Context manager support
     - Automatic directory creation

## Control Flow

1. **Initialization** (in Controller.__init__)
   - Initialize camera (USB or CSI based on config)
   - Start coordinate server for command output
   - Create shift estimator (CPU or CUDA based on config)

2. **Main Loop** (in Controller.run)
   - Capture reference frame (with 5-frame warm-up)
   - Continuously:
     - Read current frame
     - Normalize images (based on NORMALIZE_TYPE config)
     - Estimate shift using ORB features
     - Apply thresholds (SHIFT_THRESHOLD, ANGLE_THRESHOLD)
     - Send control commands via sky anchor server
     - Sleep for SLEEP_TIME (default 0.01s)

## Shift Estimation Algorithm

### Current Implementation (CpuShiftEstimator)

1. **Feature Detection**: ORB with 500 features, 30 scale levels
2. **Feature Matching**: Brute force with cross-check validation
3. **Homography Estimation**: RANSAC-based (minimum 10 matches)
4. **Extract Motion**:
   - `dx = M[0, 2]` - horizontal shift in pixels
   - `dy = M[1, 2]` - vertical shift in pixels
   - `angle = atan2(M[1, 0], M[0, 0])` - rotation in degrees

### Image Normalization Modes

The system supports 5 normalization modes controlled by `NORMALIZE_TYPE`:

- **Mode 0**: No normalization (raw images)
- **Mode 1**: Histogram equalization for both reference and current
- **Mode 2**: Mean brightness adjustment to target (128) for both
- **Mode 3**: Histogram equalization for reference, relative mean adjustment for current
- **Mode 4**: Mean adjustment for reference, relative mean adjustment for current

Modes 3 and 4 adjust current frames to match reference frame brightness, helping with changing lighting conditions.

### Control Data Format

The server broadcasts JSON messages with the following structure:
```json
{
    "dx": float,              // Horizontal shift in pixels
    "dy": float,              // Vertical shift in pixels
    "angle_deg": float,       // Rotation in degrees
    "matches_percent": float, // Percentage of features matched
    "timestamp": float        // Unix timestamp
}
```

Values below thresholds are zeroed out before transmission.

### Known Limitations

- **Planar Assumption**: Homography assumes flat ground
- **No Altitude Compensation**: Pixel shifts vary with drone height
- **Lighting Sensitivity**: ORB features affected by shadows/lighting
- **Motion Blur**: Fast movements degrade feature detection
- **No Confidence Metrics**: No quality assessment of estimates
- **Hardcoded Paths**: Log path is fixed to `/home/jetson/Documents/DroneProject/logs/`

## Improvement Roadmap

### High Priority

1. **Add Confidence Metrics**
   ```python
   # Check homography quality
   condition_number = np.linalg.cond(M)
   if condition_number > threshold:
       # Low confidence - reduce corrections
   ```

2. **Implement Outlier Detection**
   ```python
   # Detect sudden jumps
   if abs(dx - prev_dx) > max_delta:
       # Possible tracking failure
   ```

3. **Altitude-Aware Scaling**
   ```python
   # Scale corrections based on height
   actual_dx = pixel_dx * (altitude / reference_altitude)
   ```

### Medium Priority

4. **Kalman Filter for Smoothing**
   - Predict next position
   - Smooth noisy measurements
   - Handle temporary tracking loss

5. **Multi-Scale Feature Tracking**
   - Use features at different scales
   - Better handle altitude changes

6. **Adaptive Thresholds**
   - Adjust SHIFT_THRESHOLD based on conditions
   - Tighter control in calm conditions

### Future Enhancements

7. **Alternative Algorithms**
   - Optical flow for dense tracking
   - Essential matrix for 3D motion
   - IMU fusion for absolute orientation

8. **Machine Learning**
   - Train on drone-specific scenarios
   - Learn optimal control parameters

## Error Handling and Recovery

The system includes several error recovery mechanisms:

1. **Camera Initialization**: Raises RuntimeError if camera cannot be opened
2. **Reference Frame Capture**: Retries 5 times with SLEEP_TIME delays
3. **Shift Estimation Errors**: Logs error and continues with next frame
4. **Feature Matching Failures**:
   - Match percentage must be ≥ 20%
   - Descriptors must exist in both images
5. **Client Disconnections**: Automatically handled by SkyAnchorServer

## Configuration

Key environment variables in `.env`:

```bash
# Camera
DRONE_CAMERA_TYPE=USB      # USB or SCI (CSI camera)
DRONE_CAMERA_INDEX=0       # Camera device index
DRONE_CAPTURE_WIDTH=1280   # Capture resolution
DRONE_CAPTURE_HEIGHT=720
DRONE_CAPTURE_FPS=30       # Target FPS (0 for default)

# Shift detection
DRONE_SHIFT_THRESHOLD=1.5  # Pixels - ignore smaller shifts
DRONE_ANGLE_THRESHOLD=1.5  # Degrees - ignore smaller rotations
DRONE_ORB_NFEATURES=500    # Number of ORB features
MAX_FEATURE_FAILS=5        # Max consecutive feature failures

# Image normalization
NORMALIZE_TYPE=0           # 0=none, 1=histogram, 2=mean, 3-4=relative

# Performance
DRONE_SLEEP_TIME=0.01      # Loop delay in seconds
ENABLE_CUDA=False          # Use GPU acceleration
DRONE_DEBUG=True           # Save debug images
```

## Development Guidelines

1. **Testing Shift Estimation**:
   - Use recorded video for repeatable tests
   - Log confidence metrics with shifts
   - Visualize matched features in debug mode

2. **Tuning Control Loop**:
   - Start with small correction speeds
   - Test over different surfaces/altitudes
   - Monitor oscillations (sign of high gain)

3. **Adding New Features**:
   - Maintain ShiftEstimator interface
   - Add confidence to return tuple
   - Update Controller to use confidence

## Debugging and Troubleshooting

### Debug Output
When `DRONE_DEBUG=True`, the system saves:
- Reference frames: `debug_logs/reference_YYYY_MM_DD_HH:MM:SS.ffffff.png`
- Current frames: `debug_logs/current_YYYY_MM_DD_HH:MM:SS.ffffff.png`

### Common Issues

1. **"Not enough matches to compute homography"**
   - Increase `DRONE_ORB_NFEATURES`
   - Ensure good lighting and textured surfaces
   - Check camera focus and exposure settings

2. **"Match percentage too low"**
   - Scene may have changed significantly
   - Try different `NORMALIZE_TYPE` settings
   - Reduce motion blur by decreasing exposure time

3. **High CPU usage**
   - Increase `DRONE_SLEEP_TIME`
   - Reduce `DRONE_ORB_NFEATURES`
   - Enable CUDA if available

4. **No clients receiving data**
   - Check if main controller is connected to port 8888
   - Verify no firewall blocking localhost connections

### Performance Monitoring

The system logs:
- Frame processing time (when errors occur)
- Match percentages for each frame
- Number of connected clients
- All threshold applications

Use these metrics to tune parameters for your specific use case.

## Current Work: Shift Estimation Improvements

Analyzing CpuShiftEstimator for better drone stabilization:

- Current ORB+homography works well for planar hover
- Need altitude compensation for consistent control
- Add confidence metrics to detect tracking failures
- Consider Kalman filter for smoother corrections

Next steps:
1. Add homography condition number check
2. Implement simple outlier rejection
3. Add altitude parameter to shift estimator
4. Test with various lighting conditions
5. Make log paths configurable (remove hardcoded paths)