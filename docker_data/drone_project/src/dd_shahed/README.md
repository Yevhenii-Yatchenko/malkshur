# dd_shahed

**Real-time Shahed drone detection system with TensorRT FP16 inference and YOLOv11**

## Overview

This system provides optimized object detection for Shahed drones with support for multiple platforms and protocols:

- **Platforms**: Jetson Nano (ARM64) and x86_64 (RTX 4090, etc.)
- **Video Input**: Images, video files, camera streams, or TCP video streams
- **Results Output**: UDP (fast) or TCP (reliable) protocols
- **Performance**: TensorRT FP16 optimized inference at 640x640 resolution

## Features

✅ Multi-platform support (Jetson Nano + x86_64) \
✅ TCP video stream input from remote servers \
✅ UDP/TCP detection results transmission \
✅ Real-time camera and video file processing \
✅ Docker + WSL2 compatible \
✅ Direction vectors with camera FOV calculations \
✅ JSON-formatted detection output

## Quick Start

### 1. Single Image Detection

```bash
./build/bin/dd_shahed_x86_64 \
  ./models/yolov11_shahed_640s_fp16.engine \
  ./test_data/shahed_blue_sky.jpg \
  --conf=0.25
```

Or for ARM64:
```bash
./build/bin/dd_shahed_arm64 \
  ./models/yolov11_shahed_640s_fp16.engine \
  ./test_data/shahed_blue_sky.jpg \
  --conf=0.25
```

**Note:** If you enabled `COPY_BINARY_TO_BIN`, you can also use `./bin/dd_shahed_x86_64` instead.

### 2. Video File with UDP Results

```bash
# For x86_64 use:
# ./build/bin/dd_shahed_x86_64

./build/bin/dd_shahed_arm64 \
  ./models/yolov11_shahed_640s_fp16.engine \
  ./test_data/shahed_river_and_trees.mp4 \
  --conf=0.25 \
  --show=show \
  --server-host=172.17.0.1 \
  --server-port=5000 \
  --protocol=udp
```

### 3. TCP Video Stream + TCP Results

```bash
# For x86_64 use:
# ./build/bin/dd_shahed_x86_64

./build/bin/dd_shahed_arm64 \
  ./models/yolov11_shahed_640s_fp16.engine \
  tcp://172.19.173.99:5001 \
  --conf=0.25 \
  --show=show \
  --server-host=172.19.173.99 \
  --server-port=5000 \
  --protocol=tcp
```

### 4. Camera Stream with Results

```bash
# For x86_64 use:
# ./build/bin/dd_shahed_x86_64

./build/bin/dd_shahed_arm64 \
  ./models/yolov11_shahed_640s_fp16.engine \
  cam \
  --conf=0.25 \
  --show=show \
  --server-host=172.19.173.99 \
  --server-port=5000 \
  --protocol=udp
```

### 5. Camera with Custom Size and Rotation

```bash
# Camera 640x640 with 180° rotation, UDP results, detection every 2nd frame
./build/bin/dd_shahed_arm64 \
  ./models/yolov11-shashed-my-ds-model-weights-v2-640-arm.engine \
  cam \
  --conf=0.25 \
  --show=show \
  --server-host=172.17.0.1 \
  --server-port=5000 \
  --protocol=udp \
  --camera-width=640 \
  --camera-height=640 \
  --rotate180=1 \
  --detect-every=2
  --take-detect-photo-n=3
```

## Build

The project uses CMake for automated cross-platform builds. CMake automatically detects the platform (x86_64 or ARM64) and configures the build accordingly.

### Prerequisites

- CMake 3.12 or later
- CUDA toolkit
- TensorRT 8.x or later
- OpenCV 4.x with development headers
- C++14 compatible compiler (g++ or clang++)

### Build Instructions

1. **Create build directory:**
```bash
mkdir -p build
cd build
```

2. **Configure and build:**
```bash
cmake ..
make -j$(nproc)
```

The executable will be created at:
- `build/bin/dd_shahed_x86_64` (for x86_64)
- `build/bin/dd_shahed_arm64` (for ARM64)

3. **Optional: Copy binary to bin/ directory (for version control):**

If you want to copy the binary to `bin/` directory (to commit to repository), use:
```bash
cmake -DCOPY_BINARY_TO_BIN=ON ..
make -j$(nproc)
```

This will create:
- `bin/dd_shahed_x86_64` (for x86_64)
- `bin/dd_shahed_arm64` (for ARM64)

**Note:** By default, binaries are NOT copied to `bin/`. Use `-DCOPY_BINARY_TO_BIN=ON` only when you want to commit binaries to the repository.

3. **For release build (optimized):**
```bash
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)
```

4. **Build and copy to bin/ (for repository):**
```bash
cmake -DCOPY_BINARY_TO_BIN=ON ..
make -j$(nproc)
```

This will build the binary and automatically copy it to `bin/` directory with platform suffix.

### Platform-Specific Notes

**x86_64 (RTX 4090, Docker, etc.):**
- CMake will automatically detect x86_64 platform
- Links against `nvinfer` and `nvonnxparser` libraries
- Libraries are expected in `/usr/lib/x86_64-linux-gnu` or `/usr/local/lib`

**ARM64 (Jetson Nano):**
- CMake will automatically detect ARM64 platform
- Links against `nvinfer` library only (ONNX parser not required)
- Libraries are expected in `/usr/lib/aarch64-linux-gnu` or `/usr/local/lib`

### Troubleshooting Build Issues

**TensorRT not found:**
```bash
# Set TensorRT root if installed in non-standard location
export TENSORRT_ROOT=/path/to/tensorrt
cmake -DTENSORRT_ROOT=$TENSORRT_ROOT ..
```

**OpenCV not found:**
```bash
# Set OpenCV path
cmake -DOpenCV_DIR=/path/to/opencv/lib/cmake/opencv4 ..
```

**CUDA not found:**
- Ensure CUDA is installed and `nvcc` is in PATH
- Set `CUDA_ROOT` if needed: `cmake -DCUDA_ROOT=/usr/local/cuda ..`

### Convert ONNX Model to TensorRT Engine

TensorRT engines are platform-specific. To rebuild for your platform:

```bash
python3 convert_onnx_to_engine.py \
  ./models/yolov11-shashed-model-weights-v1-640.onnx \
  ./models/yolov11_shahed_640s_fp16_x86.engine \
  --fp16
```

## Command Line Arguments

The program supports both **named arguments** (recommended) and **positional arguments** (for backward compatibility).

### Named Arguments (Recommended)

```bash
./build/bin/dd_shahed_x86_64 <engine> <input> [--conf=<value>] [--show=<show|noshow>] [--server-host=<ip>] [--server-port=<port>] [--protocol=<udp|tcp>] [--camera-width=<width>] [--camera-height=<height>] [--rotate180=<0|1>] [--detect-every=<N>]
```

**Note:** If you enabled `COPY_BINARY_TO_BIN`, you can also use `./bin/dd_shahed_x86_64` instead.

### Positional Arguments (Legacy)

```bash
./build/bin/dd_shahed_x86_64 <engine> <input> [conf] [show] [server_host] [server_port] [protocol] [camera_width] [camera_height] [rotate180] [detect_every]
```

### Parameters

1. **engine** (required) - Path to TensorRT `.engine` model
2. **input** (required) - Video source:
   - Image: `./path/to/image.jpg`
   - Video: `./path/to/video.mp4`
   - Camera: `cam` or device ID like `0`
   - TCP stream: `tcp://host:port`
3. **--conf** or **conf** - Confidence threshold (default: 0.25)
4. **--show** or **show** - Display GUI: `show`/`noshow` (default: show)
5. **--server-host** or **server_host** - Results server IP (default: 172.17.0.1)
6. **--server-port** or **server_port** - Results server port (default: 5000)
7. **--protocol** or **protocol** - Results protocol: `udp` or `tcp` (default: udp)
8. **--camera-width** or **camera_width** - Camera capture width in pixels (default: 1920)
9. **--camera-height** or **camera_height** - Camera capture height in pixels (default: 1080)
10. **--rotate180** or **rotate180** - Rotate image 180 degrees: `1`/`true`/`yes`/`on` or `0`/`false`/`no`/`off` (default: 0)
11. **--detect-every** or **detect_every** - Run detection every N frames. Value 2 means detection on every 2nd frame (default: 1, every frame)

### Examples

**Named arguments with `=` format:**
```bash
./build/bin/dd_shahed_arm64 \
  model.engine cam \
  --conf=0.25 \
  --show=show \
  --server-host=172.17.0.1 \
  --server-port=5000 \
  --protocol=udp \
  --camera-width=640 \
  --camera-height=640 \
  --rotate180=1 \
  --detect-every=2
```

**Named arguments with spaces:**
```bash
./build/bin/dd_shahed_arm64 \
  model.engine cam \
  --conf 0.25 \
  --show show \
  --server-host 172.17.0.1 \
  --server-port 5000 \
  --protocol udp \
  --camera-width 640 \
  --camera-height 640 \
  --rotate180 1 \
  --detect-every 2
```

**Mixed format (named and positional):**
```bash
./build/bin/dd_shahed_arm64 \
  model.engine cam \
  --conf=0.25 \
  --show=show \
  172.17.0.1 5000 udp 640 640 1
```

## Testing Servers

### UDP Results Server
```bash
python3 mock_server.py
```

### TCP Results Server
```bash
python3 mock_server_tcp.py
```

### TCP Video Stream Server
```bash
python3 gazebo_frame_server.py
```

## Docker + WSL2 Networking

When running in Docker on WSL2, you need to use the WSL host IP instead of `172.17.0.1` or `host.docker.internal`. See the [Quick Start for Docker + WSL](#quick-start-for-docker--wsl) section below for detailed setup instructions.

### Quick Start for Docker + WSL

#### Step 1: Find WSL Host IP

```bash
# On WSL host:
hostname -I
# Example output: 172.19.173.99

# Remember the first IP!
```

#### Step 2: Start Servers on WSL Host

```bash
# Terminal 1 - video stream (if using TCP stream):
python3 gazebo_frame_server.py

# Terminal 2 - results (UDP):
python3 mock_server.py

# OR for TCP results:
python3 mock_server_tcp.py
```

#### Step 3: In Docker Container

```bash
# Use IP from Step 1
WSL_IP=172.19.173.99

# TCP video + UDP results:
./build/bin/dd_shahed_x86_64 \
  ./models/yolov11_shahed_640s_fp16_x86.engine \
  tcp://$WSL_IP:5001 \
  --conf=0.25 \
  --show=show \
  --server-host=$WSL_IP \
  --server-port=5000 \
  --protocol=udp

# TCP video + TCP results:
./build/bin/dd_shahed_x86_64 \
  ./models/yolov11_shahed_640s_fp16_x86.engine \
  tcp://$WSL_IP:5001 \
  --conf=0.25 \
  --show=show \
  --server-host=$WSL_IP \
  --server-port=5000 \
  --protocol=tcp

# Video file + TCP results:
./build/bin/dd_shahed_x86_64 \
  ./models/yolov11_shahed_640s_fp16_x86.engine \
  ./test_data/one_shahed_river_and_trees.mp4 \
  --conf=0.25 \
  --show=show \
  --server-host=$WSL_IP \
  --server-port=5000 \
  --protocol=tcp

# Camera with custom size and rotation:
./build/bin/dd_shahed_x86_64 \
  ./models/yolov11_shahed_640s_fp16_x86.engine \
  cam \
  --conf=0.25 \
  --show=show \
  --server-host=$WSL_IP \
  --server-port=5000 \
  --protocol=udp \
  --camera-width=640 \
  --camera-height=640 \
  --rotate180=1 \
  --detect-every=2
```

## Troubleshooting

### Error: Cannot connect to TCP frame server
```
[TCP-FRAME ERROR] connect() failed: Connection refused
```
**Solution:** Start `python3 gazebo_frame_server.py` on WSL host

---

### Error: UDP packets not reaching host
```
[UDP STATS] Packets sent: 100, Failed: 0
```
But nothing arrives on the server.

**Cause:** Docker Desktop on WSL doesn't route UDP to `172.17.0.1` or `host.docker.internal`

**Solution:**
1. On WSL host run: `hostname -I` (you'll get IP like 172.19.173.99)
2. Use this IP address in Docker container
3. Add firewall rule: `sudo iptables -I INPUT -p udp --dport 5000 -j ACCEPT`
4. Use helper script: `./get_host_ip.sh`

---

### Error: TCP results not reaching host
```
[SOCKET-TCP ERROR] connect() failed: Connection refused
```
**Solution:**
1. Start `python3 mock_server_tcp.py` on WSL host
2. Check IP: `hostname -I`
3. Firewall: `sudo iptables -I INPUT -p tcp --dport 5000 -j ACCEPT`

---

### Error: Invalid frame size with TCP stream
```
[TCP-FRAME ERROR] Invalid frame size: 808464432
[TCP-FRAME DEBUG] Received size bytes: 30 30 30 30 -> size=808464432
```
**Cause:** Connecting to wrong port or server with different protocol

**Solution:**
1. Verify that `gazebo_frame_server.py` is running on port 5001
2. Use correct port: `tcp://IP:5001` (not 11345 or other)
3. Debug output shows received bytes - if these are ASCII characters (0x30='0'), the protocol is incorrect

---

### Low FPS

**Performance Tips:**
- Use `--show=noshow` instead of `--show=show` (without GUI) - removes OpenCV display overhead
- Reduce JPEG quality in `gazebo_frame_server.py` - reduces network transfer time
- Use `--detect-every=2` or higher - skips detection on some frames (uses last results)
- Check GPU load: `nvidia-smi`

**Important Notes:**
- **Camera resolution (`--camera-width`, `--camera-height`) does NOT significantly affect FPS** because:
  - The model always resizes input to fixed size (640x640) before inference
  - Only affects frame reading/rotation overhead, not inference time
  - The bottleneck is GPU inference, not frame size
  
- **`--detect-every=N` may not give large FPS boost** because:
  - Frame reading (`cap.read()`) still happens every frame
  - Frame rotation, processing, and display still take time
  - Only skips the actual GPU inference, which may not be the main bottleneck
  - Useful when inference is the bottleneck, but not when I/O is the bottleneck

## Main Project Files

- **CMakeLists.txt** - CMake build configuration (automated cross-platform build)
- **src/main.cpp** - Main application logic
- **src/trt_engine.cpp** - TensorRT engine wrapper
- **include/tcp_frame_client.h** - TCP video stream client
- **scripts/convert_onnx_to_engine.py** - ONNX to TensorRT converter
- **scripts/mock_server_udp.py** - UDP results server for testing
- **scripts/mock_server_tcp.py** - TCP results server for testing

## Output Format

Detection results are sent as JSON via UDP or TCP:

```json
{
  "image_info": {
    "format": "opencv_bgr",
    "width": 1920,
    "height": 1080
  },
  "coordinates": {
    "x_min": -450.5,
    "y_min": -200.3,
    "x_max": -350.2,
    "y_max": -100.1,
    "center": [-400.35, -150.2]
  },
  "class_id": 0,
  "confidence": 0.87,
  "direction_vector": {
    "direction": [0.342, -0.198, 0.919],
    "magnitude": 458.3,
    "magnitude_normalized": 0.42
  }
}
```

**Coordinates are in center-origin system (image center = 0,0).**

## Telegram parser

### Settings example
Find Shaheds images and videos in TG chats

```json
{
    "api_id": "TG_API_ID",
    "api_hash": "TG_HASH",
    "download_path": "C:\\Users\\User\\Downloads\\TelegramMedia",
    "chats": [
        "news_channel_username",
        "https://t.me/durov",
        -1001234567890
    ],
    "keywords_regex": "(?i)(шахед|shahed|герань|мопед|дроны|дрон?)"
}
```

### Run command
```bash
python3 scripts/find_drone_images_tg.py --config "tg_parser_conf.json" --start 2025-12-01 --end 2025-12-04
```

## Requirements

- CMake 3.12 or later
- CUDA-capable GPU (Jetson Nano or NVIDIA RTX series)
- CUDA toolkit
- TensorRT 8.x or later
- OpenCV 4.x with development headers
- C++14 compatible compiler (g++ or clang++)
- Python 3.6+ (for helper scripts)
