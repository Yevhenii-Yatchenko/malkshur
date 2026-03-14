# System Overview: Docker Detection with Server Communication

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                        HOST MACHINE                         │
│                                                             │
│  ┌───────────────────────────────────────────────────┐    │
│  │  Server (Python)                                  │    │
│  │  - Listens on 0.0.0.0:5000                       │    │
│  │  - Receives JSON detection data                   │    │
│  │  - Sends acknowledgment responses                 │    │
│  └───────────────────┬───────────────────────────────┘    │
│                      │                                      │
│                      │ TCP Socket (port 5000)              │
│                      │                                      │
└──────────────────────┼──────────────────────────────────────┘
                       │
                       │ Auto-detected IP:5000 (e.g., 192.168.X.X:5000)
                       │
┌──────────────────────┼──────────────────────────────────────┐
│                DOCKER CONTAINER                             │
│                      │                                      │
│  ┌───────────────────▼───────────────────────────────┐    │
│  │  Detection Client                                 │    │
│  │  - ssd_mobilenet_v1_singel_camera.py             │    │
│  │  - Captures frames from CSI camera                │    │
│  │  - Runs object detection (SSD MobileNet)          │    │
│  │  - Sends detection results to server              │    │
│  └───────────────────┬───────────────────────────────┘    │
│                      │                                      │
│  ┌───────────────────▼───────────────────────────────┐    │
│  │  CSI Camera (csi://0)                             │    │
│  │  - Hardware camera mounted on Jetson Nano         │    │
│  └───────────────────────────────────────────────────┘    │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

## Components

### 1. Server (Host Machine)

**File:** `camera_test/standalone_server.py` or part of `run_docker_detection_with_server.py`

**Purpose:** Receive detection results from Docker container

**Configuration:**
- Host: `0.0.0.0` (listens on all network interfaces)
- Port: `5000`
- Protocol: TCP with JSON messages

**Data Format (Received):**
```json
{
  "timestamp": 1699999999.123,
  "detections": [
    {
      "class": "person",
      "confidence": 0.95,
      "bbox": [x1, y1, x2, y2]
    }
  ],
  "frame_count": 42
}
```

**Response Format (Sent):**
```json
{
  "status": "ok",
  "timestamp": 1699999999.456,
  "message": "Data received successfully"
}
```

### 2. Docker Container

**Image:** `dustynv/jetson-inference:r32.7.1`

**Volumes Mounted:**
- `/jetson-inference/data` → Host's data directory
- Camera devices: `/dev/video0`, `/dev/video1`

**Network:**
- Auto-detects host IP address (Linux doesn't support `host.docker.internal`)
- Connects to detected host IP on port 5000 (e.g., 192.168.X.X:5000)
- Fallback: Docker bridge gateway (172.17.0.1) if detection fails

### 3. Detection Script (Inside Docker)

**File:** `data/dd_smart_agent/ssd_mobilenet_v1_singel_camera.py`

**Model:**
- Type: SSD MobileNet v1
- ONNX Format: `nv-v2-L1-98-E58-ssd-mobilenet.onnx`
- Labels: `labels.txt`

**Camera:**
- Input: `csi://0` (CSI camera on Jetson Nano)
- Can also use USB cameras: `v4l2://0`

**Command Line Arguments:**
```bash
--model=models/ONNXs/nv-v2-L1-98-E58-ssd-mobilenet.onnx
--labels=models/ONNXs/labels.txt
--input-blob=input_0
--output-cvg=scores
--output-bbox=boxes
--server_host=host.docker.internal    # Server hostname/IP
--server_port=5000                     # Server port
csi://0                                # Camera input
```

## Data Flow

1. **Camera Capture**
   - CSI camera captures frames at ~30 FPS
   - Frames passed to detection model

2. **Object Detection**
   - SSD MobileNet processes each frame
   - Detects objects with bounding boxes and confidence scores
   - Applies Non-Maximum Suppression (NMS)

3. **Data Transmission**
   - Detection results formatted as JSON
   - Sent via TCP socket to `host.docker.internal:5000`
   - Waits for server acknowledgment

4. **Server Reception**
   - Server receives JSON data
   - Parses and displays detection info
   - Sends acknowledgment back to client

5. **Continuous Loop**
   - Process repeats for each frame
   - Real-time detection and transmission

## Running the System

### Fully Automatic (with pexpect)

```bash
# Install pexpect
python3 -m pip install pexpect

# Run everything
python3 camera_test/run_docker_detection_with_server.py
```

**What happens:**
1. ✅ Server starts on port 5000
2. ✅ Docker container launches
3. ✅ Password entered automatically
4. ✅ Detection script executes
5. ✅ Data flows: Camera → Detection → Server

### Semi-Automatic (without pexpect)

```bash
# Run the script
python3 camera_test/run_docker_detection_with_server.py

# When Docker starts:
# 1. Enter password: 1234
# 2. Inside container, run:
/jetson-inference/data/auto_start_detection.sh
```

### Manual (Two Terminals)

**Terminal 1:**
```bash
python3 camera_test/standalone_server.py
```

**Terminal 2:**
```bash
cd ../jetson-inference/
./docker/run.sh
# Enter password: 1234
# Inside container:
cd data/dd_smart_agent/
./ssd_mobilenet_v1_singel_camera.py \
--model=models/ONNXs/nv-v2-L1-98-E58-ssd-mobilenet.onnx \
--labels=models/ONNXs/labels.txt \
--input-blob=input_0 \
--output-cvg=scores \
--output-bbox=boxes \
--server_host=<AUTO_DETECTED_IP> \
--server_port=5000 \
csi://0
```

## Testing the Server

Before running the full system, test if the server works:

```bash
# Terminal 1: Start server
python3 camera_test/standalone_server.py

# Terminal 2: Test connection
python3 camera_test/test_server_connection.py
```

Expected output:
```
✅ Successfully connected to localhost:5000
✅ Received response from server: {'status': 'ok', ...}
🎉 SUCCESS! Server is working correctly!
```

## Network Details

### Docker to Host Communication

**On Linux/Jetson (Native Docker):**

`host.docker.internal` doesn't exist on native Linux Docker (only Docker Desktop).

**Solution:** The scripts automatically detect your host's IP address.

**How it works:**
1. Uses socket connection to Google DNS (8.8.8.8) to determine outbound interface
2. Extracts the IP address from that interface
3. Uses that IP for `--server_host` parameter
4. Fallback to Docker bridge gateway (172.17.0.1) if detection fails

**Find host IP manually:**
```bash
python3 camera_test/get_host_ip.py
```

**Manual override:**
```bash
--server_host=192.168.X.X
```

### Port Configuration

- **Server Port:** 5000 (default, configurable)
- **Protocol:** TCP
- **Firewall:** Must allow incoming connections on port 5000

Check if port is open:
```bash
netstat -tuln | grep 5000
```

## Files Summary

| File | Purpose |
|------|---------|
| `run_docker_detection_with_server.py` | All-in-one automation script |
| `standalone_server.py` | Server-only script |
| `docker_with_commands.sh` | Bash wrapper using `expect` |
| `test_server_connection.py` | Test server connectivity |
| `auto_start_detection.sh` | Auto-generated startup script (in Docker) |
| `README.md` | Full documentation |
| `QUICKSTART.md` | Quick reference guide |
| `SYSTEM_OVERVIEW.md` | This file |

## Troubleshooting

See [QUICKSTART.md](QUICKSTART.md#troubleshooting) for detailed troubleshooting steps.

Quick checks:
1. Is server running? `netstat -tuln | grep 5000`
2. Can you connect? `python3 camera_test/test_server_connection.py`
3. Is Docker running? `docker ps`
4. Is camera accessible? Check `/dev/video0` inside container
5. Check Docker output for errors

## Next Steps

1. **Integrate with Drone System**: Send detection data to drone controller
2. **Add Object Tracking**: Track detected objects across frames
3. **Implement Actions**: React to specific detections (e.g., follow person)
4. **Add Logging**: Save detection history to database
5. **Web Interface**: Display detections in real-time web dashboard
