# Server Control Demo

## Overview

This script demonstrates **server lifecycle control** while the Docker detection client keeps running. It shows that the server can be stopped and restarted at any time, and the client will automatically reconnect when the server becomes available again.

## What It Does

The script orchestrates a timeline-based demonstration:

1. **T=0s**: Start Docker client (detection runs, but no server yet)
2. **T=10s**: Start server → client connects and sends data
3. **T=15s**: Stop server → client loses connection, keeps retrying
4. **T=20s**: Start server again → client reconnects automatically
5. **T=30s**: Stop server and client → everything shuts down

## Key Features

### Automatic Logging

All output is logged to files for later analysis:

- **`camera_test/logs/docker_client.log`** - Complete Docker container output including:
  - Container initialization
  - Detection script startup
  - Camera feed status
  - Connection attempts and retries
  - Detection results

- **`camera_test/logs/server.log`** - Server activity including:
  - Server start/stop events
  - Client connections
  - Received detection data (full JSON)
  - Errors and warnings

### Clean Console Output

The console shows only essential information:
- Step progression with countdowns
- Server status changes
- Simplified detection notifications
- Log file locations

This keeps the terminal readable while comprehensive logs are saved to files.

## Architecture

### Components

#### 1. DetectionServer Class

A controllable TCP server that can be started and stopped multiple times:

```python
server = DetectionServer(host='0.0.0.0', port=5000, log_file='server.log')

# Control methods
server.start()        # Start in background thread
server.stop()         # Stop gracefully
server.is_running()   # Check status
```

Features:
- Runs in background thread
- Handles multiple simultaneous clients
- Logs all activity with timestamps
- Graceful shutdown with socket cleanup

#### 2. DockerDetectionRunner Class

Manages Docker container execution with automation:

```python
runner = DockerDetectionRunner(
    script_path='../jetson-inference/docker/run.sh',
    password='1234',
    log_file='docker_client.log'
)

# Run Docker with commands
process = runner.run(detection_commands)

# Stop logging
runner.stop()
```

Features:
- Automated password entry using pexpect
- Executes commands inside Docker container
- Continuous output logging to file
- Background thread for non-blocking operation

### Data Flow

```
┌─────────────────────────────────────────────────────────────┐
│ Docker Container (jetson-inference)                         │
│                                                               │
│  ┌─────────────────────────────────────────────────────┐   │
│  │ Detection Script (ssd_mobilenet_v1_singel_camera)   │   │
│  │   - Reads from CSI camera                           │   │
│  │   - Runs object detection                           │   │
│  │   - Sends results via TCP to host                   │   │
│  └─────────────────────────────────────────────────────┘   │
│                              │                               │
└──────────────────────────────┼───────────────────────────────┘
                               │ TCP (192.168.0.171:5000)
                               ↓
┌─────────────────────────────────────────────────────────────┐
│ Host Machine                                                 │
│                                                               │
│  ┌─────────────────────────────────────────────────────┐   │
│  │ DetectionServer                                      │   │
│  │   - Listens on 0.0.0.0:5000                         │   │
│  │   - Receives detection JSON                         │   │
│  │   - Logs to server.log                              │   │
│  │   - Sends acknowledgment                            │   │
│  └─────────────────────────────────────────────────────┘   │
│                                                               │
└─────────────────────────────────────────────────────────────┘
```

### Configuration

All settings are centralized at the top of the script:

```python
# Server configuration
SERVER_HOST_LISTEN = '0.0.0.0'  # Listen on all interfaces
SERVER_PORT = 5000              # TCP port

# Docker configuration
DOCKER_RUN_SCRIPT = "../jetson-inference/docker/run.sh"
DOCKER_PASSWORD = "1234"

# Detection model
DETECTION_MODEL = "models/ONNXs/nv-v2-L1-98-E58-ssd-mobilenet.onnx"
DETECTION_LABELS = "models/ONNXs/labels.txt"
CAMERA_INPUT = "csi://0"  # CSI camera

# Logging
LOG_DIR = "camera_test/logs"
DOCKER_LOG = "camera_test/logs/docker_client.log"
SERVER_LOG = "camera_test/logs/server.log"

# Demo timeline (in seconds)
DEMO_TIMELINE = {
    'wait_before_server_start': 10,
    'server_run_before_stop': 5,
    'wait_before_server_restart': 5,
    'server_run_before_final_stop': 10
}
```

You can easily adjust the timeline by modifying the `DEMO_TIMELINE` dictionary.

## Usage

### Prerequisites

```bash
# Install pexpect (required for automation)
python3 -m pip install pexpect
```

### Running the Demo

```bash
python3 camera_test/run_docker_detection_with_server_with_control.py
```

### Monitoring Logs in Real-Time

Open another terminal and watch the logs:

```bash
# Watch Docker output
tail -f camera_test/logs/docker_client.log

# Watch server output
tail -f camera_test/logs/server.log

# Watch both simultaneously
tail -f camera_test/logs/*.log
```

## Expected Output

### Console Output

```
======================================================================
Docker Detection with Server Control Demo
======================================================================
[LOG] Docker output → camera_test/logs/docker_client.log
[LOG] Server output → camera_test/logs/server.log

[CONFIG] Host IP: 192.168.0.171
[CONFIG] Server will listen on: 0.0.0.0:5000
[CONFIG] Docker will connect to: 192.168.0.171:5000

======================================================================
STEP 1: Starting Docker client (detection will run but no server yet)
======================================================================

Launching Docker container...
[INFO] Docker output is being logged to: camera_test/logs/docker_client.log
✅ Docker client started
   (Client is running but will fail to connect since no server yet)
   (Watch logs: tail -f camera_test/logs/docker_client.log)

======================================================================
STEP 2: Waiting before starting server...
======================================================================

   Starting server in 10 seconds...
   Starting server in 9 seconds...
   ...
🚀 Starting server on 0.0.0.0:5000...
[SERVER] Listening on 0.0.0.0:5000
✅ Server started
   (Client will now connect and send data)

[12:34:56] 📥 Data from 192.168.0.171:35421
    Detection: class=1, conf=0.89

======================================================================
STEP 3: Running with server, then stopping server...
======================================================================

   Stopping server in 5 seconds...
   ...
🛑 Stopping server...
✅ Server stopped
   (Client will lose connection and retry)

======================================================================
STEP 4: Waiting before restarting server...
======================================================================

   Starting server in 5 seconds...
   ...
🚀 Starting server on 0.0.0.0:5000...
[SERVER] Listening on 0.0.0.0:5000
✅ Server started
   (Client will reconnect automatically)

[12:35:15] 📥 Data from 192.168.0.171:35543
    Detection: class=1, conf=0.75

======================================================================
STEP 5: Running with server, then stopping everything...
======================================================================

   Stopping in 10 seconds...
   ...
🛑 Stopping server...
✅ Server stopped

🛑 Stopping Docker client...
✅ Docker client stopped
======================================================================
Demo Complete!
======================================================================

What you saw:
  1. ✅ Client started (no server)
  2. ✅ Server started → client connected
  3. ✅ Server stopped → client lost connection
  4. ✅ Server started → client reconnected
  5. ✅ Everything stopped

This demonstrates:
  • Server can be stopped and restarted while client runs
  • Client automatically reconnects when server comes back
  • Full control over server lifecycle

Check logs for detailed output:
  • Docker: camera_test/logs/docker_client.log
  • Server: camera_test/logs/server.log
======================================================================
```

### Log File Contents

**docker_client.log** - Shows everything happening inside Docker:
```
=== Docker Client Log - Started at 2025-01-08 12:34:00 ===
[2025-01-08 12:34:02] Password sent
[2025-01-08 12:34:03] Commands sent:
cd data/dd_smart_agent/
./ssd_mobilenet_v1_singel_camera.py ...

jetson.utils -- PyTorch version: 1.10.0
...
jetson.inference -- loading model from: models/ONNXs/nv-v2-L1-98-E58-ssd-mobilenet.onnx
...
[gstreamer] initialized gstreamer, version 1.14.5.0
...
video source -- opening csi://0
...
Error sending detection info: [Errno 111] Connection refused  # Before server starts
...
Sent detection info  # After server starts
...
Error sending detection info: [Errno 111] Connection refused  # After server stops
...
Sent detection info  # After server restarts
```

**server.log** - Shows all server activity:
```
=== Server Log - Started at 2025-01-08 12:34:00 ===
[2025-01-08 12:34:10.123] Listening on 0.0.0.0:5000
[2025-01-08 12:34:11.456] Received from 192.168.0.171:35421: {"class_id": 1, "class_name": "object", "confidence": 0.89, ...}
[2025-01-08 12:34:12.789] Received from 192.168.0.171:35421: {"class_id": 1, "class_name": "object", "confidence": 0.91, ...}
...
```

## Use Cases

### 1. Debugging

- Start client with detection running
- Start/stop server to capture specific events
- No need to restart Docker each time
- Examine logs to understand client behavior

### 2. Testing

- Test client behavior when server is unavailable
- Verify automatic reconnection logic
- Simulate network failures
- Stress test connection handling

### 3. Development

- Modify server code and restart without affecting client
- Test different server implementations
- Debug server issues in isolation
- Validate data format and protocol

### 4. Production Scenarios

- Rolling server updates (restart server, client stays up)
- Server maintenance windows
- Multi-client setups (one server, multiple clients)
- Graceful degradation testing

## Code Structure

The refactored code follows clean architecture principles:

### Utility Functions
- `get_host_ip()` - Auto-detect host IP for Docker communication
- `get_jetson_inference_dir()` - Resolve jetson-inference directory
- `build_detection_command()` - Generate detection script command
- `setup_logging()` - Initialize log files
- `print_step()` - Format step headers
- `countdown()` - Display countdown timers

### Classes
- `DetectionServer` - Controllable TCP server with logging
- `DockerDetectionRunner` - Docker automation with pexpect

### Main Function
- `run_control_demo()` - Orchestrates the entire demonstration

All configuration is centralized at the module level for easy customization.

## Troubleshooting

### Server won't start

**Problem**: Server returns failure on start

**Solutions**:
- Check if port 5000 is already in use: `lsof -i :5000`
- Verify firewall allows incoming connections
- Check server.log for error messages

### Docker client not connecting

**Problem**: Client shows connection errors even when server is running

**Solutions**:
- Verify IP address detection is correct (check console output)
- Ensure host IP is reachable from inside Docker container
- Check Docker network configuration
- Examine docker_client.log for detailed error messages

### No output in log files

**Problem**: Log files are created but remain empty

**Solutions**:
- Ensure pexpect is installed: `pip3 show pexpect`
- Check file permissions on log directory
- Verify Docker script path is correct
- Look for Python exceptions in console output

### Detection script not running

**Problem**: Docker starts but detection doesn't begin

**Solutions**:
- Check docker_client.log for errors
- Verify model files exist in Docker container
- Ensure camera is accessible (CSI or USB)
- Check if detection script has execute permissions

## Related Files

- [run_docker_detection_with_server.py](run_docker_detection_with_server.py) - Normal operation mode (no control demo)
- [README.md](README.md) - Complete documentation
- [SYSTEM_OVERVIEW.md](SYSTEM_OVERVIEW.md) - Architecture details
