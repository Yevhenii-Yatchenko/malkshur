# Detection System - Quick Command Reference

## Telnet Commands (localhost:2323)

### Monitor (Detection Server)

```bash
# Start server
{"msg": "monitor,start"}
{"msg": "monitor,1"}

# Stop server
{"msg": "monitor,stop"}
{"msg": "monitor,0"}
```

### Recognition Client (Docker)

```bash
# Start Docker detection client
{"msg": "recognizeClient,start"}
{"msg": "recognizeClient,1"}

# Stop Docker client
{"msg": "recognizeClient,stop"}
{"msg": "recognizeClient,0"}
```

### Drone Control

```bash
# Arm & takeoff to 5m
{"msg": "arm,0"}

# Disarm (stops all detection systems)
{"msg": "arm,1"}

# Set altitude
{"msg": "setHeight,10"}

# Enable stabilization
{"msg": "stabilize"}

# Land
{"msg": "land"}
```

## Typical Workflow

```bash
# 1. Start detection server
{"msg": "monitor,start"}

# 2. Arm drone (auto takeoff to 5m)
{"msg": "arm,0"}

# 3. Start recognition
{"msg": "recognizeClient,start"}

# Now intercept mode will activate automatically when target detected
# with confidence > 75%

# 4. Stop everything
{"msg": "arm,1"}
```

## Environment Variables

```bash
# Override confidence threshold
export INTERCEPT_CONFIDENCE_THRESHOLD=0.8

# Override timeout
export INTERCEPT_TIMEOUT_SECONDS=5.0

# Override yaw gain
export INTERCEPT_YAW_GAIN=150

# Override Docker script (for hardware)
export DETECTION_DOCKER_SCRIPT_HARDWARE=/path/to/run-jetson.sh
```

## Quick Test

```bash
# Terminal 1: Start drone controller
USE_GAZEBO=true MAVLINK_HOST=127.0.0.1 MAVLINK_PORT=5763 \
SKY_ANCHOR_PATH=/mnt/d/WSL/Project/DroneProject/sky_anchor/main.py \
python3 xbee_process_com.py

# Terminal 2: Connect via telnet
telnet localhost 2323

# Terminal 3: Monitor logs (use convenience script)
./tail_detection_logs.sh

# Or manually:
tail -f logs/controller.log logs/detection_server.log logs/detection_client.log
```

## Monitoring Intercept Mode

### Controller Logs (logs/controller.log)

Watch for intercept mode activation:

```
[WARNING] INTERCEPT MODE ACTIVATED (confidence: 81.19%)
[DEBUG] Intercept: conf=81.19%, dir_x=+0.402, dir_y=+0.234, yaw=1540, pitch=1520
[WARNING] INTERCEPT MODE DEACTIVATED (no detection for 3.0s)
```

### Detection Server Logs (logs/detection_server.log)

Server will log incoming detections:

```
[2025-11-21 21:55:10.123] Client connected: 172.17.0.2:54321
[21:55:10] 📥 Detection from 172.17.0.2:54321 - class=1, conf=81.19%, dir_x=+0.402, dir_y=+0.234
[21:55:10] Full data: {"class_id":1,"confidence":0.8119,...}
```

### Detection Client Logs (logs/detection_client.log)

Docker container output with detection script progress:

```
======================================================================
Detection client started at 2025-11-21 21:55:05
======================================================================
[2025-11-21 21:55:05] Waiting for shell prompt...
[2025-11-21 21:55:06] Shell prompt detected
[2025-11-21 21:55:06] Sending commands...
======================================================================
Docker output below:
======================================================================
[docker output from tmp_gazebo.py here]
```

## Tuning Parameters

Edit `src/detection_config.py`:

```python
# Confidence threshold (0.0-1.0)
INTERCEPT_CONFIDENCE_THRESHOLD = 0.75

# Timeout without detection (seconds)
INTERCEPT_TIMEOUT_SECONDS = 3.0

# Deadband for direction vector
INTERCEPT_DEADBAND_X = 0.1  # horizontal
INTERCEPT_DEADBAND_Y = 0.1  # vertical

# Control gains
INTERCEPT_YAW_GAIN = 100        # PWM per unit
INTERCEPT_ALTITUDE_STEP = 0.1   # meters per cycle
INTERCEPT_PITCH_OFFSET = 20     # forward speed PWM
```
