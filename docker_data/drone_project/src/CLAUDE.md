# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a drone control system that integrates MAVLink-based flight control with a computer vision stabilization system. The project consists of:

- A main drone controller that communicates via MAVLink protocol
- A "sky_anchor" stabilization system that uses computer vision to detect and correct drift
- XBee-based remote communication
- LIDAR sensor integration for altitude measurement
- Battery monitoring system for safe operation

## Key Commands

### Running the System

```bash
# Run the main drone controller
# (xbee_process_com.py remains as a backward-compat shim used by the Docker entrypoints)
python3 run_controller.py

# Run the sky_anchor stabilization system
python3 sky_anchor/main.py

# Install dependencies
pip install -r requirements.txt

# For development (optional)
pip install -r dev-requirements.txt
```

### Testing

The unit/characterization suite lives under `tests/unit/` (395 tests:
command parsing, typed wire formats, flight behaviors, composition-root
wiring, and PID math characterized against the golden JSON snapshots in
`tests/unit/data/` -- do not regenerate those when refactoring). Run it
with pytest:

```bash
# From the host, in the DroneProject container:
docker exec malkshur_droneproject python3 -m pytest tests/unit/ -q

# Or inside the container (/drone_project):
python3 -m pytest tests/unit/ -q
```

#### Example/Hardware Tests

```bash
# Run example MAVLink test
python3 examples/mavtest.py

# Test camera functionality
python3 examples/jetson_camera_test.py
python3 examples/simple_camera.py
```

## Architecture Overview

### Control Systems

The drone uses **two independent cascade PID control systems**:

#### 1. Altitude Control System (Z-axis)

Maintains target altitude using sensor feedback:

1. **Outer Loop (Position)**: Controls altitude by generating velocity setpoints
2. **Inner Loop (Velocity)**: Controls vertical velocity by adjusting throttle
3. **Sensor Filtering**: Exponential moving average filter reduces sensor noise
4. **Adaptive Gains**: PID gains can be adjusted based on altitude for optimal performance

Key files:
- `src/pid_controller.py`: AltitudeController class with cascade PID
- `src/altitude_config.py`: Tuning parameters and configuration

#### 2. Position Control System (X/Y-axis)

Maintains horizontal position using sky_anchor vision feedback:

1. **Outer Loop (Position)**: Converts pixel drift (dx/dy) to velocity setpoints
2. **Inner Loop (Velocity)**: Converts velocity error to roll/pitch PWM commands
3. **Altitude Compensation**: Scales pixel measurements based on current altitude
4. **Confidence Filtering**: Uses match percentage to weight corrections

Key files:
- `src/position_controller.py`: PositionController class with cascade PID
- `src/position_config.py`: Tuning parameters and configuration

### Main Controller (`src/controller.py`)

The `DroneController` class is the primary flight control orchestrator that manages:
- MAVLink connection to the flight controller (autopilot) - TCP for Gazebo, USB for hardware
- Remote command interface via Telnet server (port 2323)
- Command handling for flight modes, arming, takeoff, movement, and landing
- LIDAR sensor (hardware) or barometer (Gazebo) for altitude measurement
- Battery monitoring for safe operation (hardware only)
- Integration with sky_anchor stabilization system via TCP client
- Signal handling for graceful shutdown
- Two simultaneous cascade PID control systems (altitude + position)

Key components:
- `LidarSensor` or barometer: Altitude measurement at configured rate
- `BatteryMonitor`: Monitors battery voltage and current (hardware only)
- `SkyAnchorClient`: Non-blocking TCP client connecting to localhost:8888
- `TelnetServer`: Remote debugging and command interface (port 2323)
- `AltitudeController`: Cascade PID for Z-axis control
- `PositionController`: Cascade PID for X/Y-axis control using vision feedback

Key methods:
- `_set_mode()`: Changes flight modes via MAVLink
- `_setHeight()`: Sets target altitude for altitude controller
- `_armingDisarming()`: Arms/disarms the drone
- `_updateThrottle()`: **Core control method** - runs both PID controllers and sends RC commands
- `_rubStabilizerScript()`: Spawns sky_anchor subprocess
- `_connectToStabilizer()`: Establishes TCP connection to sky_anchor
- `loop()`: Main control loop (typically 50Hz) - orchestrates all control operations

### Sky Anchor Stabilization (`sky_anchor/`)

**Computer vision-based drift detection system** that provides real-time position corrections.

**Purpose**: Detects horizontal drift (dx, dy) and rotation (angle) using ORB feature matching between a reference frame and current camera frames.

**Architecture** (completely refactored with clean separation of concerns):

Key components:
- `main.py`: Entry point that creates and runs the Controller
- `app/controller.py`: Main control loop orchestrating the vision pipeline
- `app/frame/provider.py`: **Thread-safe camera frame management** with dedicated capture thread
- `app/frame/camera.py`: Camera initialization for USB/CSI/Gazebo cameras
- `app/vision/parser.py`: ORB feature extraction (keypoints + descriptors)
- `app/vision/estimator.py`: Shift estimation using feature matching and homography (CPU/CUDA)
- `app/vision/evaluator.py`: Applies thresholds and creates commands
- `app/command_publisher.py`: Publishes shift commands via TCP server
- `app/sky_anchor_server.py`: Multi-threaded TCP server on localhost:8888
- `app/config.py`: Environment variable configuration

**Vision Pipeline**:
1. **Frame Capture** (`FrameProvider`): Continuous capture in background thread
2. **Preprocessing**: Grayscale conversion and normalization (configurable modes)
3. **Feature Extraction** (`ImageParser`): ORB detection and description
4. **Shift Estimation** (`ShiftEstimator`): Feature matching → Homography → dx/dy/angle
5. **Evaluation** (`ShiftEvaluator`): Apply deadband thresholds
6. **Publishing** (`CommandPublisher`): Broadcast to TCP clients

### Communication System

#### Flight Controller Communication
- **Protocol**: MAVLink v2.0
- **Hardware Mode**: USB serial (VID=1a86, PID=7523, 57600 baud)
- **Gazebo Mode**: TCP socket (host.docker.internal:5763, with fallback ports)
- **Messages**: RC channel overrides (roll, pitch, throttle, yaw)

#### Sky Anchor Communication
- **Protocol**: TCP/IP with JSON messages
- **Server**: `SkyAnchorServer` on localhost:8888 (multi-threaded, up to 100 clients)
- **Client**: `SkyAnchorClient` (non-blocking with dedicated receive thread)
- **Message Format**:
```json
{
  "dx": 12,              // Horizontal drift in pixels
  "dy": -5,              // Vertical drift in pixels
  "angle_deg": 2,        // Rotation in degrees
  "matches_percent": 75, // Feature match confidence (0-100)
  "timestamp": 1234567890.123
}
```

#### Remote Control Interface
- **Telnet Server**: Port 2323 for remote debugging and commands
- **Commands**: JSON format `{"msg": "command,param1,param2"}`
- Examples: `mode,GUIDED`, `arm,0`, `setHeight,5`, `stabilize`, `land`

## System Integration & Data Flow

### Process Architecture

The system runs as **two separate processes** communicating via TCP:

```
┌─────────────────────────────────────────────────────────────────────┐
│ MAIN CONTROLLER PROCESS                                             │
│ (run_controller.py → DroneController)                               │
│                                                                       │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │ Main Control Loop (50 Hz typical)                           │   │
│  │  1. Read altitude from sensor                               │   │
│  │  2. Get dx/dy/angle from SkyAnchorClient.tick()             │   │
│  │  3. Run PositionController.update() → roll/pitch/yaw PWM    │   │
│  │  4. Run AltitudeController.update() → throttle PWM          │   │
│  │  5. Send RC commands via MAVLink                            │   │
│  │  6. Process Telnet commands                                 │   │
│  │  7. Log to CSV                                              │   │
│  └─────────────────────────────────────────────────────────────┘   │
│            ↕ MAVLink                    ↕ TCP Client                │
└────────────┼───────────────────────────┼─────────────────────────────┘
             │                            │ localhost:8888
    ┌────────▼────────┐                  │
    │ Flight          │                  │
    │ Controller      │                  │
    │ (ArduPilot)     │                  │
    └─────────────────┘                  │
                                          │
┌────────────────────────────────────────┼─────────────────────────────┐
│ SKY ANCHOR PROCESS                      │                             │
│ (sky_anchor/main.py → Controller)       │                             │
│                                          ↓ TCP Server                 │
│  ┌─────────────────────────────────────────────────────────────┐    │
│  │ Vision Loop (30+ Hz)                                        │    │
│  │  1. FrameProvider.capture_current() → ParsedImage           │    │
│  │  2. ShiftEvaluator.evaluate(ref, current) → ShiftCommand    │    │
│  │  3. CommandPublisher.publish(command) → broadcast via TCP   │    │
│  └─────────────────────────────────────────────────────────────┘    │
│            ↕ USB/CSI                                                  │
└────────────┼─────────────────────────────────────────────────────────┘
             │
    ┌────────▼────────┐
    │ Camera          │
    │ (USB/CSI)       │
    └─────────────────┘
```

### Data Flow in Main Controller

**Control Loop Execution** (in `DroneController.loop()` / `__updateThrottle`;
steps 1-2 run inside `StabilizationBehavior.update()`,
`src/flight/stabilization.py`):

1. **Stabilizer Data Reception** (typed, de-duplicated):
   ```python
   reading = self.__stabilizer.poll_new()  # Optional[StabilizerReading]
   # StabilizerManager hands each reading out at most once (producer
   # timestamp de-dup); StabilizerReading (src/domain/types.py) is the
   # frozen dataclass that owns the :8888 wire format
   ```

2. **Position Control** (X/Y stabilization):
   ```python
   pid_output = self.__position_controller.update(
       dx_pixels=reading.dx,
       dy_pixels=reading.dy,
       angle_deg=0,
       confidence=reading.confidence,  # matches_percent / 100.0
       altitude=current_altitude,
       target_dx_pixels=reading.target_dx_pixels,
       target_dy_pixels=reading.target_dy_pixels,
       navigation=reading.navigation,
   )
   return AttitudeSetpoints(roll_pwm=pid_output['roll_pwm'],
                            pitch_pwm=pid_output['pitch_pwm'],
                            yaw_pwm=pid_output['yaw_pwm'])
   ```
   ...which `DroneController` applies to the RC state owner:
   ```python
   self.__rc.apply(setpoints)  # RCSetpoints owns the RC PWM bases (1000-2000)
   ```

3. **Altitude Control** (Z stabilization):
   ```python
   new_throttle = self.__altitude_controller.update(
       target_altitude=self.__rc.target_altitude,
       current_altitude=current_altitude
   )
   self.__rc.throttle = new_throttle  # upper-clamped at THROTTLE['max']
   ```

4. **RC Command Transmission**:
   ```python
   self.__mavlink.send_rc_override(
       roll=self.__rc.roll, pitch=self.__rc.pitch,
       throttle=self.__rc.throttle, yaw=self.__rc.yaw
   )
   ```

### Data Flow in Sky Anchor

**Vision Pipeline Execution** (in `Controller.run()`):

1. **Frame Capture**:
   ```python
   current_image = self.frame_provider.capture_current()
   # Returns ParsedImage with keypoints + descriptors
   ```

2. **Shift Evaluation**:
   ```python
   command = self.shift_evaluator.evaluate(reference, current_image)
   # Returns ShiftCommand(dx, dy, angle_deg, matches_percent)
   ```

3. **Publishing**:
   ```python
   self.command_publisher.publish(command)
   # Broadcasts JSON to all TCP clients
   ```

### PID Control Cascade

Both position and altitude controllers use **dual-loop cascade PID**:

**Position Controller (X-axis example)**:
```
dx_pixels (from vision)
    ↓
[Outer Loop: Position PID]
    kp=2.0, ki=0.1, kd=0.5
    ↓
velocity_setpoint (pixels/sec)
    ↓
[Inner Loop: Velocity PID]
    kp=1.0, ki=0.05, kd=0.1
    ↓
roll_pwm (1000-2000)
```

**Altitude Controller**:
```
target_altitude - current_altitude
    ↓
[Outer Loop: Altitude PID]
    kp=1.5, ki=0.3, kd=0.8
    ↓
velocity_setpoint (m/s)
    ↓
[Inner Loop: Velocity PID]
    kp=100, ki=10, kd=20
    ↓
throttle_pwm (1000-2000)
```

## Configuration

### Operational Modes

The system supports two modes via `src/controller_config.py`:

**Hardware Mode** (`USE_GAZEBO=false`):
- MAVLink: USB (auto-detected via VID/PID)
- Altitude: LIDAR sensor (2-50cm precision)
- Battery: Monitoring enabled
- Camera: USB or CSI (Jetson Nano)
- Sky Anchor: `/home/jetson/Documents/DroneProject/sky_anchor/main.py`

**Gazebo Simulation Mode** (`USE_GAZEBO=true`):
- MAVLink: TCP (host.docker.internal:5763)
- Altitude: Barometer @ 100Hz
- Battery: Disabled
- Camera: Gazebo camera topic or USB
- Sky Anchor: `/drone_project/sky_anchor/main.py`

### Sky Anchor Configuration (.env file)

Create a `.env` file in `sky_anchor/` directory based on `.env.example`:

```bash
# Camera settings
DRONE_CAMERA_TYPE=USB  # or SCI for CSI camera
DRONE_CAMERA_INDEX=0
DRONE_CAPTURE_WIDTH=1280
DRONE_CAPTURE_HEIGHT=720

# Flight controller
DRONE_FC_TYPE=MAVLINK
DRONE_FC_DEVICE=/dev/ttyUSB0
DRONE_FC_BAUDRATE=57600

# Drift thresholds
DRONE_SHIFT_THRESHOLD=1.5
DRONE_ANGLE_THRESHOLD=1.5

# ORB features
DRONE_ORB_NFEATURES=1000
DRONE_DEBUG=True
```

## Logging

The project uses a custom unbuffered logging system:
- Logs are written to `logs/` directory
- Component-specific log files: `controller.log`, `drone.log`
- Debug images saved to `debug_logs/` when DEBUG_MODE=True
- Real-time writing with forced flush for immediate visibility

## Development Workflow

1. **Hardware Setup**: Ensure flight controller and XBee are connected via USB
2. **Environment**: Configure `.env` file for sky_anchor if using stabilization
3. **Testing**: Use example scripts to verify camera and MAVLink connectivity
4. **Running**: Start controller first, then sky_anchor if needed for stabilization

## Command Protocol

Commands are sent via XBee in format: `command,param1,param2,...`

Examples:
- `mode,GUIDED` - Set flight mode
- `arm,0` - Arm the drone
- `takeoff,10` - Takeoff to 10 meters
- `setHeight,5` - Set target altitude to 5 meters
- `square` - Start stabilization and fly in square pattern
- `land` - Initiate landing

## Important Notes

### Altitude Control Tuning

The altitude control system can be tuned by modifying parameters in `src/altitude_config.py`:

- **LIMITS**: Physical constraints (max velocity, acceleration)
- **THROTTLE**: PWM ranges and hover estimate
- **FILTERING**: Sensor noise filtering parameters

There is currently no offline simulation tool for testing new gains without
flying.

### System Requirements and Notes

- The system is designed for Jetson Nano but can run on other Linux systems
- Altitude is measured using LIDAR sensor for high precision
- LIDAR sensor requires 2-second initialization timeout on startup
- Battery monitoring ensures safe operation with voltage/current tracking
- The stabilization system requires good lighting and textured surfaces for ORB features
- All coordinates in the stabilization system are relative to the initial reference frame
- The altitude controller continuously adapts to maintain stable flight
- Performance metrics are logged every 100 control cycles for tuning analysis

## Data Logging and Analysis

The system includes comprehensive data logging capabilities:

### CSV Data Loggers
- `src/altitude_csv_logger.py`: Logs altitude control data for analysis
- `src/position_csv_logger.py`: Logs position and movement data

### Analysis Tools
- `plot_altitude_csv_data.py`: Visualize altitude control performance
- `plot_position_csv_data.py`: Analyze position tracking and stability

### Debug Images
- Sky anchor saves debug images in `debug_logs/` directory
- Useful for analyzing ORB feature matching and tracking performance

## Component Connection Summary

### How Components Connect

**Startup Sequence**:
1. `run_controller.py` calls `build_controller()` in `src/app.py` -- the
   composition root, the only place the object graph is constructed:
   - MAVLink connection to flight controller
   - Sensor (LIDAR or barometer), started
   - `StabilizerManager` with its `SkyAnchorClient` (localhost:8888)
   - `AltitudeController` and `PositionController` (cascade PIDs) with
     typed config objects and CSV loggers
   - `RCSetpoints`, detection server/client, flight behaviors
     (`InterceptGuidance`, `StabilizationBehavior`)
   - `CommandHandler` with its `TelnetServer` (port 2323)
2. `DroneController.__init__()` receives the pre-built collaborators and
   only registers commands, starts telnet processing, and arms the
   optional auto-arm timer
3. User sends `stabilize` command via Telnet
4. Controller spawns sky_anchor subprocess: `python3 sky_anchor/main.py`
5. `sky_anchor/main.py` creates `Controller` instance
6. Sky Anchor `Controller.__init__()` initializes:
   - `FrameProvider` (camera + capture thread)
   - `ShiftEvaluator` (vision processing)
   - `CommandPublisher` (starts TCP server on localhost:8888)
7. `SkyAnchorClient.connect()` succeeds, connection established

**Runtime Data Flow**:

```
Camera → FrameProvider → ImageParser → ShiftEstimator → ShiftEvaluator
             (capture)     (ORB)        (homography)     (thresholds)
                                                                ↓
                                                         CommandPublisher
                                                                ↓
                                                        SkyAnchorServer
                                                         (TCP:8888)
                                                                ↓
                                                         SkyAnchorClient
                                                                ↓
                                                      PositionController
                                                      (cascade PID)
                                                                ↓
                                                      roll/pitch/yaw PWM
                                                                ↓
LIDAR → AltitudeController → throttle PWM ──────────────→ RC Channels
(sensor)   (cascade PID)                                       ↓
                                                          MAVLink
                                                               ↓
                                                      Flight Controller
                                                         (ArduPilot)
```

**Key Interactions**:

1. **Vision → Position Control**:
   - Sky Anchor detects drift: `{dx: 10, dy: -5, angle_deg: 2}`
   - Publishes to TCP server
   - Main controller receives via `SkyAnchorClient.tick()`
   - `PositionController.update()` converts to roll/pitch PWM

2. **Sensor → Altitude Control**:
   - LIDAR measures altitude: `0.85m`
   - `AltitudeController.update(target=1.0, current=0.85)`
   - Returns throttle PWM: `1530`

3. **Combined Control → Flight Controller**:
   - Combine: `roll=1480, pitch=1520, throttle=1530, yaw=1500`
   - Send via MAVLink: `rc_channels_override_send()`
   - Flight controller applies commands to motors

## Recent Updates (2025)

**Major Architectural Changes**:
- **Refactored sky_anchor** with clean separation of concerns:
  - `FrameProvider` for thread-safe camera management
  - `ShiftEvaluator` + `ShiftEstimator` for vision processing
  - `CommandPublisher` abstraction for TCP server
- **Added PositionController**: Proper cascade PID for X/Y stabilization
- **Dual control system**: Altitude + Position controllers run simultaneously
- **Thread-safe frame capture**: Dedicated capture thread eliminates blocking
- **Gazebo simulation support**: Full TCP-based operation for SITL testing

**Hardware Improvements**:
- Replaced ultrasonic sensor with LIDAR for improved altitude precision (2-50cm)
- Added battery monitoring with voltage/current tracking (hardware only)
- Enhanced logging with comprehensive CSV data output for both altitude and position
- Added signal handling for graceful shutdown

**Configuration**:
- Added `controller_config.py` for hardware/Gazebo mode switching
- Environment-based configuration via `USE_GAZEBO` variable
- Automatic MAVLink connection detection (USB vs TCP)

## Recent Updates (2026)

**GRASP refactor** (behavior-preserving; PID math, configs, and wire
formats unchanged):
- **Composition root**: `build_controller()` in `src/app.py` is the only
  place the object graph is constructed; `DroneController` is a thin
  orchestrator receiving pre-built collaborators
- **Typed wire data**: `StabilizerReading` / `DetectionReading`
  (`src/domain/types.py`) replace raw-dict fishing for the sky_anchor and
  detection JSON payloads
- **Behaviors extracted**: the former `__updateThrottle` god method is
  split into `InterceptGuidance` and `StabilizationBehavior`
  (`src/flight/`); `RCSetpoints` is the single owner of the RC PWM bases
  and the throttle ceiling
- **Test suite**: 395 unit/characterization tests under `tests/unit/`
  (PID math pinned against golden JSON in `tests/unit/data/`); dead code
  quarantined under `legacy/`