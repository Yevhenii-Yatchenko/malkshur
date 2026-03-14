# CLAUDE.md

## Project Overview

Quadcopter simulation project for autonomous drone development with vision-based stabilization. Integrates ArduPilot SITL with Gazebo simulation and a custom DroneProject controller using ORB feature matching for position hold.

**Core Stack:** Gazebo 11, ArduPilot SITL (ArduCopter), pygazebo, pymavlink, OpenCV, Docker

## Common Commands

All development uses Docker containers. Use the helper script:

```bash
./docker.sh up              # Build and start all containers
./docker.sh down            # Stop and remove containers
./docker.sh stop            # Pause containers
./docker.sh start           # Resume paused containers
./docker.sh ardupilot       # Run ArduPilot with interactive MAVProxy console
./docker.sh controller      # Start DroneProject controller
./docker.sh sky-anchor      # Start sky_anchor vision system
./docker.sh shell           # Bash into Gazebo container
./docker.sh shell-ardupilot # Bash into ArduPilot container
./docker.sh shell-drone     # Bash into DroneProject container
./docker.sh logs            # Follow all container logs
./docker.sh status          # Show container status
```

## Architecture

```
┌──────────────────────────────────────────────────────────────────┐
│                    Docker Compose (host network)                  │
├────────────────────┬────────────────────┬────────────────────────┤
│  malkshur-gazebo   │  malkshur-ardupilot│  malkshur-droneproject │
│  ├─ Gazebo 11      │  ├─ ArduCopter SITL│  ├─ DroneController    │
│  │  gzserver/client│  │  (gazebo-iris)   │  │  (pymavlink TCP)    │
│  ├─ Custom plugins │  └─ MAVProxy       │  ├─ SkyAnchor          │
│  │  ArduPilotPlugin│                    │  │  (pygazebo camera)   │
│  │  FollowerDownCam│  TCP: 5760 (SITL)  │  └─ TCP :8888 internal │
│  └─ Gazebo :11345  │  UDP: 9002/9003    │                        │
│                    │  UDP: 14550 (GCS)  │                        │
│  GPU: NVIDIA       │                    │                        │
└────────────────────┴────────────────────┴────────────────────────┘
```

**Data Flow:**
1. Gazebo spawns iris_with_ardupilot + down_cam models
2. ArduPilotPlugin communicates with ArduPilot SITL via UDP 9002/9003
3. DroneProject connects to ArduPilot via MAVLink TCP (port 5763)
4. SkyAnchor gets camera frames from Gazebo via pygazebo (port 11345)
5. ORB feature matching computes drift -> PID controller -> RC overrides

**No ROS.** DroneProject uses pygazebo (classic Gazebo transport) for camera and pymavlink for ArduPilot.

## Network Ports

| Port | Protocol | Service | Description |
|------|----------|---------|-------------|
| 9002 | UDP | ArduPilot FDM | Flight dynamics input from Gazebo |
| 9003 | UDP | ArduPilot FDM | Flight dynamics output to Gazebo |
| 5760 | TCP | ArduPilot SITL | Primary MAVLink endpoint |
| 5763 | TCP | ArduPilot SITL | Forwarded MAVLink (DroneProject connects here) |
| 11345 | TCP | Gazebo transport | Classic Gazebo transport (pygazebo camera) |
| 14550 | UDP | MAVLink | GCS connection (QGroundControl) |
| 8888 | TCP | SkyAnchor | Internal: vision commands to controller |

## Environment Variables

### ArduPilot Container

| Variable | Default | Description |
|----------|---------|-------------|
| `ARDUPILOT_VEHICLE` | `ArduCopter` | Vehicle type |
| `ARDUPILOT_FRAME` | `gazebo-iris` | Frame type |
| `ARDUPILOT_HEADLESS` | `0` | `1` = no console GUI |
| `ARDUPILOT_LOCATION` | `OSRF0` | Spawn location name |
| `ARDUPILOT_PARAMS` | (none) | Optional parameter file path |

### DroneProject Container

| Variable | Default | Description |
|----------|---------|-------------|
| `USE_GAZEBO` | `true` | Enable Gazebo simulation mode |
| `MAVLINK_HOST` | `localhost` | ArduPilot MAVLink host |
| `MAVLINK_PORT` | `5763` | ArduPilot MAVLink port |
| `DRONE_CAMERA_TYPE` | `GAZEBO` | Camera source (GAZEBO/USB/SCI) |
| `GAZEBO_HOST` | `localhost` | Gazebo transport host |
| `GAZEBO_CAMERA_TOPIC` | `/gazebo/default/down_cam/cam_link/nadir_camera/image` | Gazebo camera topic |

## Directory Structure

```
malkshur/
├── Dockerfiles/
│   ├── Dockerfile.gazebo
│   ├── Dockerfile.ardupilot
│   └── Dockerfile.droneproject
├── docker_data/
│   ├── gazebo/
│   │   ├── entrypoint-gazebo.sh
│   │   ├── CMakeLists.txt         # Plugin build config
│   │   ├── cmake/                 # CMake helpers
│   │   ├── src/                   # Plugin C++ source
│   │   ├── include/               # Plugin headers
│   │   ├── models/                # Gazebo models (iris variants, down_cam, etc.)
│   │   └── worlds/                # World files
│   ├── ardupilot_sitl/
│   │   ├── entrypoint-ardupilot.sh
│   │   ├── locations.txt          # Custom spawn locations
│   │   ├── params/                # ArduPilot parameter files
│   │   └── sitl_data/             # Persisted SITL state
│   └── drone_project/
│       ├── entrypoint-droneproject.sh
│       ├── src/                   # DroneProject source (full copy)
│       ├── requirements.txt       # Python dependencies
│       ├── fix_pygazebo.py        # Python 3.10+ compatibility
│       └── logs/                  # Persisted logs
├── docker-compose.yml
├── docker.sh                      # Helper script
└── CLAUDE.md
```

## Key Configuration Files

| File | Purpose |
|------|---------|
| `docker_data/drone_project/src/src/controller_config.py` | MAVLink connection, sensor config |
| `docker_data/drone_project/src/src/altitude_config.py` | PID tuning parameters |
| `docker_data/drone_project/src/sky_anchor/app/config.py` | Vision system config (ORB, thresholds) |
| `docker_data/ardupilot_sitl/params/*.parm` | ArduPilot vehicle parameters |
| `docker_data/gazebo/worlds/iris_arducopter_cmac.world` | Default simulation world |
| `docker_data/gazebo/models/iris_with_ardupilot/model.sdf` | Iris quadcopter model |
| `docker_data/gazebo/models/down_cam/model.sdf` | Downward-facing camera model |

## Gazebo Models

| Model | Description |
|-------|-------------|
| `iris_with_ardupilot` | Base iris quadcopter with ArduPilot plugin |
| `iris_with_standoffs` | Iris frame with standoff legs |
| `iris_with_standoffs_demo` | Scaled iris with gimbal and camera |
| `down_cam` | Follower camera (tracks iris, faces nadir) |
| `gimbal_small_2d` | 2D gimbal for camera mounting |
| `ground_plane_unique` | Ground with augmented textures for ORB tracking |
| `orb_optimized_ground` | Ground optimized for ORB feature detection |

## Custom Gazebo Plugins

| Plugin | File | Description |
|--------|------|-------------|
| ArduPilotPlugin | `src/ArduPilotPlugin.cc` | ArduPilot SITL integration (FDM, motors) |
| FollowerDownCamPlugin | `src/FollowerDownCamPlugin.cc` | Camera that follows iris model |
| GimbalSmall2dPlugin | `src/GimbalSmall2dPlugin.cc` | 2D gimbal tilt controller |
| ArduCopterIRLockPlugin | `src/ArduCopterIRLockPlugin.cc` | IR beacon precision landing |

## DroneProject Components

| Component | Path | Description |
|-----------|------|-------------|
| Controller | `src/controller.py` | Main flight control loop (100Hz) |
| MAVLink Manager | `src/mavlink_manager.py` | ArduPilot communication |
| SkyAnchor | `sky_anchor/` | Vision-based drift correction |
| ORB Estimator | `sky_anchor/app/vision/estimator.py` | ORB feature matching (CPU/CUDA) |
| Gazebo Bridge | `sky_anchor/gazebo_classic_bridge.py` | pygazebo camera interface |
| PID Controller | `src/pid_controller.py` | Cascade PID for altitude + position |
| Position Controller | `src/position_controller.py` | XY stabilization from camera drift |
