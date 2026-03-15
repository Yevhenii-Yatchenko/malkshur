# Malkshur

Dockerized quadcopter simulation for autonomous drone development. Integrates ArduPilot SITL with Gazebo and a vision-based flight controller.

## Architecture

Three Docker containers on host network:

```
┌──────────────────────────────────────────────────────────────────┐
│                    Docker Compose (host network)                  │
├────────────────────┬────────────────────┬────────────────────────┤
│  Gazebo 11         │  ArduPilot SITL    │  DroneProject          │
│  ├─ gzserver/client│  ├─ ArduCopter     │  ├─ DroneController    │
│  ├─ ArduPilotPlugin│  │  (gazebo-iris)   │  │  (pymavlink)        │
│  ├─ FollowerDownCam│  └─ MAVProxy       │  ├─ SkyAnchor          │
│  └─ Custom models  │                    │  │  (ORB stabilization) │
│                    │  UDP 9002/9003     │  └─ pygazebo camera     │
│  GPU: NVIDIA       │  TCP 5760 MAVLink  │     TCP 11345           │
└────────────────────┴────────────────────┴────────────────────────┘
```

**Gazebo** spawns an iris quadcopter with a downward-facing camera. **ArduPilot** runs ArduCopter SITL connected to Gazebo via the ArduPilot plugin. **DroneProject** controls the drone using pymavlink and stabilizes position using ORB feature matching on the nadir camera feed via pygazebo.

## Prerequisites

- Docker with Compose v2
- NVIDIA GPU + nvidia-container-toolkit
- X11 display (for Gazebo GUI)

## Quick Start

```bash
# Build and start all containers
./docker.sh up

# Start the flight controller (arms + stabilizes automatically)
./docker.sh controller

# Or use ArduPilot interactive console
./docker.sh ardupilot
```

## Commands

```bash
# Lifecycle
./docker.sh up              # Build and start
./docker.sh down            # Stop and remove
./docker.sh stop / start    # Pause / resume
./docker.sh rebuild         # Full rebuild

# Interactive
./docker.sh ardupilot       # MAVProxy console
./docker.sh controller      # DroneProject controller
./docker.sh shell           # Gazebo container shell
./docker.sh shell-ardupilot # ArduPilot container shell
./docker.sh shell-drone     # DroneProject container shell

# Monitoring
./docker.sh logs            # All logs
./docker.sh status          # Container status
```

## Custom Gazebo Plugins

| Plugin | Description |
|--------|-------------|
| ArduPilotPlugin | ArduPilot SITL integration (FDM, motors) |
| FollowerDownCamPlugin | Camera that follows the iris model |
| GimbalSmall2dPlugin | 2D gimbal tilt controller |
| ArduCopterIRLockPlugin | IR beacon precision landing |

## Configuration

**Steering/params:** Edit files in `docker_data/ardupilot_sitl/params/`

**World:** Change `WORLD=` in `docker_data/gazebo/entrypoint-gazebo.sh`

**DroneProject:** Environment variables in `docker-compose.yml` control MAVLink host/port, camera type, and auto-arming.

## Project Structure

```
malkshur/
├── Dockerfiles/                # Container definitions
│   ├── Dockerfile.gazebo
│   ├── Dockerfile.ardupilot
│   └── Dockerfile.droneproject
├── docker_data/
│   ├── gazebo/
│   │   ├── src/ include/       # Custom plugin source
│   │   ├── models/             # Gazebo models (iris, down_cam, etc.)
│   │   └── worlds/             # World files
│   ├── ardupilot_sitl/
│   │   ├── params/             # ArduPilot parameter files
│   │   └── locations.txt       # Custom spawn locations
│   └── drone_project/
│       └── src/                # DroneProject source code
├── docker-compose.yml
├── docker.sh                   # Helper script
└── CLAUDE.md                   # AI assistant context
```
