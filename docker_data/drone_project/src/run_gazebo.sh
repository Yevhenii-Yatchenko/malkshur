#!/bin/bash
# Run drone controller in Gazebo simulation mode

export USE_GAZEBO=true
export MAVLINK_HOST=127.0.0.1
export MAVLINK_PORT=5763
export SKY_ANCHOR_PATH=/mnt/d/WSL/Project/DroneProject/sky_anchor/main.py

python3 /mnt/d/WSL/Project/DroneProject/xbee_process_com.py
