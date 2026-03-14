#!/bin/bash
set -e

cd /drone_project

echo "DroneProject container ready."
echo "  USE_GAZEBO: ${USE_GAZEBO:-not set}"
echo "  MAVLINK_HOST: ${MAVLINK_HOST:-not set}"
echo "  MAVLINK_PORT: ${MAVLINK_PORT:-not set}"
echo "  DRONE_CAMERA_TYPE: ${DRONE_CAMERA_TYPE:-not set}"
echo "  GAZEBO_HOST: ${GAZEBO_HOST:-not set}"
echo ""
echo "To start the controller: python3 xbee_process_com.py"
echo "Or use: ./docker.sh controller"

if [ "$1" = "controller" ]; then
    exec python3 xbee_process_com.py
elif [ "$1" = "sky_anchor" ]; then
    exec python3 sky_anchor/main.py
else
    exec "$@"
fi
