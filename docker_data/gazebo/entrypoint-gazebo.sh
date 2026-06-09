#!/bin/bash
set -e

# Source Gazebo setup (sets GAZEBO_PLUGIN_PATH, GAZEBO_RESOURCE_PATH, etc.)
source /usr/share/gazebo/setup.bash

# Custom models and worlds (mounted from host)
export GAZEBO_MODEL_PATH=/root/gazebo_models:$GAZEBO_MODEL_PATH
export GAZEBO_RESOURCE_PATH=/root/gazebo_worlds:$GAZEBO_RESOURCE_PATH

# Default world
WORLD=/root/gazebo_worlds/iris_arducopter_cmac.world

if [ "$1" = "bash" ]; then
    exec bash
elif [ "$1" = "gazebo" ]; then
    shift
    WORLD=${1:-$WORLD}
    # Camera sensors need a GL context; without a host X display render
    # into a virtual framebuffer and skip the GUI.
    HEADLESS=0
    if [ -z "$DISPLAY" ]; then
        echo "No DISPLAY set - starting Xvfb for headless rendering"
        Xvfb :99 -screen 0 1280x1024x24 &
        export DISPLAY=:99
        HEADLESS=1
        sleep 1
    fi
    echo "Starting gzserver with world: $WORLD"
    # Launch gzserver (physics engine) as main process
    gzserver --verbose $WORLD &
    GZSERVER_PID=$!
    # Wait briefly for gzserver to initialize
    sleep 3
    if [ "$HEADLESS" = "0" ]; then
        # Launch gzclient (GUI) as separate process
        echo "Starting gzclient"
        gzclient &
    fi
    # Wait for gzserver to exit
    wait $GZSERVER_PID
else
    exec "$@"
fi
