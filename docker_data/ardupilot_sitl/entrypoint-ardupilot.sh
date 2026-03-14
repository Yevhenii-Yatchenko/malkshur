#!/bin/bash
set -e

cd /ardupilot

# Default values
VEHICLE=${ARDUPILOT_VEHICLE:-ArduCopter}
FRAME=${ARDUPILOT_FRAME:-gazebo-iris}
HEADLESS=${ARDUPILOT_HEADLESS:-0}
PARAMS=${ARDUPILOT_PARAMS:-}
LOCATION=${ARDUPILOT_LOCATION:-OSRF0}

# Build command arguments
ARGS="-v $VEHICLE -f $FRAME -L $LOCATION"

# Add console if not headless
if [ "$HEADLESS" = "0" ]; then
    ARGS="$ARGS --console"
fi

# Add parameter file if specified
if [ -n "$PARAMS" ] && [ -f "$PARAMS" ]; then
    ARGS="$ARGS --add-param-file=$PARAMS"
    echo "  Params: $PARAMS"
fi

# Add any extra arguments passed to the container
if [ $# -gt 0 ]; then
    ARGS="$ARGS $@"
fi

echo "Starting ArduPilot SITL..."
echo "  Vehicle: $VEHICLE"
echo "  Frame: $FRAME"
echo "  Location: $LOCATION"
echo "  Headless: $HEADLESS"

exec python Tools/autotest/sim_vehicle.py $ARGS
