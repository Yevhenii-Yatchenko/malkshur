#!/bin/bash

# Script to run the drone controller in Docker

echo "Starting Drone Controller Docker container..."
echo "========================================"
echo ""
echo "Prerequisites:"
echo "1. Gazebo should be running: gazebo --verbose iris_arducopter_runway.world"
echo "2. ArduPilot SITL should be running: sim_vehicle.py -v ArduCopter -f gazebo-iris --console"
echo ""
echo "Press Enter to continue or Ctrl+C to cancel..."
read

# Check if container is already running
if [ "$(docker ps -q -f name=drone_controller)" ]; then
    echo "Container 'drone_controller' is already running. Stopping it..."
    docker compose down
fi

# Copy environment file
#cp .env.docker sky_anchor/.env

# Start the container
echo "Starting container..."
docker compose up -d

echo ""
echo "Container started successfully!"
echo ""
echo "Available connections:"
echo "- SSH: ssh -p 2222 drone@localhost (password: drone)"
echo "- Telnet: telnet localhost 2323"
echo ""
echo "To run the drone controller inside the container:"
echo "1. SSH into the container"
echo "2. Run: python3 src/controller_sim.py"
echo ""
echo "To run sky_anchor with Gazebo camera:"
echo "1. SSH into the container"
echo "2. Run: python3 sky_anchor/main.py"
echo ""
echo "To view logs:"
echo "- docker-compose logs -f"
echo ""
echo "To stop the container:"
echo "- docker-compose down"
