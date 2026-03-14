#!/bin/bash

set -euo pipefail
trap 'echo "Docker build failed." >&2' ERR

echo "Building Docker image for Drone Controller..."

# Build using docker-compose
docker-compose build

echo "Docker image built successfully!"
echo ""
echo "To run the container, use: ./docker_run.sh"
