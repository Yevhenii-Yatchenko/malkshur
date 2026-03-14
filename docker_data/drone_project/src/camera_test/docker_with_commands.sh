#!/bin/bash
#===============================================================================
# Docker Detection Wrapper Script (Bash + Expect)
#===============================================================================
# Automates Docker container launch with password entry and command execution
# using the 'expect' command for full automation.
#
# Usage:
#   ./camera_test/docker_with_commands.sh
#
# Requirements:
#   - expect command (install: sudo apt-get install expect)
#   - python3 for IP detection
#===============================================================================

# Configuration
DOCKER_SCRIPT="../jetson-inference/docker/run.sh"
PASSWORD="1234"

# Model configuration
MODEL="models/ONNXs/nv-v2-L1-98-E58-ssd-mobilenet.onnx"
LABELS="models/ONNXs/labels.txt"
CAMERA="csi://0"

# Detect host IP for Docker communication
# Note: host.docker.internal doesn't work on native Linux Docker
SERVER_HOST=$(python3 camera_test/get_host_ip.py 2>/dev/null || echo "172.17.0.1")
SERVER_PORT=5000

# Display configuration
echo "======================================================================"
echo "Docker Detection Wrapper (Bash + Expect)"
echo "======================================================================"
echo "[CONFIG] Host IP:          $SERVER_HOST"
echo "[CONFIG] Server port:      $SERVER_PORT"
echo "======================================================================"
echo

# Build detection command
COMMANDS="cd data/dd_smart_agent/
./ssd_mobilenet_v1_singel_camera.py \\
--model=$MODEL \\
--labels=$LABELS \\
--input-blob=input_0 \\
--output-cvg=scores \\
--output-bbox=boxes \\
--server_host=$SERVER_HOST \\
--server_port=$SERVER_PORT \\
$CAMERA"

# Check if expect is available
if command -v expect &> /dev/null; then
    echo "[INFO] Using 'expect' for automation"
    echo

    # Use expect for full automation
    expect << EOF
set timeout 30
spawn bash -c "cd ../jetson-inference && ./docker/run.sh"

# Handle password prompt
expect {
    "password" {
        send "$PASSWORD\r"
        exp_continue
    }
    "#" {
        # In Docker container - send commands
        send "$COMMANDS\r"
        interact
    }
    timeout {
        puts "ERROR: Timeout waiting for prompt"
        exit 1
    }
}
EOF

else
    # Expect not available - fallback to manual mode
    echo "[WARNING] 'expect' command not found"
    echo "[INFO] Install with: sudo apt-get install expect"
    echo
    echo "Falling back to manual mode..."
    echo "After entering password, run:"
    echo "$COMMANDS"
    echo
    echo "======================================================================"
    echo

    cd ../jetson-inference
    ./docker/run.sh
fi
