#!/bin/bash
# Get the correct WSL host IP address for network communication from Docker

echo "Finding WSL host IP address for Docker containers..."
echo ""

# Method 1: hostname -I (most reliable for WSL)
if command -v hostname &> /dev/null; then
    HOST_IP=$(hostname -I | awk '{print $1}')
    if [ -n "$HOST_IP" ]; then
        echo "✅ WSL Host IP (from hostname -I): $HOST_IP"
        echo ""
        echo "Use this IP in your Docker container:"
        echo ""
        echo "Example 1 - Video file with UDP results:"
        echo "  ./bin/yolo11_640s_fp16_infer \\"
        echo "    ./models/yolov11_shahed_640s_fp16_x86.engine \\"
        echo "    ./test_data/one_shahed_river_and_trees.mp4 \\"
        echo "    0.25 show $HOST_IP 5000 udp"
        echo ""
        echo "Example 2 - TCP video stream with TCP results:"
        echo "  ./bin/yolo11_640s_fp16_infer \\"
        echo "    ./models/yolov11_shahed_640s_fp16_x86.engine \\"
        echo "    tcp://$HOST_IP:5001 \\"
        echo "    0.25 show $HOST_IP 5000 tcp"
        echo ""
        exit 0
    fi
fi

# Method 2: ip addr (fallback)
if command -v ip &> /dev/null; then
    HOST_IP=$(ip addr show eth0 2>/dev/null | grep "inet " | awk '{print $2}' | cut -d/ -f1)
    if [ -n "$HOST_IP" ]; then
        echo "✅ WSL Host IP (from ip addr): $HOST_IP"
        echo ""
        echo "Use this IP: $HOST_IP"
        echo "Add protocol parameter: udp or tcp"
        exit 0
    fi
fi

# Method 3: ifconfig (fallback)
if command -v ifconfig &> /dev/null; then
    HOST_IP=$(ifconfig eth0 2>/dev/null | grep "inet " | awk '{print $2}')
    if [ -n "$HOST_IP" ]; then
        echo "✅ WSL Host IP (from ifconfig): $HOST_IP"
        echo ""
        echo "Use this IP: $HOST_IP"
        echo "Add protocol parameter: udp or tcp"
        exit 0
    fi
fi

echo "❌ Could not determine WSL host IP"
echo ""
echo "Try manually:"
echo "  hostname -I"
echo "  ip addr show"
