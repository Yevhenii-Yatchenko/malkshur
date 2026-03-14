#!/usr/bin/env python3
"""
Docker Detection Runner with Server
====================================
Automates jetson-inference Docker container with object detection and
server communication.

Features:
- Auto-detects host IP for Docker communication
- Runs detection server in background thread
- Supports pexpect for automatic password entry
- Generates startup script as fallback
"""

import json
import os
import socket
import subprocess
import sys
import threading
import time
from typing import Optional, Tuple

# ============================================================================
# Configuration
# ============================================================================

# Server configuration
SERVER_HOST_LISTEN = '0.0.0.0'  # Listen on all interfaces
SERVER_PORT = 5000

# Docker configuration
DOCKER_RUN_SCRIPT = "../jetson-inference/docker/run.sh"
DOCKER_PASSWORD = "1234"

# Detection model configuration
DETECTION_MODEL = "models/ONNXs/nv-v2-L1-98-E58-ssd-mobilenet.onnx"
DETECTION_LABELS = "models/ONNXs/labels.txt"
CAMERA_INPUT = "csi://0"

# ============================================================================
# Utility Functions
# ============================================================================

def get_host_ip() -> str:
    """
    Get the host machine's IP address for Docker communication.

    Returns:
        str: Host IP address or Docker bridge gateway as fallback
    """
    try:
        # Create a UDP socket to determine which interface would be used
        # (doesn't actually send data)
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("8.8.8.8", 80))  # Google DNS
            return s.getsockname()[0]
    except Exception:
        return "172.17.0.1"  # Default Docker bridge gateway


def get_jetson_inference_dir(script_path: str) -> str:
    """Get the jetson-inference directory from the script path."""
    docker_dir = os.path.dirname(os.path.abspath(script_path))
    return os.path.dirname(docker_dir)


def build_detection_command(server_host: str, server_port: int) -> str:
    """Build the detection command with all parameters."""
    return f"""cd data/dd_smart_agent/
./ssd_mobilenet_v1_singel_camera.py \\
--model={DETECTION_MODEL} \\
--labels={DETECTION_LABELS} \\
--input-blob=input_0 \\
--output-cvg=scores \\
--output-bbox=boxes \\
--server_host={server_host} \\
--server_port={server_port} \\
{CAMERA_INPUT}
"""


def print_separator(char='=', length=70):
    """Print a separator line."""
    print(char * length)


def print_manual_instructions(server_host: str, server_port: int):
    """Print manual instructions for running detection inside Docker."""
    print_separator()
    print("MANUAL STEPS REQUIRED:")
    print_separator()
    print("After Docker starts, run inside the container:")
    print()
    print("  /jetson-inference/data/auto_start_detection.sh")
    print()
    print("OR manually:")
    print(f"  cd data/dd_smart_agent/")
    print(f"  ./ssd_mobilenet_v1_singel_camera.py \\")
    print(f"    --model={DETECTION_MODEL} \\")
    print(f"    --labels={DETECTION_LABELS} \\")
    print(f"    --input-blob=input_0 \\")
    print(f"    --output-cvg=scores \\")
    print(f"    --output-bbox=boxes \\")
    print(f"    --server_host={server_host} \\")
    print(f"    --server_port={server_port} \\")
    print(f"    {CAMERA_INPUT}")
    print()
    print(f"Detection data will be sent to: {server_host}:{server_port}")
    print_separator()
    print()

# ============================================================================
# Server Class
# ============================================================================

class DetectionServer:
    """Threaded server to receive detection data from Docker container."""

    def __init__(self, host: str = SERVER_HOST_LISTEN, port: int = SERVER_PORT):
        self.host = host
        self.port = port
        self.running = False
        self.server_thread: Optional[threading.Thread] = None

    def start(self):
        """Start the server in a background thread."""
        self.running = True
        self.server_thread = threading.Thread(
            target=self._run_server,
            daemon=True,
            name="DetectionServer"
        )
        self.server_thread.start()
        print(f"[SERVER] Started on {self.host}:{self.port}")

    def stop(self):
        """Stop the server gracefully."""
        self.running = False
        if self.server_thread and self.server_thread.is_alive():
            self.server_thread.join(timeout=2)
        print("[SERVER] Stopped")

    def _run_server(self):
        """Main server loop - runs in background thread."""
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
                server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                server_socket.bind((self.host, self.port))
                server_socket.listen()
                server_socket.settimeout(1.0)  # Allow periodic running check

                print(f"[SERVER] Listening on {self.host}:{self.port}")

                while self.running:
                    try:
                        conn, addr = server_socket.accept()
                        # Handle each connection in a separate thread
                        threading.Thread(
                            target=self._handle_client,
                            args=(conn, addr),
                            daemon=True
                        ).start()
                    except socket.timeout:
                        continue
                    except Exception as e:
                        if self.running:
                            print(f"[SERVER ERROR] {e}")

        except Exception as e:
            print(f"[SERVER FATAL] {e}")

    def _handle_client(self, conn: socket.socket, addr: Tuple[str, int]):
        """Handle individual client connection."""
        try:
            with conn:
                data = conn.recv(4096)
                if not data:
                    print(f"[SERVER] Empty data from {addr}")
                    return

                try:
                    data_dict = json.loads(data.decode())
                    print(f"[SERVER] Data from {addr}:")
                    print(json.dumps(data_dict, indent=2))

                    # Send acknowledgment
                    response = {"status": "ok", "timestamp": time.time()}
                    conn.sendall(json.dumps(response).encode())

                except json.JSONDecodeError as e:
                    print(f"[SERVER ERROR] Invalid JSON: {e}")
                    error_response = {"status": "error", "message": str(e)}
                    conn.sendall(json.dumps(error_response).encode())

        except Exception as e:
            print(f"[SERVER ERROR] Client {addr}: {e}")

# ============================================================================
# Docker Runner Class
# ============================================================================

class DockerRunner:
    """Manages Docker container execution with automatic password entry."""

    def __init__(self, script_path: str, password: str):
        self.script_path = script_path
        self.password = password

    def run(self, commands: str) -> bool:
        """
        Run Docker container and execute commands.

        Args:
            commands: Shell commands to execute inside container

        Returns:
            bool: True if successful, False otherwise
        """
        if not os.path.exists(self.script_path):
            print(f"[DOCKER ERROR] Script not found: {self.script_path}")
            return False

        print(f"[DOCKER] Launching container...")

        try:
            # Try pexpect first for full automation
            try:
                import pexpect
                return self._run_with_pexpect(commands)
            except ImportError:
                print("[DOCKER] pexpect not available - manual mode")
                return self._run_with_subprocess(commands)

        except Exception as e:
            print(f"[DOCKER ERROR] {e}")
            return False

    def _run_with_pexpect(self, commands: str) -> bool:
        """Run with pexpect for automatic password and command entry."""
        import pexpect

        print("[DOCKER] Using pexpect for automation")

        jetson_dir = get_jetson_inference_dir(self.script_path)
        print(f"[DOCKER] Working directory: {jetson_dir}")

        # Spawn docker script
        child = pexpect.spawn('bash docker/run.sh', cwd=jetson_dir)
        child.logfile = sys.stdout.buffer

        # Handle password prompt
        print("[DOCKER] Waiting for password...")
        idx = child.expect(['password', pexpect.TIMEOUT], timeout=30)

        if idx == 0:
            print("[DOCKER] Entering password...")
            child.sendline(self.password)

        # Wait for shell and send commands
        print("[DOCKER] Waiting for shell...")
        child.expect(['#', '$', '>', pexpect.TIMEOUT], timeout=30)

        print("[DOCKER] Executing detection...")
        child.sendline(commands)

        # Interactive mode - user can see output and interact
        child.interact()
        return True

    def _run_with_subprocess(self, commands: str) -> bool:
        """Run with subprocess - requires manual password entry."""
        jetson_dir = get_jetson_inference_dir(self.script_path)
        print(f"[DOCKER] Working directory: {jetson_dir}")

        # Create startup script for convenience
        startup_script = os.path.join(jetson_dir, 'data', 'auto_start_detection.sh')
        try:
            with open(startup_script, 'w') as f:
                f.write("#!/bin/bash\n")
                f.write("# Auto-generated detection startup script\n\n")
                f.write(commands)

            os.chmod(startup_script, 0o755)
            print(f"[DOCKER] Created: {startup_script}")

        except Exception as e:
            print(f"[DOCKER ERROR] Failed to create startup script: {e}")

        # Print manual instructions
        print()
        print_manual_instructions(
            server_host=get_host_ip(),
            server_port=SERVER_PORT
        )

        # Run docker script
        subprocess.run(['bash', 'docker/run.sh'], cwd=jetson_dir)
        return True

# ============================================================================
# Main Function
# ============================================================================

def main():
    """Main execution function."""
    # Detect configuration
    server_host_docker = get_host_ip()

    # Print configuration
    print_separator()
    print("Docker Detection Runner with Server")
    print_separator()
    print(f"[CONFIG] Host IP (for Docker):  {server_host_docker}")
    print(f"[CONFIG] Server listening on:   {SERVER_HOST_LISTEN}:{SERVER_PORT}")
    print(f"[CONFIG] Docker connects to:    {server_host_docker}:{SERVER_PORT}")
    print_separator()

    # Build detection command
    detection_cmd = build_detection_command(server_host_docker, SERVER_PORT)

    # Start server
    print("\n[STEP 1/2] Starting detection server...")
    server = DetectionServer(SERVER_HOST_LISTEN, SERVER_PORT)
    server.start()
    time.sleep(0.5)  # Brief pause for server startup

    # Start Docker
    print("\n[STEP 2/2] Launching Docker container...")
    docker = DockerRunner(DOCKER_RUN_SCRIPT, DOCKER_PASSWORD)

    try:
        docker.run(detection_cmd)
    except KeyboardInterrupt:
        print("\n\n[INTERRUPT] Ctrl+C received")
    finally:
        print("\n[CLEANUP] Shutting down...")
        server.stop()

    print_separator()
    print("Session ended")
    print_separator()

# ============================================================================
# Entry Point
# ============================================================================

if __name__ == "__main__":
    # Check for pexpect
    try:
        import pexpect
        print("[INFO] pexpect available - full automation enabled")
    except ImportError:
        print("[WARNING] pexpect not installed - manual steps required")
        print("[INFO] Install with: python3 -m pip install pexpect")

    print()
    main()
