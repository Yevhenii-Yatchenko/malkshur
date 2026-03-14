#!/usr/bin/env python3
"""
Docker Detection Runner with Server Control Demo
=================================================
Demonstrates server start/stop/restart while Docker client keeps running.

Timeline:
1. Start Docker client (with detection script)
2. Wait 10 seconds, then start server
3. Wait 5 seconds, then stop server
4. Wait 5 seconds, then start server again
5. Wait 10 seconds, then stop server and interrupt client

This shows that:
- Client can run before server starts
- Server can be stopped and restarted while client is running
- Client automatically reconnects when server comes back

Logs are written to:
- camera_test/logs/docker_client.log - All Docker output
- camera_test/logs/server.log - All server activity
"""

import json
import os
import socket
import sys
import threading
import time
from datetime import datetime
from typing import Optional, Tuple

# ============================================================================
# Configuration
# ============================================================================

# Server configuration
SERVER_HOST_LISTEN = '0.0.0.0'
SERVER_PORT = 5000

# Docker configuration
DOCKER_RUN_SCRIPT = "../jetson-inference/docker/run.sh"
DOCKER_PASSWORD = "1234"

# Detection model configuration
DETECTION_MODEL = "models/ONNXs/nv-v2-L1-98-E58-ssd-mobilenet.onnx"
DETECTION_LABELS = "models/ONNXs/labels.txt"
CAMERA_INPUT = "csi://0"

# Logging configuration
LOG_DIR = "camera_test/logs"
DOCKER_LOG = os.path.join(LOG_DIR, "docker_client.log")
SERVER_LOG = os.path.join(LOG_DIR, "server.log")

# Demo timeline (in seconds)
DEMO_TIMELINE = {
    'wait_before_server_start': 10,
    'server_run_before_stop': 5,
    'wait_before_server_restart': 5,
    'server_run_before_final_stop': 10
}


# ============================================================================
# Utility Functions
# ============================================================================

def get_host_ip() -> str:
    """Get the host machine's IP address for Docker communication."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("8.8.8.8", 80))
            return s.getsockname()[0]
    except Exception:
        return "172.17.0.1"


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


def setup_logging() -> None:
    """Create log directory and initialize log files."""
    os.makedirs(LOG_DIR, exist_ok=True)
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    with open(DOCKER_LOG, 'w') as f:
        f.write(f"=== Docker Client Log - Started at {timestamp} ===\n")

    with open(SERVER_LOG, 'w') as f:
        f.write(f"=== Server Log - Started at {timestamp} ===\n")

    print(f"[LOG] Docker output → {DOCKER_LOG}")
    print(f"[LOG] Server output → {SERVER_LOG}")


def print_separator(char: str = '=', length: int = 70) -> None:
    """Print a separator line."""
    print(char * length)


def print_step(step_num: int, description: str) -> None:
    """Print a step header."""
    print(f"\n{'=' * 70}")
    print(f"STEP {step_num}: {description}")
    print(f"{'=' * 70}\n")


def countdown(seconds: int, message: str) -> None:
    """Print a countdown timer."""
    for i in range(seconds, 0, -1):
        print(f"   {message} in {i} seconds...")
        time.sleep(1)


# ============================================================================
# Server Class
# ============================================================================

class DetectionServer:
    """
    Controllable detection server with start/stop capabilities and logging.

    The server runs in a background thread and can be started/stopped
    multiple times during the demo.
    """

    def __init__(self, host: str, port: int, log_file: str):
        self.host = host
        self.port = port
        self.log_file = log_file
        self.running = False
        self.server_thread: Optional[threading.Thread] = None
        self.server_socket: Optional[socket.socket] = None

    def start(self) -> bool:
        """Start the server in a background thread."""
        if self.is_running():
            print("⚠️  Server already running")
            return False

        print(f"🚀 Starting server on {self.host}:{self.port}...")
        self.running = True
        self.server_thread = threading.Thread(
            target=self._run_server,
            daemon=False,
            name="DetectionServer"
        )
        self.server_thread.start()
        time.sleep(0.2)

        if self.is_running():
            print("✅ Server started")
            return True
        else:
            print("❌ Server failed to start")
            return False

    def stop(self) -> bool:
        """Stop the server gracefully."""
        if not self.is_running():
            print("⚠️  Server not running")
            return False

        print("🛑 Stopping server...")
        self.running = False

        if self.server_socket:
            try:
                self.server_socket.close()
            except:
                pass

        if self.server_thread and self.server_thread.is_alive():
            self.server_thread.join(timeout=2)

        print("✅ Server stopped")
        return True

    def is_running(self) -> bool:
        """Check if server is currently running."""
        return self.running and self.server_thread and self.server_thread.is_alive()

    def _log(self, message: str) -> None:
        """Write message to log file with timestamp."""
        try:
            with open(self.log_file, 'a') as f:
                timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                f.write(f"[{timestamp}] {message}\n")
                f.flush()
        except:
            pass

    def _run_server(self) -> None:
        """Main server loop (runs in background thread)."""
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind((self.host, self.port))
            self.server_socket.listen()
            self.server_socket.settimeout(1.0)

            msg = f"Listening on {self.host}:{self.port}"
            print(f"[SERVER] {msg}")
            self._log(msg)

            while self.running:
                try:
                    conn, addr = self.server_socket.accept()
                    threading.Thread(
                        target=self._handle_client,
                        args=(conn, addr),
                        daemon=True
                    ).start()
                except socket.timeout:
                    continue
                except OSError:
                    break
                except Exception as e:
                    if self.running:
                        msg = f"Server error: {e}"
                        print(f"[SERVER ERROR] {msg}")
                        self._log(msg)

        except Exception as e:
            msg = f"Fatal error: {e}"
            print(f"[SERVER FATAL] {msg}")
            self._log(msg)
        finally:
            if self.server_socket:
                try:
                    self.server_socket.close()
                except:
                    pass

    def _handle_client(self, conn: socket.socket, addr: Tuple[str, int]) -> None:
        """Handle individual client connection."""
        try:
            with conn:
                data = conn.recv(4096)
                if not data:
                    return

                try:
                    data_dict = json.loads(data.decode())
                    timestamp = time.strftime('%H:%M:%S')

                    # Console output (simplified)
                    print(f"\n[{timestamp}] 📥 Data from {addr[0]}:{addr[1]}")
                    if 'class_id' in data_dict:
                        conf = data_dict.get('confidence', 0)
                        print(f"    Detection: class={data_dict['class_id']}, conf={conf:.2f}")

                    # Log file output (full data)
                    self._log(f"Received from {addr[0]}:{addr[1]}: {json.dumps(data_dict)}")

                    # Send acknowledgment
                    response = {"status": "ok", "timestamp": time.time()}
                    conn.sendall(json.dumps(response).encode())

                except json.JSONDecodeError as e:
                    msg = f"Invalid JSON: {e}"
                    print(f"[ERROR] {msg}")
                    self._log(msg)

        except Exception as e:
            self._log(f"Client error: {e}")


# ============================================================================
# Docker Runner Class
# ============================================================================

class DockerDetectionRunner:
    """
    Manages Docker container execution with pexpect automation and logging.

    Handles password entry, command execution, and continuous output logging
    to file while the detection script runs inside Docker.
    """

    def __init__(self, script_path: str, password: str, log_file: str):
        self.script_path = script_path
        self.password = password
        self.log_file = log_file
        self.log_thread: Optional[threading.Thread] = None
        self.stop_logging = False

    def run(self, commands: str):
        """
        Run Docker with automated password entry and command execution.

        Returns:
            pexpect child process handle
        """
        import pexpect

        jetson_dir = get_jetson_inference_dir(self.script_path)

        # Spawn docker script
        child = pexpect.spawn(
            'bash docker/run.sh',
            cwd=jetson_dir,
            encoding='utf-8'
        )

        # Open log file for all Docker output
        log_fh = open(self.log_file, 'a')
        child.logfile = log_fh

        # Handle password prompt
        idx = child.expect(['password', pexpect.TIMEOUT], timeout=30)
        if idx == 0:
            child.sendline(self.password)
            timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            log_fh.write(f"[{timestamp}] Password sent\n")
            log_fh.flush()

        # Wait for shell prompt
        child.expect(['#', '$', '>', pexpect.TIMEOUT], timeout=30)

        # Send detection commands
        child.sendline(commands)
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        log_fh.write(f"[{timestamp}] Commands sent:\n{commands}\n")
        log_fh.flush()

        # Start background thread to continuously read output
        self.stop_logging = False

        def continuous_read():
            """Continuously read from child process and write to log."""
            try:
                while not self.stop_logging and child.isalive():
                    try:
                        child.expect('.+', timeout=0.1)
                    except pexpect.TIMEOUT:
                        continue
                    except pexpect.EOF:
                        break
            except:
                pass
            finally:
                log_fh.close()

        self.log_thread = threading.Thread(target=continuous_read, daemon=True)
        self.log_thread.start()

        print(f"[INFO] Docker output is being logged to: {self.log_file}")
        return child

    def stop(self) -> None:
        """Stop the logging thread."""
        self.stop_logging = True
        if self.log_thread:
            self.log_thread.join(timeout=1)


# ============================================================================
# Main Demo Function
# ============================================================================

def run_control_demo() -> None:
    """
    Run the server control demo with the predefined timeline.

    This demonstrates that the server can be stopped and restarted
    while the Docker detection client keeps running and automatically
    reconnects when the server comes back online.
    """
    print_separator()
    print("Docker Detection with Server Control Demo")
    print_separator()

    # Setup logging
    setup_logging()

    # Get host IP for Docker communication
    server_host_docker = get_host_ip()
    print(f"\n[CONFIG] Host IP: {server_host_docker}")
    print(f"[CONFIG] Server will listen on: {SERVER_HOST_LISTEN}:{SERVER_PORT}")
    print(f"[CONFIG] Docker will connect to: {server_host_docker}:{SERVER_PORT}")

    # Build detection command
    detection_cmd = build_detection_command(server_host_docker, SERVER_PORT)

    # Create server and Docker runner
    server = DetectionServer(SERVER_HOST_LISTEN, SERVER_PORT, SERVER_LOG)
    docker_runner = DockerDetectionRunner(DOCKER_RUN_SCRIPT, DOCKER_PASSWORD, DOCKER_LOG)

    # ========================================================================
    # STEP 1: Start Docker Client
    # ========================================================================
    print_step(1, "Starting Docker client (detection will run but no server yet)")

    print("Launching Docker container...")
    docker_process = docker_runner.run(detection_cmd)

    print("✅ Docker client started")
    print("   (Client is running but will fail to connect since no server yet)")
    print(f"   (Watch logs: tail -f {DOCKER_LOG})")

    # ========================================================================
    # STEP 2: Start Server
    # ========================================================================
    print_step(2, "Waiting before starting server...")
    countdown(DEMO_TIMELINE['wait_before_server_start'], "Starting server")

    server.start()
    print("   (Client will now connect and send data)")

    # ========================================================================
    # STEP 3: Stop Server
    # ========================================================================
    print_step(3, "Running with server, then stopping server...")
    countdown(DEMO_TIMELINE['server_run_before_stop'], "Stopping server")

    server.stop()
    print("   (Client will lose connection and retry)")

    # ========================================================================
    # STEP 4: Restart Server
    # ========================================================================
    print_step(4, "Waiting before restarting server...")
    countdown(DEMO_TIMELINE['wait_before_server_restart'], "Starting server")

    server.start()
    print("   (Client will reconnect automatically)")

    # ========================================================================
    # STEP 5: Stop Everything
    # ========================================================================
    print_step(5, "Running with server, then stopping everything...")
    countdown(DEMO_TIMELINE['server_run_before_final_stop'], "Stopping")

    # Stop server
    server.stop()

    # Stop Docker
    print("\n🛑 Stopping Docker client...")
    docker_runner.stop()
    try:
        docker_process.terminate()
        docker_process.wait(timeout=2)
    except:
        try:
            docker_process.kill()
        except:
            pass

    print("✅ Docker client stopped")

    # ========================================================================
    # Summary
    # ========================================================================
    print_separator()
    print("Demo Complete!")
    print_separator()
    print("\nWhat you saw:")
    print("  1. ✅ Client started (no server)")
    print("  2. ✅ Server started → client connected")
    print("  3. ✅ Server stopped → client lost connection")
    print("  4. ✅ Server started → client reconnected")
    print("  5. ✅ Everything stopped")
    print("\nThis demonstrates:")
    print("  • Server can be stopped and restarted while client runs")
    print("  • Client automatically reconnects when server comes back")
    print("  • Full control over server lifecycle")
    print(f"\nCheck logs for detailed output:")
    print(f"  • Docker: {DOCKER_LOG}")
    print(f"  • Server: {SERVER_LOG}")
    print_separator()


# ============================================================================
# Entry Point
# ============================================================================

def main() -> None:
    """Main entry point - check dependencies and run demo."""
    try:
        import pexpect
        print("[INFO] pexpect available - demo can run\n")
    except ImportError:
        print("[ERROR] pexpect required for this demo")
        print("[INFO] Install with: python3 -m pip install pexpect\n")
        sys.exit(1)

    run_control_demo()


if __name__ == "__main__":
    main()
