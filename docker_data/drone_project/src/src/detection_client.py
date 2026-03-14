#!/usr/bin/env python3
"""
Docker Detection Client - Manages Docker container running object recognition
"""

import os
import socket
import subprocess
import threading
import time
from typing import Optional
from datetime import datetime

from src.logger import get_logger
from src.detection_config import (
    DETECTION_DOCKER_SCRIPT,
    DETECTION_DOCKER_CONTAINER,
    DETECTION_DOCKER_PASSWORD,
    DETECTION_MODEL,
    DETECTION_LABELS,
    DETECTION_CAMERA_INPUT,
    DETECTION_THRESHOLD,
    DETECTION_SCRIPT_PATH,
    DETECTION_SERVER_PORT,
    DETECTION_CLIENT_LOG,
)


class DockerDetectionClient:
    """
    Manages Docker container running object detection client.

    Handles:
    - Starting Docker container with pexpect
    - Executing detection commands inside container
    - Logging Docker output
    - Stopping container gracefully
    """

    def __init__(self, logger=None):
        """
        Initialize Docker detection client.

        Args:
            logger: Optional external logger
        """
        self.script_path = DETECTION_DOCKER_SCRIPT
        self.container_name = DETECTION_DOCKER_CONTAINER
        self.password = DETECTION_DOCKER_PASSWORD
        self.log_file = DETECTION_CLIENT_LOG

        self.running = False
        self.docker_process = None
        self.log_thread: Optional[threading.Thread] = None
        self.stop_logging = False

        # Logger
        self.logger = logger or get_logger(
            "detection_client",
            DETECTION_CLIENT_LOG,
            log_level="INFO"
        )

    def start(self) -> bool:
        """
        Start Docker container and detection client.

        Returns:
            True if started successfully, False otherwise
        """
        if self.running:
            self.logger.warning("Detection client already running")
            return False

        try:
            import pexpect
        except ImportError:
            self.logger.error("pexpect module required. Install with: pip install pexpect")
            return False

        # Clean up any existing container
        self._cleanup_existing_container()

        # Get host IP for Docker communication
        server_host = self._get_host_ip()
        detection_cmd = self._build_detection_command(server_host, DETECTION_SERVER_PORT)

        self.logger.info(f"Starting Docker detection client: {self.container_name}")
        self.logger.info(f"Server: {server_host}:{DETECTION_SERVER_PORT}")

        # Mark as starting immediately
        self.running = True

        def start_docker_async():
            """Start Docker container in background thread."""
            try:
                # Get jetson-inference directory
                jetson_dir = os.path.dirname(os.path.abspath(self.script_path))
                script_name = os.path.basename(self.script_path)

                self.logger.info(f"Docker script directory: {jetson_dir}")
                self.logger.info(f"Docker script name: {script_name}")

                # Verify script exists
                script_full_path = os.path.join(jetson_dir, script_name)
                if not os.path.exists(script_full_path):
                    self.logger.error(f"Docker script not found: {script_full_path}")
                    self.running = False
                    return

                # Spawn docker script
                self.docker_process = pexpect.spawn(
                    f'bash {script_name}',
                    cwd=jetson_dir,
                    encoding='utf-8',
                    timeout=60
                )

                # Setup logging
                os.makedirs(os.path.dirname(self.log_file), exist_ok=True)
                log_fh = open(self.log_file, 'a')
                timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                log_fh.write(f"\n{'='*70}\n")
                log_fh.write(f"Detection client started at {timestamp}\n")
                log_fh.write(f"{'='*70}\n")
                log_fh.flush()

                self.docker_process.logfile = log_fh

                # Handle password prompt if needed
                if self.password:
                    log_fh.write(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] Waiting for password prompt...\n")
                    log_fh.flush()
                    idx = self.docker_process.expect(['password', pexpect.TIMEOUT], timeout=30)
                    if idx == 0:
                        self.docker_process.sendline(self.password)
                        log_fh.write(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] Password sent\n")
                        log_fh.flush()
                        self.logger.info("Password prompt handled")
                    else:
                        log_fh.write(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] No password prompt (timeout)\n")
                        log_fh.flush()
                        self.logger.warning("No password prompt detected")

                # Wait for shell prompt
                log_fh.write(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] Waiting for shell prompt (# $ >)...\n")
                log_fh.flush()
                try:
                    idx = self.docker_process.expect(['#', '$', '>', pexpect.TIMEOUT], timeout=30)
                    if idx < 3:  # Found shell prompt
                        prompt_char = ['#', '$', '>'][idx]
                        log_fh.write(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] Shell prompt detected: '{prompt_char}'\n")
                        log_fh.flush()
                        self.logger.info(f"Docker shell ready (prompt: '{prompt_char}')")
                    else:
                        log_fh.write(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] Shell prompt timeout after 30s\n")
                        log_fh.write(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] Last output before timeout:\n")
                        log_fh.write(self.docker_process.before if self.docker_process.before else "(no output)\n")
                        log_fh.flush()
                        self.logger.error("Shell prompt timeout - Docker may not be starting properly")
                        self.running = False
                        return
                except Exception as e:
                    log_fh.write(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] Exception waiting for prompt: {e}\n")
                    log_fh.flush()
                    self.logger.error(f"Exception waiting for shell prompt: {e}")
                    self.running = False
                    return

                # Send detection commands
                log_fh.write(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] Sending commands...\n")
                log_fh.flush()
                self.docker_process.sendline(detection_cmd)
                log_fh.write(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] Commands sent:\n{detection_cmd}\n")
                log_fh.write(f"{'='*70}\n")
                log_fh.write("Docker output below:\n")
                log_fh.write(f"{'='*70}\n")
                log_fh.flush()

                self.logger.warning(f"Detection client Docker commands executed (logs: {self.log_file})")

                # Start background logging thread
                self.stop_logging = False

                def continuous_read():
                    """Continuously read from child process and write to log."""
                    try:
                        while not self.stop_logging and self.docker_process.isalive():
                            try:
                                self.docker_process.expect('.+', timeout=0.5)
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

            except Exception as e:
                self.logger.error(f"Failed to start detection client: {e}")
                import traceback
                self.logger.error(traceback.format_exc())
                self.running = False

        # Start Docker in background thread
        docker_thread = threading.Thread(target=start_docker_async, daemon=True)
        docker_thread.start()

        self.logger.warning("Detection client starting in background...")
        return True

    def stop(self) -> bool:
        """
        Stop Docker container and detection client.

        Returns:
            True if stopped successfully, False otherwise
        """
        if not self.running:
            self.logger.warning("Detection client not running")
            return False

        self.logger.info("Stopping detection client")
        self.running = False

        # Stop logging thread
        self.stop_logging = True
        if self.log_thread:
            self.log_thread.join(timeout=1)

        # Terminate docker process
        if self.docker_process:
            try:
                self.docker_process.terminate()
                self.docker_process.wait(timeout=2)
            except:
                try:
                    self.docker_process.kill()
                except:
                    pass

        # Stop Docker container
        try:
            self.logger.info(f"Stopping Docker container: {self.container_name}")
            subprocess.run(
                ['docker', 'stop', self.container_name],
                capture_output=True,
                timeout=10
            )
            self.logger.warning("Detection client stopped successfully")
            return True
        except Exception as e:
            self.logger.error(f"Error stopping Docker container: {e}")
            return False

    def is_running(self) -> bool:
        """Check if detection client is running."""
        return self.running

    def _get_host_ip(self) -> str:
        """Get the host machine's IP address for Docker communication."""
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                s.connect(("8.8.8.8", 80))
                return s.getsockname()[0]
        except Exception:
            return "172.17.0.1"

    def _build_detection_command(self, server_host: str, server_port: int) -> str:
        """Build the detection command with all parameters."""
        # Extract directory from script path
        script_dir = os.path.dirname(DETECTION_SCRIPT_PATH)
        script_name = os.path.basename(DETECTION_SCRIPT_PATH)

        cmd = f"""cd {script_dir}
./{script_name} \\
--model={DETECTION_MODEL} \\
--labels={DETECTION_LABELS} \\
--input-blob=input_0 \\
--output-cvg=scores \\
--output-bbox=boxes \\
--server_host={server_host} \\
--server_port={server_port} \\
--threshold={DETECTION_THRESHOLD}
"""
        return cmd

    def _cleanup_existing_container(self) -> None:
        """Stop any existing container with the same name."""
        try:
            result = subprocess.run(
                ['docker', 'ps', '-a', '--filter', f'name={self.container_name}', '--format', '{{.Names}}'],
                capture_output=True,
                text=True,
                timeout=5
            )
            if self.container_name in result.stdout:
                self.logger.info(f"Cleaning up existing container: {self.container_name}")
                subprocess.run(
                    ['docker', 'stop', self.container_name],
                    capture_output=True,
                    timeout=10
                )
        except Exception as e:
            self.logger.warning(f"Could not clean up existing container: {e}")
