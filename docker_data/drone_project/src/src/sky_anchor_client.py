#!/usr/bin/env python3
"""
Sky Anchor Client - A non-blocking TCP client to receive data from server

Each JSON line received on the wire is parsed into a typed
``StabilizerReading`` right at the network edge (GRASP Step 4, IE-2);
no raw dict ever leaves this module.
"""

import socket
import json
import threading
import time
from typing import Optional

from src.domain.types import StabilizerReading
from src.logger import get_logger

class SkyAnchorClient:
    """Non-blocking sky anchor client that receives data in a separate thread"""

    def __init__(self, host='127.0.0.1', port=8888):
        self.logger = get_logger("sky_anchor_client", "logs/sky_anchor_client.log", log_level="INFO",
                                 console_output=False)
        self.host = host
        self.port = port
        self.client_socket = None
        self.connected = False
        self.running = False
        self.receive_thread = None
        self.buffer = ""
        self.__reading: Optional[StabilizerReading] = None
        self.error_message = None
        self._lock = threading.Lock()

    def connect(self):
        """Connect to the sky anchor server"""
        try:
            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.client_socket.settimeout(1.0)  # 1 second timeout for non-blocking behavior
            self.client_socket.connect((self.host, self.port))
            self.connected = True
            self.running = True

            # Start the receiving thread
            self.receive_thread = threading.Thread(target=self._receive_data, daemon=True)
            self.receive_thread.start()

            self.logger.warning(f"Connected to sky anchor server at {self.host}:{self.port}")
            return True

        except ConnectionRefusedError:
            self.error_message = f"Could not connect to server at {self.host}:{self.port}"
            self.logger.error(self.error_message)
            self.logger.error("Make sure the sky anchor server is running")
            return False
        except Exception as e:
            self.error_message = f"Connection error: {e}"
            self.logger.error(self.error_message)
            return False

    def _receive_data(self):
        """Private method to receive data in a separate thread"""
        while self.running and self.connected:
            try:
                data = self.client_socket.recv(1024)

                if not data:
                    self.logger.warning("Server disconnected")
                    self.connected = False
                    break

                # Decode and add to buffer
                self.buffer += data.decode('utf-8')

                # Process complete JSON messages (separated by newlines)
                while '\n' in self.buffer:
                    line, self.buffer = self.buffer.split('\n', 1)
                    if line.strip():
                        try:
                            payload = json.loads(line.strip())
                            reading = StabilizerReading.from_payload(payload)
                            with self._lock:
                                self.logger.info(f"data: {reading}")
                                self.__reading = reading

                        except json.JSONDecodeError as e:
                            self.logger.error(f"Error parsing JSON: {e}")
                        except (KeyError, TypeError, ValueError) as e:
                            self.logger.error(f"Malformed payload: {e}")

            except socket.timeout:
                continue
            except Exception as e:
                if self.running:
                    self.logger.error(f"Receive error: {e}")
                    self.connected = False
                break

    def tick(self) -> Optional[StabilizerReading]:
        """Return the latest reading (or None if nothing arrived yet).

        The reading is a frozen dataclass, so no defensive copy is needed.
        """
        with self._lock:
            return self.__reading

    def is_connected(self):
        """Check if client is connected to server"""
        return self.connected

    def is_running(self):
        """Check if client is running"""
        return self.running

    def get_error(self):
        """Get last error message"""
        return self.error_message

    def disconnect(self):
        """Disconnect from server and stop receiving thread"""
        self.running = False
        self.connected = False

        if self.receive_thread and self.receive_thread.is_alive():
            self.receive_thread.join(timeout=2.0)

        if self.client_socket:
            try:
                self.client_socket.close()
            except:
                pass

        self.logger.info("Client disconnected")

def main():
    """Main function demonstrating both blocking and non-blocking usage"""

    client = SkyAnchorClient('localhost', 8888)

    logger = get_logger("sky_anchor_client_main", "logs/sky_anchor_client.log")
    logger.info("Connecting to sky anchor server...")
    if client.connect():
        logger.info("Connected to sky anchor server.")
    else:
        logger.error("Failed to connect to sky anchor server.")
        return

    while True:
        data = client.tick()

        logger.debug('Waiting for data...')

        if data:
            logger.info(f"Received data: {data}")

        time.sleep(0.01)


if __name__ == "__main__":
    main()
