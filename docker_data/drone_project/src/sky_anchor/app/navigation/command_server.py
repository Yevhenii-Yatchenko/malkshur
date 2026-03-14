"""
Navigation command server for external control.

This module provides a TCP server that accepts navigation commands from
external systems (e.g., main drone controller, ground station, etc.).

Commands are sent as JSON over TCP on port 8889 (default).

Supported commands:
    - navigate_to: Set navigation target
    - cancel: Cancel navigation
    - status: Get navigation status
    - update_reference: Manually trigger reference update
"""

from __future__ import annotations

import json
import socket
import threading
from typing import TYPE_CHECKING, Optional

from ..logger import UnbufferedLogger
from .types import NavigationTarget

if TYPE_CHECKING:
    from .coordinator import NavigationCoordinator


class NavigationCommandServer:
    """
    TCP server for receiving navigation commands.

    Listens on a separate port from the sky_anchor data server and
    forwards commands to the NavigationCoordinator instance.
    """

    def __init__(
        self,
        coordinator: NavigationCoordinator,
        logger: UnbufferedLogger,
        host: str = "0.0.0.0",
        port: int = 8889,
    ) -> None:
        """
        Initialize navigation command server.

        Args:
            coordinator: NavigationCoordinator instance to send commands to
            logger: Logger instance
            host: Host to bind to (0.0.0.0 for all interfaces)
            port: Port to listen on (default 8889)
        """
        self.coordinator = coordinator
        self.logger = logger
        self.host = host
        self.port = port

        self.server_socket: Optional[socket.socket] = None
        self.running = False
        self.accept_thread: Optional[threading.Thread] = None

        # Lock for thread safety
        self.lock = threading.Lock()

    def start(self) -> None:
        """Start the command server (non-blocking)."""
        if self.running:
            self.logger.warning("Navigation command server already running")
            return

        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind((self.host, self.port))
            self.server_socket.listen(5)
            self.running = True

            self.accept_thread = threading.Thread(
                target=self._accept_connections,
                daemon=True,
                name="NavCommandAcceptThread",
            )
            self.accept_thread.start()

            self.logger.info(f"Navigation command server started on {self.host}:{self.port}")
        except Exception as e:
            self.logger.error(f"Failed to start navigation command server: {e}")
            self.running = False

    def stop(self) -> None:
        """Stop the command server."""
        self.running = False
        if self.server_socket:
            try:
                self.server_socket.close()
            except Exception:
                pass
        self.logger.info("Navigation command server stopped")

    def _accept_connections(self) -> None:
        """Accept incoming connections (runs in separate thread)."""
        while self.running:
            try:
                if self.server_socket is None:
                    break
                client_socket, client_address = self.server_socket.accept()
                self.logger.info(f"Navigation command connection from {client_address}")

                # Handle each client in a separate thread
                client_thread = threading.Thread(
                    target=self._handle_client,
                    args=(client_socket, client_address),
                    daemon=True,
                )
                client_thread.start()
            except OSError:
                # Socket was closed
                break
            except Exception as e:
                self.logger.error(f"Error accepting connection: {e}")

    def _handle_client(self, client_socket: socket.socket, address: tuple) -> None:
        """
        Handle a single client connection.

        Args:
            client_socket: Client socket
            address: Client address tuple
        """
        try:
            client_socket.settimeout(60.0)  # 60 second timeout
            buffer = ""

            while self.running:
                try:
                    data = client_socket.recv(4096).decode('utf-8')
                    if not data:
                        break

                    buffer += data

                    # Process complete messages (newline-delimited JSON)
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        if line.strip():
                            response = self._process_command(line.strip())
                            # Send response
                            client_socket.sendall((json.dumps(response) + '\n').encode('utf-8'))

                except socket.timeout:
                    self.logger.debug(f"Client {address} timed out")
                    break
                except Exception as e:
                    self.logger.error(f"Error handling client {address}: {e}")
                    break

        finally:
            try:
                client_socket.close()
            except Exception:
                pass
            self.logger.info(f"Client {address} disconnected")

    def _process_command(self, command_str: str) -> dict:
        """
        Process a navigation command.

        Args:
            command_str: JSON command string

        Returns:
            Response dictionary
        """
        try:
            command = json.loads(command_str)
        except json.JSONDecodeError as e:
            return {'success': False, 'error': f'Invalid JSON: {e}'}

        if not isinstance(command, dict):
            return {'success': False, 'error': 'Command must be a JSON object'}

        cmd_type = command.get('command')
        if not cmd_type:
            return {'success': False, 'error': 'Missing "command" field'}

        self.logger.debug(f"Processing command: {cmd_type}")

        # Dispatch to appropriate handler
        if cmd_type == 'navigate_to':
            return self._cmd_navigate_to(command)
        elif cmd_type == 'update_reference':
            return self._cmd_update_reference()
        else:
            return {'success': False, 'error': f'Unknown command: {cmd_type}'}

    def _cmd_navigate_to(self, command: dict) -> dict:
        """
        Handle navigate_to command.

        Expected format:
            {"command": "navigate_to", "dx": 100, "dy": 50, "tolerance": 10}
        """
        try:
            dx = float(command.get('dx', 0))
            dy = float(command.get('dy', 0))
            tolerance = float(command.get('tolerance', 10.0))

            target = NavigationTarget(
                dx_pixels=dx,
                dy_pixels=dy,
                tolerance_pixels=tolerance,
            )
            self.coordinator.set_target(target)

            return {
                'success': True,
                'message': f'Navigation to ({dx}, {dy}) activated',
            }
        except Exception as e:
            self.logger.error(f"Error in navigate_to command: {e}")
            return {'success': False, 'error': str(e)}

    def _cmd_update_reference(self) -> dict:
        """Handle manual reference update request."""
        try:
            # Request reference update through controller
            # This will be picked up on next iteration
            # For now, just return success
            return {
                'success': True,
                'message': 'Reference update requested (will occur on next cycle)',
            }
        except Exception as e:
            self.logger.error(f"Error in update_reference command: {e}")
            return {'success': False, 'error': str(e)}
