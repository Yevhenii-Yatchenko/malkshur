#!/usr/bin/env python3
"""
Sky Anchor Server - A TCP server that sends data to connected clients
"""

from __future__ import annotations

import json
import socket
import threading
from contextlib import suppress
from typing import Any, Dict, List, Optional, Tuple

from app.config import SKY_ANCHOR_LOG_PATH
from app.logger import UnbufferedLogger, get_logger


class SkyAnchorServer:
    def __init__(
        self,
        host: str = 'localhost',
        port: int = 8888,
        logger: Optional[UnbufferedLogger] = None,
    ) -> None:
        self.host: str = host
        self.port: int = port
        self.server_socket: Optional[socket.socket] = None
        self.running: bool = False
        self.clients: Dict[str, Dict[str, Any]] = {}
        self.client_lock = threading.Lock()  # Thread-safe access to clients
        self._accept_thread: Optional[threading.Thread] = None
        self.logger: UnbufferedLogger = logger or get_logger(
            "sky_anchor_server",
            SKY_ANCHOR_LOG_PATH,
            log_level="INFO",
        )

    def handle_client(self, client_socket: socket.socket, client_address: Tuple[str, int]) -> None:
        """Handle individual client connections"""
        client_id = f"{client_address[0]}:{client_address[1]}"

        with self.client_lock:
            self.clients[client_id] = {
                'socket': client_socket,
                'address': client_address,
            }
            client_count = len(self.clients)

        self.logger.info(
            f"Connection established with {client_address} (ID: {client_id})"
        )
        self.logger.info(f"Total clients connected: {client_count}")

        client_socket.settimeout(1.0)

        try:
            # Keep the connection alive and wait for disconnection
            while self.running:
                try:
                    data = client_socket.recv(1024)
                except socket.timeout:
                    continue  # Continue waiting

                if not data:
                    break  # Client disconnected

        except (ConnectionResetError, ConnectionAbortedError):
            self.logger.warning(f"Client {client_id} disconnected unexpectedly")
        except Exception as exc:
            self.logger.error(f"Error handling client {client_id}: {exc}")
        finally:
            with self.client_lock:
                self.clients.pop(client_id, None)
                remaining = len(self.clients)

            with suppress(OSError):
                client_socket.close()

            self.logger.info(
                f"Connection with {client_address} closed (total clients={remaining})"
            )

    def tick(self, data: Dict[str, Any]) -> int:
        """Send data to all connected clients (call this method externally)"""
        if not self.clients:
            return 0

        message = json.dumps(data) + '\n'

        with self.client_lock:
            disconnected_clients: List[Tuple[str, socket.socket]] = []
            for client_id, client_info in list(self.clients.items()):
                try:
                    client_socket = client_info['socket']
                    client_socket.send(message.encode('utf-8'))
                except (ConnectionResetError, BrokenPipeError, ConnectionAbortedError) as exc:
                    self.logger.warning(
                        f"Client {client_id} disconnected during broadcast: {exc}"
                    )
                    disconnected_clients.append((client_id, client_info['socket']))
                except Exception as exc:
                    self.logger.error(f"Error sending to client {client_id}: {exc}")
                    disconnected_clients.append((client_id, client_info['socket']))

            for client_id, client_socket in disconnected_clients:
                if self.clients.pop(client_id, None):
                    with suppress(OSError):
                        client_socket.close()

            active_clients = len(self.clients)

        if active_clients:
            self.logger.debug(f"Sent data to {active_clients} clients: {data}")

        return active_clients

    def start(self) -> None:
        """Start the sky anchor server (non-blocking)"""
        try:
            # Create socket
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

            # Allow socket reuse
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

            # Bind to address and port
            self.server_socket.bind((self.host, self.port))

            # Listen for connections
            self.server_socket.listen(100)
            self.running = True

            self.logger.info(f"Sky anchor server listening on {self.host}:{self.port}")
            self.logger.info("Server started. Call tick() method to send data to clients.")

            # Start the client acceptance thread
            self._accept_thread = threading.Thread(
                target=self._accept_clients,
                name="SkyAnchorAcceptThread",
                daemon=True,
            )
            self._accept_thread.start()

        except Exception as e:
            self.logger.error(f"Server error: {e}")
            self.stop()

    def _accept_clients(self) -> None:
        """Internal method to accept client connections in a separate thread"""
        while self.running:
            try:
                # Accept client connection
                client_socket, client_address = self.server_socket.accept()

                # Create a new thread to handle the client
                client_thread = threading.Thread(
                    target=self.handle_client,
                    args=(client_socket, client_address)
                )
                client_thread.daemon = True
                client_thread.start()

            except OSError as exc:
                if self.running:
                    self.logger.error(f"Socket error while accepting clients: {exc}")
                break

    def get_client_count(self) -> int:
        """Get the number of connected clients"""
        with self.client_lock:
            return len(self.clients)

    def stop(self) -> None:
        """Stop the sky anchor server"""
        self.running = False

        # Close all client connections
        with self.client_lock:
            for client_info in self.clients.values():
                client_socket = client_info.get('socket')
                if client_socket:
                    with suppress(OSError):
                        client_socket.close()
            self.clients.clear()

        if self.server_socket:
            with suppress(OSError):
                self.server_socket.close()
            self.server_socket = None

        if self._accept_thread and self._accept_thread.is_alive():
            self._accept_thread.join(timeout=1.0)
            self._accept_thread = None

        self.logger.info("Sky anchor server stopped")
