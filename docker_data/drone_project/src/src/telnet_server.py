import socket
import threading
import json
import queue
import time
from src.logger import get_logger


class TelnetServer:
    """
    Telnet server for drone control commands.
    Accepts commands in the same format as XBee interface.
    """

    def __init__(self, host='0.0.0.0', port=23):
        self.host = host
        self.port = port
        self.server_socket = None
        self.clients = []
        self.message_queue = queue.Queue()
        self.running = False
        self.logger = get_logger("telnet_server", "logs/telnet_server.log", log_level="INFO")

    def start(self):
        """Start the telnet server."""
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind((self.host, self.port))
            self.server_socket.listen(5)
            self.running = True

            self.logger.info(f"Telnet server started on {self.host}:{self.port}")

            # Start server thread
            server_thread = threading.Thread(target=self._accept_connections, daemon=True)
            server_thread.start()

        except Exception as e:
            self.logger.error(f"Failed to start telnet server: {e}")

    def stop(self):
        """Stop the telnet server."""
        self.running = False
        if self.server_socket:
            self.server_socket.close()

        # Close all client connections
        for client in self.clients.copy():
            self._disconnect_client(client)

        self.logger.info("Telnet server stopped")

    def _accept_connections(self):
        """Accept incoming telnet connections."""
        while self.running:
            try:
                client_socket, client_address = self.server_socket.accept()
                self.logger.info(f"New telnet connection from {client_address}")

                client_info = {
                    'socket': client_socket,
                    'address': client_address,
                    'thread': None
                }

                # Start client handler thread
                client_thread = threading.Thread(
                    target=self._handle_client,
                    args=(client_info,),
                    daemon=True
                )
                client_info['thread'] = client_thread

                self.clients.append(client_info)
                client_thread.start()

                # Send welcome message
                self._send_to_client(client_info, self._get_welcome_message())
                self._send_to_client(client_info, "drone> ")

            except Exception as e:
                if self.running:
                    self.logger.error(f"Error accepting connection: {e}")

    def _handle_client(self, client_info):
        """Handle individual client connection."""
        client_socket = client_info['socket']
        client_address = client_info['address']

        try:
            buffer = ""

            while self.running:
                try:
                    data = client_socket.recv(1024).decode('utf-8')
                    if not data:
                        break

                    buffer += data

                    # Process complete lines
                    while '\n' in buffer or '\r' in buffer:
                        if '\r\n' in buffer:
                            line, buffer = buffer.split('\r\n', 1)
                        elif '\n' in buffer:
                            line, buffer = buffer.split('\n', 1)
                        elif '\r' in buffer:
                            line, buffer = buffer.split('\r', 1)

                        line = line.strip()
                        if line:
                            response = self._process_command(line, client_info)
                            if response:
                                self._send_to_client(client_info, response + "\n")

                        self._send_to_client(client_info, "drone> ")

                except socket.timeout:
                    continue
                except ConnectionResetError:
                    break

        except Exception as e:
            self.logger.error(f"Error handling client {client_address}: {e}")
        finally:
            self._disconnect_client(client_info)

    def _process_command(self, command, client_info):
        """Process a command from telnet client."""
        try:
            command = command.strip()

            if not command:
                return ""

            # Handle special telnet commands
            if command.lower() in ['quit', 'exit', 'bye']:
                self._send_to_client(client_info, "Goodbye!\n")
                self._disconnect_client(client_info)
                return ""

            if command.lower() in ['help', '?']:
                return self._get_help_message()

            if command.lower() == 'status':
                return self._get_status_message()

            # Handle console commands (starting with 'cmd,')
            if command.startswith('cmd,'):
                # Format as JSON message like XBee interface
                json_message = json.dumps({"msg": command})
                self.message_queue.put(json_message)
                return f"Console command queued: {command[4:]}"

            # Handle drone control commands
            # Parse command format: "command,param1,param2,..."
            if ',' in command:
                # Direct command format
                json_message = json.dumps({"msg": command})
                self.message_queue.put(json_message)
                return f"Command queued: {command}"
            else:
                # Single word commands - add empty parameters
                json_message = json.dumps({"msg": command + ","})
                self.message_queue.put(json_message)
                return f"Command queued: {command}"

        except Exception as e:
            self.logger.error(f"Error processing command '{command}': {e}")
            return f"Error: {str(e)}"

    def _send_to_client(self, client_info, message):
        """Send message to specific client."""
        try:
            client_info['socket'].send(message.encode('utf-8'))
        except Exception as e:
            self.logger.error(f"Error sending to client {client_info['address']}: {e}")
            self._disconnect_client(client_info)

    def _disconnect_client(self, client_info):
        """Disconnect a client."""
        try:
            if client_info in self.clients:
                self.clients.remove(client_info)

            client_info['socket'].close()
            self.logger.info(f"Client {client_info['address']} disconnected")
        except Exception as e:
            self.logger.error(f"Error disconnecting client: {e}")

    def _get_welcome_message(self):
        """Get welcome message for new connections."""
        return """
========================================
    DRONE CONTROL TELNET INTERFACE
========================================
Welcome! You can now send commands to control the drone.

Type 'help' for available commands.
Type 'quit' or 'exit' to disconnect.

Command format: command,param1,param2,...
Example: setHeight,1.5
         square,1

"""

    def _get_help_message(self):
        """Get help message with available commands."""
        return """
Available Commands:
==================

Drone Control:
  arm,0           - Arm the drone
  arm,1           - Disarm the drone
  takeoff,<alt>   - Takeoff to altitude (meters)
  land,           - Land the drone
  setHeight,<alt> - Set target altitude (meters)
  move,<t>,<p>,<r>,<y> - Move drone (throttle,pitch,roll,yaw)
  square,         - Fly in square pattern
  returnControl,  - Return control to drone

Flight Controller:
  mode,<mode>     - Set flight mode (STABILIZE, ALT_HOLD, GUIDED, etc.)
  reboot,         - Reboot flight controller

System Commands:
  cmd,<command>   - Execute console command
  status          - Show drone status
  help            - Show this help
  quit/exit       - Disconnect

Examples:
  takeoff,2.5     - Takeoff to 2.5 meters
  arm,0           - Arm the drone
  setHeight,1.0   - Set altitude to 1 meter
  mode,ALT_HOLD   - Switch to altitude hold mode
  cmd,ls -la      - Execute 'ls -la' command

"""

    def _get_status_message(self):
        """Get current drone status message."""
        return f"""
Drone Status:
=============
Telnet Server: Running on {self.host}:{self.port}
Connected Clients: {len(self.clients)}
Message Queue Size: {self.message_queue.qsize()}
Server Time: {time.strftime('%Y-%m-%d %H:%M:%S')}

Note: For detailed drone status, use the main controller interface.
"""

    def send_response(self, message):
        """Send response to all connected clients."""
        if self.clients:
            response = f"[DRONE] {message}\n"
            for client in self.clients.copy():
                self._send_to_client(client, response)

    def broadcast_message(self, message):
        """Broadcast message to all connected clients."""
        if self.clients:
            broadcast = f"[BROADCAST] {message}\n"
            for client in self.clients.copy():
                self._send_to_client(client, broadcast)
