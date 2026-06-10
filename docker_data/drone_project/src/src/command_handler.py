#!/usr/bin/env python3
"""
Command Handler
Handles parsing and dispatching of drone control commands
"""

import json
import threading
import time
from typing import Optional, Tuple, List, Callable, Dict, Any

from src.logger import get_logger
from src.telnet_server import TelnetServer
from src import controller_config


class CommandHandler:
    """
    Handles command parsing, validation, and dispatching.

    Responsibilities:
    - Parse JSON messages from external interfaces
    - Split command strings into key and parameters
    - Dispatch commands to registered handlers
    - Provide response formatting for telnet clients
    - Run Telnet command processing in separate thread
    """

    def __init__(self, logger=None, telnet_host: str = '0.0.0.0', telnet_port: int = 2323,
                 telnet_server: Optional[TelnetServer] = None) -> None:
        """
        Initialize command handler.

        Args:
            logger: Optional logger instance (creates own if not provided)
            telnet_host: Host for Telnet server (fallback construction and
                the startup log line)
            telnet_port: Port for Telnet server (fallback construction and
                the startup log line; the command port is 2323)
            telnet_server: Optional injected TelnetServer (GRASP Step 7,
                LC-3; the composition root passes one explicitly, together
                with matching telnet_host/telnet_port).  If None (default),
                a TelnetServer(telnet_host, telnet_port) is created exactly
                as before.  Either way the handler starts it here, so the
                listener comes up at the same moment as always.
        """
        self.__logger = logger or get_logger(
            "command_handler",
            "logs/command_handler.log",
            log_level=controller_config.LOG_LEVEL
        )
        self.__command_map: Dict[str, Callable] = {}

        if telnet_server is None:
            telnet_server = TelnetServer(host=telnet_host, port=telnet_port)
        self.__telnet_server = telnet_server
        self.__telnet_server.start()
        self.__logger.info(f"Telnet server started on {telnet_host}:{telnet_port}")

        self.__running = False
        self.__thread: Optional[threading.Thread] = None
        self.__check_interval = 0.1

    def register_command(self, key: str, handler: Callable) -> None:
        """
        Register a command handler.

        Args:
            key: Command key (e.g., 'mode', 'arm', 'setHeight')
            handler: Callable that accepts params list
        """
        self.__command_map[key] = handler
        self.__logger.debug(f"Registered command: {key}")

    def parse_message(self, message: str) -> Tuple[Optional[str], Optional[List]]:
        """
        Parse a command message string into key and parameters.

        Handles both integer and string parameters gracefully.

        Args:
            message: Command string (e.g., "setHeight,5" or "mode,GUIDED")

        Returns:
            Tuple of (key, params_list) or (None, None) on error
        """
        try:
            parts = message.split(',')
            key = parts[0]

            # Try to parse as integers first, fall back to strings
            values = []
            for x in parts[1:]:
                if not x:
                    values.append(0)
                else:
                    try:
                        values.append(int(x))
                    except ValueError:
                        values.append(x)

            self.__logger.debug(f"Parsed command - Key: {key}, Values: {values}")
            return key, values

        except Exception as e:
            self.__logger.error(f"Error parsing message '{message}': {e}")
            return None, None

    def parse_json_message(self, json_message: str) -> Optional[str]:
        """
        Parse JSON message to extract command string.

        Args:
            json_message: JSON string with 'msg' field

        Returns:
            Command string or None on error
        """
        try:
            data = json.loads(json_message)
            return data.get("msg")
        except json.JSONDecodeError as e:
            self.__logger.error(f"Error decoding JSON message: {e}")
            return None
        except Exception as e:
            self.__logger.error(f"Error parsing JSON message: {e}")
            return None

    def execute_command(self, key: str, params: List) -> Dict[str, Any]:
        """
        Execute a registered command.

        Args:
            key: Command key
            params: Command parameters

        Returns:
            Dict with 'success' bool and optional 'message' string
        """
        if key is None:
            return {'success': False, 'message': 'Invalid command key'}

        handler = self.__command_map.get(key)

        if handler is None:
            self.__logger.error(f"Command '{key}' is not implemented")
            return {'success': False, 'message': f"Command '{key}' not found"}

        try:
            self.__logger.info(f"Executing command: {key} with params: {params}")
            handler(params)
            return {'success': True, 'message': f"Executed: {key} with params {params}"}

        except Exception as e:
            self.__logger.error(f"Error executing command '{key}': {e}")
            return {'success': False, 'message': f"Error: {str(e)}"}

    def process_json_command(self, json_message: str) -> Dict[str, Any]:
        """
        Process a complete JSON command message.

        Combines JSON parsing, message parsing, and command execution.

        Args:
            json_message: JSON string with command

        Returns:
            Dict with execution result
        """
        # Parse JSON
        command_string = self.parse_json_message(json_message)
        if command_string is None:
            return {'success': False, 'message': 'Invalid JSON message'}

        # Parse command string
        key, params = self.parse_message(command_string)
        if key is None:
            return {'success': False, 'message': 'Invalid command format'}

        # Execute command
        return self.execute_command(key, params)

    def get_registered_commands(self) -> List[str]:
        """
        Get list of registered command keys.

        Returns:
            List of command keys
        """
        return list(self.__command_map.keys())

    def start_telnet_processing(self) -> None:
        """Start Telnet command processing in background thread."""
        if self.__running:
            self.__logger.warning("Telnet processing already running")
            return

        self.__running = True
        self.__thread = threading.Thread(target=self.__telnet_processing_loop, daemon=True)
        self.__thread.start()
        self.__logger.info("Telnet command processing thread started")

    def stop_telnet_processing(self) -> None:
        """Stop Telnet command processing thread."""
        self.__running = False
        if self.__thread and self.__thread.is_alive():
            self.__thread.join(timeout=2.0)
        self.__logger.info("Telnet command processing thread stopped")

    def __telnet_processing_loop(self) -> None:
        """Background thread loop for processing Telnet commands."""
        while self.__running:
            try:
                if not self.__telnet_server.message_queue.empty():
                    message = self.__telnet_server.message_queue.get()
                    self.__logger.info(f"Processing Telnet message: {message}")

                    result = self.process_json_command(message)
                    response_msg = result.get('message', 'Command processed')
                    self.__telnet_server.send_response(response_msg)

                time.sleep(self.__check_interval)

            except Exception as e:
                self.__logger.error(f"Error in Telnet processing loop: {e}")
                time.sleep(self.__check_interval)

    def cleanup(self) -> None:
        """Clean up resources."""
        self.stop_telnet_processing()
        self.__telnet_server.stop()
        self.__logger.info("CommandHandler cleanup complete")