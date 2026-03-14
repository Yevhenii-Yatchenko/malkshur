import os
import sys
import datetime
from typing import Optional, TextIO


class UnbufferedLogger:
    """Custom logger that writes to a log file without buffering."""

    def __init__(self, log_file_path: str, log_level: str = "INFO", console_output: bool = True):
        """
        Initialize the unbuffered logger.

        Args:
            log_file_path: Path to the log file
            log_level: Logging level (DEBUG, INFO, WARNING, ERROR, CRITICAL)
            console_output: Whether to also print to console
        """
        self.log_file_path = log_file_path
        self.log_level = log_level
        self.console_output = console_output
        self.log_levels = {
            "DEBUG": 10,
            "INFO": 20,
            "WARNING": 30,
            "ERROR": 40,
            "CRITICAL": 50
        }
        self.current_level = self.log_levels.get(log_level.upper(), 20)

        # Create log directory if it doesn't exist
        log_dir = os.path.dirname(log_file_path)
        if log_dir and not os.path.exists(log_dir):
            os.makedirs(log_dir)

        # Open file in unbuffered mode (line buffering for text files)
        self.file_handle: Optional[TextIO] = None
        self._open_file()

    def _open_file(self):
        """Open the log file in unbuffered mode."""
        try:
            # Use line buffering (1) for text files - this ensures each line is written immediately
            self.file_handle = open(self.log_file_path, 'a', buffering=1, encoding='utf-8')
        except IOError as e:
            print(f"Failed to open log file {self.log_file_path}: {e}", file=sys.stderr)
            self.file_handle = None

    def _format_message(self, level: str, message: str) -> str:
        """Format the log message with timestamp and level."""
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        return f"[{level}] {timestamp}: {message}"

    def _write(self, level: str, message: str):
        """Write a log message if the level is appropriate."""
        level_value = self.log_levels.get(level.upper(), 20)
        if level_value < self.current_level:
            return

        formatted_message = self._format_message(level, message)

        # Write to file if handle is available
        if self.file_handle:
            try:
                self.file_handle.write(formatted_message + '\n')
                # Force flush to ensure immediate writing
                self.file_handle.flush()
                # Also flush OS buffer to disk
                os.fsync(self.file_handle.fileno())
            except IOError as e:
                print(f"Failed to write to log file: {e}", file=sys.stderr)

        # Write to console if enabled
        if self.console_output:
            print(formatted_message)

    def debug(self, message: str):
        """Log a debug message."""
        self._write("DEBUG", message)

    def info(self, message: str):
        """Log an info message."""
        self._write("INFO", message)

    def warning(self, message: str):
        """Log a warning message."""
        self._write("WARNING", message)

    def error(self, message: str):
        """Log an error message."""
        self._write("ERROR", message)

    def critical(self, message: str):
        """Log a critical message."""
        self._write("CRITICAL", message)

    def set_level(self, level: str):
        """Change the logging level."""
        self.current_level = self.log_levels.get(level.upper(), 20)

    def close(self):
        """Close the log file handle."""
        if self.file_handle:
            try:
                self.file_handle.close()
            except IOError:
                pass
            self.file_handle = None

    def __enter__(self):
        """Context manager entry."""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.close()

    def __del__(self):
        """Destructor to ensure file is closed."""
        self.close()


# Singleton instance management
_logger_instances = {}


def get_logger(name: str = "default",
               log_file_path: Optional[str] = None,
               log_level: str = "INFO",
               console_output: bool = True) -> UnbufferedLogger:
    """
    Get or create a logger instance.

    Args:
        name: Logger name (for singleton management)
        log_file_path: Path to log file (defaults to logs/custom.log)
        log_level: Logging level
        console_output: Whether to also print to console

    Returns:
        UnbufferedLogger instance
    """
    if name not in _logger_instances:
        if log_file_path is None:
            log_file_path = os.path.join("logs", f"{name}.log")
        _logger_instances[name] = UnbufferedLogger(log_file_path, log_level, console_output)
    return _logger_instances[name]