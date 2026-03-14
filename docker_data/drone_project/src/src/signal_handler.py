import signal
import sys
from src.logger import get_logger


class SignalHandler:
    """Handles system signals for graceful shutdown of the drone controller."""
    _original_handlers: dict
    shutdown_requested: bool = False

    def __init__(self):
        """
        Initialize the signal handler.
        """
        self.logger = get_logger("signal_handler", "logs/signal_handler.log")

        # Register signal handlers
        self._original_handlers = {}
        self._register_handlers()

    def _register_handlers(self):
        """Register signal handlers for graceful shutdown."""
        signals = [signal.SIGTERM]

        for sig in signals:
            # Store original handler
            self._original_handlers[sig] = signal.signal(sig, self._handle_signal)
            self.logger.info(f"Registered handler for {sig.name}")

    def _handle_signal(self, signum, frame):
        """
        Handle incoming signals.

        Args:
            signum: Signal number
            frame: Current stack frame
        """
        signal_name = signal.Signals(signum).name
        self.logger.warning(f"Received {signal_name} signal")

        if self.shutdown_requested:
            self.logger.error("Forced shutdown - signal received twice")
            sys.exit(1)

        self.shutdown_requested = True
