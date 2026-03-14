"""Performance profiling for the sky_anchor pipeline."""

from __future__ import annotations

from typing import TYPE_CHECKING

from app.config import PROFILE_SAMPLE_FRAMES

if TYPE_CHECKING:
    from app.logger import UnbufferedLogger


class PipelineProfiler:
    """Profiles a 3-stage pipeline (capture, evaluate, publish) with periodic statistics logging.

    This profiler tracks timing statistics across multiple frames and logs
    aggregated performance metrics at regular intervals.

    Attributes:
        _window_size: Number of frames between log reports (0 to disable profiling)
        _logger: Logger instance for statistics output
    """

    def __init__(self, logger: UnbufferedLogger) -> None:
        """Initialize the pipeline profiler.

        Args:
            logger: Logger instance for statistics output
        """
        self._window_size = PROFILE_SAMPLE_FRAMES if PROFILE_SAMPLE_FRAMES > 0 else 0
        self._logger = logger

        # Profiling state
        self._count = 0
        self._capture_time = 0.0
        self._evaluate_time = 0.0
        self._publish_time = 0.0
        self._total_time = 0.0

    def record_cycle(
        self,
        *,
        capture_start: float,
        capture_end: float,
        evaluate_end: float,
        publish_end: float,
    ) -> None:
        """Record timings for one complete pipeline cycle.

        Accumulates timing data and logs statistics when the window is reached.

        Args:
            capture_start: Timestamp when capture stage began
            capture_end: Timestamp when capture stage ended
            evaluate_end: Timestamp when evaluate stage ended
            publish_end: Timestamp when publish stage ended
        """
        if self._window_size <= 0:
            return

        # Calculate stage durations
        capture_time = capture_end - capture_start
        evaluate_time = evaluate_end - capture_end
        publish_time = publish_end - evaluate_end
        total_time = publish_end - capture_start

        # Accumulate timings
        self._capture_time += capture_time
        self._evaluate_time += evaluate_time
        self._publish_time += publish_time
        self._total_time += total_time
        self._count += 1

        # Log statistics when window is reached
        if self._count >= self._window_size:
            self._log_statistics()
            self._reset_counters()

    def _log_statistics(self) -> None:
        """Log accumulated profiling statistics."""
        frames = self._count

        # Convert to milliseconds for readability
        avg_capture = (self._capture_time / frames) * 1000
        avg_evaluate = (self._evaluate_time / frames) * 1000
        avg_publish = (self._publish_time / frames) * 1000
        avg_total = (self._total_time / frames) * 1000

        # Calculate FPS
        fps = frames / self._total_time if self._total_time > 0 else 0.0

        self._logger.info(
            "Profiling (last %d frames): capture=%.2f ms, evaluate=%.2f ms, "
            "publish=%.2f ms, total=%.2f ms, approx %.1f FPS",
            frames,
            avg_capture,
            avg_evaluate,
            avg_publish,
            avg_total,
            fps,
        )

    def _reset_counters(self) -> None:
        """Reset all profiling counters."""
        self._count = 0
        self._capture_time = 0.0
        self._evaluate_time = 0.0
        self._publish_time = 0.0
        self._total_time = 0.0
