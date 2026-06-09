from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Optional

from ..config import ANGLE_THRESHOLD, SHIFT_THRESHOLD
from ..logger import UnbufferedLogger
from .parser import ParsedImage
from .estimator import ShiftEstimator, get_shift_estimator


@dataclass
class ShiftCommand:
    """One drift measurement/command published on the TCP :8888 wire.

    ``navigation`` is the explicit mode flag (GRASP Step 4, IE-3): True only
    on commands modified by the navigation CommandModifier.  It replaces the
    magic ``matches_percent == 101.0`` sentinel as the control signal; the
    101.0 placeholder is still emitted alongside during navigation so the
    numeric matches_percent stream (CSV logs, plots) stays unchanged, but
    consumers must key on ``navigation`` only.
    """

    dx: int
    dy: int
    angle_deg: int
    matches_percent: int
    target_dx_pixels: Optional[float] = None
    target_dy_pixels: Optional[float] = None
    navigation: bool = False

    def to_payload(self) -> Dict[str, Any]:
        payload: Dict[str, Any] = {
            "dx": self.dx,
            "dy": self.dy,
            "angle_deg": self.angle_deg,
            "matches_percent": self.matches_percent,
            "navigation": self.navigation,
        }
        if self.target_dx_pixels is not None:
            payload["target_dx_pixels"] = self.target_dx_pixels
        if self.target_dy_pixels is not None:
            payload["target_dy_pixels"] = self.target_dy_pixels
        return payload

    def __str__(self) -> str:
        return (
            f"(dx={self.dx:.1f}, dy={self.dy:.1f})"
        )


class ShiftEvaluator:
    """Evaluate shift between parsed frames applying configured thresholds."""

    def __init__(
        self,
        logger: UnbufferedLogger,
    ) -> None:
        self._estimator: ShiftEstimator = get_shift_estimator(logger)
        self._shift_threshold_low = SHIFT_THRESHOLD
        self._shift_threshold_high = SHIFT_THRESHOLD * 100
        self._angle_threshold = ANGLE_THRESHOLD

    def evaluate(self, reference: ParsedImage, current: ParsedImage) -> ShiftCommand:
        dx, dy, angle_deg, matches_percent = self._estimator.estimate_shift(reference, current)
        return ShiftCommand(
            dx=self._apply_deadband(dx, self._shift_threshold_low, self._shift_threshold_high),
            dy=self._apply_deadband(dy, self._shift_threshold_low, self._shift_threshold_high),
            angle_deg=self._apply_deadband(angle_deg, self._angle_threshold, self._angle_threshold * 100),
            matches_percent=int(matches_percent),
        )

    @staticmethod
    def _apply_deadband(value: float, threshold_low: float, threshold_high: float) -> int:
        if abs(value) >= threshold_high:
            if value > 0:
                return int(threshold_high)
            else:
                return int(-threshold_high)
        return 0 if abs(value) <= threshold_low else int(value)