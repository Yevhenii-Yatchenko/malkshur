"""Flight behaviors extracted from ``DroneController.__updateThrottle``
(GRASP Step 5, HC-1/IE-1).

Each behavior consumes typed domain readings and returns
:class:`src.domain.types.AttitudeSetpoints` intents; the controller stays a
thin orchestrator that applies them to its RC bases and runs the altitude
PID.
"""

from src.flight.intercept import InterceptGuidance
from src.flight.stabilization import StabilizationBehavior

__all__ = ["InterceptGuidance", "StabilizationBehavior"]
