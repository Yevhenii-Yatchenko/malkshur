"""Flight behaviors extracted from ``DroneController.__updateThrottle``
(GRASP Step 5, HC-1/IE-1) plus the RC setpoint state they feed
(GRASP Step 6, IE-4).

Each behavior consumes typed domain readings and returns
:class:`src.domain.types.AttitudeSetpoints` intents; the controller stays a
thin orchestrator that applies them to :class:`RCSetpoints` (the single
owner of the RC PWM bases and limits) and runs the altitude PID.
"""

from src.flight.intercept import InterceptGuidance, InterceptResult
from src.flight.setpoints import RCSetpoints
from src.flight.stabilization import StabilizationBehavior

__all__ = [
    "InterceptGuidance",
    "InterceptResult",
    "RCSetpoints",
    "StabilizationBehavior",
]
