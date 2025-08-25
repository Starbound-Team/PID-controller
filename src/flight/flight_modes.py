"""Flight modes and state machine for drone operation.

Implements the standard flight mode progression with proper state transitions
and safety checks for real-world drone operation.
"""

from __future__ import annotations
from enum import Enum, auto
from typing import Optional
import time


class FlightMode(Enum):
    """Flight mode enumeration with safety progression."""

    DISARMED = auto()
    ARMED = auto()
    STABILIZE = auto()  # Manual attitude control
    ALT_HOLD = auto()  # Altitude hold + manual attitude
    POS_HOLD = auto()  # Position hold
    GUIDED = auto()  # Autonomous navigation
    RTL = auto()  # Return to Launch
    LAND = auto()  # Autonomous landing
    EMERGENCY = auto()  # Emergency stop


class FlightModeManager:
    """Manages flight mode transitions with safety checks."""

    def __init__(self):
        self.current_mode = FlightMode.DISARMED
        self.previous_mode = FlightMode.DISARMED
        self.mode_start_time = time.monotonic()
        self.emergency_triggered = False

    def can_transition_to(
        self,
        new_mode: FlightMode,
        sensors_healthy: bool = True,
        battery_ok: bool = True,
        gps_lock: bool = False,
    ) -> bool:
        """Check if transition to new mode is allowed."""

        # Emergency mode can always be triggered
        if new_mode == FlightMode.EMERGENCY:
            return True

        # Can't leave emergency without disarming first
        if (
            self.current_mode == FlightMode.EMERGENCY
            and new_mode != FlightMode.DISARMED
        ):
            return False

        # Basic safety checks
        if not sensors_healthy or not battery_ok:
            return False

        # Define allowed transitions
        transitions = {
            FlightMode.DISARMED: [FlightMode.ARMED],
            FlightMode.ARMED: [FlightMode.DISARMED, FlightMode.STABILIZE],
            FlightMode.STABILIZE: [
                FlightMode.DISARMED,
                FlightMode.ARMED,
                FlightMode.ALT_HOLD,
            ],
            FlightMode.ALT_HOLD: [
                FlightMode.DISARMED,
                FlightMode.STABILIZE,
                FlightMode.POS_HOLD,
            ],
            FlightMode.POS_HOLD: [
                FlightMode.DISARMED,
                FlightMode.ALT_HOLD,
                FlightMode.GUIDED,
            ],
            FlightMode.GUIDED: [
                FlightMode.DISARMED,
                FlightMode.POS_HOLD,
                FlightMode.RTL,
            ],
            FlightMode.RTL: [FlightMode.DISARMED, FlightMode.LAND],
            FlightMode.LAND: [FlightMode.DISARMED],
        }

        # GPS-dependent modes require GPS lock
        gps_modes = {FlightMode.POS_HOLD, FlightMode.GUIDED, FlightMode.RTL}
        if new_mode in gps_modes and not gps_lock:
            return False

        return new_mode in transitions.get(self.current_mode, [])

    def request_mode_change(self, new_mode: FlightMode, **safety_checks) -> bool:
        """Request mode change with safety validation."""
        if self.can_transition_to(new_mode, **safety_checks):
            self.previous_mode = self.current_mode
            self.current_mode = new_mode
            self.mode_start_time = time.monotonic()
            return True
        return False

    def trigger_emergency(self) -> None:
        """Force emergency mode (always succeeds)."""
        self.previous_mode = self.current_mode
        self.current_mode = FlightMode.EMERGENCY
        self.emergency_triggered = True
        self.mode_start_time = time.monotonic()

    def get_mode_duration(self) -> float:
        """Get time spent in current mode."""
        return time.monotonic() - self.mode_start_time

    def requires_attitude_control(self) -> bool:
        """Check if current mode needs attitude control."""
        return self.current_mode in {
            FlightMode.STABILIZE,
            FlightMode.ALT_HOLD,
            FlightMode.POS_HOLD,
            FlightMode.GUIDED,
            FlightMode.RTL,
            FlightMode.LAND,
        }

    def requires_altitude_control(self) -> bool:
        """Check if current mode needs altitude control."""
        return self.current_mode in {
            FlightMode.ALT_HOLD,
            FlightMode.POS_HOLD,
            FlightMode.GUIDED,
            FlightMode.RTL,
            FlightMode.LAND,
        }

    def requires_position_control(self) -> bool:
        """Check if current mode needs position control."""
        return self.current_mode in {
            FlightMode.POS_HOLD,
            FlightMode.GUIDED,
            FlightMode.RTL,
        }
