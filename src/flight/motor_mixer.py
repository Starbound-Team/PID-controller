"""Motor mixer for tri-propeller VTOL configuration.

Converts attitude and thrust commands into individual motor outputs
for a three-motor VTOL configuration with proper geometric mixing.
"""

from __future__ import annotations
import numpy as np
from typing import Tuple, List, Optional
from dataclasses import dataclass


@dataclass
class MotorCommand:
    """Individual motor command with safety limits."""

    motor_id: int
    throttle: float  # 0.0 to 1.0
    armed: bool = False

    def __post_init__(self):
        """Ensure throttle is within safe bounds."""
        self.throttle = max(0.0, min(1.0, self.throttle))


class TriPropMixer:
    """Motor mixer for tri-propeller VTOL configuration.

    Assumes three motors in Y-configuration:
    - Motor 0: Front (roll=0°)
    - Motor 1: Rear-Left (roll=120°)
    - Motor 2: Rear-Right (roll=240°)
    """

    def __init__(
        self,
        motor_positions: Optional[List[Tuple[float, float]]] = None,
        max_thrust: float = 1.0,
        min_throttle: float = 0.0,
        hover_throttle: float = 0.5,
    ):
        """
        Args:
            motor_positions: List of (x, y) positions in meters relative to CG
            max_thrust: Maximum thrust per motor (normalized to 1.0)
            min_throttle: Minimum throttle for armed motors
            hover_throttle: Estimated hover throttle
        """
        # Default Y-configuration positions (normalized)
        if motor_positions is None:
            arm_length = 0.25  # 25cm arm length
            motor_positions = [
                (arm_length, 0.0),  # Front motor
                (-arm_length / 2, -arm_length * np.sqrt(3) / 2),  # Rear-left
                (-arm_length / 2, arm_length * np.sqrt(3) / 2),  # Rear-right
            ]

        self.motor_positions = motor_positions
        self.max_thrust = max_thrust
        self.min_throttle = min_throttle
        self.hover_throttle = hover_throttle
        self.num_motors = len(motor_positions)

        # Pre-compute mixing matrix for efficiency
        self._compute_mixing_matrix()

    def _compute_mixing_matrix(self) -> None:
        """Compute the mixing matrix from control inputs to motor outputs."""
        # Control inputs: [thrust, roll_moment, pitch_moment, yaw_moment]
        # Motor outputs: [motor0, motor1, motor2, ...]

        self.mix_matrix = np.zeros((self.num_motors, 4))

        for i, (x, y) in enumerate(self.motor_positions):
            # Thrust contribution (all motors contribute equally)
            self.mix_matrix[i, 0] = 1.0 / self.num_motors

            # Roll moment (torque about x-axis from y-position)
            self.mix_matrix[i, 1] = -y  # Negative for right-hand rule

            # Pitch moment (torque about y-axis from x-position)
            self.mix_matrix[i, 2] = x  # Positive pitch-up

            # Yaw moment (alternating motor direction for tri-config)
            # Assuming motors 0,2 CW and motor 1 CCW
            yaw_sign = -1.0 if i == 1 else 1.0  # CCW motor produces negative yaw torque
            self.mix_matrix[i, 3] = yaw_sign

        # Normalize moment columns to prevent saturation issues
        for col in range(1, 4):
            col_max = np.max(np.abs(self.mix_matrix[:, col]))
            if col_max > 0:
                self.mix_matrix[:, col] /= col_max

    def mix_controls(
        self,
        thrust: float,  # Normalized thrust (0-1)
        roll_moment: float,  # Roll torque command (-1 to 1)
        pitch_moment: float,  # Pitch torque command (-1 to 1)
        yaw_moment: float,  # Yaw torque command (-1 to 1)
        armed: bool = False,
    ) -> List[MotorCommand]:
        """Convert control commands to individual motor outputs."""

        # Clamp inputs to safe ranges
        thrust = max(0.0, min(1.0, thrust))
        roll_moment = max(-1.0, min(1.0, roll_moment))
        pitch_moment = max(-1.0, min(1.0, pitch_moment))
        yaw_moment = max(-1.0, min(1.0, yaw_moment))

        # Control vector
        controls = np.array([thrust, roll_moment, pitch_moment, yaw_moment])

        # Apply mixing matrix
        motor_outputs = self.mix_matrix @ controls

        # Apply thrust scaling and limits
        motor_commands = []
        for i, output in enumerate(motor_outputs):
            if armed and thrust > 0.01:
                # Scale to throttle range and apply minimum
                throttle = max(self.min_throttle, output * self.max_thrust)
            else:
                # Disarmed or zero thrust
                throttle = 0.0

            motor_commands.append(
                MotorCommand(motor_id=i, throttle=throttle, armed=armed)
            )

        return motor_commands

    def get_mixing_matrix(self) -> np.ndarray:
        """Get the computed mixing matrix for analysis."""
        return self.mix_matrix.copy()

    def check_control_authority(
        self, thrust: float, roll_moment: float, pitch_moment: float, yaw_moment: float
    ) -> Tuple[bool, float]:
        """Check if control command is achievable without saturation.

        Returns:
            (achievable, saturation_margin): True if achievable, margin to saturation
        """
        controls = np.array([thrust, roll_moment, pitch_moment, yaw_moment])
        motor_outputs = self.mix_matrix @ controls

        # Check for over-saturation (>1.0) or under-saturation (<0.0)
        max_output = np.max(motor_outputs)
        min_output = np.min(motor_outputs)

        achievable = (max_output <= 1.0) and (min_output >= 0.0)

        # Calculate margin to saturation
        margin_high = 1.0 - max_output
        margin_low = min_output - 0.0
        margin = min(margin_high, margin_low)

        return achievable, margin

    def auto_scale_controls(
        self, thrust: float, roll_moment: float, pitch_moment: float, yaw_moment: float
    ) -> Tuple[float, float, float, float]:
        """Automatically scale control commands to prevent saturation."""
        controls = np.array([thrust, roll_moment, pitch_moment, yaw_moment])
        motor_outputs = self.mix_matrix @ controls

        # Find maximum scaling factor needed
        max_output = np.max(motor_outputs)
        min_output = np.min(motor_outputs)

        if max_output > 1.0:
            scale_factor = 1.0 / max_output
        elif min_output < 0.0:
            scale_factor = 1.0 / (1.0 - min_output)
        else:
            scale_factor = 1.0  # No scaling needed

        # Apply scaling while preserving thrust priority
        if scale_factor < 1.0:
            # Scale moments more aggressively than thrust
            thrust_scaled = thrust * max(0.9, scale_factor)
            roll_scaled = roll_moment * scale_factor
            pitch_scaled = pitch_moment * scale_factor
            yaw_scaled = yaw_moment * scale_factor
        else:
            thrust_scaled = thrust
            roll_scaled = roll_moment
            pitch_scaled = pitch_moment
            yaw_scaled = yaw_moment

        return thrust_scaled, roll_scaled, pitch_scaled, yaw_scaled
