from .pid_controller import PIDController


class VelocityController:
    """Single-axis velocity controller using enhanced PIDController."""

    def __init__(self, params: dict) -> None:
        self.pid = PIDController(**params)

    def control_velocity(self, setpoint_velocity: float, current_velocity: float, dt: float):
        return self.pid.calculate_control(setpoint_velocity, current_velocity, dt)
