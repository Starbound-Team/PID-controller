from .pid_controller import PIDController


class AttitudeController:
    """Composite attitude controller managing roll, pitch, yaw using enhanced PID controllers."""

    def __init__(
        self,
        roll_params: dict,
        pitch_params: dict,
        yaw_params: dict,
    ) -> None:
        self.roll = PIDController(**roll_params)
        self.pitch = PIDController(**pitch_params)
        self.yaw = PIDController(**yaw_params)

    def control(
        self,
        roll_setpoint: float,
        roll_measured: float,
        pitch_setpoint: float,
        pitch_measured: float,
        yaw_setpoint: float,
        yaw_measured: float,
        dt: float,
    ):
        return (
            self.roll.calculate_control(roll_setpoint, roll_measured, dt),
            self.pitch.calculate_control(pitch_setpoint, pitch_measured, dt),
            self.yaw.calculate_control(yaw_setpoint, yaw_measured, dt),
        )
