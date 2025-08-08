from .pid_controller import PIDController


class PositionController:
    """3-axis position controller (x,y,z) using enhanced PID controllers."""

    def __init__(self, x_params: dict, y_params: dict, z_params: dict) -> None:
        self.x = PIDController(**x_params)
        self.y = PIDController(**y_params)
        self.z = PIDController(**z_params)

    def control(
        self,
        x_setpoint: float,
        x_measured: float,
        y_setpoint: float,
        y_measured: float,
        z_setpoint: float,
        z_measured: float,
        dt: float,
    ):
        return (
            self.x.calculate_control(x_setpoint, x_measured, dt),
            self.y.calculate_control(y_setpoint, y_measured, dt),
            self.z.calculate_control(z_setpoint, z_measured, dt),
        )
