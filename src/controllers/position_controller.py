class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self.previous_error = 0

    def update(self, setpoint, measured_value, dt):
        error = setpoint - measured_value
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previous_error = error
        return output


class PositionController(PIDController):
    def __init__(self, kp, ki, kd):
        super().__init__(kp, ki, kd)

    def control(self, target_position, current_position, dt):
        control_x = super().update(target_position[0], current_position[0], dt)
        control_y = super().update(target_position[1], current_position[1], dt)
        control_z = super().update(target_position[2], current_position[2], dt)
        return control_x, control_y, control_z
