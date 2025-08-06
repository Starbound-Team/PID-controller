class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.previous_error = 0
        self.integral = 0

    def calculate(self, setpoint, measured_value, dt):
        error = setpoint - measured_value
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previous_error = error
        return output


class AttitudeController(PIDController):
    def __init__(
        self,
        kp_roll,
        ki_roll,
        kd_roll,
        kp_pitch,
        ki_pitch,
        kd_pitch,
        kp_yaw,
        ki_yaw,
        kd_yaw,
    ):
        super().__init__(kp_roll, ki_roll, kd_roll)
        self.pitch_controller = PIDController(kp_pitch, ki_pitch, kd_pitch)
        self.yaw_controller = PIDController(kp_yaw, ki_yaw, kd_yaw)

    def control(
        self,
        roll_setpoint,
        roll_measured,
        pitch_setpoint,
        pitch_measured,
        yaw_setpoint,
        yaw_measured,
        dt,
    ):
        roll_output = self.calculate(roll_setpoint, roll_measured, dt)
        pitch_output = self.pitch_controller.calculate(
            pitch_setpoint, pitch_measured, dt
        )
        yaw_output = self.yaw_controller.calculate(yaw_setpoint, yaw_measured, dt)
        return roll_output, pitch_output, yaw_output
