class PIDController:
    """Generic PID controller with robustness features.

    Features:
        * Integral windup protection via clamping
        * Optional output saturation (with integrator freeze when saturated)
        * Derivative on measurement (reduces derivative kick) or on error
        * dt validation (guards divide-by-zero / negative dt)
    """

    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        integral_limits: tuple | None = None,
        output_limits: tuple | None = None,
        derivative_on_measurement: bool = True,
        freeze_integrator_on_saturation: bool = True,
        derivative_filter_tau: float | None = None,
    ) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limits = integral_limits  # (min, max) or None
        self.output_limits = output_limits  # (min, max) or None
        self.derivative_on_measurement = derivative_on_measurement
        self.freeze_integrator_on_saturation = freeze_integrator_on_saturation
        self.derivative_filter_tau = derivative_filter_tau  # seconds (time constant)

        self.previous_error = 0.0
        self.previous_measurement = None
        self.integral = 0.0

    def reset(self) -> None:
        """Reset dynamic state (integral and previous terms)."""
        self.previous_error = 0.0
        self.previous_measurement = None
        self.integral = 0.0

    def set_parameters(self, kp: float, ki: float, kd: float) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def _apply_limits(self, value: float, limits: tuple | None) -> float:
        if limits is None:
            return value
        lo, hi = limits
        if lo is not None and value < lo:
            return lo
        if hi is not None and value > hi:
            return hi
        return value

    def calculate_control(self, setpoint: float, measured_value: float, dt: float) -> float:
        """Compute PID control output.

        Args:
            setpoint: Desired target value.
            measured_value: Current measured process value.
            dt: Elapsed time since last update (seconds).
        Returns:
            Control output (possibly saturated if output_limits supplied).
        """
        if dt <= 0:
            # Do not update state if invalid dt; return last proportional only as safe fallback
            error = setpoint - measured_value
            return self.kp * error

        error = setpoint - measured_value

        # Derivative term computation
    # Derivative term computation
        if self.derivative_on_measurement:
            if self.previous_measurement is None:
                raw_derivative = 0.0
            else:
                raw_derivative = -(measured_value - self.previous_measurement) / dt
            self.previous_measurement = measured_value
        else:
            raw_derivative = (error - self.previous_error) / dt

        # Optional first-order low-pass filter on derivative term: D_f = D_f + alpha*(raw - D_f)
        if self.derivative_filter_tau and self.derivative_filter_tau > 0:
            if not hasattr(self, "_derivative_filtered"):
                self._derivative_filtered = raw_derivative
            alpha = dt / (self.derivative_filter_tau + dt)
            self._derivative_filtered = self._derivative_filtered + alpha * (
                raw_derivative - self._derivative_filtered
            )
            derivative = self._derivative_filtered
        else:
            derivative = raw_derivative

        # Provisional integral update (will adjust if saturation & freeze policy)
        new_integral = self.integral + error * dt

        # Apply integral limits if provided
        if self.integral_limits is not None:
            lo_i, hi_i = self.integral_limits
            if lo_i is not None and new_integral < lo_i:
                new_integral = lo_i
            if hi_i is not None and new_integral > hi_i:
                new_integral = hi_i

        # Compute raw output
        output = self.kp * error + self.ki * new_integral + self.kd * derivative

        saturated = False
        if self.output_limits is not None:
            limited_output = self._apply_limits(output, self.output_limits)
            if limited_output != output:
                saturated = True
                output = limited_output

        # Decide integrator acceptance
        accept_integral = True
        if (
            saturated
            and self.freeze_integrator_on_saturation
            and self.ki != 0.0
        ):
            # Predict unsaturated output contribution signs
            proportional = self.kp * error
            integral_contrib = self.ki * new_integral
            derivative_contrib = self.kd * derivative
            # If integral contribution has same sign as (output - limited_output) we freeze
            if self.output_limits is not None:
                lo, hi = self.output_limits
                if hi is not None and output >= hi and integral_contrib > 0:
                    accept_integral = False
                if lo is not None and output <= lo and integral_contrib < 0:
                    accept_integral = False
        if accept_integral:
            self.integral = new_integral

        self.previous_error = error
        return output

