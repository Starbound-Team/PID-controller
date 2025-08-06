#!/usr/bin/env python3
"""
Interactive PID tuning tool with real-time visualization
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from src.controllers.pid_controller import PIDController


class InteractivePIDTuner:
    def __init__(self):
        self.fig, self.ax = plt.subplots(figsize=(12, 8))
        plt.subplots_adjust(bottom=0.35)

        # Initial PID parameters
        self.kp_init = 1.0
        self.ki_init = 0.1
        self.kd_init = 0.05

        # Simulation parameters
        self.dt = 0.01
        self.time_end = 5.0
        self.time_steps = int(self.time_end / self.dt)
        self.time_array = np.linspace(0, self.time_end, self.time_steps)

        # Create sliders
        self.create_sliders()

        # Initial plot
        self.update_plot(None)

        plt.show()

    def create_sliders(self):
        """Create slider controls for PID parameters"""

        # Slider axes
        ax_kp = plt.axes([0.2, 0.15, 0.5, 0.03])
        ax_ki = plt.axes([0.2, 0.10, 0.5, 0.03])
        ax_kd = plt.axes([0.2, 0.05, 0.5, 0.03])

        # Create sliders
        self.slider_kp = Slider(ax_kp, "Kp", 0.1, 5.0, valinit=self.kp_init)
        self.slider_ki = Slider(ax_ki, "Ki", 0.0, 1.0, valinit=self.ki_init)
        self.slider_kd = Slider(ax_kd, "Kd", 0.0, 1.0, valinit=self.kd_init)

        # Connect sliders to update function
        self.slider_kp.on_changed(self.update_plot)
        self.slider_ki.on_changed(self.update_plot)
        self.slider_kd.on_changed(self.update_plot)

    def simulate_response(self, kp, ki, kd):
        """Simulate system response with given PID parameters"""

        pid = PIDController(kp, ki, kd)

        # Setpoint (step input)
        setpoint = 10.0
        setpoint_array = np.ones(self.time_steps) * setpoint

        # Initialize arrays
        measured_array = np.zeros(self.time_steps)
        control_array = np.zeros(self.time_steps)

        # Simple plant simulation
        measured_value = 0.0
        plant_time_constant = 0.5

        for i in range(self.time_steps):
            control_output = pid.calculate_control(setpoint, measured_value, self.dt)
            control_array[i] = control_output

            # Plant response
            measured_value += (
                (control_output - measured_value) * self.dt / plant_time_constant
            )
            measured_array[i] = measured_value

        return setpoint_array, measured_array, control_array

    def update_plot(self, val):
        """Update plot when sliders change"""

        # Get current slider values
        kp = self.slider_kp.val
        ki = self.slider_ki.val
        kd = self.slider_kd.val

        # Simulate
        setpoint_array, measured_array, control_array = self.simulate_response(
            kp, ki, kd
        )

        # Clear and plot
        self.ax.clear()

        self.ax.plot(
            self.time_array, setpoint_array, "r--", label="Setpoint", linewidth=2
        )
        self.ax.plot(
            self.time_array, measured_array, "b-", label="Response", linewidth=2
        )

        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("Value")
        self.ax.set_title(f"PID Response (Kp={kp:.2f}, Ki={ki:.2f}, Kd={kd:.2f})")
        self.ax.legend()
        self.ax.grid(True)

        # Calculate and display metrics
        settling_time = self.calculate_settling_time(measured_array, setpoint_array[0])
        overshoot = self.calculate_overshoot(measured_array, setpoint_array[0])
        steady_state_error = abs(measured_array[-1] - setpoint_array[-1])

        metrics_text = f"Settling Time: {settling_time:.2f}s | Overshoot: {overshoot:.1f}% | SS Error: {steady_state_error:.3f}"
        self.ax.text(
            0.02,
            0.95,
            metrics_text,
            transform=self.ax.transAxes,
            bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.8),
        )

        plt.draw()

    def calculate_settling_time(self, response, setpoint, tolerance=0.02):
        """Calculate settling time"""
        target_band = setpoint * tolerance

        for i in reversed(range(len(response))):
            if abs(response[i] - setpoint) > target_band:
                return (
                    self.time_array[i + 1]
                    if i + 1 < len(self.time_array)
                    else self.time_array[-1]
                )

        return 0.0

    def calculate_overshoot(self, response, setpoint):
        """Calculate percentage overshoot"""
        max_value = np.max(response)
        if max_value > setpoint:
            return ((max_value - setpoint) / setpoint) * 100
        return 0.0


def run_interactive_tuner():
    """Launch the interactive PID tuner"""
    print("Launching Interactive PID Tuner...")
    print("Use the sliders to adjust Kp, Ki, and Kd values.")
    print("Watch how the response changes in real-time!")

    tuner = InteractivePIDTuner()


if __name__ == "__main__":
    run_interactive_tuner()
