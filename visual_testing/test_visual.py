#!/usr/bin/env python3
"""
Visual testing script for PID controller performance
"""

import numpy as np
import matplotlib.pyplot as plt
from src.controllers.pid_controller import PIDController
from src.controllers.attitude_controller import AttitudeController
from src.controllers.position_controller import PositionController


def test_step_response():
    """Test PID controller step response"""

    # Create PID controller
    pid = PIDController(kp=1.0, ki=0.1, kd=0.05)

    # Simulation parameters
    dt = 0.01  # 10ms timestep
    time_end = 5.0  # 5 seconds
    time_steps = int(time_end / dt)

    # Arrays to store results
    time_array = np.linspace(0, time_end, time_steps)
    setpoint_array = np.ones(time_steps) * 10.0  # Step input to 10
    measured_array = np.zeros(time_steps)
    control_array = np.zeros(time_steps)

    # Simple plant simulation (first-order system)
    plant_time_constant = 0.5
    measured_value = 0.0

    for i in range(time_steps):
        # Calculate control output
        control_output = pid.calculate_control(setpoint_array[i], measured_value, dt)
        control_array[i] = control_output

        # Simple plant response (first-order lag)
        measured_value += (control_output - measured_value) * dt / plant_time_constant
        measured_array[i] = measured_value

    # Plot results
    plt.figure(figsize=(12, 8))

    plt.subplot(2, 1, 1)
    plt.plot(time_array, setpoint_array, "r--", label="Setpoint", linewidth=2)
    plt.plot(time_array, measured_array, "b-", label="Measured", linewidth=2)
    plt.title("PID Controller Step Response")
    plt.ylabel("Position")
    plt.legend()
    plt.grid(True)

    plt.subplot(2, 1, 2)
    plt.plot(time_array, control_array, "g-", label="Control Output", linewidth=2)
    plt.xlabel("Time (s)")
    plt.ylabel("Control Signal")
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.show()

    # Print performance metrics
    settling_time = calculate_settling_time(
        time_array, measured_array, setpoint_array[0]
    )
    overshoot = calculate_overshoot(measured_array, setpoint_array[0])
    steady_state_error = abs(measured_array[-1] - setpoint_array[-1])

    print(f"\n=== PID Performance Metrics ===")
    print(f"Settling Time: {settling_time:.2f} seconds")
    print(f"Overshoot: {overshoot:.2f}%")
    print(f"Steady State Error: {steady_state_error:.3f}")


def test_attitude_controller_visual():
    """Visual test for attitude controller"""

    # Create attitude controller
    attitude_controller = AttitudeController(
        1.0,
        0.1,
        0.05,  # roll gains
        1.0,
        0.1,
        0.05,  # pitch gains
        1.0,
        0.1,
        0.05,  # yaw gains
    )

    # Simulation parameters
    dt = 0.01
    time_end = 3.0
    time_steps = int(time_end / dt)

    time_array = np.linspace(0, time_end, time_steps)

    # Setpoints (step inputs)
    roll_setpoint = np.ones(time_steps) * 15.0  # 15 degree roll
    pitch_setpoint = np.ones(time_steps) * 10.0  # 10 degree pitch
    yaw_setpoint = np.ones(time_steps) * 20.0  # 20 degree yaw

    # Initialize measured values
    roll_measured = np.zeros(time_steps)
    pitch_measured = np.zeros(time_steps)
    yaw_measured = np.zeros(time_steps)

    # Control outputs
    roll_control = np.zeros(time_steps)
    pitch_control = np.zeros(time_steps)
    yaw_control = np.zeros(time_steps)

    # Simple plant simulation for each axis
    roll_val, pitch_val, yaw_val = 0.0, 0.0, 0.0

    for i in range(time_steps):
        # Calculate control outputs
        controls = attitude_controller.control(
            roll_setpoint[i],
            roll_val,
            pitch_setpoint[i],
            pitch_val,
            yaw_setpoint[i],
            yaw_val,
            dt,
        )

        roll_control[i], pitch_control[i], yaw_control[i] = controls

        # Simple plant response for each axis
        roll_val += (roll_control[i] - roll_val) * dt / 0.3
        pitch_val += (pitch_control[i] - pitch_val) * dt / 0.3
        yaw_val += (yaw_control[i] - yaw_val) * dt / 0.5

        roll_measured[i] = roll_val
        pitch_measured[i] = pitch_val
        yaw_measured[i] = yaw_val

    # Plot results
    plt.figure(figsize=(15, 10))

    # Roll
    plt.subplot(3, 2, 1)
    plt.plot(time_array, roll_setpoint, "r--", label="Setpoint")
    plt.plot(time_array, roll_measured, "b-", label="Measured")
    plt.title("Roll Response")
    plt.ylabel("Angle (deg)")
    plt.legend()
    plt.grid(True)

    plt.subplot(3, 2, 2)
    plt.plot(time_array, roll_control, "g-")
    plt.title("Roll Control Signal")
    plt.ylabel("Control Output")
    plt.grid(True)

    # Pitch
    plt.subplot(3, 2, 3)
    plt.plot(time_array, pitch_setpoint, "r--", label="Setpoint")
    plt.plot(time_array, pitch_measured, "b-", label="Measured")
    plt.title("Pitch Response")
    plt.ylabel("Angle (deg)")
    plt.legend()
    plt.grid(True)

    plt.subplot(3, 2, 4)
    plt.plot(time_array, pitch_control, "g-")
    plt.title("Pitch Control Signal")
    plt.ylabel("Control Output")
    plt.grid(True)

    # Yaw
    plt.subplot(3, 2, 5)
    plt.plot(time_array, yaw_setpoint, "r--", label="Setpoint")
    plt.plot(time_array, yaw_measured, "b-", label="Measured")
    plt.title("Yaw Response")
    plt.xlabel("Time (s)")
    plt.ylabel("Angle (deg)")
    plt.legend()
    plt.grid(True)

    plt.subplot(3, 2, 6)
    plt.plot(time_array, yaw_control, "g-")
    plt.title("Yaw Control Signal")
    plt.xlabel("Time (s)")
    plt.ylabel("Control Output")
    plt.grid(True)

    plt.tight_layout()
    plt.show()


def test_parameter_comparison():
    """Compare different PID parameter sets visually"""

    # Different parameter sets to compare
    pid_configs = [
        {"name": "Conservative", "kp": 0.5, "ki": 0.05, "kd": 0.02},
        {"name": "Aggressive", "kp": 2.0, "ki": 0.3, "kd": 0.1},
        {"name": "Balanced", "kp": 1.0, "ki": 0.1, "kd": 0.05},
    ]

    plt.figure(figsize=(12, 8))

    colors = ["blue", "red", "green"]

    for idx, config in enumerate(pid_configs):
        pid = PIDController(config["kp"], config["ki"], config["kd"])

        # Simulation
        dt = 0.01
        time_end = 4.0
        time_steps = int(time_end / dt)

        time_array = np.linspace(0, time_end, time_steps)
        setpoint = 10.0
        measured_value = 0.0
        measured_array = np.zeros(time_steps)

        for i in range(time_steps):
            control_output = pid.calculate_control(setpoint, measured_value, dt)
            measured_value += (control_output - measured_value) * dt / 0.5
            measured_array[i] = measured_value

        plt.plot(
            time_array,
            measured_array,
            color=colors[idx],
            label=f"{config['name']} (Kp={config['kp']}, Ki={config['ki']}, Kd={config['kd']})",
            linewidth=2,
        )

    plt.axhline(y=10.0, color="black", linestyle="--", label="Setpoint")
    plt.title("PID Parameter Comparison")
    plt.xlabel("Time (s)")
    plt.ylabel("Response")
    plt.legend()
    plt.grid(True)
    plt.show()


def calculate_settling_time(time_array, response, setpoint, tolerance=0.02):
    """Calculate settling time (time to reach within 2% of setpoint)"""
    target_band = setpoint * tolerance

    for i in reversed(range(len(response))):
        if abs(response[i] - setpoint) > target_band:
            return time_array[i + 1] if i + 1 < len(time_array) else time_array[-1]

    return 0.0


def calculate_overshoot(response, setpoint):
    """Calculate percentage overshoot"""
    max_value = np.max(response)
    if max_value > setpoint:
        return ((max_value - setpoint) / setpoint) * 100
    return 0.0


def main():
    """Run all visual tests"""
    print("=== Visual PID Controller Testing ===")
    print("1. Running Step Response Test...")
    test_step_response()

    print("\n2. Running Attitude Controller Test...")
    test_attitude_controller_visual()

    print("\n3. Running Parameter Comparison Test...")
    test_parameter_comparison()

    print("\nAll visual tests completed!")


if __name__ == "__main__":
    main()
