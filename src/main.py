"""Real-time style control loop (sensor-free demonstration).

This module simulates a deterministic control loop at a target frequency without
hardware sensors. It exercises the enhanced PID controllers using synthetic
setpoints and a simple first-order plant per axis. Intended as a template for
later integration with real sensor acquisition & actuator outputs.
"""

from __future__ import annotations
import time
from controllers.attitude_controller import AttitudeController
from utils.config import get_pid_parameters


def _now() -> float:
    return time.perf_counter()


def main(loop_hz: int = 250, duration_s: float = 5.0) -> None:
    cfg = get_pid_parameters()["pid_parameters"]["attitude"]
    # Use same params for all axes (or customize per-axis)
    roll_params = cfg["roll"]
    pitch_params = cfg["pitch"]
    yaw_params = cfg["yaw"]
    att = AttitudeController(roll_params, pitch_params, yaw_params)

    target_dt = 1.0 / loop_hz
    end_time = _now() + duration_s

    # Synthetic plant states (roll/pitch/yaw angles)
    roll = pitch = yaw = 0.0
    tau_roll = 0.25  # time constant (s) for first-order lag simulation
    tau_pitch = 0.30
    tau_yaw = 0.35

    # Setpoints (could be dynamic / scripted)
    roll_sp = 10.0  # degrees
    pitch_sp = 5.0
    yaw_sp = 15.0

    last = _now()
    iterations = 0
    while _now() < end_time:
        now = _now()
        dt = now - last
        if dt <= 0:
            continue
        last = now

        # Controller outputs
        roll_out, pitch_out, yaw_out = att.control(
            roll_sp, roll, pitch_sp, pitch, yaw_sp, yaw, dt
        )

        # Simple first-order response for each axis
        roll += (roll_out - roll) * dt / tau_roll
        pitch += (pitch_out - pitch) * dt / tau_pitch
        yaw += (yaw_out - yaw) * dt / tau_yaw

        iterations += 1

        # Timing control (coarse): sleep remaining budget
        loop_elapsed = _now() - now
        remaining = target_dt - loop_elapsed
        if remaining > 0:
            # Use sleep to relinquish CPU; in a tighter loop use busy-wait or OS scheduler tuning
            time.sleep(remaining)

    total_time = duration_s
    achieved_hz = iterations / total_time
    print(
        f"Loop finished: iterations={iterations} target_hz={loop_hz} achieved_hz={achieved_hz:.1f} final_angles=({roll:.2f},{pitch:.2f},{yaw:.2f})"
    )


if __name__ == "__main__":  # pragma: no cover
    main()
