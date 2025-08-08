"""Automatic estimation of ultimate gain Ku and oscillation period Pu via Kp sweep.

Uses a simple first-order plant model for demonstration.
Return dict with Ku, Pu, suggested PID (Ziegler-Nichols classic) if oscillation detected.
"""
from __future__ import annotations
from typing import Optional, Dict, List, Tuple
import numpy as np

from controllers.pid_controller import PIDController


def _simulate(pid: PIDController, setpoint: float, t_end: float = 5.0, dt: float = 0.01, tau: float = 0.5):
    steps = int(t_end / dt)
    t = np.linspace(0, t_end, steps)
    x = 0.0
    resp = []
    for _ in t:
        u = pid.calculate_control(setpoint, x, dt)
        # first-order plant dx/dt = (-x + u)/tau
        x += ((-x + u) / tau) * dt
        resp.append(x)
    return t, np.array(resp)


def _detect_sustained_oscillation(signal: np.ndarray, setpoint: float, min_cycles: int = 3) -> Optional[float]:
    # Find zero crossings of error derivative sign changes around setpoint
    err = signal - setpoint
    # Use peaks: sign changes in derivative
    signs = np.sign(err)
    zero_crossings = np.where(np.diff(signs) != 0)[0]
    if len(zero_crossings) < 2 * min_cycles:
        return None
    # Period estimate: average distance between alternating crossings
    intervals = np.diff(zero_crossings)
    if len(intervals) == 0:
        return None
    avg_interval = np.mean(intervals)
    return avg_interval  # in samples


def find_ultimate_gain(
    kp_start: float = 0.1,
    kp_step: float = 0.1,
    kp_max: float = 10.0,
    setpoint: float = 1.0,
    dt: float = 0.01,
    t_end: float = 5.0,
) -> Dict[str, float]:
    ku = None
    pu = None
    for kp in np.arange(kp_start, kp_max + kp_step, kp_step):
        pid = PIDController(kp=kp, ki=0.0, kd=0.0)
        t, resp = _simulate(pid, setpoint, t_end=t_end, dt=dt)
        samples_period = _detect_sustained_oscillation(resp[-int(0.5*len(resp)):], setpoint)
        if samples_period is not None:
            ku = kp
            pu = samples_period * dt
            break
    result = {"Ku": ku if ku is not None else 0.0, "Pu": pu if pu is not None else 0.0}
    if ku and pu:
        # Ziegler–Nichols classic PID
        result.update({
            "Kp_pid": 0.6 * ku,
            "Ki_pid": 1.2 * ku / pu,  # Ki = Kp*2/Tu -> equivalently Kp* (1/(Ti)), Ti=0.5*Pu
            "Kd_pid": 0.075 * ku * pu,  # Kd = Kp*Pu/8
        })
    return result

if __name__ == "__main__":
    res = find_ultimate_gain()
    print(res)
