"""Performance metrics utilities for PID-controlled step responses.

Functions accept time array (sequence of floats), response array, and scalar setpoint.
All return float seconds or percentage where applicable. Assumes time is monotonically increasing.
"""
from __future__ import annotations
from typing import Sequence, Tuple
import math


def _as_lists(time: Sequence[float], response: Sequence[float]) -> Tuple[list, list]:
    t = list(time)
    r = list(response)
    if len(t) != len(r):
        raise ValueError("time and response length mismatch")
    if len(t) < 2:
        raise ValueError("need at least 2 samples")
    return t, r


def settling_time(time: Sequence[float], response: Sequence[float], setpoint: float, tol: float = 0.02) -> float:
    """Return time when response stays within +/- tol*setpoint for remainder of record.
    If never settles returns last time.
    """
    t, r = _as_lists(time, response)
    band = abs(setpoint) * tol
    for i in range(len(r)):
        window = r[i:]
        if all(abs(x - setpoint) <= band for x in window):
            return t[i]
    return t[-1]


def steady_state_error(response: Sequence[float], setpoint: float) -> float:
    """Absolute steady state error using final sample."""
    if len(response) == 0:  # works for list or np array
        raise ValueError("empty response")
    return abs(response[-1] - setpoint)


def overshoot(response: Sequence[float], setpoint: float) -> float:
    """Percent overshoot relative to setpoint (0 if none or setpoint 0)."""
    if setpoint == 0:
        return 0.0
    peak = max(response)
    if peak <= setpoint:
        return 0.0
    return (peak - setpoint) / abs(setpoint) * 100.0


def rise_time(time: Sequence[float], response: Sequence[float], setpoint: float, low: float = 0.1, high: float = 0.9) -> float:
    """Time for response to go from low*setpoint to high*setpoint (first crossings)."""
    t, r = _as_lists(time, response)
    if setpoint == 0:
        return 0.0
    low_val = setpoint * low
    high_val = setpoint * high
    t_low = None
    t_high = None
    for ti, xi in zip(t, r):
        if t_low is None and ((setpoint >= 0 and xi >= low_val) or (setpoint < 0 and xi <= low_val)):
            t_low = ti
        if t_high is None and ((setpoint >= 0 and xi >= high_val) or (setpoint < 0 and xi <= high_val)):
            t_high = ti
        if t_low is not None and t_high is not None:
            break
    if t_low is None or t_high is None:
        return math.nan
    return max(0.0, t_high - t_low)


def all_metrics(time: Sequence[float], response: Sequence[float], setpoint: float) -> dict:
    """Convenience: compute common metrics in one call."""
    return {
        "settling_time": settling_time(time, response, setpoint),
        "rise_time": rise_time(time, response, setpoint),
        "overshoot": overshoot(response, setpoint),
        "steady_state_error": steady_state_error(response, setpoint),
    }
