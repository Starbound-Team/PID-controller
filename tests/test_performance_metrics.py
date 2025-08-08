import numpy as np
from src.tuning.performance_metrics import settling_time, overshoot, steady_state_error, rise_time, all_metrics
from src.controllers.pid_controller import PIDController


def generate_step_response():
    pid = PIDController(kp=1.0, ki=0.5, kd=0.05)
    dt = 0.01
    t_end = 8.0
    steps = int(t_end / dt)
    t = np.linspace(0, t_end, steps)
    setpoint = 1.0
    x = 0.0
    resp = []
    for _ in t:
        u = pid.calculate_control(setpoint, x, dt)
        x += (u - x) * dt / 0.4  # first-order plant with tau=0.4
        resp.append(x)
    return t, np.array(resp), setpoint


def test_metrics_reasonable_ranges():
    t, r, sp = generate_step_response()
    st = settling_time(t, r, sp)
    ov = overshoot(r, sp)
    sse = steady_state_error(r, sp)
    rt = rise_time(t, r, sp)
    assert 0.0 <= st <= t[-1]
    assert 0.0 <= ov < 50.0  # overshoot should be modest
    assert sse < 0.07
    assert rt > 0.0


def test_all_metrics_wrapper():
    t, r, sp = generate_step_response()
    m = all_metrics(t, r, sp)
    assert set({"settling_time", "rise_time", "overshoot", "steady_state_error"}) <= set(m.keys())
