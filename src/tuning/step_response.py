from scipy import signal
import numpy as np
import matplotlib.pyplot as plt


def step_response(system, time=None):
    """
    Simulates the step response of a given system.

    Parameters:
    - system: A tuple (numerator, denominator) representing the transfer function of the system.
    - time: Optional array of time points at which to evaluate the response. If None, defaults to 0 to 10 seconds.

    Returns:
    - t: Time points at which the response is evaluated.
    - response: The step response of the system.
    """
    if time is None:
        time = np.linspace(0, 10, 100)  # Default time from 0 to 10 seconds

    # Create a transfer function system
    num, den = system
    sys = signal.TransferFunction(num, den)

    # Compute the step response
    t, response = signal.step(sys, T=time)

    return t, response


def plot_step_response(t, response):
    """
    Plots the step response.

    Parameters:
    - t: Time points.
    - response: Step response values.
    """
    plt.figure()
    plt.plot(t, response)
    plt.title("Step Response")
    plt.xlabel("Time (s)")
    plt.ylabel("Response")
    plt.grid()
    plt.show()


def conduct_step_response_test():
    """Legacy test wrapper: produce step response of a canonical first-order system."""
    t, r = step_response(( [1.0], [1.0, 1.0] ))
    return r
