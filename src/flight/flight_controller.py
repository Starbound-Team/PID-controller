"""Multi-rate flight controller with hardware integration.

Implements a complete flight control system with:
- Multi-rate control loops (attitude/altitude/position/navigation)
- Flight mode management with safety checks
- Hardware abstraction for sensors and actuators
- State estimation and sensor fusion
- Safety systems and emergency handling
"""

from __future__ import annotations
import time
import signal
import sys
import numpy as np
from typing import Optional, Dict, Any
from dataclasses import dataclass

from controllers.attitude_controller import AttitudeController
from controllers.position_controller import PositionController
from controllers.velocity_controller import VelocityController
from flight.flight_modes import FlightMode, FlightModeManager
from flight.state_estimation import StateEstimator, VehicleState
from flight.motor_mixer import TriPropMixer
from flight.hardware import HardwareManager
from utils.config import get_pid_parameters, get_sensor_config
from utils.data_logger import DataLogger


@dataclass
class ControlLoopRates:
    """Control loop execution rates in Hz."""

    attitude: int = 250  # High-rate attitude stabilization
    altitude: int = 50  # Medium-rate altitude control
    position: int = 20  # Lower-rate position control
    navigation: int = 5  # Low-rate navigation and mode logic
    logging: int = 10  # Data logging rate


@dataclass
class SafetyLimits:
    """Safety limits and thresholds."""

    max_tilt_deg: float = 45.0  # Maximum tilt angle
    max_climb_rate: float = 5.0  # Maximum climb rate (m/s)
    max_descent_rate: float = 3.0  # Maximum descent rate (m/s)
    min_battery_voltage: float = 10.8  # Minimum battery voltage
    max_loop_time: float = 0.020  # Maximum loop time before warning (20ms)
    gps_required_accuracy: float = 5.0  # Required GPS accuracy (meters)


class FlightController:
    """Main flight controller with multi-rate control loops."""

    def __init__(
        self,
        rates: Optional[ControlLoopRates] = None,
        safety: Optional[SafetyLimits] = None,
        enable_hardware: bool = True,
    ):
        """Initialize flight controller.

        Args:
            rates: Control loop rates configuration
            safety: Safety limits configuration
            enable_hardware: Enable real hardware (False for simulation)
        """
        self.rates = rates or ControlLoopRates()
        self.safety = safety or SafetyLimits()
        self.enable_hardware = enable_hardware

        # Load configurations
        self.pid_config = get_pid_parameters()["pid_parameters"]
        try:
            self.sensor_config = get_sensor_config()
        except:
            self.sensor_config = {}

        # Initialize subsystems
        self._init_controllers()
        self._init_flight_systems()
        self._init_hardware()

        # Control loop timing
        self._init_timing()

        # Safety and monitoring
        self.emergency_stop_requested = False
        self.loop_overruns = 0
        self.last_heartbeat = time.monotonic()

        # Setup signal handlers for graceful shutdown
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

    def _init_controllers(self) -> None:
        """Initialize PID controllers with configuration."""
        # Attitude controller (roll, pitch, yaw)
        att_cfg = self.pid_config["attitude"]
        self.attitude_controller = AttitudeController(
            att_cfg["roll"], att_cfg["pitch"], att_cfg["yaw"]
        )

        # Position controller (x, y, z)
        pos_cfg = self.pid_config["position"]
        self.position_controller = PositionController(
            pos_cfg["x"], pos_cfg["y"], pos_cfg["z"]
        )

        # Individual velocity controllers for each axis
        vel_cfg = self.pid_config["velocity"]
        self.velocity_x_controller = VelocityController(vel_cfg["vx"])
        self.velocity_y_controller = VelocityController(vel_cfg["vy"])
        self.velocity_z_controller = VelocityController(vel_cfg["vz"])

    def _init_flight_systems(self) -> None:
        """Initialize flight management systems."""
        self.flight_mode_manager = FlightModeManager()
        self.state_estimator = StateEstimator()
        self.motor_mixer = TriPropMixer()
        # Initialize data logger with timestamped filename
        import datetime

        log_filename = (
            f"flight_log_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        )
        self.data_logger = DataLogger(log_filename)

    def _init_hardware(self) -> None:
        """Initialize hardware interfaces."""
        if self.enable_hardware:
            self.hardware = HardwareManager(self.sensor_config)
        else:
            self.hardware = None
            print("Flight Controller: Running in simulation mode")

    def _init_timing(self) -> None:
        """Initialize control loop timing."""
        now = time.monotonic()

        # Calculate loop periods
        self.attitude_period = 1.0 / self.rates.attitude
        self.altitude_period = 1.0 / self.rates.altitude
        self.position_period = 1.0 / self.rates.position
        self.navigation_period = 1.0 / self.rates.navigation
        self.logging_period = 1.0 / self.rates.logging

        # Initialize next execution times
        self.next_attitude = now
        self.next_altitude = now
        self.next_position = now
        self.next_navigation = now
        self.next_logging = now

    def _signal_handler(self, signum, frame) -> None:
        """Handle shutdown signals gracefully."""
        print(f"\nReceived signal {signum}, initiating emergency shutdown...")
        self.emergency_stop_requested = True

    def _check_safety_limits(self, state: VehicleState) -> bool:
        """Check if current state violates safety limits."""
        attitude_deg = state.to_degrees()

        # Check attitude limits
        if (
            abs(attitude_deg["roll"]) > self.safety.max_tilt_deg
            or abs(attitude_deg["pitch"]) > self.safety.max_tilt_deg
        ):
            print(
                f"SAFETY: Excessive tilt detected: roll={attitude_deg['roll']:.1f}°, pitch={attitude_deg['pitch']:.1f}°"
            )
            return False

        # Check climb/descent rates
        if state.vz > self.safety.max_descent_rate:  # NED: positive Z is down
            print(f"SAFETY: Excessive descent rate: {state.vz:.1f} m/s")
            return False
        elif state.vz < -self.safety.max_climb_rate:
            print(f"SAFETY: Excessive climb rate: {-state.vz:.1f} m/s")
            return False

        return True

    def _attitude_loop(self, state: VehicleState, dt: float) -> tuple:
        """High-rate attitude control loop."""
        if not self.flight_mode_manager.requires_attitude_control():
            return (0.0, 0.0, 0.0)  # No attitude control needed

        # For now, use simple stabilization setpoints
        # In full implementation, these come from outer loops or pilot input
        roll_setpoint = 0.0  # Level flight
        pitch_setpoint = 0.0  # Level flight
        yaw_setpoint = state.yaw  # Hold current yaw

        # Convert state to degrees for controller
        attitude_deg = state.to_degrees()

        return self.attitude_controller.control(
            roll_setpoint,
            attitude_deg["roll"],
            pitch_setpoint,
            attitude_deg["pitch"],
            yaw_setpoint,
            attitude_deg["yaw"],
            dt,
        )

    def _altitude_loop(self, state: VehicleState, dt: float) -> float:
        """Medium-rate altitude control loop."""
        if not self.flight_mode_manager.requires_altitude_control():
            return 0.5  # Default hover throttle

        # Simple altitude hold at current altitude
        altitude_setpoint = state.altitude_agl
        if altitude_setpoint < 1.0:  # Minimum altitude
            altitude_setpoint = 1.0

        # This would normally use a dedicated altitude PID controller
        # For now, use a simple proportional control
        altitude_error = altitude_setpoint - state.altitude_agl
        altitude_output = 0.5 + 0.1 * altitude_error  # Basic P controller

        return max(0.1, min(0.9, altitude_output))

    def _position_loop(self, state: VehicleState, dt: float) -> tuple:
        """Lower-rate position control loop."""
        if not self.flight_mode_manager.requires_position_control():
            return (0.0, 0.0)  # No position control

        # Simple position hold at current location
        x_setpoint = state.x
        y_setpoint = state.y

        # This would use the position controller
        # For now, simplified implementation
        return (0.0, 0.0)  # Hold position

    def _navigation_loop(self, state: VehicleState, dt: float) -> None:
        """Low-rate navigation and mode management."""
        # Health checks
        hardware_health = (
            self.hardware.is_healthy()
            if self.hardware
            else {"imu": True, "gps": True, "pwm": True}
        )
        sensors_healthy = hardware_health.get("imu", False)
        gps_healthy = self.state_estimator.is_gps_healthy()

        # Safety limit checks
        if not self._check_safety_limits(state):
            print("SAFETY VIOLATION: Triggering emergency mode")
            self.flight_mode_manager.trigger_emergency()

        # Battery monitoring would go here
        # For now, assume battery is OK
        battery_ok = True

        # Auto-disarm if in emergency mode too long
        if (
            self.flight_mode_manager.current_mode == FlightMode.EMERGENCY
            and self.flight_mode_manager.get_mode_duration() > 5.0
        ):
            self.flight_mode_manager.request_mode_change(
                FlightMode.DISARMED,
                sensors_healthy=sensors_healthy,
                battery_ok=battery_ok,
                gps_lock=gps_healthy,
            )

    def run_control_loop(self, duration: float = 10.0) -> None:
        """Main control loop with multi-rate scheduling."""
        print(f"Starting flight control loop for {duration:.1f}s")
        print(
            f"Rates: Attitude={self.rates.attitude}Hz, Position={self.rates.position}Hz, Nav={self.rates.navigation}Hz"
        )

        start_time = time.monotonic()
        end_time = start_time + duration

        # Performance monitoring
        loop_count = 0
        last_stats_time = start_time

        try:
            while time.monotonic() < end_time and not self.emergency_stop_requested:
                loop_start = time.monotonic()

                # Read sensors
                if self.hardware:
                    sensor_data = self.hardware.read_sensors()

                    # Update state estimation
                    if sensor_data.get("imu_healthy", False):
                        self.state_estimator.update_imu(
                            sensor_data["gyro"],
                            sensor_data["accel"],
                            sensor_data["timestamp"],
                        )

                    if sensor_data.get("gps_healthy", False):
                        self.state_estimator.update_gps(
                            sensor_data["latitude"],
                            sensor_data["longitude"],
                            sensor_data["altitude"],
                            sensor_data.get("velocity_ned"),
                            sensor_data.get("fix_quality", 0),
                            sensor_data.get("num_satellites", 0),
                            sensor_data.get("accuracy", 999.0),
                            sensor_data["timestamp"],
                        )
                else:
                    # Simulation mode - create synthetic sensor data
                    t = time.monotonic() - start_time
                    self.state_estimator.update_imu(
                        (0.01 * np.sin(t), 0.01 * np.cos(t), 0.001),  # Gyro
                        (0.1 * np.sin(t * 0.5), 0.0, 9.81),  # Accel
                        loop_start,
                    )

                # Get current state
                state = self.state_estimator.get_state()

                # Multi-rate control scheduling
                now = time.monotonic()

                # Attitude loop (highest rate)
                if now >= self.next_attitude:
                    dt = now - (self.next_attitude - self.attitude_period)
                    roll_cmd, pitch_cmd, yaw_cmd = self._attitude_loop(state, dt)
                    self.next_attitude += self.attitude_period

                    # Get thrust from altitude loop (if due)
                    thrust_cmd = 0.5  # Default
                    if now >= self.next_altitude:
                        alt_dt = now - (self.next_altitude - self.altitude_period)
                        thrust_cmd = self._altitude_loop(state, alt_dt)
                        self.next_altitude += self.altitude_period

                    # Mix controls and send to motors
                    motor_commands = self.motor_mixer.mix_controls(
                        thrust_cmd,
                        roll_cmd,
                        pitch_cmd,
                        yaw_cmd,
                        armed=(
                            self.flight_mode_manager.current_mode != FlightMode.DISARMED
                        ),
                    )

                    if self.hardware:
                        self.hardware.set_motor_outputs(motor_commands)

                # Position loop (medium rate)
                if now >= self.next_position:
                    pos_dt = now - (self.next_position - self.position_period)
                    x_cmd, y_cmd = self._position_loop(state, pos_dt)
                    self.next_position += self.position_period

                # Navigation loop (lowest rate)
                if now >= self.next_navigation:
                    nav_dt = now - (self.next_navigation - self.navigation_period)
                    self._navigation_loop(state, nav_dt)
                    self.next_navigation += self.navigation_period

                # Data logging
                if now >= self.next_logging:
                    log_data = {
                        "timestamp": now,
                        "mode": self.flight_mode_manager.current_mode.name,
                        "state": state,
                        "loop_count": loop_count,
                    }
                    # self.data_logger.log(log_data)  # Uncomment when logger is ready
                    self.next_logging += self.logging_period

                # Performance monitoring
                loop_time = time.monotonic() - loop_start
                if loop_time > self.safety.max_loop_time:
                    self.loop_overruns += 1
                    if self.loop_overruns % 10 == 0:  # Warn every 10th overrun
                        print(
                            f"WARNING: Loop overrun #{self.loop_overruns}: {loop_time*1000:.1f}ms"
                        )

                # Print periodic status
                loop_count += 1
                if now - last_stats_time >= 2.0:  # Every 2 seconds
                    attitude_deg = state.to_degrees()
                    print(
                        f"Status: Mode={self.flight_mode_manager.current_mode.name} "
                        f"Attitude=({attitude_deg['roll']:.1f}°,{attitude_deg['pitch']:.1f}°,{attitude_deg['yaw']:.1f}°) "
                        f"Loops={loop_count}"
                    )
                    last_stats_time = now

                # Small sleep to prevent CPU spinning
                time.sleep(0.001)

        except KeyboardInterrupt:
            print("\nKeyboard interrupt received")
        except Exception as e:
            print(f"CRITICAL ERROR in control loop: {e}")
            self.emergency_stop_requested = True
        finally:
            self._shutdown()

    def _shutdown(self) -> None:
        """Graceful shutdown with motor safety."""
        print("Shutting down flight controller...")

        # Emergency stop motors
        if self.hardware:
            try:
                self.hardware.emergency_stop()
                print("Motors stopped")
            except Exception as e:
                print(f"Error stopping motors: {e}")

        # Force disarmed mode
        self.flight_mode_manager.request_mode_change(FlightMode.DISARMED)
        print("Flight mode: DISARMED")

        print("Flight controller shutdown complete")


def main():
    """Main entry point for hardware-integrated flight controller."""

    # Configuration
    rates = ControlLoopRates(
        attitude=250,  # 250 Hz attitude control
        altitude=50,  # 50 Hz altitude control
        position=20,  # 20 Hz position control
        navigation=5,  # 5 Hz navigation
    )

    safety = SafetyLimits(
        max_tilt_deg=30.0,  # Conservative tilt limit
        max_climb_rate=3.0,  # Moderate climb rate
        max_descent_rate=2.0,  # Moderate descent rate
    )

    # Create flight controller
    # Set enable_hardware=False for simulation, True for real hardware
    fc = FlightController(
        rates=rates,
        safety=safety,
        enable_hardware=False,  # Change to True for real drone
    )

    # Run for 30 seconds
    fc.run_control_loop(duration=30.0)


if __name__ == "__main__":
    main()
