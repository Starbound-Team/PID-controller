"""Tests for flight control systems."""

import unittest
import time
from flight.flight_modes import FlightMode, FlightModeManager
from flight.state_estimation import StateEstimator, VehicleState
from flight.motor_mixer import TriPropMixer
from flight.hardware import HardwareManager


class TestFlightModes(unittest.TestCase):
    """Test flight mode management."""

    def setUp(self):
        self.fm = FlightModeManager()

    def test_initial_state(self):
        """Test initial flight mode state."""
        self.assertEqual(self.fm.current_mode, FlightMode.DISARMED)

    def test_valid_transitions(self):
        """Test valid mode transitions."""
        # DISARMED -> ARMED
        result = self.fm.request_mode_change(
            FlightMode.ARMED, sensors_healthy=True, battery_ok=True
        )
        self.assertTrue(result)
        self.assertEqual(self.fm.current_mode, FlightMode.ARMED)

        # ARMED -> STABILIZE
        result = self.fm.request_mode_change(
            FlightMode.STABILIZE, sensors_healthy=True, battery_ok=True
        )
        self.assertTrue(result)
        self.assertEqual(self.fm.current_mode, FlightMode.STABILIZE)

    def test_invalid_transitions(self):
        """Test invalid mode transitions."""
        # DISARMED -> STABILIZE (skipping ARMED)
        result = self.fm.request_mode_change(
            FlightMode.STABILIZE, sensors_healthy=True, battery_ok=True
        )
        self.assertFalse(result)
        self.assertEqual(self.fm.current_mode, FlightMode.DISARMED)

    def test_emergency_always_allowed(self):
        """Test emergency mode can always be triggered."""
        self.fm.trigger_emergency()
        self.assertEqual(self.fm.current_mode, FlightMode.EMERGENCY)

    def test_safety_requirements(self):
        """Test safety requirements for mode changes."""
        # Should fail with unhealthy sensors
        result = self.fm.request_mode_change(
            FlightMode.ARMED, sensors_healthy=False, battery_ok=True
        )
        self.assertFalse(result)

        # Should fail with low battery
        result = self.fm.request_mode_change(
            FlightMode.ARMED, sensors_healthy=True, battery_ok=False
        )
        self.assertFalse(result)


class TestStateEstimator(unittest.TestCase):
    """Test state estimation and sensor fusion."""

    def setUp(self):
        self.estimator = StateEstimator()

    def test_initial_state(self):
        """Test initial state values."""
        state = self.estimator.get_state()
        self.assertEqual(state.roll, 0.0)
        self.assertEqual(state.pitch, 0.0)
        self.assertEqual(state.yaw, 0.0)

    def test_imu_update(self):
        """Test IMU data integration."""
        # Test that IMU update works without crashing
        timestamp = time.time()
        gyro = (0.1, 0.0, 0.0)  # Roll rate
        accel = (0.0, 0.0, 9.81)  # Level

        self.estimator.update_imu(gyro, accel, timestamp)
        state = self.estimator.get_state()

        # Should update angular rates from gyro
        self.assertEqual(state.roll_rate, 0.1)
        self.assertEqual(state.pitch_rate, 0.0)
        self.assertEqual(state.yaw_rate, 0.0)

    def test_gps_update(self):
        """Test GPS position integration."""
        # Set GPS origin
        self.estimator.update_gps(
            40.7128, -74.0060, 100.0, fix_quality=2, num_satellites=8
        )

        # Move slightly north
        self.estimator.update_gps(
            40.7129, -74.0060, 100.0, fix_quality=2, num_satellites=8
        )
        state = self.estimator.get_state()

        # Should show positive X (north) movement
        self.assertGreater(state.x, 0.0)
        self.assertAlmostEqual(state.y, 0.0, places=1)


class TestMotorMixer(unittest.TestCase):
    """Test motor mixing for tri-prop configuration."""

    def setUp(self):
        self.mixer = TriPropMixer()

    def test_pure_thrust(self):
        """Test pure thrust command."""
        commands = self.mixer.mix_controls(0.5, 0.0, 0.0, 0.0, armed=True)

        # All motors should get equal thrust
        self.assertEqual(len(commands), 3)
        for cmd in commands:
            self.assertAlmostEqual(cmd.throttle, 0.5 / 3, places=2)
            self.assertTrue(cmd.armed)

    def test_roll_moment(self):
        """Test roll moment generation."""
        commands = self.mixer.mix_controls(
            0.3, 0.5, 0.0, 0.0, armed=True
        )  # Add base thrust

        # Should generate different motor outputs for roll
        throttles = [cmd.throttle for cmd in commands]
        # Check that motors have different outputs (some higher, some lower)
        self.assertNotEqual(
            throttles[1], throttles[2]
        )  # Left vs right motors should differ

    def test_disarmed_motors(self):
        """Test disarmed motor behavior."""
        commands = self.mixer.mix_controls(0.5, 0.0, 0.0, 0.0, armed=False)

        # All motors should be off when disarmed
        for cmd in commands:
            self.assertEqual(cmd.throttle, 0.0)
            self.assertFalse(cmd.armed)

    def test_control_authority(self):
        """Test control authority checking."""
        # Test achievable command
        achievable, margin = self.mixer.check_control_authority(0.5, 0.1, 0.1, 0.1)
        self.assertTrue(achievable)
        self.assertGreater(margin, 0.0)

        # Test over-saturated command
        achievable, margin = self.mixer.check_control_authority(1.0, 1.0, 1.0, 1.0)
        self.assertFalse(achievable)


class TestHardwareManager(unittest.TestCase):
    """Test hardware abstraction layer."""

    def setUp(self):
        self.hw = HardwareManager()  # Should work in simulation mode

    def test_sensor_reading(self):
        """Test sensor data reading."""
        data = self.hw.read_sensors()

        # Should contain expected sensor data
        self.assertIn("gyro", data)
        self.assertIn("accel", data)
        self.assertIn("timestamp", data)
        self.assertIn("imu_healthy", data)

        # Gyro and accel should be 3-tuples
        self.assertEqual(len(data["gyro"]), 3)
        self.assertEqual(len(data["accel"]), 3)

    def test_health_monitoring(self):
        """Test hardware health monitoring."""
        health = self.hw.is_healthy()

        # Should report on all subsystems
        self.assertIn("imu", health)
        self.assertIn("gps", health)
        self.assertIn("pwm", health)

        # In simulation mode, should be healthy
        self.assertTrue(health["imu"])
        self.assertTrue(health["gps"])
        self.assertTrue(health["pwm"])

    def test_motor_commands(self):
        """Test motor command execution."""
        from flight.motor_mixer import MotorCommand

        commands = [
            MotorCommand(0, 0.3, True),
            MotorCommand(1, 0.4, True),
            MotorCommand(2, 0.5, True),
        ]

        # Should not raise exception in simulation mode
        try:
            self.hw.set_motor_outputs(commands)
            self.hw.emergency_stop()
        except Exception as e:
            self.fail(f"Motor command failed: {e}")


if __name__ == "__main__":
    unittest.main()
