"""Hardware abstraction layer for sensors and actuators.

Provides platform-specific implementations for IMU, GPS, and motor control
with fallback simulation modes for development and testing.
"""

from __future__ import annotations
import time
import numpy as np
from typing import Tuple, Optional, Dict, Any
from abc import ABC, abstractmethod

# Platform detection
try:
    import RPi.GPIO as GPIO
    import smbus2

    RASPBERRY_PI = True
except ImportError:
    RASPBERRY_PI = False
    # Use fake-rpi for development
    try:
        import fake_rpi.RPi.GPIO as GPIO
        import fake_rpi.smbus as smbus2
    except ImportError:
        GPIO = None
        smbus2 = None


class IMUInterface(ABC):
    """Abstract interface for IMU sensors."""

    @abstractmethod
    def read_gyro(self) -> Tuple[float, float, float]:
        """Read gyroscope data in rad/s (roll, pitch, yaw rates)."""
        pass

    @abstractmethod
    def read_accel(self) -> Tuple[float, float, float]:
        """Read accelerometer data in m/s² (x, y, z)."""
        pass

    @abstractmethod
    def read_mag(self) -> Tuple[float, float, float]:
        """Read magnetometer data in µT (x, y, z)."""
        pass

    @abstractmethod
    def is_healthy(self) -> bool:
        """Check if IMU is responding and healthy."""
        pass


class MPU6050(IMUInterface):
    """MPU6050 IMU implementation for Raspberry Pi."""

    def __init__(self, i2c_address: int = 0x68, i2c_bus: int = 1):
        self.address = i2c_address
        self.bus = None
        self.last_read_time = 0.0
        self.healthy = False

        if RASPBERRY_PI and smbus2:
            try:
                self.bus = smbus2.SMBus(i2c_bus)
                self._initialize_mpu6050()
                self.healthy = True
            except Exception as e:
                print(f"Failed to initialize MPU6050: {e}")
                self.healthy = False
        else:
            print("MPU6050: Running in simulation mode")
            self.healthy = True  # Simulation is always "healthy"

    def _initialize_mpu6050(self) -> None:
        """Initialize MPU6050 registers."""
        if self.bus:
            # Wake up the MPU6050
            self.bus.write_byte_data(self.address, 0x6B, 0)
            # Set gyro range to ±500°/s
            self.bus.write_byte_data(self.address, 0x1B, 0x08)
            # Set accel range to ±4g
            self.bus.write_byte_data(self.address, 0x1C, 0x08)

    def _read_i2c_word(self, register: int) -> int:
        """Read a 16-bit signed value from I2C."""
        if not self.bus:
            return 0

        high = self.bus.read_byte_data(self.address, register)
        low = self.bus.read_byte_data(self.address, register + 1)
        value = (high << 8) + low

        # Convert to signed 16-bit
        if value >= 0x8000:
            value = -((65535 - value) + 1)

        return value

    def read_gyro(self) -> Tuple[float, float, float]:
        """Read gyroscope data."""
        if self.bus:
            try:
                gx = self._read_i2c_word(0x43) / 65.5  # ±500°/s scale
                gy = self._read_i2c_word(0x45) / 65.5
                gz = self._read_i2c_word(0x47) / 65.5

                # Convert to rad/s
                return (np.radians(gx), np.radians(gy), np.radians(gz))
            except:
                self.healthy = False
                return (0.0, 0.0, 0.0)
        else:
            # Simulation mode - add some realistic noise
            t = time.time()
            noise = np.random.normal(0, 0.01, 3)  # Small gyro noise
            return tuple(noise)

    def read_accel(self) -> Tuple[float, float, float]:
        """Read accelerometer data."""
        if self.bus:
            try:
                ax = self._read_i2c_word(0x3B) / 8192.0  # ±4g scale
                ay = self._read_i2c_word(0x3D) / 8192.0
                az = self._read_i2c_word(0x3F) / 8192.0

                # Convert to m/s²
                return (ax * 9.81, ay * 9.81, az * 9.81)
            except:
                self.healthy = False
                return (0.0, 0.0, 9.81)  # Default to 1G down
        else:
            # Simulation mode - 1G with small tilt
            t = time.time()
            tilt = 0.1 * np.sin(t * 0.5)  # Slow oscillation
            return (tilt * 9.81, 0.0, 9.81)

    def read_mag(self) -> Tuple[float, float, float]:
        """Read magnetometer (not available on MPU6050)."""
        # MPU6050 doesn't have magnetometer, return simulated values
        return (25.0, 5.0, -40.0)  # Typical magnetic field values

    def is_healthy(self) -> bool:
        """Check IMU health."""
        return self.healthy


class GPSInterface(ABC):
    """Abstract interface for GPS receivers."""

    @abstractmethod
    def read_position(self) -> Dict[str, Any]:
        """Read GPS position data."""
        pass

    @abstractmethod
    def is_healthy(self) -> bool:
        """Check if GPS has valid fix."""
        pass


class UBLOXSimulated(GPSInterface):
    """Simulated GPS for development and testing."""

    def __init__(self, home_lat: float = 40.7128, home_lon: float = -74.0060):
        self.home_lat = home_lat  # Default: New York
        self.home_lon = home_lon
        self.start_time = time.time()
        self.healthy = True

    def read_position(self) -> Dict[str, Any]:
        """Simulate GPS movement in small area."""
        t = time.time() - self.start_time

        # Simulate small circular movement
        radius_deg = 0.0001  # ~10 meter radius
        lat = self.home_lat + radius_deg * np.sin(t * 0.1)
        lon = self.home_lon + radius_deg * np.cos(t * 0.1)
        alt = 100.0 + 5.0 * np.sin(t * 0.2)  # Varying altitude

        return {
            "latitude": lat,
            "longitude": lon,
            "altitude": alt,
            "fix_quality": 2,  # DGPS fix
            "num_satellites": 8,
            "accuracy": 2.5,
            "velocity_ned": (0.1, 0.1, 0.0),  # Slow movement
        }

    def is_healthy(self) -> bool:
        return self.healthy


class PWMInterface(ABC):
    """Abstract interface for PWM motor control."""

    @abstractmethod
    def set_throttle(self, motor_id: int, throttle: float) -> None:
        """Set motor throttle (0.0 to 1.0)."""
        pass

    @abstractmethod
    def emergency_stop(self) -> None:
        """Immediately stop all motors."""
        pass

    @abstractmethod
    def is_healthy(self) -> bool:
        """Check PWM system health."""
        pass


class PCA9685PWM(PWMInterface):
    """PCA9685 PWM controller for ESCs."""

    def __init__(self, i2c_address: int = 0x40, frequency: int = 50):
        self.address = i2c_address
        self.frequency = frequency
        self.bus = None
        self.healthy = False

        # PWM pulse width settings (microseconds)
        self.pulse_min = 1000  # Minimum pulse (motor off)
        self.pulse_max = 2000  # Maximum pulse (full throttle)

        if RASPBERRY_PI and smbus2:
            try:
                self.bus = smbus2.SMBus(1)
                self._initialize_pca9685()
                self.healthy = True
            except Exception as e:
                print(f"Failed to initialize PCA9685: {e}")
                self.healthy = False
        else:
            print("PCA9685: Running in simulation mode")
            self.healthy = True

    def _initialize_pca9685(self) -> None:
        """Initialize PCA9685 PWM controller."""
        if self.bus:
            # Set PWM frequency
            prescale = int(25000000.0 / (4096 * self.frequency) - 1)
            self.bus.write_byte_data(self.address, 0x00, 0x10)  # Sleep
            self.bus.write_byte_data(self.address, 0xFE, prescale)  # Set prescaler
            self.bus.write_byte_data(self.address, 0x00, 0x80)  # Wake up
            time.sleep(0.005)

    def set_throttle(self, motor_id: int, throttle: float) -> None:
        """Set motor throttle via PWM."""
        throttle = max(0.0, min(1.0, throttle))

        if self.bus:
            try:
                # Convert throttle to pulse width
                pulse_width = self.pulse_min + throttle * (
                    self.pulse_max - self.pulse_min
                )

                # Convert to PCA9685 tick count (0-4095)
                tick_count = int((pulse_width / 1000000.0) * self.frequency * 4096)

                # Set PWM registers for motor channel
                channel = motor_id * 4  # Each channel uses 4 registers
                self.bus.write_byte_data(self.address, 0x06 + channel, 0)  # LED_ON_L
                self.bus.write_byte_data(self.address, 0x07 + channel, 0)  # LED_ON_H
                self.bus.write_byte_data(
                    self.address, 0x08 + channel, tick_count & 0xFF
                )  # LED_OFF_L
                self.bus.write_byte_data(
                    self.address, 0x09 + channel, tick_count >> 8
                )  # LED_OFF_H
            except:
                self.healthy = False
        else:
            # Simulation mode
            print(f"Motor {motor_id}: {throttle:.2f}")

    def emergency_stop(self) -> None:
        """Stop all motors immediately."""
        for motor_id in range(4):  # Support up to 4 motors
            self.set_throttle(motor_id, 0.0)

    def is_healthy(self) -> bool:
        return self.healthy


class HardwareManager:
    """Centralized hardware management with health monitoring."""

    def __init__(self, config: Optional[Dict] = None):
        self.config = config or {}

        # Initialize hardware interfaces
        self.imu = self._create_imu()
        self.gps = self._create_gps()
        self.pwm = self._create_pwm()

        # Health monitoring
        self.last_health_check = time.time()
        self.health_check_interval = 1.0  # Check every second

    def _create_imu(self) -> IMUInterface:
        """Create IMU interface based on configuration."""
        imu_type = self.config.get("imu", {}).get("type", "MPU6050")

        if imu_type == "MPU6050":
            return MPU6050()
        else:
            raise ValueError(f"Unsupported IMU type: {imu_type}")

    def _create_gps(self) -> GPSInterface:
        """Create GPS interface based on configuration."""
        # For now, always use simulated GPS
        # In production, add UBLOX serial interface
        return UBLOXSimulated()

    def _create_pwm(self) -> PWMInterface:
        """Create PWM interface based on configuration."""
        pwm_type = self.config.get("pwm", {}).get("type", "PCA9685")

        if pwm_type == "PCA9685":
            return PCA9685PWM()
        else:
            raise ValueError(f"Unsupported PWM type: {pwm_type}")

    def read_sensors(self) -> Dict[str, Any]:
        """Read all sensor data."""
        sensor_data = {}

        # IMU data
        try:
            sensor_data["gyro"] = self.imu.read_gyro()
            sensor_data["accel"] = self.imu.read_accel()
            sensor_data["mag"] = self.imu.read_mag()
            sensor_data["imu_healthy"] = self.imu.is_healthy()
        except Exception as e:
            print(f"IMU read error: {e}")
            sensor_data["imu_healthy"] = False

        # GPS data
        try:
            gps_data = self.gps.read_position()
            sensor_data.update(gps_data)
            sensor_data["gps_healthy"] = self.gps.is_healthy()
        except Exception as e:
            print(f"GPS read error: {e}")
            sensor_data["gps_healthy"] = False

        sensor_data["timestamp"] = time.time()
        return sensor_data

    def set_motor_outputs(self, motor_commands) -> None:
        """Set motor outputs from mixer commands."""
        try:
            for cmd in motor_commands:
                if cmd.armed:
                    self.pwm.set_throttle(cmd.motor_id, cmd.throttle)
                else:
                    self.pwm.set_throttle(cmd.motor_id, 0.0)
        except Exception as e:
            print(f"Motor output error: {e}")
            self.emergency_stop()

    def emergency_stop(self) -> None:
        """Emergency stop all systems."""
        try:
            self.pwm.emergency_stop()
        except Exception as e:
            print(f"Emergency stop error: {e}")

    def is_healthy(self) -> Dict[str, bool]:
        """Get health status of all hardware."""
        return {
            "imu": self.imu.is_healthy(),
            "gps": self.gps.is_healthy(),
            "pwm": self.pwm.is_healthy(),
        }
