"""Hardware abstraction layer for sensors and actuators.

Provides platform-specific implementations for IMU, GPS, and motor control
with fallback simulation modes for development and testing.
"""

from __future__ import annotations
import time
import numpy as np
from typing import Tuple, Optional, Dict, Any
from abc import ABC, abstractmethod
from enum import Enum
from dataclasses import dataclass

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


class HealthStatus(Enum):
    """Hardware health status levels."""

    HEALTHY = "healthy"  # Fully operational
    DEGRADED = "degraded"  # Reduced performance but functional
    MARGINAL = "marginal"  # Barely functional, unreliable
    FAILED = "failed"  # Non-functional


@dataclass
class HealthMetrics:
    """Health metrics for hardware components."""

    status: HealthStatus
    error_rate: float = 0.0  # Error rate (0.0 to 1.0)
    response_time: float = 0.0  # Average response time in seconds
    last_error: Optional[str] = None
    consecutive_errors: int = 0
    last_success_time: float = 0.0


class IMUInterface(ABC):
    """Abstract interface for IMU sensors with health monitoring."""

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
    def get_health_metrics(self) -> HealthMetrics:
        """Get detailed health metrics."""
        pass

    @abstractmethod
    def is_healthy(self) -> bool:
        """Check if IMU is responding and healthy."""
        pass


class MPU6050(IMUInterface):
    """MPU6050 IMU implementation for Raspberry Pi with health monitoring."""

    def __init__(self, i2c_address: int = 0x68, i2c_bus: int = 1):
        self.address = i2c_address
        self.bus = None
        self.last_read_time = 0.0

        # Health monitoring
        self.health_metrics = HealthMetrics(HealthStatus.FAILED)
        self.error_count = 0
        self.success_count = 0
        self.last_error_time = 0.0

        if RASPBERRY_PI and smbus2:
            try:
                self.bus = smbus2.SMBus(i2c_bus)
                self._initialize_mpu6050()
                self.health_metrics.status = HealthStatus.HEALTHY
                self.health_metrics.last_success_time = time.time()
            except Exception as e:
                print(f"Failed to initialize MPU6050: {e}")
                self.health_metrics.status = HealthStatus.FAILED
                self.health_metrics.last_error = str(e)
        else:
            print("MPU6050: Running in simulation mode")
            self.health_metrics.status = (
                HealthStatus.HEALTHY
            )  # Simulation is always "healthy"

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
        start_time = time.time()

        if self.bus:
            try:
                gx = self._read_i2c_word(0x43) / 65.5  # ±500°/s scale
                gy = self._read_i2c_word(0x45) / 65.5
                gz = self._read_i2c_word(0x47) / 65.5

                # Convert to rad/s
                result = (np.radians(gx), np.radians(gy), np.radians(gz))

                # Update health metrics on success
                self._record_success(start_time)
                return result

            except (OSError, IOError, ValueError) as e:
                print(f"CRITICAL: IMU gyro read error: {e}")
                self._record_error(str(e), start_time)
                return (0.0, 0.0, 0.0)
        else:
            # Simulation mode - add some realistic noise
            t = time.time()
            noise = np.random.normal(0, 0.01, 3)  # Small gyro noise
            self._record_success(start_time)
            return tuple(noise)

    def read_accel(self) -> Tuple[float, float, float]:
        """Read accelerometer data."""
        start_time = time.time()

        if self.bus:
            try:
                ax = self._read_i2c_word(0x3B) / 8192.0  # ±4g scale
                ay = self._read_i2c_word(0x3D) / 8192.0
                az = self._read_i2c_word(0x3F) / 8192.0

                # Convert to m/s²
                result = (ax * 9.81, ay * 9.81, az * 9.81)
                self._record_success(start_time)
                return result

            except (OSError, IOError, ValueError) as e:
                print(f"CRITICAL: IMU accel read error: {e}")
                self._record_error(str(e), start_time)
                return (0.0, 0.0, 9.81)  # Default to 1G down
        else:
            # Simulation mode - 1G with small tilt
            t = time.time()
            tilt = 0.1 * np.sin(t * 0.5)  # Slow oscillation
            self._record_success(start_time)
            return (tilt * 9.81, 0.0, 9.81)

    def _record_success(self, start_time: float) -> None:
        """Record successful operation for health monitoring."""
        self.success_count += 1
        response_time = time.time() - start_time
        self.health_metrics.last_success_time = time.time()
        self.health_metrics.response_time = response_time

        # Update status based on performance
        if response_time > 0.01:  # 10ms threshold
            if self.health_metrics.status == HealthStatus.HEALTHY:
                self.health_metrics.status = HealthStatus.DEGRADED
        else:
            if self.error_count == 0:
                self.health_metrics.status = HealthStatus.HEALTHY

    def _record_error(self, error_msg: str, start_time: float) -> None:
        """Record error for health monitoring."""
        self.error_count += 1
        self.last_error_time = time.time()
        self.health_metrics.last_error = error_msg
        self.health_metrics.last_error_time = self.last_error_time

        # Determine status based on error frequency
        if self.error_count > 10:
            self.health_metrics.status = HealthStatus.FAILED
        else:
            self.health_metrics.status = HealthStatus.DEGRADED

    def get_health_metrics(self) -> HealthMetrics:
        """Get current health metrics."""
        # Update error rate
        total_ops = self.success_count + self.error_count
        if total_ops > 0:
            self.health_metrics.error_rate = self.error_count / total_ops
        else:
            self.health_metrics.error_rate = 0.0

        return self.health_metrics

    def read_mag(self) -> Tuple[float, float, float]:
        """Read magnetometer (not available on MPU6050)."""
        # MPU6050 doesn't have magnetometer, return simulated values
        return (25.0, 5.0, -40.0)  # Typical magnetic field values

    def is_healthy(self) -> bool:
        """Check IMU health."""
        return self.health_metrics.status != HealthStatus.FAILED


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

    @abstractmethod
    def get_health_metrics(self) -> HealthMetrics:
        """Get GPS health metrics."""
        pass


class UBLOXSimulated(GPSInterface):
    """Simulated GPS for development and testing."""

    def __init__(self, home_lat: float = 40.7128, home_lon: float = -74.0060):
        self.home_lat = home_lat  # Default: New York
        self.home_lon = home_lon
        self.start_time = time.time()

        # Health monitoring
        self.health_metrics = HealthMetrics(HealthStatus.HEALTHY)
        self.fix_count = 0
        self.total_reads = 0
        self.health_metrics.last_success_time = time.time()

    def read_position(self) -> Dict[str, Any]:
        """Simulate GPS movement in small area."""
        start_time = time.time()
        self.total_reads += 1

        try:
            t = time.time() - self.start_time

            # Simulate small circular movement
            radius_deg = 0.0001  # ~10 meter radius
            lat = self.home_lat + radius_deg * np.sin(t * 0.1)
            lon = self.home_lon + radius_deg * np.cos(t * 0.1)
            alt = 100.0 + 5.0 * np.sin(t * 0.2)  # Varying altitude

            # Simulate occasional fix loss
            fix_quality = 2 if (int(t) % 30) < 25 else 0  # Lose fix occasionally

            if fix_quality > 0:
                self.fix_count += 1
                self._record_success(start_time)
            else:
                self._record_degraded("GPS fix lost", start_time)

            return {
                "latitude": lat,
                "longitude": lon,
                "altitude": alt,
                "fix_quality": fix_quality,
                "num_satellites": 8 if fix_quality > 0 else 0,
                "accuracy": 2.5 if fix_quality > 0 else 99.9,
                "velocity_ned": (0.1, 0.1, 0.0),  # Slow movement
            }
        except Exception as e:
            self._record_error(str(e), start_time)
            return {
                "latitude": 0.0,
                "longitude": 0.0,
                "altitude": 0.0,
                "fix_quality": 0,
                "num_satellites": 0,
                "accuracy": 99.9,
                "velocity_ned": (0.0, 0.0, 0.0),
            }

    def is_healthy(self) -> bool:
        """Check GPS health based on fix quality."""
        return self.health_metrics.status != HealthStatus.FAILED

    def _record_success(self, start_time: float) -> None:
        """Record successful GPS read."""
        response_time = time.time() - start_time
        self.health_metrics.last_success_time = time.time()
        self.health_metrics.response_time = response_time

        # GPS is healthy if we have good fix rate
        if self.fix_count > 0 and self.total_reads > 0:
            fix_rate = self.fix_count / self.total_reads
            if fix_rate > 0.8:
                self.health_metrics.status = HealthStatus.HEALTHY
            elif fix_rate > 0.5:
                self.health_metrics.status = HealthStatus.DEGRADED

    def _record_degraded(self, reason: str, start_time: float) -> None:
        """Record degraded GPS performance."""
        self.health_metrics.last_error = reason
        self.health_metrics.last_error_time = time.time()
        self.health_metrics.status = HealthStatus.DEGRADED

    def _record_error(self, error_msg: str, start_time: float) -> None:
        """Record GPS error."""
        self.health_metrics.last_error = error_msg
        self.health_metrics.last_error_time = time.time()
        self.health_metrics.status = HealthStatus.FAILED

    def get_health_metrics(self) -> HealthMetrics:
        """Get GPS health metrics."""
        # Update fix rate
        if self.total_reads > 0:
            self.health_metrics.error_rate = 1.0 - (self.fix_count / self.total_reads)
        else:
            self.health_metrics.error_rate = 0.0

        return self.health_metrics


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

    @abstractmethod
    def get_health_metrics(self) -> HealthMetrics:
        """Get PWM health metrics."""
        pass


class PCA9685PWM(PWMInterface):
    """PCA9685 PWM controller for ESCs."""

    def __init__(self, i2c_address: int = 0x40, frequency: int = 50):
        self.address = i2c_address
        self.frequency = frequency
        self.bus = None

        # Health monitoring
        self.health_metrics = HealthMetrics(HealthStatus.FAILED)
        self.command_count = 0
        self.error_count = 0
        self.last_command_time = 0.0

        # PWM pulse width settings (microseconds)
        self.pulse_min = 1000  # Minimum pulse (motor off)
        self.pulse_max = 2000  # Maximum pulse (full throttle)

        if RASPBERRY_PI and smbus2:
            try:
                self.bus = smbus2.SMBus(1)
                self._initialize_pca9685()
                self.health_metrics.status = HealthStatus.HEALTHY
                self.health_metrics.last_success_time = time.time()
            except Exception as e:
                print(f"Failed to initialize PCA9685: {e}")
                self.health_metrics.status = HealthStatus.FAILED
                self.health_metrics.last_error = str(e)
        else:
            print("PCA9685: Running in simulation mode")
            self.health_metrics.status = HealthStatus.HEALTHY  # Simulation mode

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
        start_time = time.time()
        self.command_count += 1

        try:
            throttle = max(0.0, min(1.0, throttle))

            if self.bus:
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

                self.last_command_time = time.time()
                self._record_success(start_time)
            else:
                # Simulation mode
                print(f"Motor {motor_id}: {throttle:.2f}")
                self._record_success(start_time)

        except (OSError, IOError, ValueError) as e:
            print(f"CRITICAL: PWM write error on motor {motor_id}: {e}")
            self._record_error(str(e), start_time)

    def emergency_stop(self) -> None:
        """Stop all motors immediately with timeout protection."""
        import threading

        def _emergency_stop_with_timeout():
            """Internal emergency stop with timeout protection."""
            try:
                for motor_id in range(4):  # Support up to 4 motors
                    self.set_throttle(motor_id, 0.0)
                print("Emergency stop: All motors stopped via PWM")
            except Exception as e:
                print(f"CRITICAL: Emergency stop PWM failure: {e}")
                # In a real system, this would trigger hardware watchdog

        # Execute emergency stop with timeout
        stop_thread = threading.Thread(target=_emergency_stop_with_timeout)
        stop_thread.daemon = True
        stop_thread.start()
        stop_thread.join(timeout=0.1)  # 100ms timeout

        if stop_thread.is_alive():
            print("CRITICAL: Emergency stop timeout - PWM system unresponsive!")
            # In production: trigger hardware watchdog or GPIO emergency stop

    def is_healthy(self) -> bool:
        """Check PWM system health."""
        return self.health_metrics.status != HealthStatus.FAILED

    def _record_success(self, start_time: float) -> None:
        """Record successful PWM command."""
        response_time = time.time() - start_time
        self.health_metrics.last_success_time = time.time()
        self.health_metrics.response_time = response_time

        # Check if response time is acceptable
        if response_time > 0.005:  # 5ms threshold for PWM commands
            if self.health_metrics.status == HealthStatus.HEALTHY:
                self.health_metrics.status = HealthStatus.DEGRADED
        else:
            if self.error_count == 0:
                self.health_metrics.status = HealthStatus.HEALTHY

    def _record_error(self, error_msg: str, start_time: float) -> None:
        """Record PWM error."""
        self.error_count += 1
        self.health_metrics.last_error = error_msg
        self.health_metrics.last_error_time = time.time()

        # PWM errors are critical for safety
        if self.error_count > 3:
            self.health_metrics.status = HealthStatus.FAILED
        else:
            self.health_metrics.status = HealthStatus.DEGRADED

    def get_health_metrics(self) -> HealthMetrics:
        """Get PWM health metrics."""
        # Update error rate
        if self.command_count > 0:
            self.health_metrics.error_rate = self.error_count / self.command_count
        else:
            self.health_metrics.error_rate = 0.0

        return self.health_metrics


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
