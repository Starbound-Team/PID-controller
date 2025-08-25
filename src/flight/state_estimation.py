"""State estimation module for sensor fusion and vehicle state.

Combines IMU, GPS, and other sensor data into a unified vehicle state
estimate using complementary filtering and sensor fusion techniques.
"""

from __future__ import annotations
import time
import numpy as np
from typing import Optional, Tuple, Dict, Any
from dataclasses import dataclass


@dataclass
class VehicleState:
    """Complete vehicle state estimate."""

    timestamp: float

    # Attitude (radians)
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0

    # Angular rates (rad/s)
    roll_rate: float = 0.0
    pitch_rate: float = 0.0
    yaw_rate: float = 0.0

    # Position (meters, NED frame)
    x: float = 0.0  # North
    y: float = 0.0  # East
    z: float = 0.0  # Down (negative = altitude)

    # Velocity (m/s, NED frame)
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0

    # Altitude above ground (meters)
    altitude_agl: float = 0.0

    # Quality indicators
    gps_fix: bool = False
    num_satellites: int = 0
    gps_accuracy: float = 999.0  # meters

    def to_degrees(self) -> Dict[str, float]:
        """Convert attitude to degrees for display."""
        return {
            "roll": np.degrees(self.roll),
            "pitch": np.degrees(self.pitch),
            "yaw": np.degrees(self.yaw),
        }


class StateEstimator:
    """Sensor fusion and state estimation for vehicle navigation."""

    def __init__(
        self,
        attitude_alpha: float = 0.98,  # Complementary filter weight
        gps_timeout: float = 2.0,
    ):  # GPS timeout (seconds)
        # Complementary filter parameters
        self.attitude_alpha = attitude_alpha  # Trust gyro over accel

        # State
        self.state = VehicleState(timestamp=time.monotonic())

        # GPS processing
        self.gps_timeout = gps_timeout
        self.last_gps_time = 0.0
        self.gps_origin: Optional[Tuple[float, float, float]] = None  # lat, lon, alt

        # Calibration data
        self.gyro_bias = np.array([0.0, 0.0, 0.0])
        self.accel_bias = np.array([0.0, 0.0, 0.0])
        self.magnetometer_bias = np.array([0.0, 0.0, 0.0])

        # Previous state for differentiation
        self.prev_timestamp = time.monotonic()

    def calibrate_gyro(self, gyro_samples: np.ndarray) -> None:
        """Calibrate gyroscope bias from stationary samples."""
        if len(gyro_samples.shape) == 2 and gyro_samples.shape[1] == 3:
            self.gyro_bias = np.mean(gyro_samples, axis=0)

    def update_imu(
        self,
        gyro: Tuple[float, float, float],  # rad/s (roll, pitch, yaw rates)
        accel: Tuple[float, float, float],  # m/s² (x, y, z)
        timestamp: Optional[float] = None,
    ) -> None:
        """Update state with IMU data using complementary filter."""
        if timestamp is None:
            timestamp = time.monotonic()

        dt = timestamp - self.prev_timestamp
        if dt <= 0:
            return

        # Apply gyro bias correction
        gyro_corrected = np.array(gyro) - self.gyro_bias

        # Update angular rates
        self.state.roll_rate = gyro_corrected[0]
        self.state.pitch_rate = gyro_corrected[1]
        self.state.yaw_rate = gyro_corrected[2]

        # Integrate gyro for attitude (high-frequency, drift prone)
        gyro_roll = self.state.roll + gyro_corrected[0] * dt
        gyro_pitch = self.state.pitch + gyro_corrected[1] * dt
        gyro_yaw = self.state.yaw + gyro_corrected[2] * dt

        # Calculate attitude from accelerometer (low-frequency, noisy)
        accel_array = np.array(accel)
        accel_norm = np.linalg.norm(accel_array)

        if accel_norm > 0.1:  # Avoid division by zero
            # Tilt angles from accelerometer (assumes 1G environment)
            accel_roll = np.arctan2(accel_array[1], accel_array[2])
            accel_pitch = np.arctan2(
                -accel_array[0], np.sqrt(accel_array[1] ** 2 + accel_array[2] ** 2)
            )

            # Complementary filter: trust gyro for short term, accel for long term
            self.state.roll = (
                self.attitude_alpha * gyro_roll + (1 - self.attitude_alpha) * accel_roll
            )
            self.state.pitch = (
                self.attitude_alpha * gyro_pitch
                + (1 - self.attitude_alpha) * accel_pitch
            )
            self.state.yaw = gyro_yaw  # No accel reference for yaw
        else:
            # Pure gyro integration if accel is unreliable
            self.state.roll = gyro_roll
            self.state.pitch = gyro_pitch
            self.state.yaw = gyro_yaw

        self.state.timestamp = timestamp
        self.prev_timestamp = timestamp

    def update_gps(
        self,
        latitude: float,  # degrees
        longitude: float,  # degrees
        altitude: float,  # meters MSL
        velocity_ned: Optional[Tuple[float, float, float]] = None,
        fix_quality: int = 0,  # 0=invalid, 1=GPS, 2=DGPS
        num_satellites: int = 0,
        accuracy: float = 999.0,  # meters
        timestamp: Optional[float] = None,
    ) -> None:
        """Update state with GPS data."""
        if timestamp is None:
            timestamp = time.monotonic()

        # Set GPS origin on first good fix
        if self.gps_origin is None and fix_quality > 0:
            self.gps_origin = (latitude, longitude, altitude)

        # Convert GPS to local NED coordinates
        if self.gps_origin is not None and fix_quality > 0:
            lat_origin, lon_origin, alt_origin = self.gps_origin

            # Simple flat-earth approximation for local positioning
            # For production, use proper geodetic transforms
            R_earth = 6371000.0  # Earth radius in meters

            # Convert lat/lon differences to meters
            delta_lat = np.radians(latitude - lat_origin)
            delta_lon = np.radians(longitude - lon_origin)

            self.state.x = delta_lat * R_earth  # North
            self.state.y = delta_lon * R_earth * np.cos(np.radians(lat_origin))  # East
            self.state.z = -(altitude - alt_origin)  # Down (NED convention)
            self.state.altitude_agl = max(0, altitude - alt_origin)

            # Update velocity if provided
            if velocity_ned is not None:
                self.state.vx, self.state.vy, self.state.vz = velocity_ned

        # Update GPS quality indicators
        self.state.gps_fix = fix_quality > 0
        self.state.num_satellites = num_satellites
        self.state.gps_accuracy = accuracy
        self.last_gps_time = timestamp

    def update_magnetometer(
        self, mag_field: Tuple[float, float, float], timestamp: Optional[float] = None
    ) -> None:
        """Update yaw from magnetometer (optional enhancement)."""
        # Simple magnetic yaw correction
        # In production, implement proper magnetic declination and calibration
        mag_corrected = np.array(mag_field) - self.magnetometer_bias

        # Calculate magnetic yaw (simplified)
        mag_yaw = np.arctan2(mag_corrected[1], mag_corrected[0])

        # Apply gentle correction to prevent magnetic interference issues
        yaw_diff = mag_yaw - self.state.yaw

        # Wrap angle difference
        while yaw_diff > np.pi:
            yaw_diff -= 2 * np.pi
        while yaw_diff < -np.pi:
            yaw_diff += 2 * np.pi

        # Apply small magnetic correction
        self.state.yaw += 0.1 * yaw_diff

    def is_gps_healthy(self) -> bool:
        """Check if GPS data is recent and reliable."""
        gps_age = time.monotonic() - self.last_gps_time
        return (
            self.state.gps_fix
            and gps_age < self.gps_timeout
            and self.state.num_satellites >= 6
            and self.state.gps_accuracy < 5.0
        )

    def get_state(self) -> VehicleState:
        """Get current vehicle state estimate."""
        return self.state
