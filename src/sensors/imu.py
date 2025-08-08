class IMU:
    def __init__(self):
        self.acceleration = (0.0, 0.0, 0.0)
        self.gyroscope = (0.0, 0.0, 0.0)
        self.magnetometer = (0.0, 0.0, 0.0)
        self._initialized = True

    def read_sensor_data(self):
        # Placeholder: update with dummy static values
        self.acceleration = (0.0, 0.0, 9.81)
        self.gyroscope = (0.0, 0.0, 0.0)
        self.magnetometer = (0.3, 0.0, 0.5)

    def get_acceleration(self):
        return self.acceleration

    def get_gyroscope(self):
        return self.gyroscope

    def get_magnetometer(self):
        return self.magnetometer

    def update(self):
        self.read_sensor_data()

    def is_initialized(self) -> bool:
        return self._initialized

    def get_data(self):
        # Return key sensor subsets expected by tests
        return {"acceleration": self.acceleration, "gyroscope": self.gyroscope}
