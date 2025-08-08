class Airspeed:
    def __init__(self, sensor_pin: int = 0):
        self.sensor_pin = sensor_pin
        self.airspeed = 0.0
        self._initialized = True

    def read_airspeed(self):
        # Placeholder for actual sensor reading logic
        # In a real implementation, this would interface with the airspeed sensor
        self.airspeed = self._get_sensor_data()
        return self.airspeed

    def _get_sensor_data(self):
        # Simulate reading data from the airspeed sensor
        # Replace this with actual sensor reading code
        return 10.0  # Example static value for airspeed in m/s

    def get_airspeed(self):
        return self.read_airspeed()

    # Additional helper methods to align with test expectations
    def is_initialized(self) -> bool:
        return self._initialized

    def get_data(self):
        return {"speed": self.get_airspeed()}
