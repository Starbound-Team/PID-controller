class GPS:
    def __init__(self):
        self.latitude = 0.0
        self.longitude = 0.0
        self.altitude = 0.0
        self._initialized = True

    def update(self, latitude, longitude, altitude):
        self.latitude = latitude
        self.longitude = longitude
        self.altitude = altitude

    def get_position(self):
        return {
            "latitude": self.latitude,
            "longitude": self.longitude,
            "altitude": self.altitude,
        }

    def is_initialized(self) -> bool:
        return self._initialized

    def get_data(self):
        return {"latitude": self.latitude, "longitude": self.longitude}
