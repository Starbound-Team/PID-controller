import unittest
from src.sensors.imu import IMU
from src.sensors.gps import GPS
from src.sensors.airspeed import Airspeed


class TestSensors(unittest.TestCase):

    def setUp(self):
        self.imu = IMU()
        self.gps = GPS()
        self.airspeed = Airspeed()

    def test_imu_initialization(self):
        self.assertIsNotNone(self.imu)
        self.assertTrue(self.imu.is_initialized())

    def test_gps_initialization(self):
        self.assertIsNotNone(self.gps)
        self.assertTrue(self.gps.is_initialized())

    def test_airspeed_initialization(self):
        self.assertIsNotNone(self.airspeed)
        self.assertTrue(self.airspeed.is_initialized())

    def test_imu_data_acquisition(self):
        data = self.imu.get_data()
        self.assertIsInstance(data, dict)
        self.assertIn("acceleration", data)
        self.assertIn("gyroscope", data)

    def test_gps_data_acquisition(self):
        data = self.gps.get_data()
        self.assertIsInstance(data, dict)
        self.assertIn("latitude", data)
        self.assertIn("longitude", data)

    def test_airspeed_data_acquisition(self):
        data = self.airspeed.get_data()
        self.assertIsInstance(data, dict)
        self.assertIn("speed", data)


if __name__ == "__main__":
    unittest.main()
