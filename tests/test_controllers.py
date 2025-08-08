import unittest
from src.controllers.pid_controller import PIDController
from src.controllers.attitude_controller import AttitudeController
from src.controllers.position_controller import PositionController
from src.controllers.velocity_controller import VelocityController


class TestPIDController(unittest.TestCase):
    def setUp(self):
        self.pid = PIDController(1.0, 0.1, 0.05)

    def test_set_parameters(self):
        self.pid.set_parameters(2.0, 0.2, 0.1)
        self.assertEqual(self.pid.kp, 2.0)
        self.assertEqual(self.pid.ki, 0.2)
        self.assertEqual(self.pid.kd, 0.1)

    def test_calculate_output(self):
        output = self.pid.calculate_control(1.0, 0.0, 0.1)
        self.assertIsInstance(output, float)


class TestAttitudeController(unittest.TestCase):
    def setUp(self):
        common = dict(kp=1.0, ki=0.1, kd=0.05, integral_limits=(-1, 1), output_limits=(-2, 2))
        self.attitude_controller = AttitudeController(common, common, common)

    def test_roll_control(self):
        output = self.attitude_controller.control(10.0, 5.0, 0.0, 0.0, 0.0, 0.0, 0.1)
        self.assertIsInstance(output, tuple)
        self.assertEqual(len(output), 3)  # roll, pitch, yaw outputs

    def test_pitch_control(self):
        output = self.attitude_controller.control(0.0, 0.0, 10.0, 5.0, 0.0, 0.0, 0.1)
        self.assertIsInstance(output, tuple)
        self.assertEqual(len(output), 3)  # roll, pitch, yaw outputs

    def test_yaw_control(self):
        output = self.attitude_controller.control(0.0, 0.0, 0.0, 0.0, 10.0, 5.0, 0.1)
        self.assertIsInstance(output, tuple)
        self.assertEqual(len(output), 3)  # roll, pitch, yaw outputs


class TestPositionController(unittest.TestCase):
    def setUp(self):
        params = dict(kp=1.0, ki=0.1, kd=0.05)
        self.position_controller = PositionController(params, params, params)

    def test_position_control(self):
        output = self.position_controller.control(10.0, 8.0, 5.0, 3.0, 2.0, 1.0, 0.1)
        self.assertIsInstance(output, tuple)


class TestVelocityController(unittest.TestCase):
    def setUp(self):
        params = dict(kp=1.0, ki=0.1, kd=0.05)
        self.velocity_controller = VelocityController(params)

    def test_velocity_control(self):
        output = self.velocity_controller.control_velocity(10.0, 5.0, 0.1)
        self.assertIsInstance(output, float)


if __name__ == "__main__":
    unittest.main()
