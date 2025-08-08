import unittest
from src.controllers.pid_controller import PIDController


class TestEnhancedPIDController(unittest.TestCase):
    def test_basic_output(self):
        pid = PIDController(1.0, 0.0, 0.0)
        out = pid.calculate_control(10.0, 8.0, 0.1)
        self.assertAlmostEqual(out, 2.0, places=6)

    def test_integral_accumulation(self):
        pid = PIDController(0.0, 1.0, 0.0)
        for _ in range(10):
            pid.calculate_control(1.0, 0.0, 0.1)
        # integral should be approx error * total_time = 1 * 1.0
        self.assertAlmostEqual(pid.integral, 1.0, places=3)

    def test_integral_limits(self):
        pid = PIDController(0.0, 1.0, 0.0, integral_limits=(None, 0.5))
        for _ in range(20):
            pid.calculate_control(1.0, 0.0, 0.1)
        self.assertLessEqual(pid.integral, 0.5)

    def test_output_limits(self):
        pid = PIDController(10.0, 0.0, 0.0, output_limits=(-5.0, 5.0))
        out = pid.calculate_control(2.0, 0.0, 0.1)
        self.assertEqual(out, 5.0)

    def test_derivative_on_measurement(self):
        pid = PIDController(0.0, 0.0, 1.0, derivative_on_measurement=True)
        # first call derivative = 0
        first = pid.calculate_control(0.0, 0.0, 0.1)
        second = pid.calculate_control(0.0, 1.0, 0.1)
        # derivative on measurement gives -(meas - prev_meas)/dt = -(1-0)/0.1 = -10
        self.assertEqual(first, 0.0)
        self.assertAlmostEqual(second, -10.0, places=6)

    def test_reset(self):
        pid = PIDController(1.0, 1.0, 1.0)
        pid.calculate_control(1.0, 0.0, 0.1)
        pid.reset()
        self.assertEqual(pid.integral, 0.0)
        self.assertEqual(pid.previous_error, 0.0)
        self.assertIsNone(pid.previous_measurement)

    def test_dt_guard(self):
        pid = PIDController(1.0, 1.0, 1.0)
        out = pid.calculate_control(2.0, 1.0, 0.0)  # invalid dt => proportional only
        self.assertEqual(out, 1.0)


if __name__ == "__main__":
    unittest.main()
