import unittest
from src.tuning.parameter_estimation import estimate_parameters
from src.tuning.step_response import conduct_step_response_test
from src.tuning.performance_analyzer import analyze_performance


class TestTuningMethods(unittest.TestCase):

    def test_estimate_parameters(self):
        # Test initial parameter estimation
        params = estimate_parameters()
        self.assertIsNotNone(params)
        self.assertIn("Kp", params)
        self.assertIn("Ki", params)
        self.assertIn("Kd", params)

    def test_conduct_step_response_test(self):
        # Test step response functionality
        response = conduct_step_response_test()
        self.assertIsNotNone(response)
        self.assertTrue(len(response) > 0)

    def test_analyze_performance(self):
        # Test performance analysis
        performance_metrics = analyze_performance()
        self.assertIsNotNone(performance_metrics)
        self.assertIn("stability", performance_metrics)
        self.assertIn("response_time", performance_metrics)


if __name__ == "__main__":
    unittest.main()
