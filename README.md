# Drone PID Controller

This project implements a comprehensive PID controller system for a fixed-wing VTOL (Vertical Take-Off and Landing) UAV with 3 propellers (2 on wings, 1 on back) using a Raspberry Pi as the flight computer. The system provides robust attitude, position, and velocity control with systematic tuning methodologies and real-time performance visualization.

## Project Structure

```text
drone-pid-controller/
├── src/
│   ├── controllers/          # PID controller implementations
│   │   ├── __init__.py
│   │   ├── pid_controller.py        # Base PID controller class
│   │   ├── attitude_controller.py   # Roll, pitch, yaw control
│   │   ├── position_controller.py   # 3D position control
│   │   └── velocity_controller.py   # Velocity control
│   ├── sensors/              # Sensor data acquisition modules
│   │   ├── __init__.py
│   │   ├── imu.py           # Inertial Measurement Unit
│   │   ├── gps.py           # GPS positioning
│   │   └── airspeed.py      # Airspeed sensor
│   ├── tuning/              # Tuning methodologies and analysis
│   │   ├── __init__.py
│   │   ├── parameter_estimation.py
│   │   ├── performance_analyzer.py
│   │   ├── performance_metrics.py   # KPI utilities (settling time, overshoot, etc.)
│   │   ├── ultimate_gain_finder.py  # Ku / Pu sweep (Ziegler–Nichols start)
│   │   └── step_response.py
│   ├── utils/               # Utility functions
│   │   ├── __init__.py
│   │   ├── config.py        # Configuration management
│   │   └── data_logger.py   # Data logging utilities
│   └── main.py              # Main application entry point
├── tests/                   # Automated tests
│   ├── __init__.py
│   ├── test_controllers.py        # Controller API & basic behavior
│   ├── test_pid_enhanced.py       # Enhanced PID robustness (limits, filter, freeze)
│   ├── test_performance_metrics.py# KPI regression tests
│   ├── test_sensors.py            # Sensor interface tests
│   └── test_tuning.py             # Legacy wrappers / placeholders
├── visual_testing/          # Visual testing and analysis (headless-capable)
│   ├── __init__.py
│   ├── test_visual.py       # Static visual tests
│   └── interactive_tuner.py # Interactive PID tuning tool
├── config/                  # Configuration files
│   ├── pid_parameters.yaml  # PID parameter settings
│   └── sensor_config.yaml   # Sensor configuration
├── docs/                    # Documentation
│   ├── controller_design.md
│   ├── tuning_methodology.md
│   └── performance_analysis.md
├── requirements.txt         # Python dependencies
├── requirements-dev.txt     # Development dependencies (Windows)
├── setup.py                # Package setup script
├── .gitignore              # Git ignore rules
└── README.md               # This file
```

## Features

### Control & Architecture

- Layered controllers: Attitude (roll/pitch/yaw), Position (x/y/z), Velocity
- Unified enhanced PID core reused across all axes

### Robust PID Enhancements

- Integral windup protection (configurable clamp)
- Output saturation with optional integrator freeze
- Derivative on measurement vs. error (anti-kick)
- Optional derivative low-pass filter (`derivative_filter_tau`)
- Config-driven per-axis limits & filtering
- dt validation / safe fallback

### Tuning & Analysis

- Interactive slider-based tuning GUI
- Headless visual regression (Agg backend) for CI
- Automatic ultimate gain (Ku) & oscillation period (Pu) sweep (Ziegler–Nichols suggestions)
- Reusable performance metrics (settling time, rise time, overshoot, steady-state error)
- Step response & parameter comparison utilities

### Sensors & Simulation

- Mock IMU / GPS / Airspeed with consistent API for development & tests
- Configurable YAML-based sensor and PID parameters

### Packaging & Extras

- Editable install with optional extras: `[tuning]`, `[hardware]`, `[dev]`, `[all]`
- Cross-platform friendly (Windows dev, Raspberry Pi deploy)

### Testing & Quality

- 28 automated tests (controllers, enhanced PID, metrics, sensors, tuning, visuals)
- Headless plots avoid GUI dependencies in CI

### Documentation

- Design, tuning methodology, performance analysis docs
- Extended README with examples & configuration guidance

## Hardware Requirements

- **Flight Computer**: Raspberry Pi 4 (recommended) or Raspberry Pi 3B+
- **Sensors**:
  - IMU (e.g., MPU-6050, MPU-9250)
  - GPS module (e.g., NEO-8M)
  - Airspeed sensor (optional, for advanced control)
- **Vehicle**: Fixed-wing VTOL with 3 propellers (2 wing-mounted, 1 rear)

## Software Requirements

- Python 3.6+ (3.8+ recommended)
- Required packages (see Installation section)

## Getting Started

### Prerequisites

- **Development Environment**: Windows 10/11 with Python 3.6+
- **Target Hardware**: Raspberry Pi 4 with Raspbian OS
- **Python Packages**: Listed in requirements.txt

### Installation

#### For Development (Windows)

1. **Clone the repository**:

   ```bash
   git clone https://github.com/Starbound-Team/PID-controller.git
   cd drone-pid-controller
   ```

2. **Create virtual environment**:

   ```bash
   python -m venv venv
   venv\Scripts\activate
   ```

3. **Install development dependencies**:

   ```bash
   pip install -r requirements-dev.txt
   ```

4. **Install the package in development mode**:

   ```bash
   pip install -e .
   ```

#### For Raspberry Pi Deployment

1. **Clone and install**:

   ```bash
   git clone https://github.com/Starbound-Team/PID-controller.git
   cd drone-pid-controller
   pip install -r requirements.txt
   pip install -e .
   ```

### Quick Start

#### 1. Run Unit Tests

```bash
python -m pytest tests/test_controllers.py -v
```

#### 2. Run Visual Tests

```bash
python visual_testing/test_visual.py
```

#### 3. Interactive PID Tuning

```bash
python interactive_tuner.py
```

#### 4. Run Main Application

```bash
python src/main.py
```

## Testing

The project includes comprehensive testing tools:

### Unit Tests

- **Controller Tests**: Verify PID logic and calculations
- **Sensor Tests**: Test sensor interfaces and data processing
- **Integration Tests**: Test component interactions

```bash
# Run all tests
python -m pytest tests/ -v

# Run specific test file
python -m pytest tests/test_controllers.py -v

# Run with coverage
python -m pytest tests/ --cov=src/controllers
```

### Visual Testing

- **Step Response Analysis**: Visualize controller performance
- **Parameter Comparison**: Compare different tuning approaches
- **Real-time Plotting**: Monitor controller behavior

```bash
python visual_testing/test_visual.py
```

### Interactive Tuning

- **Real-time Parameter Adjustment**: Slider-based tuning interface
- **Live Response Visualization**: See changes immediately
- **Performance Metrics**: Automatic calculation of key metrics

```bash
python interactive_tuner.py
```

## Configuration

### PID Parameters

Edit `config/pid_parameters.yaml` to adjust controller gains:

```yaml
attitude_controller:
  roll:
    kp: 1.0
    ki: 0.1
    kd: 0.05
  pitch:
    kp: 1.0
    ki: 0.1
    kd: 0.05
  yaw:
    kp: 1.0
    ki: 0.1
    kd: 0.05

position_controller:
  kp: 1.0
  ki: 0.1
  kd: 0.05
```

### Sensor Configuration

Configure sensors in `config/sensor_config.yaml`:

```yaml
imu:
  device: "/dev/i2c-1"
  address: 0x68
  sample_rate: 100

gps:
  device: "/dev/ttyAMA0"
  baud_rate: 9600

airspeed:
  device: "/dev/i2c-1"
  address: 0x77
```

## Performance Metrics

The testing tools calculate key performance indicators:

- **Settling Time**: Time to reach within 2% of setpoint
- **Overshoot**: Maximum deviation beyond setpoint (%)
- **Steady-State Error**: Final error from desired setpoint
- **Rise Time**: Time to reach 90% of setpoint value

## Contributing

1. **Fork the repository**
2. **Create a feature branch**: `git checkout -b feature-name`
3. **Make your changes and add tests**
4. **Run the test suite**: `python -m pytest tests/ -v`
5. **Submit a pull request**

### Development Guidelines

- Follow PEP 8 style guidelines
- Add unit tests for new functionality
- Update documentation as needed
- Test on both Windows (development) and Raspberry Pi (target)

## License

This project is licensed under the MIT License. See the LICENSE file for details.

## References

- **Concept Paper Section 2.4**: Control System - [TURAG Control Systems](https://turag.de/wp-content/uploads/2021/11/Test.pdf)
- **CubePilot Orange Plus in Simulink**: [MathWorks UAV Documentation](https://www.mathworks.com/help/uav/px4/ug/deployment-cube-orange-autopilot-simulink.html)
- **Mission Planner**: Use "Airplane" firmware for VTOL vehicles
- **Hardware Setup**: Download CubePilot add-ons via MATLAB Add-Ons button

## Support

For questions and support:

- **Issues**: Use GitHub Issues for bug reports and feature requests
- **Documentation**: Check the `docs/` directory for detailed guides
- **Examples**: See `visual_testing/` for usage examples
