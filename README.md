# Drone PID Controller

This project implements a **production-ready flight control system** for a fixed-wing VTOL (Vertical Take-Off and Landing) UAV with 3 propellers (2 on wings, 1 on back) using a Raspberry Pi as the flight computer. The system ```bash
# Run all tests (40 tests total)
python -m pytest tests/ -v

# Verify hardware-ready features
python -c "
from src.flight.flight_controller import FlightController
from src.flight.state_estimation import StateEstimator
from src.flight.motor_mixer import TriPropMixer
print('✅ Flight control system ready')
"
``` attitude, position, and velocity control with **hardware integration**, systematic tuning methodologies, and real-time performance visualization.

🚁 **Ready for Real Hardware Deployment** - See [HARDWARE_GUIDE.md](HARDWARE_GUIDE.md) for complete deployment instructions.

## Project Structure

```text
drone-pid-controller/
├── src/
│   ├── controllers/          # Enhanced PID controller implementations
│   │   ├── __init__.py
│   │   ├── pid_controller.py        # Base PID with anti-windup & filtering
│   │   ├── attitude_controller.py   # Roll, pitch, yaw control
│   │   ├── position_controller.py   # 3D position control
│   │   └── velocity_controller.py   # Velocity control
│   ├── flight/              # **NEW** Hardware-ready flight control system
│   │   ├── __init__.py
│   │   ├── flight_controller.py     # Multi-rate flight controller
│   │   ├── flight_modes.py          # Flight mode state machine
│   │   ├── state_estimation.py      # IMU/GPS sensor fusion
│   │   ├── motor_mixer.py           # Tri-propeller motor mixing
│   │   └── hardware.py              # Hardware abstraction layer
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
├── tests/                   # Automated tests (40 passing tests)
│   ├── __init__.py
│   ├── test_controllers.py        # Controller API & basic behavior
│   ├── test_flight_systems.py     # **NEW** Flight control system tests
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
│   └── sensor_config.yaml   # Sensor & flight control configuration
├── docs/                    # Documentation
│   ├── controller_design.md
│   ├── tuning_methodology.md
│   └── performance_analysis.md
├── HARDWARE_GUIDE.md        # **NEW** Complete hardware deployment guide
├── requirements.txt         # Python dependencies
├── requirements-dev.txt     # Development dependencies (Windows)
├── setup.py                # Package setup script
├── .gitignore              # Git ignore rules
└── README.md               # This file
```

## Features

### 🚁 **Hardware-Ready Flight Control System**

- **Multi-rate Control Loops**: 250Hz attitude, 50Hz altitude, 20Hz position scheduling
- **Flight Mode Management**: DISARMED → ARMED → STABILIZE → ALT_HOLD → POS_HOLD progression  
- **State Estimation**: Complementary filter for IMU/GPS sensor fusion
- **Motor Mixer**: Tri-propeller VTOL configuration with geometric mixing
- **Hardware Abstraction**: Platform-specific drivers for Raspberry Pi deployment
- **Safety Systems**: Tilt limits, emergency stop, battery monitoring, GPS health checks

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
- Real hardware drivers: MPU6050 IMU, u-blox GPS, PCA9685 PWM controller
- Configurable YAML-based sensor and PID parameters

### Packaging & Extras

- Editable install with optional extras: `[tuning]`, `[hardware]`, `[dev]`, `[all]`
- Cross-platform friendly (Windows dev, Raspberry Pi deploy)

### Testing & Quality

- **40 automated tests** (controllers, enhanced PID, metrics, sensors, tuning, visuals, flight systems)
- Headless plots avoid GUI dependencies in CI

### Documentation

- Design, tuning methodology, performance analysis docs
- **[HARDWARE_GUIDE.md](HARDWARE_GUIDE.md)**: Complete hardware deployment guide
- Extended README with examples & configuration guidance

## Hardware Requirements

### **For Development**
- **Development Platform**: Windows 10/11 or Linux with Python 3.8+
- **Simulation Mode**: No hardware required for development and testing

### **For Real Hardware Deployment** 
- **Flight Computer**: Raspberry Pi 4B (4GB+ recommended) or Raspberry Pi 3B+
- **IMU**: MPU6050 or MPU9250 (I2C interface)
- **GPS**: u-blox NEO-8M/NEO-9M (UART interface)  
- **PWM Controller**: PCA9685 (I2C interface)
- **ESCs**: 3x compatible with 1000-2000μs PWM signals
- **Motors**: 3x brushless motors for tri-propeller VTOL configuration
- **Power**: 3S/4S LiPo battery with voltage monitoring
- **Frame**: Fixed-wing VTOL airframe with 3 motor mounts

**📖 Complete hardware setup guide:** [HARDWARE_GUIDE.md](HARDWARE_GUIDE.md)

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

#### 1. **Development & Testing** (Simulation Mode)

```bash
# Run unit tests
python -m pytest tests/test_controllers.py -v

# Run flight control system tests  
python -m pytest tests/test_flight_systems.py -v

# Run simulation mode (no hardware required)
python src/main.py
```

#### 2. **Hardware Deployment** (Real Drone)

```bash
# Install with hardware dependencies
pip install -e ".[hardware]"

# Test hardware interfaces
python -c "
from src.flight.hardware import HardwareManager
hw = HardwareManager()
print('Hardware Health:', hw.is_healthy())
"

# Run flight controller on real hardware
python -c "
from src.flight.flight_controller import FlightController
fc = FlightController(enable_hardware=True)
fc.run_control_loop(duration=30.0)  # 30-second test
"
```

📖 **Complete deployment guide**: [HARDWARE_GUIDE.md](HARDWARE_GUIDE.md)

#### 3. Visual Testing & Tuning

```bash
python -m pytest tests/test_controllers.py -v
```

```bash
# Run visual tests
python visual_testing/test_visual.py

# Interactive PID tuning
python interactive_tuner.py
```

#### 4. System Validation

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
