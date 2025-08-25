# Drone PID Controller

A **production-ready flight control system** for tri-propeller VTOL drones using Raspberry Pi. Features multi-rate control loops, hardware integration, safety systems, and comprehensive testing.

🚁 **Ready for Real Hardware Deployment** - See [HARDWARE_GUIDE.md](HARDWARE_GUIDE.md) for complete setup instructions.

## Features

### 🚁 Hardware-Ready Flight Control

- **Multi-rate Control**: 250Hz attitude, 50Hz altitude, 20Hz position control loops
- **Flight Modes**: DISARMED → ARMED → STABILIZE → ALT_HOLD → POS_HOLD
- **State Estimation**: IMU/GPS sensor fusion with complementary filtering
- **Motor Mixer**: Tri-propeller VTOL configuration with geometric mixing
- **Safety Systems**: Emergency stop, tilt limits, battery monitoring, health checks

### 🎛️ Advanced PID Controllers

- Integral windup protection with configurable limits
- Output saturation with optional integrator freeze
- Derivative on measurement (anti-kick) with low-pass filtering
- YAML-based configuration for all parameters

### 🔧 Hardware Support

- **IMU**: MPU6050/MPU9250 (I2C)
- **GPS**: u-blox NEO-8M/9M (UART)
- **PWM**: PCA9685 controller (I2C)
- **Platform**: Raspberry Pi 4B (recommended) or 3B+

### 📊 Tuning & Analysis

- Interactive GUI tuning with real-time visualization
- Automatic performance metrics (settling time, overshoot, rise time)
- Ziegler-Nichols parameter estimation
- Comprehensive testing suite (40+ tests)

## Quick Start

### Development (Windows/Linux)

```bash
# Clone repository
git clone https://github.com/Starbound-Team/PID-controller.git
cd drone-pid-controller

# Create virtual environment
python -m venv venv
source venv/bin/activate  # Linux/Mac
# venv\Scripts\activate   # Windows

# Install package
pip install -e .[dev]

# Run tests
python -m pytest tests/ -v

# Test simulation
python src/main.py
```

### Hardware Deployment (Raspberry Pi)

```bash
# Install with hardware support
pip install -e .[hardware]

# Test hardware
python -c "
from src.flight.hardware import HardwareManager
hw = HardwareManager()
print('Hardware Status:', hw.is_healthy())
"

# Run flight controller
python -c "
from src.flight.flight_controller import FlightController
fc = FlightController(enable_hardware=True)
fc.run_control_loop(duration=30.0)
"
```

📖 **Complete hardware guide**: [HARDWARE_GUIDE.md](HARDWARE_GUIDE.md)

## Installation Options

The `setup.py` script provides flexible installation with optional dependencies:

### Basic Installation

```bash
pip install .                    # Core functionality only
```

### With Optional Features

```bash
pip install .[tuning]           # + scipy, matplotlib for analysis
pip install .[hardware]         # + RPi.GPIO, smbus2 for Raspberry Pi
pip install .[dev]              # + pytest for development
pip install .[all]              # Everything included
```

### Development Mode

```bash
pip install -e .[all]           # Editable install - changes take effect immediately
```

### Platform Intelligence

The `setup.py` automatically detects your platform:

- **Linux (Raspberry Pi)**: Installs `RPi.GPIO`, `smbus2` for real hardware
- **Windows/Mac**: Installs `fake-rpi` for simulation and development

### From GitHub

```bash
# Install directly from repository
pip install git+https://github.com/Starbound-Team/PID-controller.git

# With specific features
pip install "drone-pid-controller[hardware] @ git+https://github.com/Starbound-Team/PID-controller.git"
```

## Project Structure

```text
drone-pid-controller/
├── src/                     # Main source code
│   ├── controllers/         # PID controllers
│   ├── flight/             # Flight control system
│   ├── sensors/            # Sensor interfaces
│   ├── tuning/             # Analysis tools
│   └── utils/              # Utilities
├── tests/                  # Test suite (40+ tests)
├── config/                 # YAML configurations
├── docs/                   # Documentation
├── setup.py               # Package installation script
└── requirements.txt       # Dependencies
```

## Configuration

### PID Parameters (`config/pid_parameters.yaml`)

```yaml
pid_parameters:
  attitude:
    roll:
      kp: 1.0
      ki: 0.1
      kd: 0.05
      integral_limits: [-0.5, 0.5]
      output_limits: [-1.0, 1.0]
```

### Sensor Setup (`config/sensor_config.yaml`)

```yaml
sensor:
  imu:
    type: "MPU6050"
    i2c_address: 0x68
    update_rate: 250
  flight_control:
    safety:
      max_tilt_deg: 30.0
      min_battery_voltage: 10.8
```

## Testing

```bash
# Run all tests
python -m pytest tests/ -v

# Specific test categories
python -m pytest tests/test_controllers.py      # PID controllers
python -m pytest tests/test_flight_systems.py   # Flight control
python -m pytest tests/test_sensors.py          # Hardware interfaces

# Visual testing and tuning
python visual_testing/test_visual.py
python visual_testing/interactive_tuner.py
```

## Safety Features

- ✅ Emergency stop with timeout protection
- ✅ Signal handlers for graceful shutdown
- ✅ Health monitoring for all hardware components
- ✅ Configuration validation with safe ranges
- ✅ Multi-level failsafe systems
- ✅ Real-time performance monitoring

## Hardware Requirements

### Development

- Windows 10/11, Linux, or macOS
- Python 3.8+
- No hardware required (simulation mode)

### Production Deployment

- Raspberry Pi 4B (4GB+ recommended) or 3B+
- MPU6050/MPU9250 IMU (I2C)
- u-blox NEO-8M/9M GPS (UART)
- PCA9685 PWM controller (I2C)
- 3x ESCs and brushless motors
- 3S/4S LiPo battery with monitoring

## Contributing

1. Fork the repository
2. Create feature branch: `git checkout -b feature-name`
3. Add tests for new functionality
4. Run test suite: `python -m pytest tests/ -v`
5. Submit pull request

## License

MIT License - see LICENSE file for details.

## Support

- **Documentation**: Check `docs/` directory
- **Issues**: GitHub Issues for bugs and features
- **Hardware Guide**: [HARDWARE_GUIDE.md](HARDWARE_GUIDE.md)
