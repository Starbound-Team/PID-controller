# 🚁 Hardware Integration Guide

This document explains how to deploy the drone PID controller to real hardware for actual flight operations.

## 🎯 Hardware Readiness Status

### ✅ **Production Ready Components**
- **Enhanced PID Controllers**: Anti-windup, saturation limits, derivative filtering
- **Multi-rate Control Loops**: 250Hz attitude, 50Hz altitude, 20Hz position, 5Hz navigation  
- **Flight Mode Management**: DISARMED → ARMED → STABILIZE → ALT_HOLD → POS_HOLD progression
- **State Estimation**: Complementary filter for IMU/GPS sensor fusion
- **Motor Mixer**: Tri-propeller VTOL configuration with geometric mixing
- **Safety Systems**: Tilt limits, emergency stop, hardware monitoring
- **Hardware Abstraction**: Platform-specific drivers with simulation fallback

### 🔧 **Hardware Requirements**

#### **Minimum Hardware Setup**
```
Flight Controller: Raspberry Pi 4B (4GB+ recommended)
IMU: MPU6050 or MPU9250 (I2C)
GPS: u-blox NEO-8M/NEO-9M (UART)
PWM Controller: PCA9685 (I2C) 
ESCs: 3x compatible with 1000-2000μs PWM
Motors: 3x brushless for tri-prop VTOL
Battery: 3S/4S LiPo with voltage monitoring
```

#### **Wiring Connections**
```
Raspberry Pi → MPU6050:
  GPIO 2 (SDA) → MPU6050 SDA
  GPIO 3 (SCL) → MPU6050 SCL
  3.3V → MPU6050 VCC
  GND → MPU6050 GND

Raspberry Pi → PCA9685:
  GPIO 2 (SDA) → PCA9685 SDA  
  GPIO 3 (SCL) → PCA9685 SCL
  5V → PCA9685 VCC
  GND → PCA9685 GND

PCA9685 → ESCs:
  PWM0 → Motor 0 (Front)
  PWM1 → Motor 1 (Rear-Left)
  PWM2 → Motor 2 (Rear-Right)

GPS → Raspberry Pi:
  GPS TX → GPIO 15 (UART RX)
  GPS RX → GPIO 14 (UART TX)
```

## 🚀 **Deployment Steps**

### **1. Hardware Installation**
```bash
# Install hardware dependencies
pip install -e ".[hardware]"

# Enable I2C and UART on Raspberry Pi
sudo raspi-config
# → Interface Options → I2C → Enable
# → Interface Options → Serial → Disable login shell, Enable hardware
```

### **2. Hardware Configuration**
Edit `config/sensor_config.yaml`:
```yaml
sensor:
  imu:
    type: "MPU6050"
    i2c_address: 0x68
    update_rate: 250
  gps:
    type: "UBLOX"
    port: "/dev/ttyS0"
    baud_rate: 38400
  pwm:
    type: "PCA9685"  
    frequency: 50
    pulse_min: 1000  # ESC minimum pulse width
    pulse_max: 2000  # ESC maximum pulse width
```

### **3. Safety Configuration**
Configure safety limits in `config/sensor_config.yaml`:
```yaml
flight_control:
  safety:
    max_tilt_deg: 30.0          # Conservative for first flights
    max_climb_rate: 3.0         # m/s maximum climb
    max_descent_rate: 2.0       # m/s maximum descent
    min_battery_voltage: 10.8   # Automatic landing trigger
    gps_required_accuracy_m: 5.0 # Required GPS accuracy
```

### **4. Pre-flight Calibration**
```bash
# Run calibration sequence
python -m src.flight.calibration

# Expected outputs:
# ✅ IMU bias calibration: [0.02, -0.01, 0.03] rad/s
# ✅ Accelerometer level: [0.1, -0.2, 9.81] m/s²  
# ✅ Motor range check: All ESCs responding
# ✅ GPS fix acquired: 8 satellites, 2.1m accuracy
```

### **5. Hardware Integration Test**
```bash
# Test hardware interfaces
python -c "
from src.flight.hardware import HardwareManager
hw = HardwareManager()
health = hw.is_healthy()
print('Hardware Health:', health)
assert all(health.values()), 'Hardware not ready'
print('✅ All hardware systems operational')
"
```

### **6. Ground Testing**
```bash
# IMPORTANT: Remove propellers for ground testing
python -c "
from src.flight.flight_controller import FlightController, ControlLoopRates
fc = FlightController(enable_hardware=True)
# This will run sensors and actuators but stay DISARMED
fc.run_control_loop(duration=10.0)
"
```

## 🛡️ **Safety Protocols**

### **Pre-flight Checklist**
- [ ] Battery voltage > 11.0V
- [ ] GPS lock with >6 satellites  
- [ ] IMU responding and calibrated
- [ ] All ESCs responding to commands
- [ ] Emergency stop tested and working
- [ ] Flight area clear of obstacles
- [ ] Failsafe radio link tested

### **Flight Mode Progression**
```
1. DISARMED → Manual safety checks complete
2. ARMED → Motors can spin, minimal stick input
3. STABILIZE → Manual attitude control with stabilization
4. ALT_HOLD → Automatic altitude hold + manual attitude
5. POS_HOLD → Automatic position and altitude hold
6. GUIDED → Autonomous navigation (future)
```

### **Emergency Procedures**
- **Emergency stop**: Kill switch cuts all motor power immediately
- **Battery low**: Automatic descent and landing at <10.8V
- **GPS loss**: Automatic return to stabilize mode  
- **IMU failure**: Emergency stop and disarm
- **Excessive tilt**: Emergency stop if >30° roll/pitch

## 📊 **Performance Tuning**

### **PID Parameter Adjustment**
Start with conservative values in `config/pid_parameters.yaml`:
```yaml
pid_parameters:
  attitude:
    roll:
      kp: 0.5      # Start low, increase for responsiveness
      ki: 0.05     # Minimal to avoid windup
      kd: 0.02     # Small for damping
      integral_limits: [-0.2, 0.2]
      output_limits: [-0.5, 0.5]  # Conservative motor authority
```

### **Flight Testing Progression**
1. **Hover Test**: 30-second stationary hover
2. **Attitude Test**: Small roll/pitch commands
3. **Altitude Test**: Climb/descent in ALT_HOLD
4. **Position Test**: GPS hold in light wind
5. **Performance Optimization**: Increase gains for responsiveness

### **Real-time Monitoring**
```bash
# Monitor flight performance
tail -f flight_log_*.csv | grep -E "(Mode|Attitude|GPS)"

# Expected output:
# 2025-01-20 15:30:15,STABILIZE,Roll=2.1°,Pitch=-0.8°,Yaw=45.2°,Alt=2.5m
# 2025-01-20 15:30:16,STABILIZE,GPS=8sat,Acc=1.8m,Pos=(1.2,0.8,2.5)
```

## 🔧 **Troubleshooting**

### **Common Issues**

**Motors not responding:**
```bash
# Check ESC calibration
python -c "
from src.flight.hardware import PCA9685PWM
pwm = PCA9685PWM()
for i in range(3):
    pwm.set_throttle(i, 0.1)  # Low throttle test
    time.sleep(1)
    pwm.set_throttle(i, 0.0)
"
```

**IMU drift:**
```bash
# Recalibrate IMU bias
python -m src.flight.calibration --imu-only
```

**GPS not acquiring fix:**
- Verify antenna connection and positioning
- Check baud rate matches GPS configuration  
- Ensure clear sky view (no indoor testing)

**Loop overruns:**
```bash
# Check system load
htop
# CPU should be <50% for reliable timing
# Consider disabling unnecessary services
```

## 📈 **Performance Metrics**

### **Expected Performance**
- **Control Loop Rates**: 250Hz attitude (±5%), 50Hz altitude
- **GPS Accuracy**: <3m typical, <5m required
- **Battery Life**: 10-15 minutes depending on weight/conditions
- **Response Time**: <100ms for attitude commands
- **Stability**: ±2° hover accuracy in calm conditions

### **Data Logging**
Flight logs automatically saved as CSV:
```
timestamp,mode,roll,pitch,yaw,x,y,z,motor0,motor1,motor2,battery
2025-01-20 15:30:15.123,STABILIZE,2.1,-0.8,45.2,1.2,0.8,2.5,0.45,0.43,0.47,11.8
```

## 🎯 **Production Deployment**

For deployment to actual flight hardware:

1. **Enable Hardware Mode**:
   ```python
   fc = FlightController(enable_hardware=True)  # Real sensors/actuators
   ```

2. **Configure Control Rates**:
   ```python
   rates = ControlLoopRates(
       attitude=250,    # High-rate stabilization
       altitude=50,     # Medium-rate altitude
       position=20,     # GPS-based positioning  
       navigation=5     # Flight mode management
   )
   ```

3. **Set Safety Limits**:
   ```python
   safety = SafetyLimits(
       max_tilt_deg=25.0,       # Conservative for initial flights
       max_climb_rate=3.0,      # Moderate climb rate
       min_battery_voltage=10.8 # Safe landing voltage
   )
   ```

The system is designed for **real-world deployment** with comprehensive safety systems, hardware abstraction, and proven control algorithms. Start with conservative settings and gradually optimize for your specific aircraft configuration.

## ⚠️ **Important Disclaimers**

- **Always test without propellers first**
- **Start with short, low-altitude flights**  
- **Maintain manual override capability**
- **Follow local regulations for UAV operation**
- **This is experimental software - use at your own risk**

This flight controller provides a solid foundation for VTOL aircraft control but requires careful testing and validation for your specific hardware configuration.
