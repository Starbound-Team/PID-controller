import time
from controllers.attitude_controller import AttitudeController
from controllers.position_controller import PositionController
from controllers.velocity_controller import VelocityController
from sensors.imu import IMU
from sensors.gps import GPS
from sensors.airspeed import Airspeed
from tuning.parameter_estimation import estimate_parameters
from utils.data_logger import DataLogger
from utils.config import load_config

def main():
    # Load configuration settings
    config = load_config('config/sensor_config.yaml')
    
    # Initialize sensors
    imu = IMU(config['imu'])
    gps = GPS(config['gps'])
    airspeed = Airspeed(config['airspeed'])
    
    # Initialize controllers
    attitude_controller = AttitudeController()
    position_controller = PositionController()
    velocity_controller = VelocityController()
    
    # Estimate initial PID parameters
    estimate_parameters(attitude_controller)
    estimate_parameters(position_controller)
    estimate_parameters(velocity_controller)
    
    # Initialize data logger
    data_logger = DataLogger()
    
    # Start flight control loop
    while True:
        # Read sensor data
        imu_data = imu.read()
        gps_data = gps.read()
        airspeed_data = airspeed.read()
        
        # Update controllers with sensor data
        attitude_control_output = attitude_controller.update(imu_data)
        position_control_output = position_controller.update(gps_data)
        velocity_control_output = velocity_controller.update(airspeed_data)
        
        # Log data
        data_logger.log(imu_data, gps_data, airspeed_data, 
                        attitude_control_output, position_control_output, velocity_control_output)
        
        # Sleep for a short duration to maintain loop rate
        time.sleep(0.01)

if __name__ == "__main__":
    main()