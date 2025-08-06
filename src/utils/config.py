from pathlib import Path
import yaml


def load_config(config_file):
    """Load configuration settings from a YAML file."""
    with open(config_file, "r") as file:
        config = yaml.safe_load(file)
    return config


def get_pid_parameters():
    """Get PID parameters from the configuration file."""
    config_path = (
        Path(__file__).resolve().parent.parent / "config" / "pid_parameters.yaml"
    )
    return load_config(config_path)


def get_sensor_config():
    """Get sensor configuration settings from the configuration file."""
    config_path = (
        Path(__file__).resolve().parent.parent / "config" / "sensor_config.yaml"
    )
    return load_config(config_path)
