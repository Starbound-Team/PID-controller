from pathlib import Path
import yaml
from typing import Any, Dict


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
    raw = load_config(config_path)
    return _normalize_pid_params(raw)


def get_sensor_config():
    """Get sensor configuration settings from the configuration file."""
    config_path = (
        Path(__file__).resolve().parent.parent / "config" / "sensor_config.yaml"
    )
    return load_config(config_path)


def _normalize_pid_params(cfg: Dict[str, Any]) -> Dict[str, Any]:
    """Internal: ensure limit arrays are tuples for immutability and consistent downstream usage."""
    def convert(node):
        if isinstance(node, dict):
            for k, v in node.items():
                if k in {"integral_limits", "output_limits"} and isinstance(v, list):
                    if len(v) == 2:
                        node[k] = tuple(v)
                else:
                    convert(v)
        elif isinstance(node, list):
            for item in node:
                convert(item)

    convert(cfg)
    return cfg
