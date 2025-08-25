from pathlib import Path
import yaml
from typing import Any, Dict


def load_config(config_file):
    """Load configuration settings from a YAML file."""
    with open(config_file, "r") as file:
        config = yaml.safe_load(file)
    return config


def validate_pid_parameters(config: Dict[str, Any]) -> None:
    """Validate PID parameters for safety and sanity."""
    for controller_name, controller_config in config.get("pid_parameters", {}).items():
        for axis_name, axis_config in controller_config.items():
            axis_path = f"{controller_name}.{axis_name}"

            # Validate required parameters
            required_params = ["kp", "ki", "kd"]
            for param in required_params:
                if param not in axis_config:
                    raise ValueError(
                        f"Missing required parameter '{param}' in {axis_path}"
                    )

                value = axis_config[param]
                if not isinstance(value, (int, float)):
                    raise ValueError(
                        f"Parameter '{param}' in {axis_path} must be numeric, got {type(value)}"
                    )

                # Safety checks
                if param == "kp" and value < 0:
                    raise ValueError(
                        f"Kp in {axis_path} must be non-negative, got {value}"
                    )
                if param in ["ki", "kd"] and abs(value) > 100:
                    raise ValueError(
                        f"Parameter '{param}' in {axis_path} seems excessive: {value}"
                    )

            # Validate limits
            for limit_name in ["integral_limits", "output_limits"]:
                if limit_name in axis_config:
                    limits = axis_config[limit_name]
                    if not isinstance(limits, (list, tuple)) or len(limits) != 2:
                        raise ValueError(
                            f"{limit_name} in {axis_path} must be [min, max], got {limits}"
                        )
                    if limits[0] >= limits[1]:
                        raise ValueError(
                            f"{limit_name} in {axis_path} invalid: min >= max"
                        )


def validate_safety_limits(config: Dict[str, Any]) -> None:
    """Validate safety configuration parameters."""
    safety_config = config.get("flight_control", {}).get("safety", {})

    # Critical safety parameters
    critical_limits = {
        "max_tilt_deg": (0, 90),
        "max_climb_rate": (0, 20),
        "max_descent_rate": (0, 20),
        "min_battery_voltage": (6.0, 30.0),
        "max_loop_time_ms": (1.0, 100.0),
        "gps_required_accuracy_m": (0.1, 100.0),
    }

    for param, (min_val, max_val) in critical_limits.items():
        if param in safety_config:
            value = safety_config[param]
            if not isinstance(value, (int, float)):
                raise ValueError(f"Safety parameter '{param}' must be numeric")
            if not (min_val <= value <= max_val):
                raise ValueError(
                    f"Safety parameter '{param}' out of range [{min_val}, {max_val}]: {value}"
                )


def get_pid_parameters():
    """Get PID parameters from the configuration file."""
    root = Path(__file__).resolve().parent.parent.parent  # .../src/utils -> repo root
    config_path = root / "config" / "pid_parameters.yaml"
    raw = load_config(config_path)

    # Validate configuration before returning
    validate_pid_parameters(raw)

    return _normalize_pid_params(raw)


def get_sensor_config():
    """Get sensor configuration settings from the configuration file."""
    root = Path(__file__).resolve().parent.parent.parent
    config_path = root / "config" / "sensor_config.yaml"
    config = load_config(config_path)

    # Validate safety-critical configuration
    validate_safety_limits(config)

    return config


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
