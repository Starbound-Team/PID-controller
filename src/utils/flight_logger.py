"""Enhanced logging system for flight control with structured logging."""

import logging
import sys
import time
from pathlib import Path
from typing import Optional, Dict, Any
from enum import Enum


class LogLevel(Enum):
    """Flight control specific log levels."""

    CRITICAL_SAFETY = 50  # Safety-critical events
    ERROR = 40  # Error events
    WARNING = 30  # Warning events
    INFO = 20  # Informational events
    DEBUG = 10  # Debug events
    TRACE = 5  # Detailed trace events


class FlightLogger:
    """Enhanced logger for flight control systems."""

    def __init__(self, name: str = "flight_control", log_file: Optional[str] = None):
        self.logger = logging.getLogger(name)
        self.logger.setLevel(LogLevel.TRACE.value)

        # Remove existing handlers
        self.logger.handlers.clear()

        # Create formatters
        self.console_formatter = logging.Formatter(
            "%(asctime)s | %(levelname)-8s | %(name)s | %(message)s", datefmt="%H:%M:%S"
        )

        self.file_formatter = logging.Formatter(
            "%(asctime)s | %(levelname)-8s | %(name)s | %(filename)s:%(lineno)d | %(message)s",
            datefmt="%Y-%m-%d %H:%M:%S",
        )

        # Console handler
        console_handler = logging.StreamHandler(sys.stdout)
        console_handler.setLevel(LogLevel.INFO.value)
        console_handler.setFormatter(self.console_formatter)
        self.logger.addHandler(console_handler)

        # File handler (if specified)
        if log_file:
            file_handler = logging.FileHandler(log_file)
            file_handler.setLevel(LogLevel.DEBUG.value)
            file_handler.setFormatter(self.file_formatter)
            self.logger.addHandler(file_handler)

    def critical_safety(self, msg: str, **kwargs) -> None:
        """Log safety-critical events."""
        self._log(LogLevel.CRITICAL_SAFETY, f"🚨 SAFETY CRITICAL: {msg}", **kwargs)

    def error(self, msg: str, **kwargs) -> None:
        """Log error events."""
        self._log(LogLevel.ERROR, f"❌ ERROR: {msg}", **kwargs)

    def warning(self, msg: str, **kwargs) -> None:
        """Log warning events."""
        self._log(LogLevel.WARNING, f"⚠️  WARNING: {msg}", **kwargs)

    def info(self, msg: str, **kwargs) -> None:
        """Log informational events."""
        self._log(LogLevel.INFO, f"ℹ️  {msg}", **kwargs)

    def debug(self, msg: str, **kwargs) -> None:
        """Log debug events."""
        self._log(LogLevel.DEBUG, f"🐛 DEBUG: {msg}", **kwargs)

    def trace(self, msg: str, **kwargs) -> None:
        """Log trace events."""
        self._log(LogLevel.TRACE, f"🔍 TRACE: {msg}", **kwargs)

    def flight_event(self, event: str, data: Dict[str, Any]) -> None:
        """Log structured flight events."""
        event_msg = f"FLIGHT_EVENT: {event}"
        if data:
            data_str = ", ".join([f"{k}={v}" for k, v in data.items()])
            event_msg += f" | {data_str}"
        self.info(event_msg)

    def performance(
        self, operation: str, duration_ms: float, details: Optional[Dict] = None
    ) -> None:
        """Log performance metrics."""
        msg = f"PERFORMANCE: {operation} took {duration_ms:.3f}ms"
        if details:
            detail_str = ", ".join([f"{k}={v}" for k, v in details.items()])
            msg += f" | {detail_str}"
        self.debug(msg)

    def sensor_data(self, sensor: str, healthy: bool, data: Dict[str, Any]) -> None:
        """Log sensor data and health."""
        status = "HEALTHY" if healthy else "UNHEALTHY"
        msg = f"SENSOR: {sensor} [{status}]"
        if data:
            data_str = ", ".join([f"{k}={v}" for k, v in data.items()])
            msg += f" | {data_str}"
        self.trace(msg)

    def control_output(
        self, controller: str, setpoint: float, measurement: float, output: float
    ) -> None:
        """Log control system outputs."""
        msg = f"CONTROL: {controller} | SP={setpoint:.3f}, PV={measurement:.3f}, CV={output:.3f}"
        self.trace(msg)

    def safety_check(
        self, check: str, passed: bool, value: float, limit: float
    ) -> None:
        """Log safety check results."""
        status = "PASS" if passed else "FAIL"
        msg = f"SAFETY_CHECK: {check} [{status}] | value={value:.3f}, limit={limit:.3f}"
        if not passed:
            self.warning(msg)
        else:
            self.trace(msg)

    def _log(self, level: LogLevel, msg: str, **kwargs) -> None:
        """Internal logging method."""
        self.logger.log(level.value, msg, **kwargs)


# Global logger instance
_flight_logger: Optional[FlightLogger] = None


def get_flight_logger() -> FlightLogger:
    """Get or create global flight logger."""
    global _flight_logger
    if _flight_logger is None:
        # Create logs directory if needed
        log_dir = Path("logs")
        log_dir.mkdir(exist_ok=True)

        # Create timestamped log file
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        log_file = log_dir / f"flight_control_{timestamp}.log"

        _flight_logger = FlightLogger(log_file=str(log_file))
    return _flight_logger


def setup_logging(
    log_level: str = "INFO", log_file: Optional[str] = None
) -> FlightLogger:
    """Setup enhanced logging system."""
    global _flight_logger

    # Map string levels to enum
    level_map = {
        "CRITICAL": LogLevel.CRITICAL_SAFETY,
        "ERROR": LogLevel.ERROR,
        "WARNING": LogLevel.WARNING,
        "INFO": LogLevel.INFO,
        "DEBUG": LogLevel.DEBUG,
        "TRACE": LogLevel.TRACE,
    }

    level = level_map.get(log_level.upper(), LogLevel.INFO)

    _flight_logger = FlightLogger(log_file=log_file)
    _flight_logger.logger.setLevel(level.value)

    return _flight_logger
