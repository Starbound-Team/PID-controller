"""Performance monitoring and profiling tools for flight control system."""

import time
import statistics
from typing import Dict, List, Optional
from dataclasses import dataclass, field
from collections import defaultdict, deque


@dataclass
class PerformanceMetrics:
    """Performance metrics for a named operation."""

    name: str
    call_count: int = 0
    total_time: float = 0.0
    min_time: float = float("inf")
    max_time: float = 0.0
    recent_times: deque = field(default_factory=lambda: deque(maxlen=100))

    def update(self, execution_time: float) -> None:
        """Update metrics with new execution time."""
        self.call_count += 1
        self.total_time += execution_time
        self.min_time = min(self.min_time, execution_time)
        self.max_time = max(self.max_time, execution_time)
        self.recent_times.append(execution_time)

    @property
    def average_time(self) -> float:
        """Average execution time."""
        return self.total_time / self.call_count if self.call_count > 0 else 0.0

    @property
    def recent_average(self) -> float:
        """Recent average execution time (last 100 calls)."""
        return statistics.mean(self.recent_times) if self.recent_times else 0.0

    @property
    def recent_std(self) -> float:
        """Recent standard deviation."""
        return (
            statistics.stdev(self.recent_times) if len(self.recent_times) > 1 else 0.0
        )


class PerformanceMonitor:
    """Real-time performance monitoring for flight control loops."""

    def __init__(self, max_history: int = 1000):
        self.metrics: Dict[str, PerformanceMetrics] = {}
        self.max_history = max_history
        self._start_times: Dict[str, float] = {}

        # Loop timing tracking
        self.loop_times = deque(maxlen=max_history)
        self.loop_frequencies = deque(maxlen=max_history)
        self.last_loop_time = time.monotonic()

    def start_timing(self, operation: str) -> None:
        """Start timing an operation."""
        self._start_times[operation] = time.perf_counter()

    def end_timing(self, operation: str) -> float:
        """End timing and record performance metrics."""
        if operation not in self._start_times:
            return 0.0

        execution_time = time.perf_counter() - self._start_times[operation]
        del self._start_times[operation]

        if operation not in self.metrics:
            self.metrics[operation] = PerformanceMetrics(operation)

        self.metrics[operation].update(execution_time)
        return execution_time

    def record_loop_timing(self) -> None:
        """Record main control loop timing."""
        now = time.monotonic()
        loop_time = now - self.last_loop_time

        self.loop_times.append(loop_time)
        if loop_time > 0:
            frequency = 1.0 / loop_time
            self.loop_frequencies.append(frequency)

        self.last_loop_time = now

    def get_summary(self) -> Dict[str, Dict]:
        """Get performance summary for all operations."""
        summary = {}

        for name, metrics in self.metrics.items():
            summary[name] = {
                "calls": metrics.call_count,
                "avg_ms": metrics.average_time * 1000,
                "recent_avg_ms": metrics.recent_average * 1000,
                "min_ms": metrics.min_time * 1000,
                "max_ms": metrics.max_time * 1000,
                "std_ms": metrics.recent_std * 1000,
                "total_s": metrics.total_time,
            }

        # Add loop statistics
        if self.loop_times:
            summary["control_loop"] = {
                "avg_period_ms": statistics.mean(self.loop_times) * 1000,
                "avg_frequency_hz": (
                    statistics.mean(self.loop_frequencies)
                    if self.loop_frequencies
                    else 0
                ),
                "min_period_ms": min(self.loop_times) * 1000,
                "max_period_ms": max(self.loop_times) * 1000,
                "frequency_std": (
                    statistics.stdev(self.loop_frequencies)
                    if len(self.loop_frequencies) > 1
                    else 0
                ),
            }

        return summary

    def print_summary(self, top_n: int = 10) -> None:
        """Print formatted performance summary."""
        summary = self.get_summary()

        print("\n🔍 PERFORMANCE MONITORING SUMMARY")
        print("=" * 50)

        # Sort by total time
        operations = [
            (name, data) for name, data in summary.items() if name != "control_loop"
        ]
        operations.sort(key=lambda x: x[1]["total_s"], reverse=True)

        print(
            f"{'Operation':<20} {'Calls':<8} {'Avg (ms)':<10} {'Min (ms)':<10} {'Max (ms)':<10} {'Std (ms)':<10}"
        )
        print("-" * 80)

        for name, data in operations[:top_n]:
            print(
                f"{name:<20} {data['calls']:<8} {data['avg_ms']:<10.3f} "
                f"{data['min_ms']:<10.3f} {data['max_ms']:<10.3f} {data['std_ms']:<10.3f}"
            )

        # Control loop summary
        if "control_loop" in summary:
            loop_data = summary["control_loop"]
            print(f"\nControl Loop Performance:")
            print(f"  Average Frequency: {loop_data['avg_frequency_hz']:.1f} Hz")
            print(f"  Average Period:    {loop_data['avg_period_ms']:.3f} ms")
            print(f"  Min Period:        {loop_data['min_period_ms']:.3f} ms")
            print(f"  Max Period:        {loop_data['max_period_ms']:.3f} ms")
            print(f"  Frequency Std:     {loop_data['frequency_std']:.2f} Hz")

    def check_performance_issues(self) -> List[str]:
        """Check for potential performance issues."""
        issues = []
        summary = self.get_summary()

        # Check for slow operations (>5ms average)
        for name, data in summary.items():
            if name != "control_loop" and data["avg_ms"] > 5.0:
                issues.append(
                    f"Slow operation '{name}': {data['avg_ms']:.2f}ms average"
                )

        # Check for high variability (std > 50% of mean)
        for name, data in summary.items():
            if name != "control_loop" and data["std_ms"] > 0.5 * data["avg_ms"]:
                issues.append(
                    f"High variability in '{name}': std={data['std_ms']:.2f}ms"
                )

        # Check control loop frequency
        if "control_loop" in summary:
            loop_data = summary["control_loop"]
            target_freq = 250  # Expected attitude control frequency
            if loop_data["avg_frequency_hz"] < target_freq * 0.9:
                issues.append(
                    f"Control loop running slow: {loop_data['avg_frequency_hz']:.1f} Hz < {target_freq} Hz"
                )

        return issues


class TimingContext:
    """Context manager for timing operations."""

    def __init__(self, monitor: PerformanceMonitor, operation: str):
        self.monitor = monitor
        self.operation = operation

    def __enter__(self):
        self.monitor.start_timing(self.operation)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.monitor.end_timing(self.operation)


# Global performance monitor instance
_global_monitor: Optional[PerformanceMonitor] = None


def get_global_monitor() -> PerformanceMonitor:
    """Get or create global performance monitor."""
    global _global_monitor
    if _global_monitor is None:
        _global_monitor = PerformanceMonitor()
    return _global_monitor


def time_operation(operation: str):
    """Decorator for timing function calls."""

    def decorator(func):
        def wrapper(*args, **kwargs):
            monitor = get_global_monitor()
            with TimingContext(monitor, operation):
                return func(*args, **kwargs)

        return wrapper

    return decorator
