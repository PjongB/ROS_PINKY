"""ROS-independent PID controller used by the docking node."""

from dataclasses import dataclass


def clamp(value: float, lower: float, upper: float) -> float:
    """Limit a value to an inclusive range."""
    return max(lower, min(value, upper))


@dataclass
class PIDController:
    """PID controller with output and integral limits."""

    kp: float
    ki: float
    kd: float
    output_min: float
    output_max: float
    integral_min: float = -1.0
    integral_max: float = 1.0

    def __post_init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        """Clear accumulated state."""
        self.integral = 0.0
        self.previous_error = None

    def update(self, error: float, dt: float) -> float:
        """Calculate a bounded PID output for one control interval."""
        if dt <= 0.0:
            raise ValueError('dt must be greater than zero')

        self.integral = clamp(
            self.integral + error * dt,
            self.integral_min,
            self.integral_max,
        )
        derivative = 0.0
        if self.previous_error is not None:
            derivative = (error - self.previous_error) / dt
        self.previous_error = error

        output = (
            self.kp * error
            + self.ki * self.integral
            + self.kd * derivative
        )
        return clamp(output, self.output_min, self.output_max)
