"""Kontroler PID untuk line tracking."""


class PIDController:
    def __init__(self, kp: float = 50.0, ki: float = 0.0, kd: float = 10.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.last_error = 0.0
        self.integral = 0.0

    def reset(self) -> None:
        self.last_error = 0.0
        self.integral = 0.0

    def compute(self, error: float, dt: float) -> float:
        if dt <= 0:
            return 0.0

        proportional_term = self.kp * error
        self.integral += error * dt
        integral_term = self.ki * self.integral
        derivative_term = self.kd * (error - self.last_error) / dt

        self.last_error = error
        return proportional_term + integral_term + derivative_term
