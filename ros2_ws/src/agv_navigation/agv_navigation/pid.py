"""PID Controller untuk navigasi pelacak garis."""

class PIDController:
    def __init__(self, kp: float, ki: float, kd: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.integral = 0.0
        self.previous_error = 0.0

    def compute(self, error: float, dt: float) -> float:
        if dt <= 0.0:
            return 0.0

        self.integral += error * dt
        # Anti-windup
        self.integral = max(-1.0, min(1.0, self.integral))

        derivative = (error - self.previous_error) / dt
        self.previous_error = error

        return (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)

    def reset(self):
        self.integral = 0.0
        self.previous_error = 0.0
