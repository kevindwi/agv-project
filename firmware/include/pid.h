#pragma once

class PIDController {
public:
  PIDController(float kp, float ki, float kd, float maxIntegral);
  void reset();
  float compute(float setpoint, float input, float dt);

private:
  float Kp;
  float Ki;
  float Kd;
  float prev_error;
  float integral;
  float max_integral;
};
