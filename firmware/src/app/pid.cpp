#include "pid.h"

PIDController::PIDController(float p, float i, float d, float max_i) {
  Kp = p;
  Ki = i;
  Kd = d;
  max_integral = max_i;
  reset();
}

void PIDController::reset() {
  prev_error = 0;
  integral = 0;
}

float PIDController::compute(float setpoint, float input, float dt) {
  if (dt <= 0.0f) {
    return 0.0f;
  }

  const float error = setpoint - input;
  integral += error * dt;

  if (integral > max_integral)
    integral = max_integral;
  if (integral < -max_integral)
    integral = -max_integral;

  const float derivative = (error - prev_error) / dt;
  prev_error = error;

  float output = Kp * error + Ki * integral + Kd * derivative;
  if (output > 255.0f)
    output = 255.0f;
  if (output < -255.0f)
    output = -255.0f;
  return output;
}
