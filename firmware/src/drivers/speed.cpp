#include "speed.h"
#include "config.h"

SpeedMonitor::SpeedMonitor(Motor &right, Motor &left)
    : _right(right), _left(left), _lastUpdateTime(0) {}

void SpeedMonitor::reset() {
  _lastUpdateTime = millis();
  _right.getAndResetPulses();
  _left.getAndResetPulses();
}

void SpeedMonitor::update() {
  unsigned long currentTime = millis();
  unsigned long elapsedMs = currentTime - _lastUpdateTime;
  if (elapsedMs < SPEED_READ_INTERVAL_MS)
    return;

  _lastUpdateTime = currentTime;

  long rPulses = _right.getAndResetPulses();
  long lPulses = _left.getAndResetPulses();

  // Pulsa per detik -> putaran roda per menit (900 pulsa/putaran roda).
  float intervalSec = elapsedMs / 1000.0f;
  float rWheelRpm = (rPulses / intervalSec / PULSES_PER_WHEEL_REV) * 60.0f;
  float lWheelRpm = (lPulses / intervalSec / PULSES_PER_WHEEL_REV) * 60.0f;

  // RPM roda -> kecepatan linier (m/s), positif maju, negatif mundur.
  float wheelCircumference = PI * WHEEL_DIAMETER_M;
  float rLinearSpeed = wheelCircumference * (rWheelRpm / 60.0f);
  float lLinearSpeed = wheelCircumference * (lWheelRpm / 60.0f);

  Serial.print("R: ");
  Serial.print(rWheelRpm, 2);
  Serial.print(" rpm, ");
  Serial.print(rLinearSpeed, 4);
  Serial.print(" m/s | L: ");
  Serial.print(lWheelRpm, 2);
  Serial.print(" rpm, ");
  Serial.print(lLinearSpeed, 4);
  Serial.println(" m/s");
}
