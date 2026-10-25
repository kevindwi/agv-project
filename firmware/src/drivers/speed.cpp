#include "speed.h"
#include "config.h"

SpeedMonitor::SpeedMonitor(Motor &right, Motor &left)
    : _right(right), _left(left), _lastReadTime(0) {}

void SpeedMonitor::update() {
  unsigned long now = millis();
  if (now - _lastReadTime < SPEED_READ_INTERVAL_MS)
    return;
  _lastReadTime = now;

  unsigned long rPulses = _right.getAndResetPulses();
  unsigned long lPulses = _left.getAndResetPulses();

  float intervalSec = SPEED_READ_INTERVAL_MS / 1000.0f;
  float rRpm = (rPulses / intervalSec / PULSES_PER_REV) * 60.0f;
  float lRpm = (lPulses / intervalSec / PULSES_PER_REV) * 60.0f;

  Serial.print("R RPM: ");
  Serial.print(rRpm / 30.0f);
  Serial.print("  |  L RPM: ");
  Serial.println(lRpm / 30.0f);
}
