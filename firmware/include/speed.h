#pragma once
#include "motor.h"

class SpeedMonitor {
public:
  SpeedMonitor(Motor &right, Motor &left);

  void update();

  // Buang pulsa yang menumpuk dan mulai jendela sampling baru.
  void reset();

private:
  Motor &_right;
  Motor &_left;
  unsigned long _lastUpdateTime;
};
