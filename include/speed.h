#pragma once
#include "motor.h"

class SpeedMonitor {
public:
  SpeedMonitor(Motor &right, Motor &left);

  void update();

private:
  Motor &_right;
  Motor &_left;
  unsigned long _lastReadTime;
};
