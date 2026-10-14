#pragma once

#include "motor.h"
enum class AgvState { IDLE, MOVING, ERROR, ESTOP };

class FSM {
public:
  FSM(Motor &right, Motor &left);

  void begin();
  void update();
  void handleSerial();
  void handlePID();
  void changeState(AgvState newState);

  AgvState state() const;
  const char *stateName() const;
  bool isMoving() const;

private:
  void stopMotors();
  Motor &_right;
  Motor &_left;
  AgvState _state;
  unsigned long _stateStartTime;
};
