#pragma once

#include "motor.h"
enum class AgvState { IDLE, MOVING, ERROR, ESTOP };

class FSM {
public:
  FSM(Motor &right, Motor &left);

  void begin();
  void update();
  void handleSerial();
  void changeState(AgvState newState);

  AgvState state() const;
  const char *stateName() const;
  bool isMoving() const;

private:
  void stopMotors();
  bool commandTimedOut() const;
  Motor &_right;
  Motor &_left;
  AgvState _state;
  unsigned long _stateStartTime;
  unsigned long _lastCommandTime;
  int _rightSpeed;
  int _leftSpeed;
  bool _commandTimedOut;
};
