#pragma once

#include "motor.h"
#include "pid.h"

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
  void parseCommand(char *line);

  Motor &_right;
  Motor &_left;
  PIDController _leftPid;
  PIDController _rightPid;

  AgvState _state;
  unsigned long _stateStartTime;
  unsigned long _lastCommandTime;
  unsigned long _lastControlTime;
  float _targetLeftTicks;
  float _targetRightTicks;
  bool _commandTimedOut;
};
