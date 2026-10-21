#include "fsm.h"
#include "Arduino.h"
#include "config.h"

FSM::FSM(Motor &right, Motor &left)
    : _right(right), _left(left), _state(AgvState::IDLE), _stateStartTime(0),
      _rightSpeed(0), _leftSpeed(0) {}

void FSM::begin() { changeState(AgvState::IDLE); }

void FSM::stopMotors() {
  _right.stop();
  _left.stop();
}

AgvState FSM::state() const { return _state; }

const char *FSM::stateName() const {
  switch (_state) {
  case AgvState::IDLE:
    return "IDLE";
  case AgvState::MOVING:
    return "MOVING";
  case AgvState::ERROR:
    return "ERROR";
  case AgvState::ESTOP:
    return "ESTOP";
  default:
    return "UNKNOWN";
  }
}

bool FSM::isMoving() const { return _state == AgvState::MOVING; }

void FSM::changeState(AgvState newState) {
  if (_state == newState)
    return;

  _state = newState;
  _stateStartTime = millis();

  SERIAL_DEBUG.print("STATE CHANGE --> ");
  SERIAL_DEBUG.println(stateName());

  if (_state == AgvState::ESTOP || _state == AgvState::ERROR) {
    stopMotors();
  }
}

void FSM::handleSerial() {

  static char buf[16];
  static byte idx = 0;

  while (Serial1.available() > 0) {
    char rc = Serial1.read();

    if (rc == '\n') {
      buf[idx] = '\0';
      // Kontrak frame: "<left_pwm>,<right_pwm>\n"
      char *token = strtok(buf, ",");
      if (token != NULL)
        _leftSpeed = atoi(token);
      token = strtok(NULL, ",");
      if (token != NULL)
        _rightSpeed = atoi(token);

      idx = 0;

      Serial.print("R: ");
      Serial.print(_rightSpeed);
      Serial.print(" | L: ");
      Serial.println(_leftSpeed);
      // ------------------------------------------
    } else if (rc != '\r' && idx < sizeof(buf) - 1) {
      buf[idx++] = rc;
    }
  }
}

void FSM::update() {
  switch (_state) {
  case AgvState::IDLE:
    stopMotors();
    break;

  case AgvState::MOVING: {
    _right.set(DIR_CW, _rightSpeed);
    _left.set(DIR_CCW, _leftSpeed);
    break;
  }

  case AgvState::ERROR:
  case AgvState::ESTOP:
    stopMotors();
    break;
  }
}
