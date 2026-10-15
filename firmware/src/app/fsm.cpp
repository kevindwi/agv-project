#include "fsm.h"
#include "Arduino.h"
#include "config.h"

int rightSpeed = 0;
int leftSpeed = 0;

FSM::FSM(Motor &right, Motor &left)
    : _right(right), _left(left), _state(AgvState::IDLE), _stateStartTime(0) {}

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
      // Kontrak frame: "<left_pwm>,<right_pwm>\n" (lihat docs/serial_protocol.md)
      char *token = strtok(buf, ",");
      if (token != NULL)
        leftSpeed = atoi(token);
      token = strtok(NULL, ",");
      if (token != NULL)
        rightSpeed = atoi(token);

      idx = 0;

      Serial.print("R: ");
      Serial.print(rightSpeed);
      Serial.print(" | L: ");
      Serial.println(leftSpeed);
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
    _right.set(DIR_CW, rightSpeed);
    _left.set(DIR_CCW, leftSpeed);
    break;
  }

  case AgvState::ERROR:
  case AgvState::ESTOP:
    stopMotors();
    break;
  }
}
