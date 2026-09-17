#include "fsm.h"
#include "Arduino.h"
#include "config.h"

FSM::FSM(Motor &right, Motor &left)
  : _right(right),
    _left(left),
    _state(AgvState::IDLE),
    _stateStartTime(0)
{
}

void FSM::begin() {
  changeState(AgvState::IDLE);
}

void FSM::stopMotors() {
  _right.stop();
  _left.stop();
}

AgvState FSM::state() const { return _state; }

const char *FSM::stateName() const {
  switch (_state) {
    case AgvState::IDLE:            return "IDLE";
    case AgvState::MOVING_FORWARD:  return "MOVING_FORWARD";
    case AgvState::MOVING_ROTATE:   return "MOVING_ROTATE";
    case AgvState::LOADING:         return "LOADING";
    case AgvState::UNLOADING:       return "UNLOADING";
    case AgvState::OBSTACLE:        return "OBSTACLE";
    case AgvState::ERROR:           return "ERROR";
    case AgvState::ESTOP:           return "ESTOP";
    default:                        return "UNKNOWN";
  }
}

bool FSM::isMoving() const {
  return _state == AgvState::MOVING_FORWARD || _state == AgvState::MOVING_ROTATE;
}

void FSM::changeState(AgvState newState) {
  if (_state == newState) return;

  _state = newState;
  _stateStartTime = millis();

  Serial.print("STATE CHANGE --> ");
  Serial.println(stateName());

  if (_state == AgvState::ESTOP || _state == AgvState::ERROR) {
    stopMotors();
  }
}

void FSM::handleSerial() {
  if (!Serial.available()) return;

  // -- Simulasi Input Sensor via Serial Monitor (Untuk Testing FSM) --
  char cmd = Serial.read();
  switch (cmd) {
    case 'T':
      if (_state == AgvState::IDLE) changeState(AgvState::MOVING_FORWARD);
      break;
    case 'L':
      if (_state == AgvState::MOVING_FORWARD) changeState(AgvState::LOADING);
      break;
    case 'R':
      if (_state == AgvState::MOVING_FORWARD) changeState(AgvState::MOVING_ROTATE);
      break;
    case 'F':
      if (_state == AgvState::MOVING_ROTATE) changeState(AgvState::MOVING_FORWARD);
      break;
    case 'D':
      if (_state == AgvState::MOVING_FORWARD || _state == AgvState::MOVING_ROTATE)
        changeState(AgvState::UNLOADING);
      break;
    case 'O':
      if (_state == AgvState::MOVING_FORWARD) changeState(AgvState::OBSTACLE);
      break;
    case 'C':
      if (_state == AgvState::OBSTACLE) changeState(AgvState::MOVING_FORWARD);
      break;
  }
}

void FSM::update() {
  unsigned long now = millis();

  switch (_state) {
    case AgvState::IDLE:
      stopMotors();
      break;

    case AgvState::MOVING_FORWARD:
      _right.set(DIR_CW, MOVING_PWM);
      _left.set(DIR_CCW, MOVING_PWM);
      break;

    case AgvState::MOVING_ROTATE:
      _right.set(DIR_CW, ROTATE_PWM);
      _left.set(DIR_CW, ROTATE_PWM);
      break;

    case AgvState::LOADING:
      stopMotors();
      if (now - _stateStartTime >= LOADING_DURATION_MS) {
        Serial.println("Goods loaded successfully.");
        changeState(AgvState::MOVING_FORWARD);
      }
      break;

    case AgvState::UNLOADING:
      stopMotors();
      if (now - _stateStartTime >= UNLOADING_DURATION_MS) {
        Serial.println("Goods unloaded. Task completed.");
        changeState(AgvState::IDLE);
      }
      break;

    case AgvState::OBSTACLE:
      stopMotors();
      if (now - _stateStartTime >= OBSTACLE_TIMEOUT_MS) {
        Serial.println("Obstacle timeout! Path blocked.");
        changeState(AgvState::ERROR);
      }
      break;

    case AgvState::ERROR:
    case AgvState::ESTOP:
      break;
  }
}
