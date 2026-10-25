#include "fsm.h"
#include "Arduino.h"
#include "config.h"

FSM::FSM(Motor &right, Motor &left)
    : _right(right), _left(left), _state(AgvState::IDLE), _stateStartTime(0),
      _lastCommandTime(0), _rightSpeed(0), _leftSpeed(0),
      _commandTimedOut(false) {}

void FSM::begin() { changeState(AgvState::IDLE); }

void FSM::stopMotors() {
  _right.stop();
  _left.stop();
}

bool FSM::commandTimedOut() const {
  // unsigned long: aman terhadap overflow millis()
  return (millis() - _lastCommandTime) >= SERIAL_COMMAND_TIMEOUT_MS;
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

  while (SERIAL_COMM.available() > 0) {
    char rc = SERIAL_COMM.read();

    if (rc == '\n') {
      buf[idx] = '\0';

      // Kontrak frame: "<left_pwm>,<right_pwm>\n" (lihat docs/serial_protocol.md)
      int leftSpeed = 0;
      int rightSpeed = 0;
      bool valid = false;

      char *token = strtok(buf, ",");
      if (token != NULL) {
        leftSpeed = atoi(token);
        token = strtok(NULL, ",");
        if (token != NULL) {
          rightSpeed = atoi(token);
          valid = true;
        }
      }

      if (valid) {
        _leftSpeed = leftSpeed;
        _rightSpeed = rightSpeed;
        _lastCommandTime = millis();

        SERIAL_DEBUG.print("R: ");
        SERIAL_DEBUG.print(_rightSpeed);
        SERIAL_DEBUG.print(" | L: ");
        SERIAL_DEBUG.println(_leftSpeed);
      } else {
        SERIAL_DEBUG.println("Frame serial tidak valid, diabaikan");
      }

      idx = 0;
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
    // Failsafe: bila tak ada frame valid selama timeout, hentikan motor.
    if (commandTimedOut()) {
      if (!_commandTimedOut) {
        _commandTimedOut = true;
        SERIAL_DEBUG.println("Timeout perintah serial, motor berhenti");
      }
      stopMotors();
      break;
    }

    _commandTimedOut = false;
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
