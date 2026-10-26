#include "fsm.h"
#include "Arduino.h"
#include "config.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

FSM::FSM(Motor &right, Motor &left)
    : _right(right), _left(left),
      _leftPid(WHEEL_PID_KP, WHEEL_PID_KI, WHEEL_PID_KD,
               WHEEL_PID_MAX_INTEGRAL),
      _rightPid(WHEEL_PID_KP, WHEEL_PID_KI, WHEEL_PID_KD,
                WHEEL_PID_MAX_INTEGRAL),
      _state(AgvState::IDLE), _stateStartTime(0), _lastCommandTime(0),
      _lastControlTime(0),
      _targetLeftTicks(0.0f), _targetRightTicks(0.0f), _commandTimedOut(false) {
}

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

  if (newState != AgvState::MOVING) {
    _targetLeftTicks = 0.0f;
    _targetRightTicks = 0.0f;
    _leftPid.reset();
    _rightPid.reset();
    stopMotors();
  } else {
    // Tunggu perintah baru setelah tombol START ditekan.
    _lastCommandTime = 0;
    _lastControlTime = millis();
  }

  SERIAL_DEBUG.print("STATE CHANGE --> ");
  SERIAL_DEBUG.println(stateName());

}

void FSM::handleSerial() {
  static char buffer[48];
  static byte index = 0;
  static bool discardLine = false;

  while (SERIAL_COMM.available() > 0) {
    char c = SERIAL_COMM.read();

    if (c == '\n') {
      if (!discardLine) {
        buffer[index] = '\0';
        parseCommand(buffer);
      }
      index = 0;
      discardLine = false;
    } else if (c != '\r') {
      if (index < sizeof(buffer) - 1) {
        buffer[index++] = c;
      } else {
        // Abaikan seluruh frame yang terlalu panjang sampai newline.
        discardLine = true;
      }
    }
  }
}

void FSM::parseCommand(char *line) {
  if (line[0] != 'C' || line[1] != ',') {
    return;
  }

  char *linearText = line + 2;
  char *comma = strchr(linearText, ',');
  if (comma == nullptr) {
    return;
  }

  *comma = '\0';
  char *endLinear;
  char *endAngular;

  // AVR libc menyediakan strtod; strtof tidak tersedia pada toolchain ini.
  float linear = (float)strtod(linearText, &endLinear);
  float angular = (float)strtod(comma + 1, &endAngular);

  if (endLinear == linearText || endAngular == comma + 1 ||
      *endLinear != '\0' || *endAngular != '\0' ||
      !isfinite(linear) || !isfinite(angular)) {
    return;
  }

  const float leftMps = linear - angular * WHEEL_SEPARATION_M / 2.0f;
  const float rightMps = linear + angular * WHEEL_SEPARATION_M / 2.0f;

  const float dt = CONTROL_PERIOD_MS / 1000.0f;
  const float metersPerTick =
      (PI * WHEEL_DIAMETER_M) / PULSES_PER_WHEEL_REV;

  _targetLeftTicks = leftMps * dt / metersPerTick;
  _targetRightTicks = rightMps * dt / metersPerTick;
  _lastCommandTime = millis();
}

void FSM::update() {
  if (_state != AgvState::MOVING) {
    stopMotors();

    // Buang pulsa saat diam agar tidak masuk ke siklus berikutnya.
    _left.getAndResetPulses();
    _right.getAndResetPulses();
    return;
  }

  if (commandTimedOut()) {
    if (!_commandTimedOut) {
      _commandTimedOut = true;
      SERIAL_DEBUG.println("Timeout perintah serial, motor berhenti");
    }

    stopMotors();
    _leftPid.reset();
    _rightPid.reset();
    _left.getAndResetPulses();
    _right.getAndResetPulses();
    _lastControlTime = millis();
    return;
  }

  const unsigned long now = millis();
  const unsigned long elapsedMs = now - _lastControlTime;

  if (elapsedMs < CONTROL_PERIOD_MS) {
    return;
  }

  _lastControlTime = now;
  _commandTimedOut = false;

  const float dt = elapsedMs / 1000.0f;

  // Ambil pulsa sekali. Nilai ini dipakai PID dan telemetri.
  const long leftTicks = _left.getAndResetPulses();
  const long rightTicks = _right.getAndResetPulses();

  const int leftPwm =
      (int)_leftPid.compute(_targetLeftTicks, (float)leftTicks, dt);
  const int rightPwm =
      (int)_rightPid.compute(_targetRightTicks, (float)rightTicks, dt);

  _left.set(leftPwm);
  _right.set(rightPwm);

  // Format balasan: E,<delta_kiri>,<delta_kanan>
  SERIAL_COMM.print("E,");
  SERIAL_COMM.print(leftTicks);
  SERIAL_COMM.print(",");
  SERIAL_COMM.println(rightTicks);
}
