#include <Arduino.h>

#include "button.h"
#include "config.h"
#include "fsm.h"
#include "motor.h"

// Globals
Motor motorRight(R_START_STOP, R_RUN_BRAKE, R_DIR, R_INT_VR, R_PWM, R_SPEED_OUT,
                 DIR_CW);
Motor motorLeft(L_START_STOP, L_RUN_BRAKE, L_DIR, L_INT_VR, L_PWM, L_SPEED_OUT,
                DIR_CCW);
FSM fsm(motorRight, motorLeft);

void handleButtons();

// Button states
bool eStopActive = false;

// State sebelumnya, untuk mendeteksi transisi ke MOVING.
AgvState previousState = AgvState::IDLE;

// Button
Button startStopBtn, resetBtn, eStopBtn;

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);

  startStopBtn.setup(START_STOP_BTN_PIN); // activeLow = true (default)
  resetBtn.setup(RESET_BTN_PIN);
  eStopBtn.setup(E_STOP_BTN_PIN, INPUT_PULLUP, false); // E-Stop active HIGH

  motorRight.begin();
  motorLeft.begin();

  fsm.begin();

  Serial.println("AGV FSM Initialized. Ready.");
}

void loop() {
  handleButtons();

  fsm.handleSerial();
  fsm.update();
}

void handleButtons() {
  // E-Stop
  if (eStopBtn.loop()) {
    eStopActive = true;
    fsm.changeState(AgvState::ESTOP);
    Serial.println("E-STOP ACTIVATED !!!");
  }

  // Reset
  if (resetBtn.loop()) {
    if (fsm.state() == AgvState::ESTOP || fsm.state() == AgvState::ERROR) {
      eStopActive = false;
      fsm.changeState(AgvState::IDLE);
    }
  }

  // Start/Stop
  if (startStopBtn.loop()) {
    if (fsm.state() == AgvState::IDLE) {
      fsm.changeState(AgvState::MOVING);
    } else if (fsm.state() == AgvState::MOVING) {

      fsm.changeState(AgvState::IDLE);
    }
  }
}
