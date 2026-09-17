#include <Arduino.h>

#include "button.h"
#include "config.h"
#include "fsm.h"
#include "motor.h"
#include "speed.h"

void handleButtons();

// Button states
bool eStopActive = false;

// Speed reading
unsigned long lastSpeedReadTime = 0;
unsigned long currentTime;

// Motor Instance
Motor motorRight(R_START_STOP, R_RUN_BRAKE, R_DIR, R_INT_VR, R_PWM, R_SPEED_OUT);
Motor motorLeft(L_START_STOP, L_RUN_BRAKE, L_DIR, L_INT_VR, L_PWM, L_SPEED_OUT);

FSM fsm(motorRight, motorLeft);
SpeedMonitor speedMonitor(motorRight, motorLeft);

// Button
Button startStopBtn;
Button resetBtn;
Button eStopBtn;

void setup()
{
  Serial.begin(115200);
  delay(1000);

  startStopBtn.setup(START_STOP_BTN_PIN);   // activeLow = true (default)
  resetBtn.setup(RESET_BTN_PIN);
  eStopBtn.setup(E_STOP_BTN_PIN, INPUT_PULLUP, false); // E-Stop active HIGH

  motorRight.begin();
  motorLeft.begin();

  fsm.begin();

  Serial.println("AGV FSM Initialized. Ready.");
  Serial.println("Kirim via Serial Monitor: 'T'=Task, 'L'=Arrive Load, 'D'=Arrive Drop, 'O'=Obstacle, 'C'=Clear Obs");
}

void loop()
{
  handleButtons();

  fsm.handleSerial();
  fsm.update();

  if (fsm.isMoving()) {
    speedMonitor.update();
  }
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
      fsm.changeState(AgvState::MOVING_FORWARD);
    } else if (fsm.state() == AgvState::MOVING_FORWARD ||
              fsm.state() == AgvState::MOVING_ROTATE ||
              fsm.state() == AgvState::LOADING ||
              fsm.state() == AgvState::UNLOADING ||
              fsm.state() == AgvState::OBSTACLE) {

      fsm.changeState(AgvState::IDLE);
    }
  }
}
