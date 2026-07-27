#include <Arduino.h>
#include "motor.h"
#include "config.h"

Motor motorRight;
Motor motorLeft;

void setup() {
  Serial.begin(115200);

  motorRight.begin(R_START_STOP, R_RUN_BRAKE, R_DIR, R_NT_VR, R_PWM, R_SPEED_OUT);
  motorLeft.begin(L_START_STOP, L_RUN_BRAKE, L_DIR, L_NT_VR, L_PWM, L_SPEED_OUT);

  delay(100);
}

void loop() {
  motorRight.setDirection(DIR_CW);
  motorRight.setSpeed(200);

  motorLeft.setDirection(DIR_CCW);
  motorLeft.setSpeed(200);

  delay(3000);

  motorRight.setDirection(DIR_CCW);
  motorRight.setSpeed(200);

  motorLeft.setDirection(DIR_CW);
  motorLeft.setSpeed(200);

  delay(3000);
}