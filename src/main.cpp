#include <Arduino.h>

#include "config.h"
#include "motor.h"

#define START_STOP_BTN_PIN 40
#define RESET_BTN_PIN 42
#define E_STOP_BTN_PIN 37

#define SPEED_READ_INTERVAL_MS 1000
#define PULSES_PER_REV 30

bool motorState = false;
bool eStopActive = false;

bool lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

bool lastResetReading = HIGH;
unsigned long lastResetDebounceTime = 0;

unsigned long lastSpeedReadTime = 0;

Motor motorRight(R_START_STOP, R_RUN_BRAKE, R_DIR, R_INT_VR, R_PWM, R_SPEED_OUT);
Motor motorLeft(L_START_STOP, L_RUN_BRAKE, L_DIR, L_INT_VR, L_PWM, L_SPEED_OUT);

void setup()
{
  Serial.begin(115200);

  pinMode(START_STOP_BTN_PIN, INPUT_PULLUP);
  pinMode(RESET_BTN_PIN, INPUT_PULLUP);
  pinMode(E_STOP_BTN_PIN, INPUT_PULLUP);

  motorRight.begin();
  motorLeft.begin();

  delay(100);
}

void loop()
{
  // --- 1. E-Stop Check (Prioritas, tanpa debounce) ---
  if (digitalRead(E_STOP_BTN_PIN) == LOW)
  {
    motorRight.stop();
    motorLeft.stop();
    motorState = false;
    eStopActive = true;
    Serial.println("E-STOP ACTIVATED");
    return;
  }

  // --- 2. Jika E-stop aktif, skip semua ---
  if (eStopActive)
    return;

  // --- 3. Reset Button (debounced) ---
  int resetReading = digitalRead(RESET_BTN_PIN);
  if (resetReading != lastResetReading)
  {
    lastResetDebounceTime = millis();
  }
  if ((millis() - lastResetDebounceTime) > debounceDelay)
  {
    static bool currentResetState = HIGH;
    if (resetReading != currentResetState)
    {
      currentResetState = resetReading;
      if (currentResetState == LOW)
      {
        eStopActive = false;
        Serial.println("E-STOP CLEARED");
      }
    }
  }
  lastResetReading = resetReading;

  // --- 4. Start/Stop Button (debounced) ---
  int reading = digitalRead(START_STOP_BTN_PIN);
  if (reading != lastButtonState)
  {
    lastDebounceTime = millis();
  }
  if ((millis() - lastDebounceTime) > debounceDelay)
  {
    static bool currentButtonState = HIGH;
    if (reading != currentButtonState)
    {
      currentButtonState = reading;
      if (currentButtonState == LOW)
      {
        motorState = !motorState;
        Serial.print("Motor State: ");
        Serial.println(motorState ? "ON" : "OFF");
      }
    }
  }
  lastButtonState = reading;

  // --- 5. Motor Control + Speed Reading ---
  if (motorState)
  {
    motorRight.set(DIR_CW, 20);
    motorLeft.set(DIR_CCW, 20);

    // Periodic speed reading
    if (millis() - lastSpeedReadTime >= SPEED_READ_INTERVAL_MS)
    {
      unsigned long r_pulses = motorRight.getAndResetPulses();
      unsigned long l_pulses = motorLeft.getAndResetPulses();

      // RPM = (frequency [Hz] / PULSES_PER_REV) * 60
      // frequency = pulses / (interval / 1000)
      float intervalSec = SPEED_READ_INTERVAL_MS / 1000.0;
      float r_rpm = (r_pulses / intervalSec / PULSES_PER_REV) * 60.0;
      float l_rpm = (l_pulses / intervalSec / PULSES_PER_REV) * 60.0;

      Serial.print("R RPM: ");
      Serial.print(r_rpm / 15);
      Serial.print("  L RPM: ");
      Serial.println(l_rpm / 15);

      lastSpeedReadTime = millis();
    }
  }
  else
  {
    motorRight.stop();
    motorLeft.stop();
  }
}
