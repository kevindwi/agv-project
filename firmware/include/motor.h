#pragma once
#include <Arduino.h>

class Motor {
public:
  Motor(uint8_t pinStartStop, uint8_t pinRunBrake, uint8_t pinDir,
        uint8_t pinIntVr, uint8_t pinPwm, uint8_t pinSpeed,
        bool forwardDirection = true);

  void begin();

  // pwm > 0: maju, pwm < 0: mundur, pwm == 0: berhenti.
  void set(int pwm);
  void stop();

  // Jumlah pulsa bertanda sejak pemanggilan terakhir.
  // Positif = maju, negatif = mundur.
  long getAndResetPulses();

  // Dipanggil dari ISR encoder setiap pulsa.
  void countEncoderPulse();

  static Motor *_instances[2];

private:
  uint8_t _pinStartStop;
  uint8_t _pinRunBrake;
  uint8_t _pinDir;
  uint8_t _pinIntVr;
  uint8_t _pinPwm;
  uint8_t _pinSpeed;
  bool _forwardDirection;
  volatile long _pulseCount;
};
