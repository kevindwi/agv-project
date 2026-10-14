#pragma once
#include <Arduino.h>

class Motor
{
public:
    uint8_t _pinStartStop;
    uint8_t _pinRunBrake;
    uint8_t _pinDir;
    uint8_t _pinIntVr;
    uint8_t _pinPwm;
    uint8_t _pinSpeed;
    volatile unsigned long _pulseCount;

    static Motor *_instances[2];

    Motor(uint8_t pinStartStop, uint8_t pinRunBrake, uint8_t pinDir, uint8_t pinIntVr,
          uint8_t pinPwm, uint8_t pinSpeed);

    void begin();
    void set(bool dir, int pwm);
    void stop();
    unsigned long getAndResetPulses();
};
