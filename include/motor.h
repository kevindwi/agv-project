#pragma once
#include <Arduino.h>

class Motor
{
private:
    uint8_t _pinStartStop;
    uint8_t _pinRunBrake;
    uint8_t _pinDir;
    uint8_t _pinNtVr;
    uint8_t _pinPwm;
    uint8_t _pinSpeed;
    volatile unsigned long _pulseCount;
public:
    Motor();

    void begin(uint8_t pinStartStop, uint8_t pinRunBrake, uint8_t pinDir, uint8_t pinNtVr, uint8_t pinPwm, uint8_t pinSpeed);

    void setSpeed(int pwm);
    
    void setDirection(bool dir);

    void stop();

    unsigned long getAndResetPulses();
};