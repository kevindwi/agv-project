#include "motor.h"
#include "config.h"

volatile unsigned long pulse_count = 0;

void pulse_ISR() { pulse_count++; }

Motor::Motor() {}

void Motor::begin(uint8_t pinStartStop, uint8_t pinRunBrake, uint8_t pinDir, uint8_t pinNtVr,
                  uint8_t pinPwm, uint8_t pinSpeed)
{
  _pinStartStop = pinStartStop;
  _pinRunBrake = pinRunBrake;
  _pinDir = pinDir;
  _pinNtVr = pinNtVr;
  _pinPwm = pinPwm;
  _pinSpeed = pinSpeed;
  _direction = DIR_CW;
  _pulseCount = 0;

  pinMode(_pinStartStop, OUTPUT);
  pinMode(_pinRunBrake, OUTPUT);
  pinMode(_pinDir, OUTPUT);
  pinMode(_pinNtVr, INPUT);
  pinMode(_pinPwm, OUTPUT);
  pinMode(_pinSpeed, INPUT_PULLUP);

  digitalWrite(_pinStartStop, LOW);
  digitalWrite(_pinRunBrake, HIGH);
  // digitalWrite(_pinDir, LOW);
  digitalWrite(_pinNtVr, LOW);

  attachInterrupt(digitalPinToInterrupt(_pinSpeed), pulse_ISR, FALLING);
}

void Motor::setSpeed(int pwm)
{
  digitalWrite(_pinStartStop, HIGH);
  digitalWrite(_pinDir, _direction);

  if (pwm <= 0)
  {
    stop();
    return;
  }
  else if (pwm > PWM_MAX)
  {
    pwm = PWM_MAX;
  }

  analogWrite(_pinPwm, pwm);
}

void Motor::setDirection(bool _dir) { _direction = _dir; }

void Motor::stop()
{
  digitalWrite(_pinStartStop, LOW);
  digitalWrite(_pinRunBrake, LOW);
  analogWrite(_pinPwm, 0);
}

unsigned long Motor::getAndResetPulses()
{
  unsigned long pulses = pulse_count;
  pulse_count = 0;
  return pulses;
}