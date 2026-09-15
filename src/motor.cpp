#include "motor.h"
#include "config.h"

Motor *Motor::_instances[2] = {nullptr, nullptr};

#if defined(INT0)
static void isrInt0()
{
    if (Motor::_instances[0])
    {
        Motor::_instances[0]->_pulseCount++;
    }
}
#endif
#if defined(INT1)
static void isrInt1()
{
    if (Motor::_instances[1])
    {
        Motor::_instances[1]->_pulseCount++;
    }
}
#endif

Motor::Motor(uint8_t pinStartStop, uint8_t pinRunBrake, uint8_t pinDir, uint8_t pinIntVr,
             uint8_t pinPwm, uint8_t pinSpeed)
    : _pinStartStop(pinStartStop),
      _pinRunBrake(pinRunBrake),
      _pinDir(pinDir),
      _pinIntVr(pinIntVr),
      _pinPwm(pinPwm),
      _pinSpeed(pinSpeed),
      _pulseCount(0)
{
}

void Motor::begin()
{
    _pulseCount = 0;

    pinMode(_pinStartStop, OUTPUT);
    pinMode(_pinRunBrake, OUTPUT);
    pinMode(_pinDir, OUTPUT);
    pinMode(_pinIntVr, INPUT);
    pinMode(_pinPwm, OUTPUT);
    pinMode(_pinSpeed, INPUT_PULLUP);

    digitalWrite(_pinStartStop, LOW);
    digitalWrite(_pinRunBrake, HIGH);
    digitalWrite(_pinDir, DIR_CW);

    // LOW: external DC voltage is enabled.
    digitalWrite(_pinIntVr, LOW); // ERROR: Tidak berpengaruh (kabel I/O dijumper)

    int8_t intNum = digitalPinToInterrupt(_pinSpeed);
    if (intNum == NOT_AN_INTERRUPT)
        return;

    _instances[intNum] = this;

    switch (intNum)
    {
#if defined(INT0)
    case INT0:
        attachInterrupt(INT0, isrInt0, FALLING);
        break;
#endif
#if defined(INT1)
    case INT1:
        attachInterrupt(INT1, isrInt1, FALLING);
        break;
#endif
    default:
        break;
    }
}

void Motor::set(bool dir, int pwm)
{
    if (pwm <= 0)
    {
        stop();
        return;
    }
    else if (pwm > PWM_MAX)
    {
        pwm = PWM_MAX;
    }

    digitalWrite(_pinStartStop, HIGH);
    digitalWrite(_pinDir, dir); // DIRECTION
    analogWrite(_pinPwm, pwm);  // SPEED
}

void Motor::stop()
{
    digitalWrite(_pinStartStop, LOW);
    analogWrite(_pinPwm, PWM_MIN);
}

unsigned long Motor::getAndResetPulses()
{
    noInterrupts();
    unsigned long pulses = _pulseCount;
    _pulseCount = 0;
    interrupts();
    return pulses;
}
