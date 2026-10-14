#include "button.h"

Button::Button()
    : _pin(0),
      _lastState(true),
      _isPressed(false)
{
}

void Button::setup(const uint8_t pin, const uint8_t mode, const bool activeLow) {
  _pin = pin;
  if (activeLow) {
    _buttonPressed = LOW;
  } else {
    _buttonPressed = HIGH;
  }

  pinMode(pin, mode);
  _lastState = digitalRead(pin);
}

bool Button::loop() {
  bool reading = digitalRead(_pin);

  if (reading != _lastState) {
    _lastDebounceTime = millis();
  }
  _lastState = reading;

  if ((millis() - _lastDebounceTime) > _debounce_ms) {
    if (reading == _buttonPressed) {
      if (!_isPressed) {
        _isPressed = true;
        return true;
      }
    } else {
      _isPressed = false;
    }
  }
  return false;
}
