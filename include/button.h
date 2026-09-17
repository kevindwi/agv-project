#pragma once
#include <Arduino.h>

class Button {
  public:
    Button();
    void setup(const uint8_t pin, const uint8_t mode = INPUT_PULLUP, const bool activeLow = true);
    bool loop();
  private:
    uint8_t _pin;
    bool _lastState;
    bool _isPressed;
    int _buttonPressed; // level of the input pin when the button is pressed (LOW or HIGH)
    const uint32_t _debounce_ms = 50;
    unsigned long _lastDebounceTime = 0;
};
