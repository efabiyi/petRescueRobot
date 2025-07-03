#pragma once
#include <Arduino.h>

class Potentiometer {
  private:
    uint8_t pin;
  public:
    Potentiometer(uint8_t analogPin);
    float readVoltage();  // returns voltage (0-3.3V)
};
