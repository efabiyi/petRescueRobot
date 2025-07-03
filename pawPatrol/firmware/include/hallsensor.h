#pragma once
#include <Arduino.h>

class HallSensor {
  private:
    uint8_t pin;
  public:
    HallSensor(uint8_t hallPin);
    float readVoltage();  // reads the voltage from the hall sensor
    bool magnetDetected();  // returns true if magnet is detected
};