#include <Arduino.h>
#include "hallsensor.h"


HallSensor::HallSensor(uint8_t analogPin) {
  pin = analogPin;
  analogReadResolution(12); // Set ADC resolution to 12 bits
}


float HallSensor::readVoltage() {
  int sensorValue = analogRead(pin);
  int fudgeFactor = 4095 - sensorValue; // Adjust for inverted reading
  float voltage = fudgeFactor * (3.3 / 4095.0); // Convert ADC value to voltage
  return voltage;
}

bool HallSensor::magnetDetected() {
  float voltage = readVoltage();
  if (voltage > 0.5) { // Adjust threshold as needed
    return true; // Pet detected, return voltage
  }
  return false; // No pet detected
}
