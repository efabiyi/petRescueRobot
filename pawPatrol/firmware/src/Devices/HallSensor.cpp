#include <Arduino.h>
#include "hallsensor.h"

HallSensor::HallSensor(uint8_t hallPin) {
  pin = hallPin;
  pinMode(pin, INPUT);
  analogReadResolution(12);  // Set ADC resolution to 12 bits
}


float HallSensor::readVoltage() {
  int sensorValue = analogRead(pin);
  float voltage = sensorValue*( 2.5/ 4095.0); // Convert ADC value to voltage
  return voltage;
}

bool HallSensor::magnetDetected() {
  float voltage = readVoltage();
  if (voltage < 2.1) { // Adjust threshold as needed
    return true; // Magnet detected
  }
  return false; // No magnet detected
}