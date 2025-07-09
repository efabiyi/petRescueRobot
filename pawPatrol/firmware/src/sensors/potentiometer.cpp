#include "potentiometer.h"

Potentiometer::Potentiometer(uint8_t analogPin) {
  pin = analogPin;
  analogReadResolution(12);  // 12-bit ADC resolution
}

float Potentiometer::readVoltage() {
  int raw = analogRead(pin);
  // ESP32 ADC range 0-4095, voltage 0-3.3V
  return (raw / 4095.0) * 5.0;
}
