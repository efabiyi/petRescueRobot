#include <Arduino.h>
#include "hallSensor.h"
#include "constants.h"
#include "logger.h"

HallSensor::HallSensor(Logger& logger) : logger(logger) {
  pinMode(HALL_PIN, INPUT);
  analogReadResolution(12); // Set ADC resolution to 12 bits
}

float HallSensor::readVoltage()
{
  int sensorValue = analogRead(HALL_PIN);
  float voltage = sensorValue * (HALL_VOLTAGE_REF / 4095.0); // Convert ADC value to voltage
  return voltage;
}

bool HallSensor::magnetDetected(float voltage)
{
  if (voltage < MAGNET_THRESHOLD_VOLTAGE) { 
    return true; 
  } 
  return false; 
}

void HallSensor::sense()
{
  float hallVoltage = readVoltage();
  bool magnet_detected = magnetDetected(hallVoltage);
  String hallData = "[Hall] Voltage: " + String(hallVoltage, 2) +
                      " V, Magnet: " + (magnet_detected ? "Yes" : "No");
  logger.log(hallData);
}