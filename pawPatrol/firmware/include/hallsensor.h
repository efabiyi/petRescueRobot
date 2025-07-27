#pragma once
#include <Arduino.h>
#include "logger.h"

class HallSensor
{
private:
  Logger &logger;

public:
  HallSensor(Logger &logger);
  float readVoltage();                // reads the voltage from the hall sensor
  bool magnetDetected(float voltage); // returns true if magnet is detected
  void sense();
};