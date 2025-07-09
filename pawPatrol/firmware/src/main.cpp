#include <WiFi.h>
#include "hallsensor.h"
#include "secrets.h"
#include "logger.h"
#include "wifiManager.h"
#include "drive.h"
#include "detect.h"


HallSensor hallSensor(32); // GPIO pin for the hall sensor
WifiManager wifiMgr;

void setup() {
  Serial.begin(115200);
  wifiMgr.startWifi();
  initialize_drive();
}

void loop() {

  drive();

  float hallVoltage = hallSensor.readVoltage();
  bool magnetDetected = hallSensor.detectMagnet(hallVoltage);
  String hallData = "Hall Sensor Voltage: " + String(hallVoltage, 2) + " V, Magnet Detected: " + (magnetDetected ? "Yes" : "No");
  Logger::send(hallData);

  
  Serial.println(hallData);
  delay(100);
}
